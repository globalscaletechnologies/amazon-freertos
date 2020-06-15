/*
 * button.c: button driver
 */
#include <wmlist.h>
#include <wmstdio.h>
#include <generic_io.h>
#include <mdev_pinmux.h>
#include <mdev_gpio.h>
#include <wm_os.h>
#include <sys_workqueue.h>
#include <lowlevel_drivers.h>
#include <button.h>

#define MAX_CFG_NUM                 4
#define DEF_DEBOUNCE_TIME           50

typedef struct _button_cfg {
    uint32_t time;
    button_cb callback;
    void *data;
} button_cfg_t;

typedef struct _button {
    input_gpio_cfg_t input;
    uint32_t pressed_time;
    uint32_t debounce_time;
    os_semaphore_t event;
    os_thread_t thread;
    int active;
    int exit;
    int cfg_index;
    int cfg_num;
    button_cfg_t configs[MAX_CFG_NUM];
    list_head_t node;
} button_t;

static bool btnInitDone = false;
static list_head_t lhButtons;

static bool button_pressed(input_gpio_cfg_t input)
{
    int val = 0;

    gpio_drv_read(NULL, input.gpio, &val);

    return (val == input.type)? true : false;
}

static button_t *get_button(input_gpio_cfg_t input)
{
    button_t *button;
    list_for_each_entry_type(button, button_t, &lhButtons,
                             node, list_head_t) {
        if (button->input.gpio == input.gpio) {
            return button;
        }
    }
    return NULL;
}

static void button_irq_handler(int pin, void *data)
{
    button_t *button = data;

    if (button) {
        os_semaphore_put(&button->event);
    }
    return;
}

static int button_perform_cb(void *data)
{
    button_t *button = data;
    int i = button->cfg_index;

    if (button->configs[i].callback) {
        button->configs[i].callback(button->input.gpio,
                                    button->configs[i].data);
    }
    return WM_SUCCESS;
}

static void _button_perform_cb(int index, void *data)
{
    button_t *button = data;
    wq_handle_t wq_handle;
    wq_job_t job = {
        .job_func = button_perform_cb,
        .param = data
    };

    button->cfg_index = index;
    wq_handle = sys_workqueue_get_handle();
    if (wq_handle) {
        wq_enqueue(wq_handle, &job);
    }
}

static void button_processing_thread(os_thread_arg_t data)
{
    button_t *button = (button_t *) data;
    uint32_t timeout = OS_WAIT_FOREVER;
    uint32_t time = 0;
    int debounce = 0;
    int ret = 0, i;

    while (1) {
        ret = os_semaphore_get(&button->event, timeout);
        if (button->exit) {
            break;
        }
        time = os_get_timestamp() / 1000;

        /* button down */
        if (ret == WM_SUCCESS && !button->pressed_time) {
            button->pressed_time = time;
            button->active = 1;
            gpio_drv_set_cb(NULL,
                            button->input.gpio,
                            GPIO_INT_DISABLE,
                            NULL,
                            NULL);
            timeout = os_msec_to_ticks(button->debounce_time);
            debounce = 1;
            continue;
        } else if (button->pressed_time && !debounce) {
            if (!button_pressed(button->input) || (ret == -WM_FAIL)) {
                /* button release */
                if (ret == WM_SUCCESS) {
                    for (i = 0; i < button->cfg_num; i++) {
                        if ((time - button->pressed_time) >
                            button->configs[i].time) {
                            _button_perform_cb(i, button);
                            break;
                        }
                    }
                } else {
                    _button_perform_cb(0, button);
                }
                button->pressed_time = 0;
                timeout = OS_WAIT_FOREVER;
                if (button->input.type == GPIO_ACTIVE_LOW) {
                    gpio_drv_set_cb(NULL,
                                    button->input.gpio,
                                    GPIO_INT_FALLING_EDGE,
                                    button,
                                    button_irq_handler);
                } else {
                    gpio_drv_set_cb(NULL,
                                    button->input.gpio,
                                    GPIO_INT_RISING_EDGE,
                                    button,
                                    button_irq_handler);
                }
                button->active = 0;
            }

        } else {
            if ((time - button->pressed_time) > button->debounce_time) {
                if (!button_pressed(button->input)) {
                    /* noise, reset debounce time */
                    button->pressed_time = time;
                } else {
                    /* setup button release interrupt */
                    if (button->input.type == GPIO_ACTIVE_LOW) {
                        gpio_drv_set_cb(NULL,
                                        button->input.gpio,
                                        GPIO_INT_RISING_EDGE,
                                        button,
                                        button_irq_handler);
                    } else {
                        gpio_drv_set_cb(NULL,
                                        button->input.gpio,
                                        GPIO_INT_FALLING_EDGE,
                                        button,
                                        button_irq_handler);
                    }
                    debounce = 0;
                    if (button->configs[0].time <
                        button->debounce_time) {
                        timeout = OS_WAIT_FOREVER;
                    } else {
                        timeout = os_msec_to_ticks(
                                  button->configs[0].time);
                    }
                }
            }
        }
    }

    os_semaphore_delete(&button->event);
    os_mem_free(button);
    os_thread_delete(NULL);
}

static int add_button_cfg(button_t *button,
                          uint32_t time,
                          button_cb callback,
                          void *data)
{
    int i, j;

    if (button->cfg_num >= MAX_CFG_NUM) {
        return -WM_FAIL;
    }

    for (i = 0; i < button->cfg_num; i++) {
        if (time > button->configs[i].time) {
            break;
        }
    }
    if (time > button->configs[i].time) {
        for (j = button->cfg_num; j > i; j--) {
            button->configs[j] = button->configs[j - 1];
        }
    }
    button->configs[i].time = time;
    button->configs[i].callback = callback;
    button->configs[i].data = data;
    button->cfg_num++;

    return WM_SUCCESS;
}

static int remove_button_cfg(button_t *button, uint32_t time)
{
    int i, j;

    if (!button) {
        return -WM_FAIL;
    }

    for (i = 0; i < button->cfg_num; i++) {
        if (time == button->configs[i].time) {

            for (j = i; j < button->cfg_num - 1; j++) {
                button->configs[j] = button->configs[j + 1];
            }
            button->cfg_num--;
            break;
        }
    }
    return WM_SUCCESS;
}

static int button_init()
{
    pinmux_drv_init();
    gpio_drv_init();

    if (!btnInitDone) {
        if (sys_workqueue_init() != WM_SUCCESS) {
            return -WM_FAIL;
        }
        INIT_LIST_HEAD(&lhButtons);
        btnInitDone = true;
    }
    return WM_SUCCESS;
}

int button_register_cb(input_gpio_cfg_t input,
                       uint32_t time,
                       button_cb callback,
                       void *data)
{
    mdev_t *pinmux_dev = NULL, *gpio_dev = NULL;
    button_t *button = NULL;
    int ret;

    if (button_init() != WM_SUCCESS) {
        return -WM_FAIL;
    }

    if (input.gpio < 0) {
        return -WM_FAIL;
    }

    button = get_button(input);

    if (button) {
        if (button->active) {
            return -WM_FAIL;
        }
        /* add button callback to config list */
        if (input.type != button->input.type) {
            return -WM_FAIL;
        }
        add_button_cfg(button, time, callback, data);
    } else {
        /* new button register */
        button = os_mem_calloc(sizeof(button_t));

        if (!button) {
            /* no memory */
            return -WM_FAIL;
        }
        button->input = input;
        button->pressed_time = 0;
        button->debounce_time = DEF_DEBOUNCE_TIME;
        button->cfg_index = 0;
        button->active = 0;
        button->exit = 0;
        add_button_cfg(button, time, callback, data);

        INIT_LIST_HEAD(&button->node);
        list_add_tail(&button->node, &lhButtons);

        if (os_semaphore_create(&button->event, "btn_event")) {
            remove_button_cfg(button, time);
            list_del(&button->node);
            os_mem_free(button);
            return -WM_FAIL;
        }
        os_semaphore_get(&button->event, OS_WAIT_FOREVER);

        os_thread_stack_define(btn_stack, 1024);
        ret = os_thread_create(&button->thread,
                               "btn_thread",
                               button_processing_thread,
                               button,
                               &btn_stack,
                               OS_PRIO_3);
        if (ret != WM_SUCCESS) {
            remove_button_cfg(button, time);
            list_del(&button->node);
            os_semaphore_delete(&button->event);
            os_mem_free(button);
            return -WM_FAIL;
        }

        /* init gpio pinmux and interrupt */
        pinmux_dev = pinmux_drv_open("MDEV_PINMUX");
        gpio_dev = gpio_drv_open("MDEV_GPIO");
        /* normal GPIO pin, gpio-expander is unnecessary to config */
        if (IS_GPIO_NO(input.gpio)) {
            pinmux_drv_setfunc(pinmux_dev, input.gpio,
                               pinmux_drv_get_gpio_func(input.gpio));
        }
        gpio_drv_setdir(gpio_dev, input.gpio, GPIO_INPUT);
        /* start interrupt */
        if (input.type == GPIO_ACTIVE_LOW) {
            gpio_drv_set_cb(gpio_dev,
                            input.gpio,
                            GPIO_INT_FALLING_EDGE,
                            button,
                            button_irq_handler);
        } else {
            gpio_drv_set_cb(gpio_dev,
                            input.gpio,
                            GPIO_INT_RISING_EDGE,
                            button,
                            button_irq_handler);
        }
        pinmux_drv_close(pinmux_dev);
        gpio_drv_close(gpio_dev);
    }
    return WM_SUCCESS;
}

int button_unregister_cb(input_gpio_cfg_t input, uint32_t time)
{
    button_t *button = get_button(input);

    if (!button || button->active) {
        return -WM_FAIL;
    }

    remove_button_cfg(button, time);

    if (!button->cfg_num) {
        gpio_drv_set_cb(NULL,
                        button->input.gpio,
                        GPIO_INT_DISABLE,
                        NULL,
                        NULL);
        button->exit = 1;
        os_semaphore_put(&button->event);

        list_del(&button->node);
    }
    return WM_SUCCESS;
}

int button_set_debounce_time(input_gpio_cfg_t input, uint32_t time)
{
    button_t *button = get_button(input);

    if (!button) {
        return -WM_FAIL;
    }
    button->debounce_time = time;
    return WM_SUCCESS;
}
