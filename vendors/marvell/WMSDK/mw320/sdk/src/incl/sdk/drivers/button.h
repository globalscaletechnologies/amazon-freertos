#ifndef _BUTTON_H_
#define _BUTTON_H_

typedef void (*button_cb) (int pin, void *param);

int button_register_cb(input_gpio_cfg_t input,
                       uint32_t time,
                       button_cb callback,
                       void *data);
int button_unregister_cb(input_gpio_cfg_t input, uint32_t time);
int button_set_debounce_time(input_gpio_cfg_t input, uint32_t time);
#endif /* _BUTTON_H_ */
