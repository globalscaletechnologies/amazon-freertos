#include <wmlist.h>
#include <wmstdio.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <workqueue.h>

typedef struct _wq_job_info {
    wq_job_t job;
    list_head_t node;
} wq_job_info_t;

typedef struct _workqueue {
    list_head_t list;
    os_mutex_t lock;
    os_semaphore_t event;
    os_thread_t thread;
    int exit;
} workqueue_t;

static void wq_thread(os_thread_arg_t data)
{
    workqueue_t *wq = (workqueue_t *) data;

    while (1) {
        os_semaphore_get(&wq->event, OS_WAIT_FOREVER);
        os_mutex_get(&wq->lock, OS_WAIT_FOREVER);

        while (!list_empty(&wq->list)) {
            wq_job_info_t *info = NULL;

            /* dequeue */
            info = list_entry(wq->list.next, wq_job_info_t, node);
            list_del(&info->node);

            os_mutex_put(&wq->lock);

            /* execute job function */
            if (info->job.job_func) {
                info->job.job_func(info->job.param);
            }
            os_mem_free(info);
            os_mutex_get(&wq->lock, OS_WAIT_FOREVER);
        }
        os_mutex_put(&wq->lock);
        if (wq->exit) {
            break;
        }
    }

    /* cleanup all jobs */
    os_mutex_get(&wq->lock, OS_WAIT_FOREVER);
    while (!list_empty(&wq->list)) {
        wq_job_info_t *info = NULL;

        info = list_entry(wq->list.next, wq_job_info_t, node);
        list_del(&info->node);
        os_mem_free(info);
    }
    os_mutex_put(&wq->lock);

    os_semaphore_delete(&wq->event);
    os_mutex_delete(&wq->lock);
    os_mem_free(wq);
    os_thread_delete(NULL);
}

wq_handle_t wq_create(const char *name, int stack_size, int prio)
{
    workqueue_t *wq = NULL;

    wq = os_mem_calloc(sizeof(workqueue_t));
    if (!wq) {
        return NULL;
    }

    wq->exit = 0;

    INIT_LIST_HEAD(&wq->list);

    if (os_semaphore_create(&wq->event, "wq_event")) {
        goto e_wq_event;
    }

    if (os_mutex_create(&wq->lock, "wq_lock", OS_MUTEX_INHERIT)) {
        goto e_wq_lock;
    }

    os_thread_stack_define(wq_stack, 1024);

    if (os_thread_create(&wq->thread,
                          "wq_thread",
                          wq_thread,
                          wq,
                          &wq_stack,
                          prio)) {
        goto e_wq_thread;
    }
    return wq;

e_wq_thread:
    os_mutex_delete(&wq->lock);
    wq->lock = NULL;
e_wq_lock:
    os_semaphore_delete(&wq->event);
    wq->event = NULL;
e_wq_event:
    os_mem_free(wq);
    return NULL;
}

int wq_enqueue(wq_handle_t handle, wq_job_t *job)
{
    workqueue_t *wq = (workqueue_t *) handle;
    wq_job_info_t *info = NULL;

    info = os_mem_calloc(sizeof(wq_job_info_t));
    if (!info) {
        return -WM_FAIL;
    }

    os_mutex_get(&wq->lock, OS_WAIT_FOREVER);

    info->job.job_func = job->job_func;
    info->job.param = job->param;

    list_add_tail(&info->node, &wq->list);

    os_semaphore_put(&wq->event);
    os_mutex_put(&wq->lock);

    return WM_SUCCESS;
}

int wq_destroy(wq_handle_t handle)
{
    workqueue_t *wq = (workqueue_t *) handle;

    if (!wq) {
        return -WM_FAIL;
    }
    os_mutex_get(&wq->lock, OS_WAIT_FOREVER);
    wq->exit = 1;
    os_semaphore_put(&wq->event);
    os_mutex_put(&wq->lock);

    return WM_SUCCESS;
}
