#ifndef _WORKQUEUE_H_
#define _WORKQUEUE_H_

typedef int (*job_func_t) (void *param);

typedef struct _wq_job {
    job_func_t job_func;
    void *param;
} wq_job_t;

typedef void * wq_handle_t;

wq_handle_t wq_create(const char *name, int stack_size, int prio);
int wq_enqueue(wq_handle_t handle, wq_job_t *job);
int wq_destroy(wq_handle_t handle);

#endif /* _WORKQUEUE_H_ */
