#include <wmlist.h>
#include <wmstdio.h>
#include <wm_os.h>
#include <lowlevel_drivers.h>
#include <workqueue.h>
#include <sys_workqueue.h>

static wq_handle_t sys_workqueue_handle = NULL;
static int sys_workqueue_init_flag = 0;

int sys_workqueue_init()
{
    if (0 == sys_workqueue_init_flag) {
        sys_workqueue_handle = wq_create("sys_workqueue",
                                         SYS_WQ_STACKSIZE,
                                         SYS_WQ_PRIORITY);
        if (NULL == sys_workqueue_handle) {
            return -WM_FAIL;
        }
        sys_workqueue_init_flag = 1;
    }
    return WM_SUCCESS;
}

wq_handle_t sys_workqueue_get_handle()
{
    return sys_workqueue_handle;
}
