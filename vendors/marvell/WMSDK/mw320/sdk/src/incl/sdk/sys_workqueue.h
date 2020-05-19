#ifndef _SYS_WORKQUEUE_H_
#define _SYS_WORKQUEUE_H_

#include <workqueue.h>

#define SYS_WQ_STACKSIZE               ( configMINIMAL_STACK_SIZE * 2 )
#define SYS_WQ_PRIORITY                ( tskIDLE_PRIORITY + 5 )

int sys_workqueue_init();
wq_handle_t sys_workqueue_get_handle();

#endif /* _SYS_WORKQUEUE_H_ */
