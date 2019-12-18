#ifndef _PCA9539_EXPANDER_WRAPPER_H_
#define _PCA9539_EXPANDER_WRAPPER_H_

#include <io_expander.h>

#define PCA9539_IDNUM   0x9539

int add_pca9539_io_expander(gpio_expander_t **ioexp, int start);

#endif /* _PCA9539_EXPANDER_WRAPPER_H_ */
