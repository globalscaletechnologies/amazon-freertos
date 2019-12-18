#ifndef _PCF8574_EXPANDER_WRAPPER_H_
#define _PCF8574_EXPANDER_WRAPPER_H_

#include <io_expander.h>

#define PCF8574_IDNUM   0x8574

int add_pcf8574_io_expander(gpio_expander_t **ioexp, int start);

#endif /* _PCF8574_EXPANDER_WRAPPER_H_ */
