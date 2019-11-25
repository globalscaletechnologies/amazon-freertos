# Copyright (C) 2008-2018, Marvell International Ltd.
# All Rights Reserved.

libs-y += libbtdriver
libbtdriver-objs-y := bt.c

libbtdriver-objs-$(CONFIG_CPU_MW300) += \
			bt_uart_common.c
libbtdriver-objs-$(CONFIG_DA14580) += \
			aw-cu3xx/bt_uart.c \
			aw-cu3xx/cu3xx_ble_requestor.c \
			aw-cu3xx/cu3xx_ble_database.c \
			aw-cu3xx/cu3xx_host_apis.c \
			aw-cu3xx/cu3xx_uart_apis.c
libbtdriver-objs-$(CONFIG_DA14585) += \
			aw-cu3xx/bt_uart.c \
			aw-cu3xx/cu3xx_ble_requestor.c \
			aw-cu3xx/cu3xx_ble_database.c \
			aw-cu3xx/cu3xx_host_apis.c \
			aw-cu3xx/cu3xx_uart_apis.c
libbtdriver-cflags-y := -I$(d)
global-cflags-y += -I$(d)

libbtdriver-supported-toolchain-y := arm_gcc iar
