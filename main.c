/**
 * Copyright (c) 2019 - Frederic Mes, RTLOC
 * 
 * This file is part of Zephyr-DWM1001.
 *
 *   Zephyr-DWM1001 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Zephyr-DWM1001 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Zephyr-DWM1001.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */
#include <zephyr.h>
#include <sys/printk.h>

#include "port.h"

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#define STACKSIZE 8192
#define PRIORITY -1
#define DELAY_TIME   K_MSEC(1000)

extern int app_main(void);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void main_thread(void * id, void * unused1, void * unused2)
{
    LOG_INF("%s", __func__);

    peripherals_init();
    spi_peripheral_init();

    k_sleep(K_MSEC(1000));

    app_main();

    while(1) { /* spin */}
}

K_THREAD_DEFINE(main_id, STACKSIZE, main_thread, 
                NULL, NULL, NULL, PRIORITY, 0, 0);



/*
CONFIG_MAIN_STACK_SIZE=2048
#CONFIG_SIZE_OPTIMIZATIONS=y
CONFIG_NO_OPTIMIZATIONS=y
#enable stack overflow test
CONFIG_STACK_SENTINEL=y
CONFIG_INIT_STACKS=y
CONFIG_STACK_USAGE=y
CONFIG_REBOOT=y

CONFIG_DEBUG_COREDUMP=y
CONFIG_DEBUG_COREDUMP_MEMORY_DUMP_MIN=y
CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING=y
CONFIG_DEBUG_INFO=y*/