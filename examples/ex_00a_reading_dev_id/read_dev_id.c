/*! ----------------------------------------------------------------------------
 *  @file    read_dev_id.c
 *  @brief   This example just read DW IC's device ID. It can be used to verify
 *           the SPI comms are working correctly.
 *
 * @attention
 *
 * Copyright 2018-2020 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2021 (c) Callender-Consulting, LLC  (port to Zephyr)
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <stdio.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>

/* zephyr includes */
#include <zephyr.h>
#include <sys/printk.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(read_devid);


/* Example application name and version to display. */
#define APP_NAME "READ DEV ID"

/**
 * Application entry point.
 */
int app_main(void)
{

    printk("Decawave good");
    int err;
    /* Display application name on LCD. */
    LOG_INF(APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    /* Target specific drive of RSTn line into DW IC low for a period. */
    reset_DWIC();

    /* Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, 
       or could wait for SPIRDY event) */
    Sleep(2);

    /* Read and validate device ID
     * returns DWT_ERROR if it does not match expected else DWT_SUCCESS 
     */
    err = dwt_check_dev_id();
    
    if (err == DWT_SUCCESS) {
        LOG_INF("DEV ID OK");
    }
    else {
        LOG_ERR("DEV ID FAILED");
    }

     while (!dwt_checkidlerc()) { 
/* spin */
 };

 if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("INIT FAILED");
        while (1) { /* spin */ };
    }
    //dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash
     * on DW3000 red eval-shield boards. */
printk("DWM3000 GOOD\n");

    return err;
}
