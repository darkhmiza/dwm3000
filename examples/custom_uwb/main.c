/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
#include <zephyr/kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include<string.h>
#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>


const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		
	};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_TX_DONE:
		// do something
		break;

	case UART_TX_ABORTED:
		// do something
		break;
		
	case UART_RX_RDY:
		// do something
		//for (int j=0;j<10;j++)
			//printk("%x",evt->data.rx.buf[evt->data.rx.offset+j]);
		//	printk("%x",rx_buf[j]);
		//printk("\n");

		break;
		

	case UART_RX_BUF_REQUEST:
		// do something
		break;

	case UART_RX_BUF_RELEASED:
		// do something
		break;
		
	case UART_RX_DISABLED:
		// do something
		break;

	case UART_RX_STOPPED:
		// do something
		break;
		
	default:
		break;
	}
}

uint8_t Get_CRC8(uint8_t * bufP, uint16_t len){
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < len; i ++)
	{
		crc ^= bufP[i ];
	}
	return crc;
}


void main(void)
{
	//LOG_ERR("This is a error message 1 !");
	char dist_str[30];
	int j=0;
	double distance = 2.55;
	 int err = uart_callback_set(uart, uart_cb, NULL);
		if (err) {
            printk("%d",err);
			return err;
		}

	while(1){
	//	LOG_ERR("This is a error message 2!");
	      j=sprintf(dist_str,"    :T2:%0.2lf\n\r",distance);
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);
						k_msleep(100);}
}

*/


/*
 * Copyright 2019-2023 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @brief This example demonstrates how to exchange message of your
 * choice with a GNSS device that is directly connected to this MCU;
 * this mechanism does not currently work if your GNSS device is
 * connected via an intermediate [cellular] module.
 *
 * The choice of module and the choice of platform on which this
 * code runs is made at build time, see the README.md for
 * instructions.
 */


/*
// Bring in all of the ubxlib public header files
#include <zephyr/kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include<string.h>
#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>
#include<time.h>
#include "ubxlib.h"





# define MY_MESSAGE_BUFFER_LENGTH  (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)



static const uDeviceCfg_t gDeviceCfg = {
    .deviceType = U_DEVICE_TYPE_GNSS,
    .deviceCfg = {
        .cfgGnss = {
            .moduleType = U_GNSS_MODULE_TYPE_M9,  // This is the module type
            .pinEnablePower = -1,
            .pinDataReady = -1
        },
    },
    .transportType = U_DEVICE_TRANSPORT_TYPE_I2C,
    .transportCfg = {
        .cfgI2c = {
            .i2c = 1, // This is the I2C HW block - ubxlib knows nothing about the device tree so you need to tell it
            .pinSda = -1,
            .pinScl = -1
        },
    },
};


// Count of messages received
static size_t gMessageCount = 0;



// Convert a lat/long into a whole number and a bit-after-the-decimal-point
// that can be printed by a version of printf() that does not support
// floating point operations, returning the prefix (either "+" or "-").
// The result should be printed with printf() format specifiers
// %c%d.%07d, e.g. something like:
//
// int32_t whole;
// int32_t fraction;
//
// printf("%c%d.%07d/%c%d.%07d", latLongToBits(latitudeX1e7, &whole, &fraction),
//                               whole, fraction,
//                               latLongToBits(longitudeX1e7, &whole, &fraction),
//                               whole, fraction);
static char latLongToBits(int32_t thingX1e7,
                          int32_t *pWhole,
                          int32_t *pFraction)
{
    char prefix = '+';

    // Deal with the sign
    if (thingX1e7 < 0) {
        thingX1e7 = -thingX1e7;
        prefix = '-';
    }
    *pWhole = thingX1e7 / 10000000;
    *pFraction = thingX1e7 % 10000000;

    return prefix;
}

// Print out the position contained in a UBX-NAV_PVT message
static void printPosition(const char *pBuffer, size_t length)
{
    char prefix[2] = {0};
    int32_t whole[2] = {0};
    int32_t fraction[2] = {0};
    int32_t longitudeX1e7;
    int32_t latitudeX1e7;

    if ((length >= 32) ) { //&& (*(pBuffer + 21) & 0x01)
        longitudeX1e7 =  uUbxProtocolUint32Decode(pBuffer + 30);
        latitudeX1e7 = uUbxProtocolUint32Decode(pBuffer + 34);
        prefix[0] = latLongToBits(longitudeX1e7, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(latitudeX1e7, &(whole[1]), &(fraction[1]));
        uPortLog("I am here: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
                 prefix[1], whole[1], fraction[1], prefix[0], whole[0], fraction[0]);
        
    }
}

// Callback for asynchronous message reception.
static void callback(uDeviceHandle_t devHandle, const uGnssMessageId_t *pMessageId,
                     int32_t errorCodeOrLength, void *pCallbackParam)
{
    char *pBuffer = (char *) pCallbackParam;
    int32_t length;

    (void) pMessageId;

    if (errorCodeOrLength >= 0) {
        // Read the message into our buffer and print it
        length = uGnssMsgReceiveCallbackRead(devHandle, pBuffer, errorCodeOrLength);
        if (length >= 0) {
            gMessageCount++;
#if !U_CFG_OS_CLIB_LEAKS && !defined(U_CFG_TEST_USING_NRF5SDK) // NRF52 goes a bit crazy if you print here
            uPortLog("%.*s", length, pBuffer);
        } else {
            uPortLog("Empty or bad message received.\n");
#endif
        }
    }
}
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		
	};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_TX_DONE:
		// do something
		break;

	case UART_TX_ABORTED:
		// do something
		break;
		
	case UART_RX_RDY:
		// do something
		//for (int j=0;j<10;j++)
			//printk("%x",evt->data.rx.buf[evt->data.rx.offset+j]);
		//	printk("%x",rx_buf[j]);
		//printk("\n");

		break;
		

	case UART_RX_BUF_REQUEST:
		// do something
		break;

	case UART_RX_BUF_RELEASED:
		// do something
		break;
		
	case UART_RX_DISABLED:
		// do something
		break;

	case UART_RX_STOPPED:
		// do something
		break;
		
	default:
		break;
	}
}

uint8_t Get_CRC8(uint8_t * bufP, uint16_t len){
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < len; i ++)
	{
		crc ^= bufP[i ];
	}
	return crc;
}




// The entry point, main(): before this is called the system
// clocks must have been started and the RTOS must be running;
// we are in task space.
void main()
{
	char prefix[2] = {0};
    int32_t whole[2] = {0};
    int32_t fraction[2] = {0};
    int32_t longitudeX1e7;
    int32_t latitudeX1e7;
		char dist_str[80];
	int j=0;
	double distance = 2.55;
	 int err = uart_callback_set(uart, uart_cb, NULL);
		if (err) {
            printk("%d",err);
			return err;
		}
    uDeviceHandle_t devHandle = NULL;
    uGnssMessageId_t messageId = {0};
    // Enough room for the UBX-NAV-PVT message, which has a body of length 92 bytes,
    // and any NMEA message (which have a maximum size of 82 bytes)
    char *pBuffer = (char *) pUPortMalloc(MY_MESSAGE_BUFFER_LENGTH);
    int32_t length = 0;
    int32_t returnCode;
    int32_t handle;

    // Initialise the APIs we will need
    uPortInit();
    uPortI2cInit(); // You only need this if an I2C interface is used
    uDeviceInit();

    // Open the device
    returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
    uPortLog("Opened device with return code %d.\n", returnCode);

    if ((returnCode == 0) && (pBuffer != NULL)) {
        // Since we are not using the common APIs we do not need
        // to call uNetworkInteraceUp()/uNetworkInteraceDown().

        // Just for when this test is running on the ubxlib test system
        // with other tests that may have switched NMEA messages off
        // (we need them a little lower down).
        uGnssCfgSetProtocolOut(devHandle, U_GNSS_PROTOCOL_NMEA, true);

        // Begin by sending a single UBX-format message to the GNSS
        // device and picking up the answer; the message does not have
        // to be a UBX-format message, it can be anything you think the
        // GNSS chip will understand (NMEA, SPARTN etc.), we are just
        // using a UBX-format message to demonstrate uUbxProtocolEncode().

        // First encode the message into pBuffer; we just send the message
        // class and ID of the UBX-NAV-PVT message (values read from the
        // GNSS interface manual - we will enumerate these at some point)
        // with an empty body: this "polls" the GNSS device for a
        // UBX-NAV-PVT message.
        length = uUbxProtocolEncode(0x01, 0x07, NULL, 0, pBuffer);
        if (uGnssMsgSend(devHandle, pBuffer, length) == length) {
            // Wait for the UBX-NAV-PVT response to come back
            messageId.type = U_GNSS_PROTOCOL_UBX;
            messageId.id.ubx = 0x0107; // This could be any UBX message ID/class
            length = uGnssMsgReceive(devHandle, &messageId, &pBuffer, MY_MESSAGE_BUFFER_LENGTH, 30000, NULL);
            if (length > 0) {
                for(int i=0;i<length;i++){
                    //printf("byte %d : %x\n",i,*(pBuffer+i));
		longitudeX1e7 =  uUbxProtocolUint32Decode(pBuffer + 30);
        latitudeX1e7 = uUbxProtocolUint32Decode(pBuffer + 34);
        prefix[0] = latLongToBits(longitudeX1e7, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(latitudeX1e7, &(whole[1]), &(fraction[1]));
					while(1){
					j=sprintf(dist_str,"    :T2:%d.%07d:%d.%07d:\r\n",whole[1], fraction[1],whole[0], fraction[0]);
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);
						uPortTaskBlock(100);}
                }

                printPosition(pBuffer, length);
            } else {
                uPortLog("Did not receive a response!\n");
            }
        } else {
            uPortLog("Unable to send message!\n");
        }

        // Alternatively, we can set up one or more message receive call-backs
        // We will set one up to capture all NMEA messages
        messageId.type = U_GNSS_PROTOCOL_NMEA;
        messageId.id.pNmea = "G"; // This means all, but could be "GPGSV", etc.
        // We give the message receiver pBuffer so that it can read messages into it
        handle = uGnssMsgReceiveStart(devHandle, &messageId, callback, pBuffer);
        if (handle >= 0) {
            // Wait a while for some messages to arrive
            uPortTaskBlock(5000);
            // Stop the message receiver(s) once more
            uGnssMsgReceiveStopAll(devHandle);
        } else {
            uPortLog("Unable to start message receiver!\n");
        }

        uPortLog("%d NMEA message(s) received.\n", gMessageCount);

        // Close the device
        // Note: we don't power the device down here in order
        // to speed up testing; you may prefer to power it off
        // by setting the second parameter to true.
        uDeviceClose(devHandle, false);

    } else {
        uPortLog("Unable to open GNSS!\n");
    }

    // Tidy up
    uDeviceDeinit();
    uPortI2cDeinit(); // You only need this if an I2C interface is used
    uPortDeinit();

    uPortLog("Done.\n");

    uPortFree(pBuffer);

}

*/

/*! ----------------------------------------------------------------------------
 *  @file    simple_tx.c
 *  @brief   Simple TX example code
 *
 * @attention
 *
 * Copyright 2015 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2021 (c) Callender-Consulting, LLC  (port to Zephyr)
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>

//zephyr includes
#include <zephyr/kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include<string.h>
#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>
#include<time.h>
#include <zephyr/drivers/gpio.h>
//ubxlib include
#include "ubxlib.h"
//Zephyr diskk includes
#include <zephyr.h>
#include <device.h>
#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include<errno.h>








LOG_MODULE_REGISTER(simple_tx);

/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"
# define MY_MESSAGE_BUFFER_LENGTH  (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)
//# define MY_MESSAGE_BUFFER_LENGTH  512




static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    .chan            = 5,               /* Channel number. */
    .txPreambLength  = DWT_PLEN_1024,    /* Preamble length. Used in TX only. */
    .rxPAC           = DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
    .txCode          = 9,               /* TX preamble code. Used in TX only. */
    .rxCode          = 9,               /* RX preamble code. Used in RX only. */
    .sfdType         = 1,    /* 0 to use standard 8 symbol SFD */
    .dataRate        = DWT_BR_850K,      /* Data rate. */
    .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
    .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
    .sfdTO           = (1025 + 8 - 32),   /* SFD timeout */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
};


static const uDeviceCfg_t gDeviceCfg = {
    .deviceType = U_DEVICE_TYPE_GNSS,
    .deviceCfg = {
        .cfgGnss = {
            .moduleType = U_GNSS_MODULE_TYPE_M9,  // This is the module type
            .pinEnablePower = -1,
            .pinDataReady = -1
        },
    },
    .transportType = U_DEVICE_TRANSPORT_TYPE_I2C,
    .transportCfg = {
        .cfgI2c = {
            .i2c = 1, // This is the I2C HW block - ubxlib knows nothing about the device tree so you need to tell it
            .pinSda = -1,
            .pinScl = -1
        },
    },
};

static char prefix[2] = {0};
static int32_t whole[2] = {0};
static int32_t fraction[2] = {0};
static int32_t latitude;
static int32_t longitude;
static int32_t svs;
static int32_t time_utc;
static int32_t time_nano;
static char rtk_mode;
static int32_t fix_mode;
static size_t gLocationCount = 0;
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));
const uGnssDecUnion_t pBody;
const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		
	};

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_TX_DONE:
		// do something
		break;

	case UART_TX_ABORTED:
		// do something
		break;
		
	case UART_RX_RDY:
		// do something
		//for (int j=0;j<10;j++)
			//printk("%x",evt->data.rx.buf[evt->data.rx.offset+j]);
		//	printk("%x",rx_buf[j]);
		//printk("\n");

		break;
		

	case UART_RX_BUF_REQUEST:
		// do something
		break;

	case UART_RX_BUF_RELEASED:
		// do something
		break;
		
	case UART_RX_DISABLED:
		// do something
		break;

	case UART_RX_STOPPED:
		// do something
		break;
		
	default:
		break;
	}
}

uint8_t Get_CRC8(uint8_t * bufP, uint16_t len){
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < len; i ++)
	{
		crc ^= bufP[i ];
	}
	return crc;
}


// Return longitude/latitude value as string
static char *locStr(int32_t loc)
{
    static char str[25];
    const char *sign = "";
    if (loc < 0) {
        loc = -loc;
        sign = "-";
    }
    snprintf(str, sizeof(str), "%s%d.%07d",
             sign, loc / 10000000, loc % 10000000);
    return str;
}


static char latLongToBits(int32_t thingX1e7,
                          int32_t *pWhole,
                          int32_t *pFraction)
{
    char prefix = '+';

    // Deal with the sign
    if (thingX1e7 < 0) {
        thingX1e7 = -thingX1e7;
        prefix = '-';
    }
    *pWhole = thingX1e7 / 10000000;
    *pFraction = thingX1e7 % 10000000;

    return prefix;
}

// Callback for position reception.
/*static void callback(uDeviceHandle_t gnssHandle,
                     int32_t errorCode,
                     int32_t latitudeX1e7,
                     int32_t longitudeX1e7,
                     int32_t altitudeMillimetres,
                     int32_t radiusMillimetres,
                     int32_t speedMillimetresPerSecond,
                     int32_t svs,
                     int64_t timeUtc)
{
  

    // Not using these, just keep the compiler happy
    (void) gnssHandle;
    (void) altitudeMillimetres;
    (void) radiusMillimetres;
    (void) speedMillimetresPerSecond;
    (void) svs;
    (void) timeUtc;
 

 

    if (errorCode == 0) {

        prefix[0] = latLongToBits(speedMillimetresPerSecond, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(radiusMillimetres, &(whole[1]), &(fraction[1]));
       // uPortLog("I am here: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
        //        prefix[1], whole[1], fraction[1], prefix[0], whole[0], fraction[0]);
        gLocationCount++;
    }
}*/
static size_t gMessageCount = 0;
char buffer[512];

void myCallback(uDeviceHandle_t gnssHandle,
                const uGnssMessageId_t *pMessageId,
                int32_t errorCodeOrLength,
                void *pCallbackParam)
{
    (void) pMessageId;
    int j=0;
    char dist_str[30];
    if (errorCodeOrLength > 0) {
        if (errorCodeOrLength > sizeof(buffer)) {
            errorCodeOrLength = sizeof(buffer);
        }
        errorCodeOrLength = uGnssMsgReceiveCallbackRead(gnssHandle,
                                                        buffer,
                                                        errorCodeOrLength);
        if (errorCodeOrLength > 0) {

            uGnssDec_t *pDec = pUGnssDecAlloc(buffer, errorCodeOrLength);
            if ((pDec != NULL) && (pDec->errorCode == 0)) {

                // Do stuff with pDec->pMsg, casting it to the correct message
                // type (see uGnssDecUnion_t in u_gnss_dec.h for the types)
                // by checking pDec->pId
                //time_utc=pDec->pBody->ubxNavPvt.month;
                    j=sprintf(dist_str,"    callback\r\n");
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);
            }
            // Must *always* free the memory that pUGnssDecAlloc() allocated
            uGnssDecFree(pDec);
        }
    }
}



/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */

static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint8_t rx_buffer2[FRAME_LEN_MAX];
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x33, 0, 0};
static uint8_t tx_msg_discovery[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x44, 0, 0};

static int num_A=3;
static double dists[3];
uint8_t A_add[] = {0x31,0x32,0x33};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
//static uint8_t A_add_test[] = {0x31,0x32}; // for testing transmitted message to multiple anchors

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 100

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 4 below. */
#define RX_RESP_TO_UUS 5000

#define DATA_FRAME_DEST_IDX 5
#define BLINK_FRAME_SRC_IDX 2

#define SW2_NODE	DT_ALIAS(sw2)

/* Get the device pointer. pin number, and pin's configuration flags through gpio_dt_spec */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW2_NODE, gpios);

//#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted
static void decode_frame(char *pMessage){

    int32_t months;
    int32_t year;
    int32_t y;
    int64_t t = -1;
     t = 0;
    // Year is 1999-2099, so need to adjust to get year since 1970
    year = ((int32_t) uUbxProtocolUint16Decode(pMessage + 10) - 1999) + 29;
    // Month (1 to 12), so take away 1 to make it zero-based
    months = *(pMessage + 12) - 1;
    months += year * 12;
    // Work out the number of seconds due to the year/month count
    t += uTimeMonthsToSecondsUtc(months);
    // Day (1 to 31)
    t += ((int32_t) * (pMessage + 13) - 1) * 3600 * 24;
    // Hour (0 to 23)
    t += ((int32_t) * (pMessage + 14)) * 3600;
    // Minute (0 to 59)
    t += ((int32_t) * (pMessage + 15)) * 60;
    // Second (0 to 60)
    t += *(pMessage + 16);
    time_utc=t;
    //time_nano=uPortGetTickTimeMs();
    svs = (int32_t) * (pMessage + 29);

    longitude =  uUbxProtocolUint32Decode(pMessage + 30);
    latitude = uUbxProtocolUint32Decode(pMessage + 34);
    rtk_mode=(uint8_t) *(pMessage+27);
    rtk_mode= rtk_mode & 0xC0;
    fix_mode=(int32_t) * (pMessage+26);
    time_nano=(int32_t) uUbxProtocolUint32Decode(pMessage + 22);
    if(time_nano<100000000)
        time_nano=0;
    time_nano=time_nano*0.000001;
}



static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}

//RTK decoding

static char RTK_decode(char rtk_mode)
    {

    // Shift the char to the left to keep only the two MSBs
    char msb = (rtk_mode << 6) & 0xC0;

    return msb;
}



/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature. These values can be
 * calibrated prior to taking reference measurements.
 * See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;
static const char *disk_mount_pt = "/SD:";
/**
 * Application entry point.
 */
void main()
{






    Sleep(20000);

    /* Display application name. */
    LOG_INF(APP_NAME);
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
     uint32_t status_reg = 0;
     /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
     uint16_t frame_len = 0;
    /*GPS handle*/
    uDeviceHandle_t devHandle = NULL;
    int32_t errorCode=0;
    // Initialise the APIs we will need
    
    peripherals_init();
    spi_peripheral_init();
    printk("TEST");
    char dist_str[256];
    char dist_str_sd[256];
    int nb_anchor_disc=0;
    int i = sprintf(dist_str, "DWM3Good000 \r\n");
    int j;
    int sd_len;

    int err = uart_callback_set(uart, uart_cb, NULL);
		if (err) {
            printk("%d",err);
			return err;
		}

        // configure button
        //int ret2 = gpio_pin_configure_dt(&button, GPIO_INPUT);
        const struct device* dev= device_get_binding("GPIO_0");
        //const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        gpio_pin_configure(dev, 6, GPIO_INPUT); 

     /* Configure SPI rate, DW3000 supports up to 38 MHz */
     port_set_dw_ic_spi_fastrate();

     /* Reset DW IC */
     reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

     Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
     while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
     {  
         j=sprintf(dist_str,"    error achieving IDLE_RC : %d\r\n",!dwt_checkidlerc());
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);
        reset_DWIC();
        Sleep(100);
        };


                        
     if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
     {
         while (1)
         { };
     }

     /* Optionally Configure GPIOs to show TX/RX activity. See NOTE 10 below. */
     //dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
     /* Optionally enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. See Note 10 below.*/
     //dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

     /* Configure DW IC. See NOTE 2 below. */
     if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
     {
         while (1)
         { };
     }

     /* Configure the TX spectrum parameters (power, PG delay and PG count) */
     dwt_configuretxrf(&txconfig_options);

     /* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
     dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

     /* Set response frame timeout. */
     dwt_setrxtimeout(RX_RESP_TO_UUS);

     dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

     /* Loop forever sending and receiving frames periodically. */
     int k=0;
     int Anch=1;
    int Tid=1;
    int Aid=0;
    char lat[30];
    char lon[30];
    char gps_char[30];
    uLocation_t location;
    uGnssMessageId_t messageId = {0};

    // Enough room for the UBX-NAV-PVT message, which has a body of length 92 bytes,
    // and any NMEA message (which have a maximum size of 82 bytes)

    int32_t length = 0;
    int32_t returnCode;
    int32_t handle;
    uPortInit();
    uPortI2cInit(); // You only need this if an I2C interface is used
    uDeviceInit();
    
    while(returnCode!=0){
    returnCode=uDeviceOpen(&gDeviceCfg, &devHandle);


    }

int l=0;
int32_t longitudeX1e7=0;
int32_t latitudeX1e7=0;    
//int32_t svs;
//int64_t timeUtc;
        
 /*uLocationGetContinuousStart(devHandle,
                                        100,
                                        U_LOCATION_TYPE_GNSS,
                                        NULL, NULL, callback);*/
//uPortFree(pBuffer);
        //char pBuffer[MY_MESSAGE_BUFFER_LENGTH];
        
       /* uGnssCfgSetRate(devHandle,100,-1,-1);
        messageId.type = U_GNSS_PROTOCOL_UBX;
        messageId.id.ubx = 0x0107;
        handle = uGnssMsgReceiveStart(devHandle, &messageId, myCallback,pBuffer);*/
                uGnssCfgSetRate(devHandle,200,-1,-1);
    bool val;
     while (1)
     {    
       // val=gpio_pin_get(dev,6);
       /* if (val==0){
          Sleep(5000);
        }*/
        dists[0]=0;
        dists[1]=0;
        dists[2]=0;
        for(k=0;k<3;k++){
            tx_msg[11]=A_add[k];
        


         /* Write frame data to DW3000 and prepare transmission. See NOTE 7 below. */
         dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
         dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

         /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
         dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

         /* We assume that the transmission is achieved normally, now poll for reception of a frame or error/timeout. See NOTE 8 below. */
         while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
         { };
         //HAL_UART_Transmit(&huart3,(uint8_t *)&anchor_ids, sizeof(anchor_ids),100);

         if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
         {
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

             int i;

             /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading
              * the RX buffer. */
             for (i = 0 ; i < FRAME_LEN_MAX; i++ )
             {
                 rx_buffer[i] = 0;
             }

             /* A frame has been received, copy it to our local buffer. */
             frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
             if (frame_len <= FRAME_LEN_MAX)
             {
                 dwt_readrxdata(rx_buffer, frame_len, 0);
             }
             /* TESTING BREAKPOINT LOCATION #1 */

             /* At this point, received frame can be examined in global "rx_buffer". An actual application would, for example, start by checking that
              * the format and/or data of the response are the expected ones. A developer might put a breakpoint here to examine this frame. */

             uint64_t init_tx_ts = get_tx_timestamp_u64();
             uint64_t init_rx_ts = get_rx_timestamp_u64();
             uint64_t time_u64 = (init_rx_ts-init_tx_ts);
             double time = (double) (time_u64 * DWT_TIME_UNITS / 2);
             double distance = time * SPEED_OF_LIGHT;
             //printk("distance : %0.2lf\n",distance);

             uint8_t test={0x45,0x32};
             //i = sprintf(dist_str, "T%0.2lf\r\n",range_num,distance);
             //HAL_UART_Transmit(&huart3,(uint8_t*)&dist_str,i,100);
             //HAL_UART_Transmit(&huart6,(uint8_t *)&rx_buffer, sizeof(rx_buffer),100);



            
             Sleep(10);
             dwt_rxenable(DWT_START_RX_IMMEDIATE);
             



           
	         while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	                  {
};
             if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                      {

                 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

             for (i = 0 ; i < FRAME_LEN_MAX; i++ )
                        {
                            rx_buffer2[i] = 0;
                        }

                        // A frame has been received, copy it to our local buffer.
                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
                        if (frame_len <= FRAME_LEN_MAX)
                        {
                            dwt_readrxdata(rx_buffer2, frame_len, 0);
                        }
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                        uint32_t resp_rx_ts, final_tx_ts;
						 uint32_t final_tx_ts_64, resp_rx_ts_64;
						 final_msg_get_ts(&rx_buffer2[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
						 final_msg_get_ts(&rx_buffer2[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
						 resp_rx_ts_64 = (uint32_t)resp_rx_ts;
						 final_tx_ts_64 = (uint32_t)final_tx_ts;
						 double time2_u64 =(double)(final_tx_ts_64-resp_rx_ts_64);
						 double time2 = (double) (time2_u64 * DWT_TIME_UNITS / 2);
						 double distance2 = time2 * SPEED_OF_LIGHT;
						 double distance3 = (distance-distance2);
						 dists[k]=distance3;



                      }
         }
         else
                       {
        	 //i = sprintf(dist_str, "TError\r\n");
             //printf("error\n");
        	 //HAL_UART_Transmit(&huart3,(uint8_t *)&dist_str, i,100);
             /* Clear RX error/timeout events in the DW3000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
         }
            
         

         /* Execute a delay between transmissions. */

               	//Sleep(TX_DELAY_MS);


         /* Increment the blink frame sequence number (modulo 256). */
         tx_msg[BLINK_FRAME_SN_IDX]++;
     }
     
        char *pBuffer = (char *) pUPortMalloc(MY_MESSAGE_BUFFER_LENGTH);
        length = uUbxProtocolEncode(0x01, 0x07, NULL, 0, pBuffer);
        if (uGnssMsgSend(devHandle, pBuffer, length) == length) {
            // Wait for the UBX-NAV-PVT response to come back
            messageId.type = U_GNSS_PROTOCOL_UBX;
            messageId.id.ubx = 0x0107; // This could be any UBX message ID/class
            length = uGnssMsgReceive(devHandle, &messageId, &pBuffer, MY_MESSAGE_BUFFER_LENGTH, 1000, NULL);
            if (length > 0) {
                //for(int i=0;i<length;i++){
                  //  printf("byte %d : %x\n",i,*(pBuffer+i));
                //}
        //longitudeX1e7 =  uUbxProtocolUint32Decode(pBuffer + 30);
        //latitudeX1e7 = uUbxProtocolUint32Decode(pBuffer + 34);
       decode_frame(pBuffer);
      //ubxNavPvtAlloc(pBuffer,MY_MESSAGE_BUFFER_LENGTH,pBody);
        prefix[0] = latLongToBits(latitude, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(longitude, &(whole[1]), &(fraction[1]));

        //printf("RTK Mode = %x\n",*(pBuffer+27));
            } else {
              //  uPortLog("Did not receive a response!\n");
                //uPortFree(pBuffer);
            }
        } else {
            //uPortLog("Unable to send message!\n");
        }
                //Sleep(50);

                uPortFree(pBuffer);
                        

       j=sprintf(dist_str,"    :T4:%0.2lf:%0.2lf:%0.2lf:%d:%x:%d.%07d:%d.%07d:%d:%d.%.03d:\r\n",dists[0],dists[1],dists[2],fix_mode,rtk_mode,whole[1], fraction[1],whole[0], fraction[0],svs,time_utc,time_nano);
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x04;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);



        // Write in sd card
        /*sd_len=sprintf(dist_str_sd,"T2:%0.2lf:%0.2lf:%0.2lf:%d:%x:%d.%07d:%d.%07d:%d:%d.%.03d:\r\n",dists[0],dists[1],dists[2],fix_mode,rtk_mode,whole[1], fraction[1],whole[0], fraction[0],svs,time_utc,time_nano);

        if (res == FR_OK) {
		lsdir(disk_mount_pt);
		fs_file_t_init(&filp);
		int resopen = fs_open(&filp,"/SD:/dist.txt",FS_O_RDWR | FS_O_CREATE);
		char test[]="1,2,3,4,t,2,5\n";
		size_t data_size=strlen(dist_str);
		int written_size=fs_write(&filp,dist_str_sd,sd_len);
        fs_close(&filp);
        break;
        }*/
        
       
     }

}

