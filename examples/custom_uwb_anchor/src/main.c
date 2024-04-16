/*! ----------------------------------------------------------------------------
 *  @file    simple_rx.c
 *  @brief   Simple RX example code
 *
 * @attention
 *
 * Copyright 2015-2020 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2021 (c) Callender-Consulting, LLC  (port to Zephyr)
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <string.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>

//zephyr includes
#include <zephyr.h>
#include <sys/printk.h>

#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>
LOG_MODULE_REGISTER(simple_rx);

/* Example application name */
#define APP_NAME "SIMPLE RX v1.0"

    static dwt_config_t config = {
    .chan            = 5,               /* Channel number. */
    .txPreambLength  = DWT_PLEN_2048,    /* Preamble length. Used in TX only. */
    .rxPAC           = DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
    .txCode          = 9,               /* TX preamble code. Used in TX only. */
    .rxCode          = 9,               /* RX preamble code. Used in RX only. */
    .sfdType         = 1,    /* 0 to use standard 8 symbol SFD */
    .dataRate        = DWT_BR_850K,      /* Data rate. */
    .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
    .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
    .sfdTO           = (2049 + 8 - 32),   /* SFD timeout */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
};

/*UART configuration*/
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


/*Messages*/
static uint8_t tx_msg[] = {0x41, 0x8C, 0, 0x9A, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 'D', 'W', 0x10, 0x00, 0, 0, 0, 0};
static uint8_t tx_msg2[] = {0x41, 0x8C, 0, 0x9A, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 'D', 'W', 0x10, 0x00, 0, 0, 0, 0};
static uint8_t tx_resp_discovery[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x33, 0, 0};
static uint8_t debug_msg[] = {'D', 'E', 'C', 'A', 'D','B'};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Indexes to access to sequence number and destination address of the data frame in the tx_msg array. */
#define DATA_FRAME_SN_IDX 2
#define DATA_FRAME_DEST_IDX 5

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];
/* Index to access to source address of the blink frame in the rx_buffer array. */
#define BLINK_FRAME_SRC_IDX 2
/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];



extern dwt_txconfig_t txconfig_options;
dwt_txconfig_t txconfig_options3 =
{
    0x34,           /* PG delay. */
    0xfefefefe,      /* TX power. */
    0x0             /*PG count*/
};
/**
 * Application entry point.
 */
void main()
{
        /* Display application name on LCD. */
		Sleep(10000);
    LOG_INF(APP_NAME);
    /* Hold copy of status register state here for reference so that it can
     * be examined at a debug breakpoint. */
    uint32_t status_reg=0;
	peripherals_init();
    spi_peripheral_init();
    /* Hold copy of frame length of frame received (if good) so that it can
     * be examined at a debug breakpoint. */
    uint16_t frame_len=0;

        char dist_str[30];
        int j;
     	int i = sprintf(dist_str, "DWM3000 Good\r\n");


        port_set_dw_ic_spi_fastrate();

	     /* Reset DW IC */
	     reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

	     Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

	     while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
	     { };

	     if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
	     {
	         while (1)
	         { };
	     }

	     /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
	     dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

	     /* Configure DW IC. See NOTE 8 below. */
	     if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
	     {
	         while (1)
	         { };
	     }

	     /* Configure the TX spectrum parameters (power, PG delay and PG count) */
	     dwt_configuretxrf(&txconfig_options3);

	     /* Loop forever sending and receiving frames periodically. */
	     while (1)
	     {

	         /* Activate reception immediately. See NOTE 4 below. */
	         dwt_rxenable(DWT_START_RX_IMMEDIATE);

	         /* Poll until a frame is properly received or an error occurs. See NOTE 5 below.
	          * STATUS register is 5 bytes long but, as the events we are looking at are in the lower bytes of the register, we can use this simplest API
	          * function to access it. */
	         while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
	         { }
	         //HAL_UART_Transmit(&huart3,(uint8_t *)&debug_msg, sizeof(debug_msg),100);


	         if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
	         {
	             /* A frame has been received, read it into the local buffer. */
	             frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
	             if (frame_len <= FRAME_LEN_MAX)
	             {
	                 dwt_readrxdata(rx_buffer, frame_len, 0);
	             }

	             /* TESTING BREAKPOINT LOCATION #1 */

	             /* Clear good RX frame event in the DW IC status register. */
	             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

	             /* Validate the frame is the one expected as sent by "TX then wait for a response" example. */


	             if ((frame_len == 14) && (rx_buffer[0] == 0xC5) && (rx_buffer[10] == 0x43) && (rx_buffer[11] == 0x32))
	             {
	                 int i;

	                 /* Copy source address of blink in response destination address. */
	                 for (i = 0; i < 8; i++)
	                 {
	                     tx_msg[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
	                     tx_msg2[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
	                 }

	                 /* Write response frame data to DW IC and prepare transmission. See NOTE 6 below.*/
	                 dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
	                 dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

	                 /* Send the response. */
	                 dwt_starttx(DWT_START_TX_IMMEDIATE);

	                 /* Poll DW IC until TX frame sent event set. */
	                 while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
	                 {

	                 };

	                 /* Clear TX frame sent event. */
	                 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
	                 uint64_t resp_tx_ts = get_tx_timestamp_u64();
	                              uint64_t resp_rx_ts = get_rx_timestamp_u64();
	                              uint64_t time_u64 = resp_tx_ts-resp_rx_ts;




	                              //Send message to tag
                            	  uint8_t test[]={0x45,0x52};
                            	  Sleep(10);

                         	     //tx_msg2[j]=Get_CRC8(dist_str,j);
                	 	        //HAL_UART_Transmit(&huart3,(uint8_t *)&dist_str, sizeof(dist_str),100);
                	 	        final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                	 	        final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], resp_tx_ts);

                	 	       dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                	 	       dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */
                	 	       dwt_starttx(DWT_START_TX_IMMEDIATE);

	                              while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
	                             	                 {
	                            	 	         //HAL_UART_Transmit(&huart3,(uint8_t *)&test, sizeof(test),100);
	                            	 	         //HAL_Delay(100);

	                             	                 };

	         	                 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

	                 /* Increment the data frame sequence number (modulo 256). */
	                 tx_msg[DATA_FRAME_SN_IDX]++;
	             }
	             else {
	            	 i = sprintf(dist_str, "RError\r\n");
	            	 //HAL_UART_Transmit(&huart3,(uint8_t *)&dist_str, i,100);
	             }
	         }
	         else
	         {
	        	 i = sprintf(dist_str, "RError\r\n");
	        	 //HAL_UART_Transmit(&huart3,(uint8_t *)&dist_str, i,100);
	             /* Clear RX error events in the DW IC status register. */
	             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	         }
	     }

}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
