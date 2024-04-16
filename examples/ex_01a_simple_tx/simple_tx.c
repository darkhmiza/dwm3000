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
#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(simple_tx);

/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"

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

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */

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





static uint8_t rx_buffer[FRAME_LEN_MAX];
static uint8_t rx_buffer2[FRAME_LEN_MAX];
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x31, 0, 0};
static uint8_t tx_msg_discovery[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x44, 0, 0};
static double dists[3];
static int num_A=3;
uint8_t A_add[] = {0x31,0x32,0x33};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
//static uint8_t A_add_test[] = {0x31,0x32}; // for testing transmitted message to multiple anchors

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 50

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 4 below. */
#define RX_RESP_TO_UUS 5000

#define DATA_FRAME_DEST_IDX 5
#define BLINK_FRAME_SRC_IDX 2

//#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted



/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature. These values can be
 * calibrated prior to taking reference measurements.
 * See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
int app_main(void)
{
    /* Display application name. */
    LOG_INF(APP_NAME);
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
     uint32_t status_reg = 0;
     /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
     uint16_t frame_len = 0;

     /* Display application name on LCD. */
      char dist_str[30];
        int nb_anchor_disc=0;
    int i = sprintf(dist_str, "DWM3Good000 \r\n");

        int err = uart_callback_set(uart, uart_cb, NULL);
		if (err) {
            printk("%d",err);
			return err;
		}



     /* Configure SPI rate, DW3000 supports up to 38 MHz */
     port_set_dw_ic_spi_fastrate();

     /* Reset DW IC */
     reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

     Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
     while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
     {      
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
    int j;
  
     while (1)
     {      
    
    dists[0]=0;
    dists[1]=0;
    dists[2]=0;
        for(k=0;k<3;k++){
            tx_msg[11]=A_add[k];
        
    	 
    		 //HAL_UART_Transmit(&huart3,(uint8_t*)&tx_msg,sizeof(tx_msg),100);


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
            printf("test status reg 2 before\n");
             if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
                      {
            printf("test status reg 2 after \n");
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
						 
                        //printk("distance : %0.2lf\n",distance3);
                        dists[k]=distance3;
                        //printf("Distance : %lf:%lf:%lf\n",dists[0],dists[1],dists[2]);
                        /*
                         j=sprintf(dist_str,"    D%d%0.2lf\r\n;",k+1,distance3);
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);*/

                        }
                      }
                       else{
        	 //i = sprintf(dist_str, "TError\r\n");
             //printf("error\n");
        	 //HAL_UART_Transmit(&huart3,(uint8_t *)&dist_str, i,100);
             /* Clear RX error/timeout events in the DW3000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
         }
            
         

         /* Execute a delay between transmissions. */



         /* Increment the blink frame sequence number (modulo 256). */
         tx_msg[BLINK_FRAME_SN_IDX]++;
     }
                    	 Sleep(TX_DELAY_MS);

     j=sprintf(dist_str,"    :T1:%0.2lf:%0.2lf:%0.2lf:\r\n",dists[0],dists[1],dists[2]);
                        dist_str[0]=0x02;
	                    dist_str[1]=0x04;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), SYS_FOREVER_MS);
     }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else's implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 3. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 4. We use polled mode of operation here to keep the example as simple as possible, but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW IC User Manual for more details on "interrupts".
 * 5. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/
