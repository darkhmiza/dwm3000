

/*
Author : HIMMICH Hamza
Last modified : 28/12/2023
Descrition : UGIMU-NAV 1 Positioning system that is able to read UWB and IMU and sends through thyone-I
             to a dedicated server
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
#include<errno.h>
// sensors includes
#include "sensors.h"











LOG_MODULE_REGISTER(simple_tx);

/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"

//# define MY_MESSAGE_BUFFER_LENGTH  512

static char prefix[2] = {0};
static int32_t whole[2] = {0};
static int32_t fraction[2] = {0};
static int32_t time_nano=0;
static uint8_t rtk_mode=0;
static int32_t fix_mode;
static int64_t time_utc;


const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart1));

const struct uart_config uart_cfg = {
		.baudrate = 115200,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		
	};

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


// Callback for position reception.


uint8_t Get_CRC8(uint8_t * bufP, uint16_t len){
	uint8_t crc = 0x00;
	for (uint16_t i = 0; i < len; i ++)
	{
		crc ^= bufP[i ];
	}
	return crc;
}










/**
 * Application entry point.
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


 extern dwt_txconfig_t txconfig_options;
dwt_txconfig_t txconfig_options3 =
{
    0x34,           /* PG delay. */
    0xfefefefe,      /* TX power. */
    0x0             /*PG count*/
};

void main()
{

    Sleep(20000);
    sensorsInit();


    /* Display application name. */
    LOG_INF(APP_NAME);
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
     uint32_t status_reg = 0;
     /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
     uint16_t frame_len = 0;
    int32_t errorCode=0;
    // Initialise the APIs we will need
  
    peripherals_init();
    spi_peripheral_init();
    printk("TEST");
    char dist_str1[256];
    char dist_str2[256];
    char dist_str[512];
    int j2=0;
    int j1=0;

    double dists[3]={0,0,0};
 

    int nb_anchor_disc=0;
    int i = sprintf(dist_str, "DWM3Good000 \r\n");
    int j;

    int err = uart_callback_set(uart, uart_cb, NULL);
		if (err) {
            printk("%d",err);
			return err;
		}

    /*Set thyone-I dest adress*/
    static uint8_t get_msg_dest []= {0x02,0x10,0x01,0x00,0x11,0x02};
static uint8_t get_msg_source []= {0x02,0x10,0x01,0x00,0x10,0x03};
static uint8_t set_msg_dest []= {0x02,0x11,0x05,0x00,0x11,0x01,0x00,0x00,0x00,0x06};
static uint8_t set_msg_source []= {0x02,0x11,0x05,0x00,0x10,0x03,0x00,0x00,0x00,0x05};
static uint8_t factory_reset_msg []= {0x02,0x1C,0x00,0x00,0x1E};
static uint8_t reset_msg []= {0x02,0x00,0x00,0x00,0x02};
static uint8_t rx_buf[10] = {0};
    int set_time=0;
    char th_resp[100];
    int t=0;
 /*    while(set_time<3){
      
    uart_tx(uart,set_msg_dest, 10, SYS_FOREVER_MS);
     
    uart_rx_enable(uart ,rx_buf,sizeof rx_buf,100);
    for(int y=0;y<11;y++){
            t=sprintf(th_resp,"    byte %d : %x\r\n",y-1,rx_buf[y-1]);
   
                        th_resp[0]=0x02;
	                    th_resp[1]=0x06;
	                    th_resp[2]=t-4;
	                    th_resp[3]=0x00;
                        
	                    th_resp[t]=Get_CRC8(th_resp,t);
                        uart_tx(uart,th_resp, sizeof(th_resp), 10);
    Sleep(100);  
    }
                        
    set_time=set_time+1;
     Sleep(100);
     uart_tx(uart,reset_msg, 10, SYS_FOREVER_MS);
    
    }*/

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
     dwt_configuretxrf(&txconfig_options3);

     /* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
     dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

     /* Set response frame timeout. */
     dwt_setrxtimeout(RX_RESP_TO_UUS);

     dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;





    
   



    int k = 0;
    int32_t length = 0;


    


    int l=0;





     while (1)
     {    
        
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
        k_msleep(50);


       j1=sprintf(dist_str1,"    :T20:%.02lf:%.02lf:%.02lf:%d:%x:%d.%07d:%d.%07d:%d.",dists[0],dists[1],dists[2],fix_mode,rtk_mode,whole[1], fraction[1],whole[0], fraction[0],time_utc);
       j2=sprintf(dist_str2,"%d%s\r\n",time_nano,pollAccelerometer());
        j=sprintf(dist_str,"%s%s",dist_str1,dist_str2);
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);




        
       
     }


}

