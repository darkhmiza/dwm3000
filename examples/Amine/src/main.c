
//zephyr includes
#include <zephyr/kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include<string.h>
#define LOG_LEVEL 3
#include <logging/log.h>
#include <zephyr/drivers/uart.h>
#include<time.h>
#include "ubxlib.h"
#include <zephyr/drivers/gpio.h>

#include <zephyr.h>
#include <device.h>
#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include<errno.h>









LOG_MODULE_REGISTER(simple_tx);

/* Example application name */
#define APP_NAME "SIMPLE TX v1.0"

/* Default communication configuration. We use default non-STS DW mode. */



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


// Return longitude/latitude value as string



//#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted

//char static prefix[2] = {0};
//    int32_t static whole[2] = {0};
//    int32_t static fraction[2] = {0};
static size_t gPositionCount = 0;
 static int sdfiles;

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
    static char prefix[2] = {0};
    static int32_t whole[2] = {0};
    static int32_t fraction[2] = {0};
    static int32_t time_nano = 5;
    static int32_t time_nano_old = 5;
    static int32_t rtk_mode32;
    static uint8_t rtk_mode8;
    static int32_t fix_mode;
    static int64_t time_utc;
    static int i_time = 0;
// Callback for position reception.
static void callback(uDeviceHandle_t gnssHandle,
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
    // We don't actually use the user context here, but
    // of course _you_ could
    
        
    if (errorCode == 0) {
        fix_mode=altitudeMillimetres;
        time_utc=timeUtc;
        time_nano=speedMillimetresPerSecond;
        i_time = 0;
         if(time_nano<10000000)
             time_nano=0;
         time_nano=time_nano*0.000001;
         if(time_nano%10==9){time_nano=time_nano+1;}
          
        rtk_mode32=radiusMillimetres;
        rtk_mode8= (uint8_t) rtk_mode32;
        rtk_mode8=rtk_mode8 & 0xC0;

        prefix[0] = latLongToBits(longitudeX1e7, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(latitudeX1e7, &(whole[1]), &(fraction[1]));
        /*$uPortLog("I am here: https://maps.google.com/?q=%c%d.%07d,%c%d.%07d\n",
                 prefix[1], whole[1], fraction[1], prefix[0], whole[0], fraction[0]);*/

        
        gPositionCount++;
    }
}
static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
    sdfiles=0;
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
                sdfiles=sdfiles+1;
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}


/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: THE EXAMPLE
 * -------------------------------------------------------------- */

// The entry point, main(): before this is called the system
// clocks must have been started and the RTOS must be running;
// we are in task space.
char* UnixToNormal(int32_t unixTimestamp) {
    char* date = (char*)malloc(80 * sizeof(char));  // Allocate memory for a char array of size 6
    if (date != NULL) {
           long long int secondsInDay = 24 * 60 * 60;
    long long int daysSinceEpoch = unixTimestamp / secondsInDay;
    long long int secondsOfDay = unixTimestamp % secondsInDay;
    
    // Unix epoch starts on January 1, 1970 (daysSinceEpoch=0)
    int year = 1970;
    int month, day;
    
    // Number of days in non-leap years
    int daysInYear = 365;
    
    while (daysSinceEpoch >= daysInYear) {
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            // Leap year
            if (daysSinceEpoch >= 366) {
                daysSinceEpoch -= 366;
                year++;
            } else {
                break;
            }
        } else {
            // Non-leap year
            daysSinceEpoch -= 365;
            year++;
        }
    }
    
    // At this point, year is the correct year
    // Now let's find the month and day
    int daysInMonth[] = {0, 31, (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0) ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (month = 1; month <= 12; month++) {
        if (daysSinceEpoch < daysInMonth[month]) {
            day = daysSinceEpoch + 1;
            break;
        }
        daysSinceEpoch -= daysInMonth[month];
    }

    int hours = secondsOfDay / 3600;
    int minutes = (secondsOfDay % 3600) / 60;
    int seconds = secondsOfDay % 60;
        //sprintf(date,"%d-%02d-%02d %02d:%02d:%02d\n", year, month, day, hours, minutes, seconds);
        sprintf(date,"%02d:%02d\n",minutes, seconds);
    }
    return date;
}


    static const char *disk_mount_pt = "/SD:";
   

void main()
{
    
   
k_msleep(10000);

int j=0;
    char dist_str[512];


    struct fs_file_t filp;
    do {
		static const char *disk_pdrv = "SD";
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);

     mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

    bool val;
    const struct device* dev= device_get_binding("GPIO_0");
        //const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
        gpio_pin_configure(dev, 6, GPIO_INPUT); 
        gpio_pin_configure(dev, 17, GPIO_OUTPUT); 

int sd_len;
int sd_len2;
int l;
char path[50];
int rate=100;
bool led_value=1;

if (res == FR_OK) {
   
		lsdir(disk_mount_pt);
		fs_file_t_init(&filp);
        l=sprintf(path,"/SD:/logs%d.txt",sdfiles);
		int resopen = fs_open(&filp,path,FS_O_RDWR | FS_O_CREATE);}



    char dist_str_sd[256];
    char dist_str_sd2[256];
    char* date;

    uDeviceHandle_t devHandle = NULL;
    int32_t returnCode;
    int32_t guardCount = 0;
    int count15=0;
    // Initialise the APIs we will need
   uPortInit();
    uPortI2cInit(); // You only need this if an I2C interface is used
    uDeviceInit();

 j=sprintf(dist_str,"    Befor\r\n");
    // j=sprintf(dist_str,"    %d:%d:%x:%d.%07d:%d:%d.%07d:\r\n",time_nano,fix_mode,rtk_mode8,whole[1], fraction[1],time_utc, whole[0], fraction[0]);


            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);
    // Open the device
    // returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
    do{
        returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
        k_msleep(100);}while (returnCode!=0);
    uPortLog("Opened device with return code %d.\n", returnCode);

    if (returnCode == 0) {


        // Since we are not using the common APIs we do not need
        // to call uNetworkInteraceUp()/uNetworkInteraceDown().

        // If you need to pass context data to
        // uGnssPosGetStreamedStart(), which does not include
        // a user parameter in its function signature, set
        // a user context for ourselves in the device, which
        // we can pick up in callback()

        // Start to get position
        uPortLog("Starting position stream.\n");
        returnCode = uGnssPosGetStreamedStart(devHandle,
                                              rate,
                                              callback);
        if (returnCode == 0) {


            uPortLog("Waiting up to 60 seconds for 5 position fixes.\n");
 
                
            while (1) {

                j=sprintf(dist_str,"    Begining of loop\r\n");
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);
                val=gpio_pin_get(dev,6);
                 if(val==0){
                        fs_close(&filp);
                        gpio_pin_set(dev,17,0);
                        //uGnssPwrOff(devHandle);
                        break;
                    } 
                uPortTaskBlock(rate- 10);
                count15=count15+1;
                // if (time_nano == time_nano_old)
                //     i_time++;
                // else{
                //     i_time = 0;
                // }

                if(i_time >= 3){
                    i_time = 3;
                    uGnssPosGetStreamedStop(devHandle);
                    uDeviceClose(devHandle, false);
                    uDeviceDeinit();
                    uPortI2cDeinit(); // You only need this if an I2C interface is used
                    uPortDeinit();

                    


                    uPortInit();
                    uPortI2cInit(); // You only need this if an I2C interface is used
                    uDeviceInit();
                    do{
                    returnCode = uDeviceOpen(&gDeviceCfg, &devHandle);
                    k_msleep(100);}while (returnCode!=0);
                    if(returnCode == 0){
                        returnCode = uGnssPosGetStreamedStart(devHandle,
                                                                100,
                                                                callback);
                            if(returnCode != 0){
                                continue;
                            }
                                
                    }
                    else{
                        continue;
                    }
                    k_msleep(2000);
                        
                }
                


    //j=sprintf(dist_str,"    I am here: https://maps.google.com/?q");
    // j=sprintf(dist_str,"    time_utc=%d:time_nano=%d:fix_mode=%d:rtk_mode=%x:%d.%07d:%d.%07d\r\n",time_utc,time_nano,fix_mode,rtk_mode8, whole[1], fraction[1], whole[0], fraction[0]);
                       // date=UnixToNormal(time_utc);

    j=sprintf(dist_str,"    %d\r\n",res);
    // j=sprintf(dist_str,"    %d:%d:%x:%d.%07d:%d:%d.%07d:\r\n",time_nano,fix_mode,rtk_mode8,whole[1], fraction[1],time_utc, whole[0], fraction[0]);


            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);

       sd_len=sprintf(dist_str_sd,"T14:%d:%x:%d.%07d:%d.%07d:%d.",fix_mode,rtk_mode8,whole[1], fraction[1], whole[0], fraction[0],time_utc);
           sd_len2=sprintf(dist_str_sd2,"%d\r\n",time_nano);
           
           time_nano_old = time_nano;
    
       if (res == FR_OK && i_time==0) {
		size_t data_size=strlen(dist_str_sd);
		int written_size=fs_write(&filp,dist_str_sd,sd_len);
        fs_write(&filp,dist_str_sd2,sd_len2);
        gpio_pin_set(dev,17,led_value);
        led_value=!led_value;
        }
        i_time += 1;

        if(count15==(900/(rate*0.001)) && res==FR_OK){

            fs_close(&filp);
        lsdir(disk_mount_pt);
		fs_file_t_init(&filp);
        l=sprintf(path,"/SD:/logs%d.txt",sdfiles);
        //l=sprintf(path,"/SD:/%s.txt",date);
		int resopen = fs_open(&filp,path,FS_O_RDWR | FS_O_CREATE);
        count15=0;
        }
             


            }
            // Stop getting position
            uGnssPosGetStreamedStop(devHandle);

        } else {
            uPortLog("Unable to start position stream!\n");
        }

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


}

