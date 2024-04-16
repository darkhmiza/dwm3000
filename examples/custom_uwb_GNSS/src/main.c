
/*! ----------------------------------------------------------------------------

 */



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
#include "sensors.h"








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
static int sdfiles;
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


static size_t gMessageCount = 0;
char buffer[512];


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

static const char *disk_mount_pt = "/SD:";
/**
 * Application entry point.
 */
void main()
{



    
    // sd card configuration
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







    /*GPS handle*/
    uDeviceHandle_t devHandle = NULL;
    int32_t errorCode=0;
    // Initialise the APIs we will need
    char dist_str[256];
    char dist_str_sd[256];
    char path[50];
    double dists[3]={0,0,0};
    int i=0;
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
        gpio_pin_configure(dev, 17, GPIO_OUTPUT); 


     /* Loop forever sending and receiving frames periodically. */



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
    sensorsInit();
    
    
    while(returnCode!=0){
    returnCode=uDeviceOpen(&gDeviceCfg, &devHandle);


    }

int l=0;
int32_t longitudeX1e7=0;
int32_t latitudeX1e7=0;    

                uGnssCfgSetRate(devHandle,100,-1,-1);
    bool val;
    if (res == FR_OK) {
		lsdir(disk_mount_pt);
		fs_file_t_init(&filp);
        l=sprintf(path,"/SD:/logs%d.txt",sdfiles);
		int resopen = fs_open(&filp,path,FS_O_RDWR | FS_O_CREATE);}

       j=sprintf(dist_str,"    Begining of loop\r\n");
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);
     while (1)
     {    
        val=gpio_pin_get(dev,6);
       /* if (val==0){
          Sleep(5000);
        }*/
      
     
        char *pBuffer = (char *) pUPortMalloc(MY_MESSAGE_BUFFER_LENGTH);
        length = uUbxProtocolEncode(0x01, 0x07, NULL, 0, pBuffer);
        if (uGnssMsgSend(devHandle, pBuffer, length) == length) {
            // Wait for the UBX-NAV-PVT response to come back
            messageId.type = U_GNSS_PROTOCOL_UBX;
            messageId.id.ubx = 0x0107; // This could be any UBX message ID/class
            length = uGnssMsgReceive(devHandle, &messageId, &pBuffer, MY_MESSAGE_BUFFER_LENGTH, 1000, NULL);
            if (length > 0) {

       decode_frame(pBuffer);
        prefix[0] = latLongToBits(latitude, &(whole[0]), &(fraction[0]));
        prefix[1] = latLongToBits(longitude, &(whole[1]), &(fraction[1]));
            } else {
              //  uPortLog("Did not receive a response!\n");
                //uPortFree(pBuffer);
            }
        } else {
            
        }
               

                uPortFree(pBuffer);
                        
    if(res !=FR_OK){
       j=sprintf(dist_str,"    :T2:%s%d:%x:%d.%07d:%d.%07d:%d:%d.%.03d:%d:\r\n",pollAccelerometer(),fix_mode,rtk_mode,whole[1], fraction[1],whole[0], fraction[0],svs,time_utc,time_nano,sdfiles);
            
                        dist_str[0]=0x02;
	                    dist_str[1]=0x06;
	                    dist_str[2]=j-4;
	                    dist_str[3]=0x00;
                        
	                    dist_str[j]=Get_CRC8(dist_str,j);
                        uart_tx(uart,dist_str, sizeof(dist_str), 10);}



        // Write in sd card
        sd_len=sprintf(dist_str_sd,"T2:%d.%.03d:%d:%x:%d.%07d:%d.%07d:%d:%s\r\n",time_utc,time_nano,fix_mode,rtk_mode,whole[1], fraction[1],whole[0], fraction[0],svs,pollAccelerometer());
        i=i+1;
       if (res == FR_OK) {
		size_t data_size=strlen(dist_str);
		int written_size=fs_write(&filp,dist_str_sd,sd_len);
        }
        if(val==0){
            fs_close(&filp);
            uGnssPwrOff(devHandle);
            break;
        }
        
       
     }

}

