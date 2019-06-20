/* drivers/sharp/shgrip/shgrip_kerl.c
 *
 * Copyright (C) 2014 Sharp Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regulator/consumer.h>
#include <sharp/sh_boot_manager.h>
#ifndef SHGRIP_FACTORY_MODE_ENABLE
#include <sharp/shtps_dev.h>
#endif /* SHGRIP_FACTORY_MODE_ENABLE */

#include "shgrip_kerl.h"
#include "shgrip_fw.h"

#include <linux/pm_qos.h>
#include <sharp/sh_smem.h>
#include <sharp/shub_driver.h>

/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
/* --------------------------------------------------------- */
/* Debug Parameter                                           */
/* --------------------------------------------------------- */
#define SHGRIP_SPI_MULTI_TRANSFER
// #define SHGRIP_FW_DATA_NOT_WRITE

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
#define SHGRIP_NAME							"shgrip"
#define SHGRIP_DEVNAME						"shgrip_dev"

#define SHGRIP_CHANNEL_NUM					(4)

#define SHGRIP_CHG_LDR_VERIFY_COUNT			(1)
#define SHGRIP_CHG_LDR_MODE_RETRY_COUNT		(5)

#define SHGRIP_INIT_FWDL_RETRY_COUNT		(3)
#define SHGRIP_GSTCLR_RETRY_COUNT			(5)

#define SHGRIP_IRQ_DISABLED					(0)
#define SHGRIP_IRQ_ENABLED					(1)
#define SHGRIP_IRQ_FREE						(0)
#define SHGRIP_IRQ_REQUESTED				(1)

#define SHGRIP_WAKE_DISABLED				(0)
#define SHGRIP_WAKE_ENABLED					(1)

#define SHGRIP_HW_ES0						(0x00)
#define SHGRIP_HW_ES1						(0x01)
#define SHGRIP_HW_PP2						(0x05)
#define SHGRIP_HW_PP2_5						(0x06)
#define SHGRIP_HW_PMP						(0x07)

#define SHGRIP_CALB_THRESHOLD_FILE_PATH 	("/durable/grip/grip_threshold.dat")
#define SHGRIP_CALB_BUF_SIZE 				(16)
/* --------------------------------------------------------- */
/* System Parameter                                          */
/* --------------------------------------------------------- */
#define SPI_BUFFER							(259)
#define SPI_CLK_SPEED						(400000)
#define SPI_BIT_WORD						(8)

#define SHGRIP_PM_QOS_LATENCY_VALUE			(1)

#define SHGRIP_GPIO_VAL_LO					(0)
#define SHGRIP_GPIO_VAL_HI					(1)

#define SHGRIP_PMIC_POW_NAME				"pm8994_l29"

/* --------------------------------------------------------- */
/* Timer Parameter                                           */
/* --------------------------------------------------------- */
#define SHGRIP_STARTUP_WAIT					(125*1000)		
#define SHGRIP_RESET_START_WAIT_US			(3*1000)		

#define SHGRIP_PWR_OFF_WAIT_US				(10)			
#define SHGRIP_REGULATOR_OFF_WAIT_US		(1*1000)		

#define SHGRIP_CS_WAIT_US					(100)			
#define SHGRIP_COMREQ_WAIT_US				(200)			

#define SHGRIP_RESET_TIMEOUT_US				(300*1000)		
#define SHGRIP_POLL_RESET_WAIT_US			(5*1000)		

#define SHGRIP_READY_TIMEOUT_US				(100*1000)
#define SHGRIP_COMRDY_WAIT_MAX_US			(150*1000)		
#define SHGRIP_COMRDY_SEND_WAIT_MAX_US		(300*1000)		

#define SHGRIP_WAIT_BYTE_US					(280)			
#define SHGRIP_WAIT_CMD_US					(500)			
#define SHGRIP_WAIT_CMDEND_US				(100)			
#define SHGRIP_WAIT_CMD_ERR_END_US			(5*1000)		
#define SHGRIP_CTS_TIMEOUT_US				(20*1000)		
#define SHGRIP_SPI_SYNC_ERR_WAIT_US			(10*1000)		

#define SHGRIP_POLL_WAIT_US					(100)			
#define SHGRIP_POLL_WAIT_US_FOR_LOADER		(5)				
#define SHGRIP_ERACE_POLL_WAIT_US			(10*1000)		
#define SHGRIP_PGWRITE_POLL_WAIT_US			(1*1000)		

#define SHGRIP_CHKID_TIMEOUT_US 			(1*1000*1000)	
#define SHGRIP_CLRSTS_TIMEOUT_US			(10*1000)		
#define SHGRIP_ERASE_TIMEOUT_US				(10*1000*1000)	
#define SHGRIP_PGWRITE_TIMEOUT_US			(1*1000*1000)	
#define SHGRIP_PGREAD_TIMEOUT_US			(1*1000*1000)	
#define SHGRIP_CHGMODE_TIMEOUT_US			(10*1000)		

#define SHGRIP_COUNTVAL_READ_WAIT_US		(140*1000)
#define SHGRIP_COUNTVAL_READ_WAIT_US_PP1	(35*1000)

#define SHGRIP_PGWRITE_WAIT_LAST			(20*1000)
#define SHGRIP_IRQ_RELEASE_WAIT_MS			(500)
#define SHGRIP_CALIBRATION_WAIT				(100*1000)	
/* --------------------------------------------------------- */
/* Device Parameter                                          */
/* --------------------------------------------------------- */
#define BIT0								(0x01)
#define BIT1								(0x02)
#define BIT2								(0x04)
#define BIT3								(0x08)
#define BIT4								(0x10)
#define BIT5								(0x20)
#define BIT6								(0x40)
#define BIT7								(0x80)

#define SHGRIP_CMD_PVER						(0x01)
#define SHGRIP_CMD_PRXREGWR					(0x02)
#define SHGRIP_CMD_PRXREGRD					(0x03)
#define SHGRIP_CMD_DLTWR					(0x04)
#define SHGRIP_CMD_DLTRD					(0x05)
#define SHGRIP_CMD_GSTCLR					(0x06)
#define SHGRIP_CMD_TSON						(0x07)
#define SHGRIP_CMD_TSOFF					(0x08)
#define SHGRIP_CMD_ROMC						(0x09)
#define SHGRIP_CMD_LVER						(0x0A)
#define SHGRIP_CMD_RDST						(0x0B)
#define SHGRIP_CMD_CDLTWR					(0x0C)
#define SHGRIP_CMD_CDLTRD					(0x0D)
#define SHGRIP_CMD_FLTWR					(0x0E)
#define SHGRIP_CMD_FLTRD					(0x0F)
#define SHGRIP_CMD_CFLTWR					(0x10)
#define SHGRIP_CMD_CFLTRD					(0x11)
#define SHGRIP_CMD_CRDST					(0x12)
#define SHGRIP_CMD_PRXCNTRD					(0x13)
#define SHGRIP_CMD_BROMC					(0x14)
#define SHGRIP_CMD_ERDST					(0x15)
#define SHGRIP_CMD_EXTDEVON					(0x16)
#define SHGRIP_CMD_EXTDEVOFF				(0x17)
#define SHGRIP_CMD_EXTDEVFLTWR				(0x18)
#define SHGRIP_CMD_EXTDEVFLTRD				(0x19)

#define SHGRIP_CMD_ACK						(0x06)
#define SHGRIP_CMD_NACK						(0x15)
#define SHGRIP_CMD_DUMMY					(0xFF)

#define SHGRIP_LDR_CMD_PAGE_READ			(0xFF)
#define SHGRIP_LDR_CMD_PAGE_PROGRAM			(0x41)
#define SHGRIP_LDR_CMD_BLOCK_ERASE			(0x20)
#define SHGRIP_LDR_CMD_READ_STS_REG			(0x70)
#define SHGRIP_LDR_CMD_CLEAR_STS_REG		(0x50)
#define SHGRIP_LDR_CMD_ID_CHECK				(0xF5)
#define SHGRIP_LDR_CMD_VERSION_INFO			(0xFB)
#define SHGRIP_LDR_CMD_ROMC					(0x10)

#define SHGRIP_LDR_ERASE_DO					(0xD0)

#define SHGRIP_LDR_FW_ID1					(0x53)
#define SHGRIP_LDR_FW_ID2					(0x31)
#define SHGRIP_LDR_FW_ID3					(0x70)
#define SHGRIP_LDR_FW_ID4					(0x35)
#define SHGRIP_LDR_FW_ID5					(0x4F)
#define SHGRIP_LDR_FW_ID6					(0x30)
#define SHGRIP_LDR_FW_ID7					(0x67)
#define SHGRIP_LDR_FW_ID8					(0x37)
#define SHGRIP_LDR_FW_ID9					(0x49)
#define SHGRIP_LDR_FW_ID10					(0x3A)

#define SHGRIP_PRM_SENSOR_ALL_ON			(0x0F)
#define SHGRIP_PRM_SENSOR_ECOMODE			(0x3F)

#define SHGRIP_PARM_SENSOR_CH0				(0x00)
#define SHGRIP_PARM_SENSOR_CH1				(0x01)
#define SHGRIP_PARM_SENSOR_CH2				(0x02)
#define SHGRIP_PARM_SENSOR_CH3				(0x03)

#define SHGRIP_THR_VAL_LOW					(0x0000)

//#define SHGRIP_THRES_DEF_CH0_A				(5)
//#define SHGRIP_THRES_DEF_CH0_B				(0)
//#define SHGRIP_THRES_DEF_CH0_C				(25)
//#define SHGRIP_THRES_DEF_CH1_A				(5)
//#define SHGRIP_THRES_DEF_CH1_B				(0)
//#define SHGRIP_THRES_DEF_CH1_C				(25)
//#define SHGRIP_THRES_DEF_CH2_A				(125)
//#define SHGRIP_THRES_DEF_CH2_B				(120)
//#define SHGRIP_THRES_DEF_CH2_C				(25)
//#define SHGRIP_THRES_DEF_CH3_A				(25)
//#define SHGRIP_THRES_DEF_CH3_B				(20)
//#define SHGRIP_THRES_DEF_CH3_C				(25)

#define SHGRIP_CALB_CNTRD_MAX					(0x00FFFF)
#define SHGRIP_CALB_THRES_CH0					(0x000F)
#define SHGRIP_CALB_THRES_CH1					(0x000F)
#define SHGRIP_CALB_THRES_CH2					(0x000F)
#define SHGRIP_CALB_THRES_CH3					(0x000F)

/* --------------------------------------------------------- */
/* Firmware Parameter                                        */
/* --------------------------------------------------------- */
#define SHGRIP_FW_LOADER_OLDEST_VERSION		0x0204			
#define SHGRIP_FW_LOADER_LATEST_VERSION		SHGRIP_LVER	

#define SHGRIP_LDR_ADDR_BLK_BOOT4			(0x01000)
#define SHGRIP_LDR_ADDR_BLK_BOOT3			(0x01400)
#define SHGRIP_LDR_ADDR_BLK_BOOT2			(0x01800)
#define SHGRIP_LDR_ADDR_BLK_BOOT1			(0x01C00)

#define SHGRIP_LDR_ADDR_BLK_USER5			(0x02000)
#define SHGRIP_LDR_ADDR_BLK_USER4			(0x02400)
#define SHGRIP_LDR_ADDR_BLK_USER3			(0x02800)
#define SHGRIP_LDR_ADDR_BLK_USER2			(0x02C00)
#define SHGRIP_LDR_ADDR_BLK_USER1			(0x03000)

#define SHGRIP_LDR_ADDR_BLK_USER1_LAST		(0x03300)

#define SHGRIP_LDR_ADDR_BLK_BOOT_TERMINATE	(0x01FFF)
#define SHGRIP_LDR_ADDR_BLK_USER_TERMINATE	(0x033FF)

#define SHGRIP_LDR_BLK_ADDR_TBL_END			(0xFFFFF)

#define SHGRIP_LDR_SRD_SEQ					(0x80)
#define SHGRIP_LDR_SRD_ERZ					(0x20)
#define SHGRIP_LDR_SRD_PRG					(0x10)

#define SHGRIP_LDR_SRD_ID_CHK				(0x0C)

#define SHGRIP_LDR_PAGE_SIZE				(256)

#define SHGRIP_BOOT_CLUSTER_SIZE			(0x1000)
#define SHGRIP_FW_INDEX_BOOT_CLASTER1		(0x00000)
#define SHGRIP_FW_INDEX_USER_DATA			SHGRIP_BOOT_CLUSTER_SIZE

/* --------------------------------------------------------- */
/* Firmware Parameter                                        */
/* --------------------------------------------------------- */
#ifndef SHGRIP_FW_DATA_NOT_WRITE
static unsigned long shgrip_blk_addr_tbl_erase[] = 
{
	SHGRIP_LDR_ADDR_BLK_USER1,
	SHGRIP_LDR_ADDR_BLK_USER2,
	SHGRIP_LDR_ADDR_BLK_USER3,
	SHGRIP_LDR_ADDR_BLK_USER4,
	SHGRIP_LDR_ADDR_BLK_USER5,
	SHGRIP_LDR_ADDR_BLK_BOOT1,
	SHGRIP_LDR_ADDR_BLK_BOOT2,
	SHGRIP_LDR_ADDR_BLK_BOOT3,
	SHGRIP_LDR_ADDR_BLK_BOOT4,
	SHGRIP_LDR_BLK_ADDR_TBL_END
};
#endif /* SHGRIP_FW_DATA_NOT_WRITE */

static unsigned long shgrip_boot_blk_addr_tbl_write[] = 
{
	SHGRIP_LDR_ADDR_BLK_BOOT4,
	SHGRIP_LDR_BLK_ADDR_TBL_END
};
static unsigned long shgrip_user_blk_addr_tbl_write[] = 
{
	SHGRIP_LDR_ADDR_BLK_USER5,
	SHGRIP_LDR_BLK_ADDR_TBL_END
};

static unsigned long *shgrip_block_addr_table_write[] = 
{
	shgrip_boot_blk_addr_tbl_write,
	shgrip_user_blk_addr_tbl_write
};

static unsigned long shgrip_block_fw_index_start_addr[] = 
{
	SHGRIP_FW_INDEX_BOOT_CLASTER1,
	SHGRIP_FW_INDEX_USER_DATA
};

static unsigned long shgrip_block_tenminate[] = 
{
	SHGRIP_LDR_ADDR_BLK_BOOT_TERMINATE,
	SHGRIP_LDR_ADDR_BLK_USER_TERMINATE
};

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
struct shgrip_drv {
	spinlock_t					lock;
	bool						suspended;
	
	int							irq;
	int							irq_request;
	int							irq_enabled;
	int							irq_wake;
	
	struct input_dev			*input;
	
	struct workqueue_struct		*work_queue;
	struct work_struct			work_data;
	
#if defined (CONFIG_ANDROID_ENGINEERING)
	struct hrtimer				shgrip_threshold_timer;
	struct work_struct			th_work_data;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	enum shgrip_state			state;
	unsigned char				ch_state[SHGRIP_CHANNEL_NUM];
	
	struct shgrip_sens_setting_params bk_adj;
	struct shgrip_sensor_state	sensor_state;
	
	struct shgrip_fw_version	ver_info;
	
	const unsigned char			*fw_data;
	struct regulator 			*reg_p;
	
	unsigned short				hw_revision;
	
	struct delayed_work			release_work_data;
};

static dev_t 				shgrip_dev;
static dev_t				shgrip_major = 0;
static dev_t				shgrip_minor = 0;
static struct cdev 			shgrip_cdev;
static struct class* 		shgrip_class;
static struct device*		shgrip_device;

static struct shgrip_drv grip_ctrl;

static bool shgrip_dev_connect = false;
static bool shgrip_set_sensor_adjust_flg = false;
static bool shgrip_calibration_flg = false;
static bool shgrip_request_calibration_err_flg = false;

static bool shgrip_fwdl_disconnect = false;

static unsigned short shgrip_smem_lth[4];
static unsigned short shgrip_smem_hth[4];
static unsigned short shgrip_calb_lth[4];
static unsigned short shgrip_calb_hth[4];

static sharp_smem_common_type *sh_smem_common = NULL;
static unsigned long shgrip_tson_mode = SHGRIP_PRM_SENSOR_ECOMODE;
static unsigned char shgrip_ready_mode = 0;
static unsigned char shgrip_release_flg = 0;
static unsigned char shgrip_ldr_verify_flg = 0;

static struct mutex shgrip_io_lock;
static struct wake_lock shgrip_wake_lock;
static struct wake_lock shgrip_io_wake_lock;
static struct wake_lock shgrip_release_wake_lock;
static struct pm_qos_request shgrip_qos_cpu_dma_latency;

static int shgrip_gpio_int = -1;
static int shgrip_gpio_cs  = -1;
static int shgrip_gpio_pu  = -1;
static int shgrip_gpio_reset   = -1;
static int shgrip_gpio_com_req = -1;
static int shgrip_gpio_com_rdy = -1;

static int shgrip_sensor_ch = 0;

int shgrip_err_log  = 1;
int shgrip_warn_log = 0;
int shgrip_info_log = 0;
int shgrip_dbg_log  = 0;
int shgrip_data_log = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shgrip_err_log,  int, 0600);
module_param(shgrip_warn_log, int, 0600);
module_param(shgrip_info_log, int, 0600);
module_param(shgrip_dbg_log,  int, 0600);
module_param(shgrip_data_log, int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

static unsigned short lthr[4];
static unsigned short hthr[4];

#if defined (CONFIG_ANDROID_ENGINEERING)
static unsigned long shgrip_th_interval_ms = 0;
static unsigned long th_msec = 0;
static unsigned long th_msec_tmp = 0;
static unsigned char hrtimer_flg = 0;
#endif /* CONFIG_ANDROID_ENGINEERING */
static unsigned char thr_flg = 0;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_dev_spi_probe(struct spi_device *spi);
static int shgrip_dev_spi_remove(struct spi_device *spi);

static irqreturn_t shgrip_irq_func(int irq, void *dev);

static int shgrip_open(struct inode *inode, struct file *file);
static int shgrip_close(struct inode *inode, struct file *file);
static ssize_t shgrip_read(struct file* filp, char __user *buf, size_t count, loff_t* offset);
static ssize_t shgrip_write(struct file* filp, const char __user *buf, size_t count, loff_t* offset);
static long shgrip_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int shgrip_command_dummy(struct shgrip_drv *ctrl);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
/* --------------------------------------------------------- */
/* Serial Interface Parameter                                */
/* --------------------------------------------------------- */
static struct spi_device *spi_dev = NULL;

#ifdef CONFIG_OF
static const struct of_device_id shgrip_dev_dt_match[] = {
	{ .compatible = "sharp,shgrip_dev",},
	{}
};
#else
#define shgrip_dev_dt_match NULL;
#endif /* CONFIG_OF */

static int shgrip_suspend(struct spi_device *spi, pm_message_t mesg);
static int shgrip_resume(struct spi_device *spi);

static struct spi_driver shgrip_dev_spi_driver = {
	.driver = {
		.name = SHGRIP_DEVNAME,
		.owner = THIS_MODULE,
		.of_match_table = shgrip_dev_dt_match,
	},
	.probe = shgrip_dev_spi_probe,
	.remove = shgrip_dev_spi_remove,
	.suspend = shgrip_suspend,
	.resume = shgrip_resume,
};

/* --------------------------------------------------------- */
/* file_operations                                           */
/* --------------------------------------------------------- */
static struct file_operations shgrip_fops = {
	.owner          = THIS_MODULE,
	.open           = shgrip_open,
	.release        = shgrip_close,
	.read           = shgrip_read,
	.write          = shgrip_write,
	.compat_ioctl   = shgrip_ioctl,
	.unlocked_ioctl = shgrip_ioctl,
};

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHGRIP_ERR(fmt, args...) \
		if(shgrip_err_log == 1) { \
			printk("[SHGRIP_ERROR][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_WARN(fmt, args...) \
		if(shgrip_warn_log == 1) { \
			printk("[SHGRIP_WARN][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_INFO(fmt, args...) \
		if(shgrip_info_log == 1) { \
			printk("[SHGRIP_INFO][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_DBG(fmt, args...) \
		if(shgrip_dbg_log == 1) { \
			printk("[SHGRIP_DBG][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_DATA(fmt, args...) \
		if(shgrip_data_log == 1) { \
			printk("[SHGRIP_DATA][%s] " fmt, __func__, ## args); \
		}

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ========================================================================= */
/* SHGRIP ATTRIBUTE                                                          */
/* ========================================================================= */
#ifdef CONFIG_ANDROID_ENGINEERING
static ssize_t shgrip_dev_store(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	struct shgrip_drv *ctrl = &grip_ctrl;
	unsigned long sec,nsec;
	int ret;
	mutex_lock(&shgrip_io_lock);
	
	ret = kstrtoul(buf, 10, &shgrip_th_interval_ms);
	if (ret) {
		mutex_unlock(&shgrip_io_lock);
		return -EINVAL;
	}

	if (shgrip_th_interval_ms) {
		if (shgrip_th_interval_ms < 140) {
			SHGRIP_WARN("interval time is Err, Set default 140ms\n");
			shgrip_th_interval_ms = 140;
		}
		
		if(th_msec_tmp == 0){
			th_msec = shgrip_th_interval_ms;
			th_msec_tmp = shgrip_th_interval_ms;
		} else {
			th_msec_tmp = shgrip_th_interval_ms;
		}
		
		switch (ctrl->state) { 
		case STATE_SENSOR_ON:
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			if (hrtimer_flg == 0) {
				hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
				hrtimer_flg = 1;
			}
			break;
			
		case STATE_POWER_OFF:
		case STATE_SENSOR_OFF:
		case STATE_FW_DL:
		default:
			th_msec = th_msec_tmp;
			break;
		}
	} else {
		th_msec = shgrip_th_interval_ms;
		th_msec_tmp = shgrip_th_interval_ms;
		hrtimer_flg = 0;
		thr_flg = 0;
	}
	
	mutex_unlock(&shgrip_io_lock);
	return strlen(buf);
}

static DEVICE_ATTR( shgrip_dev, ( S_IWUSR | S_IWGRP ) , NULL, shgrip_dev_store );
#endif /* CONFIG_ANDROID_ENGINEERING */


/* ========================================================================= */

/* SHGRIP System Function                                                    */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_sys_delay_us                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_delay_us(unsigned long usec)
{
	struct timespec tu;
	
	if (usec >= 1000*1000) {
		tu.tv_sec  = usec / 1000000;
		tu.tv_nsec = (usec % 1000000) * 1000;
	}
	else {
		tu.tv_sec  = 0;
		tu.tv_nsec = usec * 1000;
	}
	
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_request_irq                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_sys_request_irq(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned long irq_flags;
	
	if (ctrl->irq_request == SHGRIP_IRQ_FREE) {
		irq_set_status_flags(gpio_to_irq(shgrip_gpio_int), IRQ_NOAUTOEN);
		ctrl->irq_enabled = SHGRIP_IRQ_DISABLED;
		
		irq_flags = IRQF_TRIGGER_LOW;
		ret = request_irq(gpio_to_irq(shgrip_gpio_int), shgrip_irq_func, 
									irq_flags, SHGRIP_NAME, ctrl);
		if (ret) {
			SHGRIP_ERR("request_irq is failed. ret:%d\n", ret);
		} else {
			ctrl->irq_request = SHGRIP_IRQ_REQUESTED;
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_free_irq                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_free_irq(struct shgrip_drv *ctrl)
{
	if (ctrl->irq_request == SHGRIP_IRQ_REQUESTED) {
		free_irq(gpio_to_irq(shgrip_gpio_int), ctrl);
		ctrl->irq_request = SHGRIP_IRQ_FREE;
	}
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq                                                     */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_enabled == SHGRIP_IRQ_DISABLED) {
		enable_irq(ctrl->irq);
		ctrl->irq_enabled = SHGRIP_IRQ_ENABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq                                                    */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_enabled == SHGRIP_IRQ_ENABLED) {
		disable_irq_nosync(ctrl->irq);
		ctrl->irq_enabled = SHGRIP_IRQ_DISABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq_wake                                                */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq_wake(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_wake == SHGRIP_WAKE_DISABLED) {
		enable_irq_wake(ctrl->irq);
		ctrl->irq_wake = SHGRIP_WAKE_ENABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq_wake                                               */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq_wake(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_wake == SHGRIP_WAKE_ENABLED) {
		disable_irq_wake(ctrl->irq);
		ctrl->irq_wake = SHGRIP_WAKE_DISABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_qos_start                                                          */
/* ------------------------------------------------------------------------- */
void shgrip_qos_start(void)
{
	pm_qos_update_request(&shgrip_qos_cpu_dma_latency, SHGRIP_PM_QOS_LATENCY_VALUE);
}

/* ------------------------------------------------------------------------- */
/* shgrip_qos_end                                                            */
/* ------------------------------------------------------------------------- */
void shgrip_qos_end(void)
{
	pm_qos_update_request(&shgrip_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);
}

/* ------------------------------------------------------------------------- */
/* shgrip_suspend                                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = (struct shgrip_drv *)spi_get_drvdata(spi);
	
	ctrl->suspended = true;
	if (ctrl->state == STATE_SENSOR_ON) {
		shgrip_sys_disable_irq(ctrl);
		shgrip_sys_enable_irq_wake(ctrl);
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_resume                                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_resume(struct spi_device *spi)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = (struct shgrip_drv *)spi_get_drvdata(spi);
	
	ctrl->suspended = false;
	if (ctrl->state == STATE_SENSOR_ON) {
		shgrip_sys_enable_irq(ctrl);
		shgrip_sys_disable_irq_wake(ctrl);
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ========================================================================= */
/* SHGRIP Serial Interface Function                                          */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* bitflip8                                                                  */
/* ------------------------------------------------------------------------- */
#define BIT_NUM (8)
static unsigned char bitflip8(unsigned char base)
{
#if 0
	unsigned char temp;
	unsigned char val;
	int i;
	
	temp = 0xFF & base;
	val = 0;
	
	for (i = 0; i < BIT_NUM; i++) {
		val |= ((temp >> i) & 0x01) << (BIT_NUM - 1 - i);
	}
	
	return val;
#else
	return base;
#endif
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_read_block                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_read_block(struct shgrip_drv *ctrl,
										unsigned char *rbuf, int rlen)
{
#ifdef SHGRIP_SPI_MULTI_TRANSFER
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int i;
	int ret = 0;
	unsigned char readdata[SPI_BUFFER];
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}

	if ((rlen > 0) && (rlen <= SPI_BUFFER)) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = rlen;
		x->speed_hz         = SPI_CLK_SPEED;
		x->deassert_wait    = 60;
		
		shgrip_qos_start();
		spi_message_init(&msg);
		
		x->rx_buf           = readdata;
		spi_message_add_tail(x, &msg);
		
		shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		
		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			SHGRIP_ERR("spi_sync err ret=%d \n", ret);
			shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
		}

		for (i = 0; i < rlen; i++) {
			*rbuf = bitflip8(readdata[i]);
			SHGRIP_DBG("readdata[%d]:0x%02x -> 0x%02x\n", i, readdata[i], *rbuf);
			rbuf++;
		}
		
		if (ctrl->state != STATE_FW_DL) {
			shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		}
		shgrip_qos_end();
	} else {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		ret = -EINVAL;
	}
	
	return ret;
#else
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int i;
	int ret = 0;
	int time = SHGRIP_CTS_TIMEOUT_US;
	unsigned char readdata=0;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}

	if ((rlen > 0) && (rlen <= SPI_BUFFER)) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;

		shgrip_qos_start();
		
		for (i = 0; i < rlen; i++) {
			spi_message_init(&msg);
			
			x->rx_buf           = &readdata;
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			
			*rbuf = bitflip8(readdata);
			SHGRIP_DBG("readdata[%d]:0x%02x -> 0x%02x\n", i, readdata, *rbuf);
			rbuf++;
			
			if (ctrl->state == STATE_FW_DL) {
				mb();
				udelay(10);
				time = SHGRIP_CTS_TIMEOUT_US;
				while ((gpio_get_value(shgrip_gpio_int))) {
					if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
						SHGRIP_ERR("cts is timeout\n");
						shgrip_qos_end();
						return -ETIMEDOUT;
					}
					shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
				}
			} else {
				shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
			}
		}
		shgrip_qos_end();
	} else {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_SPI_MULTI_TRANSFER */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_write_block                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_write_block(struct shgrip_drv *ctrl,
											unsigned char *wbuf, int wlen)
{
#ifdef SHGRIP_SPI_MULTI_TRANSFER
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char sendcmd, senddata[SPI_BUFFER];
	int i;
	int ret = 0;
	int time = SHGRIP_CTS_TIMEOUT_US;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if (ctrl->state == STATE_FW_DL) {
		if ((wlen > 0) && (wlen <= SPI_BUFFER)) {
			data = wbuf;
			
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SPI_BIT_WORD;
			x->len              = 1;
			x->speed_hz         = SPI_CLK_SPEED;
			
			shgrip_qos_start();
			
			for (i = 0; i < wlen; i++) {
				sendcmd = bitflip8(*data);
				SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, sendcmd);
				data++;
				
				spi_message_init(&msg);
				
				x->tx_buf           = &sendcmd;
				
				spi_message_add_tail(x, &msg);
				
				ret = spi_sync(spi_dev, &msg);
				if (ret) {
					SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
					shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
				}
				mb();
				udelay(10);
				time = SHGRIP_CTS_TIMEOUT_US;
				while ((gpio_get_value(shgrip_gpio_int))) {
					if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
						SHGRIP_ERR("cts is timeout\n");
						shgrip_qos_end();
						return -ETIMEDOUT;
					}
					shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
				}
			}
			shgrip_qos_end();
		} else {
			SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
			ret = -EINVAL;
		}
	} else {
		if ((wlen > 0) && (wlen <= SPI_BUFFER)) {
			data = wbuf;
			
			memset(&xfer, 0, sizeof(xfer));
			x = &xfer;
			x->bits_per_word    = SPI_BIT_WORD;
			x->len              = wlen;
			x->speed_hz         = SPI_CLK_SPEED;
			x->deassert_wait    = 60;
			
			for (i = 0; i < wlen; i++) {
				senddata[i] = bitflip8(*data);
				SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata[i]);
				data++;
			}
			
			shgrip_qos_start();
			spi_message_init(&msg);
			
			x->tx_buf           = senddata;
			
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			if (ctrl->state != STATE_FW_DL){
				shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
			}
			shgrip_qos_end();
		} else {
			SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
			ret = -EINVAL;
		}
	}
	return ret;
#else
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char senddata;
	int i;
	int time = SHGRIP_CTS_TIMEOUT_US;
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((wlen > 0) && (wlen <= SPI_BUFFER)) {
		data = wbuf;
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;
		
		shgrip_qos_start();

		for (i = 0; i < wlen; i++) {
			senddata = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata);
			data++;
			
			spi_message_init(&msg);
			
			x->tx_buf           = &senddata;
			
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			if (ctrl->state == STATE_FW_DL){
				mb();
				udelay(10);
				time = SHGRIP_CTS_TIMEOUT_US;
				while ((gpio_get_value(shgrip_gpio_int))) {
					if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
						SHGRIP_ERR("cts is timeout\n");
						shgrip_qos_end();
					return -ETIMEDOUT;
					}
					shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
				}
			} else {
				shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
			}
		}
		shgrip_qos_end();
	} else {
		SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_SPI_MULTI_TRANSFER */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_write_block_special                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_write_block_special(struct shgrip_drv *ctrl,
												unsigned char *headbuf, int headlen,
												const unsigned char *databuf, int datalen)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	unsigned int addr;
	int i;
	addr = ((headbuf[1] << 8) | (headbuf[2] << 16));

	SHGRIP_DBG("addr:0x%08x\n", addr);

	for (i = 1; i < (datalen + 1); i++) {
		if (!(i % 16)) {
			printk("0x%02X,\n", *databuf);
		} else {
			printk("0x%02X,", *databuf);
		}
		databuf++;
	}
	return 0;
#else /* SHGRIP_FW_DATA_NOT_WRITE */
#ifdef SHGRIP_FW_DATA_MULTI_WRITE
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	int i, len;
	int ret = 0;
	unsigned char buf[SPI_BUFFER];

	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((headlen > 0) && (datalen > 0) && (headlen+datalen <= SPI_BUFFER)) {
		data = headbuf;
		
		len = headlen+datalen;
		for (i = 0; i < len; i++) {
			buf[i] = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, buf[i]);
			data++;
			
			if (i==headlen-1) {
			    data = databuf;
			}
		}
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = len;
		x->speed_hz         = SPI_CLK_SPEED;
		x->deassert_wait    = 30;
		
		shgrip_qos_start();
		spi_message_init(&msg);
		
		x->tx_buf           = buf;
		
		spi_message_add_tail(x, &msg);
		
		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
			shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
		}
		shgrip_qos_end();
	} else {
		SHGRIP_ERR("len fraud. headlen=%d datalen=%d\n", headlen, datalen);
		ret = -EINVAL;
	}
	return ret;
#else /* SHGRIP_FW_DATA_MULTI_WRITE */
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char senddata;
	int i, len;
	int ret = 0;
	int time = SHGRIP_CTS_TIMEOUT_US;

	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((headlen > 0) && (datalen > 0) && (headlen+datalen <= SPI_BUFFER)) {
		data = headbuf;
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;
		
		len = headlen+datalen;

		shgrip_qos_start();

		for (i = 0; i < len; i++) {
			senddata = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata);
			data++;
			
			if (i==headlen-1) {
			    data = databuf;
			}
			
			spi_message_init(&msg);
			
			x->tx_buf           = &senddata;
			
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
			    SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			
			mb();
			udelay(10);
			time = SHGRIP_CTS_TIMEOUT_US;
			while ((gpio_get_value(shgrip_gpio_int))) {
				if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
					SHGRIP_ERR("cts is timeout\n");
					shgrip_qos_end();
					return -ETIMEDOUT;
				}
				shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
			}
		}
		shgrip_qos_end();
	} else {
		SHGRIP_ERR("len fraud. headlen=%d datalen=%d\n", headlen, datalen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_FW_DATA_MULTI_WRITE */
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_phy                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_phy(struct shgrip_drv *ctrl,
										unsigned char *wbuf, int wlen,
										unsigned char *rbuf, int rlen)
{
	int i;
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_CS_WAIT_US);
	
	ret = shgrip_command_dummy(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_dummy err. ret=%d\n", ret);
		goto err_spi_transfer;
	}
	
	shgrip_sys_delay_us(SHGRIP_COMREQ_WAIT_US);
	
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_HI);
	
	i = SHGRIP_COMRDY_WAIT_MAX_US;
	while ((gpio_get_value(shgrip_gpio_com_rdy))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("COMRDY check is timeout cmd[%02X]\n", wbuf[0]);
			goto err_spi_transfer;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("COMRDY wait:%d us\n", (SHGRIP_COMRDY_WAIT_MAX_US - i));
	
	ret = shgrip_spi_transfer_write_block(ctrl, wbuf, wlen);
	if (ret) {
		SHGRIP_ERR("spi write err. ret=%d\n", ret);
		goto err_spi_transfer;
	}
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMD_US);
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_HI);
	
	i = SHGRIP_COMRDY_SEND_WAIT_MAX_US;
	while ((gpio_get_value(shgrip_gpio_com_rdy))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("COMRDY check is timeout (SEND) cmd[%02X]\n", wbuf[0]);
			goto err_spi_transfer;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("COMRDY (SEND) wait:%d us\n", (SHGRIP_COMRDY_SEND_WAIT_MAX_US - i));
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMD_US);
	
	ret = shgrip_spi_transfer_read_block(ctrl, rbuf, rlen);
	if (ret) {
		SHGRIP_ERR("spi read err. ret=%d\n", ret);
		goto err_spi_transfer;
	}
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_HI);
	
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMDEND_US);

	return 0;
	
err_spi_transfer:
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_HI);
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_LO);
	shgrip_sys_delay_us(SHGRIP_WAIT_CMD_ERR_END_US);
	return -1;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer(struct shgrip_drv *ctrl,
										unsigned char *wbuf, int wlen,
										unsigned char *rbuf, int rlen)
{
	int i = 0;
	int ret = 0;
	
	for (i = 1; i <= 3; i++) {
		ret = shgrip_spi_transfer_phy(ctrl, wbuf, wlen, rbuf, rlen);
		if (!ret) {
			if (rbuf[0] != SHGRIP_CMD_ACK) {
				SHGRIP_ERR("Not Ack [0x%02X], Retry:%d\n", rbuf[0], i);
				ret = -1;
			} else {
				break;
			}
		} else {
			SHGRIP_ERR("spi_transfer Err, ret:%d, Retry:%d\n", ret, i);
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_for_loader                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_for_loader(struct shgrip_drv *ctrl,
											unsigned char *wbuf, int wlen,
											unsigned char *rbuf, int rlen)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_write_block(ctrl, wbuf, wlen);
	if (ret) {
		SHGRIP_ERR("spi send err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf) {
		ret = shgrip_spi_transfer_read_block(ctrl, rbuf, rlen);
		if (ret) {
			SHGRIP_ERR("spi read err. ret=%d\n", ret);
			return ret;
		}
	}
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_HI);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_for_loader_special_write                              */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_for_loader_special_write(struct shgrip_drv *ctrl,
												unsigned char *headbuf, int headlen,
												const unsigned char *databuf, int datalen)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_write_block_special(ctrl, headbuf, headlen, databuf, datalen);
	if (ret) {
		SHGRIP_ERR("spi send err. ret=%d\n", ret);
		return ret;
	}
	
	gpio_set_value(shgrip_gpio_cs, SHGRIP_GPIO_VAL_HI);
	
	return 0;
}

/* ========================================================================= */
/* SHGRIP Device Control Function App Mode                                   */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_command_rdst                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_rdst(struct shgrip_drv *ctrl,
							unsigned char *grip, unsigned char *testch)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_RDST;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("RDST BAD Value %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
		return -EFAULT;
	}
	
	SHGRIP_INFO("RDST:%02x %02x %02x\n",
							rbuf[0], rbuf[1], rbuf[2]);
	
	*grip	= rbuf[1];
	*testch	= rbuf[2];
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_erdst                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_erdst(struct shgrip_drv *ctrl,
							unsigned char *i2c, unsigned char *internal)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[4];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_ERDST;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("ERDST BAD Value %02x %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
		return -EFAULT;
	}
	
	SHGRIP_INFO("ERDST:%02x %02x %02x %02x\n",
							rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
	
	*i2c		= rbuf[1];
	*internal	= rbuf[2];
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_spi_transfer                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_command_spi_transfer(struct shgrip_drv *ctrl,
										unsigned char *wbuf, int wlen,
										unsigned char *rbuf, int rlen, int func)
{
	int i = 0;
	int ret = 0;
	unsigned char grip = 0, testch = 0;
	unsigned char i2c = 0, internal = 0;
	
	
	for (i = 1; i <= 3; i++) {
		ret = shgrip_spi_transfer_phy(ctrl, wbuf, wlen, rbuf, rlen);
		if (!ret) {
			if (rbuf[0] != SHGRIP_CMD_ACK) {
				SHGRIP_ERR("Not Ack [0x%02X], Retry:%d\n", rbuf[0], i);
				ret = shgrip_command_rdst(ctrl, &grip, &testch);
				if (ret) {
					SHGRIP_ERR("shgrip_command_rdst err. ret=%d\n", ret);
				} else {
					SHGRIP_ERR("RDST I2C/SPI Err_Bit Val: %02x\n", grip);
				}
				ret = shgrip_command_erdst(ctrl, &i2c, &internal);
				if (ret) {
					SHGRIP_ERR("shgrip_command_erdst err. ret=%d\n", ret);
					return GRIP_RESULT_FAILURE;
 				} else {
					SHGRIP_ERR("ERDST I2C Err_Bit Val: %02x\n", i2c);
				}
				if (func == 1) {
					if (grip & 0x08) { /* GRIP SENSOR STATE BIT:0000 1000 */
						return 0;
					}
				} else {
					if (func == 2) {
						if (!(grip & 0x08)) { /* GRIP SENSOR STATE BIT:0000 1000 */
						return 0;
						}
					}
				}
				ret = -1;
			} else {
				break;
			}
		} else {
			SHGRIP_ERR("spi_transfer Err, ret:%d, Retry:%d\n", ret, i);
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_pver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_pver(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_PVER;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	ctrl->ver_info.pver = ((rbuf[1] << 8) | rbuf[2]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_prxregwr                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_prxregwr(struct shgrip_drv *ctrl, 
								struct shgrip_prx_reg *prx)
{
	int ret;
	unsigned char wbuf[4], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_PRXREGWR;
	wbuf[1]  = shgrip_sensor_ch;
	wbuf[2]  = prx->addrs;
	wbuf[3]  = prx->value;
	
	ret = shgrip_command_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_prxregrd                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_prxregrd(struct shgrip_drv *ctrl, 
								struct shgrip_prx_reg *prx)
{
	int ret;
	unsigned char wbuf[3], rbuf[2];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_PRXREGRD;
	wbuf[1]  = shgrip_sensor_ch;
	wbuf[2]  = prx->addrs;
	
	ret = shgrip_command_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	prx->value = rbuf[1];
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_dltwr                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_dltwr(struct shgrip_drv *ctrl, 
								struct shgrip_dlt_reg *dlt)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_DLTWR;
	wbuf[1]  = dlt->ch0;
	wbuf[2]  = dlt->ch1;
	wbuf[3]  = dlt->ch2;
	wbuf[4]  = dlt->ch3;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_dltrd                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_dltrd(struct shgrip_drv *ctrl, 
								struct shgrip_dlt_reg *dlt)
{
	int ret;
	unsigned char wbuf[1], rbuf[5];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_DLTRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#endif
/* ------------------------------------------------------------------------- */
/* shgrip_command_gstclr                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gstclr(struct shgrip_drv *ctrl, 
							unsigned char *grip, unsigned char *testch)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSTCLR;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("GSTCLR BAD Value %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
		return -EFAULT;
	}
	
	SHGRIP_INFO("GSTCLR:%02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
	
	*grip	= rbuf[1];
	*testch	= rbuf[2];
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_tson                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_tson(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[2], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TSON;
	wbuf[1] = shgrip_tson_mode;
	
	ret = shgrip_command_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 1);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_tsoff                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_tsoff(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[1], rbuf[1];

	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TSOFF;
	
	ret = shgrip_command_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 2);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_romc                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_romc(struct shgrip_drv *ctrl, unsigned short *sum_val)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_ROMC;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf),
									rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		*sum_val = (unsigned short)((rbuf[1] << 8) | rbuf[2]);
		SHGRIP_ERR("Ack Err ack[%02X], checksum[%04X]\n", rbuf[0], *sum_val);
		return -EFAULT;
	}
	
	*sum_val = (unsigned short)((rbuf[1] << 8) | rbuf[2]);
	SHGRIP_INFO("Ack [%02X], checksum[%04X]\n", rbuf[0], *sum_val);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_lver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_lver(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_LVER;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	ctrl->ver_info.lver0 = ((rbuf[1] << 8) | rbuf[2]);
	
	return 0;
}


/* ------------------------------------------------------------------------- */
/* shgrip_command_debug_rw_command                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_command_debug_rw_command(struct shgrip_drv *ctrl, 
											struct shgrip_dbg_command *cmd)
{
	int i;
	int ret;
	unsigned char wbuf[32+1], rbuf[32+1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = (unsigned char)(cmd->addr);
	
	if (cmd->w_size) {
		for (i = 0; i < cmd->w_size; i++) {
			wbuf[i+1] = cmd->w_buf[i];
		}
	}
	
	ret = shgrip_spi_transfer(ctrl, wbuf, (cmd->w_size + 1), rbuf, (cmd->r_size + 1));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	if (cmd->r_size) {
		for (i = 0; i < cmd->r_size; i++) {
			cmd->r_buf[i] = rbuf[i+1];
			SHGRIP_INFO("rbuf[%d]:0x%02X\n", (i + 1), cmd->r_buf[i]);
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_cdltwr                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_cdltwr(struct shgrip_drv *ctrl, 
								struct shgrip_cdlt_reg *cdlt)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_CDLTWR;
	wbuf[1]  = cdlt->ch0;
	wbuf[2]  = cdlt->ch1;
	wbuf[3]  = cdlt->ch2;
	wbuf[4]  = cdlt->ch3;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_cdltrd                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_cdltrd(struct shgrip_drv *ctrl, 
								struct shgrip_cdlt_reg *cdlt)
{
	int ret;
	unsigned char wbuf[1], rbuf[5];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_CDLTRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#endif
/* ------------------------------------------------------------------------- */
/* shgrip_command_fltwr                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_fltwr(struct shgrip_drv *ctrl, 
								struct shgrip_flt_reg *flt)
{
	int ret;
	unsigned char wbuf[3], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_FLTWR;
	wbuf[1]  = flt->tg;
	wbuf[2]  = flt->tr;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_fltrd                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_fltrd(struct shgrip_drv *ctrl, 
								struct shgrip_flt_reg *flt)
{
	int ret;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_FLTRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#endif
/* ------------------------------------------------------------------------- */
/* shgrip_command_cfltwr                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_cfltwr(struct shgrip_drv *ctrl, 
								struct shgrip_cflt_reg *cflt)
{
	int ret;
	unsigned char wbuf[3], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_CFLTWR;
	wbuf[1]  = cflt->tc;
	wbuf[2]  = cflt->tu;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_cfltrd                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_cfltrd(struct shgrip_drv *ctrl, 
								struct shgrip_cflt_reg *cflt)
{
	int ret;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_CFLTRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#endif

/* ------------------------------------------------------------------------- */
/* shgrip_command_crdst                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_crdst(struct shgrip_drv *ctrl,
							unsigned char *cover, unsigned char *testch)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_CRDST;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("CRDST BAD Value %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
		return -EFAULT;
	}
	
	SHGRIP_INFO("CRDST:%02x %02x %02x\n",
							rbuf[0], rbuf[1], rbuf[2]);
	
	*cover	= rbuf[1];
	*testch	= (rbuf[2] & (BIT3 | BIT2 | BIT1 | BIT0));
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_prxcntrd                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_prxcntrd(struct shgrip_drv *ctrl, 
									struct shgrip_sensor_count *pcount,
									int sensor_count)
{
	int i;
	int ret = 0;
	unsigned char wbuf[2], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_PRXCNTRD;
	
	for(i = 0; i < sensor_count; i++){
		wbuf[1] = i;
		ret = shgrip_command_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf), 0);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			return ret;
		}
		
		if (rbuf[0] != SHGRIP_CMD_ACK) {
			SHGRIP_ERR("CNT BAD Value %02x %02x\n",
							rbuf[0], rbuf[1]);
			return -EFAULT;
		}
#if 1
		pcount->dcount_sensor[i].high_val = (rbuf[1] & 0x3F);
		pcount->dcount_sensor[i].low_val  = rbuf[2];
#else
		pcount->dcount_sensor[i].high_val = (rbuf[2] & 0x3F);
		pcount->dcount_sensor[i].low_val  = rbuf[1];
#endif
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_bromc                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_bromc(struct shgrip_drv *ctrl, unsigned short *sum_val)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_BROMC;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf),
									rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	*sum_val = (unsigned short)((rbuf[1] << 8) | rbuf[2]);
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err ack[%02X], boot checksum[%04X]\n", rbuf[0], *sum_val);
		return -EFAULT;
	}
	SHGRIP_INFO("Ack [%02X], boot checksum[%04X]\n", rbuf[0], *sum_val);
	
	return 0;
}
#ifndef SHGRIP_FACTORY_MODE_ENABLE
/* ------------------------------------------------------------------------- */
/* shgrip_command_extdevon                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_extdevon(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[1], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_EXTDEVON;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_extdevoff                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_extdevoff(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[1], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_EXTDEVOFF;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
/* ------------------------------------------------------------------------- */
/* shgrip_command_extdevfltwr                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_extdevfltwr(struct shgrip_drv *ctrl, 
								struct shgrip_extdevflt_reg *extdevflt)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_EXTDEVFLTWR;
	wbuf[1]  = extdevflt->te1;
	wbuf[2]  = extdevflt->te2;
	wbuf[3]  = extdevflt->imtc1;
	wbuf[4]  = extdevflt->imtc2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_extdevfltrd                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_extdevfltrd(struct shgrip_drv *ctrl, 
								struct shgrip_extdevflt_reg *extdevflt)
{
	int ret;
	unsigned char wbuf[1], rbuf[5];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_EXTDEVFLTRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}
#endif
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
/* ------------------------------------------------------------------------- */
/* shgrip_command_dummy                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_dummy(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_DUMMY;
	
	ret = shgrip_spi_transfer_write_block(ctrl, wbuf, sizeof(wbuf));
	if (ret) {
		SHGRIP_ERR("spi write err. ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* ========================================================================= */
/* SHGRIP Device Control Function Loader Mode                                */
/* ========================================================================= */
#ifndef SHGRIP_FW_DATA_NOT_WRITE
/* ------------------------------------------------------------------------- */
/* shgrip_ldr_read_status                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_read_status(struct shgrip_drv *ctrl, 
							unsigned char *srd0, unsigned char *srd1)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[2];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_READ_STS_REG;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("read status is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	*srd0 = rbuf[0];
	*srd1 = rbuf[1];
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_read_status_write_page_done                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_read_status_write_page_done(struct shgrip_drv *ctrl, 
							unsigned char *srd0, unsigned char *srd1)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[2];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_READ_STS_REG;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("read status is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	*srd0 = rbuf[0];
	*srd1 = rbuf[1];
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_clr_status                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_clr_status(struct shgrip_drv *ctrl)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_CLEAR_STS_REG;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												NULL, 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CLRSTS_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("read status is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	return 0;
}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */

#if SHGRIP_CHG_LDR_VERIFY_COUNT
/* ------------------------------------------------------------------------- */
/* shgrip_ldr_read_verify                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_read_verify(struct shgrip_drv *ctrl, unsigned long addr, int fw_index)
{
	int i;
    int ret = 0;
	unsigned char wbuf[3], rbuf[SHGRIP_LDR_PAGE_SIZE];
	unsigned char *img_ptr, *read_ptr;
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_PAGE_READ;
	wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
	wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_PGREAD_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("page read is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	i = SHGRIP_LDR_PAGE_SIZE;
	img_ptr  = (unsigned char*)&ctrl->fw_data[fw_index];
	read_ptr = rbuf;
	while (i-=1) {
		if (*img_ptr != *read_ptr) {
			SHGRIP_ERR("verify err  addr[%08lX], img[%02X], reead[%02X]\n", addr, *img_ptr, *read_ptr);
			return -EFAULT;
		}
		img_ptr++;
		read_ptr++;
	}
	
	return 0;
}
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_checkID                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_checkID(struct shgrip_drv *ctrl)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	return 0;
#else	
	int i;
	unsigned char wbuf[11], srd0, srd1;
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_ID_CHECK;
	wbuf[1] = SHGRIP_LDR_FW_ID1;
	wbuf[2] = SHGRIP_LDR_FW_ID2;
	wbuf[3] = SHGRIP_LDR_FW_ID3;
	wbuf[4] = SHGRIP_LDR_FW_ID4;
	wbuf[5] = SHGRIP_LDR_FW_ID5;
	wbuf[6] = SHGRIP_LDR_FW_ID6;
	wbuf[7] = SHGRIP_LDR_FW_ID7;
	wbuf[8] = SHGRIP_LDR_FW_ID8;
	wbuf[9] = SHGRIP_LDR_FW_ID9;
	wbuf[10] = SHGRIP_LDR_FW_ID10;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf), NULL, 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CHKID_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("Check ID is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
	if (ret) {
		SHGRIP_ERR("read status err. ret=%d\n", ret);
		return ret;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
		SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EBUSY;
	}
	
	if ((srd1 & SHGRIP_LDR_SRD_ID_CHK) != SHGRIP_LDR_SRD_ID_CHK ) {
		SHGRIP_ERR("ID err srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EINVAL;
	}
	
	return 0;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_erase_blodk                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_erase_blodk(struct shgrip_drv *ctrl)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	return 0;
#else	
	int i;
	int ret = 0, block = 0;
	unsigned char wbuf[4], srd0, srd1;
	unsigned long addr;
	
	SHGRIP_DBG("start\n");
	
	addr = shgrip_blk_addr_tbl_erase[block];
	
	while (addr != SHGRIP_LDR_BLK_ADDR_TBL_END) {
		ret = shgrip_ldr_clr_status(ctrl);
		if (ret) {
			SHGRIP_ERR("clear status err. ret=%d\n", ret);
			return ret;
		}
		
		wbuf[0] = SHGRIP_LDR_CMD_BLOCK_ERASE;
		wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
		wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
		wbuf[3] = SHGRIP_LDR_ERASE_DO;
		
		ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
													NULL, 0);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			return ret;
		}
		
		i = SHGRIP_ERASE_TIMEOUT_US;
		while ((gpio_get_value(shgrip_gpio_int))) {
			if (!(i-=SHGRIP_ERACE_POLL_WAIT_US)) {
				SHGRIP_ERR("Erase is timeout\n");
				return -ETIMEDOUT;
			}
			shgrip_sys_delay_us(SHGRIP_ERACE_POLL_WAIT_US);
		};
		
		ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
		if (ret) {
			SHGRIP_ERR("read status err. ret=%d\n", ret);
			return ret;
		}
		
		if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
			SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
			/* T.B.D:  retry or error */
			return -EBUSY;
		}
		
		if ((srd0 & SHGRIP_LDR_SRD_ERZ) != 0) {
			SHGRIP_ERR("Erase err srd0[%02X] srd1[%02X]\n", srd0, srd1);
			/* T.B.D:  retry or error */
			return -EFAULT;
		}
		
		block++;
		addr = shgrip_blk_addr_tbl_erase[block];
	}
	
	return 0;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_write_page                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_write_page(struct shgrip_drv *ctrl, unsigned long addr, int fw_index)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	int ret = 0;
	unsigned char wbuf[3];
#else
	int i;
	int ret = 0;
	unsigned char wbuf[3], srd0, srd1;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	SHGRIP_DBG("start\n");
	
#ifndef SHGRIP_FW_DATA_NOT_WRITE	
	ret = shgrip_ldr_clr_status(ctrl);
	if (ret) {
		SHGRIP_ERR("clear status err. ret=%d\n", ret);
		return ret;
	}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	wbuf[0] = SHGRIP_LDR_CMD_PAGE_PROGRAM;
	wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
	wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
	
	ret = shgrip_spi_transfer_for_loader_special_write(ctrl, &wbuf[0], sizeof(wbuf),
			&ctrl->fw_data[fw_index], SHGRIP_LDR_PAGE_SIZE);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
#ifndef SHGRIP_FW_DATA_NOT_WRITE	
	i = SHGRIP_PGWRITE_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_PGWRITE_POLL_WAIT_US)) {
			SHGRIP_ERR("pagewrite is timeout\n");
			return-ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_PGWRITE_POLL_WAIT_US);
	};
	
	if (addr == SHGRIP_LDR_ADDR_BLK_USER1_LAST) {
		ret = shgrip_ldr_read_status_write_page_done(ctrl, &srd0, &srd1);
	} else {
		ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
	}
	if (ret) {
		SHGRIP_ERR("read status err. ret=%d\n", ret);
		return ret;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
		SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EBUSY;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_PRG) != 0) {
		SHGRIP_ERR("Write err srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EFAULT;
	}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_write_block                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_write_block(struct shgrip_drv *ctrl, int block)
{
	int ret;
	unsigned long fw_index = 0;
	unsigned long dl_write_ptr;
	
	SHGRIP_DBG("start");

	fw_index = shgrip_block_fw_index_start_addr[block];
	dl_write_ptr = shgrip_block_addr_table_write[block][0];

	while (1) {
		ret = shgrip_ldr_write_page(ctrl, dl_write_ptr, fw_index);
		if (ret) {
			SHGRIP_ERR("Err write page ret[%d]\n", ret);
			return ret;
		}
		
		fw_index += SHGRIP_LDR_PAGE_SIZE;
		dl_write_ptr += SHGRIP_LDR_PAGE_SIZE;
		if (dl_write_ptr >= shgrip_block_tenminate[block]) {
			break;
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_write_block_verify                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_write_block_verify(struct shgrip_drv *ctrl, int block)
{
	int ret;
	unsigned long fw_index = 0;
	unsigned long dl_write_ptr;
#if SHGRIP_CHG_LDR_VERIFY_COUNT
	int verify_ret; 
	int verify_count = 0;
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */

	SHGRIP_DBG("start");
	
	fw_index = shgrip_block_fw_index_start_addr[block];
	dl_write_ptr = shgrip_block_addr_table_write[block][0];
	while (1) {
		ret = shgrip_ldr_write_page(ctrl, dl_write_ptr, fw_index);
		if (ret) {
			SHGRIP_ERR("Err write page ret[%d]\n", ret);
			return ret;
		}
		
#if SHGRIP_CHG_LDR_VERIFY_COUNT
		verify_ret = shgrip_ldr_read_verify(ctrl, dl_write_ptr, fw_index);
		if (verify_ret == 0) {
			verify_count = 0;
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */
			fw_index += SHGRIP_LDR_PAGE_SIZE;
			dl_write_ptr += SHGRIP_LDR_PAGE_SIZE;
			if (dl_write_ptr >= shgrip_block_tenminate[block]) {
				break;
			}
#if SHGRIP_CHG_LDR_VERIFY_COUNT
		} else {
			verify_count++;
			if (verify_count >= SHGRIP_CHG_LDR_VERIFY_COUNT){
				SHGRIP_ERR("Err verify ret[%d], count[%d]\n", verify_ret, verify_count);
				return -EAGAIN;
			}
		}
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_get_lver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_get_lver(struct shgrip_drv *ctrl)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[7];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_VERSION_INFO;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf), 
													rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("LVER is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	ctrl->ver_info.lver0  = (unsigned short)((rbuf[0] << 8) | rbuf[1]);
	ctrl->ver_info.lver1  = 0;
	ctrl->ver_info.pver   = (unsigned short)((rbuf[2] << 8) | rbuf[3]);
	
	SHGRIP_INFO("lver lver[%04X] pver[%04X]\n", ctrl->ver_info.lver0,
												ctrl->ver_info.pver);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_get_romc                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_get_romc(struct shgrip_drv *ctrl)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[6];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_ROMC;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf), 
													rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return -EFAULT;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("LVER is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	if (rbuf[2] == SHGRIP_CMD_ACK) {
		if (rbuf[5] == SHGRIP_CMD_ACK) {
			return SHGRIP_CMD_ACK;
		}else {
			SHGRIP_ERR("SUM2 is wrong, SUM2:0x%02X\n", rbuf[5]);
			return SHGRIP_CMD_NACK;
		}
	} else if (rbuf[2] == SHGRIP_CMD_NACK) {
		SHGRIP_ERR("SUM1 is Nack\n");
		return SHGRIP_CMD_NACK;
	} else {
		if ((rbuf[5] == SHGRIP_CMD_ACK) || (rbuf[5] == SHGRIP_CMD_NACK)) {
			SHGRIP_ERR("SUM1 is wrong, SUM1:0x%02X\n", rbuf[2]);
			return SHGRIP_CMD_NACK;
		} else {
			SHGRIP_ERR("Device connect Err\n");
		}
	}
	return -EFAULT;
}


/* ========================================================================= */
/* SHGRIP Sequence Function                                                  */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_start_app                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_start_app(struct shgrip_drv *ctrl)
{
	int time = SHGRIP_RESET_TIMEOUT_US;
	int ret;
	int retry;
	unsigned char grip = 0, testch = 0;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_HI);
	
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_STARTUP_WAIT);
	
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(time-=SHGRIP_POLL_RESET_WAIT_US)) {
			SHGRIP_ERR("reset is timeout\n");
#ifndef SHGRIP_FACTORY_MODE_ENABLE
			return -1;
#else
			shgrip_fwdl_disconnect = true;
			SHGRIP_DBG("ctrl->state: STATE_SENSOR_OFF\n");
			ctrl->state = STATE_SENSOR_OFF;
			return 0;
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
		}
		shgrip_sys_delay_us(SHGRIP_POLL_RESET_WAIT_US);
	}
	
	SHGRIP_DBG("ctrl->state: STATE_SENSOR_OFF\n");
	ctrl->state = STATE_SENSOR_OFF;
	
	for (retry = 1; retry <= SHGRIP_GSTCLR_RETRY_COUNT; retry++) {
		ret = shgrip_command_gstclr(ctrl, &grip, &testch);
		if (ret) {
			SHGRIP_ERR("shgrip_command_gstclr failed, retry:%d\n", retry);
			if (retry == SHGRIP_GSTCLR_RETRY_COUNT) {
				SHGRIP_ERR("shgrip_command_gstclr retry_over\n");
				return ret;
			}
		} else {
			break;
		}
		shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
	}
	
#ifdef CONFIG_SHTERM
//	shterm_k_set_info(SHTERM_INFO_GRIP, 1);
#endif
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_start_loader                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_start_loader(struct shgrip_drv *ctrl)
{
	int i;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_HI);
	
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_POLL_RESET_WAIT_US);
	
	i = SHGRIP_CHGMODE_TIMEOUT_US;
	while ((gpio_get_value(shgrip_gpio_int))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("changing loader mode is timeout\n");
			return -ETIMEDOUT;
		}
		mb();
		udelay(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("ctrl->state: STATE_FW_DL\n");
	ctrl->state = STATE_FW_DL;
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_power_on                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_power_on(struct shgrip_drv *ctrl)
{
	int ret = 0;
	
	if (IS_ERR_OR_NULL(ctrl->reg_p)) {
		SHGRIP_ERR("Unable to get %s regulator\n", SHGRIP_PMIC_POW_NAME);
		return -1;
	}
	
	if (!regulator_is_enabled(ctrl->reg_p)) {
		ret = regulator_enable(ctrl->reg_p);
		if(ret != 0){
			SHGRIP_ERR("regulator_enable ret:%d",ret);
		} else {
			shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
			gpio_set_value(shgrip_gpio_pu, SHGRIP_GPIO_VAL_HI);
			shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_power_off(struct shgrip_drv *ctrl)
{
	int ret = 0;
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_PWR_OFF_WAIT_US);
	
	gpio_set_value(shgrip_gpio_com_req, SHGRIP_GPIO_VAL_LO);
	
	gpio_set_value(shgrip_gpio_pu, SHGRIP_GPIO_VAL_LO);
	
	if (IS_ERR_OR_NULL(ctrl->reg_p)) {
		SHGRIP_ERR("Unable to get %s regulator\n", SHGRIP_PMIC_POW_NAME);
		return -1;
	}
	
	if (regulator_is_enabled(ctrl->reg_p)) {
		ret = regulator_disable(ctrl->reg_p);
		if(ret != 0){
			SHGRIP_ERR("regulator_disble ret:%d",ret);
		}
	}
	
	shgrip_sys_delay_us(SHGRIP_REGULATOR_OFF_WAIT_US);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_on                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_on(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
#ifdef CONFIG_ANDROID_ENGINEERING
	unsigned long sec,nsec;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_WARN("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	ret = shgrip_sys_request_irq(ctrl);
	if (ret) {
		SHGRIP_ERR("request_irq err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
	
	ret = shgrip_command_tson(ctrl);
	if (ret) {
		SHGRIP_DBG("shgrip_command_tson err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_ON \n");
	ctrl->state = STATE_SENSOR_ON;
	
	SHGRIP_DBG("enable_irq(gpio_to_irq(shgrip_gpio_int)) \n");
	shgrip_sys_enable_irq(ctrl);

#ifdef CONFIG_ANDROID_ENGINEERING
	if (shgrip_th_interval_ms) {
		if (th_msec >= 1000) {
			sec  = th_msec / 1000;
			nsec = (th_msec % 1000) * 1000 * 1000;
		} else {
			sec  = 0;
			nsec = th_msec * 1000 * 1000;
		}
		
		if (hrtimer_flg == 0) {
			hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
			hrtimer_flg = 1;
		}
	}
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_off                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_off(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_OFF) {
		SHGRIP_WARN("state is STATE_SENSOR_OFF\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	ret = shgrip_command_tsoff(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsoff err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_OFF \n");
	ctrl->state = STATE_SENSOR_OFF;
	
	SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
	shgrip_sys_disable_irq(ctrl);
	shgrip_sys_disable_irq_wake(ctrl);
	
	shgrip_sys_free_irq(ctrl);

#ifndef SHGRIP_FACTORY_MODE_ENABLE
	if (shgrip_release_flg) {
		cancel_delayed_work_sync(&ctrl->release_work_data);
		shgrip_release_flg = 0;
		wake_unlock(&shgrip_release_wake_lock);
	}
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset                                                          */
/* ------------------------------------------------------------------------- */
static void shgrip_seq_reset(void)
{
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(shgrip_gpio_reset, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_STARTUP_WAIT);
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_recovery(struct shgrip_drv *ctrl)
{
	int i;
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		shgrip_sensor_ch = i;
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.com2_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com2 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.com3_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com3 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.com4_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com4 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.ltl_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d ltl err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.ltm_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d ltm err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.htl_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d htl err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(ctrl->bk_adj.htm_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d htm err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
	}
	ret = shgrip_command_dltwr(ctrl, &(ctrl->bk_adj.dlt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_dltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_cdltwr(ctrl, &(ctrl->bk_adj.cdlt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_cdltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_fltwr(ctrl, &(ctrl->bk_adj.flt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_fltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_cfltwr(ctrl, &(ctrl->bk_adj.cflt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_cfltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevfltwr(ctrl, &(ctrl->bk_adj.extdevflt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevfltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_set_sensor_adjust                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_set_sensor_adjust(struct shgrip_drv *ctrl,
										struct shgrip_sens_setting_params *data)
{
	int i;
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_prx_reg lth_l, lth_m, hth_l, hth_m;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		shgrip_sensor_ch = i;
		ret = shgrip_command_prxregwr(ctrl, &(data->com2_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com2 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(data->com3_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com3 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &(data->com4_val[i]));
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d com4 err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		
		lth_l.addrs = data->ltl_val[i].addrs;
		lth_m.addrs = data->ltm_val[i].addrs;
		if (shgrip_smem_lth[i] == SHGRIP_THR_VAL_LOW) {
			lth_l.value = data->ltl_val[i].value;
			lth_m.value = data->ltm_val[i].value;
		} else {
			data->ltl_val[i].value =  (shgrip_smem_lth[i] & 0x00FF);
			data->ltm_val[i].value = ((shgrip_smem_lth[i] >> 8) & 0x00FF);
			lth_l.value = data->ltl_val[i].value;
			lth_m.value = data->ltm_val[i].value;
		}
		ret = shgrip_command_prxregwr(ctrl, &lth_l);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d ltl err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &lth_m);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d ltm err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		
		hth_l.addrs = data->htl_val[i].addrs;
		hth_m.addrs = data->htm_val[i].addrs;
		if (shgrip_smem_hth[i] == SHGRIP_THR_VAL_LOW) {
			hth_l.value = data->htl_val[i].value;
			hth_m.value = data->htm_val[i].value;
		} else {
			data->htl_val[i].value =  (shgrip_smem_hth[i] & 0x00FF);
			data->htm_val[i].value = ((shgrip_smem_hth[i] >> 8) & 0x00FF);
			hth_l.value = data->htl_val[i].value;
			hth_m.value = data->htm_val[i].value;
		}
		ret = shgrip_command_prxregwr(ctrl, &hth_l);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d htl err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
		ret = shgrip_command_prxregwr(ctrl, &hth_m);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxreg ch%d htm err. ret=%d\n", i, ret);
			return GRIP_RESULT_FAILURE;
		}
	}
	
	ret = shgrip_command_dltwr(ctrl, &(data->dlt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_dltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_cdltwr(ctrl, &(data->cdlt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_cdltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_fltwr(ctrl, &(data->flt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_fltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_cfltwr(ctrl, &(data->cflt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_cfltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevfltwr(ctrl, &(data->extdevflt_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevfltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_extdevon(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevon err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	memcpy(&(ctrl->bk_adj), data, sizeof(struct shgrip_sens_setting_params));
	
	shgrip_set_sensor_adjust_flg = true;
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_state                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_state(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i;
	unsigned char grip, testch;
	unsigned char onoff = 0;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_rdst(ctrl, &grip, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_rdst err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	onoff = (grip & BIT0);
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		ctrl->ch_state[i] = ((testch >> i) & 0x01);
	}
	
	ctrl->sensor_state.state_grip = onoff;
	ctrl->sensor_state.ch0 = ctrl->ch_state[0];
	ctrl->sensor_state.ch1 = ctrl->ch_state[1];
	ctrl->sensor_state.ch2 = ctrl->ch_state[2];
	ctrl->sensor_state.ch3 = ctrl->ch_state[3];
	
	SHGRIP_INFO("state_grip:%d, ch0:%d, ch1:%d, ch2:%d, ch3:%d\n", 
									ctrl->sensor_state.state_grip,
									ctrl->sensor_state.ch0,
									ctrl->sensor_state.ch1,
									ctrl->sensor_state.ch2,
									ctrl->sensor_state.ch3);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_fw_version                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_fw_version(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if ((ctrl->state == STATE_SENSOR_ON) 
	 || (ctrl->state == STATE_CALIBRATION)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_pver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_pver err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_lver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_lver err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ctrl->ver_info.lver1 = 0;
	SHGRIP_INFO("lver lver0[0x%04X]  lver1[0x%04X]  pver[0x%04X]\n",
											ctrl->ver_info.lver0,
											ctrl->ver_info.lver1,
											ctrl->ver_info.pver);
	
	return GRIP_RESULT_SUCCESS;	
	
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw_proc                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw_proc(struct shgrip_drv *ctrl)
{
	int ret;
	int block = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_ldr_checkID(ctrl);
	if (ret) {
		SHGRIP_ERR("Err check ID ret[%d]\n", ret);
		return ret;
	}
	
	ret = shgrip_ldr_erase_blodk(ctrl);
	if (ret) {
		SHGRIP_ERR("Err erase block ret[%d]\n", ret);
		return ret;
	}
	
	for (block = 0; block < (sizeof(shgrip_block_tenminate) / sizeof(shgrip_block_tenminate[0])); block++) {
		if (block == 0) {
			ret = shgrip_ldr_write_block_verify(ctrl, block);
			if (ret == -EAGAIN) {
				SHGRIP_ERR("Err write block verify ret[%d]\n", ret);
				ret = shgrip_ldr_erase_blodk(ctrl);
				if (ret) {
					SHGRIP_ERR("Err erase block ret[%d]\n", ret);
					return ret;
				} else {
					ret = shgrip_ldr_write_block_verify(ctrl, block);
					if (ret == -EAGAIN) {
						SHGRIP_ERR("Err write block verify retry ret[%d]\n", ret);
						shgrip_seq_grip_power_off(ctrl);
						ctrl->state = STATE_POWER_OFF;
						ret = shgrip_seq_grip_power_on(ctrl);
						if (ret) {
							SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
							return ret;
						}
						ret = shgrip_seq_reset_start_app(ctrl);
						if (ret) {
							SHGRIP_ERR("reset_start_app NG ret[%d]\n", ret);
							return ret;
						}
						shgrip_ldr_verify_flg = 1;
						return -1;
					} else if (ret != 0) {
						SHGRIP_ERR("Err write block ret[%d]\n", ret);
						return ret;
					}
				}
			} else if (ret != 0) {
				SHGRIP_ERR("Err write block ret[%d]\n", ret);
				return ret;
			}
		} else {
			ret = shgrip_ldr_write_block(ctrl, block);
			if (ret) {
				SHGRIP_ERR("Err write block ret[%d]\n", ret);
				return ret;
			}
		}
	}
	
	ret = shgrip_seq_reset_start_loader(ctrl);
	if (ret) {
		SHGRIP_ERR("download_fw_proc loader Mode NG  ret[%d]\n", ret);
		return ret;
	}
	
	if ((ctrl->hw_revision != SHGRIP_HW_ES0) && (ctrl->hw_revision != SHGRIP_HW_ES1)){
		ret = shgrip_ldr_get_romc(ctrl);
		if (ret != SHGRIP_CMD_ACK) {
			SHGRIP_ERR("download_fw_proc loader Mode NG  ret[%d]\n", ret);
			return ret;
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_recdload_fw                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_recdload_fw(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
#ifndef SHGRIP_FW_DATA_NOT_WRITE
	int i = 0;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	SHGRIP_DBG("start\n");
	
	if (shgrip_fwdl_disconnect == true) {
		if (ctrl->state != STATE_SENSOR_OFF) {
			SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
			goto err_loader_mode;
		}
	} else {
		return GRIP_RESULT_SUCCESS;
	}
	
	if (!ctrl->fw_data) {
		SHGRIP_ERR("fw_data is NULL\n");
		return GRIP_RESULT_FAILURE_USER;
	}
	
#ifndef SHGRIP_FW_DATA_NOT_WRITE
	i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
	while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
		if (!(i--)) {
			SHGRIP_ERR("retry err change loader mode ret[%d]\n", ret);
			goto err_loader_mode;
		}
	}
	ret = shgrip_ldr_get_lver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_ldr_get_lver NG\n");
		goto err_loader_mode;
	}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	ret = shgrip_seq_download_fw_proc(ctrl);
	if (ret) {
		SHGRIP_ERR("download_fw_proc loader NG  ret[%d]\n", ret);
		goto err_loader_mode;
	}
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start NG after download\n");
		goto err_loader_mode;
	}
	return GRIP_RESULT_SUCCESS;
	
err_loader_mode:
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
	return GRIP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw(struct shgrip_drv *ctrl, int fw_block)
{
	int ret = GRIP_RESULT_SUCCESS;
#ifndef SHGRIP_FW_DATA_NOT_WRITE
	int i = 0;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if ((ctrl->state == STATE_SENSOR_ON) 
	 || (ctrl->state == STATE_CALIBRATION)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (!ctrl->fw_data) {
		SHGRIP_ERR("fw_data is NULL\n");
		return GRIP_RESULT_FAILURE_USER;
	}
	
#ifndef SHGRIP_FW_DATA_NOT_WRITE
	i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
	while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
		if (!(i--)) {
			SHGRIP_ERR("retry err change loader mode ret[%d]\n", ret);
			goto err_loader_mode;
		}
	}
	
	ret = shgrip_ldr_get_lver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_ldr_get_lver NG\n");
		goto err_loader_mode;
	}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	ret = shgrip_seq_download_fw_proc(ctrl);
	if (ret) {
		SHGRIP_ERR("download_fw_proc loader NG  ret[%d]\n", ret);
		goto err_loader_mode;
	}
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start NG after download\n");
		goto err_loader_mode;
	}
	return GRIP_RESULT_SUCCESS;
	
err_loader_mode:
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
	return GRIP_RESULT_FAILURE;
}

#ifndef SHGRIP_FACTORY_MODE_ENABLE
/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw_init                                               */
/* ------------------------------------------------------------------------- */
static void shgrip_seq_download_fw_init(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	
	SHGRIP_DBG("start\n");
	
	shgrip_dev_connect = true;
	ret = shgrip_seq_grip_power_on(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
		return;
	}
	gpio_direction_output(shgrip_gpio_cs, 1);

/* SH_BSP_CUST -> Add */
	if(spi_dev)
	{
		struct pinctrl *pin;
		struct pinctrl_state *pin_state;

		pin = devm_pinctrl_get(&spi_dev->dev);
		if(pin)
		{
			pin_state = pinctrl_lookup_state(pin, "grip_spi_active");
			if(pin_state){
				ret = pinctrl_select_state(pin, pin_state);
				if(ret){
					SHGRIP_ERR("GPIO(SPI) pinctrl_select_state: Err\n");
				} else {
					pin_state = NULL;
				}
			} else {
				SHGRIP_ERR("GPIO(SPI) pin_state: Null\n");	
			}
		}
	}
/* SH_BSP_CUST <- Add */

	for (i = 1; i <= SHGRIP_INIT_FWDL_RETRY_COUNT; i++) {
		ret = shgrip_seq_reset_start_loader(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_start_loader NG  ret[%d]\n", ret);
			shgrip_dev_connect = false;
			if (i == SHGRIP_INIT_FWDL_RETRY_COUNT) {
				break;
			} else {
				continue;
			}
		} else {
			shgrip_dev_connect = true;
		}
		if ((ctrl->hw_revision == SHGRIP_HW_ES0) || (ctrl->hw_revision == SHGRIP_HW_ES1)) {
			ret = SHGRIP_CMD_ACK;
			break;
		} else {
			ret = shgrip_ldr_get_romc(ctrl);
			if (ret == SHGRIP_CMD_ACK) {
				shgrip_dev_connect = true;
				break;
			} else if (ret == SHGRIP_CMD_NACK) {
				shgrip_dev_connect = true;
				break;
			} else {
				SHGRIP_ERR("Check Sum Err:%d Count:%d\n", ret, i);
				shgrip_dev_connect = false;
			}
		}
	}
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("GripSensor Not Connect\n");
		goto shgrip_fw_work_power_off;
	}
	
	if (ret == SHGRIP_CMD_ACK) {
		ret = shgrip_ldr_get_lver(ctrl);
		if (ret) {
			goto shgrip_fw_work_power_off;
		} else {
			if (ctrl->ver_info.pver == SHGRIP_PVER) {
#if 0
			if ((ctrl->ver_info.lver0 == SHGRIP_LVER) 
			&& (ctrl->ver_info.pver == SHGRIP_PVER)) {
#endif
				goto shgrip_fw_work_ready_mode;
			}
		}
	}
	
	ctrl->fw_data = (unsigned char *)shgrip_fw_image;
	for (i = 0; i < 2; i++) {
		ret = shgrip_seq_download_fw_proc(ctrl);
		if (ret) {
			SHGRIP_ERR("download_fw_proc NG  ret[%d]\n", ret);
			continue;
		} else {
			break;
		}
	}
	if (ret) {
		shgrip_dev_connect = false;
		if (shgrip_ldr_verify_flg == 1) {
			return;
		}
		goto shgrip_fw_work_power_off;
	}
	
shgrip_fw_work_ready_mode:
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start_app NG ret[%d]\n", ret);
		goto shgrip_fw_work_power_off;
	}
	shgrip_ready_mode = 1;
	return;
	
shgrip_fw_work_power_off:
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_check_sum                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_check_sum(struct shgrip_drv *ctrl, 
										unsigned short *sum_val)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if ((ctrl->state == STATE_SENSOR_ON) 
	 || (ctrl->state == STATE_CALIBRATION)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_romc(ctrl, sum_val);
	if (ret) {
		SHGRIP_DBG("shgrip_command_romc err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_debug_rw_command                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_debug_rw_command(struct shgrip_drv *ctrl, 
										struct shgrip_dbg_command *cmd)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_debug_rw_command(ctrl, cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_chprd2_value                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_chprd2_value(struct shgrip_drv *ctrl, struct shgrip_chprd2 *chprd2_pval)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	struct shgrip_sensor_count count_info;
	struct shgrip_thr thr_info;
	struct shgrip_prx_reg sensor;
	unsigned char grip, testch;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	memset(&count_info, 0x00, sizeof(struct shgrip_sensor_count));
	ret = shgrip_command_prxcntrd(ctrl, &count_info, 4);
	if (ret) {
		SHGRIP_ERR("shgrip_command_prxcntrd err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	memset(&thr_info, 0x00, sizeof(struct shgrip_thr));
	if(thr_flg == 0){
		for (i = 0; i < 4; i++) {
			shgrip_sensor_ch = i;

			sensor.addrs = 0x08;
			ret = shgrip_command_prxregrd(ctrl, &sensor);
			if (ret) {
				SHGRIP_ERR("shgrip_command_prxregrd 0x08 err. ret:%d\n", ret);
				return GRIP_RESULT_FAILURE;
			}
			thr_info.th_l_sensor[i].low_val  = sensor.value;

			sensor.addrs = 0x09;
			ret = shgrip_command_prxregrd(ctrl, &sensor);
			if (ret) {
				SHGRIP_ERR("shgrip_command_prxregrd 0x09 err. ret:%d\n", ret);
				return GRIP_RESULT_FAILURE;
			}
			thr_info.th_l_sensor[i].high_val = sensor.value;

			sensor.addrs = 0x0A;
			ret = shgrip_command_prxregrd(ctrl, &sensor);
			if (ret) {
				SHGRIP_ERR("shgrip_command_prxregrd 0x0A err. ret:%d\n", ret);
				return GRIP_RESULT_FAILURE;
			}
			thr_info.th_h_sensor[i].low_val  = sensor.value;

			sensor.addrs = 0x0B;
			ret = shgrip_command_prxregrd(ctrl, &sensor);
			if (ret) {
				SHGRIP_ERR("shgrip_command_prxregrd 0x0B err. ret:%d\n", ret);
				return GRIP_RESULT_FAILURE;
			}
			thr_info.th_h_sensor[i].high_val  = sensor.value;
			hthr[i] = ((thr_info.th_h_sensor[i].high_val      << 8) | thr_info.th_h_sensor[i].low_val);
			lthr[i] = ((thr_info.th_l_sensor[i].high_val      << 8) | thr_info.th_l_sensor[i].low_val);
		}
		thr_flg = 1;
	}
	
	ret = shgrip_command_rdst(ctrl, &grip, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_rdst err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	chprd2_pval->dcount_ch0.high_val = count_info.dcount_sensor[0].high_val;
	chprd2_pval->dcount_ch0.low_val  = count_info.dcount_sensor[0].low_val;
	chprd2_pval->dcount_ch2.high_val = count_info.dcount_sensor[1].high_val;
	chprd2_pval->dcount_ch2.low_val  = count_info.dcount_sensor[1].low_val;

	chprd2_pval->nref_ch0.high_val = thr_info.th_l_sensor[0].high_val;
	chprd2_pval->nref_ch0.low_val  = thr_info.th_l_sensor[0].low_val;
	chprd2_pval->nthr_ch0.high_val = thr_info.th_h_sensor[0].high_val;
	chprd2_pval->nthr_ch0.low_val  = thr_info.th_h_sensor[0].low_val;
	chprd2_pval->nref_ch2.high_val = thr_info.th_l_sensor[1].high_val;
	chprd2_pval->nref_ch2.low_val  = thr_info.th_l_sensor[1].low_val;
	chprd2_pval->nthr_ch2.high_val = thr_info.th_h_sensor[1].high_val;
	chprd2_pval->nthr_ch2.low_val  = thr_info.th_h_sensor[1].low_val;

	for (i = 0; i < 4; i++) {
		SHGRIP_INFO("CH%d D0:0x%02X%02X, hthr:0x%04X, lthr:0x%04X, rdst:0x%02X%02X\n",
			i,
			count_info.dcount_sensor[i].high_val, count_info.dcount_sensor[i].low_val,
			hthr[i],
			lthr[i],
			grip, testch);
	}
	
	SHGRIP_WARN("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			(count_info.dcount_sensor[0].high_val  << 8) | count_info.dcount_sensor[0].low_val,
			hthr[0],
			lthr[0],
			(count_info.dcount_sensor[1].high_val  << 8) | count_info.dcount_sensor[1].low_val,
			hthr[1],
			lthr[1],
			(count_info.dcount_sensor[2].high_val  << 8) | count_info.dcount_sensor[2].low_val,
			hthr[2],
			lthr[2],
			(count_info.dcount_sensor[3].high_val  << 8) | count_info.dcount_sensor[3].low_val,
			hthr[3],
			lthr[3]);
	

	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_sensor_count                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_sensor_count(struct shgrip_drv *ctrl,
										struct shgrip_sensor_count *pcount)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_prxcntrd(ctrl, pcount, 4);
	if (ret) {
		SHGRIP_ERR("shgrip_command_prxcntrd err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;		
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_sensor_threshold                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_sensor_threshold(struct shgrip_drv *ctrl, struct shgrip_thr *pthr)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	struct shgrip_prx_reg sensor;

	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	for (i = 0; i < 4; i++) {
		shgrip_sensor_ch = i;

		sensor.addrs = 0x08;
		ret = shgrip_command_prxregrd(ctrl, &sensor);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxregrd 0x08 err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		pthr->th_l_sensor[i].low_val  = sensor.value;

		sensor.addrs = 0x09;
		ret = shgrip_command_prxregrd(ctrl, &sensor);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxregrd 0x09 err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		pthr->th_l_sensor[i].high_val = sensor.value;

		sensor.addrs = 0x0A;
		ret = shgrip_command_prxregrd(ctrl, &sensor);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxregrd 0x0A err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		pthr->th_h_sensor[i].low_val  = sensor.value;

		sensor.addrs = 0x0B;
		ret = shgrip_command_prxregrd(ctrl, &sensor);
		if (ret) {
			SHGRIP_ERR("shgrip_command_prxregrd 0x0B err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		pthr->th_h_sensor[i].high_val  = sensor.value;
	}
	
	return GRIP_RESULT_SUCCESS;		
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_boot_check_sum                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_boot_check_sum(struct shgrip_drv *ctrl, 
										unsigned short *sum_val)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if ((ctrl->state == STATE_SENSOR_ON) 
	 || (ctrl->state == STATE_CALIBRATION)) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_bromc(ctrl, sum_val);
	if (ret) {
		SHGRIP_DBG("shgrip_command_bromc err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_cover_state                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_cover_state(struct shgrip_drv *ctrl, unsigned char *state)
{
	int ret = GRIP_RESULT_SUCCESS;
	unsigned char covered, testch;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_crdst(ctrl, &covered, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_crdst err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	*state = (covered & BIT0);
	
	return GRIP_RESULT_SUCCESS;
}
#if 1
/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_hw_revision                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_hw_revision(struct shgrip_drv *ctrl, 
										unsigned short *hw_revision)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	*hw_revision = (unsigned short)ctrl->hw_revision;
	
	return ret;
}
#endif
/* ------------------------------------------------------------------------- */
/* shgrip_seq_request_calibration                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_request_calibration(struct shgrip_drv *ctrl)
{
	
	int ret = GRIP_RESULT_SUCCESS;
	unsigned long w_shgrip_tson_mode = 0;
	
	struct shgrip_dlt_reg dlt;
	
	SHGRIP_DBG("start\n");
	
	if (ctrl->state != STATE_SENSOR_ON) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	shgrip_request_calibration_err_flg = true;
	
	ret = shgrip_command_tsoff(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsoff err. ret=%d to recovery\n", ret);
		
		shgrip_sys_disable_irq(ctrl);
		
		shgrip_seq_grip_power_off(ctrl);
		ctrl->state = STATE_POWER_OFF;
		
		ret = shgrip_seq_grip_power_on(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		
		ret = shgrip_seq_reset_start_app(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		if (shgrip_set_sensor_adjust_flg == true) {
			ret = shgrip_seq_reset_recovery(ctrl);
			if (ret) {
				SHGRIP_ERR("shgrip_seq_reset_recovery failed\n");
				return GRIP_RESULT_FAILURE;
			}
		}
	} else {
		ctrl->state = STATE_SENSOR_OFF;
		shgrip_sys_disable_irq(ctrl);
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevoff(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevoff err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	if (shgrip_release_flg) {
		cancel_delayed_work_sync(&ctrl->release_work_data);
		shgrip_release_flg = 0;
		wake_unlock(&shgrip_release_wake_lock);
	}

	if(ctrl->sensor_state.state_grip) {
		input_report_switch(ctrl->input, SW_GRIP_00, SHGRIP_OFF);
		input_sync(ctrl->input);
		
		SHGRIP_DBG("input_event sync code:0x%04X, onoff:%d\n", SW_GRIP_00, SHGRIP_OFF);
		
		ctrl->sensor_state.state_grip = 0;
		ctrl->sensor_state.ch0 = 0;
		ctrl->sensor_state.ch1 = 0;
		ctrl->sensor_state.ch2 = 0;
		ctrl->sensor_state.ch3 = 0;
		
#ifndef SHGRIP_FACTORY_MODE_ENABLE
		SHGRIP_DBG("msm_tps_set_grip_state called grip:%d\n", ctrl->sensor_state.state_grip);
		msm_tps_set_grip_state(ctrl->sensor_state.state_grip);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	}
	
	dlt.ch0 = 0x55;
	dlt.ch1 = 0x55;
	dlt.ch2 = 0x55;
	dlt.ch3 = 0x55;
	
	ret = shgrip_command_dltwr(ctrl, &dlt);
	if (ret) {
		SHGRIP_ERR("shgrip_command_dltwr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	w_shgrip_tson_mode = shgrip_tson_mode;
	
	shgrip_tson_mode = SHGRIP_PRM_SENSOR_ALL_ON;
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
	
	ret = shgrip_command_tson(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_tson err. ret=%d\n", ret);
		shgrip_tson_mode = w_shgrip_tson_mode;
		return GRIP_RESULT_FAILURE;
	}
	
	shgrip_tson_mode = w_shgrip_tson_mode;
	
	ctrl->state = STATE_CALIBRATION;
	shgrip_request_calibration_err_flg = false;
	shgrip_sys_delay_us(SHGRIP_CALIBRATION_WAIT);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_calibration_data                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_calibration_data(struct shgrip_drv *ctrl,
												struct shgrip_calibration_data *calib_data)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_sensor_count count_info;
	
	SHGRIP_DBG("start\n");
	
	if (ctrl->state != STATE_CALIBRATION) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	ret = shgrip_command_prxcntrd(ctrl, &count_info, 4);
	if (ret) {
		SHGRIP_ERR("shgrip_command_prxcntrd err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}

	calib_data->calib_count_sensor0 = ((count_info.dcount_sensor[0].high_val  << 8) | count_info.dcount_sensor[0].low_val);
	calib_data->calib_count_sensor1 = ((count_info.dcount_sensor[1].high_val  << 8) | count_info.dcount_sensor[1].low_val);
	calib_data->calib_count_sensor2 = ((count_info.dcount_sensor[2].high_val  << 8) | count_info.dcount_sensor[2].low_val);
	calib_data->calib_count_sensor3 = ((count_info.dcount_sensor[3].high_val  << 8) | count_info.dcount_sensor[3].low_val);
	
	shgrip_sys_delay_us(SHGRIP_COUNTVAL_READ_WAIT_US);
#if 0
	if((ctrl->hw_revision == SHGRIP_HW_PP2) 
	|| (ctrl->hw_revision == SHGRIP_HW_PP2_5) 
	|| (ctrl->hw_revision == SHGRIP_HW_PMP)) {
		shgrip_sys_delay_us(SHGRIP_COUNTVAL_READ_WAIT_US);
	} else {
		shgrip_sys_delay_us(SHGRIP_COUNTVAL_READ_WAIT_US_PP1);
	}
#endif
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_set_sensor_calibration                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_set_sensor_calibration(struct shgrip_drv *ctrl)
{
    int i;
    int ret;
    struct shgrip_prx_reg lth_l, lth_m, hth_l, hth_m;
    
    SHGRIP_DBG("start\n");
    
    if (((shgrip_calb_lth[0] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_calb_hth[0] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_calb_lth[1] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_calb_hth[1] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_calb_lth[2] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_calb_hth[2] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_calb_lth[3] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_calb_hth[3] != SHGRIP_THR_VAL_LOW))) {
        for (i = 0; i < 4; i++) {
            shgrip_smem_lth[i] = shgrip_calb_lth[i];
            shgrip_smem_hth[i] = shgrip_calb_hth[i];
        }
    }
   
    for (i = 0; i < 4; i++) {
        shgrip_sensor_ch = i;
        
        lth_l.addrs = ctrl->bk_adj.ltl_val[i].addrs;
        lth_m.addrs = ctrl->bk_adj.ltm_val[i].addrs;
        
        if (shgrip_smem_lth[i] == SHGRIP_THR_VAL_LOW) {
            lth_l.value = ctrl->bk_adj.ltl_val[i].value;
            lth_m.value = ctrl->bk_adj.ltm_val[i].value;
        } else {
            lth_l.value =  (shgrip_smem_lth[i] & 0x00FF);
            lth_m.value = ((shgrip_smem_lth[i] >> 8) & 0x00FF);
            ctrl->bk_adj.ltl_val[i].value = lth_l.value;
            ctrl->bk_adj.ltm_val[i].value = lth_m.value;
        }
        
        ret = shgrip_command_prxregwr(ctrl, &lth_l);
        if (ret) {
            SHGRIP_ERR("shgrip_command_prxreg ch%d ltl err. ret=%d\n", i, ret);
            return GRIP_RESULT_FAILURE;
        }
        ret = shgrip_command_prxregwr(ctrl, &lth_m);
        if (ret) {
            SHGRIP_ERR("shgrip_command_prxreg ch%d ltm err. ret=%d\n", i, ret);
            return GRIP_RESULT_FAILURE;
        }
        
        hth_l.addrs = ctrl->bk_adj.htl_val[i].addrs;
        hth_m.addrs = ctrl->bk_adj.htm_val[i].addrs;
        
        if (shgrip_smem_hth[i] == SHGRIP_THR_VAL_LOW) {
            hth_l.value = ctrl->bk_adj.htl_val[i].value;
            hth_m.value = ctrl->bk_adj.htm_val[i].value;
        } else {
            hth_l.value =  (shgrip_smem_hth[i] & 0x00FF);
            hth_m.value = ((shgrip_smem_hth[i] >> 8) & 0x00FF);
            ctrl->bk_adj.htl_val[i].value = hth_l.value;
            ctrl->bk_adj.htm_val[i].value = hth_m.value;
        }
        
        ret = shgrip_command_prxregwr(ctrl, &hth_l);
        if (ret) {
            SHGRIP_ERR("shgrip_command_prxreg ch%d htl err. ret=%d\n", i, ret);
            return GRIP_RESULT_FAILURE;
        }
        ret = shgrip_command_prxregwr(ctrl, &hth_m);
        if (ret) {
            SHGRIP_ERR("shgrip_command_prxreg ch%d htm err. ret=%d\n", i, ret);
            return GRIP_RESULT_FAILURE;
        }
    }
    
    ret = shgrip_command_dltwr(ctrl, &(ctrl->bk_adj.dlt_val));
    if (ret) {
        SHGRIP_ERR("shgrip_command_dltwr err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    shgrip_set_sensor_adjust_flg = true;
    
    return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_kernel_read_thres_file                                             */
/* ------------------------------------------------------------------------- */
static ssize_t shgrip_kernel_read_thres_file(struct file *fp, char *buf, size_t size, unsigned int offset)
{
    mm_segment_t old_fs;
    ssize_t res = 0;
    loff_t fpos = offset;
    
    if( buf == NULL ){
        SHGRIP_ERR("<buf_NULL_POINTER>\n");
        return res;
    }
    
    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->read(fp, buf, size, &fpos);
    set_fs(old_fs);
    
    return res;
}

/* ------------------------------------------------------------------------- */
/* shgrip_kernel_write_thres_file                                            */
/* ------------------------------------------------------------------------- */
static ssize_t shgrip_kernel_write_thres_file(struct file *fp, char *buf, size_t size)
{
    mm_segment_t old_fs;
    ssize_t res = 0;
    
    if( buf == NULL ){
        SHGRIP_ERR("<buf_NULL_POINTER>\n");
        return res;
    }
    
    old_fs = get_fs();
    set_fs(get_ds());
    res = fp->f_op->write(fp, buf, size, &fp->f_pos);
    set_fs(old_fs);
    
    return res;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_read_thres_calibration                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_read_thres_calibration(struct shgrip_drv *ctrl)
{
    int i, j;
    struct file *fp; 
    char buf[SHGRIP_CALB_BUF_SIZE];
    unsigned short shgrip_dur_lthr[4], shgrip_dur_hthr[4];
    
    SHGRIP_DBG("start\n");
    
    fp = filp_open(SHGRIP_CALB_THRESHOLD_FILE_PATH, O_RDWR, 0770);
    if (IS_ERR_OR_NULL(fp)) {
        SHGRIP_ERR("File create failed: %s err \n", SHGRIP_CALB_THRESHOLD_FILE_PATH);
        return GRIP_RESULT_FAILURE;
    }
    shgrip_kernel_read_thres_file(fp, buf, SHGRIP_CALB_BUF_SIZE, 0);
    
    for (i = j = 0; i < 4; i++) {
        shgrip_dur_lthr[i] = (((buf[j+1] << 8) & 0xFF00) | (buf[j]   & 0x00FF));
        shgrip_dur_hthr[i] = (((buf[j+3] << 8) & 0xFF00) | (buf[j+2] & 0x00FF));
        j = j + 4;
    }
    
    if (((shgrip_dur_lthr[0] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_dur_hthr[0] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_dur_lthr[1] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_dur_hthr[1] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_dur_lthr[2] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_dur_hthr[2] != SHGRIP_THR_VAL_LOW))
     && ((shgrip_dur_lthr[3] != SHGRIP_THR_VAL_LOW) &&
         (shgrip_dur_hthr[3] != SHGRIP_THR_VAL_LOW))) {
        for (i = 0; i < 4; i++) {
            shgrip_smem_lth[i] = shgrip_dur_lthr[i];
            shgrip_smem_hth[i] = shgrip_dur_hthr[i];
            SHGRIP_INFO("Update by Calibration lthr[%d]:0x%04X hthr[%d]:0x%04X\n", i, shgrip_smem_lth[i], i, shgrip_smem_hth[i]);
        }
    }
    
    filp_close(fp, NULL);
    
    return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_write_thres_calibration                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_write_thres_calibration(struct shgrip_drv *ctrl)
{
    int i, j;
    struct file *fp; 
    unsigned char buf[SHGRIP_CALB_BUF_SIZE];
    
    SHGRIP_DBG("start\n");
    
    fp = filp_open(SHGRIP_CALB_THRESHOLD_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC, 0770);
    if (IS_ERR_OR_NULL(fp)) {
        SHGRIP_ERR("File create failed: %s err \n", SHGRIP_CALB_THRESHOLD_FILE_PATH);
        return GRIP_RESULT_FAILURE;
    }
    
    for(i = j = 0; i < 4; i++) {
        buf[j]   =  (shgrip_calb_lth[i] & 0x00FF);
        buf[j+1] = ((shgrip_calb_lth[i] >> 8) & 0x00FF);
        buf[j+2] =  (shgrip_calb_hth[i] & 0x00FF);
        buf[j+3] = ((shgrip_calb_hth[i] >> 8) & 0x00FF);
        j = j + 4;
    }
    
    shgrip_kernel_write_thres_file(fp, buf, SHGRIP_CALB_BUF_SIZE);
    
    filp_close(fp, NULL);
    
    return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_calibration_complete                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_calibration_complete(struct shgrip_drv *ctrl,
										 struct shgrip_calibration_info *calib_info)
{
    int ret;
    int i, j, k;
    unsigned short dat[4][16], tmp;
    unsigned short max[4] = {0};
    unsigned short mid[4] = {0};
    unsigned short lst[4] = {0};
    unsigned short mst[4] = {0};
    unsigned short lth[4] = {0};
    unsigned short hth[4] = {0};
    unsigned short fixed_val[4] = { SHGRIP_CALB_THRES_CH0,
                                    SHGRIP_CALB_THRES_CH1,
                                    SHGRIP_CALB_THRES_CH2,
                                    SHGRIP_CALB_THRES_CH3 };

    SHGRIP_DBG("start\n");
    
    ret = shgrip_command_tsoff(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_tsoff err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
#if 0
    sh_smem_common = sh_smem_get_common_address();
    
    if (sh_smem_common == NULL) {
        shgrip_smem_a[0] = SHGRIP_THRES_DEF_CH0_A;
        shgrip_smem_b[0] = SHGRIP_THRES_DEF_CH0_B;
        shgrip_smem_c[0] = SHGRIP_THRES_DEF_CH0_C;
        shgrip_smem_a[1] = SHGRIP_THRES_DEF_CH1_A;
        shgrip_smem_b[1] = SHGRIP_THRES_DEF_CH1_B;
        shgrip_smem_c[1] = SHGRIP_THRES_DEF_CH1_C;
        shgrip_smem_a[2] = SHGRIP_THRES_DEF_CH2_A;
        shgrip_smem_b[2] = SHGRIP_THRES_DEF_CH2_B;
        shgrip_smem_c[2] = SHGRIP_THRES_DEF_CH2_C;
        shgrip_smem_a[3] = SHGRIP_THRES_DEF_CH3_A;
        shgrip_smem_b[3] = SHGRIP_THRES_DEF_CH3_B;
        shgrip_smem_c[3] = SHGRIP_THRES_DEF_CH3_C;
    } else {
		for(i = 0; i < 4; i++) {
			shgrip_smem_a[i] = sh_smem_common->sh_proxgrip_a[i];
			shgrip_smem_b[i] = sh_smem_common->sh_proxgrip_b[i];
			shgrip_smem_c[i] = sh_smem_common->sh_proxgrip_c[i];
		}
    }
#endif
    for (i = 0; i < 16; i++) {
        if (((calib_info->calib_data[i].calib_count_sensor0) + 
           (((calib_info->thres.val[3] << 8) & 0x00FF00) | (calib_info->thres.val[2] & 0x0000FF)))
            > SHGRIP_CALB_CNTRD_MAX) {
            goto roll_back;
        } else {
            dat[0][i] = calib_info->calib_data[i].calib_count_sensor0;
        }
        if (((calib_info->calib_data[i].calib_count_sensor1) + 
           (((calib_info->thres.val[7] << 8) & 0x00FF00) | (calib_info->thres.val[6] & 0x0000FF)))
            > SHGRIP_CALB_CNTRD_MAX) {
            goto roll_back;
        } else {
            dat[1][i] = calib_info->calib_data[i].calib_count_sensor1;
        }
        if (((calib_info->calib_data[i].calib_count_sensor2) + 
           (((calib_info->thres.val[11] << 8) & 0x00FF00) | (calib_info->thres.val[10] & 0x0000FF)))
            > SHGRIP_CALB_CNTRD_MAX) {
            goto roll_back;
        } else {
            dat[2][i] = calib_info->calib_data[i].calib_count_sensor2;
        }
        if (((calib_info->calib_data[i].calib_count_sensor3) + 
           (((calib_info->thres.val[15] << 8) & 0x00FF00) | (calib_info->thres.val[14] & 0x0000FF)))
            > SHGRIP_CALB_CNTRD_MAX) {
            goto roll_back;
        } else {
            dat[3][i] = calib_info->calib_data[i].calib_count_sensor3;
        }
        SHGRIP_DATA("DAT%02d : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", i, dat[0][i], dat[1][i], dat[2][i], dat[3][i]);
    }
    
    for (j = 0; j < 4; j++) {
        for (i = 0; i < 16; i++) {
            for (k = i + 1; k < 16; k++) {
                if (dat[j][i] > dat[j][k]) {
                    tmp = dat[j][i];
                    dat[j][i] = dat[j][k];
                    dat[j][k] = tmp;
                }
            }
        }
        mid[j] = dat[j][7];
        if (mid[j] > fixed_val[j]) {
            lst[j] = (mid[j] - fixed_val[j]);
        } else {
            lst[j] = 0;
        }
        if (mid[j] < (0xFFFF - fixed_val[j])) {
           mst[j] = (mid[j] + fixed_val[j]);
        } else {
            mst[j] = 0xFFFF;
        }
       if ((dat[j][0] < lst[j]) || (dat[j][15] > mst[j])) {
            goto roll_back;
        }
        max[j] = dat[j][15];
        lth[j] = (((calib_info->thres.val[(j*4)+1] << 8) & 0xFF00) | (calib_info->thres.val[(j*4)]   & 0x00FF));
        hth[j] = (((calib_info->thres.val[(j*4)+3] << 8) & 0xFF00) | (calib_info->thres.val[(j*4)+2] & 0x00FF));
    }
    SHGRIP_DATA("MID   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", mid[0], mid[1], mid[2], mid[3]);
    SHGRIP_DATA("FXD   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", fixed_val[0], fixed_val[1], fixed_val[2], fixed_val[3]);
    SHGRIP_DATA("LST   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", lst[0], lst[1], lst[2], lst[3]);
    SHGRIP_DATA("MST   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", mst[0], mst[1], mst[2], mst[3]);
    SHGRIP_DATA("MAX   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", max[0], max[1], max[2], max[3]);
    SHGRIP_DATA("LTH   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", lth[0], lth[1], lth[2], lth[3]);
    SHGRIP_DATA("HTH   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", hth[0], hth[1], hth[2], hth[3]);
    
#if 0
    shgrip_calb_lth[0] = shgrip_pomax_ch0 + ((shgrip_smem_b[0] + shgrip_smem_c[0]) * shgrip_pomax_ch0 / 100 );
    shgrip_calb_hth[0] = shgrip_pomax_ch0 + ((shgrip_smem_a[0] + shgrip_smem_c[0]) * shgrip_pomax_ch0 / 100 );
    shgrip_calb_lth[1] = shgrip_pomax_ch1 + ((shgrip_smem_b[1] + shgrip_smem_c[1]) * shgrip_pomax_ch1 / 100 );
    shgrip_calb_hth[1] = shgrip_pomax_ch1 + ((shgrip_smem_a[1] + shgrip_smem_c[1]) * shgrip_pomax_ch1 / 100 );
    shgrip_calb_lth[2] = shgrip_pomax_ch2 + ((shgrip_smem_b[2] + shgrip_smem_c[2]) * shgrip_pomax_ch2 / 100 );
    shgrip_calb_hth[2] = shgrip_pomax_ch2 + ((shgrip_smem_a[2] + shgrip_smem_c[2]) * shgrip_pomax_ch2 / 100 );
    shgrip_calb_lth[3] = shgrip_pomax_ch3 + ((shgrip_smem_b[3] + shgrip_smem_c[3]) * shgrip_pomax_ch3 / 100 );
    shgrip_calb_hth[3] = shgrip_pomax_ch3 + ((shgrip_smem_a[3] + shgrip_smem_c[3]) * shgrip_pomax_ch3 / 100 );
    
    if ((shgrip_calb_lth[0] == shgrip_calb_hth[0])
     && (shgrip_calb_lth[1] == shgrip_calb_hth[1])
     && (shgrip_calb_lth[2] == shgrip_calb_hth[2])
     && (shgrip_calb_lth[3] == shgrip_calb_hth[3])) {
		shgrip_calb_lth[0] = shgrip_pomax_ch0 + ((SHGRIP_THRES_DEF_CH0_B + SHGRIP_THRES_DEF_CH0_C) * shgrip_pomax_ch0 / 100 );
		shgrip_calb_hth[0] = shgrip_pomax_ch0 + ((SHGRIP_THRES_DEF_CH0_A + SHGRIP_THRES_DEF_CH0_C) * shgrip_pomax_ch0 / 100 );
		shgrip_calb_lth[1] = shgrip_pomax_ch1 + ((SHGRIP_THRES_DEF_CH1_B + SHGRIP_THRES_DEF_CH1_C) * shgrip_pomax_ch1 / 100 );
		shgrip_calb_hth[1] = shgrip_pomax_ch1 + ((SHGRIP_THRES_DEF_CH1_A + SHGRIP_THRES_DEF_CH1_C) * shgrip_pomax_ch1 / 100 );
		shgrip_calb_lth[2] = shgrip_pomax_ch2 + ((SHGRIP_THRES_DEF_CH2_B + SHGRIP_THRES_DEF_CH2_C) * shgrip_pomax_ch2 / 100 );
		shgrip_calb_hth[2] = shgrip_pomax_ch2 + ((SHGRIP_THRES_DEF_CH2_A + SHGRIP_THRES_DEF_CH2_C) * shgrip_pomax_ch2 / 100 );
		shgrip_calb_lth[3] = shgrip_pomax_ch3 + ((SHGRIP_THRES_DEF_CH3_B + SHGRIP_THRES_DEF_CH3_C) * shgrip_pomax_ch3 / 100 );
		shgrip_calb_hth[3] = shgrip_pomax_ch3 + ((SHGRIP_THRES_DEF_CH3_A + SHGRIP_THRES_DEF_CH3_C) * shgrip_pomax_ch3 / 100 );
	}
#endif
    SHGRIP_DATA("Calibration Success to Update\n" );
    
    for(i = 0; i < 4; i++) {
        shgrip_calb_lth[i] = max[i] + (((calib_info->thres.val[(i*4)+1] << 8) & 0xFF00) | (calib_info->thres.val[(i*4)]   & 0x00FF));
        shgrip_calb_hth[i] = max[i] + (((calib_info->thres.val[(i*4)+3] << 8) & 0xFF00) | (calib_info->thres.val[(i*4)+2] & 0x00FF));
    }

    ret = shgrip_seq_set_sensor_calibration(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_seq_set_sensor_calibration err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevon(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevon err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
	
    ret = shgrip_command_tson(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_tson err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    SHGRIP_DBG("state: STATE_SENSOR_ON \n");
    ctrl->state = STATE_SENSOR_ON;
    
    SHGRIP_DBG("enable_irq(gpio_to_irq(shgrip_gpio_int)) \n");
    shgrip_sys_enable_irq(ctrl);
    
    ret = shgrip_seq_write_thres_calibration(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_seq_write_thres_calibration err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    return GRIP_RESULT_SUCCESS;
    
roll_back:
    SHGRIP_DATA("Calibration Failed to Roll_Back \n" );
    if (i != 16) {
        SHGRIP_DATA("DAT%02d : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", i, dat[0][i], dat[1][i], dat[2][i], dat[3][i]);
    }
    SHGRIP_DATA("MID   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", mid[0], mid[1], mid[2], mid[3]);
    SHGRIP_DATA("FXD   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", fixed_val[0], fixed_val[1], fixed_val[2], fixed_val[3]);
    SHGRIP_DATA("LST   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", lst[0], lst[1], lst[2], lst[3]);
    SHGRIP_DATA("MST   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", mst[0], mst[1], mst[2], mst[3]);
    SHGRIP_DATA("MAX   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", max[0], max[1], max[2], max[3]);
    SHGRIP_DATA("LTH   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", lth[0], lth[1], lth[2], lth[3]);
    SHGRIP_DATA("HTH   : ch0[%04X] ch1[%04X] ch2[%04X] ch3[%04X]\n", hth[0], hth[1], hth[2], hth[3]);
     
    if (shgrip_set_sensor_adjust_flg == true) {
        ret = shgrip_seq_reset_recovery(ctrl);
        if (ret) {
            SHGRIP_ERR("shgrip_seq_reset_recovery failed\n");
            return GRIP_RESULT_FAILURE;
        }
    }
    
#ifndef SHGRIP_FACTORY_MODE_ENABLE
    ret = shgrip_command_extdevon(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_extdevon err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
    
    ret = shgrip_command_tson(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_tson err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    SHGRIP_DBG("state: STATE_SENSOR_ON \n");
    ctrl->state = STATE_SENSOR_ON;
    
    SHGRIP_DBG("enable_irq(gpio_to_irq(shgrip_gpio_int)) \n");
    shgrip_sys_enable_irq(ctrl);
    
    shgrip_request_calibration_err_flg = false;
    
    return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_calibration_rollback                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_calibration_rollback(struct shgrip_drv *ctrl)
{
    int ret;
    
    SHGRIP_DBG("start\n");
    
    ret = shgrip_command_tsoff(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_tsoff err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    if (shgrip_set_sensor_adjust_flg == true) {
        ret = shgrip_seq_reset_recovery(ctrl);
        if (ret) {
            SHGRIP_ERR("shgrip_seq_reset_recovery failed\n");
            return GRIP_RESULT_FAILURE;
        }
    }
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevon(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevon err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
	
    ret = shgrip_command_tson(ctrl);
    if (ret) {
        SHGRIP_ERR("shgrip_command_tson err. ret=%d\n", ret);
        return GRIP_RESULT_FAILURE;
    }
    
    SHGRIP_DBG("state: STATE_SENSOR_ON \n");
    ctrl->state = STATE_SENSOR_ON;
    
    SHGRIP_DBG("enable_irq(gpio_to_irq(shgrip_gpio_int)) \n");
    shgrip_sys_enable_irq(ctrl);
    shgrip_request_calibration_err_flg = false;
    
    return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_end_calibration                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_end_calibration(struct shgrip_drv *ctrl, 
                                    struct shgrip_calibration_info *calib_info)
{
    int ret;
    
    SHGRIP_DBG("start\n");
    
    if ((ctrl->state == STATE_POWER_OFF)
     || (ctrl->state == STATE_FW_DL)) {
        SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
        return GRIP_RESULT_FAILURE_USER;
    }
    
    if ((ctrl->state == STATE_SENSOR_ON)
     || (ctrl->state == STATE_SENSOR_OFF)) {
		if(shgrip_request_calibration_err_flg == false) {
			SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
			return GRIP_RESULT_FAILURE_USER;
		}
    }
    
    if (!(calib_info->calib_connect)) {
        ret = shgrip_seq_calibration_rollback(ctrl);
        if (ret) {
            SHGRIP_ERR("shgrip_seq_calibration_rollback failed\n");
        }
        return ret;
    }
    
    if ((ctrl->state == STATE_SENSOR_OFF)
     || (ctrl->state == STATE_SENSOR_ON)) {
        SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
        return GRIP_RESULT_FAILURE_USER;
    }
    
    ret = shgrip_seq_calibration_complete(ctrl, calib_info);
    if (ret) {
        SHGRIP_ERR("shgrip_seq_calibration_complete failed\n");
        return ret;
    }
    
    return GRIP_RESULT_SUCCESS;
}

/* ========================================================================= */
/* SHGRIP ioctl Function                                                     */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_grip_sensor_on                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_grip_sensor_on(void)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_grip_sensor_on(ctrl);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_grip_sensor_off                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_grip_sensor_off(void)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_grip_sensor_off(ctrl);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_sens_setting_params data;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&data, argp,
							sizeof(struct shgrip_sens_setting_params));
	
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_set_sensor_adjust(ctrl, &data);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	if (ctrl->state == STATE_CALIBRATION) {
		SHGRIP_ERR("state is STATE_CALIBRATION\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = copy_to_user(argp, &(ctrl->bk_adj.setting_val), 
							sizeof(struct shgrip_user_setting));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_state                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_state(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_state(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_state failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &ctrl->sensor_state, 
							sizeof(struct shgrip_sensor_state));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_fw_version                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_fw_version(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_fw_version(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_fw_version failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &(ctrl->ver_info), 
							sizeof(struct shgrip_fw_version));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_download_fw                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_fw_data fw_info;
	unsigned char *fw_data;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&fw_info, argp, 
							sizeof(struct shgrip_fw_data));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	if(fw_info.size < 0 || fw_info.size > SHGRIP_FW_SIZE){
		SHGRIP_ERR("fw_info.size err\n");
		return GRIP_RESULT_FAILURE;
	}
	
	fw_data = kmalloc(fw_info.size, GFP_KERNEL);
	if (!fw_data) {
		SHGRIP_ERR("kmalloc failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	if(!fw_info.data){
		SHGRIP_ERR("fw_info.data is NULL\n");
		kfree(fw_data);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = copy_from_user(fw_data, fw_info.data, fw_info.size);
	if (ret) {
		SHGRIP_ERR("copy_from_user fw.data failed\n");
		ret = GRIP_RESULT_FAILURE;
		goto dl_fw_func_done;
	}
	
	ctrl->fw_data = fw_data;
	
	ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
	}
	
dl_fw_func_done:
	kfree(fw_data);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_download_fw                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	int fw_block;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&fw_block, argp, sizeof(int));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ctrl->fw_data = (unsigned char *)shgrip_fw_image;
	
	ret = shgrip_seq_diag_recdload_fw(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_recdload_fw failed\n");
		return ret;
	}
	if (shgrip_fwdl_disconnect == false) {
		ret = shgrip_seq_download_fw(ctrl, fw_block);
	} else {
		shgrip_fwdl_disconnect = false;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_check_sum                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_check_sum(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	unsigned short sum_val = 0;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_diag_check_sum(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_diag_check_sum failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &sum_val, sizeof(unsigned short));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_drv_status                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_drv_status(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_to_user(argp, &(ctrl->state), sizeof(unsigned char));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_debug_rw_command                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_debug_rw_command(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_dbg_command cmd;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&cmd, 0, sizeof(struct shgrip_dbg_command));
	
	ret = copy_from_user(&cmd, argp, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_debug_rw_command(ctrl, &cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &cmd, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_chprd2_value                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_chprd2_value(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_chprd2 val;
		
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&val, 0, sizeof(struct shgrip_chprd2));
	
	thr_flg = 0;
	
	ret = shgrip_seq_get_chprd2_value(ctrl, &val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_chprd2_value failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &val, sizeof(struct shgrip_chprd2));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_reset                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_reset(void)
{
	SHGRIP_DBG("start\n");
	
	shgrip_seq_reset();
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_sensor_count                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_sensor_count(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_sensor_count count_info;	

	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&count_info, 0x00, sizeof(struct shgrip_sensor_count));
	ret = shgrip_seq_get_sensor_count(ctrl, &count_info);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_sensor_count failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &count_info, sizeof(struct shgrip_sensor_count));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_sensor_threshold                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_sensor_threshold(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_thr thr_info;	

	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&thr_info, 0x00, sizeof(struct shgrip_thr));
	ret = shgrip_seq_get_sensor_threshold(ctrl, &thr_info);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_sensor_threshold failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &thr_info, sizeof(struct shgrip_thr));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_boot_check_sum                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_boot_check_sum(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	unsigned short sum_val = 0;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_diag_boot_check_sum(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_diag_ldr_check_sum failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &sum_val, sizeof(unsigned short));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_cover_state                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_cover_state(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	unsigned char state;

	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_cover_state(ctrl, &state);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_cover_state failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &state, sizeof(unsigned char));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}
#if 1
/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_hw_revision                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_hw_revision(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	unsigned short hw_revision;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_hw_revision(ctrl, &hw_revision);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_hw_revision failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &hw_revision, sizeof(unsigned short));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}
#endif
/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_request_calibration                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_request_calibration(void)
{

	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_request_calibration(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_request_calibration failed\n");
		return ret;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_calibration_data                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_calibration_data(void __user *argp)
{

	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_calibration_data calib_data;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_calibration_data(ctrl,&calib_data);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_calibration_data failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &calib_data, sizeof(struct shgrip_calibration_data));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_end_calibration                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_end_calibration(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_calibration_info calib_info;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&calib_info, argp, sizeof(struct shgrip_calibration_info));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_end_calibration(ctrl, &calib_info);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_end_calibration failed\n");
		return ret;
	}
	
	return GRIP_RESULT_SUCCESS;
}
/* ========================================================================= */
/* SHGRIP Input Event Function                                               */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_release_work                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_release_work(struct work_struct *work)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:%d\n", ctrl->sensor_state.state_grip);
	msm_tps_set_grip_state(ctrl->sensor_state.state_grip);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	shgrip_release_flg = 0;
	
	mutex_unlock(&shgrip_io_lock);
	wake_unlock(&shgrip_release_wake_lock);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_input_event                                                        */
/* ------------------------------------------------------------------------- */
static void shgrip_input_event(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char grip = 0, onoff = 0, testch = 0;
	int code = 0;
	int i;
	
	for (i = 0; i < 3; i++) {
		ret = shgrip_command_gstclr(ctrl, &grip, &testch);
		if (ret) {
			SHGRIP_ERR("shgrip_command_gstclr failed, retry:%d\n", i);
		} else {
			if (!(grip & 0x30)) {
				onoff = (grip & BIT0);
				goto before_report;
			}
			break;
		}
	}
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
	
	ret = shgrip_seq_grip_power_on(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
		goto after_recovery;
	}
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
		goto after_recovery;
	}
	
	if (shgrip_set_sensor_adjust_flg == true) {
		ret = shgrip_seq_reset_recovery(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_recovery failed\n");
			goto after_recovery;
		}
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	ret = shgrip_command_extdevon(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_extdevon err. ret=%d\n", ret);
		goto after_recovery;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	ret = shub_api_reset_completely_still();
	if (ret) {
		SHGRIP_ERR("shub_api_reset_completely_still err. ret=%d\n", ret);
	}
	
	ret = shgrip_command_tson(ctrl);
	if (ret) {
		SHGRIP_DBG("shgrip_command_tson err. ret=%d\n", ret);
		goto after_recovery;
	}
	ctrl->state = STATE_SENSOR_ON;
	
	ret = shgrip_command_gstclr(ctrl, &grip, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_gstclr failed\n");
	} else {
		onoff = (grip & BIT0);
		goto before_report;
	}
	
after_recovery:
	onoff = SHGRIP_OFF;
	testch = 0;
	
before_report:
	code = SW_GRIP_00;
	input_report_switch(ctrl->input, code, onoff);
	input_sync(ctrl->input);
	
	SHGRIP_DBG("input_event sync code:0x%04X, onoff:%d\n", code, onoff);
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		ctrl->ch_state[i] = ((testch >> i) & 0x01);
	}
	
	ctrl->sensor_state.state_grip = onoff;
	ctrl->sensor_state.ch0 = ctrl->ch_state[0];
	ctrl->sensor_state.ch1 = ctrl->ch_state[1];
	ctrl->sensor_state.ch2 = ctrl->ch_state[2];
	ctrl->sensor_state.ch3 = ctrl->ch_state[3];
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	
	switch (ctrl->sensor_state.state_grip) {
	case SHGRIP_OFF:
		shgrip_release_flg = 1;
		wake_lock(&shgrip_release_wake_lock);
		queue_delayed_work(ctrl->work_queue, &ctrl->release_work_data, msecs_to_jiffies(SHGRIP_IRQ_RELEASE_WAIT_MS));
		break;
	case SHGRIP_ON:
		if (shgrip_release_flg) {
			cancel_delayed_work_sync(&ctrl->release_work_data);
			shgrip_release_flg = 0;
			wake_unlock(&shgrip_release_wake_lock);
		} else {
			SHGRIP_DBG("msm_tps_set_grip_state called grip:%d\n", ctrl->sensor_state.state_grip);
			msm_tps_set_grip_state(ctrl->sensor_state.state_grip);
		}
		break;
	default:
		SHGRIP_ERR("grip state failed, grip:%d\n", ctrl->sensor_state.state_grip);
		break;
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_work_func                                                          */
/* ------------------------------------------------------------------------- */
static void shgrip_work_func(struct work_struct *work)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = container_of(work, struct shgrip_drv, work_data);
	if (!ctrl) {
		SHGRIP_ERR("ctrl is NULL\n");
		goto shgrip_work_func_done;
	}
	if (ctrl->suspended) {
		SHGRIP_WARN("Already Suspended\n");
		goto shgrip_work_func_done;
	}
	switch (ctrl->state) { 
	case STATE_SENSOR_ON:
		shgrip_input_event(ctrl);
		shgrip_sys_enable_irq(ctrl);
		break;
	case STATE_POWER_OFF:
	case STATE_SENSOR_OFF:
	case STATE_FW_DL:
	case STATE_CALIBRATION:
	default:
		SHGRIP_WARN("state is changed state:%d\n", ctrl->state); 
		break;
	}
	
shgrip_work_func_done:
	mutex_unlock(&shgrip_io_lock);
	wake_unlock(&shgrip_wake_lock);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_irq_func                                                           */
/* ------------------------------------------------------------------------- */
static irqreturn_t shgrip_irq_func(int irq, void *dev)
{
	struct shgrip_drv *ctrl = dev;
	
	SHGRIP_DBG("start\n");
	
	wake_lock(&shgrip_wake_lock);
	
	shgrip_sys_disable_irq(ctrl);
	
	queue_work(ctrl->work_queue, &(ctrl->work_data));
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_ANDROID_ENGINEERING
/* ========================================================================= */
/* SHGRIP Debug Function                                                     */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_th_dump_func                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_th_dump_func(struct work_struct *work)
{
	int ret = 0;
	unsigned long sec,nsec;
	struct shgrip_drv *ctrl;
	struct shgrip_chprd2 chprd2_val;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = container_of(work, struct shgrip_drv, th_work_data);
	if (!ctrl) {
		SHGRIP_ERR("ctrl is NULL\n");
		goto shgrip_work_func_done;
	}
	
	switch (ctrl->state) { 
	case STATE_SENSOR_ON:
		if (shgrip_th_interval_ms) {
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
			ret = shgrip_seq_get_chprd2_value(ctrl, &chprd2_val);
			if (ret) {
				SHGRIP_ERR("shgrip_command_chprd2 err. ret:%d\n", ret);
			}
		}
		break;
	case STATE_POWER_OFF:
	case STATE_SENSOR_OFF:
	case STATE_FW_DL:
	default:
		th_msec = th_msec_tmp;
		hrtimer_flg = 0;
		thr_flg = 0;
		break;
	}
shgrip_work_func_done:
	mutex_unlock(&shgrip_io_lock);
	wake_unlock(&shgrip_wake_lock);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_th_timer_callback                                                  */
/* ------------------------------------------------------------------------- */
static enum hrtimer_restart shgrip_th_timer_callback(struct hrtimer *timer)
{
	struct shgrip_drv *ctrl = container_of(timer, struct shgrip_drv, shgrip_threshold_timer);
	wake_lock(&shgrip_wake_lock);
	queue_work(ctrl->work_queue, &(ctrl->th_work_data));
	return HRTIMER_NORESTART;
}
#endif /* CONFIG_ANDROID_ENGINEERING */



/* ========================================================================= */
/* SHGRIP Interface                                                          */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_open                                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct shgrip_drv *ctrl;
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		return -1;
	}
	
	mutex_lock(&shgrip_io_lock);
	wake_lock(&shgrip_io_wake_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	switch (ctrl->state) {
	case STATE_POWER_OFF:
		if (shgrip_ready_mode == 1) {
			ctrl->state = STATE_SENSOR_OFF;
			if (shgrip_calibration_flg == false) {
                ret = shgrip_seq_read_thres_calibration(ctrl);
                if (!ret) {
                    shgrip_calibration_flg = true;
                }
            }
			wake_unlock(&shgrip_io_wake_lock);
			mutex_unlock(&shgrip_io_lock);
			return 0;
		}
		ret = shgrip_seq_grip_power_on(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
			wake_unlock(&shgrip_io_wake_lock);
			mutex_unlock(&shgrip_io_lock);
			return -1;
		}
		ret = shgrip_seq_reset_start_app(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
			shgrip_seq_grip_power_off(ctrl);
			ctrl->state = STATE_POWER_OFF;
			wake_unlock(&shgrip_io_wake_lock);
			mutex_unlock(&shgrip_io_lock);
			return -1;
		}
		shgrip_ready_mode = 1;
		
		if (shgrip_calibration_flg == false) {
            ret= shgrip_seq_read_thres_calibration(ctrl);
            if  (!ret) {
                shgrip_calibration_flg = true;
            }
        }
		break;
	case STATE_SENSOR_OFF:
	case STATE_SENSOR_ON:
	case STATE_FW_DL:
	case STATE_CALIBRATION:
	default:
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		break;
	}
	
	wake_unlock(&shgrip_io_wake_lock);
	mutex_unlock(&shgrip_io_lock);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_close                                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_close(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct shgrip_drv *ctrl;
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		return -1;
	}
	
	mutex_lock(&shgrip_io_lock);
	wake_lock(&shgrip_io_wake_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	switch (ctrl->state) {
	case STATE_SENSOR_OFF:
		shgrip_fwdl_disconnect = false;
		break;
	case STATE_SENSOR_ON:
	case STATE_CALIBRATION:
		ret = shgrip_seq_grip_sensor_off(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_sensor_off failed\n");
			SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
			shgrip_sys_disable_irq(ctrl);
			shgrip_sys_disable_irq_wake(ctrl);
			shgrip_sys_free_irq(ctrl);
			shgrip_ready_mode = 0;
		}
		break;
	case STATE_POWER_OFF:
	case STATE_FW_DL:
	default:
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		wake_unlock(&shgrip_wake_lock);
		wake_unlock(&shgrip_io_wake_lock);
		mutex_unlock(&shgrip_io_lock);
		return 0;
	}
	
	ctrl->state = STATE_POWER_OFF;
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	if (shgrip_release_flg) {
		cancel_delayed_work_sync(&ctrl->release_work_data);
		shgrip_release_flg = 0;
		wake_unlock(&shgrip_release_wake_lock);
	}
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	wake_unlock(&shgrip_wake_lock);
	wake_unlock(&shgrip_io_wake_lock);
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_read                                                               */
/* ------------------------------------------------------------------------- */
static ssize_t shgrip_read(struct file* filp, char __user *buf, 
										size_t count, loff_t* offset)
{
	SHGRIP_DBG("start\n");
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_write                                                              */
/* ------------------------------------------------------------------------- */
static ssize_t shgrip_write(struct file* filp, const char __user *buf, 
										size_t count, loff_t* offset)
{
	SHGRIP_DBG("start\n");
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl                                                              */
/* ------------------------------------------------------------------------- */
static long shgrip_ioctl(struct file *file, unsigned int cmd, 
										unsigned long arg)
{
	int ret;
	void __user *argp = (void __user*)arg;

	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_lock(&shgrip_io_lock);
	wake_lock(&shgrip_io_wake_lock);
	
	SHGRIP_INFO("start, cmd:%d\n", (cmd & 0x000000FF));
	
	switch (cmd) {
	case SHGRIP_IOCTL_GRIP_SENSOR_ON:
		ret = shgrip_ioctl_grip_sensor_on();
		break;
	case SHGRIP_IOCTL_GRIP_SENSOR_OFF:
		ret = shgrip_ioctl_grip_sensor_off();
		break;
	case SHGRIP_IOCTL_SET_SENSOR_ADJUST:
		ret = shgrip_ioctl_set_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_GET_SENSOR_ADJUST:
		ret = shgrip_ioctl_get_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_GET_STATE:
		ret = shgrip_ioctl_get_state(argp);
		break;
	case SHGRIP_IOCTL_GET_FW_VERSION:
		ret = shgrip_ioctl_get_fw_version(argp);
		break;
	case SHGRIP_IOCTL_DOWNLOAD_FW:
		ret = shgrip_ioctl_download_fw(argp);
		break;
	case SHGRIP_IOCTL_DIAG_SET_SENSOR_ADJUST:
//		ret = shgrip_ioctl_diag_set_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_DIAG_GET_SENSOR_ADJUST:
//		ret = shgrip_ioctl_diag_get_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_DIAG_GET_SENSOR_LEVEL:
//		ret = shgrip_ioctl_diag_get_sensor_level(argp);
		break;
	case SHGRIP_IOCTL_DIAG_CHANGE_RUN_MODE:
//		ret = shgrip_ioctl_diag_change_run_mode(argp);
		break;
	case SHGRIP_IOCTL_DIAG_DOWNLOAD_FW:
		ret = shgrip_ioctl_diag_download_fw(argp);
		break;
	case SHGRIP_IOCTL_DIAG_CHECK_SUM:
		ret = shgrip_ioctl_diag_check_sum(argp);
		break;
	case SHGRIP_IOCTL_GET_DRV_STATUS:
		ret = shgrip_ioctl_get_drv_status(argp);
		break;
	case SHGRIP_IOCTL_GET_CHPRD2_VALUE:
		ret = shgrip_ioctl_get_chprd2_value(argp);
		break;
	case SHGRIP_IOCTL_DEBUG_RW_COMMAND:
		ret = shgrip_ioctl_debug_rw_command(argp);
		break;
	case SHGRIP_IOCTL_SET_BK_ADJUST:
//		ret = shgrip_ioctl_set_bk_adjust(argp);
		break;
	case SHGRIP_IOCTL_SET_RESET:
		ret = shgrip_ioctl_set_reset();
		break;
	case SHGRIP_IOCTL_GET_SENSOR_COUNT:
		ret = shgrip_ioctl_get_sensor_count(argp);
		break;
	case SHGRIP_IOCTL_GET_SENSOR_THRESHOLD:
		ret = shgrip_ioctl_get_sensor_threshold(argp);
		break;
	case SHGRIP_IOCTL_DIAG_BOOT_CHECK_SUM:
		ret = shgrip_ioctl_diag_boot_check_sum(argp);
		break;
	case SHGRIP_IOCTL_GET_COVER_STATUS:
		ret = shgrip_ioctl_get_cover_state(argp);
		break;
	case SHGRIP_IOCTL_GET_HW_REVISION:
		ret = shgrip_ioctl_get_hw_revision(argp);
		break;
	case SHGRIP_IOCTL_REQUEST_CALIBRATION:
		ret = shgrip_ioctl_request_calibration();
		break;
	case SHGRIP_IOCTL_GET_CALIBRATION_DATA:
		ret = shgrip_ioctl_get_calibration_data(argp);
		break;
	case SHGRIP_IOCTL_END_CALIBRATION:
		ret = shgrip_ioctl_end_calibration(argp);
		break;
	default:
		SHGRIP_DBG("invalid_value cmd:%d\n", (cmd & 0x000000FF));
		ret = GRIP_RESULT_FAILURE;
		break;
	}
	
	SHGRIP_INFO("end, cmd:%d, ret=%d\n", (cmd & 0x000000FF), ret);
	
	if (ret) {
		shgrip_ready_mode = 0;
	}
	wake_unlock(&shgrip_io_wake_lock);
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_spi_probe                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_dev_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct shgrip_drv *ctrl;
	struct device_node *node;
	struct pinctrl *pin;
	struct pinctrl_state *pin_state;
    unsigned long bootmode;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	spi_set_drvdata(spi, ctrl);
	spi_dev = spi;
	
	ctrl->state = STATE_POWER_OFF;
	
#ifdef CONFIG_ANDROID_ENGINEERING
	ret = device_create_file(&spi->dev, &dev_attr_shgrip_dev);
	if (ret){
		SHGRIP_ERR("device_create_file failed ret:%d\n", ret);
		return ret;
	}
	
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
	
	ctrl->hw_revision = sh_boot_get_hw_revision();
	
	ctrl->reg_p = regulator_get(&spi->dev, SHGRIP_PMIC_POW_NAME);
	if (IS_ERR_OR_NULL(ctrl->reg_p)) {
		SHGRIP_ERR("Unable to get %s regulator\n", SHGRIP_PMIC_POW_NAME);
		return -1;
	}	
	
	node = spi->dev.of_node;
	shgrip_gpio_cs = of_get_named_gpio(node, "sharp,spi-cs", 0);
	if (shgrip_gpio_cs < 0) {
		SHGRIP_ERR("GPIO(CS) of_get_named_gpio: Err");
		goto gpio_cs_err;
	}
	gpio_request(shgrip_gpio_cs, "shgrip_gpio_cs");
	
	shgrip_gpio_pu = of_get_named_gpio(node, "sharp,grip-pu", 0);
	if (shgrip_gpio_pu < 0) {
		SHGRIP_ERR("GPIO(PU) of_get_named_gpio: Err");
		goto gpio_pu_err;
	}
	gpio_request(shgrip_gpio_pu, "shgrip_gpio_pu");
	
	shgrip_gpio_int = of_get_named_gpio(node, "sharp,irq-int", 0);
	if (shgrip_gpio_int < 0) {
		SHGRIP_ERR("GPIO(INT) of_get_named_gpio: Err");
		goto gpio_int_err;
	}
	gpio_request(shgrip_gpio_int, "shgrip_gpio_int");
	
	shgrip_gpio_reset = of_get_named_gpio(node, "sharp,irq-rst", 0);
	if (shgrip_gpio_reset < 0) {
		SHGRIP_ERR("GPIO(RST) of_get_named_gpio: Err");
		goto gpio_rst_err;
	}
	gpio_request(shgrip_gpio_reset, "shgrip_gpio_reset");
	
	shgrip_gpio_com_req = of_get_named_gpio(node, "sharp,com-req", 0);
	if (shgrip_gpio_com_req < 0) {
		SHGRIP_ERR("GPIO(COMREQ) of_get_named_gpio: Err");
		goto gpio_comreq_err;
	}
	gpio_request(shgrip_gpio_com_req, "shgrip_gpio_com_req");
	
	shgrip_gpio_com_rdy = of_get_named_gpio(node, "sharp,com-rdy", 0);
	if (shgrip_gpio_com_rdy < 0) {
		SHGRIP_ERR("GPIO(COMRDY) of_get_named_gpio: Err");
		goto gpio_comrdy_err;
	}
	gpio_request(shgrip_gpio_com_rdy, "shgrip_gpio_com_rdy");
	
	pin = devm_pinctrl_get(&spi->dev);
	if(pin) {
		pin_state = pinctrl_lookup_state(pin, "grip_spi_cs_active");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(SPI_CS) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(SPI_CS) pin_state: Null\n");	
		}
		pin_state = pinctrl_lookup_state(pin, "grip_comreq_active");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(COMREQ) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(COMREQ) pin_state: Null\n");	
		}
		
		pin_state = pinctrl_lookup_state(pin, "grip_comrdy_active");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(COMRDY) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(COMRDY) pin_state: Null\n");	
		}
		
		pin_state = pinctrl_lookup_state(pin, "grip_int_active");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(INT) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(INT) pin_state: Null\n");	
		}
	}
	
	ctrl->irq = gpio_to_irq(shgrip_gpio_int);
	ret = shgrip_sys_request_irq(ctrl);
	if (ret) {
		SHGRIP_ERR("GPIO(INT) REQUEST IRQ NG\n");
		goto request_irq_err;
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	shgrip_seq_download_fw_init(ctrl);
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("FWDL(INIT) NG\n");
		goto fw_dl_err;
	}
#else
	ret = shgrip_seq_grip_power_on(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_grip_power_on failed ret:%d\n", ret);
		goto request_irq_err;
	}
	gpio_direction_output(shgrip_gpio_cs, 1);

/* SH_BSP_CUST -> Add */
	if(pin)
	{
		pin_state = pinctrl_lookup_state(pin, "grip_spi_active");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(SPI) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(SPI) pin_state: Null\n");	
		}
	}
/* SH_BSP_CUST <- Add */
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start_app NG ret[%d]\n", ret);
		shgrip_seq_grip_power_off(ctrl);
		ctrl->state = STATE_POWER_OFF;
	} else {
		shgrip_ready_mode = 1;
	}
	shgrip_dev_connect = true;
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
    bootmode = sh_boot_get_bootmode();
	if ((bootmode == SH_BOOT_D) || (bootmode == SH_BOOT_F_F)) {
    	shgrip_tson_mode = SHGRIP_PRM_SENSOR_ALL_ON;
        SHGRIP_INFO("BOOT_D/F");
    } else {
    	shgrip_tson_mode = SHGRIP_PRM_SENSOR_ECOMODE;
        SHGRIP_INFO("BOOT_NORMAL");
    }
	
	return 0;

#ifndef SHGRIP_FACTORY_MODE_ENABLE
fw_dl_err:
	shgrip_sys_free_irq(ctrl);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
request_irq_err:
	if(pin) {

/* SH_BSP_CUST -> Add */
		pin_state = pinctrl_lookup_state(pin, "grip_spi_suspend");
		if(pin_state){
			pinctrl_select_state(pin, pin_state);
		}
/* SH_BSP_CUST <- Add */

		pin_state = pinctrl_lookup_state(pin, "grip_spi_cs_suspend");
		if(pin_state){
			pinctrl_select_state(pin, pin_state);
		}
		pin_state = pinctrl_lookup_state(pin, "grip_int_suspend");
		if(pin_state){
			pinctrl_select_state(pin, pin_state);
		}
		pin_state = pinctrl_lookup_state(pin, "grip_comrdy_suspend");
		if(pin_state){
			pinctrl_select_state(pin, pin_state);
		}
		pin_state = pinctrl_lookup_state(pin, "grip_comreq_suspend");
		if(pin_state){
			pinctrl_select_state(pin, pin_state);
		}
	}
	gpio_free(shgrip_gpio_com_rdy);
gpio_comrdy_err:
	gpio_free(shgrip_gpio_com_req);
gpio_comreq_err:
	gpio_free(shgrip_gpio_reset);
gpio_rst_err:
	gpio_free(shgrip_gpio_int);
gpio_int_err:
	gpio_free(shgrip_gpio_pu);
gpio_pu_err:
	gpio_free(shgrip_gpio_cs);
gpio_cs_err:
	regulator_put(ctrl->reg_p);
	ctrl->reg_p = NULL;
	
	return -1;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_spi_remove                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_dev_spi_remove(struct spi_device *spi)
{
	int ret = 0;
	struct shgrip_drv *ctrl;
	struct pinctrl *pin;
	struct pinctrl_state *pin_state;
	
	mutex_lock(&shgrip_io_lock);
	wake_lock(&shgrip_io_wake_lock);
	
	SHGRIP_DBG("start\n");
	
#ifdef CONFIG_ANDROID_ENGINEERING
	device_remove_file(&spi->dev, &dev_attr_shgrip_dev);
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */

	
	ctrl = &grip_ctrl;
	
	switch (ctrl->state) {
	case STATE_SENSOR_OFF:
		shgrip_seq_grip_power_off(ctrl);
		break;
	case STATE_SENSOR_ON:
		ret = shgrip_seq_grip_sensor_off(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_sensor_off failed\n");
			SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
			shgrip_sys_disable_irq(ctrl);
			shgrip_sys_disable_irq_wake(ctrl);
		}
		shgrip_seq_grip_power_off(ctrl);
		break;
	case STATE_POWER_OFF:
	case STATE_FW_DL:
	default:
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		goto cls_err;
	}
	
	ctrl->state = STATE_POWER_OFF;
	shgrip_ready_mode = 0;
	shgrip_set_sensor_adjust_flg = false;
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
cls_err:
	shgrip_sys_free_irq(ctrl);
	
	pin = devm_pinctrl_get(&spi->dev);
	if(pin) {
		pin_state = pinctrl_lookup_state(pin, "grip_comrdy_suspend");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(COMRDY) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(COMRDY) pin_state: Null\n");	
		}
		
		pin_state = pinctrl_lookup_state(pin, "grip_comreq_suspend");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(COMREQ) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(COMREQ) pin_state: Null\n");	
		}
		pin_state = pinctrl_lookup_state(pin, "grip_int_suspend");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(INT) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(INT) pin_state: Null\n");	
		}
		pin_state = pinctrl_lookup_state(pin, "grip_spi_cs_suspend");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(SPI_CS) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(SPI_CS) pin_state: Null\n");	
		}

/* SH_BSP_CUST -> Add */
		pin_state = pinctrl_lookup_state(pin, "grip_spi_suspend");
		if(pin_state){
			ret = pinctrl_select_state(pin, pin_state);
			if(ret){
				SHGRIP_ERR("GPIO(SPI) pinctrl_select_state: Err\n");
			} else {
				pin_state = NULL;
			}
		} else {
			SHGRIP_ERR("GPIO(SPI) pin_state: Null\n");	
		}
/* SH_BSP_CUST <- Add */

	}
	
	gpio_free(shgrip_gpio_com_rdy);
	gpio_free(shgrip_gpio_com_req);
	gpio_free(shgrip_gpio_reset);
	gpio_free(shgrip_gpio_int);
	gpio_free(shgrip_gpio_pu);
	gpio_free(shgrip_gpio_cs);
	
	shgrip_gpio_com_rdy = -1;
	shgrip_gpio_com_req = -1;
	shgrip_gpio_reset = -1;
	shgrip_gpio_int = -1;
	shgrip_gpio_pu = -1;
	shgrip_gpio_cs = -1;
	
	if(!IS_ERR_OR_NULL(ctrl->reg_p)) {
		regulator_put(ctrl->reg_p);
		ctrl->reg_p = NULL;
	}
	
	wake_unlock(&shgrip_wake_lock);
	wake_unlock(&shgrip_io_wake_lock);
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_init                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_dev_init(struct shgrip_drv *ctrl)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
		
	ret = spi_register_driver(&shgrip_dev_spi_driver);
	if (ret) {
		SHGRIP_ERR("spi_register_driver err\n");
		return ret;
	}
	
	ctrl->state = STATE_POWER_OFF;
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_dev_exit(struct shgrip_drv *ctrl)
{
	SHGRIP_DBG("start\n");
	
	spi_unregister_driver(&shgrip_dev_spi_driver);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_init                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_drv_init(struct shgrip_drv *ctrl)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	ctrl->input = input_allocate_device();
	if (!(ctrl->input)) {
		SHGRIP_ERR("input_allocate_device failed\n");
		return -ENOMEM;
	}
	
	ctrl->input->name         = SHGRIP_NAME;
	ctrl->input->phys         = "shgrip/input0";
	ctrl->input->id.vendor    = 0x0001;
	ctrl->input->id.product   = 0x0001;
	ctrl->input->id.version   = 0x0001;
	
	ctrl->input->evbit[0]     = BIT_MASK(EV_SW);
	
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_00);
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_01);
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_02);
	
	input_set_drvdata(ctrl->input, ctrl);
	
	ret = input_register_device(ctrl->input);
	if (ret) {
		SHGRIP_ERR("input_register_device failed\n");
		input_free_device(ctrl->input);
		return ret;
	}
#ifdef CONFIG_ANDROID_ENGINEERING
	hrtimer_init(&ctrl->shgrip_threshold_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ctrl->shgrip_threshold_timer.function = shgrip_th_timer_callback;
#endif /* CONFIG_ANDROID_ENGINEERING */

	
	ctrl->work_queue = create_singlethread_workqueue("shgrip_workqueue");
	if (ctrl->work_queue) {
		INIT_WORK(&ctrl->work_data, shgrip_work_func);
		INIT_DELAYED_WORK(&ctrl->release_work_data, shgrip_release_work);
#ifdef CONFIG_ANDROID_ENGINEERING
		INIT_WORK(&ctrl->th_work_data, shgrip_th_dump_func);
#endif /* CONFIG_ANDROID_ENGINEERING */
	} else {
		SHGRIP_ERR("create_singlethread_workqueue failed\n");
		ret = -ENOMEM;
		input_unregister_device(ctrl->input);
		return ret;
	}
	
	mutex_init(&shgrip_io_lock);
	spin_lock_init(&(ctrl->lock));
	
    shgrip_qos_cpu_dma_latency.type = PM_QOS_REQ_AFFINE_CORES;
    shgrip_qos_cpu_dma_latency.cpus_affine.bits[0] = 0x0f;  /* little cluster */
	pm_qos_add_request(&shgrip_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	
	wake_lock_init(&shgrip_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_wake_lock");
	wake_lock_init(&shgrip_io_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_io_wake_lock"); // [TN150508_01 ADD]
	wake_lock_init(&shgrip_release_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_release_wake_lock");
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_get_boot_threshold                                                 */
/* ------------------------------------------------------------------------- */
static void shgrip_get_boot_threshold(struct shgrip_drv *ctrl)
{
	int i;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
    
    sh_smem_common = sh_smem_get_common_address();
	
    if (sh_smem_common == NULL) {
    	for (i = 0; i < 4; i++) {
        	shgrip_smem_lth[i] = SHGRIP_THR_VAL_LOW;
        	shgrip_smem_hth[i] = SHGRIP_THR_VAL_LOW;
    	}
    } else {
		for(i = 0; i < 4; i++) {
			shgrip_smem_lth[i] = sh_smem_common->sh_proxgrip_lth[i];
			shgrip_smem_hth[i] = sh_smem_common->sh_proxgrip_hth[i];
		}
    }
    for (i = 0; i < 4; i++) {
        SHGRIP_INFO("boot_threshold lthr[%d]:0x%04X hthr[%d]:0x%04X\n", i, shgrip_smem_lth[i], i, shgrip_smem_hth[i]);
    }
    
    return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_drv_exit(struct shgrip_drv *ctrl)
{
	SHGRIP_DBG("start\n");
	
	if (ctrl->work_queue) {
		flush_workqueue(ctrl->work_queue);
		destroy_workqueue(ctrl->work_queue);
		ctrl->work_queue = NULL;
	}
	
	input_unregister_device(ctrl->input);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_init                                                               */
/* ------------------------------------------------------------------------- */
static int __init shgrip_init(void)
{
	int ret;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = alloc_chrdev_region(&shgrip_dev, 0, 1, SHGRIP_NAME);
	if (!ret) {
		shgrip_major = MAJOR(shgrip_dev);
		shgrip_minor = MINOR(shgrip_dev);
    }else{
		SHGRIP_ERR("alloc_chrdev_region failed\n");
		return ret;
	}
	
	cdev_init(&shgrip_cdev, &shgrip_fops);
	
	shgrip_cdev.owner = THIS_MODULE;
	shgrip_cdev.ops   = &shgrip_fops;
	
	ret = cdev_add(&shgrip_cdev, MKDEV(shgrip_major, 0), 1);
	if (ret) {
		SHGRIP_ERR("cdev_add failed\n");
		goto error_cdev_add;
	}
	
	shgrip_class = class_create(THIS_MODULE, SHGRIP_NAME);
	if (IS_ERR(shgrip_class)) {
		ret = PTR_ERR(shgrip_class);
		SHGRIP_ERR("class_create failed\n");
		goto error_class_create;
	}
	
	shgrip_device = device_create(shgrip_class, NULL, 
									shgrip_dev, &shgrip_cdev, SHGRIP_NAME);
	if (IS_ERR(shgrip_device)) {
		ret = PTR_ERR(shgrip_device);
		SHGRIP_ERR("device_create failed\n");
		goto error_device_create;
	}
	
	ret = shgrip_drv_init(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_drv_init failed\n");
		goto error_drv_init;
	}
	
	ret = shgrip_dev_init(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_dev_init failed\n");
		goto error_dev_init;
	}
	
	shgrip_get_boot_threshold(ctrl);
	
	return 0;
	
error_dev_init:
	shgrip_drv_exit(ctrl);
error_drv_init:
	device_destroy(shgrip_class, MKDEV(shgrip_major, 0));
error_device_create:
	class_destroy(shgrip_class);
error_class_create:
	cdev_del(&shgrip_cdev);
error_cdev_add:
	unregister_chrdev_region(MKDEV(shgrip_major, 0), 1);
	SHGRIP_ERR("FAILED\n");
	return ret;
}
module_init(shgrip_init);

/* ------------------------------------------------------------------------- */
/* shgrip_exit                                                               */
/* ------------------------------------------------------------------------- */
static void __exit shgrip_exit(void)
{
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	shgrip_dev_exit(ctrl);
	
	shgrip_drv_exit(ctrl);
	
	pm_qos_remove_request(&shgrip_qos_cpu_dma_latency);
	
	wake_unlock(&shgrip_io_wake_lock);
	wake_lock_destroy(&shgrip_io_wake_lock);
	
	wake_unlock(&shgrip_wake_lock);
	wake_lock_destroy(&shgrip_wake_lock);
	
	device_destroy(shgrip_class, MKDEV(shgrip_major,0));
	class_destroy(shgrip_class);
	cdev_del(&shgrip_cdev);
	unregister_chrdev_region(MKDEV(shgrip_major, 0), 1);
	
	return;
}
module_exit(shgrip_exit);

/* ------------------------------------------------------------------------- */
MODULE_DESCRIPTION("SHARP GRIP SENSOR DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
