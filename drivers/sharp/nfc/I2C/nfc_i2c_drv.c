/*
*
* NFC Driver ( Toshiba )
*
* Copyright (C) 2015, TOSHIBA CORPORATION
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2.
*
* This program is distributed "as is" WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
/* WAIT TIME for TEST DEBUG */
//#define TEST_WAIT_AFTER_NINT 1000
//#define TEST_WAIT_AFTER_PENDING 1000

/******************************************************************************
 * include
 ******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <asm/uaccess.h>
#include <linux/irq.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include <linux/mutex.h>
#endif

#include <linux/miscdevice.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include "nfc_i2c_drv.h"
#include "nfc_i2c_drv_config.h"
#include "../nfc.h"
#include <sharp/snfc_i2c.h>

#include <linux/wakelock.h>
/******************************************************************************
 *
 ******************************************************************************/
typedef enum nfc_i2c_occure_enum {
    NONE,
    OCCURS,
} nfc_i2c_occure;

typedef enum nfc_i2c_pending_enum {
    READ_ACCEPT = 0,
    READ_REJECT,
    READ_TIME_WAIT,
} nfc_i2c_pending;

typedef struct nfc_i2c_info_tag
{
    struct miscdevice   nfc_i2c_misc_driver;
    struct i2c_client   *nfc_i2c_client;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
/* mutex */
    struct mutex        nfc_i2c_irq_mutex;              /* exclusive control */
    struct mutex        nfc_i2c_mutex;                  /* exclusive control */
    struct mutex        nfc_i2c_drv_mutex;                  /* exclusive control */
    struct mutex        nfc_clf_read_mutex;             /* exclusive control */
    struct mutex        nfc_i2c_pon_irq_mutex;          /* exclusive control */


#endif
    wait_queue_head_t   irq_wait;
    wait_queue_head_t   read_wait;
    wait_queue_head_t   read_pending_wait;
    loff_t              read_offset;
    nfc_i2c_occure      read_request;
    nfc_i2c_occure      ponh_happen;
    int                 close_thread;
    int                 close_pending_thread;
    int                 nint_hold;
    int                 pon_h_request;
    int                 nint_irq;
    unsigned int        nint_gpio;
    nfc_i2c_pending     read_pending_flag;
    struct task_struct* thread_task;
    struct task_struct* pending_task;
    unsigned char       rx_buf[NFC_I2C_READ_BUFFER_SIZE];
    unsigned int        rx_buf_w, rx_buf_r;
} nfc_i2c_info;


/******************************************************************************
 * data
 ******************************************************************************/
static atomic_t         ref_open;
static atomic_t         ref_read;
static int              i2c_access_flg;
static nfc_i2c_info*    g_nfc_i2c_info = NULL;
static struct wake_lock g_wake_lock;
/******************************************************************************
 * function
 ******************************************************************************/
/*** I2C Device ***/
static int nfc_i2c_drv_probe        ( struct i2c_client*,
                                      const struct i2c_device_id* );

static int nfc_i2c_drv_remove       ( struct i2c_client* );

/*** I2C Device Operation ***/
static ssize_t nfc_i2c_read        ( struct file* ,
                                     char* ,
                                     size_t ,
                                     loff_t*  );

static ssize_t nfc_i2c_write       ( struct file* ,
                                     const char* ,
                                     size_t ,
                                     loff_t*  );

static unsigned int nfc_i2c_poll   ( struct file *,
                                     poll_table* );

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int nfc_i2c_ioctl           ( struct inode* ,
                                     struct file* ,
                                     unsigned int ,
                                     unsigned long  );
#else  /* LINUX_VERSION_CODE */
static long nfc_i2c_ioctl          ( struct file *,
                                     unsigned int ,
                                     unsigned long  );
#endif /* LINUX_VERSION_CODE */
static int nfc_i2c_open            ( struct inode* ,
                                     struct file*  );

static int nfc_i2c_close           ( struct inode* ,
                                     struct file*  );

/*** Other Operation ***/
static irqreturn_t      nfc_i2c_irq_thread_handler  ( int,
                                                      void* );
static int              nfc_i2c_init_thread         ( nfc_i2c_info* );

static int              nfc_i2c_read_thread         ( void* );

static int              nfc_i2c_read_pending_thread ( void* );

/******************************************************************************
 *
 ******************************************************************************/

#ifdef CONFIG_PM
static int nfc_i2c_suspend(struct device *dev)
{
    INFO_PRINT("START");
    INFO_PRINT("END");
    return 0;
}

static int nfc_i2c_resume(struct device *dev)
{
    INFO_PRINT("START");
    INFO_PRINT("END");
    return 0;
}

static SIMPLE_DEV_PM_OPS(nfc_i2c_pm_ops, nfc_i2c_suspend, nfc_i2c_resume);
#endif

static const struct i2c_device_id nfc_i2c_drv_id[] = {
                                    {NFC_I2C_CONFIG_DRIVER_NAME, 0},
                                    {}
};


MODULE_DEVICE_TABLE ( i2c, nfc_i2c_drv_id );

static const struct of_device_id nfc_i2c_match_table[] = {
    { .compatible = NFC_I2C_CONFIG_DRIVER_NAME, },
    {}
};

static struct i2c_driver nfc_i2c_driver = {
                            .driver = { .name = NFC_I2C_CONFIG_DRIVER_NAME,
                                        .owner = THIS_MODULE,
#ifdef CONFIG_PM
                                        .pm = &nfc_i2c_pm_ops,
#endif //CONFIG_PM
                                        .of_match_table = nfc_i2c_match_table,
                                      },
                            .probe      = nfc_i2c_drv_probe,
                            .remove     = nfc_i2c_drv_remove,
                            .class      = I2C_CLASS_HWMON,
                            .id_table   = nfc_i2c_drv_id,
                        };

/* entry point */
static struct file_operations nfc_i2c_drv_fops = {
                                .owner   = THIS_MODULE,
                                .read    = nfc_i2c_read,
                                .write   = nfc_i2c_write,
                                .poll    = nfc_i2c_poll,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
                                .ioctl   = nfc_i2c_ioctl,
#else  /* LINUX_VERSION_CODE */
                                .unlocked_ioctl = nfc_i2c_ioctl,
#ifdef CONFIG_COMPAT
                                .compat_ioctl   = nfc_i2c_ioctl,
#endif //CONFIG_COMPAT
#endif /* LINUX_VERSION_CODE */
                                .open    = nfc_i2c_open,
                                .release = nfc_i2c_close
                            };
/******************************************************************************
 * define area
 ******************************************************************************/
#define MAX_CLF_PACKET_LENGTH  258
/******************************************************************************
 * code area
 ******************************************************************************/
#ifdef DEBUG_NFC_DRV
void dumpData(const char *data, int len, const char *log)
{
	int i;
	const char *p = data;
	char wk_buf[5];
	char out_buf[80];

	INFO_PRINT("dump start(len:%d)",len);
	memset(out_buf, 0x00, sizeof(out_buf));
	strcat(out_buf,log);
	for (i = 1; i <= len; i++) {
		memset(wk_buf, 0x00, sizeof(wk_buf));
		sprintf(wk_buf,"%02X ",*p++);
		strcat(out_buf,wk_buf);
		if ((i % 16 == 0) && (i != len)) {
			printk(KERN_INFO "%s\n", out_buf);
			memset(out_buf, 0x00, sizeof(out_buf));
			strcat(out_buf,log);
		}
	}
	printk(KERN_INFO "%s\n", out_buf);
	INFO_PRINT( "dump end");

	return;
}
#endif
 /******************************************************************************
 * Kernel Init
 ******************************************************************************/
/******************************************************************************
 *    function:   nfc_i2c_drv_start
 *    brief   :   initialization control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
 static int __init nfc_i2c_drv_start ( void )
 {
    int ret;

    INFO_PRINT("Start.");

    ret = i2c_add_driver ( &nfc_i2c_driver );
    if ( ret < 0 )
    {
        ERROR_PRINT("i2c_add_driver :  Failed[%d]", ret );
        return ( ret );
    }
    INFO_PRINT("END.");

    return ( 0 );
}


/******************************************************************************
 *    function:   nfc_i2c_drv_end
 *    brief   :   exit control of a driver
 *    date    :
 *    author  :
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
static void __exit  nfc_i2c_drv_end        ( void )
{
    INFO_PRINT("Start.");

    i2c_del_driver ( &nfc_i2c_driver );

    INFO_PRINT("End.");

    return;
}


/******************************************************************************
 * I2C Driver
 ******************************************************************************/
/******************************************************************************
 *    function:   nfc_i2c_drv_probe
 *    brief   :   probe control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   client
 *            :   devid
 *    output  :   none
 ******************************************************************************/
static int nfc_i2c_drv_probe   ( struct i2c_client *client,
                                 const struct i2c_device_id *devid )
{
    nfc_i2c_info  *info = NULL;
    int     ret = 0;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;

    INFO_PRINT("Probe Start.");

    if ( NULL == client )
    {
        ERROR_PRINT("client: Failed[NULL]." );
        return ( -EINVAL );
    }
    info = kzalloc ( sizeof ( nfc_i2c_info ), GFP_KERNEL );
    if ( NULL == info )
    {
        ERROR_PRINT("kzalloc(info): Failed[NULL]." );
        return ( -EINVAL );
    }
    INFO_PRINT("Client Info" );
    INFO_PRINT("flags         = %d.", client->flags );
    INFO_PRINT("addr          = %x.", client->addr );
    INFO_PRINT("name          = %s.", client->name );
    INFO_PRINT("adapter       = %p.", client->adapter );
    INFO_PRINT("irq           = %d.", client->irq );

    info->nfc_i2c_misc_driver.minor  = MISC_DYNAMIC_MINOR;
    info->nfc_i2c_misc_driver.name   = NFC_I2C_CONFIG_DRIVER_NAME;
    info->nfc_i2c_misc_driver.fops   = &nfc_i2c_drv_fops;
    info->nfc_i2c_misc_driver.parent = &client->dev;

    info->nfc_i2c_client = client;
    info->read_request   = NONE;
    info->nint_hold      = 0;
    info->pon_h_request  = 0;
    info->rx_buf_w       = 0;
    info->rx_buf_r       = 0;
    info->thread_task    = NULL;
    info->pending_task   = NULL;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    mutex_init ( &info->nfc_i2c_irq_mutex );
    mutex_init ( &info->nfc_i2c_mutex );
    mutex_init ( &info->nfc_i2c_drv_mutex );
    mutex_init ( &info->nfc_clf_read_mutex );
    mutex_init ( &info->nfc_i2c_pon_irq_mutex );
#endif
    init_waitqueue_head ( &info->irq_wait );
    init_waitqueue_head ( &info->read_wait );
    init_waitqueue_head ( &info->read_pending_wait );
    info->read_pending_flag = READ_REJECT;

    i2c_set_clientdata ( client, info );

    ret = misc_register ( &info->nfc_i2c_misc_driver );
    if ( 0 > ret )
    {
        ERROR_PRINT("misc_register: Failed[NULL]." );
        kfree ( info );
        return ( -EINVAL );
    }

    info->nint_gpio = of_get_named_gpio(client->dev.of_node,"sharp,nfc-nint-gpio", 0);

    /*** GPIO Interrupt Setting */
    ret = gpio_request ( info->nint_gpio, NFC_I2C_CONFIG_DRIVER_NAME );
    if ( ret )
    {
        ERROR_PRINT("gpio_request: Failed[%d].", ret );
        goto error_ret2;
    }

    info->nint_irq = gpio_to_irq ( info->nint_gpio );
    if ( 0 > info->nint_irq )
    {
        ERROR_PRINT("gpio_to_irq: Failed[%d].", info->nint_irq );
        pin = devm_pinctrl_get(&client->dev);
        pin_state = pinctrl_lookup_state(pin, "nfc_nint_suspend");
        ret = pinctrl_select_state(pin, pin_state);
        goto error_ret1;
    }
    INFO_PRINT("info->nint_irq  = %d.", info->nint_irq  );

    g_nfc_i2c_info = info;
    atomic_set(&ref_open, 0);
    atomic_set(&ref_read, 0);

    ret = nfc_i2c_init_thread ( info );
    if ( 0 > ret ){
        ERROR_PRINT("nfc_i2c_init_thread :  Failed[%d]", ret );
        goto error_ret1;
    }

    wake_lock_init(&g_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

    INFO_PRINT("Probe End.");

    return ( 0 );

error_ret1:
    gpio_free ( info->nint_gpio );
error_ret2:
    kfree ( info );
    return ( -EINVAL );

}

/******************************************************************************
 *    function:   nfc_i2c_drv_remove
 *    brief   :   remove control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   client
 *    output  :   none
 ******************************************************************************/
static int  nfc_i2c_drv_remove  ( struct i2c_client *client )
{
    nfc_i2c_info  *info = NULL;
    int ret = 0;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;

    INFO_PRINT("Remove Start.");

    if ( NULL == client )
    {
        ERROR_PRINT("client: Failed[NULL]." );
        return ( -EINVAL );
    }
    info = i2c_get_clientdata ( client );

    info->close_pending_thread = 1;
    info->close_thread         = 1;
    wake_up_interruptible( &info->read_pending_wait );
    wake_up_interruptible( &info->irq_wait );
    free_irq ( info->nint_irq , info );
    pin = devm_pinctrl_get(&client->dev);
    pin_state = pinctrl_lookup_state(pin, "nfc_nint_suspend");
    ret = pinctrl_select_state(pin, pin_state);
    gpio_free ( info->nint_gpio );

    if (wake_lock_active(&g_wake_lock)) {
        wake_unlock(&g_wake_lock);
    }
    wake_lock_destroy(&g_wake_lock);

    /* unregister is MISC Driver */
    misc_deregister ( &info->nfc_i2c_misc_driver );

    kfree ( info );

    INFO_PRINT("Remove End.");

    return ( 0 );
}

/******************************************************************************
 * I2C Driver fops
 ******************************************************************************/
/******************************************************************************
 *    function:   nfc_i2c_open
 *    brief   :   open control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   File
 *    output  :   none
 ******************************************************************************/
static int  nfc_i2c_open   ( struct inode* Inode,
                             struct file*  File )
{
    nfc_i2c_info        *info = NULL;

    INFO_PRINT("I2C OPEN Start.");

    if ( NULL == File )
    {
        ERROR_PRINT("Argument :  Failed[NULL]");
        return ( -EINVAL );
    }

    if (atomic_read(&ref_open))
    {
        ERROR_PRINT("Double Open[%d] : Warning", atomic_read(&ref_open));
        return ( -EBUSY );
    }

    info = (nfc_i2c_info*)container_of ( File->private_data,
                                         nfc_i2c_info,
                                         nfc_i2c_misc_driver );
    atomic_inc(&ref_open);

    mutex_lock ( &info->nfc_i2c_mutex );
    File->f_pos = info->read_offset;
    info->nint_hold = 0;
    info->pon_h_request = 0;
    mutex_lock(&info->nfc_clf_read_mutex);
    info->rx_buf_w = 0;
    info->rx_buf_r = 0;
    mutex_unlock(&info->nfc_clf_read_mutex );


    mutex_unlock ( &info->nfc_i2c_mutex );
    i2c_access_flg = NFC_I2C_ACCESS_NONE;

    /* START PON = H */

   INFO_PRINT("PON is HIGH.");
   mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
   info->ponh_happen = OCCURS;
   info->read_pending_flag = READ_TIME_WAIT;
   mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );

   INFO_PRINT("START read_pending_timer");
   wake_up_interruptible ( &info->read_pending_wait );

    if ( 0 == gpio_get_value( info->nint_gpio ))
    {
        INFO_PRINT("Already NINT occure.");
        info->nint_hold = 1;
    }

    INFO_PRINT("ref_open_count[%d]", atomic_read(&ref_open));
    INFO_PRINT("I2C OPEN End.");

    return ( 0 );
}

/******************************************************************************
 *    function:   nfc_i2c_close
 *    brief   :   close control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   File
 *    output  :   none
 ******************************************************************************/
static int  nfc_i2c_close  ( struct inode* Inode,
                             struct file*  File )
{
    nfc_i2c_info* info = NULL;

    INFO_PRINT("CLOSE Start.");

    if (atomic_read(&ref_open)){
        if ( NULL == File ){
            ERROR_PRINT("Argument :  Failed[NULL]");
            return ( -EINVAL );
        }
        info = (nfc_i2c_info*)container_of ( File->private_data,
                                             nfc_i2c_info,
                                             nfc_i2c_misc_driver );

        mutex_lock(&info->nfc_clf_read_mutex );
        atomic_dec(&ref_open);
        mutex_unlock(&info->nfc_clf_read_mutex );

        /* END PON = L */
        info->read_pending_flag = READ_REJECT;

    } else {
        INFO_PRINT("Already Closed.");

    }
    INFO_PRINT("Close End.");

    return ( 0 );


}

/******************************************************************************
 *    function:   nfc_i2c_read
 *    brief   :   read control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   File
 *            :   Buffer
 *            :   Count
 *            :   OffsetPosition
 *    output  :   none
 ******************************************************************************/
static ssize_t nfc_i2c_read ( struct file*     File,
                              char*            Buffer,
                              size_t           Count,
                              loff_t*          OffsetPosition )
{
    int ret = 0;
    ssize_t len = 0;
    ssize_t temp_len = 0;
    nfc_i2c_info *info = NULL;
    unsigned int local_rx_buf_w;
    unsigned char tmp_buffer[MAX_CLF_PACKET_LENGTH];

    INFO_PRINT("Start.");

    if (!atomic_read(&ref_open)) {
        ERROR_PRINT("No Open Device[%d].", atomic_read(&ref_open));
        return ( -EBADF );
    }

    if (atomic_read(&ref_read))
    {
        ERROR_PRINT("multiple read[%d].", atomic_read(&ref_read));
        return ( -EBUSY );
    }

    if (( NULL == File )    ||
        ( NULL == Buffer )  ||
        ( 0 == Count ))
    {
        ERROR_PRINT("Argument :  Failed[NULL]");
        return ( -EINVAL );
    }
    atomic_inc(&ref_read);

    info = (nfc_i2c_info*)container_of ( File->private_data,
                                         nfc_i2c_info,
                                         nfc_i2c_misc_driver );

    ret = wait_event_interruptible_timeout (info->read_wait, ((info->rx_buf_r != info->rx_buf_w) || (!atomic_read(&ref_open))), 2*HZ);

    local_rx_buf_w = info->rx_buf_w;

    if (info->rx_buf_r <= local_rx_buf_w) {
        len = local_rx_buf_w - info->rx_buf_r;
    } else {
        len = local_rx_buf_w + (sizeof(info->rx_buf) - info->rx_buf_r);
    }

    if(len > Count){
        len = Count;
    }

    if ( info->rx_buf_r + len > sizeof(info->rx_buf)){
        temp_len = sizeof(info->rx_buf) - info->rx_buf_r;
        memcpy ( tmp_buffer, &info->rx_buf[info->rx_buf_r], temp_len);
        memcpy ( &tmp_buffer[temp_len], info->rx_buf, len - temp_len);
        info->rx_buf_r = len - temp_len;
    } else {
        memcpy ( tmp_buffer, &info->rx_buf[info->rx_buf_r], len);
        info->rx_buf_r += len; 
        if (info->rx_buf_r == sizeof(info->rx_buf)){
            info->rx_buf_r = 0;
        }
    }

#ifdef DEBUG_NFC_DRV
    dumpData(tmp_buffer, (int)len, "[NFC][i2c_read  ] ");
#endif

    mutex_lock ( &info->nfc_i2c_mutex );
    ret = copy_to_user(Buffer, tmp_buffer, len);
    mutex_unlock ( &info->nfc_i2c_mutex );

    *OffsetPosition = len;

    atomic_dec(&ref_read);
    INFO_PRINT("End.");

    return (len);
}

/******************************************************************************
 *    function:   nfc_i2c_write
 *    brief   :   write control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0        normal exit
 *            :   -1        error exit
 *    input   :   File
 *            :   Buffer    [slave][reg.addr][data-0][data-1]...[data-(n-1)]
 *            :   Count     (>=3) n+2
 *            :   OffsetPosition
 *    output  :   none
 ******************************************************************************/
static ssize_t nfc_i2c_write ( struct file*    File,
                               const char*     Buffer,
                               size_t          Count,
                               loff_t*         OffsetPosition )
{
    int ret = 0;
    nfc_i2c_info *info = NULL;
    unsigned char tmp_buffer[512];

    INFO_PRINT("Start.");

    if (!atomic_read(&ref_open))
    {
        ERROR_PRINT("No Open Device[%d].", atomic_read(&ref_open));
        return ( -EINVAL );
    }

    if (( NULL == File )    ||
        ( NULL == Buffer )  ||
        ( 0 == Count )      ||
        ( NFC_I2C_BUF_SIZE < Count ))
    {
        ERROR_PRINT("Argument :  Failed[NULL]");
        return ( -EINVAL );
    }
    info = (nfc_i2c_info*)container_of ( File->private_data,
                                         nfc_i2c_info,
                                         nfc_i2c_misc_driver );

    ret = copy_from_user ( tmp_buffer,
                           Buffer,
                           Count );
    if ( 0 != ret )
    {
        ERROR_PRINT("copy_from_user :  Failed[%d]", ret );
        return ( -ENOMEM );
    }

    while ( NFC_I2C_ACCESS_NONE != i2c_access_flg )
    {
        msleep ( NFC_I2C_ACCESS_CHECK_WAIT );
        if (!atomic_read(&ref_open))
        {
            INFO_PRINT("Write Cancel.");
            return ( -EINVAL );
        }
    }
    i2c_access_flg |= NFC_I2C_ACCESS_WRITE;

#ifdef DEBUG_NFC_DRV
    dumpData(tmp_buffer, (int)Count, "[NFC][i2c_write ] ");
#endif

    mutex_lock ( &info->nfc_i2c_drv_mutex );

    ret = i2c_master_send ( info->nfc_i2c_client,
                            tmp_buffer,
                            Count );

    mutex_unlock ( &info->nfc_i2c_drv_mutex );

    i2c_access_flg &= ~NFC_I2C_ACCESS_WRITE;

    if ( 0 > ret )
    {
        ERROR_PRINT("i2c_master_send :  Failed[%d]", ret );
        if ( -EREMOTEIO == ret )
        {
            usleep_range ( NFC_I2C_ACCESS_TO_WAIT_MIN,
                           NFC_I2C_ACCESS_TO_WAIT_MAX );
        }
        return ( ret );
    }

    if ( Count != ret )
    {
        ERROR_PRINT("i2c_master_send : Failed -> Remaining Packet[%d]", (int)(Count - ret));
        return ( -EREMOTEIO );
    }

    INFO_PRINT("End.");

    return ( ret );
}


/******************************************************************************
 *    function:   nfc_i2c_poll
 *    brief   :   poll control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   File
 *            :   poll_tbl
 *    output  :   none
 ******************************************************************************/
static unsigned int nfc_i2c_poll ( struct file*   File,
                                   poll_table*    poll_tbl )
{
    int ret = 0;
    nfc_i2c_info    *info = NULL;

    INFO_PRINT("Start.");

    if (!atomic_read(&ref_open))
    {
        ERROR_PRINT("No Open Device[%d].", atomic_read(&ref_open));
        return ( POLLNVAL );
    }
    if ( NULL == File )
    {
        ERROR_PRINT("Argument :  Failed[NULL]");
        return ( POLLERR );
    }

    info = (nfc_i2c_info*)container_of ( File->private_data,
                                         nfc_i2c_info,
                                         nfc_i2c_misc_driver );

    poll_wait ( File,
                &info->read_wait,
                poll_tbl );

    if (( info->rx_buf_r != info->rx_buf_w )|| info->pon_h_request ){
        info->pon_h_request = 0;
        ret = POLLIN | POLLRDNORM;
    }
    INFO_PRINT("End[%d].", ret );

    return ( ret );
}
/******************************************************************************
 *    function:   nfc_i2c_ioctl
 *    brief   :   ioctl control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   Inode
 *            :   File
 *            :   uCommand
 *            :   uArgument
 *    output  :   none
 ******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int nfc_i2c_ioctl ( struct inode*   Inode,
                           struct file*    File,
                           unsigned int    uCommand,
                           unsigned long   uArgument )
#else  /* LINUX_VERSION_CODE */
static long nfc_i2c_ioctl ( struct file*   File,
                            unsigned int   uCommand,
                            unsigned long  uArgument )
#endif /* LINUX_VERSION_CODE */
{
    int ret = 0;
    nfc_i2c_info    *info = NULL;

    INFO_PRINT("Start.");

    if (!atomic_read(&ref_open))
    {
        ERROR_PRINT("No Open Device[%d].", atomic_read(&ref_open));
        return ( -EINVAL );
    }

    info = (nfc_i2c_info*)container_of ( File->private_data,
                                         nfc_i2c_info,
                                         nfc_i2c_misc_driver );

    switch ( uCommand )
    {
        case NFC_I2C_IOCTL_SET_PON_L:
            if ( info->read_pending_flag == READ_REJECT ){
                INFO_PRINT("Now PON is LOW.");
            } else {
                INFO_PRINT("PON is LOW.");
                mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
                info->read_pending_flag = READ_REJECT;
                mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );
                INFO_PRINT("SET read_pending_flag");

                if( info->nint_hold ) {
                    info->pon_h_request = 1;
                    INFO_PRINT("Remain NINT detect. (read_wait)");
                    wake_up_interruptible ( &info->read_wait );

                    /* nint_hold is not setting 0. This is PON_H-request notify */
                }
            }
            break;

        case NFC_I2C_IOCTL_SET_PON_H:
            if ( info->read_pending_flag == READ_ACCEPT ||( info->read_pending_flag == READ_TIME_WAIT )){
                INFO_PRINT("Now PON is HIGH.");
            } else {
                INFO_PRINT("PON is HIGH.");
                mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
                info->ponh_happen = OCCURS;
                info->read_pending_flag = READ_TIME_WAIT;
                mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );

                INFO_PRINT("START read_pending_timer");
                wake_up_interruptible ( &info->read_pending_wait );

                if( info->rx_buf_r != info->rx_buf_w ) {
                    info->nint_hold = 0;
                    INFO_PRINT("Alredy read complete. (read_wait)");
                    wake_up_interruptible ( &info->read_wait );
                }
            }
            break;
        default:
            ERROR_PRINT("Nod kind cmd:  Failed[%d]", uCommand );
            ret = -ENOIOCTLCMD;
            break;
    }

    INFO_PRINT("End.");

    return ( ret );
}


/******************************************************************************
 * Other Module
 ******************************************************************************/
/******************************************************************************
 *    function:   nfc_i2c_init_thread
 *    brief   :   i2c Read Thread
 *    date    :
 *    author  :
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
static int nfc_i2c_init_thread ( nfc_i2c_info *info )
{
    int     ret = 0;
    struct i2c_client   *client = NULL;

#ifdef NFC_I2C_SCHED_CHG
    struct sched_param param = { .sched_priority = NFC_I2C_KTHD_PRI };
#endif

    INFO_PRINT("Start.");
    info->close_thread         = 0;
    info->close_pending_thread = 0;
    /*** Thread generation and run ***/
    info->thread_task = kthread_run ( nfc_i2c_read_thread,
                                      (void*)info,
                                      "NFC_I2C_READ_THREAD" );

    if ( IS_ERR ( info->thread_task ))
    {
        ERROR_PRINT("kthread_run :  Failed");
        return ( -1 );
    }

    info->pending_task = kthread_run ( nfc_i2c_read_pending_thread,
                                      (void*)info,
                                      "NFC_I2C_PENDING_TREAD" );

    if ( IS_ERR ( info->pending_task ))
    {
        info->close_thread = 1;
        ERROR_PRINT("kthread_run :  Failed");
        wake_up_interruptible( &info->irq_wait );
        return ( -1 );
    }

    client = info->nfc_i2c_client;

    /* Set Interrupt Handler */
    /* Get NINT */
    ret = request_threaded_irq ( info->nint_irq,
                                 NULL,
                                 nfc_i2c_irq_thread_handler,
                                 (IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
                                 NFC_I2C_CONFIG_DRIVER_NAME,
                                 info );
    if ( 0 > ret ){
        info->close_thread         = 1;
        info->close_pending_thread = 1;
        ERROR_PRINT("Irq Request :  Failed");
        wake_up_interruptible( &info->irq_wait );
        wake_up_interruptible( &info->read_pending_wait );
        return ( -EINVAL );
    }

    enable_irq_wake(info->nint_irq);

    INFO_PRINT("End.");

    return ( 0 );

}


/******************************************************************************
 *    function:   nfc_i2c_irq_thread_handler
 *    brief   :   interrpu control of a driver
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :   irq
 *            :   dev_id
 *    output  :   none
 ******************************************************************************/
irqreturn_t     nfc_i2c_irq_thread_handler   ( int   irq,
                                               void* dev_id )
{
    nfc_i2c_info  *info = NULL;

    INFO_PRINT("Start.");

    if ( NULL == dev_id )
    {
        ERROR_PRINT("dev_id :  Failed[NULL]");
        return ( IRQ_NONE );
    }
    if (!atomic_read(&ref_open))
    {
        ERROR_PRINT("No Open Device[%d].", atomic_read(&ref_open));
        return ( IRQ_HANDLED );
    }
    info = (nfc_i2c_info*)dev_id;
    if ( OCCURS == info->read_request )
    {
        ERROR_PRINT("NINT : read_request Double.");
        return ( IRQ_HANDLED );
    }
    if ( info->read_pending_flag == READ_ACCEPT ){
        /* PON = H */
        mutex_lock ( &info->nfc_i2c_irq_mutex );
        info->read_request = OCCURS;
        mutex_unlock ( &info->nfc_i2c_irq_mutex );

        INFO_PRINT("GET NINT:RESUME read_thread. (irq_wait)");
        wake_up_interruptible ( &info->irq_wait );
    } else if ( info->read_pending_flag == READ_REJECT ){
        /* PON = L */
        info->nint_hold = 1;
        info->pon_h_request = 1;

        INFO_PRINT("GET NINT:Wait until PON H (read_wait) ");
        wake_up_interruptible ( &info->read_wait );
        //wakelock
        if(wake_lock_active(&g_wake_lock))
        {
            wake_unlock(&g_wake_lock);
        }
        INFO_PRINT("Set wake_lock_timeout for 800 msec. !!!");
        wake_lock_timeout(&g_wake_lock,((HZ*4)/5));

    } else {    /* info->read_pending_flag == READ_TIME_WAIT */
        /* PON = H & Now Read Pending*/
        info->nint_hold = 1;
        INFO_PRINT("GET NINT:NOW READ PENDING");
    }
    INFO_PRINT("End.");

    return ( IRQ_HANDLED );
}


/******************************************************************************
 *    function:   nfc_i2c_read_thread
 *    brief   :   i2c Read Thread
 *    date    :
 *    author  :
 *
 *    return  :   none
 *    input   :   none
 *    output  :   none
 ******************************************************************************/
static int nfc_i2c_read_thread ( void  *arg )
{
    unsigned char buffer[MAX_CLF_PACKET_LENGTH];
    int ret       = 0;
    int len       = 0;
    int temp_ret  = 0;
    unsigned int  local_rx_buf_r;

    nfc_i2c_info*   info = NULL;

    INFO_PRINT("Read Thread Start.");
    if ( NULL == arg )
    {
        ERROR_PRINT("arg :  Failed[NULL]");
        return ( 0 );
    }

    info = (nfc_i2c_info*)arg;

    mutex_lock ( &info->nfc_i2c_irq_mutex );
    info->read_request = NONE;
    mutex_unlock ( &info->nfc_i2c_irq_mutex );
    while ( !kthread_should_stop() && info->close_thread == 0 ){
        /* Wait Interrupt Handeler */
        ret = wait_event_interruptible ( info->irq_wait,( info->read_request == OCCURS || info->close_thread ));
        mutex_lock( &info->nfc_clf_read_mutex );

        if ( 0 != ret ){
            ERROR_PRINT("Interrupt Wait :  Failed[%d]", ret );
            mutex_unlock( &info->nfc_clf_read_mutex );
            continue;
        }
        if ( info->close_thread ){
            INFO_PRINT("Free Read Thread.");
            mutex_unlock( &info->nfc_clf_read_mutex );
            break;
        }
        INFO_PRINT("wait_event_interruptible : Occur.");

        mutex_lock ( &info->nfc_i2c_irq_mutex );
        info->read_request = NONE;
        mutex_unlock ( &info->nfc_i2c_irq_mutex );

        while ( NFC_I2C_ACCESS_NONE != i2c_access_flg ){
            msleep ( NFC_I2C_ACCESS_CHECK_WAIT );
            INFO_PRINT("wait_I2C_write.");
        }
        i2c_access_flg |= NFC_I2C_ACCESS_READ;
        if ( info->close_thread ){
            INFO_PRINT("Free Read Thread.");
            mutex_unlock( &info->nfc_clf_read_mutex );
            break;
        }

        /* Get Packet length */
        if ( info->read_pending_flag == READ_ACCEPT ){
            mutex_lock ( &info->nfc_i2c_drv_mutex );
            ret = i2c_master_recv (info->nfc_i2c_client, buffer, 2 );
            mutex_unlock( &info->nfc_i2c_drv_mutex ) ;
#ifdef DEBUG_NFC_DRV
            dumpData(buffer, ret,"[NFC][i2c_recive] ");
#endif

        } else {
            INFO_PRINT("Not accept.");
            ret = 0;
            len = 0;
        }
        if ( info->close_thread ){
            INFO_PRINT("Free Read Thread.");
            mutex_unlock( &info->nfc_clf_read_mutex );
            break;
        }

        if ( ret == 2 ) {
            len =( buffer[0] << 8 ) | buffer[1];
        }
        if ( len > 0 ){
            while ( len ) {
                if ( len > MAX_CLF_PACKET_LENGTH ){
                    ret = MAX_CLF_PACKET_LENGTH;
                } else {
                    ret = len;
                }
                if ( info->read_pending_flag == READ_ACCEPT ){
                    mutex_lock ( &info->nfc_i2c_drv_mutex );
                    ret = i2c_master_recv (info->nfc_i2c_client, buffer, ret);
                    mutex_unlock ( &info->nfc_i2c_drv_mutex );
                } else {
                    break;
                }
                if (0 < ret) {
#ifdef DEBUG_NFC_DRV
                    dumpData(buffer, ret,"[NFC][i2c_recive] ");
#endif
                    if (info->rx_buf_w + ret >= sizeof(info->rx_buf)){
                        INFO_PRINT("reach Buffer END.");
                        temp_ret = sizeof(info->rx_buf)- info->rx_buf_w;

                        local_rx_buf_r = info->rx_buf_r;

                        while((info->rx_buf_w < local_rx_buf_r)||(local_rx_buf_r <= ret - temp_ret )){
                            INFO_PRINT("Buffer FULL Wait START. (read_wait)");
                            wake_up_interruptible ( &info->read_wait );

                            usleep_range(NFC_I2C_ACCESS_BE_WAIT_MIN, NFC_I2C_ACCESS_BE_WAIT_MAX);
                            local_rx_buf_r = info->rx_buf_r;
                            if ( info->read_pending_flag != READ_ACCEPT){
                                break;
                            }
                        }
                        memcpy(&info->rx_buf[info->rx_buf_w], buffer, temp_ret);
                        if ( ret - temp_ret ){
                            memcpy(info->rx_buf, &buffer[temp_ret], ret - temp_ret);
                        }
                        info->rx_buf_w = ret - temp_ret;
                    } else {
                        local_rx_buf_r = info->rx_buf_r;
                        while (((info->rx_buf_w + ret ) >= local_rx_buf_r)&&(info->rx_buf_w < local_rx_buf_r)) {
                            INFO_PRINT("Buffer FULL Wait START. (read_wait)");
                            wake_up_interruptible ( &info->read_wait );

                            usleep_range(NFC_I2C_ACCESS_BE_WAIT_MIN, NFC_I2C_ACCESS_BE_WAIT_MAX);
                            local_rx_buf_r = info->rx_buf_r;
                            if ( info->read_pending_flag != READ_ACCEPT ){
                                break;
                            }
                        }
                        memcpy (&info->rx_buf[info->rx_buf_w], buffer, ret);
                        info->rx_buf_w += ret;
                    } 
                    len -= ret;
                } else {
                    ERROR_PRINT("Received DATA number :  Failed[%d]", ret );
                    break;
                }
            }
            if ( info->read_pending_flag == READ_ACCEPT){
                if( info->rx_buf_r != info->rx_buf_w ) {
                    info->nint_hold = 0;
#ifdef TEST_WAIT_AFTER_NINT
                    INFO_PRINT("100 mSec Wait START.");
                    msleep ( 100 );
#endif /* TEST_WAIT_AFTER_NINT */
                    INFO_PRINT("Wake up read complete. (read_wait)");
                    wake_up_interruptible ( &info->read_wait );
                }
            }
        }
        i2c_access_flg &= ~NFC_I2C_ACCESS_READ;
        mutex_unlock( &info->nfc_clf_read_mutex );
    }
    info->nint_hold = 0;

    INFO_PRINT("Read Thread End.");
    return ( 0 );
}
static int nfc_i2c_read_pending_thread ( void *arg )
{
    int ret              = 0;
    nfc_i2c_info*   info = NULL;

    INFO_PRINT("Pending Thread Start.");

    if ( NULL == arg )
    {
        ERROR_PRINT("arg :  Failed[NULL]");
        return ( 0 );
    }

    info = (nfc_i2c_info*)arg;

    mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
    info->ponh_happen = NONE;
    mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );

    while ( !kthread_should_stop()&& info->close_pending_thread == 0 )
    {

        /* Wait Interrupt Handeler */
        ret = wait_event_interruptible ( info->read_pending_wait,( info->ponh_happen == OCCURS || info->close_pending_thread ));

        if ( 0 != ret ){
            ERROR_PRINT("Interrupt Wait :  Failed[%d]", ret );
            continue;
        }
        if ( info->close_pending_thread ){
            INFO_PRINT("Free Pending Thread.");
            break;
        }
        INFO_PRINT("Pending_event_interruptible : Occur.");

        do {
            if ( info->ponh_happen == OCCURS ){
                mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
                info->ponh_happen = NONE;
                mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );
            }
            usleep_range ( NFC_I2C_ACCESS_PENDING_WAIT, NFC_I2C_ACCESS_PENDING_WAIT );
        } while ( info->ponh_happen == OCCURS && info->read_pending_flag == READ_TIME_WAIT && info->close_pending_thread == 0);

#ifdef TEST_WAIT_AFTER_PENDING
        INFO_PRINT("100 mSec Wait START.");
        msleep ( 100 );
#endif /* TEST_WAIT_AFTER_PENDING */

        if ( info->read_pending_flag == READ_TIME_WAIT ){
            mutex_lock ( &info->nfc_i2c_pon_irq_mutex );
            info->read_pending_flag = READ_ACCEPT;
            mutex_unlock ( &info->nfc_i2c_pon_irq_mutex );

            INFO_PRINT("Pending timer finished.");
            if ( info->nint_hold ){
               mutex_lock ( &info->nfc_i2c_irq_mutex );
               info->read_request = OCCURS;
               mutex_unlock ( &info->nfc_i2c_irq_mutex );

               INFO_PRINT("T.O.:RESUME read_thread (irq_wait)");
               wake_up_interruptible ( &info->irq_wait );
            }
        }
        else {
            INFO_PRINT("Pending timer skip.");
        }
    }

    INFO_PRINT("Pending Thread End.");
    return ( 0 );
}

/******************************************************************************
 *    function:   nfc_gpio_nint_active
 *    brief   :
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :
 *            :
 *    output  :
 ******************************************************************************/
int nfc_i2c_nint_active(void)
{
    int ret = 0;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;

    pin = devm_pinctrl_get(&g_nfc_i2c_info->nfc_i2c_client->dev);
    pin_state = pinctrl_lookup_state(pin, "nfc_nint_active");
    ret = pinctrl_select_state(pin, pin_state);
    if(ret){
        ERROR_PRINT("pinctrl_select_state Failed[%d].", ret );
    }

    return ret;
}

/******************************************************************************
 *    function:   nfc_gpio_nint_suspend
 *    brief   :
 *    date    :
 *    author  :
 *
 *    return  :    0                   normal exit
 *            :   -1                   error exit
 *    input   :
 *            :
 *    output  :
 ******************************************************************************/
int nfc_i2c_nint_suspend(void)
{
    int ret = 0;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;

    pin = devm_pinctrl_get(&g_nfc_i2c_info->nfc_i2c_client->dev);
    pin_state = pinctrl_lookup_state(pin, "nfc_nint_suspend");
    ret = pinctrl_select_state(pin, pin_state);
    if(ret){
        ERROR_PRINT("pinctrl_select_state Failed[%d].", ret );
    }

    return ret;
}

/******************************************************************************/
/***                                                                        ***/
/******************************************************************************/
module_init ( nfc_i2c_drv_start );
module_exit ( nfc_i2c_drv_end );

/******************************************************************************/
/***                                                                        ***/
/******************************************************************************/
MODULE_AUTHOR("Toshiba");
MODULE_DESCRIPTION("NFC GPIO Driver");
MODULE_LICENSE("GPL v2");
