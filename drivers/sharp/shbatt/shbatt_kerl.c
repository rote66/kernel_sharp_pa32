/* drivers/sharp/shbatt/shbatt_kerl.c
 *
 * Copyright (C) 2014 SHARP CORPORATION All rights reserved.
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/
/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/alarmtimer.h>	/* Timer */
#include <linux/time.h>	/* Timer */
#include <linux/qpnp/qpnp-adc.h>
#include <linux/namei.h>

#include "sharp/shbatt_kerl.h"
#include "shbatt_type.h"
#include "sharp/shpwr_log.h"
#include "sharp/sh_smem.h"
#ifdef CONFIG_SHTERM
#include "sharp/shterm_k.h"
#endif /* CONFIG_SHTERM */
#include <linux/cpufreq.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "SHBATT:%s: " fmt, __func__

#define SHBATT_ERROR(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_INFO(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_INFO,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_TRACE(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,x)

#define SHBATT_DEV_NAME						"shbatt"
#define SHBATT_OF_DEV_NAME					"sharp,shbatt"
#define SHBATT_ATTR_ARRAY_END_NAME			"END_NULL"
#define SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT 5

#define SHBATT_WAKE_CTL(x)										\
{																\
	do															\
	{															\
		if(x == 0)												\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) call shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			if(atomic_dec_return(&shbatt_wake_lock_num) == 0)	\
			{													\
				wake_unlock(&shbatt_wake_lock); 				\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) done shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			}													\
		}														\
		else													\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) call shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			if(atomic_inc_return(&shbatt_wake_lock_num) == 1)	\
			{													\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) done shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
				wake_lock(&shbatt_wake_lock);					\
			}													\
		}														\
	} while(0); 												\
}

#define SHBATT_ATTR_END											\
{																\
	.attr  =													\
	{															\
		.name = SHBATT_ATTR_ARRAY_END_NAME,						\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show  = NULL,												\
	.store = NULL,												\
}

#ifndef MAX
#define MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b)	(((a) < (b)) ? (a) : (b))
#endif

#define SHBATT_DEBUG_PRINT(_flg, _str, _val) \
	if (_flg) printk(KERN_ERR "%s: %s = %d\n", __FUNCTION__, #_str, _val)

#define DEBUG_CAPACITY_IS_ENABLE() \
		((debug_capacity >= 0) && (debug_capacity <= 100))

#define SHPWR_LOG_INFO(fmt, ...) { \
     shpwr_add_dbg_log(pr_fmt(fmt), ##__VA_ARGS__); \
     pr_info(fmt, ##__VA_ARGS__); \
}
#define SHPWR_DUMP_REG_INFO(fmt, ...) shpwr_add_dump_reg(false, fmt, ##__VA_ARGS__)
#define SHPWR_DUMP_REG_INFO_AND_FORCESAVE(fmt, ...) shpwr_add_dump_reg(true, fmt, ##__VA_ARGS__)

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/
#define GISPRODUCT_F_MASK					0x00000080
#define GISSOFTUP_F_MASK					0x00000010

/* for calc base battery capacity */
#define SHBATT_INVALID_CALC_BASE_CAPACITY	(-1)
#define SHBATT_INVALID_CALC_BASE_BAT_THERM	(-255)

/* Timer */
#define SHBATT_TIMER_FG_PERIOD_NS			(10LL * NSEC_PER_SEC)
#define SHBATT_TIMER_MSEC_TO_NSEC(x)		(x * NSEC_PER_MSEC)
#define SHBATT_TIMER_SEC_TO_NSEC(x)			(x * NSEC_PER_SEC)

/* wake_lock */
#define SHBATT_LOCK_FUNC_LEN				64	/* SH_PWR_DEBUG T.B.D */

#define SHBATT_FAIL_SAFE_INPUT_EVENT		0
#define SHBATT_FAIL_SAFE_RESUME				1
#define SHBATT_NON_DISP_CHAR				(-128)

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
typedef enum
{
	SHBATT_TASK_COMMAND_LOCK,
	SHBATT_MUTEX_TYPE_NUM
}shbatt_kernel_mutex_type;

typedef enum
{
	SHBATT_TIMER_TYPE_HRTIMER,
	SHBATT_TIMER_TYPE_ALARMTIMER,
	SHBATT_TIMER_TYPE_NUM
}shbatt_timer_type;

#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
typedef enum {
	SHBATT_SYSFS_PARAM_TYPE_STRING,
	SHBATT_SYSFS_PARAM_TYPE_STRING_DATE,
	SHBATT_SYSFS_PARAM_TYPE_STRING_DEC,
	SHBATT_SYSFS_PARAM_TYPE_STRING_HEX,
	SHBATT_SYSFS_PARAM_TYPE_UINT8_DEC,
	SHBATT_SYSFS_PARAM_TYPE_UINT8_HEX,
	SHBATT_SYSFS_PARAM_TYPE_UINT16_DEC,
	SHBATT_SYSFS_PARAM_TYPE_UINT16_HEX,
	SHBATT_SYSFS_PARAM_TYPE_MAX
} shbatt_sysfs_parameter_type_t;
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
/* Timer */
typedef struct shbatt_timer_tag
{
	union
	{
		struct alarm			alarm_timer;
		struct hrtimer			hr_timer;

	}alm;
	union
	{
		enum alarmtimer_restart	(*alarm_cb)(struct alarm *, ktime_t);
		enum hrtimer_restart	(*hrtimer_cb)(struct hrtimer *);
	}cb_func;
	shbatt_timer_type			timer_type;
	enum alarmtimer_type		alarm_type;
	int							prm;
} shbatt_timer_t;

#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
typedef struct {
	const char*					name;
	char*						address;
	const unsigned int			size;
	char						str_buf[16];
	shbatt_sysfs_parameter_type_t	param_type;
} write_info_t;
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static bool						shbatt_task_is_initialized = false;
static struct wake_lock			shbatt_wake_lock;
static atomic_t					shbatt_wake_lock_num;

static dev_t					shbatt_dev;
static int						shbatt_major;
static int						shbatt_minor;
static struct cdev				shbatt_cdev;
static struct class*			shbatt_dev_class;
static atomic_t					shbatt_usse_op_cnt;
struct timespec					shbatt_last_hrtimer_expire_time;
struct timespec					shbatt_last_timer_expire_time;
static bool						shbatt_timer_restarted = false;

/* Timer */
static spinlock_t				shbatt_pkt_lock;
static struct mutex				shbatt_task_lock;
static struct workqueue_struct*	shbatt_task_workqueue_p;
static wait_queue_head_t		shbatt_usse_wait;
static struct completion		shbatt_usse_cmp;
static shbatt_packet_t			shbatt_pkt[16];
static shbatt_usse_packet_t		shbatt_usse_pkt;

/* wake_lock */
static struct timespec			shbatt_lock_time[SHBATT_MUTEX_TYPE_NUM];
static struct timespec			shbatt_unlock_time[SHBATT_MUTEX_TYPE_NUM];
static char						shbatt_lock_func[SHBATT_MUTEX_TYPE_NUM][SHBATT_LOCK_FUNC_LEN];

struct shbatt_chip {
	struct qpnp_vadc_chip        *pm_vadc_dev;
	struct qpnp_vadc_chip        *pmi_vadc_dev;
};

static struct shbatt_chip *the_chip = NULL;
static int shbatt_cur_depleted_val = 100;
static int shbatt_depleted_capacity_pos = 0;
static int shbatt_depleted_capacity_array[5] = {0,};
static char shbatt_depleted_calc_ver = 2;
static int shbatt_avr_depleted_val = 0;


#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
static char						pack_name[] = "0000";
module_param_string(pack_name, pack_name, sizeof(pack_name), 0600);

static char						cell_maker_name[] = "0000000000000000";
module_param_string(cell_maker_name, cell_maker_name, sizeof(cell_maker_name), 0600);

static char						maker_name[] = "0000000000000000";
module_param_string(maker_name, maker_name, sizeof(maker_name), 0600);

static char						cell_date[] = "00000000";
module_param_string(cell_date, cell_date, sizeof(cell_date), 0644);

static char						cell_line[] = "00";
module_param_string(cell_line, cell_line, sizeof(cell_line), 0644);

static char						pack_date[] = "00000000";
module_param_string(pack_date, pack_date, sizeof(pack_date), 0644);

static char						pack_line[] = "00";
module_param_string(pack_line, pack_line, sizeof(pack_line), 0644);

static char						pack_manu_num[] = "0000";
module_param_string(pack_manu_num, pack_manu_num, sizeof(pack_manu_num), 0644);

#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */

#ifdef CONFIG_PM_SUPPORT_BATT_AUTH
static int						batt_auth = -1;
module_param(batt_auth, int, 0644);
#else /* CONFIG_PM_SUPPORT_BATT_AUTH */
#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
static int						batt_auth = 1;
module_param(batt_auth, int, 0644);
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */
#endif /*CONFIG_PM_SUPPORT_BATT_AUTH*/

static int shbatt_limit_lock_callback(struct notifier_block *nb, unsigned long event, void *data);
static struct notifier_block shbatt_limit_lock_notifier =
{
	.notifier_call = shbatt_limit_lock_callback,
};
static u32 shbatt_limit_freq = UINT_MAX;

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static int shbatt_drv_create_device( void );

/* task */
/* task(Timer) */
static void shbatt_task(
	struct work_struct*			work_p );

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p );

/* seq */
static shbatt_result_t shbatt_seq_initialize( void );

static shbatt_result_t shbatt_seq_exec_fuelgauge_soc_poll_sequence( void );
static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time );

static shbatt_result_t shbatt_seq_battlog_event(
	int							evt );

/* api */
static shbatt_result_t shbatt_api_initialize( void );

static shbatt_result_t shbatt_api_exec_fuelgauge_soc_poll_sequence( void );

/* from ioctrl. */
/* Timer */
static int shbatt_drv_ioctl_cmd_initialize(
	struct file*				fi_p,
	unsigned					long arg );

static int shbatt_drv_ioctl_cmd_pull_usse_packet(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_done_usse_packet(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_set_timer(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_clr_timer(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_get_boot_time(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_get_shbatt_log_info(
	struct file*				fi_p,
	unsigned long				arg );


/* Timer */
static shbatt_packet_t* shbatt_task_get_packet( void );

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt );

static shbatt_result_t shbatt_seq_call_user_space_sequence_executor( void );

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE PROTO TYPE DECLARE :                               |*/
/*+-----------------------------------------------------------------------------+*/
/* attribute store */

/* driver I/F */
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p );

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p );

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p );

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg );

static int shbatt_drv_probe(
	struct platform_device*		dev_p );

static int shbatt_drv_remove(
	struct platform_device*		dev_p );

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p );

static int shbatt_drv_resume(
	struct platform_device*		dev_p);

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state );
static int shbatt_depleted_backup_params(void);
static int shbatt_depleted_restore_params(void);

static int __init shbatt_drv_module_init( void );
static void __exit shbatt_drv_module_exit( void );

bool is_shbatt_prs_launched( void );

/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static struct file_operations shbatt_fops =
{
	.owner			= THIS_MODULE,
	.open			= shbatt_drv_open,
	.release		= shbatt_drv_release,
	.poll			= shbatt_drv_poll,
	.unlocked_ioctl	= shbatt_drv_ioctl,
	.compat_ioctl	= shbatt_drv_ioctl,
};

#ifdef CONFIG_OF
static struct of_device_id shbatt_match_table[] = {
	{ .compatible = SHBATT_OF_DEV_NAME },
	{}
};
#else  /* CONFIG_OF */
#define shbatt_match_table NULL;
#endif /* CONFIG_OF */

static struct platform_driver shbatt_platform_driver = {
	.probe		= shbatt_drv_probe,
	.remove		= shbatt_drv_remove,
	.shutdown	= shbatt_drv_shutdown,
	.driver		= {
		.name	= SHBATT_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = shbatt_match_table,
	},
	.resume		= shbatt_drv_resume,
	.suspend	= shbatt_drv_suspend,
};

/* Timer */
static void (*const shbatt_task_cmd_func[])( shbatt_packet_t* pkt_p ) =
{
	shbatt_task_cmd_invalid,									/* SHBATT_TASK_CMD_INVALID */
	shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence,			/* SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE */
	shbatt_task_cmd_battlog_event,								/* SHBATT_TASK_CMD_BATTLOG_EVENT */
};

static shbatt_timer_t			shbatt_poll_timer[NUM_SHBATT_POLL_TIMER_TYPE] =
{
	{
		.cb_func =
		{
			.alarm_cb = shbatt_seq_fuelgauge_soc_poll_timer_expire_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_ALARMTIMER,
	},
};

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

shbatt_result_t shbatt_api_get_smem_info( shbatt_smem_info_t * p_smem_info )
{
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	sharp_smem_common_type * p_smem = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	p_smem = sh_smem_get_common_address();

	if( p_smem_info && p_smem )
	{
#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
		strncpy(p_smem_info->traceability_info, p_smem->shpwr_traceability, sizeof(p_smem->shpwr_traceability));
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */
	}
	else
	{
		SHBATT_ERROR("%s : invalid pointer, p_smem_info=0x%p, p_smem=0x%p\n",__FUNCTION__, p_smem_info, p_smem);
		result = SHBATT_RESULT_FAIL;
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return result;
}

static int shbatt_drv_create_device( void )
{
	struct device*				dev_p;
	int							ret;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	ret = alloc_chrdev_region(&shbatt_dev,0,1,SHBATT_DEV_NAME);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : alloc_chrdev_region failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_0;
	}

	shbatt_major = MAJOR(shbatt_dev);
	shbatt_minor = MINOR(shbatt_dev);

	cdev_init(&shbatt_cdev,&shbatt_fops);

	shbatt_cdev.owner = THIS_MODULE;

	ret = cdev_add(&shbatt_cdev,shbatt_dev,1);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : cdev_add failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_1;
	}

	shbatt_dev_class = class_create(THIS_MODULE,SHBATT_DEV_NAME);

	if(IS_ERR(shbatt_dev_class))
	{
		ret = PTR_ERR(shbatt_dev_class);
		SHBATT_ERROR("%s : class_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_2;
	}

	dev_p = device_create(shbatt_dev_class,NULL,shbatt_dev,&shbatt_cdev,SHBATT_DEV_NAME);

	if(IS_ERR(dev_p))
	{
		ret = PTR_ERR(dev_p);
		SHBATT_ERROR("%s : device_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_3;
	}

	atomic_set(&shbatt_usse_op_cnt,0);

	/* Timer */
	init_waitqueue_head(&shbatt_usse_wait);
	init_completion(&shbatt_usse_cmp);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;

create_device_exit_3:
	class_destroy(shbatt_dev_class);

create_device_exit_2:
	cdev_del(&shbatt_cdev);

create_device_exit_1:
	unregister_chrdev_region(shbatt_dev,1);

create_device_exit_0:

	return ret;
}

static void shbatt_task(
	struct work_struct*			work_p
){
	shbatt_packet_t*			pkt_p;

	shbatt_seq_lock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	pkt_p = (shbatt_packet_t*)work_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"shbatt_task %d \n",pkt_p->hdr.cmd );

	if(pkt_p->hdr.cmd < NUM_SHBATT_TASK_CMD)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task OK \n");
		shbatt_task_cmd_func[pkt_p->hdr.cmd](pkt_p);
	}
	else
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task NG \n");
		shbatt_task_cmd_invalid(pkt_p);
	}

	SHBATT_WAKE_CTL(0);

	shbatt_task_free_packet(pkt_p);

	shbatt_seq_unlock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_exec_fuelgauge_soc_poll_sequence();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_battlog_event(pkt_p->prm.evt);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static shbatt_packet_t* shbatt_task_get_packet( void )
{
	int							idx;
	unsigned long				flags;
	shbatt_packet_t*			ret = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	for( idx = 0; idx < 16; idx++ )
	{
		if( shbatt_pkt[idx].is_used == false )
		{
			shbatt_pkt[idx].is_used = true;

			ret = &shbatt_pkt[idx];

			break;
		}
	}

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return ret;
}

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt
){
	unsigned long				flags;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	pkt->is_used = false;

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return;
}

static shbatt_result_t shbatt_seq_initialize( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_task_is_initialized = true;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static shbatt_result_t shbatt_seq_exec_fuelgauge_soc_poll_sequence( void )
{
	shbatt_result_t				result;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_usse_pkt.hdr.cmd = SHBATT_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE;
	shbatt_usse_pkt.hdr.ret = SHBATT_RESULT_FAIL;

	result = shbatt_seq_call_user_space_sequence_executor();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time
){
	enum alarmtimer_restart		ret = ALARMTIMER_NORESTART;
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_api_exec_fuelgauge_soc_poll_sequence();

	get_monotonic_boottime( &shbatt_last_timer_expire_time );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[P] %s shbatt_last_timer_expire_time.tv_sec=%010lu shbatt_last_timer_expire_time.tv_nsec=%09lu\n",__FUNCTION__,
				 shbatt_last_timer_expire_time.tv_sec, shbatt_last_timer_expire_time.tv_nsec);

	if(shbatt_timer_restarted == true)
	{
		shbatt_timer_restarted = false;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
	return ret;
}

static shbatt_result_t shbatt_seq_call_user_space_sequence_executor( void )
{
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	//INIT_COMPLETION(shbatt_usse_cmp);
	reinit_completion(&shbatt_usse_cmp);
	atomic_inc(&shbatt_usse_op_cnt);
	wake_up_interruptible(&shbatt_usse_wait);
	wait_for_completion_killable(&shbatt_usse_cmp);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );


	return shbatt_usse_pkt.hdr.ret;
}

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	strncpy( &shbatt_lock_func[type][0], func, SHBATT_LOCK_FUNC_LEN - 1 );
	get_monotonic_boottime( &shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG, SHPWR_LOG_TYPE_BATT,
				"[P] %s() lock start\n", &shbatt_lock_func[type][0] );

	mutex_lock(&shbatt_task_lock);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	struct timespec						diff;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	mutex_unlock(&shbatt_task_lock);
	get_monotonic_boottime( &shbatt_unlock_time[type] );

	memset(&diff, 0x00, sizeof( diff ) );
	diff = timespec_sub( shbatt_unlock_time[type], shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
				"[P] %s() locktime:%lu.%09lu\n", &shbatt_lock_func[type][0], diff.tv_sec, diff.tv_nsec );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

/* api(Timer) */
static shbatt_result_t shbatt_api_initialize( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == true)
	{
		shbatt_task_is_initialized = false;
	}
	result = shbatt_seq_initialize();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static shbatt_result_t shbatt_api_exec_fuelgauge_soc_poll_sequence( void )
{
	shbatt_packet_t*			pkt_p;
	ktime_t						set_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		memset( &set_time,0x00, sizeof( set_time ) );
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"[P] %s() timer restart\n",__FUNCTION__);
		set_time.tv64 = SHBATT_TIMER_FG_PERIOD_NS;
		alarm_start_relative(&(shbatt_poll_timer[SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC].alm.alarm_timer),
					set_time );
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt
){
	shbatt_packet_t*			pkt_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_BATTLOG_EVENT;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.evt		= evt;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status
){
	int charge_status_event = SHBATTLOG_EVENT_NONE;
	static int pre_charge_status_event = SHBATTLOG_EVENT_NONE;

	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_START;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_END;
		break;
	case POWER_SUPPLY_STATUS_FULL:
		charge_status_event = SHBATTLOG_EVENT_CHG_COMP;
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_ERROR;
		break;
	default:
		charge_status_event = SHBATTLOG_EVENT_NONE;
	}

	if(charge_status_event != SHBATTLOG_EVENT_NONE && charge_status_event != pre_charge_status_event) {
		SHPWR_LOG_INFO("pre_charge_status_event = %d, charge_status_event = %d\n", pre_charge_status_event, charge_status_event);

		shbatt_api_battlog_event(charge_status_event);
		pre_charge_status_event = charge_status_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_event
){
	static int pre_charge_error_event = SHBATTLOG_EVENT_NONE;

	if(charge_error_event != SHBATTLOG_EVENT_NONE && charge_error_event != pre_charge_error_event) {
		SHPWR_LOG_INFO("pre_charge_error_event = %d, charge_error_event = %d\n", pre_charge_error_event, charge_error_event);

		shbatt_api_battlog_event(charge_error_event);
		pre_charge_error_event = charge_error_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status
){
	static int jeita_pre_status = POWER_SUPPLY_HEALTH_UNKNOWN;

	if (jeita_cur_status  != jeita_pre_status) {
		SHPWR_LOG_INFO("jeita_pre_status = %d,jeita_cur_status  = %d\n", jeita_pre_status , jeita_cur_status );

		switch (jeita_cur_status){
		case POWER_SUPPLY_HEALTH_OVERHEAT:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_COLD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_WARM:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_COOL:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_GOOD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FAST_ST);
			break;
		default:
			break;
		}
	}
	jeita_pre_status = jeita_cur_status;

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity
){
	static int pre_capacity = 80;
	static const shbattlog_event_num event_tbl[10] =
	{
		SHBATTLOG_EVENT_FGIC_EX10,
		SHBATTLOG_EVENT_FGIC_EX20,
		SHBATTLOG_EVENT_FGIC_EX30,
		SHBATTLOG_EVENT_FGIC_EX40,
		SHBATTLOG_EVENT_FGIC_EX50,
		SHBATTLOG_EVENT_FGIC_EX60,
		SHBATTLOG_EVENT_FGIC_EX70,
		SHBATTLOG_EVENT_FGIC_EX80,
		SHBATTLOG_EVENT_FGIC_EX90,
		SHBATTLOG_EVENT_FGIC_EX100
	};

	if( pre_capacity != cur_capacity ){
		pr_debug("pre_capacity = %d cur_capacity  = %d\n", pre_capacity, cur_capacity );

		if(cur_capacity == 0)
		{
			shbatt_api_battlog_event(SHBATTLOG_EVENT_INDICATER_0);
			shbatt_api_battlog_event(SHBATTLOG_EVENT_FATAL_BATT);
		}
		else
		{
			if(cur_capacity % 10 == 0)
			{
				pr_debug("event cur_cap  = %d tbl:%d\n", cur_capacity, event_tbl[(cur_capacity/10)-1] );
				shbatt_api_battlog_event(event_tbl[(cur_capacity/10)-1]);
			}
		}
		pre_capacity = cur_capacity;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type
){
	int usb_type_event = SHBATTLOG_EVENT_NONE;

	switch (usb_type) {
	case 0:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_SDP;
		break;
	case 2:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_DCP;
		break;
	case 3:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_CDP;
		break;
	default:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_OTHER;
	}

	SHPWR_LOG_INFO("usb_type = %d, usb_type_event = %d\n", usb_type, usb_type_event);

	shbatt_api_battlog_event(usb_type_event);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_get_battery_log_info(
	shbatt_batt_log_info_t*		bli_p
){
	int							ret;
	union power_supply_propval	val = {0,};
	struct power_supply*		batt_psy = NULL;
	struct power_supply*		bms_psy = NULL;
	struct qpnp_vadc_result		vadc_val;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() shbatt_task_is_initialized is false. \n", __FUNCTION__ );
		return SHBATT_RESULT_REJECTED;
	}
	batt_psy = power_supply_get_by_name("battery");
	if( batt_psy == NULL )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() batt_psy is NULL.\n", __FUNCTION__ );
		return SHBATT_RESULT_REJECTED;
	}
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s() power_supply_get_by_name()\n",__FUNCTION__);

	bms_psy = power_supply_get_by_name("bms");
	if( bms_psy == NULL )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() bms_psy is NULL.\n", __FUNCTION__ );
		return SHBATT_RESULT_REJECTED;
	}
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s() power_supply_get_by_name()\n",__FUNCTION__);

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_VOLTAGE_NOW is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->bat_vol = val.intval / 1000;
	bli_p->avg_vol = val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. bat_vol:%d.avg_vol:%d.\n",__FUNCTION__,
				val.intval, bli_p->bat_vol, bli_p->avg_vol );
	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_TEMP is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->bat_temp = val.intval / 10;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. bat_temp:%d.\n",__FUNCTION__,
				val.intval, bli_p->bat_temp );
	val.intval = 0;

	if ((bli_p->event_num == SHBATTLOG_EVENT_INDICATER_0)
		|| (bli_p->event_num == SHBATTLOG_EVENT_FATAL_BATT)) {
		val.intval = 0;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX10) {
		val.intval = 10;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX20) {
		val.intval = 20;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX30) {
		val.intval = 30;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX40) {
		val.intval = 40;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX50) {
		val.intval = 50;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX60) {
		val.intval = 60;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX70) {
		val.intval = 70;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX80) {
		val.intval = 80;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX90) {
		val.intval = 90;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX100) {
		val.intval = 100;
	} else {
		ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val );
		if( ret < 0 )
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d POWER_SUPPLY_PROP_CAPACITY is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
	}
	bli_p->vol_per = val.intval;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. vol_per:%d.\n",__FUNCTION__,
				val.intval, bli_p->vol_per );
	val.intval = 0;

	if (the_chip->pmi_vadc_dev) {
		ret = qpnp_vadc_read(the_chip->pmi_vadc_dev,USBIN,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d shbatt_vadc_channel_read() USBIN is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->chg_vol = (int)vadc_val.physical / 1000;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. chg_vol:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->chg_vol );
	} else {
		 SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s() pmi dev is not exist \n", __FUNCTION__ );

			return SHBATT_RESULT_REJECTED;
	}

	if (the_chip->pm_vadc_dev) {
		ret = qpnp_vadc_read(the_chip->pm_vadc_dev,LR_MUX4_PU1_AMUX_THM1,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d qpnp_vadc_read() LR_MUX4_PU1_AMUX_THM1 is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
	        }
		bli_p->cpu_temp = (int)vadc_val.physical;
		bli_p->chg_temp = (int)vadc_val.physical;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. cpu_temp:%d. chg_temp:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->cpu_temp, bli_p->chg_temp );

		ret = qpnp_vadc_read(the_chip->pm_vadc_dev,DIE_TEMP,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d qpnp_vadc_read() DIE_TEMP is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->pmic_temp = (int)vadc_val.physical / 1000;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. pmic_temp:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->pmic_temp );

		ret = qpnp_vadc_read(the_chip->pm_vadc_dev,LR_MUX6_PU1_AMUX_THM3,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d qpnp_vadc_read() LR_MUX6_PU1_AMUX_THM3 is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->pa_temp = (int)vadc_val.physical;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. pa_temp:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->pa_temp );

		ret = qpnp_vadc_read(the_chip->pm_vadc_dev,LR_MUX5_PU1_AMUX_THM2,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d qpnp_vadc_read() LR_MUX5_PU1_AMUX_THM2 is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->cam_temp = (int)vadc_val.physical;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. cam_temp:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->cam_temp );

		ret = qpnp_vadc_read(the_chip->pm_vadc_dev,LR_MUX9_PU1_AMUX_THM5,&vadc_val);
		if ( ret < 0 ) {
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d qpnp_vadc_read() LR_MUX9_PU1_AMUX_THM5 is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->lcd_temp = (int)vadc_val.physical;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
						"[P] %s()vadc_val.physical:%ld. lcd_temp:%d.\n",__FUNCTION__,
						(long)vadc_val.physical, bli_p->lcd_temp );
	} else {
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s() PM dev is not exist \n", __FUNCTION__ );

		return SHBATT_RESULT_REJECTED;
	}

	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_CURRENT_NOW is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->latest_cur = -val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. latest_cur:%d.\n",__FUNCTION__,
				-val.intval, bli_p->latest_cur );

	val.intval = 0;

	ret = bms_psy->get_property(bms_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_CURRENT_AVG is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->avg_cur = -val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. avg_cur:%d.\n",__FUNCTION__,
				-val.intval, bli_p->avg_cur );

	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_INPUT_CURRENT_NOW is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->chg_cur = val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. chg_cur:%d.\n",__FUNCTION__,
				val.intval, bli_p->chg_cur );
	val.intval = 0;

	bli_p->acc_cur = 0;
	bli_p->cur_dep_per = shbatt_cur_depleted_val;
	bli_p->avg_dep_per = shbatt_avr_depleted_val;

	if (bli_p->event_num == SHBATTLOG_EVENT_NONE)	// SHBATTLOG_EVENT_BATT_REPORT_NORM or SHBATTLOG_EVENT_BATT_REPORT_CHG
		sh_dump_regs();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
static int shbatt_api_chk_smem_str( write_info_t *file )
{
	unsigned int	chk_cntr;
	char*			tmp_buf = NULL;

	SHBATT_TRACE( "[S] %s str:%s\n", __FUNCTION__, file->str_buf );

	tmp_buf = file->str_buf;
	for( chk_cntr = 0; chk_cntr < file->size; chk_cntr++ ) {
		if( ( tmp_buf[chk_cntr] >= '0' ) &&
			( tmp_buf[chk_cntr] <= '9' ) ) {
			;
		}
		else if( ( ( tmp_buf[chk_cntr] >= 'A' ) &&
				( tmp_buf[chk_cntr] <= 'F' ) ) ||
				( ( tmp_buf[chk_cntr] >= 'a' ) &&
				( tmp_buf[chk_cntr] <= 'f' ) ) ) {
			switch( file->param_type ) {
			case SHBATT_SYSFS_PARAM_TYPE_STRING_DATE:
			case SHBATT_SYSFS_PARAM_TYPE_STRING_DEC:
			case SHBATT_SYSFS_PARAM_TYPE_UINT8_DEC:
			case SHBATT_SYSFS_PARAM_TYPE_UINT16_DEC:
				SHBATT_ERROR( "[E]A-F a-f found in str_buff[%d]:%c. type:%d\n", chk_cntr, tmp_buf[chk_cntr], file->param_type );
				snprintf( file->str_buf, file->size+1, "000000000000000" );
				return 1;
			default:
				break;
			}
		}
		else if( ( ( tmp_buf[chk_cntr] >= 'G' ) &&
				( tmp_buf[chk_cntr] <= 'Z' ) ) ||
				( ( tmp_buf[chk_cntr] >= 'g' ) &&
				( tmp_buf[chk_cntr] <= 'z' ) ) ) {
			switch( file->param_type ) {
			case SHBATT_SYSFS_PARAM_TYPE_STRING:
				break;
			default:
				SHBATT_ERROR( "[E]G-Z a-f found in str_buff[%d]:%c. type:%d\n", chk_cntr, tmp_buf[chk_cntr], file->param_type );
				snprintf( file->str_buf, file->size+1, "000000000000000" );
				return 1;
			}
		}
		else if( tmp_buf[chk_cntr] == '\0' ) {
			SHBATT_ERROR( "[E]str_buf[%d] is NULL. type:%d\n",chk_cntr, file->param_type );
			snprintf( file->str_buf, file->size+1, "000000000000000" );
			return 1;
		}
		else {
			SHBATT_ERROR( "[E]str_buf[%d]:%c. type:%d\n",chk_cntr, tmp_buf[chk_cntr], file->param_type );
			snprintf( file->str_buf, file->size+1, "000000000000000" );
			return 1;
		}
	}

	SHBATT_TRACE( "[E] %s\n", __FUNCTION__ );
	return 0;
}

static shbatt_result_t shbatt_api_write_traceability( void )
{
	static write_info_t	write_attr[] =
	{
		{
			.name = "cell_date",
			.address = &cell_date[0],
			.size = sizeof("YYYYMMDD")-1,
			.str_buf = "",
			.param_type = SHBATT_SYSFS_PARAM_TYPE_STRING_DATE
		},
		{
			.name = "cell_line",
			.address = &cell_line[0],
			.size = sizeof("xx")-1,
			.str_buf = "",
			.param_type = SHBATT_SYSFS_PARAM_TYPE_STRING_HEX
		},
		{
			.name = "pack_date",
			.address = &pack_date[0],
			.size = sizeof("yyyymmdd")-1,
			.str_buf = "",
			.param_type = SHBATT_SYSFS_PARAM_TYPE_STRING_DATE
		},
		{
			.name = "pack_manu_num",
			.address = &pack_manu_num[0],
			.size = sizeof("ssss")-1,
			.str_buf = "",
			.param_type = SHBATT_SYSFS_PARAM_TYPE_UINT16_DEC
		},
	};
	shbatt_smem_info_t			smem_info;
	char*						write_p;
	shbatt_result_t				shbatt_result;
	unsigned int				write_cntr;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	memset(&smem_info, 0x00, sizeof(shbatt_smem_info_t));
	shbatt_result = shbatt_api_get_smem_info(&smem_info);
	if (shbatt_result != SHBATT_RESULT_SUCCESS) {
		SHBATT_ERROR("[E] shbatt_api_get_smem_info failed:%d.\n", shbatt_result );
		return shbatt_result;
	}

	write_p = smem_info.traceability_info;

	for( write_cntr = 0; write_cntr < sizeof(write_attr)/sizeof(write_attr[0]); write_cntr++ ) {

		write_info_t *w = &write_attr[write_cntr];
		snprintf( w->str_buf, w->size+1, "%s", write_p );

		if( shbatt_api_chk_smem_str( w ) == 1 ) {
			SHBATT_ERROR("smem check %s. end fail-safe.\n", w->name);
		}

		SHBATT_TRACE("[P] %s. size:%d str:%s\n", w->name, w->size, w->str_buf );

		strncpy(w->address, w->str_buf, w->size);

		write_p += w->size;
	}

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */
#define SHBATT_DEPLETED_CACPCITY		("/durable/shbatt_depleted_calc_ver")
static int shbatt_depleted_backup_params(void)
{
	struct file *fp = NULL;
	int count = 0;
	int average_depleted_val = 0;
	char write_data[50];
	mm_segment_t old_fs;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	SHBATT_TRACE("[S] %s\n", __FUNCTION__);
	for(count = 0; count < shbatt_depleted_capacity_pos; count++)
	{
		average_depleted_val += shbatt_depleted_capacity_array[count];
	}
	average_depleted_val = average_depleted_val / shbatt_depleted_capacity_pos;
	shbatt_avr_depleted_val = average_depleted_val;

	fp = filp_open(SHBATT_DEPLETED_CACPCITY, O_RDWR | O_CREAT, 0644);
	if (IS_ERR_OR_NULL(fp)) {
		SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
		result = SHBATT_RESULT_FAIL;
	} else {
		old_fs = get_fs();
		set_fs(get_ds());
		count = snprintf(write_data, sizeof(write_data), "%d\n%d\n%d\n",
				shbatt_depleted_calc_ver, average_depleted_val, shbatt_depleted_capacity_pos);
		if((res = (int)fp->f_op->write(fp, write_data, count, &fp->f_pos)) == 0) {
			SHBATT_ERROR("%s : fwrite result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}

		if((res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0)) != 0) {
			SHBATT_ERROR("%s : fsync result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	}
	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

#define SHBATT_READ_DATA_BUF_LEN 50
static int shbatt_depleted_restore_params(void)
{
	struct file *fp = NULL;
	int res = -EINVAL;
	int ver;
	int average_depleted_val = 0;
	int depleted_capacity_pos = 0;
	int count = 0;
	mm_segment_t old_fs;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	struct path  path;
	char read_data[50];
	loff_t fpos = 0;
	int lvl_index = 0;
	int i =0;
	int buff[3];
	char* val_start;
	int ret = 0;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	res = kern_path(SHBATT_DEPLETED_CACPCITY, LOOKUP_OPEN, &path);
	if (res == 0) {
		fp = filp_open(SHBATT_DEPLETED_CACPCITY, O_RDONLY, 0644);
		if (!IS_ERR_OR_NULL(fp)) {
			old_fs = get_fs();
			set_fs(get_ds());

			if ((res = (int)fp->f_op->read(fp, read_data, SHBATT_READ_DATA_BUF_LEN, &fpos)) != 0) {
				read_data[res] = '\0';

				for (lvl_index = 0; lvl_index < res && i < res; lvl_index++) {
					while(read_data[i] == ' ' && i < res) i++; /* skip the blank space of head */
					if (i == res) {
						break;
					}
					val_start = &read_data[i];
					while(read_data[i] != ' ' && read_data[i] != '\0' && read_data[i] != '\n'&& i < res) i++; /* to the next blank space */
					if (read_data[i] == '\n') {
						read_data[i] = '\0';
						i++;
					}
					if(lvl_index > 2){
						break;
					}
					ret = kstrtoint(val_start, 0, &buff[lvl_index]);
					if(ret) {
						break;
					}
					if (read_data[i] == '\0') {
						break;
					}
				}
				ver = buff[0];
				average_depleted_val = buff[1];
				depleted_capacity_pos = buff[2];
				for(count = 0; count < depleted_capacity_pos; count++)
				{
					shbatt_depleted_capacity_array[count] = average_depleted_val;
				}
				shbatt_depleted_capacity_pos = depleted_capacity_pos;
			}

			set_fs(old_fs);
			filp_close(fp, NULL);
		} else {
			SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
			if (!is_shbatt_prs_launched()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			result = SHBATT_RESULT_FAIL;
		}
	} else {
		SHBATT_TRACE("[P] %s : Not Exist file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
		if (!is_shbatt_prs_launched()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
		result = SHBATT_RESULT_FAIL;
	}

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

#define SHBATT_DEP_RESULT_FILE		("/durable/shbatt")
#define SHBATT_DEP_RESULT_BUF_LEN	1
shbatt_result_t shbatt_api_store_fg_cap_learning_result(
	int64_t learned_cc_uah,
	int nom_mah,
	int high_thresh,
	int low_thresh
) {
	int dep_per;
	int dep_result;
	static int prev_dep_result = -1; /* Uninitialized : -1 */
	struct file *fp = NULL;
	char *bufp;
	mm_segment_t old_fs;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	struct path  path;
	loff_t fpos = 0;
	int count;
	static bool call_once = false;

	if(!call_once)
	{
		shbatt_depleted_restore_params();
		call_once = true;
	}

	dep_per = div64_s64(learned_cc_uah * 100, nom_mah * 1000);
	if(shbatt_depleted_capacity_pos < SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT)
	{
		shbatt_depleted_capacity_array[shbatt_depleted_capacity_pos] = dep_per;
		shbatt_depleted_capacity_pos++;
	} else {
		for(count = 1; count < shbatt_depleted_capacity_pos; count++)
		{
			shbatt_depleted_capacity_array[count-1] = shbatt_depleted_capacity_array[count];
		}
		shbatt_depleted_capacity_array[count-1] = dep_per;
	}
	shbatt_depleted_backup_params();

	shbatt_cur_depleted_val = dep_per;
	
	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	/* Good: 0, Depleted: 1, Acceptable: 2 */
	if(shbatt_avr_depleted_val >= high_thresh)
		dep_result = 0;
	else if(shbatt_avr_depleted_val <= low_thresh)
		dep_result = 1;
	else /* low_thresh < dep_per < high_thresh*/
		dep_result = 2;
	
	SHBATT_TRACE("[P] %s : prev_dep_result=%d dep_result=%d\n", __FUNCTION__, prev_dep_result, dep_result);
	
	bufp = kmalloc(SHBATT_DEP_RESULT_BUF_LEN + 1, GFP_KERNEL);
	if (!bufp) return SHBATT_RESULT_FAIL;
	
	memset(bufp, 0, SHBATT_DEP_RESULT_BUF_LEN + 1);
	if (prev_dep_result == -1) {
		res = kern_path(SHBATT_DEP_RESULT_FILE, LOOKUP_OPEN, &path);
		if (res == 0) {
			fp = filp_open(SHBATT_DEP_RESULT_FILE, O_RDONLY, 0644);
			if (!IS_ERR_OR_NULL(fp)) {
				old_fs = get_fs();
				set_fs(get_ds());
				
				if ((res = (int)fp->f_op->read(fp, bufp, SHBATT_DEP_RESULT_BUF_LEN, &fpos)) != 0) {
					bufp[res] = '\0';
					if (kstrtol(bufp, 10, (long *)&prev_dep_result) != 0)
						prev_dep_result = -1;
					SHBATT_TRACE("[P] %s : read file: %s bufp:%s --> prev_dep_result=%d \n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, bufp, prev_dep_result);
				}
				set_fs(old_fs);
				
				filp_close(fp, NULL);
			} else {
				SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
				if (!is_shbatt_prs_launched()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			}
		}
		 else {
			SHBATT_TRACE("[P] %s : Not Exist file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
			if (!is_shbatt_prs_launched()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
		}
	}
	if(dep_result != prev_dep_result)
	{
		fp = filp_open(SHBATT_DEP_RESULT_FILE, O_RDWR | O_CREAT, 0644);
		if (IS_ERR_OR_NULL(fp)) {
			SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
			if (!is_shbatt_prs_launched()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			result = SHBATT_RESULT_FAIL;
			if (prev_dep_result != -1) prev_dep_result = dep_result;
		} else {
			sprintf(bufp, "%d", dep_result);
			old_fs = get_fs();
			set_fs(get_ds());

			if((res = (int)fp->f_op->write(fp, bufp, SHBATT_DEP_RESULT_BUF_LEN, &fp->f_pos)) == 0) {
				SHBATT_ERROR("%s : fwrite result: %d\n", __FUNCTION__, res);
				result = SHBATT_RESULT_FAIL;
			}

			if((res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0)) != 0) {
				SHBATT_ERROR("%s : fsync result: %d\n", __FUNCTION__, res);
				result = SHBATT_RESULT_FAIL;
			}
			set_fs(old_fs);
			
			filp_close(fp, NULL);
			prev_dep_result = dep_result;
		}
	}
	kfree(bufp);
	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

static shbatt_result_t shbatt_seq_battlog_event(
	int		evt
) {
	shbattlog_info_t			shterm_bli;
	shbatt_batt_log_info_t		bli;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset(&shterm_bli, 0, sizeof(shterm_bli));
	shterm_bli.event_num = evt;

	memset(&bli, 0, sizeof(bli));
	bli.event_num = evt;

	switch( shterm_bli.event_num )
	{
	case SHBATTLOG_EVENT_CHG_INSERT_USB:
	case SHBATTLOG_EVENT_CHG_REMOVE_USB:
	case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE:
	case SHBATTLOG_EVENT_CHG_PUT_CRADLE:
	case SHBATTLOG_EVENT_CHG_START:
	case SHBATTLOG_EVENT_CHG_END:
	case SHBATTLOG_EVENT_CHG_ERROR:
	case SHBATTLOG_EVENT_CHG_COMP:
	case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
	case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
	case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
	case SHBATTLOG_EVENT_CHG_FAST_ST:
	case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
	case SHBATTLOG_EVENT_CHG_HOT_STOP_ST:
	case SHBATTLOG_EVENT_CHG_COLD_FAST_ST:
	case SHBATTLOG_EVENT_CHG_COLD_STOP_ST:
	case SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST:
	case SHBATTLOG_EVENT_INDICATER_0:
	case SHBATTLOG_EVENT_FATAL_BATT:
	case SHBATTLOG_EVENT_FGIC_EX10:
	case SHBATTLOG_EVENT_FGIC_EX20:
	case SHBATTLOG_EVENT_FGIC_EX30:
	case SHBATTLOG_EVENT_FGIC_EX40:
	case SHBATTLOG_EVENT_FGIC_EX50:
	case SHBATTLOG_EVENT_FGIC_EX60:
	case SHBATTLOG_EVENT_FGIC_EX70:
	case SHBATTLOG_EVENT_FGIC_EX80:
	case SHBATTLOG_EVENT_FGIC_EX90:
	case SHBATTLOG_EVENT_FGIC_EX100:
	case SHBATTLOG_EVENT_CHG_TYPE_SDP:
	case SHBATTLOG_EVENT_CHG_TYPE_CDP:
	case SHBATTLOG_EVENT_CHG_TYPE_DCP:
	case SHBATTLOG_EVENT_CHG_TYPE_HVDCP:
	case SHBATTLOG_EVENT_CHG_TYPE_OTHER:
		if( shbatt_api_get_battery_log_info(&bli) == SHBATT_RESULT_SUCCESS )
		{
			shterm_bli.bat_vol     = bli.bat_vol;
			shterm_bli.chg_vol     = bli.chg_vol;
			shterm_bli.chg_cur     = bli.chg_cur;
			shterm_bli.bat_temp    = bli.bat_temp;
			shterm_bli.cpu_temp    = bli.cpu_temp;
			shterm_bli.chg_temp    = bli.chg_temp;
			shterm_bli.cam_temp    = bli.cam_temp;
			shterm_bli.pmic_temp   = bli.pmic_temp;
			shterm_bli.pa_temp     = bli.pa_temp;
			shterm_bli.lcd_temp    = bli.lcd_temp;
			shterm_bli.avg_cur     = bli.avg_cur;
			shterm_bli.avg_vol     = bli.avg_vol;
			shterm_bli.latest_cur  = bli.latest_cur;
			shterm_bli.acc_cur     = bli.acc_cur;
			shterm_bli.vol_per     = bli.vol_per;
			shterm_bli.cur_dep_per = bli.cur_dep_per;
			shterm_bli.avg_dep_per = bli.avg_dep_per;
		}
		else
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
						"[P] %s() event:%d. shbatt_api_get_battery_log_info() failed.\n", __FUNCTION__,
						shterm_bli.event_num );
			// data error
			memset(&shterm_bli, 0, sizeof(shterm_bli));
			shterm_bli.event_num = evt;
		}
		break;

	default:
		memset(&shterm_bli, 0, sizeof(shterm_bli));
		shterm_bli.event_num = evt;
		break;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_vol    = %d\n",shterm_bli.chg_vol     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_cur    = %d\n",shterm_bli.chg_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:bat_temp   = %d\n",shterm_bli.bat_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cpu_temp   = %d\n",shterm_bli.cpu_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_temp   = %d\n",shterm_bli.chg_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cam_temp   = %d\n",shterm_bli.cam_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:pmic_temp  = %d\n",shterm_bli.pmic_temp   );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:pa_temp    = %d\n",shterm_bli.pa_temp     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:lcd_temp   = %d\n",shterm_bli.lcd_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_cur    = %d\n",shterm_bli.avg_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_vol    = %d\n",shterm_bli.avg_vol     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:latest_cur = %d\n",shterm_bli.latest_cur  );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:acc_cur    = %d\n",shterm_bli.acc_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:vol_per    = %d\n",shterm_bli.vol_per     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cur_dep_per= %d\n",shterm_bli.cur_dep_per );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_dep_per= %d\n",shterm_bli.avg_dep_per );
#ifdef CONFIG_SHTERM
	shterm_k_set_event(&shterm_bli);
#endif /* CONFIG_SHTERM */

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

/*from ioctl.*/
static int shbatt_drv_ioctl_cmd_initialize(
	struct file*				fi_p,
	unsigned					long arg
){
	int							ret;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	ret = shbatt_api_initialize();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_pull_usse_packet(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_usse_packet_t*		pkt_p;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	pkt_p = &shbatt_usse_pkt;

	if(copy_to_user((shbatt_usse_packet_t*)arg,pkt_p,sizeof(shbatt_usse_packet_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_done_usse_packet(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_usse_packet_t*		pkt_p;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	pkt_p = &shbatt_usse_pkt;

	if(copy_from_user(pkt_p,(shbatt_usse_packet_t*)arg,sizeof(shbatt_usse_packet_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_set_timer(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_poll_timer_info_t	pti;
	int							ret = 0;
	ktime_t						set_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(copy_from_user(&pti,(shbatt_poll_timer_info_t*)arg,sizeof(shbatt_poll_timer_info_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		switch( pti.ptt )
		{
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC:
			memset( &set_time, 0x00, sizeof( set_time ) );
			set_time.tv64 = SHBATT_TIMER_MSEC_TO_NSEC(pti.ms);
			shbatt_poll_timer[pti.ptt].prm = pti.prm;
			alarm_cancel( &(shbatt_poll_timer[pti.ptt].alm.alarm_timer) );
			alarm_start_relative(&(shbatt_poll_timer[pti.ptt].alm.alarm_timer),
						set_time);
			break;

		default:
			SHBATT_ERROR("%s : timer type invalid.\n",__FUNCTION__);
			ret = -EPERM;
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_clr_timer(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_poll_timer_info_t	pti;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(copy_from_user(&pti,(shbatt_poll_timer_info_t*)arg,sizeof(shbatt_poll_timer_info_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		switch( pti.ptt ) {
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC:
			alarm_cancel( &(shbatt_poll_timer[pti.ptt].alm.alarm_timer) );
			break;

		default:
			SHBATT_ERROR("%s : timer type invalid.\n",__FUNCTION__);
			ret = -EPERM;
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_get_boot_time(
	struct file*				fi_p,
	unsigned long				arg
){
	int							result = 0;
	struct timespec				ts;
	shbatt_boottime_info_t		time_t;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset( &ts, 0x00, sizeof( ts ) );
	get_monotonic_boottime( &ts );

	time_t.boot_sec	= ts.tv_sec;
	time_t.boot_nsec= ts.tv_nsec;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
			"[P] %s(): tv_sec=%lu tv_nsec=%lu sec=%lld nsec=%lld\n",__FUNCTION__,
			ts.tv_sec, ts.tv_nsec, time_t.boot_sec, time_t.boot_nsec );

	result = copy_to_user( (shbatt_boottime_info_t __user *)arg, &time_t, sizeof( shbatt_boottime_info_t ) );
	if( result != 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s(): copy_to_user failed.\n",__FUNCTION__);
		return -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_ioctl_cmd_get_shbatt_log_info(
	struct file*				fi_p,
	unsigned long				arg
){
	int							result = 0;
	shbatt_log_info_t info;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	result = sh_fg_get_log_info(&info);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT, "[P] %s(): result=%d\n",__FUNCTION__,result);

	result = copy_to_user( (shbatt_log_info_t __user *)arg, &info, sizeof( shbatt_log_info_t ) );
	if( result != 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s(): copy_to_user failed.\n",__FUNCTION__);
		return -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return 0;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p
){
	unsigned					int mask = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	if(atomic_read(&shbatt_usse_op_cnt) > 0)
	{
		atomic_dec(&shbatt_usse_op_cnt);

		mask = POLLIN;
	}
	else
	{
		poll_wait(fi_p,&shbatt_usse_wait,wait_p);
		complete(&shbatt_usse_cmp);
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return mask;
}

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg
){
	int							ret = -EPERM;

	SHBATT_TRACE("[S] %s() cmd=0x%x\n",__FUNCTION__,cmd);

	switch(cmd) {
	case SHBATT_DRV_IOCTL_CMD_INITIALIZE:
		ret = shbatt_drv_ioctl_cmd_initialize( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_PULL_USSE_PACKET:
		ret = shbatt_drv_ioctl_cmd_pull_usse_packet( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_DONE_USSE_PACKET:
		ret = shbatt_drv_ioctl_cmd_done_usse_packet( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_SET_TIMER:
		ret = shbatt_drv_ioctl_cmd_set_timer( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_CLR_TIMER:
		ret = shbatt_drv_ioctl_cmd_clr_timer( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_GET_BOOT_TIME:
		ret = shbatt_drv_ioctl_cmd_get_boot_time( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_GET_SHBATT_LOG_INFO:
		ret = shbatt_drv_ioctl_cmd_get_shbatt_log_info( fi_p,arg );
		break;

	default:
		SHBATT_ERROR( "[P] %s(): bad cmd. 0x%x\n", __FUNCTION__, cmd );
		ret = -EINVAL;
		break;
	}

	SHBATT_TRACE("[E] %s() ret=%d \n",__FUNCTION__,ret);

	return ret;
}

static int shbatt_drv_probe(
	struct platform_device*		dev_p
){
	int rc = 0;
	static struct shbatt_chip *chip;
	struct qpnp_vadc_chip *pm_vadc_dev;
	struct qpnp_vadc_chip *pmi_vadc_dev;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	if (of_find_property(dev_p->dev.of_node, "qcom,shbatt-pm-vadc", NULL)) {
		pm_vadc_dev = qpnp_get_vadc(&dev_p->dev, "shbatt-pm");
		if (IS_ERR(pm_vadc_dev)) {
		rc = PTR_ERR(pm_vadc_dev);
		if (rc != -EPROBE_DEFER)
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s() Couldn't get pm vadc rc =%d.\n", __FUNCTION__ ,rc );
			return rc;
		}
	}

	if (of_find_property(dev_p->dev.of_node, "qcom,shbatt-pmi-vadc", NULL)) {
		pmi_vadc_dev = qpnp_get_vadc(&dev_p->dev, "shbatt-pmi");
		if (IS_ERR(pmi_vadc_dev)) {
		rc = PTR_ERR(pmi_vadc_dev);
		if (rc != -EPROBE_DEFER)
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s() Couldn't get pmi vadc rc =%d.\n", __FUNCTION__ ,rc );
			return rc;
		}
	}

	chip = devm_kzalloc(&dev_p->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&dev_p->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pm_vadc_dev = pm_vadc_dev;
	chip->pmi_vadc_dev = pmi_vadc_dev;
	the_chip = chip;

	if(shbatt_drv_create_device() < 0)
	{
		SHBATT_ERROR("%s : create device failed.\n",__FUNCTION__);
		return -EPERM;
	}
	shbatt_task_is_initialized = true;

#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
	shbatt_api_write_traceability();
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_remove(
	struct platform_device*		dev_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	the_chip = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p
){
	int							alm_cnt;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_task_is_initialized = false;

	for( alm_cnt = 0; alm_cnt < NUM_SHBATT_POLL_TIMER_TYPE; alm_cnt++ )
	{
		switch(shbatt_poll_timer[alm_cnt].timer_type)
		{
		case SHBATT_TIMER_TYPE_HRTIMER:
			hrtimer_cancel(&(shbatt_poll_timer[alm_cnt].alm.hr_timer));
			break;

		case SHBATT_TIMER_TYPE_ALARMTIMER:
			alarm_cancel( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer) );
			break;

		default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s(): alarm type err. default=%d \n",__FUNCTION__, shbatt_poll_timer[alm_cnt].timer_type);
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static int shbatt_drv_resume(
	struct platform_device*		dev_p
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return 0;

}

void shbatt_api_cpu_clock_limit_lock(u32 level)
{
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	if(shbatt_limit_freq > level)
	{
		shbatt_limit_freq = level;
		sh_cpufreq_update_policy_try();
	}
}
EXPORT_SYMBOL_GPL(shbatt_api_cpu_clock_limit_lock);

void shbatt_api_cpu_clock_limit_unlock(void)
{
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	if(shbatt_limit_freq != UINT_MAX)
	{
		shbatt_limit_freq = UINT_MAX;
		sh_cpufreq_update_policy_try();
	}
}
EXPORT_SYMBOL_GPL(shbatt_api_cpu_clock_limit_unlock);

static int shbatt_limit_lock_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy*)data;
	SHBATT_TRACE("[S]%s event=%lu\n", __FUNCTION__, event);

	SHBATT_INFO("[P]%s shbatt_limit_freq=%u\n", __FUNCTION__, shbatt_limit_freq);

	switch(event)
	{
		case CPUFREQ_INCOMPATIBLE:
			cpufreq_verify_within_limits(policy, 0, shbatt_limit_freq);
			break;
		default:
			/* nothing todo.*/
			break;
	}

	SHBATT_TRACE("[E]%s \n", __FUNCTION__);

	return NOTIFY_OK;
}

static int __init shbatt_drv_module_init( void )
{
	int							alm_cnt;

	SHBATT_TRACE( "[S] %s \n", __FUNCTION__ );

	if(cpufreq_register_notifier(&shbatt_limit_lock_notifier, CPUFREQ_POLICY_NOTIFIER) != 0)
	{
		SHBATT_ERROR("cpufreq_register_notifier failed.\n");
	}
	else
	{
		SHBATT_TRACE("cpufreq_register_notifier succeeded.\n");
	}

	shbatt_task_workqueue_p = create_singlethread_workqueue("shbatt_task");

	mutex_init(&shbatt_task_lock);

	spin_lock_init(&shbatt_pkt_lock);

	wake_lock_init( &shbatt_wake_lock, WAKE_LOCK_SUSPEND, "shbatt_wake" );

	atomic_set( &shbatt_wake_lock_num, 0 );

	for( alm_cnt = 0; alm_cnt < NUM_SHBATT_POLL_TIMER_TYPE; alm_cnt++ )
	{
		switch(shbatt_poll_timer[alm_cnt].timer_type)
		{
		case SHBATT_TIMER_TYPE_HRTIMER:
			memset( &(shbatt_poll_timer[alm_cnt].alm.hr_timer), 0x00, sizeof(struct hrtimer) );
			hrtimer_init( &(shbatt_poll_timer[alm_cnt].alm.hr_timer),
						shbatt_poll_timer[alm_cnt].alarm_type, HRTIMER_MODE_ABS );
			shbatt_poll_timer[alm_cnt].alm.hr_timer.function = shbatt_poll_timer[alm_cnt].cb_func.hrtimer_cb;
			break;

		case SHBATT_TIMER_TYPE_ALARMTIMER:
			memset( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer), 0x00, sizeof(struct alarm) );
			alarm_init( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer),
						shbatt_poll_timer[alm_cnt].alarm_type,
						shbatt_poll_timer[alm_cnt].cb_func.alarm_cb );
			break;

		default:
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s(): alarm type err. default=%d \n",__FUNCTION__, shbatt_poll_timer[alm_cnt].timer_type);
			break;
		}
	}

	platform_driver_register( &shbatt_platform_driver );

	/* wake_lock */
	memset( &shbatt_lock_time, 0x00, sizeof(shbatt_lock_time) );
	memset( &shbatt_unlock_time, 0x00, sizeof(shbatt_unlock_time) );
	memset( &shbatt_lock_func, 0x00, sizeof(shbatt_lock_func) );

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void __exit shbatt_drv_module_exit( void )
{

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	platform_driver_unregister(&shbatt_platform_driver);

	if(cpufreq_unregister_notifier(&shbatt_limit_lock_notifier, CPUFREQ_POLICY_NOTIFIER) != 0)
		pr_err("%s: cpufreq_unregister_notifier failed.\n", __func__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
}

bool is_shbatt_prs_launched( void ) {
	return shbatt_task_is_initialized;
}

module_init(shbatt_drv_module_init);
module_exit(shbatt_drv_module_exit);

MODULE_DESCRIPTION("SH Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
