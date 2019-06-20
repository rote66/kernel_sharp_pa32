/* drivers/sharp/shtps/sy3000/shtps_rmi_ldm.c
 *
 * Copyright (c) 2015, Sharp. All rights reserved.
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
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <sharp/shtps_dev.h>
#include <sharp/sh_smem.h>

#include "shtps_rmi.h"
#include "shtps_rmi_sub.h"
#include "shtps_fwctl.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"
#include "shtps_filter.h"

/* -----------------------------------------------------------------------------------
 */
dev_t 					shtpsif_devid;
struct class*			shtpsif_class;
struct device*			shtpsif_device;
struct cdev 			shtpsif_cdev;
/* -----------------------------------------------------------------------------------
 */
/* for sysfs I/F */
#if defined(SHTPS_ENGINEER_BUILD_ENABLE)
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUGO | S_IWUSR | S_IWGRP), show_func, store_func)
#else
	#define SHTPSIF_DEFINE(name, show_func, store_func) \
	static struct kobj_attribute shtpsif_##name = \
		__ATTR(name, (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP), show_func, store_func)
#endif /* SHTPS_ENGINEER_BUILD_ENABLE */

#define SHTPSIF_SHOW_COMMON		shtpsif_show_common
#define SHTPSIF_STORE_COMMON	shtpsif_store_common

#define SHTPSIF_LOG_FUNC_CALL()								\
    if((gShtpsIfLog & 0x01) != 0){							\
        printk(KERN_DEBUG TPS_ID1" %s()\n", __func__);	\
    }

int gShtpsIfLog = 0;

static char shtps_fwupdate_buffer[SHTPS_FWDATA_BLOCK_SIZE_MAX];
static int shtps_fwupdate_datasize = 0;

static unsigned short shtpsif_reg_read_addr = 0;
static unsigned char shtpsif_reg_read_size = 1;

#define SHTPSIF_REG_READ_SIZE_MAX	100
#define SHTPSIF_REG_WRITE_SIZE_MAX	100

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CREATE_KOBJ_ENABLE)
static ssize_t store_hover_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;
		int on;
		sscanf(buf,"%d", &on);

		shtps_mutex_lock_ctrl();

		if(on == 0){
			shtps_set_hover_detect_disable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 1);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}else{
			shtps_set_hover_detect_enable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 0);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}

		shtps_mutex_unlock_ctrl();
	}
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	return count;
}

static ssize_t show_hover_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;
		int data;

		shtps_mutex_lock_ctrl();
		data = shtps_fwctl_get_hover_status(ts);
		shtps_mutex_unlock_ctrl();

		return snprintf(buf, PAGE_SIZE, "Hover Detect : %s\n", data == 0x00 ? "OFF" : "ON");
	}
	#else /* SHTPS_HOVER_DETECT_ENABLE */
		return snprintf(buf, PAGE_SIZE, "Hover not support\n");
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
}

static struct kobj_attribute hover_enable = 
	__ATTR(hover_enable, S_IRUSR | S_IWUSR, show_hover_enable, store_hover_enable);

static ssize_t show_grip_state(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	#if defined(SHTPS_LPWG_MODE_ENABLE) && defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE)
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;
		return snprintf(buf, PAGE_SIZE, "grip state = %s\n", ts->lpwg.grip_state  == 0x00 ? "OFF" : "ON");
	#else
		return snprintf(buf, PAGE_SIZE, "Not supported\n");
	#endif /* defined(SHTPS_LPWG_MODE_ENABLE) && defined(SHTPS_LPWG_GRIP_SUPPORT_ENABLE) */
}

static ssize_t store_grip_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int on;
	sscanf(buf,"%d", &on);
	
	msm_tps_set_grip_state(on);
	
	return count;
}

static struct kobj_attribute grip_state = 
	__ATTR(grip_state, S_IRUSR | S_IWUSR, show_grip_state, store_grip_state);

static ssize_t show_cover_state(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	#if defined(SHTPS_COVER_ENABLE)
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;
		return snprintf(buf, PAGE_SIZE, "cover state = %s\n", ts->cover_state  == 0x00 ? "OFF" : "ON");
	#else
		return snprintf(buf, PAGE_SIZE, "Not supported\n");
	#endif /* SHTPS_COVER_ENABLE */
}

static ssize_t store_cover_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int on;
	sscanf(buf,"%d", &on);
	
	msm_tps_set_cover_state(on);
	
	return count;
}

static struct kobj_attribute cover_state = 
	__ATTR(cover_state, S_IRUSR | S_IWUSR, show_cover_state, store_cover_state);

#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static int shtps_ioctl_set_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg);
#endif
#if defined(SHTPS_LPWG_MODE_ENABLE)
static int shtps_ioctl_lpwg_enable(struct shtps_rmi_spi *ts, unsigned long arg);
#endif /* SHTPS_LPWG_MODE_ENABLE */
static int shtps_ioctl_hover_enable(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);
static int shtps_ioctl_set_lcd_bright_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg);
#if defined(SHTPS_GLOVE_DETECT_ENABLE)
static int shtps_ioctl_set_glove_mode(struct shtps_rmi_spi *ts, unsigned long arg);
#endif /* SHTPS_GLOVE_DETECT_ENABLE */
#if defined(SHTPS_CTRL_FW_REPORT_RATE)
static int shtps_ioctl_set_high_report_mode(struct shtps_rmi_spi *ts, unsigned long arg);
#endif /* #if defined(SHTPS_CTRL_FW_REPORT_RATE) */

static ssize_t show_suspend_spi_test_state(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "suspend spi test cmd\n"
									"  1: set suspend state\n"
									"  2: clr suspend state\n"
									"  3: [test request] grip\n"
									"  4: [test request] charger\n"
									"  5: [test request] set sleep\n"
									"  6: [test request] set lpwg\n"
									"  7: [test request] set hover\n"
									"  8: [test request] set lpmode\n"
									"  9: [test request] set continuous lpmode\n"
									" 10: [test request] set lcd bright lpmode\n"
									" 11: [test request] tps open\n"
									" 12: [test request] tps enable\n"
									" 13: [test request] tps close\n"
									" 14: [test request] cover\n"
									" 15: [test request] glove\n"
									" 16: [test request] high report\n"
									);
}

static ssize_t store_suspend_spi_test_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int test_no;
	int param;
	
	sscanf(buf,"%d %d", &test_no, &param);
	
	switch(test_no){
		case 1: shtps_set_suspend_state(ts);
			break;
		case 2: shtps_clr_suspend_state(ts);
			break;
		case 3: msm_tps_set_grip_state(param);
			break;
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		case 4: shtps_ioctl_set_charger_armor(ts, param);
			break;
#endif
		case 5: msm_tps_setsleep(param);
			break;
#if defined(SHTPS_LPWG_MODE_ENABLE)
		case 6: shtps_ioctl_lpwg_enable(ts, param);
			break;
#endif /* SHTPS_LPWG_MODE_ENABLE */
		case 7: shtps_ioctl_hover_enable(ts, param);
			break;
		case 8: shtps_ioctl_set_low_power_mode(ts, param);
			break;
		case 9: shtps_ioctl_set_continuous_low_power_mode(ts, param);
			break;
		case 10: shtps_ioctl_set_lcd_bright_low_power_mode(ts, param);
			break;
		case 11: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_OPEN);
			break;
		case 12: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);
			break;
		case 13: shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_CLOSE);
			break;
		case 14: msm_tps_set_cover_state(param);
			break;
#if defined(SHTPS_GLOVE_DETECT_ENABLE)
		case 15: shtps_ioctl_set_glove_mode(ts, param);
			break;
#endif /* SHTPS_GLOVE_DETECT_ENABLE */
#if defined(SHTPS_CTRL_FW_REPORT_RATE)
		case 16: shtps_ioctl_set_high_report_mode(ts, param);
			break;
#endif /* #if defined(SHTPS_CTRL_FW_REPORT_RATE) */
		default:
			break;
	}
	
	return count;
}

static struct kobj_attribute suspend_spi_test = 
	__ATTR(suspend_spi_test, S_IRUSR | S_IWUSR, show_suspend_spi_test_state, store_suspend_spi_test_state);

#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

static struct attribute *attrs_ctrl[] = {
	&hover_enable.attr,
	&grip_state.attr,
	&cover_state.attr,

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		&suspend_spi_test.attr,
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	NULL
};

static struct attribute_group attr_grp_ctrl = {
	.name = "ctrl",
	.attrs = attrs_ctrl,
};
#endif /* SHTPS_CREATE_KOBJ_ENABLE */

void shtps_init_debugfs(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_CREATE_KOBJ_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(ts->kobj != NULL){
			if(sysfs_create_group(ts->kobj, &attr_grp_ctrl)){
				SHTPS_LOG_ERR_PRINT("kobj create failed : attr_grp_ctrl\n");
			}
		}
	#endif /* SHTPS_CREATE_KOBJ_ENABLE */
}

void shtps_deinit_debugfs(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_CREATE_KOBJ_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(ts->kobj != NULL){
			sysfs_remove_group(ts->kobj, &attr_grp_ctrl);
			kobject_put(ts->kobj);
		}
	#endif /* SHTPS_CREATE_KOBJ_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_copy_to_user(u8 *krnl, unsigned long arg, int size, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1){
		if(copy_to_user(compat_ptr(arg), krnl, size)){
			return -EFAULT;
		}
		return 0;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		if(copy_to_user((u8*)arg, krnl, size)){
			return -EFAULT;
		}
		return 0;
	}
}

static int shtps_copy_from_user(u8 *krnl, unsigned long arg, int size, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1){
		if(copy_from_user(krnl, compat_ptr(arg), size)){
			return -EFAULT;
		}
		return 0;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		if(copy_from_user(krnl, (u8*)arg, size)){
			return -EFAULT;
		}
		return 0;
	}
}

static int shtps_bl_info_copy_to_user(struct shtps_bootloader_info *krnl, unsigned long arg, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1){
		int err = 0;
		struct shtps_compat_bootloader_info *usr = 
			(struct shtps_compat_bootloader_info __user *) arg;
		
		err |= put_user(krnl->block_size, &usr->block_size);
		err |= put_user(krnl->program_block_num, &usr->program_block_num);
		err |= put_user(krnl->config_block_num, &usr->config_block_num);
		
		if(err){
			SHTPS_LOG_ERR_PRINT("shtps_bl_info_copy_to_user error[%d]",err);
			return -EFAULT;
		}
		
		return 0;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		int err = 0;
		struct shtps_bootloader_info *usr = 
			(struct shtps_bootloader_info __user *) arg;
		
		err |= put_user(krnl->block_size, &usr->block_size);
		err |= put_user(krnl->program_block_num, &usr->program_block_num);
		err |= put_user(krnl->config_block_num, &usr->config_block_num);
		
		if(err){
			SHTPS_LOG_ERR_PRINT("shtps_bl_info_copy_to_user error[%d]",err);
			return -EFAULT;
		}
		
		return 0;
	}
}


static int shtps_touch_info_copy_to_user(struct shtps_touch_info *krnl, unsigned long arg, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1)
	{
		int i;
		int err = 0;
		struct shtps_compat_touch_info *usr = 
			(struct shtps_compat_touch_info __user *) arg;

		err |= put_user(krnl->gs1, &usr->gs1);
		err |= put_user(krnl->gs2, &usr->gs2);
		err |= put_user(krnl->flick_x, &usr->flick_x);
		err |= put_user(krnl->flick_y, &usr->flick_y);
		err |= put_user(krnl->flick_time, &usr->flick_time);
		err |= put_user(krnl->finger_num, &usr->finger_num);
		
		for(i = 0;i < SHTPS_FINGER_MAX;i++){
			err |= put_user(krnl->fingers[i].x, &usr->fingers[i].x);
			err |= put_user(krnl->fingers[i].y, &usr->fingers[i].y);
			err |= put_user(krnl->fingers[i].state, &usr->fingers[i].state);
			err |= put_user(krnl->fingers[i].wx, &usr->fingers[i].wx);
			err |= put_user(krnl->fingers[i].wy, &usr->fingers[i].wy);
			err |= put_user(krnl->fingers[i].z, &usr->fingers[i].z);
		}
		
		if(err){
			return -EFAULT;
		}
		
		return 0;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		int i;
		int err = 0;
		struct shtps_touch_info *usr = 
			(struct shtps_touch_info __user *) arg;

		err |= put_user(krnl->gs1, &usr->gs1);
		err |= put_user(krnl->gs2, &usr->gs2);
		err |= put_user(krnl->flick_x, &usr->flick_x);
		err |= put_user(krnl->flick_y, &usr->flick_y);
		err |= put_user(krnl->flick_time, &usr->flick_time);
		err |= put_user(krnl->finger_num, &usr->finger_num);
		
		for(i = 0;i < SHTPS_FINGER_MAX;i++){
			err |= put_user(krnl->fingers[i].x, &usr->fingers[i].x);
			err |= put_user(krnl->fingers[i].y, &usr->fingers[i].y);
			err |= put_user(krnl->fingers[i].state, &usr->fingers[i].state);
			err |= put_user(krnl->fingers[i].wx, &usr->fingers[i].wx);
			err |= put_user(krnl->fingers[i].wy, &usr->fingers[i].wy);
			err |= put_user(krnl->fingers[i].z, &usr->fingers[i].z);
		}
		
		if(err){
			return -EFAULT;
		}
		
		return 0;
	}
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static int shtps_touchkey_info_copy_to_user(struct shtps_touch_key_info *krnl, unsigned long arg, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1)
	{
		int err = 0;
		struct shtps_compat_touch_key_info *usr = 
			(struct shtps_compat_touch_key_info __user *) arg;

		unsigned char		menu_key_state;
		unsigned char		home_key_state;
		unsigned char		back_key_state;
		unsigned char		down_key_state;
		unsigned char		up_key_state;


		err |= put_user(krnl->menu_key_state, &usr->menu_key_state);
		err |= put_user(krnl->home_key_state, &usr->home_key_state);
		err |= put_user(krnl->back_key_state, &usr->back_key_state);
		err |= put_user(krnl->down_key_state, &usr->down_key_state);
		err |= put_user(krnl->up_key_state, &usr->up_key_state);
		
		if(err){
			return -EFAULT;
		}
		
		return 0;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		int err = 0;
		struct shtps_touch_key_info *usr = 
			(struct shtps_touch_key_info __user *) arg;

		unsigned char		menu_key_state;
		unsigned char		home_key_state;
		unsigned char		back_key_state;
		unsigned char		down_key_state;
		unsigned char		up_key_state;


		err |= put_user(krnl->menu_key_state, &usr->menu_key_state);
		err |= put_user(krnl->home_key_state, &usr->home_key_state);
		err |= put_user(krnl->back_key_state, &usr->back_key_state);
		err |= put_user(krnl->down_key_state, &usr->down_key_state);
		err |= put_user(krnl->up_key_state, &usr->up_key_state);
		
		if(err){
			return -EFAULT;
		}
		
		return 0;
	}
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static int shtps_io_ctl_param_copy_from_user(struct shtps_ioctl_param *krnl, unsigned long arg, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1)
	{
		int err = 0;
		int size = 0;
		struct shtps_compat_ioctl_param *usr = 
			(struct shtps_compat_ioctl_param __user *) arg;

		if(!usr || usr->size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
			return -EINVAL;
		}
		
		err |= get_user(krnl->size, &usr->size);
		if(!err){
			if(!usr->data){
				krnl->data = NULL;
			}else{
				if(krnl->size < 0){
					size = sizeof(int);

				}else{
					size = krnl->size;
				}
				krnl->data = (u8*)kmalloc(size, GFP_KERNEL);
				if(!krnl->data){
					SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
					return -ENOMEM;
				}

				if(copy_from_user(krnl->data, compat_ptr(usr->data), size)){
					SHTPS_LOG_ERR_PRINT("[%s] copy_from_user error\n", __func__);
					kfree(krnl->data);
					return -EFAULT;
				}
			}
		}

		return err;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		int err = 0;
		int size = 0;
		struct shtps_ioctl_param *usr = 
			(struct shtps_ioctl_param __user *) arg;

		if(!usr || usr->size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
			return -EINVAL;
		}
		
		err |= get_user(krnl->size, &usr->size);
		if(!err){
			if(!usr->data){
				krnl->data = NULL;
			}else{
				if(krnl->size < 0){
					size = sizeof(int);

				}else{
					size = krnl->size;
				}
				krnl->data = (u8*)kmalloc(size, GFP_KERNEL);
				if(!krnl->data){
					SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
					return -ENOMEM;
				}

				if(copy_from_user(krnl->data, usr->data, size)){
					SHTPS_LOG_ERR_PRINT("[%s] copy_from_user error\n", __func__);
					kfree(krnl->data);
					return -EFAULT;
				}
			}
		}

		return err;
	}
}

static int shtps_io_ctl_param_copy_to_user(struct shtps_ioctl_param *krnl, unsigned long arg, int isCompat)
{
#ifdef CONFIG_COMPAT
	if(isCompat == 1)
	{
		int err = 0;
		struct shtps_compat_ioctl_param *usr = 
			(struct shtps_compat_ioctl_param __user *) arg;
		
		if(krnl->data && usr->data){
			if(copy_to_user((u8 __user *)compat_ptr(usr->data), krnl->data, usr->size)){
				err = -EFAULT;
			}
		}
		
		return err;
	}
	else
#endif /* CONFIG_COMPAT */
	{
		int err = 0;
		struct shtps_ioctl_param *usr = 
			(struct shtps_ioctl_param __user *) arg;
		
		if(krnl->data && usr->data){
			if(copy_to_user(usr->data, krnl->data, usr->size)){
				err = -EFAULT;
			}
		}
		
		return err;
	}
}

static void shtps_io_ctl_param_release(struct shtps_ioctl_param *krnl)
{
	if(krnl->data){
		kfree(krnl->data);
	}
}

static int shtps_ioctl_enable(struct shtps_rmi_spi *ts)
{
	int ret;

	SHTPS_LOG_FUNC_CALL();
	ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);

	return ret;
}

static int shtps_ioctl_disable(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);

	return 0;
}

static int shtps_ioctl_reset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPS_LOG_FUNC_CALL();
	if(power == 0){
		shtps_irq_enable(ts);
	}

	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_softreset(struct shtps_rmi_spi *ts)
{
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPS_LOG_FUNC_CALL();
	if(power == 0){
		shtps_irq_enable(ts);
	}

	SHTPS_LOG_ANALYSIS("SW Reset execute\n");
	if(shtps_fwctl_soft_reset(ts) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return 0;
}

static int shtps_ioctl_getver(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8  tps_stop_request = 0;
	u16 ver;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(ts->state_mgr.state == SHTPS_STATE_IDLE){
		tps_stop_request = 1;
	}

	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERR_PRINT("[%s] error - shtps_start()\n", __func__);
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	shtps_mutex_lock_ctrl();

	ver = shtps_fwver(ts);

	shtps_mutex_unlock_ctrl();

	if(tps_stop_request){
		shtps_shutdown(ts);
	}

	if(shtps_copy_to_user((u8*)&ver, arg, sizeof(ver), isCompat)){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] version = 0x%04x\n", __func__, ver);
	return 0;
}

static int shtps_ioctl_enter_bootloader(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int rc;
	struct shtps_bootloader_info info;

	if(0 == arg){
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();
	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);

	if(0 == rc){
		info.block_size        = shtps_fwctl_loader_get_blocksize(ts);
		info.program_block_num = shtps_fwctl_loader_get_firm_blocknum(ts);
		info.config_block_num  = shtps_fwctl_loader_get_config_blocknum(ts);

		if(shtps_bl_info_copy_to_user(&info, arg, isCompat)){
			return -EFAULT;
		}
	}

	if(rc){
		return -EFAULT;
	}

	return 0;
}



static int shtps_param_copy_from_user_alloc(struct shtps_ioctl_param *krnl, int size)
{
	int err = 0;
			
	if(size <= 0){
		krnl->data = (u8*)kmalloc(sizeof(int), GFP_KERNEL);
	}else{
		krnl->data = (u8*)kmalloc(size, GFP_KERNEL);
	}
	if(!krnl->data){
		SHTPS_LOG_ERR_PRINT("error: krnl->data[null]\n");
		return -ENOMEM;
	}
	return err;
}

static int shtps_ioctl_lockdown_bootloader(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int rc;
	struct shtps_ioctl_param param;
	SHTPS_LOG_FUNC_CALL();
	
	param.size = 0x30;
	rc = shtps_param_copy_from_user_alloc(&param, param.size);
	if(rc){
		return -EFAULT;
	}
	
	rc = shtps_copy_from_user(param.data, arg, param.size, isCompat);
	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error: copy_from_user\n", __func__);
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}
	
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] normal.\n", __func__);
	rc = shtps_lockdown_bootloader(ts, param.data);
#else
	rc = shtps_lockdown_bootloader(ts, data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	shtps_io_ctl_param_release(&param);

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}

	return 0;
}


static int shtps_ioctl_lockdown_bootloader_builtin(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int position;
	int rc;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */
	SHTPS_LOG_FUNC_CALL();
	
	position = (int)arg;
	
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
	if (position < 0 || position > shtps_fwsize_builtin(ts)) {
		SHTPS_LOG_DBG_PRINT("[%s] position erro \n", __func__);
		return -EINVAL;
	}

	fw_data = shtps_fwdata_builtin(ts);
	if(fw_data == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}
	rc = shtps_lockdown_bootloader(ts, (u8*)&(fw_data[position]));
#else
	rc = shtps_lockdown_bootloader(ts, data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}
	
	return 0;
}

static int shtps_ioctl_erase_flash(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_flash_erase(ts);
}

static int shtps_ioctl_write_image(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	
	int rc;
	struct shtps_ioctl_param param;
	SHTPS_LOG_FUNC_CALL();
	
	param.size = shtps_fwctl_loader_get_blocksize(ts);
	rc = shtps_param_copy_from_user_alloc(&param, param.size);
	if(rc){
		return -EFAULT;
	}
	
	rc = shtps_copy_from_user(param.data, arg, param.size, isCompat);
	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_from_user\n", __func__);
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}
	

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}
	SHTPS_LOG_FUNC_CALL();

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
	rc = shtps_flash_writeImage(ts, param.data);
#else
	rc = shtps_flash_writeImage(ts, param.data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	shtps_io_ctl_param_release(&param);

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_write_image_builtin(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int rc;
	int position;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */
	
	SHTPS_LOG_FUNC_CALL();
	
	position = (int)arg;
	
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);

	if (position < 0 || position > shtps_fwsize_builtin(ts)) {
		return -EINVAL;
	}

	fw_data = shtps_fwdata_builtin(ts);
	if(fw_data == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}
	rc = shtps_flash_writeImage(ts, (u8*)&(fw_data[position]));
#else
	rc = shtps_flash_writeImage(ts, data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}
	return 0;
}


static int shtps_ioctl_write_config(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int rc;
	unsigned long block_size = 0;
	unsigned long config_block_num = 0;
	
	struct shtps_ioctl_param param;
	SHTPS_LOG_FUNC_CALL();
	
	
	block_size = shtps_fwctl_loader_get_blocksize(ts);
	config_block_num = shtps_fwctl_loader_get_config_blocknum(ts);
	param.size = block_size * config_block_num;
	
	rc = shtps_param_copy_from_user_alloc(&param, param.size);
	if(rc){
		return -EFAULT;
	}
	
	rc = shtps_copy_from_user(param.data, arg, param.size, isCompat);
	if(rc){
		shtps_io_ctl_param_release(&param);
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_from_user\n", __func__);
		return -EFAULT;
	}

	if(param.size > SHTPS_FWDATA_BLOCK_SIZE_MAX){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
	rc = shtps_flash_writeConfig(ts, param.data);
#else
	rc = shtps_flash_writeConfig(ts, param.data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	shtps_io_ctl_param_release(&param);

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}

	return 0;

}

static int shtps_ioctl_write_config_builtin(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int position;
	int rc;
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	const unsigned char* fw_data = NULL;
#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */
	SHTPS_LOG_FUNC_CALL();
	
	position = (int)arg;
	
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
	if (position < 0 || position > shtps_fwsize_builtin(ts)) {
		SHTPS_LOG_ERR_PRINT("[%s] error - position\n", __func__);
		return -EINVAL;
	}

	fw_data = shtps_fwdata_builtin(ts);
	if(fw_data == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}
	rc = shtps_flash_writeConfig(ts, (u8*)&(fw_data[position]));
#else
	rc = shtps_flash_writeConfig(ts, data);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		SHTPS_LOG_ERR_PRINT("[%s] error\n", __func__);
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	SHTPS_LOG_FUNC_CALL();
	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	if(shtps_touch_info_copy_to_user(&info, arg, isCompat)){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_get_touchinfo_untrans(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	if(shtps_touch_info_copy_to_user(&ts->report_info, arg, isCompat)){
		return -EFAULT;
	}

	return 0;
}

static int shtps_ioctl_set_touchinfo_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.pos_mode = arg;
	return 0;
}

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static int shtps_ioctl_get_touchkeyinfo(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int ret;
	struct shtps_touch_key_info info;

	SHTPS_LOG_FUNC_CALL();
	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event_touchkey == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(ret != 0){
		info.up_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_UP) & 0x01);
		info.down_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01);

		ts->diag.event_touchkey = 0;

		if(shtps_touchkey_info_copy_to_user(&info, arg, isCompat)){
			return -EFAULT;
		}

		return 0;
	}

	return -EFAULT;
}
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static int shtps_ioctl_reg_read(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8 buf;
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}

	if(param.size == 1){
		if(M_READ_FUNC(ts->fwctl_p, param.data[0], &buf, 1)){
			shtps_io_ctl_param_release(&param);
			return -EFAULT;
		}
	}else if(param.size == 2){
		if(M_READ_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], &buf, 1)){
			shtps_io_ctl_param_release(&param);
			return -EFAULT;
		}
	}else{
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}
	
	param.data[0] = buf;
	if(shtps_io_ctl_param_copy_to_user(&param, arg, isCompat)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_allread(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8 dbbuf[0x100];
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}
	
	if(param.size < 1 || param.size < 0x100){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

	if(M_READ_FUNC(ts->fwctl_p, param.data[0] << 0x08, dbbuf, 0x100)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	memcpy(param.data, dbbuf, 0x100);
	if(shtps_io_ctl_param_copy_to_user(&param, arg, isCompat)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_write(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}

	if(param.size == 2){
		if(M_WRITE_FUNC(ts->fwctl_p, param.data[0], param.data[1])){
			shtps_io_ctl_param_release(&param);
			return -EFAULT;
		}
	}else if(param.size == 3){
		if(M_WRITE_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], param.data[2])){
			shtps_io_ctl_param_release(&param);
			return -EFAULT;
		}
	}else{
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_write_block(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}
	
	if(param.size < 3){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

	if(M_WRITE_BLOCK_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], &param.data[2], (param.size - 2))){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_read_block(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8 buf[0x100];
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}
	
	if(param.size < 2 || param.size > 0x100){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

	if(M_READ_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], buf, param.size)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	memcpy(param.data, buf, param.size);
	if(shtps_io_ctl_param_copy_to_user(&param, arg, isCompat)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_write_packet(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}

	if(param.size < 3){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}
	
	if(M_WRITE_PACKET_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], &param.data[2], (param.size - 2))){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_reg_read_packet(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8 buf[0x100];
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}

	if(param.size < 2 || param.size > 0x100){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}
	
	if(M_READ_PACKET_FUNC(ts->fwctl_p, param.data[0] << 0x08 | param.data[1], buf, param.size)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	memcpy(param.data, buf, param.size);
	if(shtps_io_ctl_param_copy_to_user(&param, arg, isCompat)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);
	return 0;
}

static int shtps_ioctl_tm_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_mode = SHTPS_FWTESTMODE_V01;
	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_tm_stop(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	if(shtps_copy_to_user((u8*)ts->diag.tm_data, arg,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2, isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_baseline_raw(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE_RAW);
	if(shtps_copy_to_user((u8*)ts->diag.tm_data, arg,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2, isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_frameline(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	if(shtps_copy_to_user((u8*)ts->diag.tm_data, arg,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2, isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_hybrid_adc(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_tmdata(ts, SHTPS_TMMODE_HYBRID_ADC);
	if(shtps_copy_to_user((u8*)ts->diag.tm_data, arg,
		(shtps_get_tm_rxsize(ts) + shtps_get_tm_txsize(ts)) * 4, isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_adc_range(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_read_tmdata(ts, SHTPS_TMMODE_ADC_RANGE);
	if(shtps_copy_to_user((u8*)ts->diag.tm_data, arg,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2, isCompat)){
		return -EFAULT;
	}
	return 0;
}

#if defined(SHTPS_GLOVE_DETECT_ENABLE)
static int shtps_ioctl_set_glove_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GLOVE_DETECT_ENABLE)
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETGLOVE, (u8)arg) == 0){
				shtps_ioctl_setglove_proc(ts, (u8)arg);
			}
		#else
		{
			int on = (int)arg;
			int ret = 0;

			shtps_mutex_lock_ctrl();

			if(on == 0){
				ts->glove_enable_state = 0;

				if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
					ret = shtps_set_glove_detect_disable(ts);
				}
			}else{
				ts->glove_enable_state = 1;

				if(ts->state_mgr.state == SHTPS_STATE_ACTIVE){
					ret = shtps_set_glove_detect_enable(ts);
				}
			}

			shtps_mutex_unlock_ctrl();

			return ret;
		}
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	#endif /* SHTPS_GLOVE_DETECT_ENABLE */

	return 0;
}
#endif /* SHTPS_GLOVE_DETECT_ENABLE */

static int shtps_ioctl_start_facetouchmode(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);
}

static int shtps_ioctl_stop_facetouchmode(struct shtps_rmi_spi *ts)
{
	int ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	ts->facetouch.wake_sig = 1;
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_facetouch_wakelock(ts, 0);	//8926
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return ret;
}

static int shtps_ioctl_poll_facetouchoff(struct shtps_rmi_spi *ts)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	int rc;

	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("wait for face touch off detect\n");
	rc = wait_event_interruptible(ts->facetouch.wait_off,
		(ts->facetouch.detect == 1) || (ts->facetouch.mode == 0) ||
		(ts->facetouch.wake_sig == 1));

	_log_msg_recv( LOGMSG_ID__DETECT_FACETOUCH, "%d|%d|%d",
						ts->facetouch.detect, ts->facetouch.mode, ts->facetouch.wake_sig);

	ts->facetouch.wake_sig = 0;

	if(ts->facetouch.detect){
		SHTPS_LOG_DBG_PRINT("face touch %s detect\n", (ts->facetouch.state == 0)? "off" : "on");
		rc = (ts->facetouch.state == 0)? TPSDEV_FACETOUCH_OFF_DETECT : TPSDEV_FACETOUCH_DETECT;
		ts->facetouch.detect = 0;
	}else{
		SHTPS_LOG_DBG_PRINT("wait for facetouch was cancelled\n");
		rc = TPSDEV_FACETOUCH_NOCHG;
	}

	return rc;
#else
	SHTPS_LOG_FUNC_CALL();
	return TPSDEV_FACETOUCH_NOCHG;
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
}

static int shtps_ioctl_get_fwstatus(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	unsigned char status;

	SHTPS_LOG_FUNC_CALL();
	shtps_fwctl_get_device_status(ts, &status);
	status = status & 0x0F;

	if(shtps_copy_to_user((u8*)&status, arg, sizeof(status), isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_get_fwdate(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u8 year;
	u8 month;
	unsigned short date;

	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();

	shtps_fwdate(ts, &year, &month);

	shtps_mutex_unlock_ctrl();

	date = (year << 0x08) | month;

	if(shtps_copy_to_user((u8*)&date, arg, sizeof(date), isCompat)){
		return -EFAULT;
	}
	return 0;
}

static int shtps_ioctl_calibration_param(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	struct shtps_ioctl_param param;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}

	if(param.size > sizeof(struct shtps_offset_info)){
		shtps_io_ctl_param_release(&param);
		return -EINVAL;
	}

	shtps_mutex_lock_ctrl();

	memcpy(ts->offset.base, param.data, sizeof(u16) * 5);
	ts->offset.diff[0] = (signed short)(param.data[11] << 0x08 | param.data[10]);
	ts->offset.diff[1] = (signed short)(param.data[13] << 0x08 | param.data[12]);
	ts->offset.diff[2] = (signed short)(param.data[15] << 0x08 | param.data[14]);
	ts->offset.diff[3] = (signed short)(param.data[17] << 0x08 | param.data[16]);
	ts->offset.diff[4] = (signed short)(param.data[19] << 0x08 | param.data[18]);
	ts->offset.diff[5] = (signed short)(param.data[21] << 0x08 | param.data[20]);
	ts->offset.diff[6] = (signed short)(param.data[23] << 0x08 | param.data[22]);
	ts->offset.diff[7] = (signed short)(param.data[25] << 0x08 | param.data[24]);
	ts->offset.diff[8] = (signed short)(param.data[27] << 0x08 | param.data[26]);
	ts->offset.diff[9] = (signed short)(param.data[29] << 0x08 | param.data[28]);
	ts->offset.diff[10]= (signed short)(param.data[31] << 0x08 | param.data[30]);
	ts->offset.diff[11]= (signed short)(param.data[33] << 0x08 | param.data[32]);
	
	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}

	shtps_mutex_unlock_ctrl();
	
	shtps_io_ctl_param_release(&param);

	shtps_report_stflib_kevt(ts, SHTPS_DEF_STFLIB_KEVT_CALIBRATION, 0);

	return 0;
}

static int shtps_ioctl_get_calibration_param(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	struct shtps_ioctl_param param;
	int i;
	char *data;
	char strbuf[20];
	int ret = 0;

	SHTPS_LOG_FUNC_CALL();
	if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
		return -EINVAL;
	}


	shtps_mutex_lock_ctrl();

	data = param.data;
	data[0] = '\0';

	for(i = 0; i < 5; i++){
		if(strlen(data) + 5 > param.size){
			SHTPS_LOG_ERR_PRINT("return buffer size is few.(%d byre)\n", param.size);
			ret = -ENOMEM;
			break;
		}

		if(i > 0){
			strcat(data, ",");
		}
		sprintf(strbuf, "%d", ts->offset.base[i]);
		strcat(data, strbuf);
	}

	for(i = 0; i < 12; i++){
		if(strlen(data) + 5 > param.size){
			SHTPS_LOG_ERR_PRINT("return buffer size is few.(%d byre)\n", param.size);
			ret = -ENOMEM;
			break;
		}

		strcat(data, ",");

		sprintf(strbuf, "%d", ts->offset.diff[i]);
		strcat(data, strbuf);
	}

	shtps_mutex_unlock_ctrl();


	if(shtps_io_ctl_param_copy_to_user(&param, arg, isCompat)){
		shtps_io_ctl_param_release(&param);
		return -EFAULT;
	}

	shtps_io_ctl_param_release(&param);

	return ret;
}


static int shtps_ioctl_calibration_pen_param(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	#if defined(SHTPS_PEN_DETECT_ENABLE)
		struct shtps_ioctl_param param;

		SHTPS_LOG_FUNC_CALL();
		if(0 == arg || 0 != shtps_io_ctl_param_copy_from_user(&param, arg, isCompat)){
			return -EINVAL;
		}

		if(param.size > sizeof(struct shtps_offset_info)){
			shtps_io_ctl_param_release(&param);
			return -EINVAL;
		}

		memcpy(ts->offset_pen.base, param.data, sizeof(u16) * 5);
		ts->offset_pen.diff[0] = (signed short)(param.data[11] << 0x08 | param.data[10]);
		ts->offset_pen.diff[1] = (signed short)(param.data[13] << 0x08 | param.data[12]);
		ts->offset_pen.diff[2] = (signed short)(param.data[15] << 0x08 | param.data[14]);
		ts->offset_pen.diff[3] = (signed short)(param.data[17] << 0x08 | param.data[16]);
		ts->offset_pen.diff[4] = (signed short)(param.data[19] << 0x08 | param.data[18]);
		ts->offset_pen.diff[5] = (signed short)(param.data[21] << 0x08 | param.data[20]);
		ts->offset_pen.diff[6] = (signed short)(param.data[23] << 0x08 | param.data[22]);
		ts->offset_pen.diff[7] = (signed short)(param.data[25] << 0x08 | param.data[24]);
		ts->offset_pen.diff[8] = (signed short)(param.data[27] << 0x08 | param.data[26]);
		ts->offset_pen.diff[9] = (signed short)(param.data[29] << 0x08 | param.data[28]);
		ts->offset_pen.diff[10]= (signed short)(param.data[31] << 0x08 | param.data[30]);
		ts->offset_pen.diff[11]= (signed short)(param.data[33] << 0x08 | param.data[32]);

		shtps_io_ctl_param_release(&param);

		if(ts->offset_pen.base[0] == 0){
			ts->offset_pen.enabled = 0;
		}else{
			ts->offset_pen.enabled = 1;
		}
	#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

	return 0;
}

static int shtps_ioctl_debug_reqevent(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	request_event(ts, (int)arg, 0);
	return 0;
}

static int shtps_ioctl_set_dragstep_x(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_DRAG_STEP_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if((int)arg != 0){
			SHTPS_DRAG_THRESH_VAL_X_1ST = (int)(arg & 0xFF);
			SHTPS_DRAG_THRESH_VAL_X_2ND = (int)((arg >> 0x08) & 0xFF);
			SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
			SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
		}
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return SHTPS_DRAG_THRESH_VAL_X_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI << 0x18) & 0xFF000000);
#else
	return 0;
#endif	/* SHTPS_DRAG_STEP_ENABLE */
}
static int shtps_ioctl_set_dragstep_y(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_DRAG_STEP_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if((int)arg != 0){
			SHTPS_DRAG_THRESH_VAL_Y_1ST = (int)(arg & 0xFF);
			SHTPS_DRAG_THRESH_VAL_Y_2ND = (int)((arg >> 0x08) & 0xFF);
			SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI = (int)((arg >> 0x10) & 0xFF);
			SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI = (int)((arg >> 0x18) & 0xFF);
		}
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return SHTPS_DRAG_THRESH_VAL_Y_1ST |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND << 0x08) & 0x0000FF00) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI << 0x10) & 0x00FF0000) |
		   ((SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI << 0x18) & 0xFF000000);
#else
	return 0;
#endif	/* SHTPS_DRAG_STEP_ENABLE */
}
static int shtps_ioctl_set_dragstep(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	return 0;
}

static int shtps_ioctl_set_fingerfixtime(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_DRAG_STEP_ENABLE )
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if((int)arg < 0xFFFF){
			SHTPS_DRAG_THRESH_RETURN_TIME =	(int)arg;
		}
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
		return SHTPS_DRAG_THRESH_RETURN_TIME;
#else
	return 0;
#endif	/* SHTPS_DRAG_STEP_ENABLE */
}

static int shtps_ioctl_rezero(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();

	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}

	shtps_mutex_unlock_ctrl();

	return 0;
}

static int shtps_ioctl_ack_facetouchoff(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	shtps_mutex_lock_ctrl();
	shtps_facetouch_wakelock(ts, 0);	//8926
	shtps_mutex_unlock_ctrl();
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */
	return 0;
}

static int shtps_ioctl_tmf05_start(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	ts->diag.tm_mode = shtps_fwctl_get_tm_mode(ts);

	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, arg)){
		return -EFAULT;
	}
	return 0;
}

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static int shtps_ioctl_log_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	gLogOutputEnable = (int)arg;
	return 0;
}
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

static int shtps_ioctl_set_invalid_area(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if 0 /* defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
	#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
		if( (0 <= (int)arg) && ((int)arg <= 1279) ){
			shtps_invalid_area[0].sx = 0;
			shtps_invalid_area[0].sy = (int)arg;
		}else if( (int)arg < 0 ){
			shtps_invalid_area[0].sx = -1;
		}
	#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return shtps_invalid_area[0].sy;

#else
	return -EFAULT;

#endif /* #if defined( SHTPS_VKEY_INVALID_AREA_ENABLE ) */
}

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static int shtps_ioctl_getver_builtin(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	u16 ver = shtps_fwver_builtin(ts);

	SHTPS_LOG_FUNC_CALL();
	if(shtps_fwdata_builtin(ts) == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}
	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	if(shtps_copy_to_user((u8*)&ver, arg, sizeof(ver), isCompat)){
		SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in version = 0x%04x\n", __func__, ver);
	return 0;
}
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

#if defined( SHTPS_SMEM_BASELINE_ENABLE )
static int shtps_ioctl_get_smem_baseline(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();

	SHTPS_LOG_FUNC_CALL();
	if(!smemdata){
		return -EFAULT;
	}
	
	if(shtps_copy_to_user((u8*)(smemdata->shdiag_TpsBaseLineTbl), arg,
		shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2, isCompat)){
		return -EFAULT;
	}
	return 0;
}
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */

static int shtps_ioctl_set_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE, (u8)arg) == 0){
			shtps_ioctl_setlpmode_proc(ts, (u8)arg);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, (int)arg);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_continuous_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE, (u8)arg) == 0){
			shtps_ioctl_setconlpmode_proc(ts, (u8)arg);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, (int)arg);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

static int shtps_ioctl_set_lcd_bright_low_power_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE, (u8)arg) == 0){
			shtps_ioctl_setlcdbrightlpmode_proc(ts, (u8)arg);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, (int)arg);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return 0;
}

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static int shtps_ioctl_set_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, (u8)arg) == 0){
			shtps_charger_armor_proc(ts, (u8)arg);
		}
	#else
		shtps_mutex_lock_ctrl();

		ret = shtps_filter_set_charger_armor(ts, (int)ts->state_mgr.state, (int)arg);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return ret;
}
#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static int shtps_ioctl_set_wireless_charger_armor(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 0;

	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, (u8)arg) == 0){
			shtps_charger_armor_proc(ts, (u8)arg);
		}
	#else
		shtps_mutex_lock_ctrl();

		ret = shtps_filter_set_charger_armor(ts, (int)ts->state_mgr.state, (int)arg);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return ret;
}
#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */

#if defined( SHTPS_LPWG_MODE_ENABLE )
static int shtps_ioctl_lpwg_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG, (u8)arg) == 0){
			shtps_ioctl_setlpwg_proc(ts, (u8)arg);
		}
	#else
	    shtps_mutex_lock_ctrl();
		ts->lpwg.lpwg_sweep_on_req_state = arg;
		ts->lpwg.notify_enable = 1;
	    SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_sweep_on_req_state = %d\n", ts->lpwg.lpwg_sweep_on_req_state);
		shtps_set_lpwg_sleep_check(ts);
	    shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return 0;
}
#endif /* SHTPS_LPWG_MODE_ENABLE */

static int shtps_ioctl_set_veilview_state(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_set_veilview_state(ts, arg);
}

static int shtps_ioctl_get_veilview_pattern(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	return shtps_get_veilview_pattern(ts);
}

static int shtps_ioctl_hover_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETHOVER, (u8)arg) == 0){
				shtps_ioctl_sethover_proc(ts, (u8)arg);
			}
		#else
		{
			int on = (int)arg;

			shtps_mutex_lock_ctrl();

			if(on == 0){
				shtps_set_hover_detect_disable(ts);

				#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
					shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 1);
				#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
			}else{
				shtps_set_hover_detect_enable(ts);

				#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
					shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 0);
				#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
			}

			shtps_mutex_unlock_ctrl();
		}
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	#endif /* SHTPS_HOVER_DETECT_ENABLE */

	return 0;
}

static int shtps_ioctl_baseline_offset_disable(struct shtps_rmi_spi *ts)
{
	int ret;
	
	SHTPS_LOG_FUNC_CALL();
	shtps_mutex_lock_ctrl();

	ret = shtps_baseline_offset_disable(ts);

	shtps_mutex_unlock_ctrl();

	return ret;
}

#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static int shtps_ioctl_check_crc_error(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR);

	return 0;
}
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

static int shtps_ioctl_set_narrow_frame_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE)
	{
		shtps_mutex_lock_ctrl();

		shtps_filter_edge_fail_touch_switch(ts, arg);

		shtps_mutex_unlock_ctrl();
	}
	#endif /* SHTPS_EDGE_FAIL_TOUCH_REJECTION_ENABLE */

	return 0;
}

static int shtps_ioctl_get_serial_number(struct shtps_rmi_spi *ts, unsigned long arg, int isCompat)
{
	int ret;
	u8 serial_number[SHTPS_READ_SERIAL_NUMBER_SIZE+1] = {0};

	if(0 == arg){
		SHTPS_LOG_ERR_PRINT("[%s] error - arg == 0\n", __func__);
		return -EINVAL;
	}

	shtps_mutex_lock_ctrl();

	ret = shtps_get_serial_number(ts, serial_number);

	shtps_mutex_unlock_ctrl();

	if(!ret){
		if(shtps_copy_to_user(serial_number, arg, sizeof(serial_number), isCompat)){
			SHTPS_LOG_ERR_PRINT("[%s] error - copy_to_user()\n", __func__);
			return -EFAULT;
		}
	}
	
	return (ret == 0)? 0 : -EFAULT;
}


static int shtps_ioctl_get_serial_number_size(struct shtps_rmi_spi *ts)
{
	return SHTPS_READ_SERIAL_NUMBER_SIZE;
}


#if defined(SHTPS_PEN_DETECT_ENABLE)
static int shtps_ioctl_set_pen_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	#if defined(SHTPS_FACTORY_MODE_ENABLE)
		return 0;
	#else
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETPEN, (u8)arg) == 0){
				shtps_ioctl_setpen_proc(ts, arg);
			}
		#else
		{
			int ret = -1;
			shtps_mutex_lock_ctrl();

			ret = shtps_set_pen_enable(ts, arg);

			shtps_mutex_unlock_ctrl();

			if(ret < 0){
				return ret;
			}
		}
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

		return 0;
	#endif /* SHTPS_FACTORY_MODE_ENABLE */
}

static int shtps_ioctl_get_pen_enable(struct shtps_rmi_spi *ts)
{
	return ts->pen_enable;
}
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

#if defined(SHTPS_LOW_REPORTRATE_MODE)
static int shtps_ioctl_set_low_reportrate_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	int ret = 1;
	
	if(SHTPS_LOW_REPORTRATE_MODE_ENABLE == 1){

		shtps_mutex_lock_ctrl();
		if(arg == 0){
			if( shtps_fwctl_initparam_reportrate(ts, 0) == 0){
				SHTPS_LOG_DBG_LOW_REPRATE_PRINT("low report rate -> OFF");
				ts->low_report_rate_mode_state = 0;
				ret = 0;
			}
		}else{
			if( shtps_fwctl_initparam_reportrate(ts, 1) == 0){
				SHTPS_LOG_DBG_LOW_REPRATE_PRINT("low report rate -> ON");
				ts->low_report_rate_mode_state = 1;
				ret = 0;
			}
		}

		if(ts->low_report_rate_mode_state == 0){
			shtps_report_stflib_kevt(ts, SHTPS_DEF_STFLIB_KEVT_SCANRATE_MODE, SHTPS_DEF_STFLIB_KEVT_SCANRATE_MODE_NORMAL);
		}else{
			shtps_report_stflib_kevt(ts, SHTPS_DEF_STFLIB_KEVT_SCANRATE_MODE, SHTPS_DEF_STFLIB_KEVT_SCANRATE_MODE_LOW);
		}

		shtps_mutex_unlock_ctrl();
	}
	
	return ret;
}
#endif /* #if defined(SHTPS_LOW_REPORTRATE_MODE) */

#if defined(SHTPS_LPWG_DOUBLE_TAP_ENABLE)
static int shtps_ioctl_lpwg_double_tap_enable(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG_DOUBLETAP, (u8)arg) == 0){
			shtps_ioctl_setlpwg_doubletap_proc(ts, (u8)arg);
		}
	#else
	    shtps_mutex_lock_ctrl();
		ts->lpwg.lpwg_double_tap_req_state = arg;
		ts->lpwg.notify_enable = 1;
	    SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_double_tap_req_state = %d\n", ts->lpwg.lpwg_double_tap_req_state);
		shtps_set_lpwg_sleep_check(ts);
	    shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return 0;
}
#endif /* SHTPS_LPWG_DOUBLE_TAP_ENABLE */

static int shtps_ioctl_get_double_tap_enable(struct shtps_rmi_spi *ts)
{
#if defined(SHTPS_LPWG_DOUBLE_TAP_ENABLE)
	return 1;
#else	/* SHTPS_LPWG_DOUBLE_TAP_ENABLE */
	return 0;
#endif	/* SHTPS_LPWG_DOUBLE_TAP_ENABLE */
}

#if defined(SHTPS_CTRL_FW_REPORT_RATE)
static int shtps_ioctl_set_high_report_mode(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();

	if(SHTPS_CTRL_FW_REPORT_RATE_ENABLE == 1){
		#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
			if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SET_HIGH_REPORT_MODE, (u8)arg) == 0){
				shtps_ioctl_set_high_report_mode_proc(ts, (u8)arg);
			}
		#else
		    shtps_mutex_lock_ctrl();
			if( arg == 0 ){
				ts->fw_report_rate_req_state = 0;
			}else{
				ts->fw_report_rate_req_state = 1;
			}
			shtps_set_fw_report_rate(ts);
		    SHTPS_LOG_DBG_PRINT(" [HIGH_REPORT_MODE] req_state = %d\n", ts->fw_report_rate_req_state);
		    shtps_mutex_unlock_ctrl();
		#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
	}

	return 0;
}
#endif /* #if defined(SHTPS_CTRL_FW_REPORT_RATE) */

static int shtps_ioctl_boot_fw_update(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_BOOT_FW_UPDATE);
	return 0;
}

static int shtps_ioctl_boot_fw_crc_check(struct shtps_rmi_spi *ts, unsigned long arg)
{
	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_BOOT_FW_CRC_CHECK);
	return 0;
}

/* -----------------------------------------------------------------------------------
 */
static int shtpsif_open(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_OPEN, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"Open(PID:%ld)\n", sys_getpid());
	return 0;
}

static int shtpsif_release(struct inode *inode, struct file *file)
{
	_log_msg_sync( LOGMSG_ID__DEVICE_RELEASE, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"Close(PID:%ld)\n", sys_getpid());
	return 0;
}

static ssize_t shtpsif_read(struct file *file, char *buf, size_t count, loff_t *pos)
{
	int i;
	int fingerMax;
	struct shtps_touch_info info;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_READ, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}
	fingerMax = shtps_get_fingermax(ts);

	wait_event_interruptible(ts->diag.wait, ts->diag.event == 1);

	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	if(copy_to_user((u8*)buf, (u8*)&info, sizeof(info))){
		_log_msg_sync( LOGMSG_ID__DEVICE_READ_FAIL, "");
		return -EFAULT;
	}

	ts->diag.event = 0;
	_log_msg_sync( LOGMSG_ID__DEVICE_READ_DONE, "");
	return sizeof(ts->report_info);
}

static unsigned int shtpsif_poll(struct file *file, poll_table *wait)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL, "");

	if(NULL == ts){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_FAIL, "");
		return POLLERR;
	}

	ret = wait_event_interruptible_timeout(ts->diag.wait,
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 != ret){
		_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "%d", POLLIN | POLLRDNORM);
		return POLLIN | POLLRDNORM;
	}

	_log_msg_sync( LOGMSG_ID__DEVICE_POLL_DONE, "0");
	return 0;
}

static long shtpsif_ioctl_cmd(struct shtps_rmi_spi *ts, unsigned int cmd, unsigned long arg, int isCompat)
{
	int	rc = 0;

	switch(cmd){
	case TPSDEV_ENABLE: 				rc = shtps_ioctl_enable(ts);					break;
	case TPSDEV_DISABLE:				rc = shtps_ioctl_disable(ts);					break;
	case TPSDEV_RESET:					rc = shtps_ioctl_reset(ts);						break;
	case TPSDEV_SOFT_RESET:				rc = shtps_ioctl_softreset(ts);					break;
	case TPSDEV_GET_FW_VERSION:			rc = shtps_ioctl_getver(ts, arg, isCompat);		break;
	case TPSDEV_ENTER_BOOTLOADER:		rc = shtps_ioctl_enter_bootloader(ts, arg, isCompat);	break;
	case TPSDEV_LOCKDOWN_BOOTLOADER:	rc = shtps_ioctl_lockdown_bootloader(ts, arg, isCompat);	break;
	case TPSDEV_LOCKDOWN_BOOTLOADER_BUILTIN:	rc = shtps_ioctl_lockdown_bootloader_builtin(ts, arg);	break;
	case TPSDEV_ERASE_FLASE:			rc = shtps_ioctl_erase_flash(ts, arg);			break;
	case TPSDEV_WRITE_IMAGE:			rc = shtps_ioctl_write_image(ts, arg, isCompat);	break;
	case TPSDEV_WRITE_IMAGE_BUILTIN:	rc = shtps_ioctl_write_image_builtin(ts, arg);	break;
	case TPSDEV_WRITE_CONFIG:			rc = shtps_ioctl_write_config(ts, arg, isCompat);	break;
	case TPSDEV_WRITE_CONFIG_BUILTIN:	rc = shtps_ioctl_write_config_builtin(ts, arg);	break;
	case TPSDEV_GET_TOUCHINFO:			rc = shtps_ioctl_get_touchinfo(ts, arg, isCompat);	break;
	case TPSDEV_GET_TOUCHINFO_UNTRANS:	rc = shtps_ioctl_get_touchinfo_untrans(ts, arg, isCompat);break;
	case TPSDEV_SET_TOUCHMONITOR_MODE:	rc = shtps_ioctl_set_touchinfo_mode(ts, arg);	break;
	case TPSDEV_READ_REG:				rc = shtps_ioctl_reg_read(ts, arg, isCompat);	break;
	case TPSDEV_READ_ALL_REG:			rc = shtps_ioctl_reg_allread(ts, arg, isCompat);	break;
	case TPSDEV_WRITE_REG:				rc = shtps_ioctl_reg_write(ts, arg, isCompat);	break;
	case TPSDEV_START_TM:				rc = shtps_ioctl_tm_start(ts, arg);				break;
	case TPSDEV_STOP_TM:				rc = shtps_ioctl_tm_stop(ts);					break;
	case TPSDEV_GET_BASELINE:			rc = shtps_ioctl_get_baseline(ts, arg, isCompat);	break;
	case TPSDEV_GET_FRAMELINE:			rc = shtps_ioctl_get_frameline(ts, arg, isCompat);	break;
	case TPSDEV_START_FACETOUCHMODE:	rc = shtps_ioctl_start_facetouchmode(ts);		break;
	case TPSDEV_STOP_FACETOUCHMODE:		rc = shtps_ioctl_stop_facetouchmode(ts);		break;
	case TPSDEV_POLL_FACETOUCHOFF:		rc = shtps_ioctl_poll_facetouchoff(ts);			break;
	case TPSDEV_GET_FWSTATUS:			rc = shtps_ioctl_get_fwstatus(ts, arg, isCompat);	break;
	case TPSDEV_GET_FWDATE:				rc = shtps_ioctl_get_fwdate(ts, arg, isCompat);	break;
	case TPSDEV_CALIBRATION_PARAM:		rc = shtps_ioctl_calibration_param(ts, arg, isCompat);	break;
	case TPSDEV_DEBUG_REQEVENT:			rc = shtps_ioctl_debug_reqevent(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP:			rc = shtps_ioctl_set_dragstep(ts, arg);			break;
	case TPSDEV_SET_FINGERFIXTIME:		rc = shtps_ioctl_set_fingerfixtime(ts, arg);	break;
	case TPSDEV_REZERO:					rc = shtps_ioctl_rezero(ts, arg);				break;
	case TPSDEV_ACK_FACETOUCHOFF:		rc = shtps_ioctl_ack_facetouchoff(ts, arg);		break;
	case TPSDEV_START_TM_F05:			rc = shtps_ioctl_tmf05_start(ts, arg);			break;
	case TPSDEV_SET_DRAGSTEP_X:			rc = shtps_ioctl_set_dragstep_x(ts, arg);		break;
	case TPSDEV_SET_DRAGSTEP_Y:			rc = shtps_ioctl_set_dragstep_y(ts, arg);		break;
#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
	case TPSDEV_LOGOUTPUT_ENABLE:		rc = shtps_ioctl_log_enable(ts, arg);			break;
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */
#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
	case TPSDEV_GET_TOUCHKEYINFO:		rc = shtps_ioctl_get_touchkeyinfo(ts, arg, isCompat);	break;
#endif /* #if defined( SHTPS_PHYSICAL_KEY_ENABLE ) */
#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	case TPSDEV_GET_FW_VERSION_BUILTIN:	rc = shtps_ioctl_getver_builtin(ts, arg, isCompat);	break;
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */
	case TPSDEV_SET_INVALID_AREA:		rc = shtps_ioctl_set_invalid_area(ts, arg);		break;
#if defined( SHTPS_SMEM_BASELINE_ENABLE )
	case TPSDEV_GET_SMEM_BASELINE:		rc = shtps_ioctl_get_smem_baseline(ts, arg, isCompat);	break;
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */
	case TPSDEV_SET_LOWPOWER_MODE:		rc = shtps_ioctl_set_low_power_mode(ts, arg);	break;
	case TPSDEV_SET_CONT_LOWPOWER_MODE:	rc = shtps_ioctl_set_continuous_low_power_mode(ts, arg); break;
	case TPSDEV_SET_LCD_LOWPOWER_MODE:	rc = shtps_ioctl_set_lcd_bright_low_power_mode(ts, arg);	break;
#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
	case TPSDEV_SET_CHARGER_ARMOR:		rc = shtps_ioctl_set_charger_armor(ts, arg); break;
	case TPSDEV_SET_WIRELESS_CHARGER_ARMOR:	rc = shtps_ioctl_set_wireless_charger_armor(ts, arg); break;
#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */
#if defined( SHTPS_LPWG_MODE_ENABLE )
	case TPSDEV_LPWG_ENABLE:			rc = shtps_ioctl_lpwg_enable(ts, arg); 			break;
#endif /* SHTPS_LPWG_MODE_ENABLE */
	case TPSDEV_SET_VEILVIEW_STATE:		rc = shtps_ioctl_set_veilview_state(ts, arg);	break;
	case TPSDEV_READ_REG_BLOCK:			rc = shtps_ioctl_reg_read_block(ts, arg, isCompat);	break;
	case TPSDEV_WRITE_REG_BLOCK:		rc = shtps_ioctl_reg_write_block(ts, arg, isCompat);	break;
	case TPSDEV_GET_VEILVIEW_PATTERN:	rc = shtps_ioctl_get_veilview_pattern(ts);		break;
	case TPSDEV_READ_REG_PACKET:		rc = shtps_ioctl_reg_read_packet(ts, arg, isCompat);	break;
	case TPSDEV_WRITE_REG_PACKET:		rc = shtps_ioctl_reg_write_packet(ts, arg, isCompat);	break;
	case TPSDEV_HOVER_ENABLE:			rc = shtps_ioctl_hover_enable(ts, arg);			break;
	case TPSDEV_GET_BASELINE_RAW:		rc = shtps_ioctl_get_baseline_raw(ts, arg, isCompat);	break;
	case TPSDEV_CALIBRATION_PEN_PARAM:	rc = shtps_ioctl_calibration_pen_param(ts, arg, isCompat);	break;
	case TPSDEV_BASELINE_OFFSET_DISABLE:rc = shtps_ioctl_baseline_offset_disable(ts); 	break;
#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
	case TPSDEV_CHECK_CRC_ERROR:		rc = shtps_ioctl_check_crc_error(ts); 			break;
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */
	case TPSDEV_SET_NARROW_FRAME_MODE:	rc = shtps_ioctl_set_narrow_frame_mode(ts, arg);	break;
#if defined(SHTPS_PEN_DETECT_ENABLE)
	case TPSDEV_SET_PEN_ENABLE:			rc = shtps_ioctl_set_pen_enable(ts, arg);		break;
	case TPSDEV_GET_PEN_ENABLE:			rc = shtps_ioctl_get_pen_enable(ts);			break;
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

#if defined(SHTPS_LOW_REPORTRATE_MODE)
	case TPSDEV_SET_LOW_REPORTRATE_MODE:	rc = shtps_ioctl_set_low_reportrate_mode(ts, arg);	break;
#endif /* #if defined(SHTPS_LOW_REPORTRATE_MODE) */
	case TPSDEV_GET_SERIAL_NUMBER:	rc = shtps_ioctl_get_serial_number(ts, arg, isCompat);	break;
	case TPSDEV_GET_SERIAL_NUMBER_SIZE:	rc = shtps_ioctl_get_serial_number_size(ts);	break;
	case TPSDEV_GET_HYBRID_ADC:			rc = shtps_ioctl_get_hybrid_adc(ts, arg, isCompat);	break;
	case TPSDEV_GET_ADC_RANGE:			rc = shtps_ioctl_get_adc_range(ts, arg, isCompat);	break;
#if defined(SHTPS_LPWG_DOUBLE_TAP_ENABLE)
	case TPSDEV_LPWG_DOUBLE_TAP_ENABLE:	rc = shtps_ioctl_lpwg_double_tap_enable(ts, arg); 	break;
#endif /* SHTPS_LPWG_DOUBLE_TAP_ENABLE */
#if defined(SHTPS_GLOVE_DETECT_ENABLE)
	case TPSDEV_SET_GLOVE_MODE:			rc = shtps_ioctl_set_glove_mode(ts, arg);	break;
#endif /* SHTPS_GLOVE_DETECT_ENABLE */
#if defined(SHTPS_CTRL_FW_REPORT_RATE)
	case TPSDEV_SET_HIGH_REPORT_MODE:	rc = shtps_ioctl_set_high_report_mode(ts, arg);	break;
#endif /* SHTPS_CTRL_FW_REPORT_RATE */
	case TPSDEV_BOOT_FW_UPDATE_REQ:	rc = shtps_ioctl_boot_fw_update(ts, arg);	break;
	case TPSDEV_BOOT_FW_UPDATE_CRC_ERROR_CHECK_REQ:	rc = shtps_ioctl_boot_fw_crc_check(ts, arg);	break;
	case TPSDEV_GET_DOUBLE_TAP_ENABLE:	rc = shtps_ioctl_get_double_tap_enable(ts);			break;
	case TPSDEV_GET_CALIBRATION_PARAM:	rc = shtps_ioctl_get_calibration_param(ts, arg, isCompat);	break;

	default:
		SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_ioctl switch(cmd) default\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static long shtpsif_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int isCompat = 0;

	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL, "%ld|0x%08X|0x%lX", sys_getpid(), cmd, arg);
	SHTPS_LOG_DBG_PRINT(TPS_ID1"ioctl(PID:%ld,CMD:0x%08X(%d),ARG:0x%lX)\n",
											sys_getpid(), cmd, cmd & 0xff, arg);

	if(ts == NULL){
		SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_ioctl ts == NULL\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		return -EFAULT;
	}

	rc = shtpsif_ioctl_cmd(ts, cmd, arg, isCompat);
	
	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_DONE, "%d", rc);
	return rc;
}

#ifdef CONFIG_COMPAT
static long shtpsif_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int	rc = 0;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int isCompat = 1;

	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL, "%ld|0x%08X|0x%lX", sys_getpid(), cmd, arg);
	SHTPS_LOG_DBG_PRINT(TPS_ID1"compat_ioctl(PID:%ld,CMD:0x%08X(%d),ARG:0x%lX)\n",
											sys_getpid(), cmd, cmd & 0xff, arg);

	if(ts == NULL){
		SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_compat_ioctl ts == NULL\n");
		_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_FAIL, "");
		return -EFAULT;
	}

	switch(cmd){
	case COMPAT_TPSDEV_ENTER_BOOTLOADER:		cmd = TPSDEV_ENTER_BOOTLOADER;		break;
	case COMPAT_TPSDEV_LOCKDOWN_BOOTLOADER:		cmd = TPSDEV_LOCKDOWN_BOOTLOADER;	break;
	case COMPAT_TPSDEV_WRITE_IMAGE:				cmd = TPSDEV_WRITE_IMAGE;			break;
	case COMPAT_TPSDEV_WRITE_CONFIG:			cmd = TPSDEV_WRITE_CONFIG;			break;
	case COMPAT_TPSDEV_READ_REG:				cmd = TPSDEV_READ_REG;				break;
	case COMPAT_TPSDEV_READ_ALL_REG:			cmd = TPSDEV_READ_ALL_REG;			break;
	case COMPAT_TPSDEV_WRITE_REG:				cmd = TPSDEV_WRITE_REG;				break;
	case COMPAT_TPSDEV_CALIBRATION_PARAM:		cmd = TPSDEV_CALIBRATION_PARAM;		break;
	case COMPAT_TPSDEV_GET_TOUCHKEYINFO:		cmd = TPSDEV_GET_TOUCHKEYINFO;		break;
	case COMPAT_TPSDEV_READ_REG_BLOCK:			cmd = TPSDEV_READ_REG_BLOCK;		break;
	case COMPAT_TPSDEV_WRITE_REG_BLOCK:			cmd = TPSDEV_WRITE_REG_BLOCK;		break;
	case COMPAT_TPSDEV_READ_REG_PACKET:			cmd = TPSDEV_READ_REG_PACKET;		break;
	case COMPAT_TPSDEV_WRITE_REG_PACKET:		cmd = TPSDEV_WRITE_REG_PACKET;		break;
	case COMPAT_TPSDEV_CALIBRATION_PEN_PARAM:	cmd = TPSDEV_CALIBRATION_PEN_PARAM;	break;
	case COMPAT_TPSDEV_GET_TOUCHINFO:			cmd = TPSDEV_GET_TOUCHINFO;			break;
	case COMPAT_TPSDEV_GET_TOUCHINFO_UNTRANS:	cmd = TPSDEV_GET_TOUCHINFO_UNTRANS;	break;
	case COMPAT_TPSDEV_GET_BASELINE:			cmd = TPSDEV_GET_BASELINE;			break;
	case COMPAT_TPSDEV_GET_FRAMELINE:			cmd = TPSDEV_GET_FRAMELINE;			break;
	case COMPAT_TPSDEV_GET_SMEM_BASELINE:		cmd = TPSDEV_GET_SMEM_BASELINE;		break;
	case COMPAT_TPSDEV_GET_BASELINE_RAW:		cmd = TPSDEV_GET_BASELINE_RAW;		break;
	case COMPAT_TPSDEV_GET_SERIAL_NUMBER:		cmd = TPSDEV_GET_SERIAL_NUMBER;		break;
	case COMPAT_TPSDEV_GET_HYBRID_ADC:			cmd = TPSDEV_GET_HYBRID_ADC;		break;
	case COMPAT_TPSDEV_GET_ADC_RANGE:			cmd = TPSDEV_GET_ADC_RANGE;			break;
	case COMPAT_TPSDEV_GET_CALIBRATION_PARAM:	cmd = TPSDEV_GET_CALIBRATION_PARAM;	break;
	}

	rc = shtpsif_ioctl_cmd(ts, cmd, arg, isCompat);
	
	_log_msg_sync( LOGMSG_ID__DEVICE_IOCTL_DONE, "%d", rc);
	return rc;
}
#endif /* CONFIG_COMPAT */

static const struct file_operations shtpsif_fileops = {
	.owner   = THIS_MODULE,
	.open    = shtpsif_open,
	.release = shtpsif_release,
	.read    = shtpsif_read,
	.poll    = shtpsif_poll,
	.unlocked_ioctl   = shtpsif_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =	shtpsif_compat_ioctl,
#endif /* CONFIG_COMPAT */
};

int __init shtpsif_init(void)
{
	int rc;

	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT, "");
	rc = alloc_chrdev_region(&shtpsif_devid, 0, 1, SH_TOUCH_IF_DEVNAME);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT(TPS_ID1":alloc_chrdev_region error\n");
		return rc;
	}

	shtpsif_class = class_create(THIS_MODULE, SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_class)) {
		rc = PTR_ERR(shtpsif_class);
		SHTPS_LOG_ERR_PRINT(TPS_ID1":class_create error\n");
		goto error_vid_class_create;
	}

	shtpsif_device = device_create(shtpsif_class, NULL,
								shtpsif_devid, &shtpsif_cdev,
								SH_TOUCH_IF_DEVNAME);
	if (IS_ERR(shtpsif_device)) {
		rc = PTR_ERR(shtpsif_device);
		SHTPS_LOG_ERR_PRINT(TPS_ID1":device_create error\n");
		goto error_vid_class_device_create;
	}

	cdev_init(&shtpsif_cdev, &shtpsif_fileops);
	shtpsif_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shtpsif_cdev, shtpsif_devid, 1);
	if(rc < 0){
		SHTPS_LOG_ERR_PRINT(TPS_ID1":cdev_add error\n");
		goto err_via_cdev_add;
	}

	SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_init() done\n");
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_DONE, "");

	return 0;

err_via_cdev_add:
	cdev_del(&shtpsif_cdev);
error_vid_class_device_create:
	class_destroy(shtpsif_class);
error_vid_class_create:
	unregister_chrdev_region(shtpsif_devid, 1);
	_log_msg_sync( LOGMSG_ID__DEVICE_INIT_FAIL, "");

	return rc;
}
module_init(shtpsif_init);

static void __exit shtpsif_exit(void)
{
	cdev_del(&shtpsif_cdev);
	class_destroy(shtpsif_class);
	unregister_chrdev_region(shtpsif_devid, 1);

	_log_msg_sync( LOGMSG_ID__DEVICE_EXIT_DONE, "");
	SHTPS_LOG_DBG_PRINT(TPS_ID1"shtpsif_exit() done\n");
}
module_exit(shtpsif_exit);

/* -----------------------------------------------------------------------------------
 */
static int shtps_numStrToList(
	const char	*numStr,		/* [I  ] num strings */
	int			*numList,		/* [I/O] num list */
	int			numListMaxSize	/* [I/ ] num list size */
)
{
	int			i;
	int			numListNum;
	int			isParam;
	char		buf[10];
	int			buf_current;
	int			num;
	int			rc;

	if((numStr == NULL) || (numList == NULL) || (numListMaxSize < 1)){
		return 0;
	}

	numListNum = 0;
	isParam = 0;
	buf_current = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if((numStr[i] == '\0') || (numStr[i] == '\n') || (numStr[i] == ',') || (numStr[i] == ' ') || (numStr[i] == '\t')){
			if(isParam == 1){
				buf[buf_current] = '\0';
				rc = kstrtoint(buf, 0, &num);
				if(rc == 0){
					numList[numListNum] = num;
				}
				else{
					/* Conversion failure */
					return 0;
				}
				numListNum++;
			}
			isParam = 0;
			if(numListNum >= numListMaxSize){
				break;
			}
			if( (numStr[i] == '\0') ){
				break;
			}
		}
		else{
			if(isParam == 0){
				isParam = 1;
				buf_current = 0;
			}
			buf[buf_current] = numStr[i];
			buf_current++;
			if(buf_current >= sizeof(buf)){
				/* Conversion failure */
				return 0;
			}
		}
	}

	return numListNum;
}
int shtpsif_get_arguments(
	char	*argStr,		/* [I/O] arguments strings (processed in function) */
	char	**argList,		/* [I/O] arguments pointer output buffer */
	int		argListMaxSize	/* [I/ ] arguments list size */
)
{
	int i;
	int argListNum = 0;
	int isParam;

	if((argStr == NULL) || (argList == NULL) || (argListMaxSize < 1)){
		return 0;
	}

	isParam = 0;

	for(i = 0; i < PAGE_SIZE; i++){
		if( (argStr[i] == '\0') ){
			if(isParam == 1){
				argListNum++;
			}
			break;
		}
		else if( (argStr[i] == '\n') || (argStr[i] == ',') || (argStr[i] == ' ') ){
			argStr[i] = '\0';
			if(isParam == 1){
				argListNum++;
			}
			isParam = 0;
			if(argListNum >= argListMaxSize){
				break;
			}
			continue;
		}
		else{
			if(isParam == 0){
				isParam = 1;
				argList[argListNum] = &argStr[i];
			}
		}
	}

	return argListNum;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_common(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return -EINVAL;
}

static ssize_t shtpsif_store_common(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}
/* -----------------------------------------------------------------------------------
 */
static ssize_t shtpsif_show_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gShtpsIfLog);
}
static ssize_t shtpsif_store_shtpsiflog(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc >= 1){
		gShtpsIfLog = argv[0];
	}
	else{
		return -EINVAL;
	}

	return count;
}
SHTPSIF_DEFINE(shtpsiflog, shtpsif_show_shtpsiflog, shtpsif_store_shtpsiflog);

static ssize_t shtpsif_store_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_ENABLE);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(enable, SHTPSIF_SHOW_COMMON, shtpsif_store_enable);

static ssize_t shtpsif_store_disable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_DISABLE);

	return count;
}
SHTPSIF_DEFINE(disable, SHTPSIF_SHOW_COMMON, shtpsif_store_disable);

static ssize_t shtpsif_store_reset(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPSIF_LOG_FUNC_CALL();

	if(power == 0){
		shtps_irq_enable(ts);
	}

	shtps_reset(ts);
	msleep(SHTPS_HWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return count;
}
SHTPSIF_DEFINE(reset, SHTPSIF_SHOW_COMMON, shtpsif_store_reset);

static ssize_t shtpsif_store_softreset(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int power = (ts->state_mgr.state == SHTPS_STATE_IDLE)? 0 : 1;

	SHTPSIF_LOG_FUNC_CALL();

	if(power == 0){
		shtps_irq_enable(ts);
	}

	SHTPS_LOG_ANALYSIS("SW Reset execute\n");
	if(shtps_fwctl_soft_reset(ts) != 0){
		return -EFAULT;
	}
	msleep(SHTPS_SWRESET_WAIT_MS);

	if(power == 0){
		shtps_irq_disable(ts);
	}

	return count;
}
SHTPSIF_DEFINE(softreset, SHTPSIF_SHOW_COMMON, shtpsif_store_softreset);

static ssize_t shtpsif_show_fwver(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8  tps_stop_request = 0;
	u16 ver;

	SHTPSIF_LOG_FUNC_CALL();

	if(ts->state_mgr.state == SHTPS_STATE_IDLE){
		tps_stop_request = 1;
	}

	if(0 != shtps_start(ts)){
		SHTPS_LOG_ERR_PRINT("[%s] error - shtps_start()\n", __func__);
		return -EFAULT;
	}
	shtps_wait_startup(ts);

	shtps_mutex_lock_ctrl();

	ver = shtps_fwver(ts);

	shtps_mutex_unlock_ctrl();

	if(tps_stop_request){
		shtps_shutdown(ts);
	}

	SHTPS_LOG_DBG_PRINT("[%s] version = 0x%04x\n", __func__, ver);

	return snprintf(buf, PAGE_SIZE, "%04X\n", ver);
}
SHTPSIF_DEFINE(fwver, shtpsif_show_fwver, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", shtps_fwupdate_datasize);
}
static ssize_t shtpsif_store_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	if((shtps_fwupdate_datasize + count) > sizeof(shtps_fwupdate_buffer)){
		return -ENOMEM;
	}
	else{
		memcpy(&shtps_fwupdate_buffer[shtps_fwupdate_datasize], buf, count);
		shtps_fwupdate_datasize += count;
	}

	return count;
}
SHTPSIF_DEFINE(fwupdate_buffer, shtpsif_show_fwupdate_buffer, shtpsif_store_fwupdate_buffer);

static ssize_t shtpsif_store_clear_fwupdate_buffer(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	shtps_fwupdate_datasize = 0;

	return count;
}
SHTPSIF_DEFINE(clear_fwupdate_buffer, SHTPSIF_SHOW_COMMON, shtpsif_store_clear_fwupdate_buffer);

static ssize_t shtpsif_show_enter_bootloader(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;
#ifdef CONFIG_COMPAT
	struct shtps_compat_bootloader_info info;
#else
	struct shtps_bootloader_info info;
#endif /* CONFIG_COMPAT */

	SHTPSIF_LOG_FUNC_CALL();

	request_event(ts, SHTPS_EVENT_STOP, 0);
	rc = shtps_enter_bootloader(ts);

	if(0 == rc){
		info.block_size        = shtps_fwctl_loader_get_blocksize(ts);
		info.program_block_num = shtps_fwctl_loader_get_firm_blocknum(ts);
		info.config_block_num  = shtps_fwctl_loader_get_config_blocknum(ts);

		memcpy(buf, (u8*)&info, sizeof(info));
	}

	if(rc){
		return -EFAULT;
	}

	return sizeof(info);
}
SHTPSIF_DEFINE(enter_bootloader, shtpsif_show_enter_bootloader, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_store_lockdown_bootloader(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {

			if(count == 4){
				int position = 0;
				memcpy(&position, buf, sizeof(position));
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_lockdown_bootloader(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal.\n", __func__);
			rc = shtps_lockdown_bootloader(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_lockdown_bootloader(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(lockdown_bootloader, SHTPSIF_SHOW_COMMON, shtpsif_store_lockdown_bootloader);

static ssize_t shtpsif_store_erase_flash(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = shtps_flash_erase(ts);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(erase_flash, SHTPSIF_SHOW_COMMON, shtpsif_store_erase_flash);

static ssize_t shtpsif_store_write_image(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {
			if(count == 4){
				int position = 0;
				memcpy(&position, buf, sizeof(position));
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_flash_writeImage(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
			rc = shtps_flash_writeImage(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_flash_writeImage(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(write_image, SHTPSIF_SHOW_COMMON, shtpsif_store_write_image);

static ssize_t shtpsif_store_write_config(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
	{
		const unsigned char* fw_data = NULL;
		if (shtps_fwupdate_datasize == 0) {
			if(count == 4){
				int position = 0;
				SHTPS_LOG_DBG_PRINT("[%s] builtin !\n", __func__);
				memcpy(&position, buf, sizeof(position));

				if (position < 0 || position > shtps_fwsize_builtin(ts)) {
					return -EINVAL;
				}

				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data == NULL){
					SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
					return -EFAULT;
				}
				rc = shtps_flash_writeConfig(ts, (u8*)&(fw_data[position]));
			}
			else{
				return -EINVAL;
			}
		}
		else {
			SHTPS_LOG_DBG_PRINT("[%s] normal !\n", __func__);
			rc = shtps_flash_writeConfig(ts, shtps_fwupdate_buffer);
		}
	}
	#else
		rc = shtps_flash_writeConfig(ts, shtps_fwupdate_buffer);
	#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

	if(rc){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(write_config, SHTPSIF_SHOW_COMMON, shtpsif_store_write_config);

static ssize_t shtpsif_show_touchinfo(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int i;
	int fingerMax = shtps_get_fingermax(ts);
	struct shtps_touch_info info;

	SHTPSIF_LOG_FUNC_CALL();

	memcpy(&info, &ts->report_info, sizeof(info));
	for(i = 0;i < fingerMax;i++){
		info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
		info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
	}

	memcpy(buf, (u8*)&info, sizeof(info));

	return sizeof(info);
}
SHTPSIF_DEFINE(touchinfo, shtpsif_show_touchinfo, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_touchinfo_untrans(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	memcpy(buf, (u8*)&ts->report_info, sizeof(ts->report_info));

	return sizeof(ts->report_info);
}
SHTPSIF_DEFINE(touchinfo_untrans, shtpsif_show_touchinfo_untrans, SHTPSIF_STORE_COMMON);


static ssize_t shtpsif_store_touchinfo_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	ts->diag.pos_mode = argv[0];

	return count;
}
SHTPSIF_DEFINE(touchinfo_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_touchinfo_mode);

#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
static ssize_t shtpsif_show_touchkeyinfo(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;
	struct shtps_touch_key_info info;

	SHTPSIF_LOG_FUNC_CALL();

	ret = wait_event_interruptible_timeout(ts->diag.wait, 
			ts->diag.event_touchkey == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(ret != 0){
		info.up_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_UP) & 0x01);
		info.down_key_state = ((ts->key_state >> SHTPS_PHYSICAL_KEY_DOWN) & 0x01);

		ts->diag.event_touchkey = 0;

		memcpy(buf, (u8*)&info, sizeof(info));

		return sizeof(info);
	}

	return -EFAULT;
}
SHTPSIF_DEFINE(touchkeyinfo, shtpsif_show_touchkeyinfo, SHTPSIF_STORE_COMMON);
#endif /* SHTPS_PHYSICAL_KEY_ENABLE */

static ssize_t shtpsif_show_reg_read(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u8 data[SHTPSIF_REG_READ_SIZE_MAX];
	char word[10];
	int i;
	int read_addr = shtpsif_reg_read_addr;
	int read_size = shtpsif_reg_read_size;

	SHTPSIF_LOG_FUNC_CALL();

	if(read_size > sizeof(data)){
		read_size = sizeof(data);
	}
	if(M_READ_PACKET_FUNC(ts->fwctl_p, read_addr, (u8*)&data, read_size)){
		return -EFAULT;
	}
	else{
		snprintf(buf, PAGE_SIZE, "0x%04X", read_addr);
		for(i = 0; i < read_size; i++){
			snprintf(word, sizeof(word), ",0x%02X", data[i]);
			strcat(buf, word);
		}
		strcat(buf, "\n");
	}

	return strlen(buf);
}
static ssize_t shtpsif_store_reg_read(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[2];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtpsif_reg_read_addr = argv[0];
	if(argc >= 2){
		shtpsif_reg_read_size = argv[1];
		if(shtpsif_reg_read_size > SHTPSIF_REG_READ_SIZE_MAX){
			shtpsif_reg_read_size = SHTPSIF_REG_READ_SIZE_MAX;
		}
	}
	else{
		shtpsif_reg_read_size = 1;
	}

	return count;
}
SHTPSIF_DEFINE(reg_read, shtpsif_show_reg_read, shtpsif_store_reg_read);

static ssize_t shtpsif_store_reg_write(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	unsigned short addr;
	u8 data[SHTPSIF_REG_WRITE_SIZE_MAX];
	int argc;
	int argv[1 + SHTPSIF_REG_WRITE_SIZE_MAX];
	int datanum;
	int i;

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	datanum = argc - 1;

	if(datanum <= 0){
		return -EINVAL;
	}

	addr = argv[0];

	for(i = 0; i < datanum; i++){
		data[i] = argv[1 + i];
	}

	if(M_WRITE_PACKET_FUNC(ts->fwctl_p, addr, data, datanum)){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(reg_write, SHTPSIF_SHOW_COMMON, shtpsif_store_reg_write);

static ssize_t shtpsif_store_tm_start(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	ts->diag.tm_mode = shtps_fwctl_get_tm_mode(ts);

	if(0 != request_event(ts, SHTPS_EVENT_STARTTM, argv[0])){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(tm_start, SHTPSIF_SHOW_COMMON, shtpsif_store_tm_start);

static ssize_t shtpsif_store_tm_stop(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	if(0 != request_event(ts, SHTPS_EVENT_STOPTM, 0)){
		return -EFAULT;
	}

	return count;
}
SHTPSIF_DEFINE(tm_stop, SHTPSIF_SHOW_COMMON, shtpsif_store_tm_stop);

static ssize_t shtpsif_show_baseline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(baseline, shtpsif_show_baseline, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_baseline_raw(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_BASELINE_RAW);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(baseline_raw, shtpsif_show_baseline_raw, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_hybrid_adc(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_HYBRID_ADC);
	size = (shtps_get_tm_rxsize(ts) + shtps_get_tm_txsize(ts)) * 4;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(hybrid_adc, shtpsif_show_hybrid_adc, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_adc_range(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_ADC_RANGE);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(adc_range, shtpsif_show_adc_range, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_frameline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int size;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_read_tmdata(ts, SHTPS_TMMODE_FRAMELINE);
	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy(buf, (u8*)ts->diag.tm_data, size);

	return size;
}
SHTPSIF_DEFINE(frameline, shtpsif_show_frameline, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_poll_touch_info(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret;
	int i;
	int fingerMax;
	struct shtps_touch_info info;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

//	SHTPSIF_LOG_FUNC_CALL();

	if(NULL == ts){
		return POLLERR;
	}

	ret = wait_event_interruptible_timeout(ts->diag.wait,
			ts->diag.event == 1,
			msecs_to_jiffies(SHTPS_DIAGPOLL_TIME));

	if(0 == ret){
		return 0;
	}

	fingerMax = shtps_get_fingermax(ts);

	memcpy(&info, &ts->report_info, sizeof(info));
	if(ts->diag.pos_mode == TPSDEV_TOUCHINFO_MODE_LCDSIZE){
		for(i = 0;i < fingerMax;i++){
			info.fingers[i].x = info.fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000;
			info.fingers[i].y = info.fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000;
		}
	}
	memcpy((u8*)buf, (u8*)&info, sizeof(info));

	ts->diag.event = 0;
	return sizeof(ts->report_info);
}
SHTPSIF_DEFINE(poll_touch_info, shtpsif_show_poll_touch_info, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_store_start_facetouchmode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_ON, 0);

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(start_facetouchmode, SHTPSIF_SHOW_COMMON, shtpsif_store_start_facetouchmode);

static ssize_t shtpsif_store_stop_facetouchmode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret;

	SHTPSIF_LOG_FUNC_CALL();

	ret = request_event(ts, SHTPS_EVENT_FACETOUCHMODE_OFF, 0);
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	_log_msg_send( LOGMSG_ID__DETECT_FACETOUCH, "");
	ts->facetouch.wake_sig = 1;
	wake_up_interruptible(&ts->facetouch.wait_off);
	shtps_facetouch_wakelock(ts, 0);	//8926
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(stop_facetouchmode, SHTPSIF_SHOW_COMMON, shtpsif_store_stop_facetouchmode);

static ssize_t shtpsif_show_poll_facetouchoff(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int rc;

	SHTPSIF_LOG_FUNC_CALL();

	#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;

		SHTPS_LOG_DBG_PRINT("wait for face touch off detect\n");
		rc = wait_event_interruptible(ts->facetouch.wait_off,
			(ts->facetouch.detect == 1) || (ts->facetouch.mode == 0) ||
			(ts->facetouch.wake_sig == 1));

		_log_msg_recv( LOGMSG_ID__DETECT_FACETOUCH, "%d|%d|%d",
							ts->facetouch.detect, ts->facetouch.mode, ts->facetouch.wake_sig);

		ts->facetouch.wake_sig = 0;

		if(ts->facetouch.detect){
			SHTPS_LOG_DBG_PRINT("face touch %s detect\n", (ts->facetouch.state == 0)? "off" : "on");
			rc = (ts->facetouch.state == 0)? TPSDEV_FACETOUCH_OFF_DETECT : TPSDEV_FACETOUCH_DETECT;
			ts->facetouch.detect = 0;
		}else{
			SHTPS_LOG_DBG_PRINT("wait for facetouch was cancelled\n");
			rc = TPSDEV_FACETOUCH_NOCHG;
		}
	}
	#else
		rc = TPSDEV_FACETOUCH_NOCHG;
	#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return snprintf(buf, PAGE_SIZE, "%d\n", rc);
}
SHTPSIF_DEFINE(poll_facetouchoff, shtpsif_show_poll_facetouchoff, SHTPSIF_STORE_COMMON);

static ssize_t shtpsif_show_calibration_param(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
						ts->offset.base[0],
						ts->offset.base[1],
						ts->offset.base[2],
						ts->offset.base[3],
						ts->offset.base[4],
						ts->offset.diff[0],
						ts->offset.diff[1],
						ts->offset.diff[2],
						ts->offset.diff[3],
						ts->offset.diff[4],
						ts->offset.diff[5],
						ts->offset.diff[6],
						ts->offset.diff[7],
						ts->offset.diff[8],
						ts->offset.diff[9],
						ts->offset.diff[10],
						ts->offset.diff[11]);
}
static ssize_t shtpsif_store_calibration_param(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	const u8 *data;

	SHTPSIF_LOG_FUNC_CALL();

	if(count < (sizeof(ts->offset.base) + sizeof(ts->offset.diff))){
		return -EINVAL;
	}
	data = buf;

	shtps_mutex_lock_ctrl();

	memcpy(ts->offset.base, data, sizeof(u16) * 5);
	ts->offset.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
	ts->offset.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
	ts->offset.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
	ts->offset.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
	ts->offset.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
	ts->offset.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
	ts->offset.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
	ts->offset.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
	ts->offset.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
	ts->offset.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
	ts->offset.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
	ts->offset.diff[11]= (signed short)(data[33] << 0x08 | data[32]);

	if(ts->offset.base[0] == 0){
		ts->offset.enabled = 0;
	}else{
		ts->offset.enabled = 1;
	}

	shtps_mutex_unlock_ctrl();

	shtps_report_stflib_kevt(ts, SHTPS_DEF_STFLIB_KEVT_CALIBRATION, 0);

	return count;
}
SHTPSIF_DEFINE(calibration_param, shtpsif_show_calibration_param, shtpsif_store_calibration_param);

#if defined( SHTPS_PEN_DETECT_ENABLE )
static ssize_t shtpsif_store_calibration_pen_param(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
		struct shtps_rmi_spi *ts = gShtps_rmi_spi;
		const u8 *data;

		SHTPSIF_LOG_FUNC_CALL();

		if(count < (sizeof(ts->offset_pen.base) + sizeof(ts->offset_pen.diff))){
			return -EINVAL;
		}
		data = buf;

		memcpy(ts->offset_pen.base, data, sizeof(u16) * 5);
		ts->offset_pen.diff[0] = (signed short)(data[11] << 0x08 | data[10]);
		ts->offset_pen.diff[1] = (signed short)(data[13] << 0x08 | data[12]);
		ts->offset_pen.diff[2] = (signed short)(data[15] << 0x08 | data[14]);
		ts->offset_pen.diff[3] = (signed short)(data[17] << 0x08 | data[16]);
		ts->offset_pen.diff[4] = (signed short)(data[19] << 0x08 | data[18]);
		ts->offset_pen.diff[5] = (signed short)(data[21] << 0x08 | data[20]);
		ts->offset_pen.diff[6] = (signed short)(data[23] << 0x08 | data[22]);
		ts->offset_pen.diff[7] = (signed short)(data[25] << 0x08 | data[24]);
		ts->offset_pen.diff[8] = (signed short)(data[27] << 0x08 | data[26]);
		ts->offset_pen.diff[9] = (signed short)(data[29] << 0x08 | data[28]);
		ts->offset_pen.diff[10]= (signed short)(data[31] << 0x08 | data[30]);
		ts->offset_pen.diff[11]= (signed short)(data[33] << 0x08 | data[32]);

		if(ts->offset_pen.base[0] == 0){
			ts->offset_pen.enabled = 0;
		}else{
			ts->offset_pen.enabled = 1;
		}

	return count;
}
SHTPSIF_DEFINE(calibration_pen_param, SHTPSIF_SHOW_COMMON, shtpsif_store_calibration_pen_param);
#endif /* SHTPS_PEN_DETECT_ENABLE */

static ssize_t shtpsif_store_rezero(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();

	if(ts->poll_info.boot_rezero_flag == 0){
		ts->poll_info.boot_rezero_flag = 1;
			shtps_rezero_request(ts,
								 SHTPS_REZERO_REQUEST_REZERO |
								 SHTPS_REZERO_REQUEST_AUTOREZERO_ENABLE,
								 SHTPS_REZERO_TRIGGER_BOOT);
	}

	shtps_mutex_unlock_ctrl();

	return count;
}
SHTPSIF_DEFINE(rezero, SHTPSIF_SHOW_COMMON, shtpsif_store_rezero);

static ssize_t shtpsif_store_ack_facetouchoff(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();
	shtps_facetouch_wakelock(ts, 0);	//8926
	shtps_mutex_unlock_ctrl();
#endif /* #if defined( CONFIG_SHTPS_SY3000_FACETOUCH_OFF_DETECT ) */

	return count;
}
SHTPSIF_DEFINE(ack_facetouchoff, SHTPSIF_SHOW_COMMON, shtpsif_store_ack_facetouchoff);

#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
static ssize_t shtpsif_show_log_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", gLogOutputEnable);
}
static ssize_t shtpsif_store_log_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	gLogOutputEnable = argv[0];

	return count;
}
SHTPSIF_DEFINE(log_enable, shtpsif_show_log_enable, shtpsif_store_log_enable);
#endif /* #if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE ) */

#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
static ssize_t shtpsif_show_fwver_builtin(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	u16 ver = shtps_fwver_builtin(ts);

	SHTPSIF_LOG_FUNC_CALL();

	if(shtps_fwdata_builtin(ts) == NULL){
		SHTPS_LOG_ERR_PRINT("[%s] error - No built-in FW\n", __func__);
		return -EFAULT;
	}

	SHTPS_LOG_DBG_PRINT("[%s] built-in version = 0x%04x\n", __func__, ver);

	return snprintf(buf, PAGE_SIZE, "%04X\n", ver);
}
SHTPSIF_DEFINE(fwver_builtin, shtpsif_show_fwver_builtin, SHTPSIF_STORE_COMMON);
#endif /* #if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE ) */

#if defined( SHTPS_SMEM_BASELINE_ENABLE )
static ssize_t shtpsif_show_smem_baseline(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();
	int size = 0;

	SHTPSIF_LOG_FUNC_CALL();

	if(!smemdata){
		return -EFAULT;
	}

	size = shtps_get_tm_rxsize(ts) * shtps_get_tm_txsize(ts) * 2;
	memcpy((u8*)buf, (u8*)(smemdata->shdiag_TpsBaseLineTbl), size);

	return size;
}
SHTPSIF_DEFINE(smem_baseline, shtpsif_show_smem_baseline, SHTPSIF_STORE_COMMON);
#endif /* #if defined( SHTPS_SMEM_BASELINE_ENABLE ) */

static ssize_t shtpsif_store_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPMODE, argv[0]) == 0){
			shtps_ioctl_setlpmode_proc(ts, argv[0]);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_COMMON, argv[0]);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_low_power_mode);

static ssize_t shtpsif_store_continuous_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETCONLPMODE, argv[0]) == 0){
			shtps_ioctl_setconlpmode_proc(ts, argv[0]);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_ECO, argv[0]);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(continuous_low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_continuous_low_power_mode);

static ssize_t shtpsif_store_lcd_bright_low_power_mode(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLCDBRIGHTLPMODE, argv[0]) == 0){
			shtps_ioctl_setlcdbrightlpmode_proc(ts, argv[0]);
		}
	#else
		shtps_mutex_lock_ctrl();

		shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_NON_CONTINUOUS, SHTPS_LPMODE_REQ_LCD_BRIGHT, argv[0]);

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */

	return count;
}
SHTPSIF_DEFINE(lcd_bright_low_power_mode, SHTPSIF_SHOW_COMMON, shtpsif_store_lcd_bright_low_power_mode);

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static ssize_t shtpsif_store_charger_armor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, argv[0]) == 0){
			shtps_charger_armor_proc(ts, argv[0]);
		}
	#else
	{
		int ret = -1;
		shtps_mutex_lock_ctrl();

		ret = shtps_filter_set_charger_armor(ts, (int)ts->state_mgr.state, argv[0]);

		shtps_mutex_unlock_ctrl();

		if(ret < 0){
			return ret;
		}
	}
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(charger_armor, SHTPSIF_SHOW_COMMON, shtpsif_store_charger_armor);
#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */

#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
static ssize_t shtpsif_store_wireless_charger_armor(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CHARGER_ARMOR, argv[0]) == 0){
			shtps_charger_armor_proc(ts, argv[0]);
		}
	#else
	{
		int ret = -1;
		shtps_mutex_lock_ctrl();

		ret = shtps_filter_set_charger_armor(ts, (int)ts->state_mgr.state, argv[0]);

		shtps_mutex_unlock_ctrl();

		if(ret < 0){
			return ret;
		}
	}
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(wireless_charger_armor, SHTPSIF_SHOW_COMMON, shtpsif_store_wireless_charger_armor);
#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */

#if defined( SHTPS_LPWG_MODE_ENABLE )
static ssize_t shtpsif_show_lpwg_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->lpwg.lpwg_sweep_on_req_state);
}
static ssize_t shtpsif_store_lpwg_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

 	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETLPWG, argv[0]) == 0){
			shtps_ioctl_setlpwg_proc(ts, argv[0]);
		}
	#else
		shtps_mutex_lock_ctrl();
		ts->lpwg.lpwg_sweep_on_req_state = argv[0];
		ts->lpwg.notify_enable = 1;
		SHTPS_LOG_DBG_PRINT(" [LPWG] lpwg_sweep_on_req_state = %d\n", ts->lpwg.lpwg_sweep_on_req_state);
		shtps_set_lpwg_sleep_check(ts);
		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(lpwg_enable, shtpsif_show_lpwg_enable, shtpsif_store_lpwg_enable);
#endif /* SHTPS_LPWG_MODE_ENABLE */

#if defined(SHTPS_LPWG_DOUBLE_TAP_ENABLE)
static ssize_t shtpsif_show_lpwg_double_tap_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->lpwg.lpwg_double_tap_req_state);
}
static ssize_t shtpsif_store_lpwg_double_tap_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtps_ioctl_lpwg_double_tap_enable(ts, argv[0]);

	return count;
}
SHTPSIF_DEFINE(lpwg_double_tap_enable, shtpsif_show_lpwg_double_tap_enable, shtpsif_store_lpwg_double_tap_enable);
#endif /* SHTPS_LPWG_DOUBLE_TAP_ENABLE */

static ssize_t shtpsif_store_veilview_state(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	SHTPSIF_LOG_FUNC_CALL();

	return count;
}
SHTPSIF_DEFINE(veilview_state, SHTPSIF_SHOW_COMMON, shtpsif_store_veilview_state);

static ssize_t shtpsif_show_veilview_pattern(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", SHTPS_VEILVIEW_PATTERN);
}
static ssize_t shtpsif_store_veilview_pattern(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if defined( SHTPS_DEBUG_VARIABLE_DEFINES )
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	SHTPS_VEILVIEW_PATTERN = argv[0];
#else /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */
	return -EFAULT;
#endif /* #if defined( SHTPS_DEBUG_VARIABLE_DEFINES ) */

	return count;
}
SHTPSIF_DEFINE(veilview_pattern, shtpsif_show_veilview_pattern, shtpsif_store_veilview_pattern);

#if defined(SHTPS_HOVER_DETECT_ENABLE)
static ssize_t shtpsif_store_hover_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETHOVER, (u8)argv[0]) == 0){
			shtps_ioctl_sethover_proc(ts, argv[0]);
		}
	#else
		shtps_mutex_lock_ctrl();

		if(argv[0] == 0){
			shtps_set_hover_detect_disable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 1);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}else{
			shtps_set_hover_detect_enable(ts);

			#if defined( SHTPS_LOW_POWER_MODE_ENABLE )
				shtps_set_lpmode(ts, SHTPS_LPMODE_TYPE_CONTINUOUS, SHTPS_LPMODE_REQ_HOVER_OFF, 0);
			#endif /* #if defined( SHTPS_LOW_POWER_MODE_ENABLE ) */
		}

		shtps_mutex_unlock_ctrl();
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(hover_enable, SHTPSIF_SHOW_COMMON, shtpsif_store_hover_enable);
#endif /* SHTPS_HOVER_DETECT_ENABLE */

static ssize_t shtpsif_store_baseline_offset_disable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = -1;
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_mutex_lock_ctrl();

	ret = shtps_baseline_offset_disable(ts);

	shtps_mutex_unlock_ctrl();

	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(baseline_offset_disable, SHTPSIF_SHOW_COMMON, shtpsif_store_baseline_offset_disable);

#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
static ssize_t shtpsif_store_check_crc_error(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	shtps_func_request_sync(ts, SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR);

	return count;
}
SHTPSIF_DEFINE(check_crc_error, SHTPSIF_SHOW_COMMON, shtpsif_store_check_crc_error);
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

#if defined(SHTPS_PEN_DETECT_ENABLE)
static ssize_t shtpsif_show_pen_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->pen_enable);
}
static ssize_t shtpsif_store_pen_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
		SHTPS_LOG_FUNC_CALL();
		if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_IOCTL_SETPEN, argv[0]) == 0){
			shtps_ioctl_setpen_proc(ts, argv[0]);
		}
	#else
	{
		int ret = -1;
		shtps_mutex_lock_ctrl();

		ret = shtps_set_pen_enable(ts, argv[0]);

		shtps_mutex_unlock_ctrl();

		if(ret < 0){
			return ret;
		}
	}
	#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */

	return count;
}
SHTPSIF_DEFINE(pen_enable, shtpsif_show_pen_enable, shtpsif_store_pen_enable);
#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */

#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
static ssize_t shtpsif_show_device_access_log_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", shtps_get_spi_transfer_log_enable());
}
static ssize_t shtpsif_store_device_access_log_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtps_set_spi_transfer_log_enable(argv[0]);

	return count;
}
SHTPSIF_DEFINE(device_access_log_enable, shtpsif_show_device_access_log_enable, shtpsif_store_device_access_log_enable);
#endif /* #if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE) */

#if defined(SHTPS_GLOVE_DETECT_ENABLE)
static ssize_t shtpsif_store_glove_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtps_ioctl_set_glove_mode(ts, argv[0]);

	return count;
}
SHTPSIF_DEFINE(glove_enable, SHTPSIF_SHOW_COMMON, shtpsif_store_glove_enable);
#endif /* SHTPS_GLOVE_DETECT_ENABLE */

#if defined(SHTPS_CTRL_FW_REPORT_RATE)
static ssize_t shtpsif_show_high_report_mode_enable(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->fw_report_rate_req_state);
}
static ssize_t shtpsif_store_high_report_mode_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int argc;
	int argv[1];

	SHTPSIF_LOG_FUNC_CALL();

	argc = shtps_numStrToList(buf, argv, sizeof(argv)/sizeof(int));
	if(argc < 1){
		return -EINVAL;
	}

	shtps_ioctl_set_high_report_mode(ts, argv[0]);

	return count;
}
SHTPSIF_DEFINE(high_report_mode_enable, shtpsif_show_high_report_mode_enable, shtpsif_store_high_report_mode_enable);
#endif /* SHTPS_CTRL_FW_REPORT_RATE */

static ssize_t shtpsif_show_hw_revision(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
//	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int hw_revision;
	int hw_type;
//	int tpin;

	SHTPSIF_LOG_FUNC_CALL();

	hw_revision = shtps_system_get_hw_revision();
	hw_type = shtps_system_get_hw_type();
//	tpin = shtps_tpin_enable_check(ts);

//	return snprintf(buf, PAGE_SIZE, "hw_revision=%s(%d) hw_type=%s(%d) tpin=%d\n",
	return snprintf(buf, PAGE_SIZE, "hw_revision=%s(%d) hw_type=%s(%d)\n",
		(hw_revision == SHTPS_GET_HW_VERSION_RET_ES_0 ? "ES0" :
		(hw_revision == SHTPS_GET_HW_VERSION_RET_ES_1 ? "ES1" :
		(hw_revision == SHTPS_GET_HW_VERSION_RET_PP_1 ? "PP1" :
		(hw_revision == SHTPS_GET_HW_VERSION_RET_PP_1_5 ? "PP1.5" :
		(hw_revision == SHTPS_GET_HW_VERSION_RET_PP_2 ? "PP2" :
		(hw_revision == SHTPS_GET_HW_VERSION_RET_MP ? "PMP" :
		 "UNKNOWN")))))),
		hw_revision,
		(hw_type == SHTPS_HW_TYPE_BOARD ? "BOARD" : "HANDSET"),
		hw_type
//		tpin
		);
}
SHTPSIF_DEFINE(hw_revision, shtpsif_show_hw_revision, SHTPSIF_STORE_COMMON);

#ifdef SHTPS_DEVELOP_MODE_ENABLE
static ssize_t shtpsif_show_stflib_change_parameter(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;

	SHTPSIF_LOG_FUNC_CALL();

	return snprintf(buf, PAGE_SIZE, "%s\n", ts->stflib_param_str);
}
static ssize_t shtpsif_store_stflib_change_parameter(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct shtps_rmi_spi *ts = gShtps_rmi_spi;
	int ret = 0;
	char *argbuf;
	int argc;
	char *argv[10];
	int i;

	SHTPSIF_LOG_FUNC_CALL();

	argbuf = (char*)kzalloc(count + 1, GFP_KERNEL);
	if(argbuf == NULL){
		SHTPS_LOG_ERR_PRINT( "argbuf alloc error.\n" );
		return -ENOMEM;
	}
	memcpy(argbuf, buf, count);

	argc = shtpsif_get_arguments( argbuf, argv, sizeof(argv)/sizeof(char *) );

	ts->stflib_param_str[0] = '\0';
	for(i = 0; i < argc; i++){
		if(strlen(ts->stflib_param_str) + 2 >= sizeof(ts->stflib_param_str) - 1){
			SHTPS_LOG_ERR_PRINT("parameter too long. (max %lu)\n", sizeof(ts->stflib_param_str) - 1);
			break;
		}

		if(i > 0){
			strcat(ts->stflib_param_str, ",");
		}
		strcat(ts->stflib_param_str, argv[i]);
	}

	shtps_report_stflib_kevt(ts, SHTPS_DEF_STFLIB_KEVT_CHANGE_PARAM, 0);

	kfree(argbuf);
	if(ret < 0){
		return ret;
	}
	return count;
}
SHTPSIF_DEFINE(stflib_change_parameter, shtpsif_show_stflib_change_parameter, shtpsif_store_stflib_change_parameter);
#endif /* SHTPS_DEVELOP_MODE_ENABLE */

/* -----------------------------------------------------------------------------------
 */

static struct attribute *attrs_shtpsif[] = {
	&shtpsif_shtpsiflog.attr,
	&shtpsif_enable.attr,
	&shtpsif_disable.attr,
	&shtpsif_reset.attr,
	&shtpsif_softreset.attr,
	&shtpsif_fwver.attr,
	&shtpsif_fwupdate_buffer.attr,
	&shtpsif_clear_fwupdate_buffer.attr,
	&shtpsif_enter_bootloader.attr,
	&shtpsif_lockdown_bootloader.attr,
	&shtpsif_erase_flash.attr,
	&shtpsif_write_image.attr,
	&shtpsif_write_config.attr,
	&shtpsif_touchinfo.attr,
	&shtpsif_touchinfo_untrans.attr,
	&shtpsif_touchinfo_mode.attr,
	#if defined( SHTPS_PHYSICAL_KEY_ENABLE )
		&shtpsif_touchkeyinfo.attr,
	#endif /* SHTPS_PHYSICAL_KEY_ENABLE */
	&shtpsif_reg_read.attr,
	&shtpsif_reg_write.attr,
	&shtpsif_tm_start.attr,
	&shtpsif_tm_stop.attr,
	&shtpsif_baseline.attr,
	&shtpsif_baseline_raw.attr,
	&shtpsif_hybrid_adc.attr,
	&shtpsif_adc_range.attr,
	&shtpsif_frameline.attr,
	&shtpsif_poll_touch_info.attr,
	&shtpsif_start_facetouchmode.attr,
	&shtpsif_stop_facetouchmode.attr,
	&shtpsif_poll_facetouchoff.attr,
	&shtpsif_calibration_param.attr,
	&shtpsif_rezero.attr,
	&shtpsif_ack_facetouchoff.attr,
	#if defined( SHTPS_LOG_OUTPUT_SWITCH_ENABLE )
		&shtpsif_log_enable.attr,
	#endif /* SHTPS_LOG_OUTPUT_SWITCH_ENABLE */
	#if defined( SHTPS_FWUPDATE_BUILTIN_ENABLE )
		&shtpsif_fwver_builtin.attr,
	#endif	/* SHTPS_FWUPDATE_BUILTIN_ENABLE */
	#if defined( SHTPS_SMEM_BASELINE_ENABLE )
		&shtpsif_smem_baseline.attr,
	#endif /* SHTPS_SMEM_BASELINE_ENABLE */
	&shtpsif_low_power_mode.attr,
	&shtpsif_continuous_low_power_mode.attr,
	&shtpsif_lcd_bright_low_power_mode.attr,
	#if defined( SHTPS_CHARGER_ARMOR_ENABLE )
		&shtpsif_charger_armor.attr,
		&shtpsif_wireless_charger_armor.attr,
	#endif	/* SHTPS_CHARGER_ARMOR_ENABLE */
	#if defined( SHTPS_LPWG_MODE_ENABLE )
		&shtpsif_lpwg_enable.attr,
	#endif /* SHTPS_LPWG_MODE_ENABLE */
	#if defined(SHTPS_LPWG_DOUBLE_TAP_ENABLE)
		&shtpsif_lpwg_double_tap_enable.attr,
	#endif /* SHTPS_LPWG_DOUBLE_TAP_ENABLE */
	&shtpsif_veilview_state.attr,
	&shtpsif_veilview_pattern.attr,
	#if defined(SHTPS_HOVER_DETECT_ENABLE)
		&shtpsif_hover_enable.attr,
	#endif /* SHTPS_HOVER_DETECT_ENABLE */
	&shtpsif_baseline_offset_disable.attr,
	#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
		&shtpsif_check_crc_error.attr,
	#endif /* SHTPS_CHECK_CRC_ERROR_ENABLE */
	#if defined(SHTPS_PEN_DETECT_ENABLE)
		&shtpsif_calibration_pen_param.attr,
		&shtpsif_pen_enable.attr,
	#endif /* #if defined(SHTPS_PEN_DETECT_ENABLE) */
	#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
		&shtpsif_device_access_log_enable.attr,
	#endif /* #if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE) */
	#if defined(SHTPS_GLOVE_DETECT_ENABLE)
		&shtpsif_glove_enable.attr,
	#endif /* SHTPS_GLOVE_DETECT_ENABLE */
	#if defined(SHTPS_CTRL_FW_REPORT_RATE)
		&shtpsif_high_report_mode_enable.attr,
	#endif /* SHTPS_CTRL_FW_REPORT_RATE */
		&shtpsif_hw_revision.attr,
	#ifdef SHTPS_DEVELOP_MODE_ENABLE
		&shtpsif_stflib_change_parameter.attr,
	#endif /* SHTPS_DEVELOP_MODE_ENABLE */
	NULL
};

static struct attribute_group shtps_attr_grp_shtpsif = {
	.name = "shtpsif",
	.attrs = attrs_shtpsif,
};

/*  -----------------------------------------------------------------------------------
 */
void shtps_init_io_debugfs(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	ts->kobj = kobject_create_and_add("shtps", kernel_kobj);
	if(ts->kobj == NULL){
		SHTPS_LOG_ERR_PRINT("kobj create failed : shtps\n");
	}else{
		if(sysfs_create_group(ts->kobj, &shtps_attr_grp_shtpsif)){
			SHTPS_LOG_ERR_PRINT("kobj create failed : shtps_attr_grp_shtpsif\n");
		}
	}
}

void shtps_deinit_io_debugfs(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();
	if(ts->kobj != NULL){
		sysfs_remove_group(ts->kobj, &shtps_attr_grp_shtpsif);
		kobject_put(ts->kobj);
	}
}
