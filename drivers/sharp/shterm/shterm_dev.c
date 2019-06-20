/* drivers/sharp/shterm/shterm_dev.c
 *
 * Copyright (C) 2011 Sharp Corporation
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

#include <linux/string.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/alarmtimer.h>
#include <linux/ioctl.h>
#include <linux/power_supply.h>
#include "sharp/shdisp_kerl.h"
#include "sharp/shterm_k.h"
#include "sharp/shbatt_kerl.h"

#define SHTERM_NOR_TIMER_SEC (60*60)
#define SHTERM_CHG_TIMER_SEC (10*60)
#define SHTERM_TMP_TIMER_SEC (30*60)

#define SHTERM_CPU_TMP_THRESHOLD 50

#define SHTERM_MAGIC 't'
#define SHTERM_CMD_START_TIMER _IO(SHTERM_MAGIC, 256)
#define SHTERM_CMD_GET_TIMER _IO(SHTERM_MAGIC, 255)

static struct alarm shterm_dev_alarms;
static struct work_struct shterm_dev_work;
#ifdef SHTERM_BS_TIMER
static int timer_type = -1;
#endif
static struct wake_lock shterm_wake_lock;

static void shterm_work_periodic_event( struct work_struct *work_p )
{
    shbatt_result_t ret = 0;
    shbatt_batt_log_info_t b_info = {0};
    shbattlog_info_t info = {0};
    struct timespec ts;
    static struct power_supply *psy;
    union power_supply_propval ps_ret = {0,};

#ifdef SHTERM_BS_TIMER
    if( timer_type != -1 ){
        shdisp_api_check_blackscreen_timeout( timer_type );
    }
#endif
    /* set info */
    ret = shbatt_api_get_battery_log_info( &b_info );
    if( SHBATT_RESULT_SUCCESS != ret ){
        shbatt_api_get_battery_log_info( &b_info );
    }
    info.bat_vol = b_info.bat_vol;
    info.bat_temp = b_info.bat_temp;
    info.cpu_temp = b_info.cpu_temp;
    info.cam_temp = b_info.cam_temp;
    info.lcd_temp = b_info.lcd_temp;
    info.pa_temp = b_info.pa_temp;
    info.avg_cur = b_info.avg_cur;
    info.avg_vol = b_info.avg_vol;
    info.chg_vol = b_info.chg_vol;
    info.chg_cur = b_info.chg_cur;
    info.latest_cur = b_info.latest_cur;
    info.acc_cur = b_info.acc_cur;
    info.vol_per = b_info.vol_per;
    info.cur_dep_per = b_info.cur_dep_per;
    info.avg_dep_per = b_info.avg_dep_per;
    getnstimeofday(&ts);

    /* chg? */
    if (psy == NULL) {
        psy = power_supply_get_by_name("battery");
    }
    if (psy != NULL) {
        if (psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &ps_ret)) {
            pr_err("failed to get prop battery\n");
            ps_ret.intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
    }
    else {
         pr_err("failed to get by name battery\n");
         ps_ret.intval = POWER_SUPPLY_STATUS_DISCHARGING;
    }

    if( POWER_SUPPLY_STATUS_CHARGING == ps_ret.intval ){
        info.event_num = SHBATTLOG_EVENT_BATT_REPORT_CHG;
        ts.tv_sec += SHTERM_CHG_TIMER_SEC;
#ifdef SHTERM_BS_TIMER
        timer_type = SHDISP_BLACKSCREEN_TYPE_T10;
#endif
    }
    else if( info.cpu_temp >= SHTERM_CPU_TMP_THRESHOLD ){
        info.event_num = SHBATTLOG_EVENT_BATT_REPORT_NORM;
        ts.tv_sec += SHTERM_TMP_TIMER_SEC;
#ifdef SHTERM_BS_TIMER
        timer_type = SHDISP_BLACKSCREEN_TYPE_T30;
#endif
    }
    else {
        info.event_num = SHBATTLOG_EVENT_BATT_REPORT_NORM;
        ts.tv_sec += SHTERM_NOR_TIMER_SEC;
#ifdef SHTERM_BS_TIMER
        timer_type = SHDISP_BLACKSCREEN_TYPE_T30;
#endif
    }

    /* send event */
    shterm_k_set_event( &info );
    /* set alarm */
    alarm_start( &shterm_dev_alarms, timespec_to_ktime(ts) );
}

static enum alarmtimer_restart shterm_timer_func( struct alarm *alarm, ktime_t now )
{
    if( !wake_lock_active(&shterm_wake_lock) ){
        wake_lock_timeout( &shterm_wake_lock, 5 * HZ );
    }
    schedule_work( &shterm_dev_work );

    return ALARMTIMER_NORESTART;
}

static int shterm_dev_open( struct inode *inode, struct file *filp )
{
    return 0;
}

static long shterm_dev_ioctl( struct file *file, unsigned int cmd, unsigned long arg )
{
    int ret;
    struct timespec ts;

    switch( cmd ){
    case SHTERM_CMD_START_TIMER:
        getnstimeofday(&ts);
        ts.tv_sec += 60;
        alarm_start( &shterm_dev_alarms, timespec_to_ktime(ts) );
        ret = 0;
        break;

    case SHTERM_CMD_GET_TIMER:
        if( wake_lock_active(&shterm_wake_lock) ){
            wake_unlock( &shterm_wake_lock );
        }
        ret = 0;
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

static int shterm_dev_close( struct inode *inode, struct file *filp )
{
    return 0;
}

static struct file_operations shterm_dev_fops = {
    .owner          = THIS_MODULE,
    .open           = shterm_dev_open,
    .unlocked_ioctl = shterm_dev_ioctl,
    .release        = shterm_dev_close,
};

static struct miscdevice shterm_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "shterm_dev",
    .fops = &shterm_dev_fops,
};

static int __init shterm_init( void )
{
    int ret;

    ret = misc_register( &shterm_dev );
    if( ret ){
        printk( "misc_register failure shterm_init\n" );
        return ret;
    }

    wake_lock_init( &shterm_wake_lock, WAKE_LOCK_SUSPEND, "shterm");

    INIT_WORK( &shterm_dev_work, shterm_work_periodic_event );

    /* init & set alarm */
    alarm_init( &shterm_dev_alarms, ALARM_REALTIME, shterm_timer_func );

    return ret;
}

module_init(shterm_init);
