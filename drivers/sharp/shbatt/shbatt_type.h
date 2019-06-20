/* drivers/sharp/shbatt/shbatt_type.h
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

#ifndef SHBATT_TYPE_H
#define SHBATT_TYPE_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/
#define SHBATT_IOC_MAGIC						's'

/* Timer */
#define SHBATT_DRV_IOCTL_CMD_INITIALIZE								_IO(  SHBATT_IOC_MAGIC,  1)
#define SHBATT_DRV_IOCTL_CMD_PULL_USSE_PACKET						_IOR( SHBATT_IOC_MAGIC,  2, shbatt_usse_packet_t)
#define SHBATT_DRV_IOCTL_CMD_DONE_USSE_PACKET						_IOW( SHBATT_IOC_MAGIC,  3, shbatt_usse_packet_t)
#define SHBATT_DRV_IOCTL_CMD_SET_TIMER								_IOW( SHBATT_IOC_MAGIC,  4, shbatt_poll_timer_info_t)
#define SHBATT_DRV_IOCTL_CMD_CLR_TIMER								_IOW( SHBATT_IOC_MAGIC,  5, shbatt_poll_timer_info_t)
#define SHBATT_DRV_IOCTL_CMD_GET_BOOT_TIME							_IOR( SHBATT_IOC_MAGIC,  6, shbatt_boottime_info_t*)
#define SHBATT_DRV_IOCTL_CMD_GET_SHBATT_LOG_INFO					_IOR( SHBATT_IOC_MAGIC,  7, shbatt_log_info_t*)

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
/*attributes.*/
/*cable(ac,usb)status attributes.*/

/* Timer */
typedef enum shbatt_api_to_tsk_command_tag
{
	SHBATT_TASK_CMD_INVALID,
	SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	SHBATT_TASK_CMD_BATTLOG_EVENT,
/*| TODO: New API add point */
	NUM_SHBATT_TASK_CMD,
} shbatt_api_to_tsk_command_t;

typedef enum
{
	SHBATT_CMD_INVALID,
	SHBATT_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE,
	NUM_SHBATT_CMD,
} shbatt_kernel_to_user_command_t;

typedef enum shbatt_poll_timer_type_tag
{
	SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC,
	NUM_SHBATT_POLL_TIMER_TYPE
} shbatt_poll_timer_type_t;

typedef enum shbatt_low_battery_event_tag
{
	SHBATT_LOW_BATTERY_EVENT_LOW_INTERRUPT,
	SHBATT_LOW_BATTERY_EVENT_FATAL_INTERRUPT,
	SHBATT_LOW_BATTERY_EVENT_LOW_TIMER,
	SHBATT_LOW_BATTERY_EVENT_FATAL_TIMER,
	SHBATT_LOW_BATTERY_EVENT_LOW_SHUTDOWN,
	NUM_SHBATT_LOW_BATTERY_EVENT
} shbatt_low_battery_event_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shbatt_packet_hdr_tag
{
	shbatt_api_to_tsk_command_t	cmd;
	struct completion*			cmp_p;
	shbatt_result_t* 			ret_p;
} shbatt_packet_hdr_t;

typedef union shbatt_usse_packet_prm_tag
{
	int							evt;
} shbatt_usse_packet_prm_t;

typedef union shbatt_packet_prm_tag
{
	int							evt;
	void*						param_p;
} shbatt_packet_prm_t;

typedef struct shbatt_packet_tag
{
	struct work_struct			work;
	shbatt_packet_hdr_t			hdr;
	shbatt_packet_prm_t			prm;
	bool						is_used;
} shbatt_packet_t;

typedef struct shbatt_usse_packet_hdr_tag
{
	shbatt_kernel_to_user_command_t	cmd;
	shbatt_result_t					ret;
} shbatt_usse_packet_hdr_t;


typedef struct shbatt_usse_packet_tag
{
	shbatt_usse_packet_hdr_t	hdr;
	shbatt_usse_packet_prm_t	prm;
	void*						param_p;
} shbatt_usse_packet_t;

typedef struct shbatt_poll_timer_info_tag
{
	shbatt_poll_timer_type_t	ptt;
	int							ms;
	int							prm;
} shbatt_poll_timer_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ CALLBACK FUNCTION TYPE DECLARE :                                          |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_TYPE_H */
