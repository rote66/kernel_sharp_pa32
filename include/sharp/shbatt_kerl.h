/*
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef SHBATT_KERL_H
#define SHBATT_KERL_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shbatt_result_tag
{
	SHBATT_RESULT_SUCCESS,
	SHBATT_RESULT_FAIL,
	SHBATT_RESULT_REJECTED,
	NUM_SHBATT_RESULT

} shbatt_result_t;

typedef enum shbatt_voltage_alarm_type_tag
{
	SHBATT_VOLTAGE_ALARM_TYPE_LOW_BATTERY,
	SHBATT_VOLTAGE_ALARM_TYPE_FATAL_BATTERY,
	NUM_SHBATT_VOLTAGE_ALARM_TYPE
} shbatt_voltage_alarm_type_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
typedef struct shbatt_gain_offset_data_tag
{
	int						gain;
	int						offset;
} shbatt_gain_offset_data_t;

typedef struct shbatt_batt_log_info_tag
{
	int						event_num;
	int						bat_vol;
	int						chg_vol;
	int						chg_cur;
	int						bat_temp;
	int						cpu_temp;
	int						chg_temp;
	int						cam_temp;
	int						pmic_temp;
	int						pa_temp;
	int						lcd_temp;
	int						avg_cur;
	int						avg_vol;
	int						latest_cur;
	int						acc_cur;
	int						vol_per;
	int						cur_dep_per;
	int						avg_dep_per;
} shbatt_batt_log_info_t;

typedef struct shbatt_smem_info_tag
{
	unsigned char			traceability_info[22];
} shbatt_smem_info_t;

typedef struct shbatt_boottime_info_tag
{
	int64_t					boot_sec;
	int64_t					boot_nsec;
} shbatt_boottime_info_t;

typedef struct shbatt_log_info_tag
{
	int ocv;		/* OCV estimate */
	int esr;			/* ESR actual */
	int rslow;		/* Rslow */
	int soc_cutoff;		/* SOC Cutoff   */
	int soc_full;		/* SOC Full     */
	int battery_soc;	/* Battery SOC */
	int cc_soc;		/* CC SOC */
	int soc_system;		/* SOC System */
	int soc_monotonic;	/* SOC Monotonic */
	int aging;		/* FG AGING STRAGE */
	int tmp;		/* Battery temperature */
	int vol_shadow;		/* Voltage shadow */
	int cur_shadow;		/* Current shadow */
	int sw_cc_soc;		/* SW CC SOC */
	int v_predicted;	/* V current predicted */
	int batt_info;		/* Latest battery info */
} shbatt_log_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/
shbatt_result_t shbatt_api_get_battery_log_info(
	shbatt_batt_log_info_t*		bli_p );

int sh_fg_get_log_info(shbatt_log_info_t *info);

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt);

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status);

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_event);

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status);

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity);

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type);

bool is_shbatt_prs_launched( void );

void sh_dump_regs(void);

void shbatt_api_cpu_clock_limit_lock(u32 level);

void shbatt_api_cpu_clock_limit_unlock(void);

shbatt_result_t shbatt_api_store_fg_cap_learning_result(
	int64_t learned_cc_uah,
	int nom_cap_uah,
	int high_thresh,
	int low_thresh
);
/*+-----------------------------------------------------------------------------+*/
/*| @ PRIVATE FUNCTION PROTO TYPE DECLARE :                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_KERL_H */
