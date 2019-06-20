/* drivers/sharp/shtps/sy3000/shtps_filter_temperature_ghost_reject_check.c
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
/* -------------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined(SHTPS_TEMPERATURE_GHOST_REJECT_CHECK_ENABLE)
struct shtps_filter_temperature_ghost_reject_info{
	int		is_ghost[SHTPS_FINGER_MAX];
};

/* -------------------------------------------------------------------------- */
void shtps_filter_temperature_ghost_reject_check(struct shtps_rmi_spi *ts, struct shtps_touch_info *info)
{
	int w;
	int i;
	int fingerMax = shtps_get_fingermax(ts);

	if(!SHTPS_TEMPERATURE_GHOST_REJECT_ENABLE){
		return;
	}

	for(i = 0;i < fingerMax;i++){
		if( (ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_FINGER) ||
			(ts->fw_report_info.fingers[i].state == SHTPS_TOUCH_STATE_GLOVE) )
		{
			w = (ts->fw_report_info.fingers[i].wx >= ts->fw_report_info.fingers[i].wy)?
						ts->fw_report_info.fingers[i].wx : ts->fw_report_info.fingers[i].wy;

			if(ts->temperature_ghost_reject_p->is_ghost[i] == 0){
				if(w >= SHTPS_TEMPERATURE_GHOST_REJECT_W_THRESH){
					SHTPS_TEMPERATURE_GHOST_REJECT_PRINT("<%d> detect ghost (w:%d >= %d)\n", i, w, SHTPS_TEMPERATURE_GHOST_REJECT_W_THRESH);
					ts->temperature_ghost_reject_p->is_ghost[i] = 1;
				}
			}

			if(ts->temperature_ghost_reject_p->is_ghost[i] != 0){
				SHTPS_LOG_DBG_PRINT("[temperature ghost] finger[%d] is ghost\n", i);
				info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
			}
		}
		else if(ts->temperature_ghost_reject_p->is_ghost[i] != 0){
			ts->temperature_ghost_reject_p->is_ghost[i] = 0;
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_temperature_ghost_reject_sleep(struct shtps_rmi_spi *ts)
{
	int i, fingerMax = shtps_get_fingermax(ts);

	for(i = 0;i < fingerMax;i++){
		ts->temperature_ghost_reject_p->is_ghost[i] = 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_temperature_ghost_reject_forcetouchup(struct shtps_rmi_spi *ts)
{
	int i, fingerMax = shtps_get_fingermax(ts);

	for(i = 0;i < fingerMax;i++){
		ts->temperature_ghost_reject_p->is_ghost[i] = 0;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_temperature_ghost_reject_init(struct shtps_rmi_spi *ts)
{
	ts->temperature_ghost_reject_p = kzalloc(sizeof(struct shtps_filter_temperature_ghost_reject_info), GFP_KERNEL);
	if(ts->temperature_ghost_reject_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_temperature_ghost_reject_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->temperature_ghost_reject_p)	kfree(ts->temperature_ghost_reject_p);
	ts->temperature_ghost_reject_p = NULL;
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_TEMPERATURE_GHOST_REJECT_CHECK_ENABLE */
