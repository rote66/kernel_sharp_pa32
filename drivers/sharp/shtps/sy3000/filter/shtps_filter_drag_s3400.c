/* drivers/sharp/shtps/sy3000/shtps_filter_drag_s3400.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
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
#include <linux/hrtimer.h>

#include <sharp/shtps_dev.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_log.h"

#include "shtps_filter.h"
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
	struct RawPointerData_Pointer{
		int32_t id;
		int32_t x;
		int32_t y;
		int32_t toolType;
	};
	struct shtps_flattery_history{
		int32_t cnt_available_data;
		int32_t restricted_cnt;
		int32_t coordinates[SHTPS_DRAG_FLATTERY_HISTORY_SIZE_MAX];
		int32_t denominator;
		int32_t pre_comp_history;
		bool skip_comp;
		int real_gap;
	};
#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */

struct shtps_drag_hist{
	int							pre;
	u8							dir;
	u8							count;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int						drag_smooth_slow_mode;
		int						drag_smooth_slow_count;
		unsigned long			drag_smooth_init_judge_starttime;
		int						history[SHTPS_DRAG_HISTORY_SIZE_MAX];
		int						pre_comp_history_FIXED;
		int						history_old;
		int						count_up_base;
		int						history_gap[SHTPS_DRAG_HISTORY_SIZE_MAX];
		int						hull_1st_gap[SHTPS_DRAG_HISTORY_SIZE_MAX];
		bool					skip_comp;
		int						real_gap_FIXED;
		int						hull_ratio;
		int						hull_judge_base_point_x;
		int						hull_judge_base_point_y;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
};

struct shtps_filter_drag_hist_info{
	struct shtps_touch_info		center_info;
	struct shtps_drag_hist	drag_hist[SHTPS_FINGER_MAX][2];
	#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
		struct shtps_flattery_history	flatteryOriginalHistory[2][SHTPS_FINGER_MAX];
	#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */
};

#if defined( SHTPS_DRAG_SMOOTH_ENABLE ) || defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
static int shtps_sqrt(int x);
#endif /* SHTPS_DRAG_SMOOTH_ENABLE || SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */
#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
static void shtps_flattery_PutHistory(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer);
static void shtps_flattery_DragSmoothRevise(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer);
static void shtps_flattery_DragLimitRevise(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer);
static void shtps_flattery_ClearHistory(struct shtps_rmi_spi *ts, int clear_all_flg, int *InUse_flg);
#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */


/* -------------------------------------------------------------------------- */
#if defined( SHTPS_DRAG_SMOOTH_ENABLE ) || defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
static int shtps_sqrt(int x)
{
	int s, t;

	if (x <= 0) return 0;

	s = 1;	t = x;
	while (s < t) { s <<= 1; t >>= 1; }
	do {
		t = s;
		s = (x / s + s) >> 1;
	} while (s < t);

	return t;
}
static int shtps_is_low_report_rate_mode(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CTRL_FW_REPORT_RATE )
		if(SHTPS_CTRL_FW_REPORT_RATE_ENABLE){
			#if defined(SHTPS_DEF_CTRL_FW_REPORT_RATE_LINKED_LCD_BRIGHT_ENABLE)
				if(ts->fw_report_rate_cur == SHTPS_CTRL_FW_REPORT_RATE_PARAM_LCD_BRIGHT_LOW){
					return 1;
				}
			#endif
		}
	#endif /* SHTPS_CTRL_FW_REPORT_RATE */
	return 0;
}
static int shtps_is_high_report_rate_mode(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_CTRL_FW_REPORT_RATE )
		if(SHTPS_CTRL_FW_REPORT_RATE_ENABLE){
			if(ts->fw_report_rate_cur == SHTPS_CTRL_FW_REPORT_RATE_PARAM_HIGH){
				return 1;
			}
		}
	#endif /* SHTPS_CTRL_FW_REPORT_RATE */
	return 0;
}
#endif /* SHTPS_DRAG_SMOOTH_ENABLE || SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */

#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
#if defined( SHTPS_LOG_DEBUG_ENABLE )
int SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(int val)
{
	int i;
	int dec_val = (val) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_FIXED_TO_INT((val)));
	int ret = 0;
	int add_val = 10;
	for(i = 0; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val *= 10;
	}
	for(i = 1; i < SHTPS_DRAG_SMOOTH_FIXED_SHIFT; i++){
		add_val /= 2;
		if((dec_val >> (SHTPS_DRAG_SMOOTH_FIXED_SHIFT - i)) & 0x01){
			ret += add_val;
		}
	}
	return ret/1000000;
}
#endif /* SHTPS_LOG_DEBUG_ENABLE */
static void shtps_filter_pos_compensation(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int xy)
{
	int inc_ave;
	int inc_ave_FIXED,temp_FIXED;
	int i;
	int drag_smooth_leave_max_dot_FIXED;
	int last_history;
	int last_history_FIXED;

	int j;
	int temp_HULL_numerator = 0;
	int temp_HULL_denominator = 0;

	int DRAG_SMOOTH_COUNT_MIN;
	int DRAG_SMOOTH_COUNT_MAX;
	int DRAG_SMOOTH_LEAVE_MAX_DOT;

	for(i = 0;i < shtps_get_fingermax(ts);i++){
		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER /* info->fingers[i].state == SHTPS_TOUCH_STATE_PEN */){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){

				if(ts->drag_hist_p->drag_hist[i][xy].drag_smooth_slow_mode != 1){
					DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_SMOOTH_HULL_COUNT;
					DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_SMOOTH_HULL_COUNT;
					DRAG_SMOOTH_LEAVE_MAX_DOT = SHTPS_DRAG_SMOOTH_HULL_LEAVE_MAX_DOT;
				}
				else{
					DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_SMOOTH_COUNT_MIN;
					DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_SMOOTH_COUNT_MAX;
					DRAG_SMOOTH_LEAVE_MAX_DOT = SHTPS_DRAG_SMOOTH_LEAVE_MAX_DOT;
				}

				if(ts->drag_hist_p->drag_hist[i][xy].count >= DRAG_SMOOTH_COUNT_MIN){
					drag_smooth_leave_max_dot_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(DRAG_SMOOTH_LEAVE_MAX_DOT);
					last_history		= ts->drag_hist_p->drag_hist[i][xy].history[ts->drag_hist_p->drag_hist[i][xy].count - 1];
					last_history_FIXED	= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(last_history);

					if(SHTPS_DRAG_SMOOTH_USE_HULL && (ts->drag_hist_p->drag_hist[i][xy].drag_smooth_slow_mode != 1)){
						// HULL 1st
						for(j = ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 1; j < ts->drag_hist_p->drag_hist[i][xy].count - 1; j++){
							temp_HULL_numerator += ts->drag_hist_p->drag_hist[i][xy].history_gap[j] * (j - (ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 2));
							temp_HULL_denominator += j - (ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 2);
						}
						inc_ave = 2 * temp_HULL_numerator / temp_HULL_denominator;

						temp_HULL_numerator = 0;
						temp_HULL_denominator = 0;

						for(j = 0; j < ts->drag_hist_p->drag_hist[i][xy].count - 1; j++){
							temp_HULL_numerator += ts->drag_hist_p->drag_hist[i][xy].history_gap[j] * (j+1);
							temp_HULL_denominator += j + 1;
						}

						inc_ave = inc_ave - temp_HULL_numerator / temp_HULL_denominator;

						if(SHTPS_DRAG_SMOOTH_HULL_MULTI){
							temp_HULL_numerator = 0;
							temp_HULL_denominator = 0;

							for(j= 0; j < ts->drag_hist_p->drag_hist[i][xy].count-2; j++){
								ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[j] = ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[j+1];
							}
							ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[ts->drag_hist_p->drag_hist[i][xy].count-2] = inc_ave;

							// HULL 2nd
							for(j = ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 1; j < ts->drag_hist_p->drag_hist[i][xy].count - 1; j++){
								temp_HULL_numerator += ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[j] * (j - (ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 2));
								temp_HULL_denominator += j - (ts->drag_hist_p->drag_hist[i][xy].count - ts->drag_hist_p->drag_hist[i][xy].hull_ratio - 2);
							}
							inc_ave = 2 * temp_HULL_numerator / temp_HULL_denominator;

							temp_HULL_numerator = 0;
							temp_HULL_denominator = 0;

							for(j = 0; j < ts->drag_hist_p->drag_hist[i][xy].count - 1; j++){
								temp_HULL_numerator += ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[j] *  (j+1);
								temp_HULL_denominator += j + 1;
							}

							inc_ave = inc_ave - temp_HULL_numerator / temp_HULL_denominator;
						}

						if(xy == SHTPS_POSTYPE_X){
							if(SHTPS_DRAG_SMOOTH_HULL_MULTI){
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_X]HULL_1st=%d HULL_2nd=%d\n", 
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[ts->drag_hist_p->drag_hist[i][xy].count-2],
										inc_ave);
							}else{
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_X]inc_ave=%d\n", 
										inc_ave);
							}
						}else{
							SHTPS_LOG_DRAG_SMOOTH("[DEBUG_Y]real_gap %d %d %d %d %d %d %d %d %d %d\n", 
									ts->drag_hist_p->drag_hist[i][xy].history_gap[0],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[1],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[2],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[3],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[4],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[5],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[6],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[7],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[8],
									ts->drag_hist_p->drag_hist[i][xy].history_gap[9]);

							if(SHTPS_DRAG_SMOOTH_HULL_MULTI){
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_Y]HULL_1st %d %d %d %d %d %d %d %d %d %d\n", 
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[0],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[1],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[2],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[3],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[4],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[5],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[6],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[7],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[8],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[9]);

								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_Y]real_gap=%d HULL_1st=%d HULL_2nd=%d\n", 
										ts->drag_hist_p->drag_hist[i][xy].history_gap[ts->drag_hist_p->drag_hist[i][xy].count-2],
										ts->drag_hist_p->drag_hist[i][xy].hull_1st_gap[ts->drag_hist_p->drag_hist[i][xy].count-2],
										inc_ave);
							}else{
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_Y]real_gap=%d inc_ave=%d\n", 
										ts->drag_hist_p->drag_hist[i][xy].history_gap[ts->drag_hist_p->drag_hist[i][xy].count-2],
										inc_ave);
							}
						}

						inc_ave_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(inc_ave);

						inc_ave_FIXED = ((ts->drag_hist_p->drag_hist[i][xy].dir == SHTPS_DRAG_DIR_PLUS && inc_ave_FIXED < 0) ||
										 (ts->drag_hist_p->drag_hist[i][xy].dir == SHTPS_DRAG_DIR_MINUS && inc_ave_FIXED > 0)) ? 0 : inc_ave_FIXED;
					}
					else{
						inc_ave_FIXED = 
							SHTPS_DRAG_SMOOTH_INT_TO_FIXED(
								last_history- ts->drag_hist_p->drag_hist[i][xy].history[0]) / 
								(ts->drag_hist_p->drag_hist[i][xy].count-1);
					}

					if(xy == SHTPS_POSTYPE_X){
						if(ts->drag_hist_p->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(!ts->drag_hist_p->drag_hist[i][xy].skip_comp || (ts->drag_hist_p->drag_hist[i][xy].drag_smooth_slow_mode == 1)){
								if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
									temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}else if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
									temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}else{
									temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [X]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}
							}else{
								temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + ts->drag_hist_p->drag_hist[i][xy].real_gap_FIXED;
							}

							if(temp_FIXED < 0){
								info->fingers[i].x = 0;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_GET_PANEL_SIZE_X(ts))){
								info->fingers[i].x = SHTPS_GET_PANEL_SIZE_X(ts) -1;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x);
							} else{
								info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist_p->drag_hist[i][xy].history_old = last_history;
						}

						SHTPS_LOG_DRAG_SMOOTH("[DEBUG_X]inc_ave=%d.%03d befor=%d after=%d\n", 
							SHTPS_DRAG_SMOOTH_FIXED_TO_INT(inc_ave_FIXED), 
							SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(inc_ave_FIXED), 
							last_history,
							info->fingers[i].x);

					}else{
						if(ts->drag_hist_p->drag_hist[i][xy].history_old == last_history){
							info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED);
						}else{
							if(!ts->drag_hist_p->drag_hist[i][xy].skip_comp || (ts->drag_hist_p->drag_hist[i][xy].drag_smooth_slow_mode == 1)){
								if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) > drag_smooth_leave_max_dot_FIXED){
									temp_FIXED = last_history_FIXED + drag_smooth_leave_max_dot_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(upper limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}else if(((ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED) - last_history_FIXED ) < (0-drag_smooth_leave_max_dot_FIXED)){
									temp_FIXED = last_history_FIXED - drag_smooth_leave_max_dot_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d(lower limit)\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}else{
									temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + inc_ave_FIXED;
									SHTPS_LOG_DRAG_SMOOTH("   [Y]temp=%d.%03d\n", SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED), SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(temp_FIXED));
								}
							}else{
								temp_FIXED = ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED + ts->drag_hist_p->drag_hist[i][xy].real_gap_FIXED;
							}

							if(temp_FIXED < 0){
								info->fingers[i].y = 0;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = 0;
							}else if(temp_FIXED >= SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_GET_PANEL_SIZE_Y(ts))){
								info->fingers[i].y = SHTPS_GET_PANEL_SIZE_Y(ts) -1;
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y);
							} else{
								info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(temp_FIXED);
								ts->drag_hist_p->drag_hist[i][xy].pre_comp_history_FIXED = temp_FIXED;
							}
							ts->drag_hist_p->drag_hist[i][xy].history_old = last_history;
						}

						SHTPS_LOG_DRAG_SMOOTH("[DEBUG_Y]inc_ave=%d.%03d befor=%d after=%d\n", 
							SHTPS_DRAG_SMOOTH_FIXED_TO_INT(inc_ave_FIXED), 
							SHTPS_DRAG_SMOOTH_GET_DEC_FROM_FIXED(inc_ave_FIXED), 
							last_history,
							info->fingers[i].y);

					}
				}
			}
		}
	}
}
#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
/* -------------------------------------------------------------------------- */
#if defined( SHTPS_DRAG_STEP_ENABLE )
/* -------------------------------------------------------------------------- */
static void shtps_rec_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
//	SHTPS_LOG_FUNC_CALL();
	ts->touch_state.drag_timeout[index][xy] = jiffies + msecs_to_jiffies(ts->touch_state.dragStepReturnTime[index][xy]);
}

/* -------------------------------------------------------------------------- */
static int shtps_chk_notify_time(struct shtps_rmi_spi *ts, int xy, int index)
{
//	SHTPS_LOG_FUNC_CALL();
	if(time_after(jiffies, ts->touch_state.drag_timeout[index][xy])){
		return -1;
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_get_dragstep(struct shtps_rmi_spi *ts, int xy, int type, int fingers, int state)
{
	int dragStep;

	if((SHTPS_DRAG_STEP_FINGER_ENABLE == 0) && (state == SHTPS_TOUCH_STATE_FINGER)){
		return 1;
	}
	if((SHTPS_DRAG_STEP_PEN_ENABLE == 0) && (state == SHTPS_TOUCH_STATE_PEN)){
		return 1;
	}

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_ZERO : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_ZERO : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_ZERO : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_1ST : SHTPS_DRAG_THRESH_VAL_X_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_1ST : SHTPS_PEN_DRAG_THRESH_VAL_X_1ST_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_1ST : SHTPS_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST : SHTPS_PEN_DRAG_THRESH_VAL_Y_1ST_MULTI;
			}
		}
	}else{
		if(xy == SHTPS_POSTYPE_X){
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_X_2ND : SHTPS_DRAG_THRESH_VAL_X_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_X_2ND : SHTPS_PEN_DRAG_THRESH_VAL_X_2ND_MULTI;
			}
		}else{
			if(state == SHTPS_TOUCH_STATE_FINGER){
				dragStep = (fingers <= 1)? SHTPS_DRAG_THRESH_VAL_Y_2ND : SHTPS_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}else{
				dragStep = (fingers <= 1)? SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND : SHTPS_PEN_DRAG_THRESH_VAL_Y_2ND_MULTI;
			}
		}
	}

	return dragStep;
}

/* -------------------------------------------------------------------------- */
static void shtps_set_dragstep(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, int type, int xy, int finger)
{
//	SHTPS_LOG_FUNC_CALL();
	_log_msg_sync( LOGMSG_ID__SET_DRAG_STEP, "%d|%d|%d", type, xy, finger);

	if(type == SHTPS_DRAG_THRESHOLD_ZERO){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
	}else if(type == SHTPS_DRAG_THRESHOLD_1ST){
		ts->touch_state.dragStep[finger][xy] = type;
		ts->touch_state.dragStepReturnTime[finger][xy] = SHTPS_DRAG_THRESH_RETURN_TIME;
	}else{
		if(xy == SHTPS_POSTYPE_X){
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[finger].x);
			ts->drag_hist_p->center_info.fingers[finger].x = info->fingers[finger].x;
		}else{
			_log_msg_sync( LOGMSG_ID__SET_FINGER_CENTER, "%d|%d|%d", xy,
							info->fingers[finger].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[finger].y);
			ts->drag_hist_p->center_info.fingers[finger].y = info->fingers[finger].y;
		}
		ts->touch_state.dragStep[finger][xy] = type;
		shtps_rec_notify_time(ts, xy, finger);
	}
}

/* -------------------------------------------------------------------------- */
#endif /* SHTPS_DRAG_STEP_ENABLE */

/* -------------------------------------------------------------------------- */
static void shtps_init_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
//	SHTPS_LOG_FUNC_CALL();
	ts->drag_hist_p->drag_hist[finger][xy].pre   = pos;
	ts->drag_hist_p->drag_hist[finger][xy].count = 0;
	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		ts->drag_hist_p->drag_hist[finger][xy].count_up_base = 0;
		ts->drag_hist_p->drag_hist[finger][xy].history_old = 0;
		ts->drag_hist_p->drag_hist[finger][xy].drag_smooth_slow_mode = -1;
		ts->drag_hist_p->drag_hist[finger][xy].drag_smooth_slow_count = 0;

		if(SHTPS_DRAG_SMOOTH_USE_HULL){
			ts->drag_hist_p->drag_hist[finger][xy].skip_comp = false;
		}
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
}

/* -------------------------------------------------------------------------- */
static void shtps_add_drag_hist(struct shtps_rmi_spi *ts, int xy, int finger, int pos)
{
	int pre = ts->drag_hist_p->drag_hist[finger][xy].pre;
	u8 dir  = (pos > pre)? SHTPS_DRAG_DIR_PLUS :
			  (pos < pre)? SHTPS_DRAG_DIR_MINUS :
						   SHTPS_DRAG_DIR_NONE;

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		int i;
		int drag_smooth_count_limit;
		int drag_smooth_count_limit_new;
		int DRAG_SMOOTH_COUNT_MIN;
		int DRAG_SMOOTH_COUNT_MAX;
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

//	SHTPS_LOG_FUNC_CALL();
	SHTPS_LOG_DBG_PRINT("add drag hist[%d][%s] pre = %d, cur = %d, dir = %s, cnt = %d, remain time = %d\n",
		finger, (xy == SHTPS_POSTYPE_X)? "X" : "Y",
		pre, pos, 
		(dir == SHTPS_DRAG_DIR_PLUS)? "PLUS" : (dir == SHTPS_DRAG_DIR_MINUS)? "MINUS" : "NONE",
		ts->drag_hist_p->drag_hist[finger][xy].count,
		time_after(jiffies, ts->touch_state.drag_timeout[finger][xy]));

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
		if(ts->drag_hist_p->drag_hist[finger][xy].drag_smooth_slow_mode != 1){
			DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_SMOOTH_HULL_COUNT;
			DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_SMOOTH_HULL_COUNT;
		}
		else{
			DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_SMOOTH_COUNT_MIN;
			DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_SMOOTH_COUNT_MAX;
		}
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

	if(dir != SHTPS_DRAG_DIR_NONE){
		if(ts->drag_hist_p->drag_hist[finger][xy].count == 0){
			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(SHTPS_DRAG_SMOOTH){
					#if defined( SHTPS_DEVELOP_MODE_ENABLE )
						if(DRAG_SMOOTH_COUNT_MIN > SHTPS_DRAG_HISTORY_SIZE_MAX){
							DRAG_SMOOTH_COUNT_MIN = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
						if(DRAG_SMOOTH_COUNT_MAX > SHTPS_DRAG_HISTORY_SIZE_MAX){
							DRAG_SMOOTH_COUNT_MAX = SHTPS_DRAG_HISTORY_SIZE_MAX;
						}
					#endif /* #if defined( SHTPS_DEVELOP_MODE_ENABLE ) */
					ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count] = pos;
					ts->drag_hist_p->drag_hist[finger][xy].history_old = pos;
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			ts->drag_hist_p->drag_hist[finger][xy].dir   = dir;
			ts->drag_hist_p->drag_hist[finger][xy].count = 1;

			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(SHTPS_DRAG_SMOOTH_USE_HULL){
					if(xy == SHTPS_POSTYPE_X){
						ts->drag_hist_p->drag_hist[finger][xy].hull_judge_base_point_x = pos;
					}else{
						ts->drag_hist_p->drag_hist[finger][xy].hull_judge_base_point_y = pos;
					}
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
		}else{

			#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
				if(SHTPS_DRAG_SMOOTH){
					drag_smooth_count_limit = ts->drag_hist_p->drag_hist[finger][xy].count;
					drag_smooth_count_limit_new = 
						DRAG_SMOOTH_COUNT_MIN + 
						(ts->drag_hist_p->drag_hist[finger][xy].count_up_base / SHTPS_DRAG_SMOOTH_COUNT_UP_STEP);
					if(drag_smooth_count_limit < drag_smooth_count_limit_new){
						drag_smooth_count_limit = drag_smooth_count_limit_new;
					}
					if(drag_smooth_count_limit > DRAG_SMOOTH_COUNT_MAX){
						drag_smooth_count_limit = DRAG_SMOOTH_COUNT_MAX;
					}

					if(ts->drag_hist_p->drag_hist[finger][xy].dir != dir){
						drag_smooth_count_limit = DRAG_SMOOTH_COUNT_MIN;
						if(drag_smooth_count_limit < ts->drag_hist_p->drag_hist[finger][xy].count){
							for(i= 0; i < drag_smooth_count_limit; i++){
								ts->drag_hist_p->drag_hist[finger][xy].history[i] = 
									ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count - 
									drag_smooth_count_limit + i];
							}
							ts->drag_hist_p->drag_hist[finger][xy].count = drag_smooth_count_limit;
						}

						ts->drag_hist_p->drag_hist[finger][xy].count_up_base = 0;
					}
					
					if(ts->drag_hist_p->drag_hist[finger][xy].count < DRAG_SMOOTH_COUNT_MIN-1){
						if(SHTPS_DRAG_SMOOTH_USE_HULL){
							ts->drag_hist_p->drag_hist[finger][xy].history_gap[ts->drag_hist_p->drag_hist[finger][xy].count-1] = pos - ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count-1];
							if(SHTPS_DRAG_SMOOTH_HULL_MULTI){
								ts->drag_hist_p->drag_hist[finger][xy].hull_1st_gap[ts->drag_hist_p->drag_hist[finger][xy].count-1] = pos - ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count-1];
							}
						}
						ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count] = pos;
						ts->drag_hist_p->drag_hist[finger][xy].pre_comp_history_FIXED = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(pos);
						ts->drag_hist_p->drag_hist[finger][xy].history_old = pos;
						ts->drag_hist_p->drag_hist[finger][xy].count++;
					}else{

						if(ts->drag_hist_p->drag_hist[finger][xy].count == drag_smooth_count_limit-1){
							if(SHTPS_DRAG_SMOOTH_USE_HULL){
								ts->drag_hist_p->drag_hist[finger][xy].history_gap[ts->drag_hist_p->drag_hist[finger][xy].count-1] = pos - ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count-1];
								if(SHTPS_DRAG_SMOOTH_HULL_MULTI){
									ts->drag_hist_p->drag_hist[finger][xy].hull_1st_gap[ts->drag_hist_p->drag_hist[finger][xy].count-1] = pos - ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count-1];
								}
							}
							ts->drag_hist_p->drag_hist[finger][xy].count++;
						}else{
							for(i= 0; i < drag_smooth_count_limit-1; i++){
								ts->drag_hist_p->drag_hist[finger][xy].history[i] = 
									ts->drag_hist_p->drag_hist[finger][xy].history[i+1];
							}

							if(SHTPS_DRAG_SMOOTH_USE_HULL){
								for(i= 0; i < drag_smooth_count_limit-2; i++){
									ts->drag_hist_p->drag_hist[finger][xy].history_gap[i] = 
										ts->drag_hist_p->drag_hist[finger][xy].history_gap[i+1];
								}
								ts->drag_hist_p->drag_hist[finger][xy].history_gap[drag_smooth_count_limit-2] = pos - ts->drag_hist_p->drag_hist[finger][xy].history[ts->drag_hist_p->drag_hist[finger][xy].count-2];

								if(shtps_sqrt(ts->drag_hist_p->drag_hist[finger][xy].history_gap[drag_smooth_count_limit-2] * ts->drag_hist_p->drag_hist[finger][xy].history_gap[drag_smooth_count_limit-2]) < SHTPS_DRAG_SMOOTH_HULL_RATIO_CHANGE_THRESHOLD){
									ts->drag_hist_p->drag_hist[finger][xy].hull_ratio = SHTPS_DRAG_SMOOTH_HULL_LOW_SPEED_RATIO;
								}else{
									ts->drag_hist_p->drag_hist[finger][xy].hull_ratio = SHTPS_DRAG_SMOOTH_HULL_HIGH_SPEED_RATIO;
								}
							}
						}
						ts->drag_hist_p->drag_hist[finger][xy].history[drag_smooth_count_limit-1] = pos;
						ts->drag_hist_p->drag_hist[finger][xy].count_up_base++;
					}
				}
			#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

			#if defined( SHTPS_DRAG_STEP_ENABLE )
				if(ts->drag_hist_p->drag_hist[finger][xy].dir == dir){
					#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
						if(!SHTPS_DRAG_SMOOTH){
							if(ts->drag_hist_p->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
								ts->drag_hist_p->drag_hist[finger][xy].count++;
							}
						}
					#else
					if(ts->drag_hist_p->drag_hist[finger][xy].count < SHTPS_DRAG_DIR_FIX_CNT){
						ts->drag_hist_p->drag_hist[finger][xy].count++;
					}
					#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

					if(ts->drag_hist_p->drag_hist[finger][xy].count >= SHTPS_DRAG_DIR_FIX_CNT &&
							ts->touch_state.dragStep[finger][xy] == SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_rec_notify_time(ts, xy, finger);
						if(xy == SHTPS_POSTYPE_X){
							ts->drag_hist_p->center_info.fingers[finger].x = pos;
						}else{
							ts->drag_hist_p->center_info.fingers[finger].y = pos;
						}
						SHTPS_LOG_DBG_PRINT("update center pos(%d, %d) time=%lu\n",
									ts->drag_hist_p->center_info.fingers[finger].x, ts->drag_hist_p->center_info.fingers[finger].y,
									ts->touch_state.drag_timeout[finger][xy]);
					}
				}else{
					#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
						if(!SHTPS_DRAG_SMOOTH){
							ts->drag_hist_p->drag_hist[finger][xy].count = 1;
						}
					#else
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
				}
			#else
				#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
					if(!SHTPS_DRAG_SMOOTH){
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					}
				#else
					if(ts->drag_hist_p->drag_hist[finger][xy].dir != dir){
						ts->drag_hist_p->drag_hist[finger][xy].count = 1;
					}
				#endif /* SHTPS_DRAG_SMOOTH_ENABLE */
			#endif /* SHTPS_DRAG_STEP_ENABLE */

			ts->drag_hist_p->drag_hist[finger][xy].dir = dir;
		}

		ts->drag_hist_p->drag_hist[finger][xy].pre = pos;
	}
}

#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
static void shtps_flattery_RevisePointFromHistory(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer){

	//*****************************************************//
	// 1)Keep the latest raw point into "flatteryOriginalHistory".//
	//*****************************************************//
	shtps_flattery_PutHistory(ts, inPointer);

	//*****************************************************//
	// 2)Smooth-revision:                                  //
	//-----------------------------------------------------//
	//   Revise the point to improve the drag smoother.    //
	//*****************************************************//
	shtps_flattery_DragSmoothRevise(ts, inPointer);

	//*****************************************************//
	// 3)Check result                                      //
	//-----------------------------------------------------//
	//   Revise the point to the upper & bottom limit.     //
	//*****************************************************//
	shtps_flattery_DragLimitRevise(ts, inPointer);

	return;
}

static void shtps_flattery_PutHistory(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer){

	struct shtps_flattery_history *pOriginalHistory;
	int32_t xy;
	int32_t *input_coordinate;
	int32_t weighted_moving_ave_denominator;

	if(shtps_is_high_report_rate_mode(ts) != 0){
		weighted_moving_ave_denominator = SHTPS_DRAG_SMOOTH_FLATTERY_WEIGHTED_MOVING_AVE_DENOMINATOR_HIGH_REPORT_RATE;
	}
	else{
		weighted_moving_ave_denominator = SHTPS_DRAG_SMOOTH_FLATTERY_WEIGHTED_MOVING_AVE_DENOMINATOR;
	}

	for(xy = 0 ; xy < 2 ; xy++){
		pOriginalHistory = &ts->drag_hist_p->flatteryOriginalHistory[xy][inPointer->id];
		if(xy == SHTPS_POSTYPE_X){
			input_coordinate = &(inPointer->x);
		}else{
			input_coordinate = &(inPointer->y);
		}
		
		// Keep the raw point in "flatteryOriginalHistory".
		//[0]~[4] : Keep the point with no restriction.
		if(pOriginalHistory->cnt_available_data < SHTPS_DRAG_SMOOTH_FLATTERY_SMOOTH_SAMPLE_COUNT){
			if(pOriginalHistory->cnt_available_data == 0){
				pOriginalHistory->denominator = weighted_moving_ave_denominator + 1;
			}
			pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data] = *input_coordinate;
			pOriginalHistory->cnt_available_data++;
			if(pOriginalHistory->cnt_available_data != SHTPS_DRAG_SMOOTH_FLATTERY_SMOOTH_SAMPLE_COUNT){
				pOriginalHistory->pre_comp_history = *input_coordinate;
			}
		}else{
			//[5]~	  : Keep the point within the restriction.
			int i = 0;
			for(i = 0; i < (pOriginalHistory->cnt_available_data - 1); i++){
				pOriginalHistory->coordinates[i] = pOriginalHistory->coordinates[i+1];
			}
			pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1] = *input_coordinate;
		}
	}

	return;
}

static void shtps_flattery_DragSmoothRevise(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer){

	struct shtps_flattery_history *pOriginalHistory;
	int32_t xy;
	int32_t *input_coordinate;
	int32_t wma;
	int32_t i;
	int32_t Recent_direction;
	int32_t Flattery_point;
	int32_t flattery_ratio;

	if(shtps_is_high_report_rate_mode(ts) != 0){
		flattery_ratio = SHTPS_DRAG_SMOOTH_FLATTERY_RATIO_HIGH_REPORT_RATE;
	}
	else{
		flattery_ratio = SHTPS_DRAG_SMOOTH_FLATTERY_RATIO;
	}

	for(xy = 0 ; xy < 2 ; xy++){
		pOriginalHistory = &ts->drag_hist_p->flatteryOriginalHistory[xy][inPointer->id];
		wma = 0;
		if(xy == SHTPS_POSTYPE_X){
			input_coordinate = &(inPointer->x);
		}else{
			input_coordinate = &(inPointer->y);
		}

		if(pOriginalHistory->skip_comp){
			*input_coordinate = pOriginalHistory->pre_comp_history + pOriginalHistory->real_gap;

			if(xy == SHTPS_POSTYPE_X){
				SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[DEBUG_FLATTERY_X]exclude flattery : gap=%d *input_coordinate=%d\n", 
									pOriginalHistory->real_gap, 
									*input_coordinate);
			}else{
				SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[DEBUG_FLATTERY_Y]exclude flattery : gap=%d *input_coordinate=%d\n", 
									pOriginalHistory->real_gap, 
									*input_coordinate);
			}
			continue;
		}

		if(pOriginalHistory->cnt_available_data >= SHTPS_DRAG_SMOOTH_FLATTERY_SMOOTH_SAMPLE_COUNT){
			if(flattery_ratio < pOriginalHistory->denominator){
				pOriginalHistory->denominator--;
			}else if(flattery_ratio >= pOriginalHistory->denominator){
				pOriginalHistory->denominator = flattery_ratio;
			}
			
			for(i=0 ; i < pOriginalHistory->cnt_available_data - 1 ; i++){
				wma = wma + ((pOriginalHistory->coordinates[i+1] - pOriginalHistory->coordinates[i]) * (i+1));
			}

			SHTPS_LOG_DRAG_SMOOTH_FLATTERY("xy=%d, wma=%d, denominator=%d\n",xy ,wma ,pOriginalHistory->denominator);

			Flattery_point = pOriginalHistory->pre_comp_history + (int32_t)(wma / pOriginalHistory->denominator);

			//	3-1)calc direction
			if(pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1] > pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 2]){
				Recent_direction = SHTPS_DRAG_DIR_PLUS;
			}else if(pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1] < pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 2]){
				Recent_direction = SHTPS_DRAG_DIR_MINUS;
			}else{
			}
			

			//	3-2)judge direction
			if(((Recent_direction == SHTPS_DRAG_DIR_PLUS)&&(Flattery_point < pOriginalHistory->pre_comp_history))
			 ||((Recent_direction == SHTPS_DRAG_DIR_MINUS)&&(Flattery_point > pOriginalHistory->pre_comp_history))){

				Flattery_point = pOriginalHistory->pre_comp_history;
			}
			
			*input_coordinate = Flattery_point;
		}
	}

	return;
}

static void shtps_flattery_DragLimitRevise(struct shtps_rmi_spi *ts, struct RawPointerData_Pointer *inPointer){

	struct shtps_flattery_history *pOriginalHistory;
	int32_t xy;
	int32_t *input_coordinate;
	int32_t disp_size;

	int32_t diagonal_distance = 0;

	for(xy = 0 ; xy < 2 ; xy++){
		pOriginalHistory = &ts->drag_hist_p->flatteryOriginalHistory[xy][inPointer->id];
		if(xy == SHTPS_POSTYPE_X){
			input_coordinate = &(inPointer->x);
			disp_size = SHTPS_GET_PANEL_SIZE_X(ts);
		}else{
			input_coordinate = &(inPointer->y);
			disp_size = SHTPS_GET_PANEL_SIZE_Y(ts);
		}

		// 1)IF the revised point(inPointer->x,y) is too far from real point,
		//	 revise the point with the "max_leave_dot".
		if((*input_coordinate - pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1])
			> SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT){

			*input_coordinate = pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1] + SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT;

		}else if((*input_coordinate - pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1])
			< (-1 * SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT)){

			*input_coordinate = pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1] - SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT;

		}else{
			// Need no revision.
		}

		// 2)IF revised point go out of the display, suit the point to the size of display.
		if(*input_coordinate < 0){
			*input_coordinate = 0;
		}else if(*input_coordinate > (disp_size - 1)){
			*input_coordinate = disp_size - 1;
		}else{
			// Need no revision.
		}

		pOriginalHistory->pre_comp_history = *input_coordinate;

		diagonal_distance += (*input_coordinate - pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1]) * (*input_coordinate - pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1]);

		if(xy == SHTPS_POSTYPE_X){
			SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[Touch_Flattery_X] before=%d after=%d\n", pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1], *input_coordinate);
		}else{
			SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[Touch_Flattery_Y] before=%d after=%d\n", pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1], *input_coordinate);
		}
	}

	SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[Touch_Flattery_D] diagonal_distance=%d diagonal_d=%d\n", diagonal_distance, (int32_t)(shtps_sqrt(diagonal_distance)));

	if((int32_t)(shtps_sqrt(diagonal_distance)) > SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT){
		for(xy = 0 ; xy < 2 ; xy++){
			pOriginalHistory = &ts->drag_hist_p->flatteryOriginalHistory[xy][inPointer->id];
			if(xy == SHTPS_POSTYPE_X){
				input_coordinate = &(inPointer->x);
			}else{
				input_coordinate = &(inPointer->y);
			}

			*input_coordinate = pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1]
									+ (*input_coordinate - pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1])
										* SHTPS_DRAG_SMOOTH_FLATTERY_LEAVE_MAX_DOT / (int32_t)(shtps_sqrt(diagonal_distance));

			pOriginalHistory->pre_comp_history = *input_coordinate;

			if(xy == SHTPS_POSTYPE_X){
				SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[Touch_Flattery_X_D] before=%d D_Revise=%d\n", pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1], *input_coordinate);
			}else{
				SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[Touch_Flattery_Y_D] before=%d D_Revise=%d\n", pOriginalHistory->coordinates[pOriginalHistory->cnt_available_data - 1], *input_coordinate);
			}
		}
	}

	return;
}

static void shtps_flattery_ClearHistory(struct shtps_rmi_spi *ts, int clear_all_flg, int *InUse_flg){
	int i = 0, xy = 0;

	for(xy = 0; xy < 2; xy++){
		for(i = 0; i < SHTPS_FINGER_MAX; i++){
			if(clear_all_flg == 0){
				if(InUse_flg[i] == 1){continue;}
			}
			ts->drag_hist_p->flatteryOriginalHistory[xy][i].cnt_available_data = 0;
		}
	}
}
#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */

/* -------------------------------------------------------------------------- */
static int shtps_apply_dragstep(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 *event)
{
#if defined( SHTPS_DRAG_STEP_ENABLE )
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	int 	diff_x;
	int 	diff_y;
	int 	diff_cx;
	int 	diff_cy;
	int		dragStep1stX;
	int		dragStep1stY;
	int		FingerDragStep1stX;
	int		FingerDragStep1stY;
	int		PenDragStep1stX;
	int		PenDragStep1stY;
	int		dragStepCurX;
	int		dragStepCurY;
	int		numOfFingers = 0;
	int		numOfPen = 0;

	if( (SHTPS_DRAG_STEP_FINGER_ENABLE == 0) && (SHTPS_DRAG_STEP_PEN_ENABLE == 0) ){
		return -1;
	}

	for (i = 0; i < fingerMax; i++) {
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
				numOfFingers++;
			}else if(info->fingers[i].state == SHTPS_TOUCH_STATE_PEN){
				numOfPen++;
			}
		}
	}

	FingerDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_TOUCH_STATE_FINGER);
	FingerDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfFingers, SHTPS_TOUCH_STATE_FINGER);

	PenDragStep1stX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_TOUCH_STATE_PEN);
	PenDragStep1stY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, SHTPS_DRAG_THRESHOLD_1ST, numOfPen, SHTPS_TOUCH_STATE_PEN);

	for(i = 0;i < fingerMax;i++){
		_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
							info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
							info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
							info->fingers[i].x,
							info->fingers[i].y,
							info->fingers[i].wx,
							info->fingers[i].wy,
							info->fingers[i].z);

		if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
			dragStep1stX = FingerDragStep1stX;
			dragStep1stY = FingerDragStep1stY;
			dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
			dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
		}else{
			dragStep1stX = PenDragStep1stX;
			dragStep1stY = PenDragStep1stY;
			dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X, ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_TOUCH_STATE_PEN);
			dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y, ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_TOUCH_STATE_PEN);
		}

		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){

			diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));
			diff_cx= shtps_get_diff(info->fingers[i].x, ts->drag_hist_p->center_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
			diff_cy= shtps_get_diff(info->fingers[i].y, ts->drag_hist_p->center_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(diff_cy >= dragStep1stY){
					if(ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);
						if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
						}else{
							dragStepCurX = shtps_get_dragstep(ts, SHTPS_POSTYPE_X,
												ts->touch_state.dragStep[i][SHTPS_POSTYPE_X], numOfPen, SHTPS_TOUCH_STATE_PEN);
						}
					}
				}

				if(diff_x >= dragStepCurX){
					if(diff_cx >= dragStep1stX){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						if(ts->touch_state.dragStep[i][0] != SHTPS_DRAG_THRESHOLD_2ND){
							shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);
							if(info->fingers[i].state == SHTPS_TOUCH_STATE_FINGER){
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfFingers, SHTPS_TOUCH_STATE_FINGER);
							}else{
								dragStepCurY = shtps_get_dragstep(ts, SHTPS_POSTYPE_Y,
													ts->touch_state.dragStep[i][SHTPS_POSTYPE_Y], numOfPen, SHTPS_TOUCH_STATE_PEN);
							}
						}
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_X, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_X, i) == 0 ||
								ts->touch_state.dragStep[i][SHTPS_POSTYPE_X] != SHTPS_DRAG_THRESHOLD_2ND){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);

					}else{
						info->fingers[i].x = ts->report_info.fingers[i].x;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_X, i);
					}
				}else{
					info->fingers[i].x = ts->report_info.fingers[i].x;
				}

				if(diff_y >= dragStepCurY){
					if(diff_cy >= dragStep1stY){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_2ND, SHTPS_POSTYPE_Y, i);

					}else if(shtps_chk_notify_time(ts, SHTPS_POSTYPE_Y, i) == 0 ||
								ts->touch_state.dragStep[i][1] != SHTPS_DRAG_THRESHOLD_2ND)
					{
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
						ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_2ND;

					}else{
						info->fingers[i].y = ts->report_info.fingers[i].y;
						shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_1ST, SHTPS_POSTYPE_Y, i);
					}
				}else{
					info->fingers[i].y = ts->report_info.fingers[i].y;
				}
			}else{
				ts->drag_hist_p->center_info.fingers[i].x = info->fingers[i].x;
				ts->drag_hist_p->center_info.fingers[i].y = info->fingers[i].y;
			}
		}else{
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_X, i);
			shtps_set_dragstep(ts, info, SHTPS_DRAG_THRESHOLD_ZERO, SHTPS_POSTYPE_Y, i);
		}

		if(info->fingers[i].state != ts->report_info.fingers[i].state){
			shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
		}
	}

	return 0;
#else
	return (-1);
#endif /* SHTPS_DRAG_STEP_ENABLE */

}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag(struct shtps_rmi_spi *ts, struct shtps_touch_info *info, u8 *event)
{
	int		i;
	int		fingerMax = shtps_get_fingermax(ts);
	int 	diff_x;
	int 	diff_y;
	int		ret;

	SHTPS_LOG_FUNC_CALL();

	for(i = 0;i < fingerMax;i++){
		if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
			if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
				shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
			}
		}else{
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
			shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
		}
	}

	#if defined( SHTPS_DRAG_SMOOTH_ENABLE )
	if(SHTPS_DRAG_SMOOTH){
		int	before_x_fixed[SHTPS_FINGER_MAX];
		int	before_y_fixed[SHTPS_FINGER_MAX];
		int	after_x_fixed[SHTPS_FINGER_MAX];
		int	after_y_fixed[SHTPS_FINGER_MAX];
		int	diagonal_fixed[SHTPS_FINGER_MAX];

		int	x_gap_fixed[SHTPS_FINGER_MAX];
		int	y_gap_fixed[SHTPS_FINGER_MAX];
		int	hull_cancel_judge_area;
		int	x_gap_from_base_point_fixed[SHTPS_FINGER_MAX];
		int	y_gap_from_base_point_fixed[SHTPS_FINGER_MAX];
		int x_diff, y_diff;
		int exclude_dot;


		if(SHTPS_DRAG_SMOOTH_USE_HULL){
			hull_cancel_judge_area = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_HULL_CANCEL_JUDGE_AREA);

			for(i = 0;i < fingerMax;i++){
				before_x_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x);
				before_y_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y);
			}

			for(i = 0;i < fingerMax;i++){
				if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					if(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].count > 0){
						x_gap_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].history_old);
						x_gap_from_base_point_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].hull_judge_base_point_x);
					}
					else{
						x_gap_fixed[i] = 0;
						x_gap_from_base_point_fixed[i] = 0;
					}
					if(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].count > 0){
						y_gap_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].history_old);
						y_gap_from_base_point_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y) - SHTPS_DRAG_SMOOTH_INT_TO_FIXED(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].hull_judge_base_point_y);
					}
					else{
						y_gap_fixed[i] = 0;
						y_gap_from_base_point_fixed[i] = 0;
					}

					ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].real_gap_FIXED = x_gap_fixed[i];
					ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].real_gap_FIXED = y_gap_fixed[i];

					SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH]base_point_gap_x[%d] = %3d mode=%s%s\n", i, SHTPS_DRAG_SMOOTH_FIXED_TO_INT(x_gap_from_base_point_fixed[i]),
																					(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode == 0 ? "HULL" : (ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode == 1 ? "slow" : "pending")),
																					(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode == 0 ? (ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].skip_comp ? " skip_comp=true" : " skip_comp=false") : ""));
					SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH]base_point_gap_y[%d] = %3d mode=%s%s\n", i, SHTPS_DRAG_SMOOTH_FIXED_TO_INT(y_gap_from_base_point_fixed[i]),
																					(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode == 0 ? "HULL" : (ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode == 1 ? "slow" : "pending")),
																					(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode == 0 ? (ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].skip_comp ? " skip_comp=true" : " skip_comp=false") : ""));

					if( (x_gap_from_base_point_fixed[i] > hull_cancel_judge_area ||
						 x_gap_from_base_point_fixed[i] < (0-hull_cancel_judge_area) ||
						 y_gap_from_base_point_fixed[i] > hull_cancel_judge_area ||
						 y_gap_from_base_point_fixed[i] < (0-hull_cancel_judge_area)) && 
						 !ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].skip_comp &&
						 !ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].skip_comp){

						if( x_gap_from_base_point_fixed[i] * y_gap_from_base_point_fixed[i] != 0 &&
							(y_gap_from_base_point_fixed[i] / x_gap_from_base_point_fixed[i] > (0-SHTPS_DRAG_SMOOTH_HULL_CANCEL_THRESHOLD)) &&
							(y_gap_from_base_point_fixed[i] / x_gap_from_base_point_fixed[i] < SHTPS_DRAG_SMOOTH_HULL_CANCEL_THRESHOLD) &&
							(x_gap_from_base_point_fixed[i] / y_gap_from_base_point_fixed[i] > (0-SHTPS_DRAG_SMOOTH_HULL_CANCEL_THRESHOLD)) &&
							(x_gap_from_base_point_fixed[i] / y_gap_from_base_point_fixed[i] < SHTPS_DRAG_SMOOTH_HULL_CANCEL_THRESHOLD) ){
							SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH]skip_comp[%d] = true\n", i);
							ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].skip_comp = true;
							ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].skip_comp = true;
						}

						SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH]rebase_point_x[%d] = %3d\n", i, info->fingers[i].x);
						SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH]rebase_point_y[%d] = %3d\n", i, info->fingers[i].y);
						ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].hull_judge_base_point_x = info->fingers[i].x;
						ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].hull_judge_base_point_y = info->fingers[i].y;

					}

					if((ts->fw_report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH) &&
					   (ts->fw_report_info_store.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH)){
						x_diff = SHTPS_ABS_CALC(ts->fw_report_info.fingers[i].x, ts->fw_report_info_store.fingers[i].x);
						y_diff = SHTPS_ABS_CALC(ts->fw_report_info.fingers[i].y, ts->fw_report_info_store.fingers[i].y);
					}
					else{
						x_diff = 0;
						y_diff = 0;
					}

					if(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode == -1 ||
					   ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode == -1){

						if((x_diff != 0) || (y_diff != 0)){
							if(shtps_is_high_report_rate_mode(ts) != 0){
								exclude_dot = SHTPS_DRAG_SMOOTH_EXCLUDE_DOT_HIGH_REPORT_RATE;
							}
							else{
								exclude_dot = SHTPS_DRAG_SMOOTH_EXCLUDE_DOT;
							}
							if((x_diff < exclude_dot) && (y_diff < exclude_dot)){

								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_count++;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_count++;
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH] finger[%d] check slow mode (count = %d)\n", i, ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_count);

								if(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_count >= SHTPS_DRAG_SMOOTH_SLOW_JUDGE_COUNT ||
								   ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_count >= SHTPS_DRAG_SMOOTH_SLOW_JUDGE_COUNT){

									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode = 1;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode = 1;
									SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH] finger[%d] change to slow mode (count = %d)\n", i, ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_count);

									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].count = 0;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].count_up_base = 0;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].count = 0;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].count_up_base = 0;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_init_judge_starttime = jiffies;
									ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_init_judge_starttime = jiffies;
								}
							}
							else{
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode = 0;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode = 0;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_init_judge_starttime = jiffies;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_init_judge_starttime = jiffies;
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH] finger[%d] change to fast mode\n", i);
							}
						}
					}
					else if(ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode == 1 ||
						    ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode == 1){
						if((x_diff != 0) || (y_diff != 0)){
							if(time_after(jiffies, ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_init_judge_starttime + msecs_to_jiffies(SHTPS_DRAG_SMOOTH_INIT_JUDGE_TIME_MS)) ){
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode = -1;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode = -1;
								SHTPS_LOG_DRAG_SMOOTH("[DEBUG_SMOOTH] finger[%d] init slow/fast mode (check time = %d)\n", i, SHTPS_DRAG_SMOOTH_INIT_JUDGE_TIME_MS);

								shtps_init_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
								shtps_init_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);

								shtps_add_drag_hist(ts, SHTPS_POSTYPE_X, i, info->fingers[i].x);
								shtps_add_drag_hist(ts, SHTPS_POSTYPE_Y, i, info->fingers[i].y);
							}
							else{
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_init_judge_starttime = jiffies;
								ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_init_judge_starttime = jiffies;
							}
						}
					}
				}
			}
		}

		shtps_filter_pos_compensation(ts, info, SHTPS_POSTYPE_X);
		shtps_filter_pos_compensation(ts, info, SHTPS_POSTYPE_Y);

		if(SHTPS_DRAG_SMOOTH_USE_HULL){
			for(i = 0;i < fingerMax;i++){
				if((ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_X].drag_smooth_slow_mode != 1) &&
				   (ts->drag_hist_p->drag_hist[i][SHTPS_POSTYPE_Y].drag_smooth_slow_mode != 1)){
					after_x_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].x);
					after_y_fixed[i] = SHTPS_DRAG_SMOOTH_INT_TO_FIXED(info->fingers[i].y);
					diagonal_fixed[i] = shtps_sqrt((after_x_fixed[i] - before_x_fixed[i]) * (after_x_fixed[i] - before_x_fixed[i]) +
						(after_y_fixed[i] - before_y_fixed[i]) * (after_y_fixed[i] - before_y_fixed[i]));

					if (diagonal_fixed[i] > SHTPS_DRAG_SMOOTH_INT_TO_FIXED(SHTPS_DRAG_SMOOTH_HULL_LEAVE_MAX_DOT)) {
						info->fingers[i].x = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(before_x_fixed[i] + ((after_x_fixed[i] - before_x_fixed[i]) /
							SHTPS_DRAG_SMOOTH_FIXED_TO_INT(diagonal_fixed[i]) *
							SHTPS_DRAG_SMOOTH_HULL_LEAVE_MAX_DOT));
						info->fingers[i].y = SHTPS_DRAG_SMOOTH_FIXED_TO_INT(before_y_fixed[i] + ((after_y_fixed[i] - before_y_fixed[i]) /
							SHTPS_DRAG_SMOOTH_FIXED_TO_INT(diagonal_fixed[i]) *
							SHTPS_DRAG_SMOOTH_HULL_LEAVE_MAX_DOT));
					}
				}
			}
		}
	}
	#endif /* SHTPS_DRAG_SMOOTH_ENABLE */

	#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
	if(SHTPS_DRAG_SMOOTH_FLATTERY){
		int	x_gap_flattery[SHTPS_FINGER_MAX];
		int	y_gap_flattery[SHTPS_FINGER_MAX];
		int	diagonal_gap_flattery[SHTPS_FINGER_MAX];
		struct shtps_flattery_history *pOriginalHistory_x;
		struct shtps_flattery_history *pOriginalHistory_y;
		int	flattery_exclude_dot;

		if(shtps_is_high_report_rate_mode(ts) != 0){
			flattery_exclude_dot = SHTPS_DRAG_SMOOTH_FLATTERY_EXCLUDE_DOT_HIGH_REPORT_RATE;
		}
		else if(shtps_is_low_report_rate_mode(ts) != 0){
			flattery_exclude_dot = SHTPS_DRAG_SMOOTH_FLATTERY_EXCLUDE_DOT_LOW_REPORT_RATE;
		}
		else{
			flattery_exclude_dot = SHTPS_DRAG_SMOOTH_FLATTERY_EXCLUDE_DOT;
		}

		for(i = 0;i < fingerMax;i++){
			struct RawPointerData_Pointer outPointer;

			outPointer.x = info->fingers[i].x;
			outPointer.y = info->fingers[i].y;
			outPointer.id = i;
			outPointer.toolType = info->fingers[i].state;

			if(outPointer.toolType != SHTPS_TOUCH_STATE_NO_TOUCH){

				pOriginalHistory_x = &ts->drag_hist_p->flatteryOriginalHistory[SHTPS_POSTYPE_X][i];
				pOriginalHistory_y = &ts->drag_hist_p->flatteryOriginalHistory[SHTPS_POSTYPE_Y][i];

				if(pOriginalHistory_x->cnt_available_data > 0){
					x_gap_flattery[i] = info->fingers[i].x - pOriginalHistory_x->coordinates[pOriginalHistory_x->cnt_available_data - 1];
					y_gap_flattery[i] = info->fingers[i].y - pOriginalHistory_y->coordinates[pOriginalHistory_y->cnt_available_data - 1];
					diagonal_gap_flattery[i] = shtps_sqrt((x_gap_flattery[i] * x_gap_flattery[i]) + (y_gap_flattery[i] * y_gap_flattery[i]));
				}
				else{
					x_gap_flattery[i] = 0;
					y_gap_flattery[i] = 0;
					diagonal_gap_flattery[i] = 0;
				}

				SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[DEBUG_FLATTERY]diagonal_gap = %d\n", diagonal_gap_flattery[i]);

				pOriginalHistory_x->skip_comp = false;
				pOriginalHistory_y->skip_comp = false;
				if(pOriginalHistory_x->cnt_available_data >= SHTPS_DRAG_SMOOTH_FLATTERY_SMOOTH_SAMPLE_COUNT){
					if((diagonal_gap_flattery[i] > 0-flattery_exclude_dot) && (diagonal_gap_flattery[i] < flattery_exclude_dot)){
						SHTPS_LOG_DRAG_SMOOTH_FLATTERY("[DEBUG_FLATTERY]skip_comp = true\n");
						pOriginalHistory_x->skip_comp = true;
						pOriginalHistory_y->skip_comp = true;
						pOriginalHistory_x->real_gap = x_gap_flattery[i];
						pOriginalHistory_y->real_gap = y_gap_flattery[i];
					}
				}

				shtps_flattery_RevisePointFromHistory(ts, &outPointer);

				info->fingers[i].x = outPointer.x;
				info->fingers[i].y = outPointer.y;
			}
			else{
				ts->drag_hist_p->flatteryOriginalHistory[SHTPS_POSTYPE_X][i].cnt_available_data = 0;
				ts->drag_hist_p->flatteryOriginalHistory[SHTPS_POSTYPE_Y][i].cnt_available_data = 0;
			}

		}
	}
	#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */

	/* apply dragstep and set event type */
	ret = shtps_apply_dragstep(ts, info, event);
	if(ret < 0){
		/* set event type */
		for(i = 0;i < fingerMax;i++){
			_log_msg_sync( LOGMSG_ID__FW_EVENT, "%d|%d|%d|%d|%d|%d|%d|%d|%d", i, info->fingers[i].state,
								info->fingers[i].x * SHTPS_POS_SCALE_X(ts) / 10000,
								info->fingers[i].y * SHTPS_POS_SCALE_Y(ts) / 10000,
								info->fingers[i].x,
								info->fingers[i].y,
								info->fingers[i].wx,
								info->fingers[i].wy,
								info->fingers[i].z);

			if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
				if(ts->report_info.fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH){
					diff_x = shtps_get_diff(info->fingers[i].x, ts->report_info.fingers[i].x, SHTPS_POS_SCALE_X(ts));
					diff_y = shtps_get_diff(info->fingers[i].y, ts->report_info.fingers[i].y, SHTPS_POS_SCALE_Y(ts));

					if((diff_x > 0) || (diff_y > 0)){
						shtps_set_eventtype(event, SHTPS_EVENT_DRAG);
					}
				}
			}

			if(info->fingers[i].state != ts->report_info.fingers[i].state){
				shtps_set_eventtype(event, SHTPS_EVENT_MTDU);
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag_init(struct shtps_rmi_spi *ts)
{
	ts->drag_hist_p = kzalloc(sizeof(struct shtps_filter_drag_hist_info), GFP_KERNEL);
	if(ts->drag_hist_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// memset(ts->drag_hist_p, 0, sizeof(struct shtps_filter_drag_hist_info));	// no need
	// memset(&ts->center_info, 0, sizeof(ts->center_info));	// no need
	#if defined( SHTPS_DRAG_STEP_ENABLE )
	{
		int i;
		for(i = 0;i < SHTPS_FINGER_MAX;i++){
			ts->touch_state.dragStep[i][0] = SHTPS_DRAG_THRESHOLD_ZERO;
			ts->touch_state.dragStep[i][1] = SHTPS_DRAG_THRESHOLD_ZERO;
			ts->touch_state.dragStepReturnTime[i][0] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
			ts->touch_state.dragStepReturnTime[i][1] = SHTPS_DRAG_THRESH_RETURN_TIME_ZERO;
		}
	}
	#endif /* SHTPS_DRAG_STEP_ENABLE */

	#if defined( SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE )
		shtps_flattery_ClearHistory(ts, 1, NULL);// Clear all histories
	#endif /* SHTPS_DRAG_SMOOTH_FLATTERY_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_drag_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->drag_hist_p)	kfree(ts->drag_hist_p);
	ts->drag_hist_p = NULL;
}
/* -------------------------------------------------------------------------- */

