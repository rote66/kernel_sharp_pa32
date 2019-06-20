/* drivers/sharp/shdisp/shdisp_bl71y8.c  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "shdisp_kerl_priv.h"

#include "shdisp_system.h"
#include "shdisp_io.h"
#include "shdisp_bdic.h"
#include "shdisp_dbg.h"
#include "shdisp_pm.h"
#include "./data/shdisp_bl71y8_ctrl.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_BKL_DTV_OFF                 (0)
#define SHDISP_BDIC_BKL_DTV_ON                  (1)

#define SHDISP_BDIC_BKL_LOWBKL_OFF              (0)
#define SHDISP_BDIC_BKL_LOWBKL_ON               (1)
#define SHDISP_BDIC_BKL_LOWBKL_EXE              (2)

#define SHDISP_BDIC_BKL_CHG_OFF                 (0)
#define SHDISP_BDIC_BKL_CHG_ON                  (1)

#define SHDISP_BDIC_BKL_AUTO_FIX_PARAM_MIN_ADO  (0)

#define SHDISP_BDIC_GET_ADO_RETRY_TIMES         (4)
#define SHDISP_BDIC_MAX_ADO_VALUE          (0xFFFF)
#define SHDISP_BDIC_RATIO_OF_ALS0          (64 / 4)
#define SHDISP_BDIC_RATIO_OF_ALS1          (60 / 4)

#define SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE    (ARRAY_SIZE(shdisp_bdic_bkl_ado_tbl))
#define SHDISP_BDIC_LUX_DIVIDE_COFF         (100)

#define SHDISP_BDIC_REGSET(x)               (shdisp_bdic_API_seq_regset(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_BACKUP_REGS_BDIC(x)     (shdisp_bdic_seq_backup_bdic_regs(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_BACKUP_REGS_ALS(x)      (shdisp_bdic_seq_backup_als_regs(x, ARRAY_SIZE(x)))
#define SHDISP_BDIC_RESTORE_REGS(x)         (shdisp_bdic_API_seq_regset(x, ARRAY_SIZE(x)))

#define SHDISP_BDIC_AVE_ADO_READ_TIMES          (5)
#define SHDISP_BDIC_INVALID_ADO                 (-1)
#define SHDISP_BDIC_INVALID_RANGE               (-1)

#define PSALS_CORRECT_PROH_HIGH_VAL(val) \
            if ((val==0x068) || (val==0x69)) { \
                val = 0x6A; \
            }
#define PSALS_CORRECT_PROH_LOW_VAL(val) \
            if ((val==0x068) || (val==0x69)) { \
                val = 0x67; \
            }

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_BDIC_BKL_SLOPE_MODE_NONE,
    SHDISP_BDIC_BKL_SLOPE_MODE_FAST,
    SHDISP_BDIC_BKL_SLOPE_MODE_SLOW,
};

enum {
    SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_NONE,
    SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_EMG,
#ifdef SHDISP_LOWBKL
    SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_LOWBKL,
#endif /* SHDISP_LOWBKL */
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_seq_backlight_off(void);
static void shdisp_bdic_seq_backlight_fix_on(int param);
static void shdisp_bdic_seq_backlight_auto_on(int param);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *ado, unsigned int *lux, unsigned short *clear, unsigned short *ir);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir);
#ifdef SHDISP_ALS_INT
static int shdisp_bdic_LD_PHOTO_SENSOR_set_alsint(struct shdisp_photo_sensor_int_trigger *value);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_alsint(struct shdisp_photo_sensor_int_trigger *value);
static int shdisp_bdic_LD_set_als_int(struct shdisp_photo_sensor_int_trigger *value);
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lightinfo(struct shdisp_light_info *value);
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_TRV_NM2
static int shdisp_bdic_LD_LCD_BKL_trv_param(struct shdisp_trv_param param);
static int shdisp_bdic_LD_LCD_BKL_trv_level_calculation(int level, int l_limit);
#endif /* SHDISP_TRV_NM2 */
#ifdef SHDISP_LOWBKL
static void shdisp_bdic_LD_LCD_BKL_lowbkl_on(void);
static void shdisp_bdic_LD_LCD_BKL_lowbkl_off(void);
#endif /* SHDISP_LOWBKL */
static int shdisp_bdic_LD_LCD_BKL_param_revision(int param_auto);
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void);
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void);
static void shdisp_bdic_LD_LCD_BKL_set_emg_mode(int emg_mode, bool force);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);
static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *value);
static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned char *opt_val);
static void shdisp_bdic_LD_LCD_BKL_chg_on(void);
static void shdisp_bdic_LD_LCD_BKL_chg_off(void);

static void shdisp_bdic_PD_LCD_POS_PWR_on(void);
static void shdisp_bdic_PD_LCD_POS_PWR_off(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_on(void);
static void shdisp_bdic_PD_LCD_NEG_PWR_off(void);
static int shdisp_bdic_seq_backup_bdic_regs(shdisp_bdicRegSetting_t *regs, int size);
static int shdisp_bdic_seq_backup_als_regs(shdisp_bdicRegSetting_t *regs, int size);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static unsigned char shdisp_bdic_PD_opt_th_shift(int index);
static void shdisp_bdic_PD_BKL_update_led_value(void);
static void shdisp_bdic_PD_BKL_set_led_value(void);
static void shdisp_bdic_PD_BKL_set_opt_value(void);
static int shdisp_bdic_PD_get_sensor_state(void);
static int shdisp_bdic_PD_wait4i2ctimer_stop(void);
static int shdisp_bdic_PD_psals_power_on(void);
static int shdisp_bdic_PD_psals_power_off(void);
static int shdisp_bdic_PD_psals_ps_init_als_off(void);
static int shdisp_bdic_PD_psals_ps_init_als_on(void);
static int shdisp_bdic_PD_psals_ps_deinit_als_off(void);
static int shdisp_bdic_PD_psals_ps_deinit_als_on(void);
static int shdisp_bdic_PD_psals_als_init_ps_off(void);
static int shdisp_bdic_PD_psals_als_init_ps_on(void);
static int shdisp_bdic_PD_psals_als_deinit_ps_off(void);
static int shdisp_bdic_PD_psals_als_deinit_ps_on(void);
static int shdisp_bdic_PD_get_ave_ado(struct shdisp_ave_ado *ave_ado);

static int shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *ado, unsigned short *clear, unsigned short *ir);
static int shdisp_bdic_PD_REG_RAW_DATA_get_opt(unsigned short *clear, unsigned short *ir);
#ifdef SHDISP_ALS_INT
static int shdisp_bdic_PD_REG_int_setting(struct shdisp_photo_sensor_int_trigger *trigger);
static int shdisp_bdic_chk_als_trigger(struct shdisp_photo_sensor_trigger *trigger);
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_LED_INT
static int shdisp_bdic_led_auto_low_process(void);
static int shdisp_bdic_led_auto_low_enable(void);
static int shdisp_bdic_led_auto_low_disable(void);
#endif /* SHDISP_LED_INT */

static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params);

static int shdisp_bdic_PD_i2c_throughmode_ctrl(bool ctrl);

static void shdisp_bdic_als_shift_ps_on_table_adjust(struct shdisp_photo_sensor_adj *adj);
static void shdisp_bdic_als_shift_ps_off_table_adjust(struct shdisp_photo_sensor_adj *adj);

static int shdisp_bdic_als_user_to_devtype(int type);
#ifdef SHDISP_LOWBKL
static int shdisp_bdic_set_lowbkl_mode(int onoff);
#endif /* SHDISP_LOWBKL */
static int shdisp_bdic_set_opt_value_slow(void);
static int shdisp_bdic_set_opt_value_fast(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_bdic_state_str s_state_str;

static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_bkl_param;
static int shdisp_bdic_bkl_param_auto;
static int shdisp_bdic_bkl_before_mode;
static int shdisp_bdic_bkl_slope_slow_allow;

static int shdisp_bdic_dtv;

static int shdisp_bdic_emg;
static int shdisp_bdic_emg_before;
static bool shdisp_bdic_emg_off_pending;

static int shdisp_bdic_chg;

static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_OFF },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_OFF }
};


static unsigned int shdisp_bdic_irq_fac = 0;
static unsigned int shdisp_bdic_irq_fac_exe = 0;

static int  shdisp_bdic_irq_prioriy[SHDISP_IRQ_MAX_KIND];

static unsigned char shdisp_backup_irq_photo_req[3];

static int shdisp_bdic_irq_det_flag = 0;

#ifdef SHDISP_TRV_NM2
static int  l_limit_m = 24;
static int  l_limit_a = 85;
#endif /* SHDISP_TRV_NM2 */

#ifdef SHDISP_LOWBKL
static int shdisp_bdic_lowbkl;
static int shdisp_bdic_lowbkl_before;
#endif /* SHDISP_LOWBKL */

static int psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;

static int mled_delay_ms1   = 400;
static int slope_fast       = 0x21;

#if defined(CONFIG_ANDROID_ENGINEERING)
module_param(mled_delay_ms1, int, 0600);
module_param(slope_fast, int, 0600);
#ifdef SHDISP_TRV_NM2
module_param(l_limit_m, int, 0600);
module_param(l_limit_a, int, 0600);
#endif /* SHDISP_TRV_NM2 */
#endif /* CONFIG_ANDROID_ENGINEERING */

#ifdef SHDISP_ALS_INT
static struct shdisp_photo_sensor_int_trigger sensor_int_trigger;
#endif /* SHDISP_ALS_INT */

static int shdisp_bdic_ado_for_brightness;
static signed char shdisp_bdic_before_range;
static unsigned short shdisp_bdic_before_ado;

#define SHDISP_MAIN_BKL_LOWBKL_AUTO_DIVIDE           (3)
#define SHDISP_MAIN_BKL_LOWBKL_AUTO_VAL              (0x14)
#define SHDISP_MAIN_BKL_LOWBKL_AUTO_PARAM_THRESHOLD  (0x14)
#define SHDISP_MAIN_BKL_LOWBKL_AUTO_DIVIDE_THRESHOLD (0x44)
#define SHDISP_MAIN_BKL_LOWBKL_FIX_DIVIDE            (5)
#define SHDISP_MAIN_BKL_LOWBKL_FIX_VAL               (0x40)
#define SHDISP_MAIN_BKL_LOWBKL_FIX_PARAM_THRESHOLD   (0x40)
#define SHDISP_MAIN_BKL_LOWBKL_FIX_DIVIDE_THRESHOLD  (0x80)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_initialize(struct shdisp_bdic_state_str *state_str)
{
    struct shdisp_led_init_param init_param;

    s_state_str.bdic_chipver                    = state_str->bdic_chipver;

    shdisp_bdic_bkl_mode        = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_param       = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_bdic_bkl_param_auto  = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_bdic_bkl_slope_slow_allow = SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_NONE;

    shdisp_bdic_dtv             = SHDISP_BDIC_BKL_DTV_OFF;

    shdisp_bdic_emg             = SHDISP_BDIC_BKL_EMG_OFF;
    shdisp_bdic_emg_before      = SHDISP_BDIC_BKL_EMG_OFF;
    shdisp_bdic_emg_off_pending = false;

    shdisp_bdic_chg             = SHDISP_BDIC_BKL_CHG_OFF;

    shdisp_bdic_ado_for_brightness      = SHDISP_BDIC_INVALID_ADO;
    shdisp_bdic_before_range    = SHDISP_BDIC_INVALID_RANGE;
    shdisp_bdic_before_ado      = 0;

    memcpy(&(s_state_str.photo_sensor_adj),
                                &(state_str->photo_sensor_adj), sizeof(struct shdisp_photo_sensor_adj));

    if (s_state_str.photo_sensor_adj.status == SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED) {
        shdisp_bdic_als_shift_ps_on_table_adjust(&(s_state_str.photo_sensor_adj));
        shdisp_bdic_als_shift_ps_off_table_adjust(&(s_state_str.photo_sensor_adj));
    }
    init_param.handset_color                   = state_str->handset_color;
    init_param.bdic_chipver                    = state_str->bdic_chipver;
    shdisp_led_API_initialize(&init_param);

#ifdef SHDISP_ALS_INT
    memset(&sensor_int_trigger, 0x00, sizeof(struct shdisp_photo_sensor_int_trigger));
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_TRV_NM2
    memset(&s_state_str.trv_param, 0x00, sizeof(struct shdisp_trv_param));
#endif /* SHDISP_TRV_NM2 */
#ifdef SHDISP_LOWBKL
    shdisp_bdic_lowbkl          = SHDISP_BDIC_BKL_LOWBKL_OFF;
    shdisp_bdic_lowbkl_before   = SHDISP_BDIC_BKL_LOWBKL_OFF;
#endif /* SHDISP_LOWBKL */
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_prox_sensor_param                                     */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_set_prox_sensor_param(struct shdisp_prox_params *prox_params)
{
    shdisp_bdic_PD_psals_write_threshold(prox_params);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_on                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_power_on(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_PD_LCD_POS_PWR_on();
    SHDISP_TRACE("out")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_power_off                                             */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_power_off(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_PD_LCD_POS_PWR_off();
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_on                                            */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_m_power_on(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_PD_LCD_NEG_PWR_on();
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_m_power_off                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_m_power_off(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_PD_LCD_NEG_PWR_off();
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_off(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_seq_backlight_off();
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    SHDISP_TRACE("in")
    shdisp_bdic_seq_backlight_fix_on(param);
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_auto_on                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_auto_on(int param)
{
    SHDISP_TRACE("in")
    shdisp_bdic_seq_backlight_auto_on(param);
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_param(struct shdisp_bdic_bkl_info *bkl_info)
{
    int mode = 0;
    unsigned char value;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        bkl_info->mode = SHDISP_BDIC_BKL_MODE_FIX;
        bkl_info->param = shdisp_bdic_bkl_param;
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &value);
        bkl_info->value = value;
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        bkl_info->mode = SHDISP_BDIC_BKL_MODE_AUTO;
        bkl_info->param = shdisp_bdic_bkl_param_auto;
        bkl_info->value = 0x100;
        break;

    default:
        bkl_info->mode = SHDISP_BDIC_BKL_MODE_OFF;
        bkl_info->param = 0;
        bkl_info->value = 0;
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;

    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;


    SHDISP_TRACE("in. tmp->mode %d, tmp->param %d ", tmp->mode, tmp->param);

    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if ((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
               (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_WEAK)) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param != SHDISP_MAIN_BKL_PARAM_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO].param;
    } else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }
    SHDISP_TRACE("out. req->mode %d, req->param %d ", req->mode, req->param);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_on                                            */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_dtv_on(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_dtv_off                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_dtv_off(void)
{
    shdisp_bdic_LD_LCD_BKL_dtv_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_emg_mode                                      */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_set_emg_mode(int emg_mode)
{
    shdisp_bdic_LD_LCD_BKL_set_emg_mode(emg_mode, false);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_on                                            */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_chg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_chg_off                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_chg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_chg_off();

    return;
}

#ifdef SHDISP_LOWBKL
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_lowbkl_on                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_lowbkl_on(void)
{
    shdisp_bdic_LD_LCD_BKL_lowbkl_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_lowbkl_off                                        */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LCD_BKL_lowbkl_off(void)
{
    shdisp_bdic_LD_LCD_BKL_lowbkl_off();

    return;
}
#endif /* SHDISP_LOWBKL */

#ifdef SHDISP_TRV_NM2
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_trv_param                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_LCD_BKL_trv_param(struct shdisp_trv_param param)
{
    int ret;

    ret = shdisp_bdic_LD_LCD_BKL_trv_param(param);

    return ret;
}
#endif /* SHDISP_TRV_NM2 */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_lux                                      */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned int *lux)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(value, lux, NULL, NULL);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_raw_als                                  */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(clear, ir);

    return ret;
}

#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_set_alsint                                   */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_PHOTO_SENSOR_set_alsint(struct shdisp_photo_sensor_int_trigger *value)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_set_alsint(value);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_alsint                                   */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_PHOTO_SENSOR_get_alsint(struct shdisp_photo_sensor_int_trigger *value)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_alsint(value);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PHOTO_SENSOR_get_light_info                                  */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_PHOTO_SENSOR_get_light_info(struct shdisp_light_info *value)
{
    int ret;

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lightinfo(value);

    return ret;
}
#endif /* SHDISP_ALS_INT */

#ifdef SHDISP_LED_INT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_led_auto_low_enable                                       */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_led_auto_low_enable(bool enable)
{
    int ret;

    if (enable) {
        ret = shdisp_bdic_led_auto_low_enable();
    } else {
        ret = shdisp_bdic_led_auto_low_disable();
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_led_auto_low_process                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_led_auto_low_process(void)
{
    int ret;

    ret = shdisp_bdic_led_auto_low_process();

    return ret;
}
#endif /* SHDISP_LED_INT */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_i2c_transfer                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int reg = 0;

    if (msg == NULL) {
        SHDISP_ERR("<NULL_POINTER> msg.");
        return SHDISP_RESULT_FAILURE;
    }

    reg = (int)msg->wbuf[0];

    if (msg->mode >= NUM_SHDISP_BDIC_I2C_M) {
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).", msg->mode);
        return SHDISP_RESULT_FAILURE;
    }

    if (reg < SENSOR_REG_COMMAND1 || reg > SENSOR_REG_D2_MSB) {
        SHDISP_ERR("<RESULT_FAILURE> Register address out of range.");
        return SHDISP_RESULT_FAILURE;
    }


    shdisp_bdic_PD_i2c_throughmode_ctrl(true);
    ret = shdisp_bdic_API_IO_i2c_transfer(msg);
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_write_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    unsigned char dummy = 0;

    shdisp_bdic_API_IO_bank_set(0x00);
    shdisp_bdic_API_IO_read_reg(BDIC_REG_GINF4, &dummy);

    if (dummy & 0x04) {
        return SHDISP_RESULT_SUCCESS;
    } else {
        return SHDISP_RESULT_FAILURE;
    }
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned short   shdisp_log_lv_bk;

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_ON);
    pbuf = kzalloc(256 * 2, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d", 256 * 2);
        return;
    }
    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_bank1_dump);

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    shdisp_bdic_PD_wait4i2ctimer_stop();

    p = pbuf;
    shdisp_bdic_API_IO_bank_set(0x00);
    for (idx = 0x00; idx <= 0xFF; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_API_IO_bank_set(0x01);
    for (idx = 0x00; idx <= 0xFF; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_API_IO_bank_set(0x00);
    shdisp_log_lv = shdisp_log_lv_bk;

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_bank1_dump);
    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_OFF);

    printk("[SHDISP] BDIC INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_chipver               = %d.\n", s_state_str.bdic_chipver);
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_bkl_param_auto             = %d.\n", shdisp_bdic_bkl_param_auto);
    printk("[SHDISP] shdisp_bdic_dtv                        = %d.\n", shdisp_bdic_dtv);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_emg_off_pending            = %d.\n", shdisp_bdic_emg_off_pending);
#ifdef SHDISP_LOWBKL
    printk("[SHDISP] shdisp_bdic_lowbkl                     = %d.\n", shdisp_bdic_lowbkl);
#endif /* SHDISP_LOWBKL */
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);
#ifdef SHDISP_TRV_NM2
    printk("[SHDISP] trv_param.request                      = %d.\n", s_state_str.trv_param.request);
    printk("[SHDISP] trv_param.strength                     = %d.\n", s_state_str.trv_param.strength);
    printk("[SHDISP] trv_param.adjust                       = %d.\n", s_state_str.trv_param.adjust);
#endif /* SHDISP_TRV_NM2 */

    for (idx = 0; idx < NUM_SHDISP_MAIN_BKL_DEV_TYPE; idx++) {
        printk("[SHDISP] shdisp_bkl_priority_table[%d]       = (mode:%d, param:%d).\n",
                                    idx, shdisp_bkl_priority_table[idx].mode, shdisp_bkl_priority_table[idx].param);
    }

    p = pbuf;
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK0 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
        p += 8;
    }
    for (idx = 0x00; idx < 0xFF; idx += 8) {
        printk("[SHDISP] BDIC_REG_BANK1 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                    idx, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
        p += 8;
    }
    printk("[SHDISP] BDIC INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_OPT_INFO_output                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_OPT_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned short   shdisp_log_lv_bk;

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_ON);
    pbuf = kzalloc((BDIC_REG_OPT23 - BDIC_REG_OPT0 + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d", (BDIC_REG_OPT23 - BDIC_REG_OPT0 + 1));
        return;
    }
    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_bank1_dump);

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    shdisp_bdic_PD_wait4i2ctimer_stop();

    p = pbuf;
    shdisp_bdic_API_IO_bank_set(0x01);
    for (idx = BDIC_REG_OPT0; idx <= BDIC_REG_OPT23; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_bdic_API_IO_bank_set(0x00);
    shdisp_log_lv = shdisp_log_lv_bk;

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_bank1_dump);
    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_OFF);

    printk("[SHDISP] BDIC_REG_OPT INFO ->>\n");
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_bkl_param_auto             = %d.\n", shdisp_bdic_bkl_param_auto);
    printk("[SHDISP] shdisp_bdic_emg                        = %d.\n", shdisp_bdic_emg);
    printk("[SHDISP] shdisp_bdic_emg_off_pending            = %d.\n", shdisp_bdic_emg_off_pending);
    printk("[SHDISP] shdisp_bdic_chg                        = %d.\n", shdisp_bdic_chg);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_OPT0  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        BDIC_REG_OPT0, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] BDIC_REG_OPT8  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        BDIC_REG_OPT8, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] BDIC_REG_OPT16 0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        BDIC_REG_OPT16, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));

    printk("[SHDISP] BDIC_REG_OPT INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_PSALS_INFO_output                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_PSALS_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    printk("[SHDISP] in PSALS SENSOR INFO ->>\n");
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_INIT);
    pbuf = kzalloc((((SENSOR_REG_D2_MSB + 7) / 8) * 8), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d", (((SENSOR_REG_D2_MSB + 7) / 8) * 8));
        return;
    }

    shdisp_IO_API_delay_us(1000 * 1000);

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);

    p = pbuf;
    for (idx = SENSOR_REG_COMMAND1; idx <= SENSOR_REG_D2_MSB; idx++) {
        *p = 0x00;
        shdisp_bdic_API_IO_psals_read_reg(idx, p);
        p++;
    }
    p = pbuf;
    printk("[SHDISP] SENSOR_REG_DUMP 0x00: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                        *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x08: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                        *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7));
    p += 8;
    printk("[SHDISP] SENSOR_REG_DUMP 0x10: %02x %02x                              \n", *p, *(p + 1));
    kfree(pbuf);

    shdisp_bdic_PD_i2c_throughmode_ctrl(false);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_REQ_OFF);

    printk("[SHDISP] out PSALS SENSOR INFO <<-\n");
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_check_type(int irq_type)
{
    if ((irq_type < SHDISP_IRQ_TYPE_ALS) || (irq_type >= NUM_SHDISP_IRQ_TYPE)) {
        return SHDISP_RESULT_FAILURE;
    }
    if (irq_type == SHDISP_IRQ_TYPE_DET) {
        if (!(SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) {
           return SHDISP_RESULT_FAILURE;
        }
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_save_fac                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_save_fac(void)
{
    unsigned char value1 = 0, value2 = 0, value3 = 0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    shdisp_bdic_API_IO_read_reg(BDIC_REG_GFAC1, &value1);
    value1 &= 0x7F;
    shdisp_bdic_API_IO_read_reg(BDIC_REG_GFAC3, &value2);
    shdisp_bdic_API_IO_read_reg(BDIC_REG_GFAC4, &value3);
    SHDISP_DEBUG("GFAC4=%02x GFAC3=%02x GFAC1=%02x", value3, value2, value1);

    shdisp_bdic_irq_fac = (unsigned int)value1 | ((unsigned int)value2 << 8) | ((unsigned int)value3 << 16);

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x08);
        SHDISP_ERR("ps_als error : INT_I2C_ERR_REQ(GFAC4[3]) detect");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_I2C_ERROR;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
    }

    if (shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_DET | SHDISP_BDIC_INT_GFAC_PS2 | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_ALS)) {
        shdisp_bdic_API_IO_read_reg(BDIC_REG_GIMR3, &shdisp_backup_irq_photo_req[0]);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_GIMF3, &shdisp_backup_irq_photo_req[1]);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_GIMR4, &shdisp_backup_irq_photo_req[2]);
    }

    if ((shdisp_bdic_irq_fac & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }

    if (shdisp_bdic_irq_det_flag == 1) {
        shdisp_bdic_irq_fac |= SHDISP_BDIC_INT_GFAC_DET;
        shdisp_bdic_irq_det_flag = 2;
    }

#ifdef SHDISP_ALS_INT
    value1 = 0;
    shdisp_bdic_API_IO_read_reg(BDIC_REG_GFAC2, &value1);
    value1 &= 0x30;
    SHDISP_DEBUG("GFAC2=%02x ", value1);
    shdisp_bdic_irq_fac = shdisp_bdic_irq_fac | ((unsigned int)value1 << 20);
#endif /* SHDISP_ALS_INT */

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                             */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_check_DET(void)
{
    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        return SHDISP_BDIC_IRQ_TYPE_DET;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_I2C_ERR                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_check_I2C_ERR(void)
{
    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        return SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                             */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_check_fac(void)
{
    int i;

    if (shdisp_bdic_irq_fac == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_irq_fac_exe = (shdisp_bdic_irq_fac & SHDISP_INT_ENABLE_GFAC);
    if (shdisp_bdic_irq_fac_exe == 0) {
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_IRQ_MAX_KIND; i++) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_NONE;
    }

    i = 0;
    if ((shdisp_bdic_irq_fac_exe &
        (SHDISP_BDIC_INT_GFAC_PS | SHDISP_BDIC_INT_GFAC_I2C_ERR | SHDISP_BDIC_INT_GFAC_PS2))
        == SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_PS;
        i++;
    }

    if ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_DET) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_DET;
        i++;
    } else if (((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_I2C_ERR) != 0)) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_I2C_ERR;
        i++;
    } else if ((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS | SHDISP_BDIC_INT_GFAC_OPTSEL)) != 0) {
        shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS;
        i++;
#ifdef SHDISP_ALS_INT
    } else if ((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS_TRG1 | SHDISP_BDIC_INT_GFAC_ALS_TRG2)) != 0) {
        if ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_ALS_TRG1) != 0) {
            shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER1;
        }
        if ((shdisp_bdic_irq_fac_exe & SHDISP_BDIC_INT_GFAC_ALS_TRG2) != 0) {
            shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER2;
        }
        if ((shdisp_bdic_irq_fac_exe & (SHDISP_BDIC_INT_GFAC_ALS_TRG1 | SHDISP_BDIC_INT_GFAC_ALS_TRG2))
            == (SHDISP_BDIC_INT_GFAC_ALS_TRG1 | SHDISP_BDIC_INT_GFAC_ALS_TRG2)) {
            shdisp_bdic_irq_prioriy[i] = SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER;
        }
        i++;
#endif /* SHDISP_ALS_INT */
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IRQ_get_fac(int iQueFac)
{
    if (iQueFac >= SHDISP_IRQ_MAX_KIND) {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
    return shdisp_bdic_irq_prioriy[iQueFac];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_Clear                                                 */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_Clear(void)
{
    unsigned char out1, out2, out3;

    if (shdisp_bdic_irq_fac == 0) {
        return;
    }

    out1 = (unsigned char)(shdisp_bdic_irq_fac & 0x000000FF);
    out2 = (unsigned char)((shdisp_bdic_irq_fac >>  8) & 0x000000FF);
    out3 = (unsigned char)((shdisp_bdic_irq_fac >> 16) & 0x000000FF);

    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x00);

    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) && (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS)) {
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) && (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2)) {
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMF3, 0x02);
    }


    if (((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_ALS)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2)) ||
        ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_I2C_ERR) &&
         (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR))) {
        if (shdisp_backup_irq_photo_req[0] & 0x01) {
            shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMR3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[1] & 0x01) {
            shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMF3, 0x01);
        }
        if (shdisp_backup_irq_photo_req[2] & 0x20) {
            shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMR4, 0x20);
        }
    }

#ifdef SHDISP_ALS_INT
    if ((shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG1) || (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG2)) {
        out1 = 0;
        if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG1) {
            out1 = out1 | SHDISP_BDIC_OPT_TABLE1_IMR;
        }
        if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG2) {
            out1 = out1 | SHDISP_BDIC_OPT_TABLE2_IMR;
        }
        shdisp_bdic_reg_als_int_clear1[1].mask = out1;
        SHDISP_BDIC_REGSET(shdisp_bdic_reg_als_int_clear1);

        shdisp_bdic_PD_wait4i2ctimer_stop();

        SHDISP_BDIC_REGSET(shdisp_bdic_reg_als_int_clear2);

        out1 = 0;
        if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG1) {
            out1 = out1 | SHDISP_BDIC_OPT_TABLE1_IMR;
            shdisp_bdic_API_IO_write_reg(BDIC_REG_OPT_INT1, 0x00);
            memset(&(sensor_int_trigger.trigger1), 0x00, sizeof(struct shdisp_photo_sensor_trigger));
            SHDISP_DEBUG("<OTHER>TRIGGER1 clear");
        }
        if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS_TRG2) {
            out1 = out1 | SHDISP_BDIC_OPT_TABLE2_IMR;
            shdisp_bdic_API_IO_write_reg(BDIC_REG_OPT_INT2, 0x00);
            memset(&(sensor_int_trigger.trigger2), 0x00, sizeof(struct shdisp_photo_sensor_trigger));
            SHDISP_DEBUG("<OTHER>TRIGGER2 clear");
        }
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GSCR2, out1);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GSCR2, out1);
    }
#endif /* SHDISP_ALS_INT */
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_i2c_error_Clear                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_i2c_error_Clear(void)
{
    unsigned char out2 = 0, out3 = 0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_I2C_ERR) {
        out3 = 0x08;
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_PS2) {
        out2 = 0x02;
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, out2);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    }
    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_DURING;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_fac_Clear                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_fac_Clear(void)
{
    unsigned char out3 = 0;

    if (SHDISP_INT_ENABLE_GFAC & SHDISP_BDIC_INT_GFAC_DET) {
        out3 = 0x04;
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, out3);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x00);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_ctrl                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl)
{
    shdisp_bdic_API_IO_bank_set(0x00);
    if (ctrl) {
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GIMF4, 0x04);
    } else {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_Clear_All                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_dbg_Clear_All(void)
{
    unsigned char out1, out2, out3;

    out1 = (unsigned char)(SHDISP_INT_ENABLE_GFAC & 0x000000FF);
    out2 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >>  8) & 0x000000FF);
    out3 = (unsigned char)((SHDISP_INT_ENABLE_GFAC >> 16) & 0x000000FF);

    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR1, out1);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, out2);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, out3);

    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR1, 0x00);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR3, 0x00);
    shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x00);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_dbg_set_fac                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC)
{
    shdisp_bdic_irq_fac = nGFAC;

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_DET) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF4, 0x04);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR1, 0x08);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF1, 0x08);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_ALS) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR3, 0x01);
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x01);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_PS2) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMF3, 0x02);
    }

    if (shdisp_bdic_irq_fac & SHDISP_BDIC_INT_GFAC_OPTSEL) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x20);
    }
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_als_sensor_pow_ctl                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_als_sensor_pow_ctl(int dev_type, int power_mode)
{
    unsigned long type = 0;
    int param_chk = 0;

    type = shdisp_bdic_als_user_to_devtype(dev_type);
    if (type == NUM_SHDISP_PHOTO_SENSOR_TYPE) {
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->type(%d).", dev_type);
    }

    switch (power_mode) {
    case SHDISP_PHOTO_SENSOR_DISABLE:
        shdisp_pm_API_als_user_manager(type, SHDISP_DEV_REQ_OFF);
        break;
    case SHDISP_PHOTO_SENSOR_ENABLE:
        shdisp_pm_API_als_user_manager(type, SHDISP_DEV_REQ_ON);
        break;
    default:
        param_chk = 1;
        SHDISP_ERR("<INVALID_VALUE> ctl->power(%d).", power_mode);
        break;
    }

    if (param_chk == 1) {
        return SHDISP_RESULT_FAILURE;
    }

#ifdef SHDISP_ALS_INT
    if ((dev_type == sensor_int_trigger.type) && (power_mode == SHDISP_PHOTO_SENSOR_DISABLE)) {
        memset(&sensor_int_trigger, 0x00, sizeof(struct shdisp_photo_sensor_int_trigger));
        shdisp_bdic_PD_REG_int_setting(&sensor_int_trigger);

        return SHDISP_RESULT_ALS_INT_OFF;
    }
#endif /* SHDISP_ALS_INT */

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_on                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_power_on();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_power_off                                           */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_power_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_power_off();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_ps_init_als_off();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_init_als_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_ps_init_als_on();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_off();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_ps_deinit_als_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_ps_deinit_als_on();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_off                                     */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_als_init_ps_off();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_init_ps_on                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_als_init_ps_on();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_off                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_off();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_als_deinit_ps_on                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_psals_als_deinit_ps_on();
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_psals_is_recovery_successful                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_psals_is_recovery_successful(void)
{
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    if (psals_recovery_flag == SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER) {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GIMR4, 0x08);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x08);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_GSCR4, 0x00);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_RECOVERY_NG;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_get_ave_ado                                               */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_get_ave_ado(ave_ado);
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_update_led_value                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_update_led_value(void)
{
    SHDISP_TRACE("in");
    shdisp_bdic_PD_BKL_update_led_value();
    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD4;
        break;

    default:
        return;;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_off                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_backlight_off(void)
{
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_OFF);

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_OFF);

    if (shdisp_bdic_emg_off_pending) {
        shdisp_bdic_LD_LCD_BKL_set_emg_mode(SHDISP_BDIC_BKL_EMG_OFF, true);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_fix_on                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_backlight_fix_on(int param)
{
    SHDISP_TRACE("in param:%d", param);

    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 START");
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_LED_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ON, 0);
    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 END");

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_FIX_START, 0);

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_ON);

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_auto_on                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_backlight_auto_on(int param)
{
    (void)shdisp_pm_API_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_ON);
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_REQ_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 START");
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_LED_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ON, 0);
    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0010 END");

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_AUTO_START, 0);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lux                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lux(unsigned short *ado, unsigned int *lux, unsigned short *clear, unsigned short *ir)
{
    int ret;
    unsigned int i;
    unsigned int ret_lux;
    unsigned short ado_tmp;

    SHDISP_TRACE("in");

    if (shdisp_pm_API_is_als_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<OTHER> photo sensor user none.");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_PD_REG_ADO_get_opt(&ado_tmp, clear, ir);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    ret_lux = 0;
    if (ado_tmp != 0) {
        for (i = 0; i < SHDISP_BDIC_LUX_TABLE_ARRAY_SIZE; i++) {
            if ((ado_tmp >= shdisp_bdic_bkl_ado_tbl[i].range_low) &&
                (ado_tmp < shdisp_bdic_bkl_ado_tbl[i].range_high)) {
                ret_lux  = ((unsigned int)ado_tmp) * shdisp_bdic_bkl_ado_tbl[i].param_a;
                ret_lux += shdisp_bdic_bkl_ado_tbl[i].param_b;
                ret_lux += (SHDISP_BDIC_LUX_DIVIDE_COFF / 2);
                ret_lux /= SHDISP_BDIC_LUX_DIVIDE_COFF;
                break;
            }
        }
    }

    *lux = ret_lux;
    if (ado) {
        *ado = ado_tmp;
    }

    SHDISP_TRACE("out ado=0x%04X, lux=%u", ado_tmp, ret_lux);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir)
{
    SHDISP_TRACE("in");

    if (shdisp_pm_API_is_als_active() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<OTHER> photo sensor user none.");
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_PD_REG_RAW_DATA_get_opt(clear, ir);

    SHDISP_TRACE("out clear=0x%04X, ir=0x%04X", (unsigned int)*clear, (unsigned int)*ir);
    return SHDISP_RESULT_SUCCESS;
}

#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_set_alsint                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_PHOTO_SENSOR_set_alsint(struct shdisp_photo_sensor_int_trigger *value)
{
    unsigned int type = 0;
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_chk_als_trigger(&(value->trigger1));
    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<INVALID_VALUE> trigger1.");
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }
    ret = shdisp_bdic_chk_als_trigger(&(value->trigger2));
    if (ret == SHDISP_RESULT_FAILURE) {
        SHDISP_ERR("<INVALID_VALUE> trigger2.");
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }
    type = shdisp_bdic_als_user_to_devtype(value->type);
    if (type == NUM_SHDISP_PHOTO_SENSOR_TYPE) {
        SHDISP_ERR("<INVALID_VALUE> type(%d).", value->type);
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_pm_API_is_active_als_user(type) != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("<OTHER> photo sensor user none.");
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_SUCCESS;
    }

    if ((sensor_int_trigger.trigger1.enable != 0) || (sensor_int_trigger.trigger2.enable != 0)) {
        if (sensor_int_trigger.type != value->type) {
            SHDISP_WARN("request from other user.");
        }
    }

    shdisp_bdic_LD_set_als_int(value);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_alsint                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_PHOTO_SENSOR_get_alsint(struct shdisp_photo_sensor_int_trigger *value)
{
    SHDISP_TRACE("in");

    memcpy(value, &sensor_int_trigger, sizeof(struct shdisp_photo_sensor_int_trigger));
    value->result = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_als_int                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_set_als_int(struct shdisp_photo_sensor_int_trigger *value)
{
    int ret;

    SHDISP_TRACE("in");

    if (value->trigger1.enable == 0) {
        SHDISP_DEBUG("trigger1 disable");
        memset(&value->trigger1, 0x00, sizeof(struct shdisp_photo_sensor_trigger));
    }
    if (value->trigger2.enable == 0) {
        SHDISP_DEBUG("trigger2 disable");
        memset(&value->trigger2, 0x00, sizeof(struct shdisp_photo_sensor_trigger));
    }

    memcpy(&sensor_int_trigger, value, sizeof(struct shdisp_photo_sensor_int_trigger));
    value->result = SHDISP_RESULT_SUCCESS;

    ret = shdisp_bdic_PD_REG_int_setting(&sensor_int_trigger);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_PHOTO_SENSOR_get_lightinfo                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_PHOTO_SENSOR_get_lightinfo(struct shdisp_light_info *value)
{
    int ret;
    unsigned int lux = 0;
    unsigned short clear = 0, ir = 0;
    unsigned char level = 0;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_LD_PHOTO_SENSOR_get_lux(NULL, &lux, &clear, &ir);
    if (ret != SHDISP_RESULT_SUCCESS) {
        value->result = SHDISP_RESULT_FAILURE;
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_DEBUG("lux=%d clear=%d ir=%d", lux, clear, ir);
    value->lux = lux;
    if (clear < 1) {
        SHDISP_DEBUG("[caution]clear is zero");
        value->clear_ir_rate = 0;
    } else {
        value->clear_ir_rate = (((unsigned int)ir * 1000) / (unsigned int)clear + 5) / 10;
    }
    shdisp_bdic_API_IO_bank_set(0x00);
    shdisp_bdic_API_IO_read_reg(BDIC_REG_ADO_INDEX, &level);
    value->level = (unsigned short)(level & 0x1F);
    SHDISP_DEBUG("level=%d", value->level);

    value->result = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_ALS_INT */

#ifdef SHDISP_LED_INT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_led_auto_low_process                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_led_auto_low_process(void)
{
    unsigned char ginf3 = 0;

    SHDISP_TRACE("in");

    shdisp_bdic_API_IO_bank_set(0x00);
    shdisp_bdic_API_IO_read_reg(BDIC_REG_GINF3, &ginf3);

    if (ginf3 & 0x01) {
        SHDISP_DEBUG("normal.");
        SHDISP_BDIC_REGSET(shdisp_bdic_reg_led_cur_norm);
    } else {
        SHDISP_DEBUG("low.");
        SHDISP_BDIC_REGSET(shdisp_bdic_reg_led_cur_low);
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_led_auto_low_enable                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_led_auto_low_enable(void)
{
    SHDISP_TRACE("in");

    shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LED_AUTO_LOW, SHDISP_DEV_REQ_ON);
    SHDISP_BDIC_REGSET(shdisp_bdic_reg_als_req_on);
    shdisp_bdic_led_auto_low_process();

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_led_auto_low_disable                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_led_auto_low_disable(void)
{
    SHDISP_TRACE("in");

    SHDISP_BDIC_REGSET(shdisp_bdic_reg_als_req_off);
    SHDISP_BDIC_REGSET(shdisp_bdic_reg_led_cur_norm);
    shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LED_AUTO_LOW, SHDISP_DEV_REQ_OFF);

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_LED_INT */

#ifdef SHDISP_TRV_NM2
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_trv_param                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_LCD_BKL_trv_param(struct shdisp_trv_param param)
{
    SHDISP_TRACE("in");

    if ((param.request != SHDISP_TRV_PARAM_ON) && (param.request != SHDISP_TRV_PARAM_OFF)) {
        return SHDISP_RESULT_FAILURE;
    }

    if (s_state_str.trv_param.request == param.request) {
        return SHDISP_RESULT_SUCCESS;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_TRV_REQUEST, param.request);
    SHDISP_DEBUG("strength(%d), adjust(%d)", param.strength, param.adjust);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_trv_level_calculation                              */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_LCD_BKL_trv_level_calculation(int level, int l_limit)
{
    long long_level = 0;

    long_level = (long)((level * (255 - l_limit) * 2 + 255) / (255 * 2));
    long_level = long_level + (long)l_limit;

    SHDISP_DEBUG("TRV_NM2: before level(%d), after level(%d)", level, (int)long_level);

    return (int)long_level;
}
#endif /* SHDISP_TRV_NM2 */

#ifdef SHDISP_LOWBKL
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_lowbkl_on                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_lowbkl_on(void)
{
    if ((shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_ON) || (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_EXE)) {
        return;
    }

    shdisp_bdic_bkl_slope_slow_allow = SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_LOWBKL;

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_LOWBKL_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    shdisp_bdic_bkl_slope_slow_allow = SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_NONE;

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_lowbkl_off                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_lowbkl_off(void)
{
    if (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_LOWBKL_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}
#endif /* SHDISP_LOWBKL */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_param_revision                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_LCD_BKL_param_revision(int param_auto)
{
    unsigned long  level_n;
    int level;

    SHDISP_TRACE("in before_level=%d ", param_auto);

    level_n = ((((long)param_auto * (long)param_auto ) * 37) + ((long)param_auto * 588));
    level = (int)(level_n / 10000);

    if (level > SHDISP_MAIN_BKL_PARAM_MAX_AUTO) {
        level = SHDISP_MAIN_BKL_PARAM_MAX_AUTO;
    }
    if (level < SHDISP_MAIN_BKL_PARAM_MIN_AUTO) {
        level = SHDISP_MAIN_BKL_PARAM_MIN_AUTO;
    }

    SHDISP_TRACE("out after_level=%d ", level);
    return level;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_dtv_on(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_dtv_off                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_dtv_off(void)
{
    if (shdisp_bdic_dtv == SHDISP_BDIC_BKL_DTV_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_DTV_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_set_emg_mode                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_set_emg_mode(int emg_mode, bool force)
{
    SHDISP_TRACE("in emg_mode=%d force=%d", emg_mode, force);

    shdisp_bdic_emg_off_pending = false;

    if (emg_mode == shdisp_bdic_emg) {
        SHDISP_DEBUG("out: same request.");
        return;
    }

    if (!force &&
        (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) &&
        (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON_LEVEL0) &&
        (emg_mode == SHDISP_BDIC_BKL_EMG_OFF)) {
        SHDISP_DEBUG("out: off pending.");
        shdisp_bdic_emg_off_pending = true;
        return;
    }

    shdisp_bdic_bkl_slope_slow_allow = SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_EMG;

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_EMG_MODE, emg_mode);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }

    shdisp_bdic_bkl_slope_slow_allow = SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_NONE;
    shdisp_bdic_emg_before = shdisp_bdic_emg;

    SHDISP_TRACE("out");

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{

    *mode = SHDISP_BKL_TBL_MODE_NORMAL;

    switch (shdisp_bdic_emg) {
    case SHDISP_BDIC_BKL_EMG_ON_LEVEL0:
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL0;
        break;
    case SHDISP_BDIC_BKL_EMG_ON_LEVEL1:
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL1;
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_fix_param                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *param)
{
    unsigned char value;

    if (param == NULL) {
        return;
    }

#ifdef SHDISP_TRV_NM2
    if (s_state_str.trv_param.request == SHDISP_TRV_PARAM_ON) {
        level = shdisp_bdic_LD_LCD_BKL_trv_level_calculation(level, l_limit_m);
    }
#endif /* SHDISP_TRV_NM2 */

#ifdef SHDISP_LOWBKL
    if (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_EXE) {
        if (level > SHDISP_MAIN_BKL_LOWBKL_FIX_DIVIDE_THRESHOLD) {
            level = ((level * SHDISP_MAIN_BKL_LOWBKL_FIX_DIVIDE) / 10);
            if (!level) {
                level++;
            }
        } else {
            level = SHDISP_MAIN_BKL_LOWBKL_FIX_VAL;
        }
    }
#endif /* SHDISP_LOWBKL */
    value = shdisp_main_bkl_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL0:
        if (value > SHDISP_BKL_EMERGENCY_LIMIT_FIX_LEVEL0) {
            value = SHDISP_BKL_EMERGENCY_LIMIT_FIX_LEVEL0;
        }
        break;
    case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL1:
        if (value > SHDISP_BKL_EMERGENCY_LIMIT_FIX_LEVEL1) {
            value = SHDISP_BKL_EMERGENCY_LIMIT_FIX_LEVEL1;
        }
        break;
    }

    *param = value;
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_chg_on(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_chg_off                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_chg_off(void)
{
    if (shdisp_bdic_chg == SHDISP_BDIC_BKL_CHG_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_CHG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_LCD_POS_PWR_on(void)
{
    SHDISP_TRACE("in")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_on);
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_POS_PWR_off                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_LCD_POS_PWR_off(void)
{
    SHDISP_TRACE("in")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsp_off);
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_LCD_NEG_PWR_on(void)
{
    SHDISP_TRACE("in")
    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        SHDISP_DEBUG("VSN ON(TS2) chipver=%d", s_state_str.bdic_chipver);
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on_ts2);
    } else {
        SHDISP_DEBUG("VSN ON(TS1) chipver=%d", s_state_str.bdic_chipver);
        SHDISP_BDIC_REGSET(shdisp_bdic_vsn_on_ts1);
    }
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_LCD_NEG_PWR_off                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_LCD_NEG_PWR_off(void)
{
    SHDISP_TRACE("in")
    SHDISP_BDIC_REGSET(shdisp_bdic_vsn_off);
    SHDISP_TRACE("out")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backup_bdic_regs                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_backup_bdic_regs(shdisp_bdicRegSetting_t *regs, int size)
{
    int i;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t *tbl;

    tbl = (shdisp_bdicRegSetting_t *)regs;
    for (i = 0; i < size; i++) {
        switch (tbl->flg) {
        case SHDISP_BDIC_STR:
        case SHDISP_BDIC_STRM:
            ret = shdisp_bdic_API_IO_read_reg(tbl->addr, &(tbl->data));
            break;
        case SHDISP_BDIC_SET:
        case SHDISP_BDIC_CLR:
        case SHDISP_BDIC_RMW:
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_API_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_IO_API_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
        case SHDISP_ALS_RMW:
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic Read Error addr=%02X, data=%02X, mask=%02X", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backup_als_regs                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_backup_als_regs(shdisp_bdicRegSetting_t *regs, int size)
{
    int i;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t *tbl;

    tbl = (shdisp_bdicRegSetting_t *)regs;
    for (i = 0; i < size; i++) {
        switch (tbl->flg) {
        case SHDISP_BDIC_STR:
        case SHDISP_BDIC_STRM:
            ret = shdisp_bdic_API_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
        case SHDISP_BDIC_CLR:
        case SHDISP_BDIC_RMW:
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_API_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_IO_API_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        case SHDISP_ALS_STR:
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            ret = shdisp_bdic_API_IO_psals_read_reg(tbl->addr, &(tbl->data));
            break;
        case SHDISP_ALS_RMW:
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic Read Error addr=%02X, data=%02X, mask=%02X", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{
    int ret;
    unsigned char val = 0x00;

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_BKL_ON:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);
            shdisp_bdic_API_IO_bank_set(0x00);
            ret = shdisp_bdic_API_IO_read_reg(BDIC_REG_GINF3, &val);
            SHDISP_DEBUG("DCDC1 Err Chk. ret=%d. val=0x%02x", ret, val);
            if (ret == SHDISP_RESULT_SUCCESS) {
#if defined(CONFIG_ANDROID_ENGINEERING)
                if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_BDIC_ERROR_DCDC1) {
                    val = val | SHDISP_BDIC_GINF3_DCDC1_OVD;
                    SHDISP_DEBUG("DEBUG DCDC1 Err set. val=0x%02x", val);
                }
#endif /* defined (CONFIG_ANDROID_ENGINEERING) */
                if ((val & SHDISP_BDIC_GINF3_DCDC1_OVD) == SHDISP_BDIC_GINF3_DCDC1_OVD) {
                    SHDISP_ERR("DCDC1_OVD bit ON.");
                    SHDISP_BDIC_REGSET(shdisp_bdic_dcdc1_err);
                }
            } else {
                SHDISP_ERR("BDIC GINF3 read error.");
            }
        }
        break;

    case SHDISP_BDIC_REQ_BKL_FIX_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            shdisp_bdic_bkl_fix_on[2].data  = (unsigned char)slope_fast;
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix_on);

        } else if (shdisp_bdic_bkl_before_mode != SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_bkl_fix[2].data  = (unsigned char)slope_fast;
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_fix);
        }
        break;

    case SHDISP_BDIC_REQ_BKL_AUTO_START:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto_on);

        } else if (shdisp_bdic_bkl_before_mode != SHDISP_BDIC_BKL_MODE_AUTO) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_auto);
        }
        break;

    case SHDISP_BDIC_REQ_BKL_SET_LED_VALUE:
        if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PD_BKL_set_led_value();
        } else if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO) {
            if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
                shdisp_bdic_PD_BKL_set_led_value();
            }
        }
        break;

    case SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE:
        shdisp_bdic_PD_BKL_set_opt_value();
        break;

    case SHDISP_BDIC_REQ_START:
        if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PD_BKL_set_led_value();
        } else if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_AUTO) {
            shdisp_bdic_PD_BKL_set_opt_value();
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = SHDISP_MAIN_BKL_PARAM_OFF;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_off);
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
#ifdef SHDISP_LOWBKL
        if (shdisp_bdic_lowbkl != SHDISP_BDIC_BKL_LOWBKL_OFF) {
            shdisp_bdic_set_lowbkl_mode(true);
        }
#endif /* SHDISP_LOWBKL */
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_AUTO;
        shdisp_bdic_bkl_param_auto = param;
#ifdef SHDISP_LOWBKL
        if (shdisp_bdic_lowbkl != SHDISP_BDIC_BKL_LOWBKL_OFF) {
            shdisp_bdic_set_lowbkl_mode(true);
        }
#endif /* SHDISP_LOWBKL */
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_DTV_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_dtv = SHDISP_BDIC_BKL_DTV_ON;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_EMG_MODE:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = param;
        break;

#ifdef SHDISP_LOWBKL
    case SHDISP_BDIC_REQ_BKL_LOWBKL_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_set_lowbkl_mode(false);
        break;

    case SHDISP_BDIC_REQ_BKL_LOWBKL_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_set_lowbkl_mode(true);
        break;
#endif /* SHDISP_LOWBKL */

    case SHDISP_BDIC_REQ_BKL_CHG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_CHG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_chg = SHDISP_BDIC_BKL_CHG_ON;
        break;

#ifdef SHDISP_TRV_NM2
    case SHDISP_BDIC_REQ_BKL_TRV_REQUEST:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        s_state_str.trv_param.request = param;
#ifdef SHDISP_LOWBKL
        shdisp_bdic_set_lowbkl_mode(shdisp_bdic_lowbkl);
#endif /* SHDISP_LOWBKL */
        break;
#endif /* SHDISP_TRV_NM2 */

    default:
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char reg;
    unsigned char bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPIO_0;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPIO_1;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPIO_2;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPIO_3;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPIO_4;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD5:
        reg = BDIC_REG_GPIO_5;
        bit = 0x01;
        break;
    default:
        return;
    }
    if (status == SHDISP_BDIC_GPIO_HIGH) {
        shdisp_bdic_API_IO_set_bit_reg(reg, bit);
    } else {
        shdisp_bdic_API_IO_clr_bit_reg(reg, bit);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_opt_th_shift                                               */
/* ------------------------------------------------------------------------- */
static unsigned char shdisp_bdic_PD_opt_th_shift(int index)
{
    unsigned char opt_th_shift[SHDISP_BKL_AUTO_OPT_TBL_NUM];
    opt_th_shift[0] = (0x07 & BDIC_REG_OPT_TH_SHIFT_1_0_VAL);
    opt_th_shift[1] = (0x70 & BDIC_REG_OPT_TH_SHIFT_1_0_VAL) >> 4;
    opt_th_shift[2] = (0x07 & BDIC_REG_OPT_TH_SHIFT_3_2_VAL);
    opt_th_shift[3] = (0x70 & BDIC_REG_OPT_TH_SHIFT_3_2_VAL) >> 4;
    opt_th_shift[4] = (0x07 & BDIC_REG_OPT_TH_SHIFT_4_5_VAL);
    opt_th_shift[5] = (0x70 & BDIC_REG_OPT_TH_SHIFT_4_5_VAL) >> 4;
    opt_th_shift[6] = (0x07 & BDIC_REG_OPT_TH_SHIFT_6_7_VAL);
    opt_th_shift[7] = (0x70 & BDIC_REG_OPT_TH_SHIFT_6_7_VAL) >> 4;
    opt_th_shift[8] = (0x07 & BDIC_REG_OPT_TH_SHIFT_8_9_VAL);
    opt_th_shift[9] = (0x70 & BDIC_REG_OPT_TH_SHIFT_8_9_VAL) >> 4;
    opt_th_shift[10] = (0x07 & BDIC_REG_OPT_TH_SHIFT_11_10_VAL);
    opt_th_shift[11] = (0x70 & BDIC_REG_OPT_TH_SHIFT_11_10_VAL) >> 4;
    opt_th_shift[12] = (0x07 & BDIC_REG_OPT_TH_SHIFT_13_12_VAL);
    opt_th_shift[13] = (0x70 & BDIC_REG_OPT_TH_SHIFT_13_12_VAL) >> 4;
    opt_th_shift[14] = (0x07 & BDIC_REG_OPT_TH_SHIFT_15_14_VAL);
    opt_th_shift[15] = (0x70 & BDIC_REG_OPT_TH_SHIFT_15_14_VAL) >> 4;
    opt_th_shift[16] = (0x07 & BDIC_REG_OPT_TH_SHIFT_17_16_VAL);
    opt_th_shift[17] = (0x70 & BDIC_REG_OPT_TH_SHIFT_17_16_VAL) >> 4;
    opt_th_shift[18] = (0x07 & BDIC_REG_OPT_TH_SHIFT_19_18_VAL);
    opt_th_shift[19] = (0x70 & BDIC_REG_OPT_TH_SHIFT_19_18_VAL) >> 4;
    opt_th_shift[20] = (0x07 & BDIC_REG_OPT_TH_SHIFT_21_20_VAL);
    opt_th_shift[21] = (0x70 & BDIC_REG_OPT_TH_SHIFT_21_20_VAL) >> 4;
    opt_th_shift[22] = (0x07 & BDIC_REG_OPT_TH_SHIFT_23_22_VAL);
    opt_th_shift[23] = (0x70 & BDIC_REG_OPT_TH_SHIFT_23_22_VAL) >> 4;
    if (index >= SHDISP_BKL_AUTO_OPT_TBL_NUM || index < 0) {
        SHDISP_ERR("invalid index");
    }
    return opt_th_shift[index];
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_update_led_value                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_update_led_value(void)
{
    int ret;
    unsigned short ado_tmp;

    shdisp_bdic_ado_for_brightness = SHDISP_BDIC_INVALID_ADO;
    ret = shdisp_bdic_PD_REG_ADO_get_opt(&ado_tmp, NULL, NULL);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_ado_for_brightness = (int)ado_tmp;
    }
    SHDISP_DEBUG("ado=0x%08X", (unsigned int)shdisp_bdic_ado_for_brightness);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_set_led_value                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_set_led_value(void)
{
    int i = 0, mode = 0;
    unsigned short ado_tmp;
    unsigned short ado;
    int slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_NONE;

    SHDISP_TRACE("in shdisp_bdic_bkl_mode=%d->%d", shdisp_bdic_bkl_before_mode, shdisp_bdic_bkl_mode);

    if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_OFF) {
        SHDISP_DEBUG("out: blk_off.");
        return;
    }

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &shdisp_bdic_bkl_led_value[1].data);
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &shdisp_bdic_bkl_led_value[2].data);
        if (shdisp_bdic_bkl_before_mode != SHDISP_BDIC_BKL_MODE_FIX) {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_NONE;
        } else {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_FAST;
        }
#ifdef SHDISP_LOWBKL
        if ((shdisp_bdic_bkl_slope_slow_allow == SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_LOWBKL) &&
            (shdisp_bdic_lowbkl_before != SHDISP_BDIC_BKL_LOWBKL_EXE) &&
            (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_EXE)) {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_SLOW;
        }
        if (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_OFF) {
            shdisp_bdic_lowbkl_before = SHDISP_BDIC_BKL_LOWBKL_OFF;
        }
#endif /* SHDISP_LOWBKL */
        if ((shdisp_bdic_bkl_slope_slow_allow == SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_EMG) &&
            (shdisp_bdic_emg_before == SHDISP_BDIC_BKL_EMG_OFF) &&
            (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON_LEVEL0)) {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_SLOW;
        }
        switch (slope_mode) {
        case SHDISP_BDIC_BKL_SLOPE_MODE_FAST:
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_fast);
            break;
        case SHDISP_BDIC_BKL_SLOPE_MODE_SLOW:
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_slow);
            break;
        }
        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:
        if (shdisp_bdic_ado_for_brightness >= 0) {
            ado_tmp = (unsigned short)shdisp_bdic_ado_for_brightness;
            shdisp_bdic_ado_for_brightness = SHDISP_BDIC_INVALID_ADO;
        } else {
            shdisp_bdic_PD_REG_ADO_get_opt(&ado_tmp, NULL, NULL);
        }
        ado = (unsigned long)ado_tmp << 4;
        if (ado <= SHDISP_BDIC_BKL_AUTO_FIX_PARAM_MIN_ADO) {
            shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, 0, &shdisp_bdic_bkl_led_value[1].data);
            shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, 0, &shdisp_bdic_bkl_led_value[2].data);
        } else {
            for (i = 0; i < SHDISP_BKL_AUTO_OPT_TBL_NUM; i++) {
                if ((ado >> shdisp_bdic_PD_opt_th_shift(i)) <= shdisp_bdic_bkl_ado_index[i]) {
                    shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &shdisp_bdic_bkl_led_value[1].data);
                    shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &shdisp_bdic_bkl_led_value[2].data);
                    break;
                }
            }
        }
        SHDISP_DEBUG("ado=0x%04X, current=%02x, index=%02x", (unsigned int)ado, shdisp_bdic_bkl_led_value[1].data, i);
        break;
    default:

        return;
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_set_opt_value                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_BKL_set_opt_value(void)
{
    int i;
    int mode = 0;
    int slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_FAST;

    SHDISP_TRACE("in shdisp_bdic_bkl_mode=%d->%d", shdisp_bdic_bkl_before_mode, shdisp_bdic_bkl_mode);

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_OFF:
        break;

    case SHDISP_BDIC_BKL_MODE_FIX:
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        for (i = 0; i < SHDISP_BKL_AUTO_OPT_TBL_NUM; i++) {
            shdisp_bdic_LD_LCD_BKL_get_pwm_param(mode, i, &(shdisp_bdic_bkl_opt_value[i].data));
        }
#ifdef SHDISP_LOWBKL
        if ((shdisp_bdic_bkl_slope_slow_allow == SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_LOWBKL) &&
            (shdisp_bdic_lowbkl_before != SHDISP_BDIC_BKL_LOWBKL_EXE) &&
            (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_EXE)) {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_SLOW;
        }
        if (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_OFF) {
            shdisp_bdic_lowbkl_before = SHDISP_BDIC_BKL_LOWBKL_OFF;
        }
#endif /* SHDISP_LOWBKL */
        if ((shdisp_bdic_bkl_slope_slow_allow == SHDISP_BDIC_BKL_SLOPE_SLOW_ALLOW_EMG) &&
            (shdisp_bdic_emg_before == SHDISP_BDIC_BKL_EMG_OFF) &&
            (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON_LEVEL0)) {
            slope_mode = SHDISP_BDIC_BKL_SLOPE_MODE_SLOW;
        }
        switch (slope_mode) {
        case SHDISP_BDIC_BKL_SLOPE_MODE_FAST:
            shdisp_bdic_set_opt_value_fast();
            break;
        case SHDISP_BDIC_BKL_SLOPE_MODE_SLOW:
            shdisp_bdic_set_opt_value_slow();
            break;
        }
        break;

    default:
        break;
    }

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_pwm_param                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_LCD_BKL_get_pwm_param(int mode, int level, unsigned char *opt_val)
{
    unsigned long  pwm_val;
    unsigned short min_val;
    unsigned short max_val;
    int bkl_param_auto;

    bkl_param_auto = shdisp_bdic_LD_LCD_BKL_param_revision(shdisp_bdic_bkl_param_auto);

#ifdef SHDISP_TRV_NM2
    if (s_state_str.trv_param.request == SHDISP_TRV_PARAM_ON) {
        bkl_param_auto = shdisp_bdic_LD_LCD_BKL_trv_level_calculation(bkl_param_auto, l_limit_a);
    }
#endif /* SHDISP_TRV_NM2 */

    min_val = shdisp_main_bkl_min_tbl[level];
    max_val = shdisp_main_bkl_max_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL0:
    case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL1:
#ifdef SHDISP_LOWBKL
        if (shdisp_bdic_lowbkl == SHDISP_BDIC_BKL_LOWBKL_EXE) {
            if (bkl_param_auto > SHDISP_MAIN_BKL_LOWBKL_AUTO_DIVIDE_THRESHOLD) {
                bkl_param_auto = ((bkl_param_auto * SHDISP_MAIN_BKL_LOWBKL_AUTO_DIVIDE) / 10);
                if (bkl_param_auto < SHDISP_MAIN_BKL_PARAM_MIN_AUTO) {
                    bkl_param_auto = SHDISP_MAIN_BKL_PARAM_MIN_AUTO;
                }
            } else {
                bkl_param_auto = SHDISP_MAIN_BKL_LOWBKL_AUTO_VAL;
            }
        }
#endif /* SHDISP_LOWBKL */
        if (bkl_param_auto <= SHDISP_MAIN_BKL_PARAM_MIN_AUTO) {
            pwm_val = (unsigned long)min_val;
        } else if (bkl_param_auto >= SHDISP_MAIN_BKL_PARAM_MAX_AUTO) {
            pwm_val = (unsigned long)max_val;
        } else {
            pwm_val = (unsigned long)((max_val - min_val) * (bkl_param_auto - 2));
            pwm_val /= (unsigned char)SHDISP_BKL_AUTO_STEP_NUM;
            pwm_val += (unsigned long)min_val;
        }
        switch (mode) {
        case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL0:
            if (pwm_val > SHDISP_BKL_EMERGENCY_LIMIT_AUTO_LEVEL0) {
                pwm_val = SHDISP_BKL_EMERGENCY_LIMIT_AUTO_LEVEL0;
            }
            break;
        case SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL1:
            if (pwm_val > SHDISP_BKL_EMERGENCY_LIMIT_AUTO_LEVEL1) {
                pwm_val = SHDISP_BKL_EMERGENCY_LIMIT_AUTO_LEVEL1;
            }
            break;
        }
        break;
    default:
        if (level == 0) {
            pwm_val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT_ZERO;
        } else {
            pwm_val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT;
        }
        break;
    }

    if (pwm_val < (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT) {
        if (level == 0) {
            if (pwm_val < (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT_ZERO) {
                pwm_val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT_ZERO;
            }
        } else {
            pwm_val = (unsigned long)SHDISP_BKL_PWM_LOWER_LIMIT;
        }
    } else if (pwm_val > (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT) {
        pwm_val = (unsigned long)SHDISP_BKL_PWM_UPPER_LIMIT;
    } else {
        ;
    }

    pwm_val *= (unsigned char)SHDISP_BKL_CURRENT_UPPER_LIMIT;
    pwm_val /= (unsigned short)SHDISP_BKL_PWM_UPPER_LIMIT;
    *opt_val = shdisp_bdic_API_correct_proh_val((unsigned char)pwm_val);
    SHDISP_INFO("mode=%d, param=%d, pwm=0x%2lX, opt_val=0x%2X", mode, bkl_param_auto, pwm_val, *opt_val);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_get_sensor_state                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_get_sensor_state(void)
{
    int ret;
    unsigned char reg = SENSOR_REG_COMMAND1;
    unsigned char val = 0x00;
    SHDISP_TRACE("in");

    ret = shdisp_bdic_API_IO_psals_read_reg(reg, &val);

    if (ret != SHDISP_RESULT_SUCCESS) {
        val = 0x00;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_PSALS) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force sensor state error.");
        val = 0x00;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_TRACE("out");
    return val;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_wait4i2ctimer_stop                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_wait4i2ctimer_stop(void)
{
    int waitcnt;
    int retry_cnt = 3;
    int ret;
    unsigned char reg = BDIC_REG_SYSTEM8;
    unsigned char val = 0x00;

    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_stop);

    for (waitcnt = 0; waitcnt < retry_cnt; waitcnt++) {
        shdisp_IO_API_delay_us(BDIC_WAIT_TIMER_STOP);
        ret = shdisp_bdic_API_IO_read_reg(reg, &val);

        if ((ret == SHDISP_RESULT_SUCCESS) &&
            (val == 0xA0)) {
                break;
        }

        SHDISP_WARN("retry(%d)!! SYSTEM8 = 0x%02x", waitcnt, val);

    }

    if (waitcnt == retry_cnt) {
        SHDISP_ERR("i2ctimer wait failed.");
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_i2c_throughmode_ctrl                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_i2c_throughmode_ctrl(bool ctrl)
{

    if (ctrl) {
        shdisp_bdic_PD_wait4i2ctimer_stop();
        SHDISP_BDIC_REGSET(shdisp_bdic_i2c_throughmode_on);
    } else {
        SHDISP_BDIC_REGSET(shdisp_bdic_i2c_throughmode_off);
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_on(void)
{
    int sensor_state;
    int i;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_on);
    SHDISP_BDIC_REGSET(shdisp_bdic_psals_init);

    sensor_state = shdisp_bdic_PD_get_sensor_state();
    if (sensor_state == 0x00) {
        SHDISP_ERR("psals poweron(first) failed!! sensor_state = %02x", sensor_state);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PSALS;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_PSALS_ON_NG;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */

        for (i = 0; i < 3; i++) {
            SHDISP_BDIC_REGSET(shdisp_bdic_psals_init);

            sensor_state = shdisp_bdic_PD_get_sensor_state();
            if (sensor_state != 0x00) {
                break;
            }
            SHDISP_WARN("try psals poweron failed(%d)!! sensor_state = %02x", i + 1, sensor_state);
        }

        if (i == 3) {
            SHDISP_ERR("psals poweron retry over!!");
            if (psals_recovery_flag == SHDISP_BDIC_PSALS_RECOVERY_DURING) {
                psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER;
            }
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_PSALS;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_POWER_ON_NG;
            shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        }
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_power_off                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_power_off(void)
{
    SHDISP_TRACE("in");
    SHDISP_BDIC_REGSET(shdisp_bdic_sensor_power_off);
    psals_recovery_flag = SHDISP_BDIC_PSALS_RECOVERY_NONE;
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_off                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_off3);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_init_als_on                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_init_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_set_als_shift_ps_on);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on3);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out5");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_init_als_on4);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out6");
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_off                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_ps_deinit_als_on                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_ps_deinit_als_on(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_set_als_shift_ps_off);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_ps_deinit_als_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_off                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_set_als_shift_ps_off);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_init_ps_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out43");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_init_ps_on                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_init_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_set_als_shift_ps_on);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_init_ps_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4");
        return ret;
    }
    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_off                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_off(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_off1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_off2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    shdisp_bdic_before_range    = SHDISP_BDIC_INVALID_RANGE;
    shdisp_bdic_before_ado      = 0;

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_als_deinit_ps_on                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_als_deinit_ps_on(void)
{
    int ret;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2");
        return ret;
    }

    ret = SHDISP_BDIC_REGSET(shdisp_bdic_als_deinit_ps_on2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        return ret;
    }

    shdisp_bdic_before_range    = SHDISP_BDIC_INVALID_RANGE;
    shdisp_bdic_before_ado      = 0;

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_psals_write_threshold                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_psals_write_threshold(struct shdisp_prox_params *prox_params)
{
    unsigned char temp_low[2];
    unsigned char temp_high[2];
    if (!prox_params) {
        return SHDISP_RESULT_FAILURE;
    }

    temp_low[0] = (unsigned char)(prox_params->threshold_low & 0x00FF);
    temp_low[1] = (unsigned char)((prox_params->threshold_low >> 8) & 0x00FF);
    temp_high[0] = (unsigned char)(prox_params->threshold_high & 0x00FF);
    temp_high[1] = (unsigned char)((prox_params->threshold_high >> 8) & 0x00FF);
    SHDISP_DEBUG("prox_pramams from: low = 0x%02x, 0x%02x: high = 0x%02x, 0x%02x",
            temp_low[0], temp_low[1], temp_high[0], temp_high[1]);

    PSALS_CORRECT_PROH_LOW_VAL(temp_low[0])
    PSALS_CORRECT_PROH_LOW_VAL(temp_low[1])
    PSALS_CORRECT_PROH_HIGH_VAL(temp_high[0])
    PSALS_CORRECT_PROH_HIGH_VAL(temp_high[1])

    shdisp_bdic_ps_init_set_threshold[0].data = temp_low[0];
    shdisp_bdic_ps_init_set_threshold[1].data = temp_low[1];
    shdisp_bdic_ps_init_set_threshold[2].data = temp_high[0];
    shdisp_bdic_ps_init_set_threshold[3].data = temp_high[1];

    SHDISP_DEBUG("prox_pramams to  : low = 0x%02x, 0x%02x: high = 0x%02x, 0x%02x",
            temp_low[0], temp_low[1], temp_high[0], temp_high[1]);
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_ADO_get_opt                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_REG_ADO_get_opt(unsigned short *ado, unsigned short *clear, unsigned short *ir)
{
    int ret, shift_tmp;
    unsigned long ado0, ado1;
    unsigned long als0, als1;
    unsigned short alpha, beta, gmm;
    unsigned char rval[(SENSOR_REG_D1_MSB + 1) - SENSOR_REG_D0_LSB];
    signed char range0, res;
    unsigned int retry, flag_a;

    SHDISP_TRACE("in");

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);
    ret = shdisp_bdic_API_IO_psals_read_reg(SENSOR_REG_COMMAND2, rval);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        goto i2c_err;
    }
    range0 = rval[0] & 0x07;
    res    = (rval[0] & 0x38) >> 3;
    SHDISP_DEBUG("range0=%d, res=%d", range0, res);

    if (res != 0x05) {
        for (retry = 0; retry < SHDISP_BDIC_GET_ADO_RETRY_TIMES; retry++) {
            ret = shdisp_bdic_API_IO_psals_read_reg(SENSOR_REG_COMMAND1, rval);
            if (ret != SHDISP_RESULT_SUCCESS) {
                SHDISP_ERR("out2");
                goto i2c_err;
            }
            flag_a = (rval[0] & 0x02) >> 1;
            if (flag_a == 1) {
                break;
            }

            shdisp_IO_API_delay_us(25 * 1000);
        }
        SHDISP_DEBUG("retry=%d", retry);
        if (retry >= SHDISP_BDIC_GET_ADO_RETRY_TIMES) {
            SHDISP_WARN("als flag_a check retry over");
        }
    }

    ret = shdisp_bdic_API_IO_psals_burst_read_reg(SENSOR_REG_D0_LSB, rval, sizeof(rval));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        goto i2c_err;
    }
    als0 = (rval[1] << 8 | rval[0]);
    als1 = (rval[3] << 8 | rval[2]);
    SHDISP_DEBUG("als1*16=%ld, als0*15=%ld", als1 * SHDISP_BDIC_RATIO_OF_ALS0, als0 * SHDISP_BDIC_RATIO_OF_ALS1);

    if ((als1 * SHDISP_BDIC_RATIO_OF_ALS0) > (als0 * SHDISP_BDIC_RATIO_OF_ALS1)) {
        alpha = s_state_str.photo_sensor_adj.als_adjust[1].als_adj0;
        beta  = s_state_str.photo_sensor_adj.als_adjust[1].als_adj1;
        gmm = s_state_str.photo_sensor_adj.als_adjust[1].als_shift;
        if (gmm < 16) {
            ado0 = (((als0 * alpha) - (als1 * beta)) << gmm) >> 15;
        } else {
            ado0 = (((als0 * alpha) - (als1 * beta)) >> (32 - gmm)) >> 15;
        }
        SHDISP_DEBUG("ROUTE-1 als0=%04lX, als1=%04lX, alpha=%04X, beta=%04X, gmm=%02x, ado0=%08lx", als0, als1, alpha, beta, gmm, ado0);
    } else {
        alpha = s_state_str.photo_sensor_adj.als_adjust[0].als_adj0;
        beta  = s_state_str.photo_sensor_adj.als_adjust[0].als_adj1;
        gmm = s_state_str.photo_sensor_adj.als_adjust[0].als_shift;
        if (gmm < 16) {
            ado0 = (((als0 * alpha) - (als1 * beta)) << gmm) >> 15;
        } else {
            ado0 = (((als0 * alpha) - (als1 * beta)) >> (32 - gmm)) >> 15;
        }
        SHDISP_DEBUG("ROUTE-2 als0=%04lX, als1=%04lX, alpha=%04X, beta=%04X, gmm=%02x, ado0=%08lx", als0, als1, alpha, beta, gmm, ado0);
    }

    if (res < 3) {
        shift_tmp = (res - 3) + (range0 - 4);
    } else {
        shift_tmp = ((res - 3) * 2) + (range0 - 4);
    }

    if (shift_tmp < 0) {
        shift_tmp = (-1) * shift_tmp;
        ado1 = ado0 >> shift_tmp;
    } else {
        ado1 = ado0 << shift_tmp;
    }

    if (ado1 > SHDISP_BDIC_MAX_ADO_VALUE) {
        ado1 = SHDISP_BDIC_MAX_ADO_VALUE;
    }
    if (ado) {
        *ado = (unsigned short)ado1;
    }
    if (clear) {
        *clear = (unsigned short)als0;
    }
    if (ir) {
        *ir = (unsigned short)als1;
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_reg_ar_ctrl);
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    shdisp_IO_API_delay_us(1000);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;

i2c_err:
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_RAW_DATA_get_opt                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_REG_RAW_DATA_get_opt(unsigned short *clear, unsigned short *ir)
{
    int retry, ret;
    unsigned short als0, als1;
    unsigned char rval[(SENSOR_REG_D1_MSB + 1) - SENSOR_REG_D0_LSB];
    unsigned char flag_a;

    SHDISP_TRACE("in");

    shdisp_bdic_PD_i2c_throughmode_ctrl(true);

    for (retry = 0; retry < SHDISP_BDIC_GET_ADO_RETRY_TIMES; retry++) {
        ret = shdisp_bdic_API_IO_psals_read_reg(SENSOR_REG_COMMAND1, rval);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out2");
            goto i2c_err;
        }
        flag_a = (rval[0] & 0x02) >> 1;
        if (flag_a == 1) {
            break;
        }
        shdisp_IO_API_delay_us(25 * 1000);
    }
    SHDISP_DEBUG("retry=%d", retry);

    ret = shdisp_bdic_API_IO_psals_burst_read_reg(SENSOR_REG_D0_LSB, rval, sizeof(rval));
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3");
        goto i2c_err;
    }
    als0 = (rval[1] << 8 | rval[0]);
    als1 = (rval[3] << 8 | rval[2]);

    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    shdisp_IO_API_delay_us(1000);
    *clear = als0;
    *ir = als1;

    SHDISP_DEBUG("als0=0x%04X, als1=0x%04x", (unsigned int)als0, (unsigned int)als1);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;

i2c_err:
    shdisp_bdic_PD_i2c_throughmode_ctrl(false);
    return SHDISP_RESULT_FAILURE;
}

#ifdef SHDISP_ALS_INT
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_REG_int_setting                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_REG_int_setting(struct shdisp_photo_sensor_int_trigger *trigger)
{
    unsigned char opt_int_val1 = 0;
    unsigned char opt_int_val2 = 0;
    unsigned char gimr2_val = 0;

    if (trigger->trigger1.enable) {
        SHDISP_DEBUG("trigger1 level(%d) side(%d) hi_edge(%d) lo_edge(%d)", trigger->trigger1.level, trigger->trigger1.side,
                                                                            trigger->trigger1.en_hi_edge, trigger->trigger1.en_lo_edge);
        gimr2_val = gimr2_val | SHDISP_BDIC_OPT_TABLE1_IMR;
        opt_int_val1 = trigger->trigger1.level;
        if (trigger->trigger1.side) {
            opt_int_val1 = opt_int_val1 | SHDISP_BDIC_OPT_TH_SIDE;
        }
        if (trigger->trigger1.en_hi_edge) {
            opt_int_val1 = opt_int_val1 | SHDISP_BDIC_OPT_H_EDGE_EN;
        }
        if (trigger->trigger1.en_lo_edge) {
            opt_int_val1 = opt_int_val1 | SHDISP_BDIC_OPT_L_EDGE_EN;
        }
    }
    if (trigger->trigger2.enable) {
        SHDISP_DEBUG("trigger2 level(%d) side(%d) hi_edge(%d) lo_edge(%d)", trigger->trigger2.level, trigger->trigger2.side,
                                                                            trigger->trigger2.en_hi_edge, trigger->trigger2.en_lo_edge);
        gimr2_val = gimr2_val | SHDISP_BDIC_OPT_TABLE2_IMR;
        opt_int_val2 = trigger->trigger2.level;
        if (trigger->trigger2.side) {
            opt_int_val2 = opt_int_val2 | SHDISP_BDIC_OPT_TH_SIDE;
        }
        if (trigger->trigger2.en_hi_edge) {
            opt_int_val2 = opt_int_val2 | SHDISP_BDIC_OPT_H_EDGE_EN;
        }
        if (trigger->trigger2.en_lo_edge) {
            opt_int_val2 = opt_int_val2 | SHDISP_BDIC_OPT_L_EDGE_EN;
        }
    }
    SHDISP_DEBUG("write OPT_INT1(0x%x) OPT_INT2(0x%x) GIMR2(0x%x)", opt_int_val1, opt_int_val2, gimr2_val);
    shdisp_bdic_reg_als_int_setting[1].data = opt_int_val1;
    shdisp_bdic_reg_als_int_setting[2].data = opt_int_val2;
    shdisp_bdic_reg_als_int_setting[3].data = gimr2_val;

    SHDISP_BDIC_REGSET(shdisp_bdic_reg_als_int_setting);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_chk_als_trigger                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_chk_als_trigger(struct shdisp_photo_sensor_trigger *trigger)
{
    int ret = SHDISP_RESULT_SUCCESS;

    if ((trigger->level < 0) || (trigger->level >= SHDISP_BKL_AUTO_OPT_TBL_NUM)) {
        ret = SHDISP_RESULT_FAILURE;
    }
    if ((trigger->side < 0) || (trigger->side > 1)) {
        ret = SHDISP_RESULT_FAILURE;
    }
    if ((trigger->en_hi_edge < 0) || (trigger->en_hi_edge > 1)) {
        ret = SHDISP_RESULT_FAILURE;
    }
    if ((trigger->en_lo_edge < 0) || (trigger->en_lo_edge > 1)) {
        ret = SHDISP_RESULT_FAILURE;
    }
    if ((trigger->enable < 0) || (trigger->enable > 1)) {
        ret = SHDISP_RESULT_FAILURE;
    }

    return ret;
}
#endif /* SHDISP_ALS_INT */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_get_ave_ado                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_get_ave_ado(struct shdisp_ave_ado *ave_ado)
{
    int i, ret;
    unsigned long als0 = 0;
    unsigned long als1 = 0;
    unsigned long ado  = 0;
    unsigned char rvalL, rvalH;

    SHDISP_TRACE("in");

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    SHDISP_BDIC_BACKUP_REGS_BDIC(shdisp_bdic_restore_regs_for_ave_ado_bdic);
    SHDISP_BDIC_BACKUP_REGS_ALS(shdisp_bdic_restore_regs_for_ave_ado_als);

    SHDISP_BDIC_REGSET(shdisp_bdic_ave_ado_param1);
    shdisp_bdic_ave_ado_param2[2].data = (0x18 | (ave_ado->als_range & 0x07));
    if (shdisp_bdic_restore_regs_for_ave_ado_als[1].data != shdisp_bdic_ave_ado_param2[2].data) {
        SHDISP_BDIC_REGSET(shdisp_bdic_ave_ado_param2);
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);

    for (i = 0; i < SHDISP_BDIC_AVE_ADO_READ_TIMES; i++) {
        SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_stop_ave_ado);

        shdisp_bdic_API_IO_read_reg(BDIC_REG_I2C_RDATA0, &rvalL);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_I2C_RDATA1, &rvalH);
        als0 += (unsigned long)((rvalH << 8) | rvalL);

        shdisp_bdic_API_IO_read_reg(BDIC_REG_I2C_RDATA2, &rvalL);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_I2C_RDATA3, &rvalH);
        als1 += (unsigned long)((rvalH << 8) | rvalL);

        shdisp_bdic_API_IO_read_reg(BDIC_REG_ADOL, &rvalL);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_ADOH, &rvalH);
        ado  += (unsigned long)((rvalH << 8) | rvalL);

        SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);
    }

    ret = shdisp_bdic_PD_wait4i2ctimer_stop();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1");
        return ret;
    }

    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_ave_ado_bdic);
    SHDISP_BDIC_RESTORE_REGS(shdisp_bdic_restore_regs_for_ave_ado_als);
    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start_ave_ado);

    ave_ado->ave_als0 = (unsigned short)(als0 / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    ave_ado->ave_als1 = (unsigned short)(als1 / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    ave_ado->ave_ado  = (unsigned short)(ado  / SHDISP_BDIC_AVE_ADO_READ_TIMES);
    SHDISP_DEBUG("ave_als0:0x%04X ave_als1:0x%04X ave_ado:0x%04x",
                            ave_ado->ave_als0, ave_ado->ave_als1, ave_ado->ave_ado);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_shift_ps_on_table_adjust                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_als_shift_ps_on_table_adjust(struct shdisp_photo_sensor_adj *adj)
{
    unsigned char als_shift1;
    unsigned char als_shift2;

    als_shift1 = (adj->als_adjust[1].als_shift & 0x1F);
    if ((als_shift1 == 0x0E) || (als_shift1 == 0x0F)) {
        als_shift1 = 0x0F;
    } else {
        als_shift1 = (als_shift1 + 0x02) & 0x1F;
    }

    als_shift2 = (adj->als_adjust[0].als_shift & 0x1F);
    if ((als_shift2 == 0x0E) || (als_shift2 == 0x0F)) {
        als_shift2 = 0x0F;
    } else {
        als_shift2 = (als_shift2 + 0x02) & 0x1F;
    }

    shdisp_bdic_set_als_shift_ps_on[1].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_on[2].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_on[3].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_on[4].data = als_shift2;

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_shift_ps_off_table_adjust                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_als_shift_ps_off_table_adjust(struct shdisp_photo_sensor_adj *adj)
{
    unsigned char als_shift1;
    unsigned char als_shift2;

    als_shift1 = (adj->als_adjust[1].als_shift & 0x1F);
    als_shift2 = (adj->als_adjust[0].als_shift & 0x1F);

    shdisp_bdic_set_als_shift_ps_off[1].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_off[2].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_off[3].data = als_shift1;
    shdisp_bdic_set_als_shift_ps_off[4].data = als_shift2;

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_als_user_to_devtype                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_als_user_to_devtype(int sensor_type)
{
    int dev_type = NUM_SHDISP_PHOTO_SENSOR_TYPE;

    switch (sensor_type) {
    case SHDISP_PHOTO_SENSOR_TYPE_APP:
        dev_type = SHDISP_DEV_TYPE_ALS_APP;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_CAMERA:
        dev_type = SHDISP_DEV_TYPE_CAMERA;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_KEYLED:
        dev_type = SHDISP_DEV_TYPE_KEYLED;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_DIAG:
        dev_type = SHDISP_DEV_TYPE_DIAG;
        break;
    case SHDISP_PHOTO_SENSOR_TYPE_SENSORHUB:
        dev_type = SHDISP_DEV_TYPE_SENSORHUB;
        break;
    default:
        break;
    }

    return dev_type;
}

#ifdef SHDISP_LOWBKL
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_lowbkl_mode                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_set_lowbkl_mode(int onoff)
{
    shdisp_bdic_lowbkl_before = shdisp_bdic_lowbkl;

    if (onoff) {
        switch (shdisp_bdic_bkl_mode) {
        case SHDISP_BDIC_BKL_MODE_AUTO:
            if (shdisp_bdic_bkl_param_auto > SHDISP_MAIN_BKL_LOWBKL_AUTO_PARAM_THRESHOLD) {
                shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_EXE;
            } else {
                shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_ON;
            }
            break;
        case SHDISP_BDIC_BKL_MODE_FIX:
            if (shdisp_bdic_bkl_param > SHDISP_MAIN_BKL_LOWBKL_FIX_PARAM_THRESHOLD) {
                shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_EXE;
            } else {
                shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_ON;
            }
            break;
        }
#ifdef SHDISP_TRV_NM2
        if (s_state_str.trv_param.request == SHDISP_TRV_PARAM_ON) {
            shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_ON;
        }
#endif /* SHDISP_TRV_NM2 */
    } else {
        shdisp_bdic_lowbkl = SHDISP_BDIC_BKL_LOWBKL_OFF;
    }
    if (shdisp_bdic_lowbkl_before != shdisp_bdic_lowbkl) {
        SHDISP_DEBUG("Change lowbkl mode. before=%d. after=%d", shdisp_bdic_lowbkl_before, shdisp_bdic_lowbkl);
    }
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_LOWBKL */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_opt_value_slow                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_set_opt_value_slow(void)
{
    int ret = 0;
    unsigned char val = 0x00;

    SHDISP_TRACE("in");

    shdisp_bdic_PD_wait4i2ctimer_stop();

    shdisp_bdic_API_IO_bank_set(0x00);
    ret = shdisp_bdic_API_IO_read_reg(BDIC_REG_NLED1H, &val);
    SHDISP_DEBUG("NLED1H read. ret=%d. val=0x%02x", ret, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_bkl_led_value[1].data = val;
        shdisp_bdic_bkl_led_value[2].data = val;
    } else {
        SHDISP_ERR("BDIC NLED1H read error.");
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_off);

    shdisp_bdic_API_IO_bank_set(0x01);
    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_value);
    shdisp_bdic_API_IO_bank_set(0x00);

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_on);

    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_opt_value_fast                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_set_opt_value_fast(void)
{
    unsigned char ado_idx = 0x00;

    SHDISP_TRACE("in");

    shdisp_bdic_PD_wait4i2ctimer_stop();

    if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO || shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX) {
        shdisp_bdic_bkl_slope_fast[1].data  = (unsigned char)slope_fast;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_fast);
        shdisp_bdic_API_IO_read_reg(BDIC_REG_ADO_INDEX, &ado_idx);
        ado_idx = ado_idx & 0x1F;
        if (ado_idx >= SHDISP_BKL_AUTO_OPT_TBL_NUM) {
            ado_idx = SHDISP_BKL_AUTO_OPT_TBL_NUM - 1;
        }
        shdisp_bdic_bkl_led_value[1].data   = shdisp_bdic_bkl_opt_value[ado_idx].data;
        shdisp_bdic_bkl_led_value[2].data   = shdisp_bdic_bkl_opt_value[ado_idx].data;
    }

    shdisp_bdic_API_IO_bank_set(0x01);
    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_value);
    shdisp_bdic_API_IO_bank_set(0x00);

    if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO || shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX) {
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_off);
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);
        shdisp_IO_API_delay_us(1000 * mled_delay_ms1);
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_opt_mode_on);
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_i2ctimer_start);
    if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_AUTO || shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_FIX) {
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_slope_slow);
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
