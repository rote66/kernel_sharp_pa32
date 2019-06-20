/* drivers/sharp/shdisp/shdisp_hayabusa.c  (Display Driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
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
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/msm_mdp.h>
#include <linux/clk.h>
#include "shdisp_panel.h"
#include "shdisp_hayabusa.h"
#include "shdisp_system.h"
#include "shdisp_io.h"
#include "shdisp_bdic.h"
#include "shdisp_type.h"
#include "shdisp_pm.h"
#include "shdisp_dbg.h"
#include "shdisp_kerl_priv.h"
#include <sharp/sh_smem.h>
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_POWER_MODE_CHK

#define SHDISP_HAYABUSA_VCOM_REG_NUM                (7)

#define SHDISP_HAYABUSA_GMM_SETTING_SIZE          (60)
#define SHDISP_HAYABUSA_GMM_LEVEL_MIN             (1)
#define SHDISP_HAYABUSA_GMM_LEVEL_MAX             (30)
#define SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET       (30)
#define SHDISP_HAYABUSA_GMM_GROUP_BELONG_LEVEL    (2)
#define SHDISP_HAYABUSA_GMM_GROUP_BELONG_ADDR     (4)

#define SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE     (5)

#define SHDISP_HAYABUSA_CUT_ID                      (0x80)

#ifdef SHDISP_HAYABUSA_VDD
#define SHDISP_HAYABUSA_VDDIO_ID                    "hayabusa_vddio"
#endif /* SHDISP_HAYABUSA_VDD */

#define SHDISP_HAYABUSA_BBCLK2_ID                   "bbclk2"


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_hayabusa_calc_vcom_in {
    unsigned short vcom;
    unsigned short vcom_low;
};

struct shdisp_hayabusa_calc_vcom_out {
    char vcom1_l;
    char vcom2_l;
    char vcom12_h;
    char lpvcom1;
    char lpvcom2;
    char vcomoff_l;
    char vcomoff_h;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_calc_vcom_param(struct shdisp_hayabusa_calc_vcom_in in, struct shdisp_hayabusa_calc_vcom_out *out, unsigned short vcomoffset);
static int shdisp_hayabusa_init_flicker_param(unsigned short vcom, unsigned short vcom_low);
static int shdisp_hayabusa_API_init_io(struct shdisp_panel_context *panel_ctx);
static int shdisp_hayabusa_API_exit_io(void);
static int shdisp_hayabusa_API_power_on(int mode);
static int shdisp_hayabusa_API_power_off(int mode);
static int shdisp_hayabusa_API_disp_on(void);
static int shdisp_hayabusa_API_disp_off(void);
static int shdisp_hayabusa_API_start_display(void);
static int shdisp_hayabusa_API_post_video_start(void);
static int shdisp_hayabusa_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size);
static int shdisp_hayabusa_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size);
static int shdisp_hayabusa_API_diag_set_flicker_param(struct shdisp_diag_flicker_param vcom);
static int shdisp_hayabusa_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_hayabusa_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *vcom);
static int shdisp_hayabusa_API_check_recovery(void);
static int shdisp_hayabusa_API_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_hayabusa_API_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info);
static int shdisp_hayabusa_API_diag_set_gmm(struct shdisp_diag_gamma *gmm);
static int shdisp_hayabusa_API_shutdown(void);
static void shdisp_hayabusa_API_dump(int type);
static int shdisp_hayabusa_API_set_irq(int enable);
static int shdisp_hayabusa_API_mfr(int mfr);
static int shdisp_hayabusa_API_chg_mfr_param(struct shdisp_panel_mode p_mode);
static int shdisp_hayabusa_API_set_freq_param(struct shdisp_freq_params *freq);
static void shdisp_hayabusa_hw_reset(bool);
static void shdisp_hayabusa_cmds_initial20_set(const char powercontrol[]);
static void shdisp_hayabusa_pow_ctl_flicker_exchange(void);
static void shdisp_hayabusa_pow_ctl_app_vol_exchange(void);
static int shdisp_hayabusa_mipi_cmd_lcd_on(void);
static int shdisp_hayabusa_mipi_cmd_lcd_off(void);
static int shdisp_hayabusa_mipi_cmd_display_on(void);
static int shdisp_hayabusa_set_switchcommand(char val);
static int shdisp_panel_hayabusa_reg_read(unsigned char addr, unsigned char *out_data);
static int shdisp_hayabusa_mipi_mfr_param_setting(int id, int mfr);
static int shdisp_hayabusa_mipi_mfr_setting(int id, int mfr);
static int shdisp_hayabusa_get_mfr_brightness_id(struct shdisp_panel_mode p_mode);

#ifndef SHDISP_NOT_SUPPORT_FLICKER
static int shdisp_hayabusa_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param);
static int shdisp_hayabusa_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_hayabusa_power_mode_chk(unsigned char addr);
#endif /* SHDISP_POWER_MODE_CHK */

static int shdisp_hayabusa_sleepout_wait_dsi(void);
static int shdisp_hayabusa_sleepout_wait_proc1(void);
static int shdisp_hayabusa_sleepout_wait_proc2(void);
static int shdisp_hayabusa_devicecode_read(int *version);

#ifndef SHDISP_NOT_SUPPORT_NO_OS
static int shdisp_hayabusa_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm);
#endif /* SHDISP_NOT_SUPPORT_NO_OS */
extern void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
extern int mdss_shdisp_dsi_bus_clk_ctrl(bool enable);
static irqreturn_t shdisp_hayabusa_int_isr(int irq_num, void *data);
static int shdisp_hayabusa_check_mipi_err(void);
static int shdisp_hayabusa_register_driver(void);
static void shdisp_hayabusa_workqueue_handler(struct work_struct *work);

#ifdef SHDISP_HAYABUSA_VDD
static int shdisp_hayabusa_vddio_on(void);
static int shdisp_hayabusa_vddio_off(void);
static void shdisp_hayabusa_vdd_on(void);
static void shdisp_hayabusa_vdd_off(void);
#endif /* SHDISP_HAYABUSA_VDD */
static void shdisp_hayabusa_power_off_for_shutdown(void);

static int shdisp_hayabusa_external_clk_ctl(int enable);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_NOT_SUPPORT_FLICKER
static unsigned char hayabusa_wdata[SHDISP_HAYABUSA_VCOM_REG_NUM];
static unsigned char hayabusa_rdata[SHDISP_HAYABUSA_VCOM_REG_NUM];
static char Hayabusa_VCOM_Reg[SHDISP_HAYABUSA_VCOM_REG_NUM] = {0x13, 0x14, 0x15, 0x5A, 0x5B, 0x5C, 0x5E};
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

#ifdef SHDISP_POWER_MODE_CHK
static int shdisp_dispon_chk = SHDISP_PANEL_DISPON_CHK_INIT;
#endif /* SHDISP_POWER_MODE_CHK */

static struct shdisp_panel_context shdisp_panel_ctx;
static struct workqueue_struct    *shdisp_wq_hayabusa = NULL;
static struct work_struct         shdisp_wq_hayabusa_wk;
static int shdisp_hayabusa_irq_gpio = 0;
static int shdisp_hayabusa_rst_gpio = 0;
#ifdef SHDISP_HAYABUSA_ADDITIONAL_VDD
static int shdisp_hayabusa_vdd_gpio = 0;
#endif /* SHDISP_HAYABUSA_ADDITIONAL_VDD */
static int shdisp_hayabusa_mipi_err_gpio = 0;
static struct platform_device *pshdisp_hayabusa_irq_port_dev = NULL;
static struct wake_lock shdisp_hayabusa_wakelock;
#ifdef SHDISP_HAYABUSA_VDD
static struct regulator *shdisp_hayabusa_vddio = NULL;
static int shdisp_hayabusa_vddio_status = false;
#endif /* SHDISP_HAYABUSA_VDD */
static struct clk *shdisp_hayabusa_bbclk2 = NULL;
static struct timespec shdisp_hayabusa_sleepout_start = {0};
static int internal_osc_type = SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_A;

static int shdisp_hayabusa_mfr_param = -1;

/* ------------------------------------------------------------------------- */
/*      packet header                                                        */
/* ------------------------------------------------------------------------- */
/*      LCD ON                                                              */
/*      Initial Setting                                                     */

#if defined(CONFIG_ARCH_LYNX_DL90)
#include "./data/shdisp_hayabusa_data_default.h"
#else  /* CONFIG_ARCH_XXX */
#include "./data/shdisp_hayabusa_data_default.h"
#endif /* CONFIG_ARCH_XXX */

#ifdef SHDISP_POWER_MODE_CHK
static struct shdisp_dsi_cmd_desc mipi_sh_hayabusa_cmds_dispon_check[] = {
    {SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmd_SwitchCommand10, 0, 0}
};
#endif /* SHDISP_POWER_MODE_CHK */

static struct shdisp_panel_operations shdisp_hayabusa_fops = {
    shdisp_hayabusa_API_init_io,
    shdisp_hayabusa_API_exit_io,
    NULL,
    shdisp_hayabusa_API_power_on,
    shdisp_hayabusa_API_power_off,
    shdisp_hayabusa_API_disp_on,
    shdisp_hayabusa_API_disp_off,
    shdisp_hayabusa_API_start_display,
    shdisp_hayabusa_API_post_video_start,
    NULL,
    shdisp_hayabusa_API_diag_write_reg,
    shdisp_hayabusa_API_diag_read_reg,
    shdisp_hayabusa_API_diag_set_flicker_param,
    shdisp_hayabusa_API_diag_get_flicker_param,
    shdisp_hayabusa_API_diag_get_flicker_low_param,
    shdisp_hayabusa_API_check_recovery,
    shdisp_hayabusa_API_diag_set_gmmtable_and_voltage,
    shdisp_hayabusa_API_diag_get_gmmtable_and_voltage,
    shdisp_hayabusa_API_diag_set_gmm,
    shdisp_hayabusa_API_shutdown,
    shdisp_hayabusa_API_dump,
    shdisp_hayabusa_API_set_irq,
    shdisp_hayabusa_API_mfr,
    shdisp_hayabusa_API_chg_mfr_param,
    shdisp_hayabusa_API_set_freq_param,
};

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define MIPI_DSI_COMMAND_TX(x)              (shdisp_panel_API_mipi_dsi_cmds_tx(0, x, ARRAY_SIZE(x)))
#define MIPI_DSI_COMMAND_TX_COMMIT(x)       (shdisp_panel_API_mipi_dsi_cmds_tx(1, x, ARRAY_SIZE(x)))
#define IS_FLICKER_ADJUSTED(param)          (((param & 0xF000) == 0x9000) ? 1 : 0)
#define TRACKING_MAX_VAL                    (-1)
#define TRACKING_MIN_VAL                    (-3)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ANDROID_ENGINEERING)
static int shdisp_hayabusa_dump_reg(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_create                                                */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_hayabusa_API_create(void)
{
    return &shdisp_hayabusa_fops;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_calc_vcom_param                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_calc_vcom_param(struct shdisp_hayabusa_calc_vcom_in in, struct shdisp_hayabusa_calc_vcom_out *out, unsigned short vcomoffset)
{
    unsigned short tmp;
    unsigned short vcomadj;
    signed short vcomadj_tmp;
    unsigned short vcomdcoff;

    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x vcom_low=0x%04x vcom_offset=0x%04x", in.vcom, in.vcom_low, vcomoffset);

    if (out == NULL) {
        SHDISP_ERR("<NULL_POINTER> out.");
        return SHDISP_RESULT_FAILURE;
    }

    vcomadj_tmp = (signed short)in.vcom + vcomoffset;
    if (vcomadj_tmp > VCOM_MAX) {
        vcomadj_tmp = VCOM_MAX;
    }
    if (vcomadj_tmp < VCOM_MIN) {
        vcomadj_tmp = VCOM_MIN;
    }
    vcomadj = (unsigned short)vcomadj_tmp;
    SHDISP_DEBUG("vcomadj=0x%04x", vcomadj);

    out->vcom1_l = vcomadj & 0xFF;
    out->vcom2_l = out->vcom1_l;
    out->vcom12_h = 0x00;
    if ((vcomadj >> 8) & 0x01) {
        out->vcom12_h |= 0x03;
    }

    SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x",
                        out->vcom1_l,
                        out->vcom2_l,
                        out->vcom12_h);

    vcomdcoff = (vcomadj + 1) / 2;

    if (in.vcom_low >= in.vcom) {
        tmp = ((in.vcom_low - in.vcom) & 0x0F);
    } else {
        tmp = (((in.vcom - in.vcom_low - 1) & 0x0F) | 0x10);
    }
    out->lpvcom1 = ((tmp & 0x1F) | 0x60);
    out->lpvcom2 = ((tmp & 0x1F) | 0x40);

    out->vcomoff_l = (unsigned char) (vcomdcoff & 0xFF);

    if (vcomdcoff & 0x100) {
        out->vcomoff_h = (unsigned char) (0xA0);
    } else {
        out->vcomoff_h = (unsigned char) (0x80);
    }

    SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF_L=0x%02x VCOMOFF_H=0x%02x",
                        out->lpvcom1,
                        out->lpvcom2,
                        out->vcomoff_l,
                        out->vcomoff_h);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_init_flicker_param                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_init_flicker_param(unsigned short vcom, unsigned short vcom_low)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret;
    struct shdisp_hayabusa_calc_vcom_in in;
    struct shdisp_hayabusa_calc_vcom_out out;
    unsigned short vcomoffset;

    SHDISP_TRACE("in");
    SHDISP_DEBUG("vcom=0x%04x vcom_low=0x%04x", vcom, vcom_low);

    in.vcom = vcom;
    in.vcom_low = vcom_low;

    if (shdisp_SYS_API_check_diag_mode()) {
        vcomoffset = 0;
    } else {
        vcomoffset = VCOM_OFFSET_UP;
    }

    ret = shdisp_hayabusa_calc_vcom_param(in, &out, vcomoffset);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_calc_vcom_param.");
        return SHDISP_RESULT_FAILURE;
    }

    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM1_L] = out.vcom1_l;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM2_L] = out.vcom2_l;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM12_H] = out.vcom12_h;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_LPVCOM1] = out.lpvcom1;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_LPVCOM2] = out.lpvcom2;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOMOFF_L] = out.vcomoff_l;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOMOFF_H] = out.vcomoff_h;

    SHDISP_TRACE("out");

#endif
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_init_io                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_init_io(struct shdisp_panel_context *panel_ctx)
{
    int funcret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    memcpy(&(shdisp_panel_ctx), panel_ctx, sizeof(struct shdisp_panel_context));

#ifndef SHDISP_NOT_SUPPORT_NO_OS
    if (shdisp_hayabusa_init_flicker_param(shdisp_panel_ctx.vcom, shdisp_panel_ctx.vcom_low)) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_init_flicker_param.");
    }

    if (shdisp_hayabusa_init_phy_gmm(&shdisp_panel_ctx.lcddr_phy_gmm)) {
        SHDISP_DEBUG("<RESULT_FAILURE> shdisp_hayabusa_init_phy_gmm.");
    }

#ifdef SHDISP_POWER_MODE_CHK
    shdisp_dispon_chk = shdisp_panel_ctx.disp_on_status;
#endif /* SHDISP_POWER_MODE_CHK */
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

    shdisp_wq_hayabusa = create_singlethread_workqueue("shdisp_hayabusa_queue");
    if (!shdisp_wq_hayabusa) {
        SHDISP_ERR("failed to create_singlethread_workqueue().");
        funcret = SHDISP_RESULT_FAILURE;
        goto exit_with_error;
    }

    INIT_WORK(&shdisp_wq_hayabusa_wk, shdisp_hayabusa_workqueue_handler);

    wake_lock_init(&shdisp_hayabusa_wakelock, WAKE_LOCK_SUSPEND, "hayabusa_wake_lock");

    shdisp_hayabusa_register_driver();

    if (shdisp_api_get_boot_disp_status() == SHDISP_MAIN_DISP_ON) {
#ifdef SHDISP_HAYABUSA_VDD
        shdisp_hayabusa_vdd_on();
#endif /* SHDISP_HAYABUSA_VDD */
        shdisp_hayabusa_hw_reset(false);
        shdisp_hayabusa_external_clk_ctl(true);
    }
    goto exit;
exit_with_error:
    destroy_workqueue(shdisp_wq_hayabusa);
    shdisp_wq_hayabusa = NULL;

exit:

    SHDISP_TRACE("out");
    return funcret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_exit_io                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_exit_io(void)
{
    SHDISP_TRACE("in");

    if (shdisp_wq_hayabusa) {
        flush_workqueue(shdisp_wq_hayabusa);
        destroy_workqueue(shdisp_wq_hayabusa);
        shdisp_wq_hayabusa = NULL;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_power_on                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_power_on(int mode)
{
    SHDISP_TRACE("in mode=%d", mode);

    switch (mode) {
    case SHDISP_PANEL_POWER_FIRST_ON:
    case SHDISP_PANEL_POWER_RECOVERY_ON:
#ifdef SHDISP_HAYABUSA_VDD
        shdisp_hayabusa_vdd_on();
#else  /* SHDISP_HAYABUSA_VDD */
        shdisp_IO_API_delay_us(10 * 1000);
#endif /* SHDISP_HAYABUSA_VDD */
        break;
    case SHDISP_PANEL_POWER_NORMAL_ON:
    default:
        shdisp_hayabusa_hw_reset(true);
        shdisp_IO_API_delay_us(3 * 1000);
        break;
    }
    shdisp_bdic_API_LCD_power_on();
    shdisp_bdic_API_LCD_m_power_on();

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_power_off                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_power_off(int mode)
{
    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
        SHDISP_TRACE("in RECOVERY_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        SHDISP_TRACE("in SHUTDOWN_OFF: mode=%d", mode);
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        SHDISP_TRACE("in NORMAL_OFF: mode=%d", mode);
        break;
    }

    shdisp_bdic_API_LCD_m_power_off();
    shdisp_bdic_API_LCD_power_off();

    shdisp_hayabusa_external_clk_ctl(false);

    switch (mode) {
    case SHDISP_PANEL_POWER_RECOVERY_OFF:
    case SHDISP_PANEL_POWER_SHUTDOWN_OFF:
        shdisp_hayabusa_power_off_for_shutdown();
        break;
    case SHDISP_PANEL_POWER_NORMAL_OFF:
    default:
        break;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_disp_on                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_disp_on(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_hayabusa_mipi_cmd_lcd_on();

    SHDISP_TRACE("out ret=%d", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_disp_off                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_disp_off(void)
{
    int ret = 0;

    SHDISP_TRACE("in");

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_OFF);
    ret = shdisp_hayabusa_mipi_cmd_lcd_off();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_start_display                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_start_display(void)
{
    int ret = 0;
    SHDISP_TRACE("in");

    ret = shdisp_hayabusa_mipi_cmd_display_on();

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_post_video_start                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_post_video_start(void)
{
    SHDISP_TRACE("in");
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_set_irq                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in (enable=%d)", enable);

    switch (enable) {
    case SHDISP_IRQ_ENABLE:
        ret = devm_request_irq(&pshdisp_hayabusa_irq_port_dev->dev,
                                   shdisp_hayabusa_irq_gpio, shdisp_hayabusa_int_isr,
                                   IRQF_TRIGGER_RISING, "shdisp_hayabusa", NULL);
        if (ret) {
            ret = SHDISP_RESULT_FAILURE;
            SHDISP_ERR("failed to request_irq(). (ret=%d irq=%d)", ret, shdisp_hayabusa_irq_gpio);
        }
        shdisp_hayabusa_check_mipi_err();
        break;
    case SHDISP_IRQ_DISABLE:
        disable_irq(shdisp_hayabusa_irq_gpio);
        free_irq(shdisp_hayabusa_irq_gpio, NULL);
        break;
    default:
        ret = SHDISP_RESULT_FAILURE;
        SHDISP_ERR("invalid argument. (enable=%d)", enable);
    }


    SHDISP_TRACE("out (ret=%d)", ret);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_mfr                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_mfr(int mfr)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int mfrwk;
    int id = -1;

    SHDISP_TRACE("in");

    if (mfr == VAL_MFR_RELEASE) {
        mfrwk = VAL_MFR_DEFAULT;
    } else {
        mfrwk = mfr;
    }

    if (mfrwk <= VAL_MFR_15HZ) {
        id = SHDISP_MFR_FIXED_15HZ_EQ_OVER;
    } else {
        id = SHDISP_MFR_FIXED_15HZ_LESS;
    }
    shdisp_hayabusa_mfr_param = id;

    ret = shdisp_hayabusa_mipi_mfr_setting(id, mfrwk);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipi_mfr_param_setting                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipi_mfr_param_setting(int id, int mfr)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i = 0;
    int index = 0;

    SHDISP_TRACE("in  id=%d, mfr=%d", id, mfr);

    switch (id) {
    case SHDISP_MFR_LOW_CANCEL:
        index = SHDISP_MFR_TBL_LOW_CANCEL;
        break;
    case SHDISP_MFR_BKL_FIXED_AUTOADJUST_UPPER:
        index = SHDISP_MFR_TBL_BKL_FIXED_AUTOADJUST_UPPER;
        break;
    case SHDISP_MFR_BKL_FIXED_MIDDLE_LOWER:
        index = SHDISP_MFR_TBL_BKL_FIXED_MIDDLE_LOWER;
        break;
    case SHDISP_MFR_BKL_AUTOADJUST_LOWER:
        index = SHDISP_MFR_TBL_BKL_AUTOADJUST_LOWER;
        break;
    case SHDISP_MFR_FIXED_15HZ_EQ_OVER:
        index = SHDISP_MFR_TBL_FIX_SETTING;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_NREF][index] = mfr;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_ADDR23][index] = 0x11;
        break;
    case SHDISP_MFR_FIXED_15HZ_LESS:
        index = SHDISP_MFR_TBL_FIX_SETTING;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_NREF][index] = mfr;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_ADDR23][index] = 0x37;
        break;
#ifdef SHDISP_TRV_NM2
    case SHDISP_MFR_TRV:
        index = SHDISP_MFR_TBL_FIX_SETTING;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_NREF][index] = VAL_MFR_120HZ;
        mipi_sh_hayabusa_cmd_MFR_Parameter[POS_MFR_LP3_ADDR23][index] = 0x11;
        break;
#endif /* SHDISP_TRV_NM2 */
    default:
        return ret;
    }

    for (i = 0; i < MFR_PARAM_TBL_ROW; i++) {
        mipi_sh_hayabusa_cmd_MFR_Parameter_temp[i][1] = mipi_sh_hayabusa_cmd_MFR_Parameter[i][index];
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_get_mfr_brightness_id                                     */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_get_mfr_brightness_id(struct shdisp_panel_mode p_mode)
{
    int id  = -1;

    SHDISP_TRACE("in");

    switch (p_mode.bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        if ((p_mode.bkl_param >= 1) && (p_mode.bkl_param < LOW_BKL_FIXED)) {
            id = SHDISP_MFR_LOW_CANCEL;
        } else if ((p_mode.bkl_param >= LOW_BKL_FIXED) && (p_mode.bkl_param < UPPER_BKL_FIXED)) {
            id = SHDISP_MFR_BKL_FIXED_MIDDLE_LOWER;
        } else if ((p_mode.bkl_param >= UPPER_BKL_FIXED) && (p_mode.bkl_param <= 255)) {
            id = SHDISP_MFR_BKL_FIXED_AUTOADJUST_UPPER;
        }
        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:
        if ((p_mode.bkl_param >= 1) && (p_mode.bkl_param < UPPER_BKL_AUTO)) {
            id = SHDISP_MFR_BKL_AUTOADJUST_LOWER;
        } else if ((p_mode.bkl_param >= UPPER_BKL_AUTO) && (p_mode.bkl_param <= 255)) {
            id = SHDISP_MFR_BKL_FIXED_AUTOADJUST_UPPER;
        }
        break;
    default:
        break;
    }

    SHDISP_TRACE("out");
    return id;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipi_mfr_setting                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipi_mfr_setting(int id, int mfr)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in  id=%d, mfr=%d", id, mfr);

    ret = shdisp_hayabusa_mipi_mfr_param_setting(id, mfr);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_SwitchCommand26);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2 ret=%d", ret);
        return ret;
    }
    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_MFR_Setting);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", ret);
    }

    SHDISP_TRACE("out");
    return ret;
}
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_chg_mfr_param                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_chg_mfr_param(struct shdisp_panel_mode p_mode)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int id  = -1;

    SHDISP_TRACE("in");

    if (!shdisp_SYS_API_check_diag_mode()) {
#ifdef SHDISP_TRV_NM2
        if (p_mode.trv_mode == SHDISP_TRV_PARAM_ON) {
            id = SHDISP_MFR_TRV;
        } else {
            id = shdisp_hayabusa_get_mfr_brightness_id(p_mode);
        }
#else  /* SHDISP_TRV_NM2 */
        id = shdisp_hayabusa_get_mfr_brightness_id(p_mode);
#endif /* SHDISP_TRV_NM2 */
    }
    if ((id != -1) && (id != shdisp_hayabusa_mfr_param)) {
        shdisp_hayabusa_mfr_param = id;

        ret = shdisp_hayabusa_mipi_mfr_setting(id, 0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out ret=%d", ret);
            return ret;
        }
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_chg_mfr_param                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_chg_mfr_param(struct shdisp_panel_mode p_mode)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in  bkl_mode=%d bkl_param=%d", p_mode.bkl_mode, p_mode.bkl_param);

    ret = shdisp_hayabusa_chg_mfr_param(p_mode);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_osc_upd_on_cmd                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_osc_upd_on_cmd(char param[][2])
{
    mipi_sh_hayabusa_cmd_OSCtrimONF0[1][1] = param[1][1];
    mipi_sh_hayabusa_cmd_OSCtrimONF0[2][1] = param[2][1];
    mipi_sh_hayabusa_cmd_TG_OPT_CTRL[2][1] = param[4][1];
    mipi_sh_hayabusa_cmd_STV26[0][1] = param[5][1];
    mipi_sh_hayabusa_cmd_STV26[1][1] = param[6][1];
    mipi_sh_hayabusa_cmd_GCK26[0][1] = param[7][1];
    mipi_sh_hayabusa_cmd_GCK26[1][1] = param[8][1];
    mipi_sh_hayabusa_cmd_PowerOnSequence26[1][1] = param[9][1];
    mipi_sh_hayabusa_cmd_SourceEQ[3][1] = param[10][1];
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_osc_update                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_osc_update(int internal_osc)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in internal_osc=%d", internal_osc);

    if (internal_osc != SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_A &&
            internal_osc != SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_B) {
        SHDISP_DEBUG("out1 internal_osc is not TYPE_A or TYPE_B");
        return ret;
    }

    if (internal_osc == internal_osc_type) {
        SHDISP_DEBUG("out2 internal_osc is same");
        return ret;
    }

    switch (internal_osc) {
    case SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_A:
        shdisp_hayabusa_osc_upd_on_cmd(mipi_sh_hayabusa_cmd_OSC_ParamTypeA);
        break;
    case SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_B:
        shdisp_hayabusa_osc_upd_on_cmd(mipi_sh_hayabusa_cmd_OSC_ParamTypeB);
        break;
    }

    if (shdisp_api_get_main_disp_status() == SHDISP_MAIN_DISP_ON) {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_OSC_ParamCmmFixMFR);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("Fix MFR COMMAND err ret=%d", ret);
        }
        shdisp_IO_API_delay_us(40 * 1000);
        switch (internal_osc) {
        case SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_A:
            ret |= MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_OSC_ParamTypeA);
            break;
        case SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_B:
            ret |= MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_OSC_ParamTypeB);
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("osc Parameter COMMAND err ret=%d", ret);
        }
        shdisp_IO_API_delay_us(10 * 1000);
        mipi_sh_hayabusa_cmd_OSC_ParamCmmResumeMFR[0][1]
                    = mipi_sh_hayabusa_cmd_MFR_Parameter_temp[POS_MFR_LP3_NREF][1];
        mipi_sh_hayabusa_cmd_OSC_ParamCmmResumeMFR[1][1]
                    = mipi_sh_hayabusa_cmd_MFR_Parameter_temp[POS_MFR_LPMC_HIST][1];
        ret |= MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_OSC_ParamCmmResumeMFR);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("Resume MFR COMMAND err ret=%d", ret);
        }
    }

    internal_osc_type = internal_osc;

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_set_freq_param                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_set_freq_param(struct shdisp_freq_params *freq)
{
    int ret = SHDISP_RESULT_SUCCESS;
    SHDISP_TRACE("in");

    if (freq == NULL) {
        SHDISP_ERR("out1 freq is NULL");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_hayabusa_osc_update(freq->internal_osc);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2 ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_start_video                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_start_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x10, 1, 0 },
        { 0x29, 0x00, 0, 0 },
     };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    for (i = 0; i < 2; i++) {
        ret = shdisp_hayabusa_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_IO_API_delay_us(reg_data[i].wait);
        }
    }
    enable_irq(shdisp_hayabusa_irq_gpio);
    shdisp_bdic_API_IRQ_det_irq_ctrl(true);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_stop_video                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_stop_video(void)
{
    struct {
        unsigned char addr;
        unsigned char data;
        unsigned char size;
        unsigned long wait;
    } reg_data[] = {
        { 0xFF, 0x10, 1, 0 },
        { 0x28, 0x00, 0, WAIT_1FRAME_US * 1 },
    };
    int i;
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_bdic_API_IRQ_det_irq_ctrl(false);
    disable_irq(shdisp_hayabusa_irq_gpio);
    for (i = 0; i < 2; i++) {
        ret = shdisp_hayabusa_API_diag_write_reg(reg_data[i].addr, &reg_data[i].data, reg_data[i].size);
        if (reg_data[i].wait) {
            shdisp_IO_API_delay_us(reg_data[i].wait);
        }
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_write_reg                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_write_reg(unsigned char addr, unsigned char *write_data, unsigned char size)
{
    int ret = 0;
    char dtype;

    SHDISP_TRACE("in");

    if (size == 0) {
        dtype = SHDISP_DTYPE_DCS_WRITE;
    } else if (size == 1) {
        dtype = SHDISP_DTYPE_DCS_WRITE1;
    } else {
        dtype = SHDISP_DTYPE_DCS_LWRITE;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(dtype, addr, write_data, size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_DEBUG("out dokick err ret=%d", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_read_reg                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_read_reg(unsigned char addr, unsigned char *read_data, unsigned char size)
{
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, addr, read_data, size);
    if (ret) {
        SHDISP_DEBUG("out1");
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_set_flicker_param                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_set_flicker_param(struct shdisp_diag_flicker_param flicker_param)
{
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int ret = 0;

    SHDISP_TRACE("in");

    ret = shdisp_hayabusa_diag_set_flicker_param_internal(flicker_param);

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_diag_set_flicker_param_internal.");
        return SHDISP_RESULT_FAILURE;
    } else {
        shdisp_hayabusa_diag_set_flicker_param_ctx(flicker_param);
    }

    SHDISP_TRACE("out");
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return SHDISP_RESULT_SUCCESS;
}

#ifndef SHDISP_NOT_SUPPORT_FLICKER
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_diag_set_flicker_param_internal                           */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_diag_set_flicker_param_internal(struct shdisp_diag_flicker_param flicker_param)
{
    int vcom = flicker_param.master_alpha;
    int vcom_low = flicker_param.master_alpha;
    int i;
    int ret = 0;
    unsigned char hayabusa_rdata_tmp[8];
    struct shdisp_hayabusa_calc_vcom_in in;
    struct shdisp_hayabusa_calc_vcom_out out;

    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_REG_WRITE) {

        for (i = 0; i < SHDISP_HAYABUSA_VCOM_REG_NUM; i++) {
            hayabusa_rdata_tmp[i] = 0;
            hayabusa_wdata[i] = 0;
        }

        shdisp_hayabusa_set_switchcommand(0x20);

        in.vcom = vcom;
        in.vcom_low = vcom_low;

        ret = shdisp_hayabusa_calc_vcom_param(in, &out, 0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_calc_vcom_param.");
            return SHDISP_RESULT_FAILURE;
        }

        hayabusa_rdata_tmp[0] = out.vcom1_l;
        hayabusa_rdata_tmp[1] = out.vcom2_l;
        hayabusa_rdata_tmp[2] = out.vcom12_h;
        hayabusa_rdata_tmp[3] = out.lpvcom1;
        hayabusa_rdata_tmp[4] = out.lpvcom2;
        hayabusa_rdata_tmp[5] = out.vcomoff_l;
        hayabusa_rdata_tmp[6] = out.vcomoff_h;

        for (i = 0; i < SHDISP_HAYABUSA_VCOM_REG_NUM; i++) {
            hayabusa_wdata[0] = hayabusa_rdata_tmp[i];
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, Hayabusa_VCOM_Reg[i], &hayabusa_wdata[0], 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg!!");
                break;
            }
        }

        SHDISP_DEBUG("VCOM1_L=0x%02x VCOM2_L=0x%02x VCOM12_H=0x%02x",
                              hayabusa_rdata_tmp[0], hayabusa_rdata_tmp[1], hayabusa_rdata_tmp[2]);
        SHDISP_DEBUG("LPVCOM1=0x%02x LPVCOM2=0x%02x VCOMOFF_L=0x%02x VCOMOFF_H=0x%02x",
                              hayabusa_rdata_tmp[3], hayabusa_rdata_tmp[4], hayabusa_rdata_tmp[5], hayabusa_rdata_tmp[6]);
        SHDISP_DEBUG("vcom=0x%04x", vcom);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("out dokick err ret=%d", ret);
            return ret;
        }
    }

    if (flicker_param.request & (SHDISP_SAVE_VALUE | SHDISP_SAVE_VALUE_LOW)) {
        if (!(flicker_param.request & SHDISP_SAVE_VALUE)) {
            vcom = shdisp_panel_ctx.vcom;
        }
        if (!(flicker_param.request & SHDISP_SAVE_VALUE_LOW)) {
            vcom_low = shdisp_panel_ctx.vcom_low;
        }

        if (shdisp_hayabusa_init_flicker_param(vcom, vcom_low)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_init_flicker_param.");
        }
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        if (shdisp_hayabusa_init_flicker_param(flicker_param.master_alpha, flicker_param.master_alpha)) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_init_flicker_param.");
        }
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_diag_set_flicker_param_ctx                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_diag_set_flicker_param_ctx(struct shdisp_diag_flicker_param flicker_param)
{
    SHDISP_TRACE("in");

    if (flicker_param.request & SHDISP_SAVE_VALUE) {
        shdisp_panel_ctx.vcom = flicker_param.master_alpha;
        shdisp_panel_ctx.vcom_nvram = 0x9000 | flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_SAVE_VALUE_LOW) {
        shdisp_panel_ctx.vcom_low = flicker_param.master_alpha;
    }
    if (flicker_param.request & SHDISP_RESET_VALUE) {
        shdisp_panel_ctx.vcom = 0;
        shdisp_panel_ctx.vcom_low = 0;
        shdisp_panel_ctx.vcom_nvram = 0;
    }

    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_NOT_SUPPORT_FLICKER */

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_get_flicker_param                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_get_flicker_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char hayabusa_rdata_tmp[8];

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_HAYABUSA_VCOM_REG_NUM; i++) {
        hayabusa_rdata[i] = 0;
        hayabusa_rdata_tmp[i] = 0;
    }

    shdisp_hayabusa_set_switchcommand(0x20);

    for (i = 0; i < 3; i++) {
        if (i != 1) {
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, Hayabusa_VCOM_Reg[i], hayabusa_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x", Hayabusa_VCOM_Reg[i],
                                                                                                  hayabusa_rdata[0]);
            }
            hayabusa_rdata_tmp[i] = hayabusa_rdata[0];
        }
    }

    flicker_param->master_alpha = ((hayabusa_rdata_tmp[2] & 0x01) << 8) | hayabusa_rdata_tmp[0];

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_get_flicker_low_param                            */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_get_flicker_low_param(struct shdisp_diag_flicker_param *flicker_param)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifndef SHDISP_NOT_SUPPORT_FLICKER
    int i;
    unsigned char hayabusa_rdata_tmp[8];
    unsigned short tmp_vcom;

    SHDISP_TRACE("in");

    if (flicker_param == NULL) {
        SHDISP_ERR("<NULL_POINTER> flicker_param.");
        return SHDISP_RESULT_FAILURE;
    }

    for (i = 0; i < SHDISP_HAYABUSA_VCOM_REG_NUM; i++) {
        hayabusa_rdata[i] = 0;
        hayabusa_rdata_tmp[i] = 0;
    }

    shdisp_hayabusa_set_switchcommand(0x20);

    for (i = 0; i < 4; i++) {
        if (i != 1) {
            ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ, Hayabusa_VCOM_Reg[i], hayabusa_rdata, 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg addr=0x%02x data=0x%02x", Hayabusa_VCOM_Reg[i],
                                                                                                  hayabusa_rdata[0]);
            }
            hayabusa_rdata_tmp[i] = hayabusa_rdata[0];
        }
    }

    tmp_vcom = ((hayabusa_rdata_tmp[2] & 0x01) << 8) | hayabusa_rdata_tmp[0];
    if (hayabusa_rdata_tmp[3] & 0x10) {
        flicker_param->master_alpha = tmp_vcom - (hayabusa_rdata_tmp[3] & 0x0f) - 1;
    } else {
        flicker_param->master_alpha = tmp_vcom + (hayabusa_rdata_tmp[3] & 0x0f);
    }

    SHDISP_TRACE("out master_alpha=0x%04X", flicker_param->master_alpha);
#endif /* SHDISP_NOT_SUPPORT_FLICKER */
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_check_recovery                                        */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_check_recovery(void)
{
    int ret1 = SHDISP_RESULT_SUCCESS, ret2;
    int ret_devcode = SHDISP_RESULT_SUCCESS;
    int version = 0;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif  /* SHDISP_RESET_LOG */


    SHDISP_TRACE("in");

#ifdef SHDISP_POWER_MODE_CHK
    SHDISP_DEBUG("shdisp_dispon_chk=%d", shdisp_dispon_chk);

    switch (shdisp_dispon_chk) {
    case SHDISP_PANEL_DISPON_CHK_INIT:
        ret1 = shdisp_hayabusa_power_mode_chk(0x0A);
        if (ret1 != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_power_mode_chk.");
        }
        break;
    case SHDISP_PANEL_DISPON_CHK_NG:
        SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_power_mode_chk.");
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DISPON_CHK);
        ret1 = SHDISP_RESULT_FAILURE;
    default:
        break;
    }
#endif /* SHDISP_POWER_MODE_CHK */

    ret_devcode = shdisp_hayabusa_devicecode_read(&version);
    if (ret_devcode == SHDISP_RESULT_SUCCESS && version == VER_CUT1_0) {
        SHDISP_DEBUG("through if cut1.0 panel.");
        return SHDISP_RESULT_SUCCESS;
    }

    ret2 = shdisp_bdic_API_RECOVERY_check_restoration();

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DETLOW) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        SHDISP_DEBUG("force lcd det low.");
        ret2 = SHDISP_RESULT_FAILURE;
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret2 != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_bdic_API_RECOVERY_check_restoration.");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DET_LOW;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_DET_LOW);
#endif /* SHDISP_RESET_LOG */
    }

    if ((ret1 != SHDISP_RESULT_SUCCESS) || (ret2 != SHDISP_RESULT_SUCCESS)) {
        return SHDISP_RESULT_FAILURE;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_diag_set_gmmtable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info,
                                                       int set_applied_voltage)
{
    int i, j = 0;
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char hayabusa_gmm_wdata[401];
    unsigned char hayabusa_gmm_addr[401] = {
        mipi_sh_hayabusa_cmd_SwitchCommand20[0],
        mipi_sh_hayabusa_cmd_RDPSGMM[0][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[1][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[2][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[3][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[4][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[5][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[6][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[7][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[8][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[9][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[10][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[11][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[12][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[13][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[14][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[15][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[16][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[17][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[18][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[19][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[20][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[21][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[22][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[23][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[24][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[25][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[26][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[27][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[28][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[29][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[30][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[31][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[32][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[33][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[34][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[35][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[36][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[37][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[38][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[39][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[40][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[41][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[42][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[43][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[44][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[45][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[46][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[47][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[48][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[49][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[50][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[51][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[52][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[53][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[54][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[55][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[56][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[57][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[58][0],
        mipi_sh_hayabusa_cmd_RDPSGMM[59][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[0][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[1][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[2][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[3][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[4][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[5][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[6][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[7][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[8][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[9][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[10][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[11][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[12][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[13][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[14][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[15][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[16][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[17][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[18][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[19][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[20][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[21][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[22][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[23][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[24][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[25][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[26][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[27][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[28][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[29][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[30][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[31][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[32][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[33][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[34][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[35][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[36][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[37][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[38][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[39][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[40][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[41][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[42][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[43][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[44][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[45][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[46][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[47][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[48][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[49][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[50][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[51][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[52][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[53][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[54][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[55][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[56][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[57][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[58][0],
        mipi_sh_hayabusa_cmd_RDNGGMM[59][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[0][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[1][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[2][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[3][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[4][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[5][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[6][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[7][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[8][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[9][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[10][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[11][0],
        mipi_sh_hayabusa_cmd_SwitchCommand21[0],
        mipi_sh_hayabusa_cmd_GRPSGMM[12][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[13][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[14][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[15][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[16][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[17][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[18][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[19][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[20][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[21][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[22][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[23][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[24][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[25][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[26][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[27][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[28][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[29][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[30][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[31][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[32][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[33][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[34][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[35][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[36][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[37][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[38][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[39][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[40][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[41][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[42][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[43][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[44][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[45][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[46][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[47][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[48][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[49][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[50][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[51][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[52][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[53][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[54][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[55][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[56][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[57][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[58][0],
        mipi_sh_hayabusa_cmd_GRPSGMM[59][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[0][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[1][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[2][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[3][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[4][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[5][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[6][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[7][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[8][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[9][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[10][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[11][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[12][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[13][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[14][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[15][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[16][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[17][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[18][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[19][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[20][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[21][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[22][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[23][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[24][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[25][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[26][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[27][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[28][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[29][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[30][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[31][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[32][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[33][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[34][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[35][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[36][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[37][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[38][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[39][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[40][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[41][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[42][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[43][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[44][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[45][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[46][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[47][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[48][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[49][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[50][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[51][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[52][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[53][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[54][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[55][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[56][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[57][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[58][0],
        mipi_sh_hayabusa_cmd_GRNGGMM[59][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[0][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[1][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[2][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[3][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[4][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[5][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[6][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[7][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[8][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[9][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[10][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[11][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[12][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[13][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[14][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[15][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[16][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[17][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[18][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[19][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[20][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[21][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[22][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[23][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[24][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[25][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[26][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[27][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[28][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[29][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[30][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[31][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[32][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[33][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[34][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[35][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[36][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[37][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[38][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[39][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[40][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[41][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[42][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[43][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[44][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[45][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[46][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[47][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[48][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[49][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[50][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[51][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[52][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[53][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[54][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[55][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[56][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[57][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[58][0],
        mipi_sh_hayabusa_cmd_BLPSGMM[59][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[0][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[1][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[2][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[3][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[4][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[5][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[6][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[7][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[8][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[9][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[10][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[11][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[12][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[13][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[14][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[15][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[16][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[17][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[18][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[19][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[20][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[21][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[22][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[23][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[24][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[25][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[26][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[27][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[28][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[29][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[30][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[31][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[32][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[33][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[34][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[35][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[36][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[37][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[38][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[39][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[40][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[41][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[42][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[43][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[44][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[45][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[46][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[47][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[48][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[49][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[50][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[51][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[52][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[53][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[54][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[55][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[56][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[57][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[58][0],
        mipi_sh_hayabusa_cmd_BLNGGMM[59][0],
        mipi_sh_hayabusa_cmd_SwitchCommand20[0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGH][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGL][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDN][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP2][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGHO][0],
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGLO][0],
        mipi_sh_hayabusa_cmd_SwitchCommand22[0],
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[4][0],
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[4][0],
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[4][0],
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[4][0],
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[4][0],
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[0][0],
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[1][0],
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[2][0],
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[3][0],
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[4][0],
    };

    SHDISP_TRACE("in");

    for (i = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        if (i == 0) {
            hayabusa_gmm_wdata[j++] = mipi_sh_hayabusa_cmd_SwitchCommand20[1];
        }
        hayabusa_gmm_wdata[j++] = ((gmm_info->gammaR[i] >> 8) & 0x0003);
        hayabusa_gmm_wdata[j++] = (gmm_info->gammaR[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        if (i == 6) {
            hayabusa_gmm_wdata[j++] = mipi_sh_hayabusa_cmd_SwitchCommand21[1];
        }
        hayabusa_gmm_wdata[j++] = ((gmm_info->gammaG[i] >> 8) & 0x0003);
        hayabusa_gmm_wdata[j++] = (gmm_info->gammaG[i] & 0x00FF);
    }

    for (i = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        hayabusa_gmm_wdata[j++] = ((gmm_info->gammaB[i] >> 8) & 0x0003);
        hayabusa_gmm_wdata[j++] = (gmm_info->gammaB[i] & 0x00FF);
    }

    if (!set_applied_voltage) {
        for (i = 0; i < j; i++) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, hayabusa_gmm_addr[i], &hayabusa_gmm_wdata[i], 1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!");
                goto shdisp_end;
            }
        }
        goto shdisp_end;
    }

    hayabusa_gmm_wdata[j++] = mipi_sh_hayabusa_cmd_SwitchCommand20[1];
    hayabusa_gmm_wdata[j++] = gmm_info->vgh;
    hayabusa_gmm_wdata[j++] = gmm_info->vgl;
    hayabusa_gmm_wdata[j++] = gmm_info->gvddp;
    hayabusa_gmm_wdata[j++] = gmm_info->gvddn;
    hayabusa_gmm_wdata[j++] = gmm_info->gvddp2;
    hayabusa_gmm_wdata[j++] = gmm_info->vgho;
    hayabusa_gmm_wdata[j++] = gmm_info->vglo;

    hayabusa_gmm_wdata[j++] = mipi_sh_hayabusa_cmd_SwitchCommand22[1];
    memcpy(&hayabusa_gmm_wdata[j], gmm_info->adv_gamma, SHDISP_LCDDR_ADVANCED_GAMMA_SIZE);
    j += SHDISP_LCDDR_ADVANCED_GAMMA_SIZE;

    for (i = 0; i < j; i++) {
        ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1, hayabusa_gmm_addr[i], &hayabusa_gmm_wdata[i], 1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg!!" );
            goto shdisp_end;
        }
    }

shdisp_end:
    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                               mipi_sh_hayabusa_cmd_SwitchCommand10[0],
                                               &mipi_sh_hayabusa_cmd_SwitchCommand10[1],
                                               1);

    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_diag_get_gmmtable_and_voltage                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info,
                                                       int set_applied_voltage)
{
    int i, j;
    int ret = 0;
    unsigned char hayabusa_rdata[1];
    unsigned short hayabusa_temp_data[SHDISP_HAYABUSA_GMM_SETTING_SIZE];

    SHDISP_TRACE("in");

    if (gmm_info == NULL) {
        SHDISP_ERR("<NULL_POINTER> gmm_info.");
        return SHDISP_RESULT_FAILURE;
    }

    memset(hayabusa_temp_data, 0, sizeof(hayabusa_temp_data));
    for (i = 0, j = 0; i < (SHDISP_HAYABUSA_GMM_SETTING_SIZE / 2); i++) {
        if (i == 0) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                       mipi_sh_hayabusa_cmd_SwitchCommand20[0],
                                                       &mipi_sh_hayabusa_cmd_SwitchCommand20[1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg.");
                goto shdisp_end;
            }
        }
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_RDPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_RDPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_RDNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_RDNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaR, hayabusa_temp_data, sizeof(hayabusa_temp_data));

    memset(hayabusa_temp_data, 0, sizeof(hayabusa_temp_data));
    for (i = 0, j = 0; i < (SHDISP_HAYABUSA_GMM_SETTING_SIZE / 2); i++) {
        if (i == 6) {
            ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                                       mipi_sh_hayabusa_cmd_SwitchCommand21[0],
                                                       &mipi_sh_hayabusa_cmd_SwitchCommand21[1],
                                                       1);
            if (ret) {
                SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
                goto shdisp_end;
            }
        }
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_GRPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_GRPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_GRNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_GRNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaG, hayabusa_temp_data, sizeof(hayabusa_temp_data));

    memset(hayabusa_temp_data, 0, sizeof(hayabusa_temp_data));
    for (i = 0, j = 0; i < (SHDISP_HAYABUSA_GMM_SETTING_SIZE / 2); i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_BLPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_BLPSGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }

    for (i = SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET, j = 0; i < SHDISP_HAYABUSA_GMM_SETTING_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_BLNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] = ((hayabusa_rdata[0] << 8) & 0x0300);
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_BLNGGMM[j++][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        hayabusa_temp_data[i] |= (hayabusa_rdata[0] & 0x00FF);
    }
    memcpy(gmm_info->gammaB, hayabusa_temp_data, sizeof(hayabusa_temp_data));

    if (!set_applied_voltage) {
        goto shdisp_end;
    }

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                               mipi_sh_hayabusa_cmd_SwitchCommand20[0],
                                               &mipi_sh_hayabusa_cmd_SwitchCommand20[1],
                                               1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_write_reg.");
        goto shdisp_end;
    }

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGH][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgh = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGL][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgl = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddp = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDN][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddn = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP2][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->gvddp2 = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGHO][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vgho = hayabusa_rdata[0];

    memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
    ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                              mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGLO][0],
                                              hayabusa_rdata,
                                              1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> mipi_sharp_diag_read_reg.");
        goto shdisp_end;
    }
    gmm_info->vglo = hayabusa_rdata[0];

    ret = shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                               mipi_sh_hayabusa_cmd_SwitchCommand22[0],
                                               &mipi_sh_hayabusa_cmd_SwitchCommand22[1],
                                               1);
    if (ret) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_write_reg.");
        goto shdisp_end;
    }
    j = 0;
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }
    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        memset(hayabusa_rdata, 0, sizeof(hayabusa_rdata));
        ret = shdisp_panel_API_mipi_diag_read_reg(SHDISP_DTYPE_DCS_READ,
                                                  mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[i][0],
                                                  hayabusa_rdata,
                                                  1);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_panel_API_mipi_diag_read_reg.");
            goto shdisp_end;
        }
        gmm_info->adv_gamma[j++] = hayabusa_rdata[0];
    }

shdisp_end:
    shdisp_panel_API_mipi_diag_write_reg(SHDISP_DTYPE_DCS_WRITE1,
                                         mipi_sh_hayabusa_cmd_SwitchCommand10[0],
                                         &mipi_sh_hayabusa_cmd_SwitchCommand10[1],
                                         1);

    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_set_gmmtable_and_voltage                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_set_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = 0;
    int pcnt, ncnt, i;

    SHDISP_TRACE("in");

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_hayabusa_diag_set_gmmtable_and_voltage(gmm_info, 1);
    mdss_shdisp_dsi_bus_clk_ctrl(false);
    if (ret) {
        return ret;
    }

    shdisp_panel_ctx.lcddr_phy_gmm.status = SHDISP_LCDDR_GMM_STATUS_OK;
    for (pcnt = 0; pcnt < SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET; pcnt++) {
        ncnt = pcnt + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET;
        i = pcnt * 2;
        mipi_sh_hayabusa_cmd_RDPSGMM_adjusted[i][1]      = ((gmm_info->gammaR[pcnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_RDPSGMM_adjusted[i + 1][1]  = ( gmm_info->gammaR[pcnt] & 0x00FF);
        mipi_sh_hayabusa_cmd_RDNGGMM_adjusted[i][1]      = ((gmm_info->gammaR[ncnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_RDNGGMM_adjusted[i + 1][1]  = ( gmm_info->gammaR[ncnt] & 0x00FF);
        mipi_sh_hayabusa_cmd_GRPSGMM_adjusted[i][1]      = ((gmm_info->gammaG[pcnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_GRPSGMM_adjusted[i + 1][1]  = ( gmm_info->gammaG[pcnt] & 0x00FF);
        mipi_sh_hayabusa_cmd_GRNGGMM_adjusted[i][1]      = ((gmm_info->gammaG[ncnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_GRNGGMM_adjusted[i + 1][1]  = ( gmm_info->gammaG[ncnt] & 0x00FF);
        mipi_sh_hayabusa_cmd_BLPSGMM_adjusted[i][1]      = ((gmm_info->gammaB[pcnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_BLPSGMM_adjusted[i + 1][1]  = ( gmm_info->gammaB[pcnt] & 0x00FF);
        mipi_sh_hayabusa_cmd_BLNGGMM_adjusted[i][1]      = ((gmm_info->gammaB[ncnt] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_BLNGGMM_adjusted[i + 1][1]  = ( gmm_info->gammaB[ncnt] & 0x00FF);
    }

    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGH]    = gmm_info->vgh;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGL]    = gmm_info->vgl;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP]  = gmm_info->gvddp;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDN]  = gmm_info->gvddn;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP2] = gmm_info->gvddp2;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGHO]   = gmm_info->vgho;
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGLO]   = gmm_info->vglo;

    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 0];
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 1];
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 2];
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 3];
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 4];
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM_adjusted[i][1] =
            gmm_info->adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 5];
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_get_gmmtable_and_voltage                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_get_gmmtable_and_voltage(struct shdisp_diag_gamma_info *gmm_info)
{
    int ret = 0;

    SHDISP_TRACE("in");

    shdisp_hayabusa_stop_video();
    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_hayabusa_diag_get_gmmtable_and_voltage(gmm_info, 1);
    mdss_shdisp_dsi_bus_clk_ctrl(false);
    shdisp_hayabusa_start_video();
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_diag_set_gmm                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_diag_set_gmm(struct shdisp_diag_gamma *gmm)
{
    int ret = 0;
    int cut_version = 0;
    int i,dst_i;
    struct shdisp_diag_gamma_info gmm_info_temp;

    SHDISP_TRACE("in");

    if (gmm == NULL) {
        SHDISP_ERR("<NULL_POINTER> gmm.");
        return SHDISP_RESULT_FAILURE;
    }

    if (gmm->level != SHDISP_LCDDR_GMM_SET_POINT_INI) {
        SHDISP_ERR("<INVALID_VALUE> gmm->level(%d).", gmm->level);
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_panel_ctx.lcddr_phy_gmm.status = SHDISP_LCDDR_GMM_STATUS_NOT_SET;

    ret = shdisp_hayabusa_devicecode_read(&cut_version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }

    for (i = 0; i < SHDISP_PANEL_GMM_TBL_SIZE; i++) {
        dst_i = i >> 1;
        if ((i % 2) == 0) {
            gmm_info_temp.gammaR[dst_i]   = mipi_sh_hayabusa_cmd_RDPSGMM[i][1] << 8;
            gmm_info_temp.gammaR[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                            = mipi_sh_hayabusa_cmd_RDNGGMM[i][1] << 8;
            gmm_info_temp.gammaG[dst_i]   = mipi_sh_hayabusa_cmd_GRPSGMM[i][1] << 8;
            gmm_info_temp.gammaG[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                            = mipi_sh_hayabusa_cmd_GRNGGMM[i][1] << 8;
            gmm_info_temp.gammaB[dst_i]   = mipi_sh_hayabusa_cmd_BLPSGMM[i][1] << 8;
            gmm_info_temp.gammaB[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                            = mipi_sh_hayabusa_cmd_BLNGGMM[i][1] << 8;
        } else {
            gmm_info_temp.gammaR[dst_i]  |= mipi_sh_hayabusa_cmd_RDPSGMM[i][1];
            gmm_info_temp.gammaR[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                           |= mipi_sh_hayabusa_cmd_RDNGGMM[i][1];
            gmm_info_temp.gammaG[dst_i]  |= mipi_sh_hayabusa_cmd_GRPSGMM[i][1];
            gmm_info_temp.gammaG[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                           |= mipi_sh_hayabusa_cmd_GRNGGMM[i][1];
            gmm_info_temp.gammaB[dst_i]  |= mipi_sh_hayabusa_cmd_BLPSGMM[i][1];
            gmm_info_temp.gammaB[dst_i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET]
                                           |= mipi_sh_hayabusa_cmd_BLNGGMM[i][1];
        }
    }

    switch (cut_version) {
    case VER_CUT1_1:
        gmm_info_temp.vgh         = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_VGH];
        gmm_info_temp.vgl         = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_VGL];
        gmm_info_temp.gvddp       = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_GVDDP];
        gmm_info_temp.gvddn       = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_GVDDN];
        gmm_info_temp.gvddp2      = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_GVDDP2];
        gmm_info_temp.vgho        = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_VGHO];
        gmm_info_temp.vglo        = mipi_sh_hayabusa_cmd_PowerControl_cut1_1[SHDISP_HAYABUSA_VGLO];
        break;
    case VER_CUT2_0:
        gmm_info_temp.vgh         = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGH];
        gmm_info_temp.vgl         = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGL];
        gmm_info_temp.gvddp       = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDP];
        gmm_info_temp.gvddn       = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDN];
        gmm_info_temp.gvddp2      = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDP2];
        gmm_info_temp.vgho        = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGHO];
        gmm_info_temp.vglo        = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGLO];
        break;
    default:
        gmm_info_temp.vgh         = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGH];
        gmm_info_temp.vgl         = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGL];
        gmm_info_temp.gvddp       = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDP];
        gmm_info_temp.gvddn       = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDN];
        gmm_info_temp.gvddp2      = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_GVDDP2];
        gmm_info_temp.vgho        = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGHO];
        gmm_info_temp.vglo        = mipi_sh_hayabusa_cmd_PowerControl_cut2_0[SHDISP_HAYABUSA_VGLO];
        SHDISP_ERR("Not Support Panel Versino=%d", cut_version);
        break;
    }

    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 0]
                                            = mipi_sh_hayabusa_cmd_AdvancedRDPSGMM[i][1];
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 1]
                                            = mipi_sh_hayabusa_cmd_AdvancedRDNGGMM[i][1];
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 2]
                                            = mipi_sh_hayabusa_cmd_AdvancedGRPSGMM[i][1];
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 3]
                                            = mipi_sh_hayabusa_cmd_AdvancedGRNGGMM[i][1];
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 4]
                                            = mipi_sh_hayabusa_cmd_AdvancedBLPSGMM[i][1];
        gmm_info_temp.adv_gamma[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 5]
                                            = mipi_sh_hayabusa_cmd_AdvancedBLNGGMM[i][1];
    }

    mdss_shdisp_dsi_bus_clk_ctrl(true);
    ret = shdisp_hayabusa_diag_set_gmmtable_and_voltage(&gmm_info_temp, 1);
    mdss_shdisp_dsi_bus_clk_ctrl(false);
    if (ret) {
        return ret;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_shutdown                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_API_shutdown(void)
{
    shdisp_hayabusa_power_off_for_shutdown();
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_API_dump                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_API_dump(int type)
{
#if defined(CONFIG_ANDROID_ENGINEERING)
    shdisp_hayabusa_dump_reg();
#endif /* CONFIG_ANDROID_ENGINEERING */
}

#ifdef SHDISP_HAYABUSA_VDD
/* ------------------------------------------------------------------------- */
/*      shdisp_hayabusa_vddio_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_vddio_on(void)
{
    int res;

    if (!shdisp_hayabusa_vddio) {
        SHDISP_ERR("shdisp_hayabusa_vddio is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_hayabusa_vddio_status) {
        SHDISP_WARN("supply is already on.");
        return SHDISP_RESULT_SUCCESS;
    }
    shdisp_hayabusa_vddio_status = true;

    res = regulator_enable(shdisp_hayabusa_vddio);
    if (res) {
        SHDISP_ERR("regulator_enable(shdisp_hayabusa_vddio) failure. (%d)", res);
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_hayabusa_vddio_off                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_vddio_off(void)
{
    int res;

    if (!shdisp_hayabusa_vddio) {
        SHDISP_ERR("shdisp_hayabusa_vddio is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (!(shdisp_hayabusa_vddio_status)) {
        SHDISP_WARN("supply is already off.");
        return SHDISP_RESULT_SUCCESS;
    }
    shdisp_hayabusa_vddio_status = false;

    res = regulator_disable(shdisp_hayabusa_vddio);
    if (res) {
        SHDISP_ERR("regulator_disable(shdisp_hayabusa_vddio) failure. (%d)", res);
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_vdd_on                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_vdd_on(void)
{
    shdisp_hayabusa_vddio_on();
#ifdef SHDISP_HAYABUSA_ADDITIONAL_VDD
    shdisp_IO_API_Host_gpio_request(shdisp_hayabusa_vdd_gpio, "LCD_VDD");
    shdisp_IO_API_set_Host_gpio(shdisp_hayabusa_vdd_gpio, SHDISP_GPIO_CTL_HIGH);
#endif /* SHDISP_HAYABUSA_ADDITIONAL_VDD */
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_vdd_off                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_vdd_off(void)
{
#ifdef SHDISP_HAYABUSA_ADDITIONAL_VDD
    shdisp_IO_API_set_Host_gpio(shdisp_hayabusa_vdd_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_IO_API_Host_gpio_free(shdisp_hayabusa_vdd_gpio);
#endif /* SHDISP_HAYABUSA_ADDITIONAL_VDD */
    shdisp_hayabusa_vddio_off();
}
#endif /* SHDISP_HAYABUSA_VDD */

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_power_off_for_shutdown                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_power_off_for_shutdown(void)
{
    SHDISP_TRACE("in");
    SHDISP_DEBUG("excute hayabusa HW reset");
    shdisp_hayabusa_hw_reset(true);
    shdisp_IO_API_usleep(80 * 1000);
#ifdef SHDISP_HAYABUSA_VDD
    SHDISP_DEBUG("hayabusa vddio off");
    shdisp_hayabusa_vdd_off();
    shdisp_IO_API_usleep(60 * 1000);
#endif /* SHDISP_HAYABUSA_VDD */
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_set_switchcommand                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_set_switchcommand(char page)
{
    struct shdisp_dsi_cmd_desc cmd;
    char payload[2] = {0xff, 00};

    payload[1] = page;
    memset(&cmd, 0, sizeof(cmd));
    cmd.dtype = SHDISP_DTYPE_DCS_WRITE1;
    cmd.dlen = 2;
    cmd.wait = 0;
    cmd.payload = payload;

    shdisp_panel_API_mipi_dsi_cmds_tx(1, &cmd, 1);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_panel_hayabusa_reg_read                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_panel_hayabusa_reg_read(unsigned char addr, unsigned char *out_data)
{
    struct shdisp_dsi_cmd_desc cmd[1];
    char cmd_buf[1 + 2];

    memset(cmd_buf, 0x00, sizeof(cmd_buf));
    cmd_buf[0] = addr;
    cmd_buf[1] = 0x00;

    memset(cmd, 0x00, sizeof(cmd));
    cmd[0].dtype = SHDISP_DTYPE_DCS_READ;
    cmd[0].wait = 0x00;
    cmd[0].dlen = 1;
    cmd[0].payload = cmd_buf;
    cmd[0].mode = 0;

    if (shdisp_panel_API_mipi_dsi_cmds_rx(out_data, cmd, 1) != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_hw_reset                                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_hw_reset(bool reset)
{
    SHDISP_TRACE("call reset=%d", reset);
    if (reset) {
        shdisp_IO_API_set_Host_gpio(shdisp_hayabusa_rst_gpio, SHDISP_GPIO_CTL_LOW);
        shdisp_IO_API_Host_gpio_free(shdisp_hayabusa_rst_gpio);
    } else {
        shdisp_IO_API_Host_gpio_request(shdisp_hayabusa_rst_gpio, "PANEL_RST_N");
        shdisp_IO_API_set_Host_gpio(shdisp_hayabusa_rst_gpio, SHDISP_GPIO_CTL_HIGH);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipi_cmd_display_on                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipi_cmd_display_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = shdisp_hayabusa_sleepout_wait_proc2();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("Failed to shdisp_hayabusa_sleepout_wait_proc2(). (ret=%d)", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_display_on_2);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2 ret=%d", ret);
    }

#ifdef SHDISP_POWER_MODE_CHK
    shdisp_dispon_chk = SHDISP_PANEL_DISPON_CHK_INIT;
#endif /* SHDISP_POWER_MODE_CHK */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipi_cmd_lcd_off                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipi_cmd_lcd_off(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in");

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_display_off);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", ret);
    }

    SHDISP_TRACE("out");
    return ret;
}

#ifdef SHDISP_POWER_MODE_CHK
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_power_mode_chk                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_power_mode_chk(unsigned char addr)
{
    int ret;
    unsigned char read_data = 0x00;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */
    int dsi_ret;

    dsi_ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_dispon_check);
    if (dsi_ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", dsi_ret);
    }

    ret = shdisp_panel_hayabusa_reg_read(addr, &read_data);

#if defined(CONFIG_ANDROID_ENGINEERING)
    if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON_READ) {
        shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
        if (ret == SHDISP_RESULT_SUCCESS) {
            SHDISP_DEBUG("force disp on read error.");
            ret = SHDISP_RESULT_FAILURE;
        } else {
            SHDISP_DEBUG("disp on read error. (ret=%d addr=0x%02x)", ret, addr);
        }
    }
#endif /* CONFIG_ANDROID_ENGINEERING */

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_rx error");
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_READ_ERROR;
        err_code.subcode = SHDISP_DBG_SUBCODE_DISPON_CHK;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

#if defined(CONFIG_ANDROID_ENGINEERING)
        if (shdisp_dbg_API_get_recovery_check_error() == SHDISP_DBG_RECOVERY_ERROR_DISPON) {
            shdisp_dbg_API_update_recovery_check_error(SHDISP_DBG_RECOVERY_ERROR_NONE);
            SHDISP_DEBUG("force disp on error.");
            read_data = 0xFF;
        }
#endif /* CONFIG_ANDROID_ENGINEERING */

    SHDISP_DEBUG("addr = 0x%02x.read_data = 0x%02x", addr, read_data);

    if (read_data != 0x9C) {
        SHDISP_ERR("POWER_MODE error.addr = 0x%02x.read_data = 0x%02x", addr, read_data);
#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_DISPON_CHK;
        shdisp_dbg_API_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_POWER_MODE_CHK */

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_sleepout_wait_dsi                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_sleepout_wait_dsi(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_MFR_Setting_DispOn);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error1 ret=%d", ret);
        return ret;
    }

    if (shdisp_SYS_API_check_diag_mode()) {
        shdisp_hayabusa_mfr_param = SHDISP_MFR_FIXED_15HZ_EQ_OVER;
        ret = shdisp_hayabusa_mipi_mfr_param_setting(SHDISP_MFR_FIXED_15HZ_EQ_OVER, VAL_MFR_DEFAULT);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out ret=%d", ret);
            return ret;
        }
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_MFR_Setting);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("mipi_dsi_cmds_tx error2 ret=%d", ret);
        }
    } else {
        shdisp_hayabusa_mfr_param = -1;
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_MFR_Update);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("mipi_dsi_cmds_tx error3 ret=%d", ret);
        }
    }

    if (shdisp_panel_ctx.lcddr_phy_gmm.status == SHDISP_LCDDR_GMM_STATUS_OK) {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_gmm_adjusted);
    } else {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_gmm);
    }
    return(ret);
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_sleepout_wait_proc1                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_sleepout_wait_proc1(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    struct timespec ts1, ts2, ts3;
    unsigned long long wtime = 0;
    SHDISP_TRACE("in");

    getnstimeofday(&ts1);
    shdisp_hayabusa_sleepout_start = ts1;

    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_INIT);
    getnstimeofday(&ts2);
    wtime = (ts2.tv_sec - ts1.tv_sec) * 1000000;
    wtime += (ts2.tv_nsec - ts1.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of psals_power_on wait=%lld, wtime=%llu", ((10 * 1000) - wtime), wtime);

    if (wtime < (10 * 1000)) {
        shdisp_IO_API_delay_us((10 * 1000) - wtime);
    }

    ret = shdisp_hayabusa_sleepout_wait_dsi();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("mipi_dsi_cmds_tx error ret=%d", ret);
    }

    getnstimeofday(&ts3);
    wtime = (ts3.tv_sec - ts2.tv_sec) * 1000000;
    wtime += (ts3.tv_nsec - ts2.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of gmm wait=%lld, wtime=%llu", ((10 * 1000) - wtime), wtime);

    if (wtime < (10 * 1000)) {
        shdisp_IO_API_delay_us((10 * 1000) - wtime);
    }

    shdisp_bdic_API_update_led_value();
    (void)shdisp_pm_API_als_user_manager(SHDISP_DEV_TYPE_LCD, SHDISP_DEV_REQ_ON);

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_sleepout_wait_proc2                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_sleepout_wait_proc2(void)
{
    int ret = SHDISP_RESULT_SUCCESS;

    struct timespec ts_blk_end;
    unsigned long long wtime_all = 0;

    SHDISP_TRACE("in");

    getnstimeofday(&ts_blk_end);
    wtime_all = (ts_blk_end.tv_sec - shdisp_hayabusa_sleepout_start.tv_sec) * 1000000;
    wtime_all += (ts_blk_end.tv_nsec - shdisp_hayabusa_sleepout_start.tv_nsec) / 1000;
    SHDISP_PERFORMANCE("rest of all wait=%lld", ((60 * 1000) - wtime_all));

    if (wtime_all < (60 * 1000)) {
        shdisp_IO_API_delay_us((60 * 1000) - wtime_all);
    }

    SHDISP_TRACE("out");
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_devicecode_read                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_devicecode_read(int *version)
{
    int ret = SHDISP_RESULT_SUCCESS;

    unsigned char device_codeID41 = 0x00;
    unsigned char device_codeWID1 = 0x00;

    SHDISP_TRACE("in");

    if (shdisp_panel_ctx.device_code != 0xff) {
        SHDISP_DEBUG("cut version already checked !! version = 0x%02x", shdisp_panel_ctx.device_code);
        *version = shdisp_panel_ctx.device_code;
    } else {
        ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_SwitchCommand20);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out1 ret=%d", ret);
        }

        ret = shdisp_panel_hayabusa_reg_read(ADDR_ID41, &device_codeID41);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out2 ret=%d", ret);
        }

        ret = shdisp_panel_hayabusa_reg_read(ADDR_WID1, &device_codeWID1);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out3 ret=%d", ret);
        }

        SHDISP_DEBUG("device code: addr.3A, 44 = %02x, %02x", device_codeID41, device_codeWID1);

        switch (device_codeID41) {
        case VAL_ID41_3:
            *version = VER_CUT2_0;
            break;
        case VAL_ID41_2:
            *version = VER_CUT1_1;
            break;
        case VAL_ID41_1:
            switch (device_codeWID1) {
            case VAL_WID1_0:
                *version = VER_CUT1_0;
                break;
            case VAL_WID1_1:
                *version = VER_CUT1_0_FIB;
                break;
            default:
                *version = VER_CUT_DEFAULT;
                break;
            }
            break;
        default:
            *version = VER_CUT_DEFAULT;
            break;
        }

        shdisp_panel_ctx.device_code = *version;
        SHDISP_DEBUG("cut version = 0x%02x", *version);

    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_cmds_initial20_set                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_cmds_initial20_set(const char powercontrol[])
{
    unsigned int i;

    for (i = 0; i < (sizeof(mipi_sh_hayabusa_cmd_PowerControl) / sizeof(mipi_sh_hayabusa_cmd_PowerControl[0])); i++) {
        mipi_sh_hayabusa_cmd_PowerControl[i][1] = powercontrol[i];
    }
    shdisp_hayabusa_pow_ctl_flicker_exchange();
    shdisp_hayabusa_pow_ctl_app_vol_exchange();
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_pow_ctl_flicker_exchange                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_pow_ctl_flicker_exchange(void)
{
    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        mipi_sh_hayabusa_cmd_PowerControl[NO_VCOM1_L][1]   = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM1_L];
        mipi_sh_hayabusa_cmd_PowerControl[NO_VCOM2_L][1]   = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM2_L];
        mipi_sh_hayabusa_cmd_PowerControl[NO_VCOM12_H][1]  = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOM12_H];
        mipi_sh_hayabusa_cmd_PowerControl[NO_LPVCOM1][1]   = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_LPVCOM1];
        mipi_sh_hayabusa_cmd_PowerControl[NO_LPVCOM2][1]   = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_LPVCOM2];
        mipi_sh_hayabusa_cmd_PowerControl[NO_VCOMOFF_L][1] = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOMOFF_L];
        mipi_sh_hayabusa_cmd_PowerControl[NO_VCOMOFF_H][1] = mipi_sh_hayabusa_cmd_PowerControl_adjusted[NO_VCOMOFF_H];
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_pow_ctl_app_vol_exchange                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_pow_ctl_app_vol_exchange(void)
{
    if (shdisp_panel_ctx.lcddr_phy_gmm.status == SHDISP_LCDDR_GMM_STATUS_OK) {
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGH][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGH];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGL][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGL];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDN][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDN];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_GVDDP2][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP2];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGHO][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGHO];
        mipi_sh_hayabusa_cmd_PowerControl[SHDISP_HAYABUSA_VGLO][1]
                                            = mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGLO];
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipi_cmd_lcd_on                                           */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipi_cmd_lcd_on(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int version = 0;
    const char *ptr = NULL;

    SHDISP_TRACE("in");


    shdisp_hayabusa_hw_reset(false);
    shdisp_IO_API_delay_us(20 * 1000);
    shdisp_hayabusa_hw_reset(true);
    shdisp_IO_API_delay_us(1 * 1000);
    shdisp_hayabusa_hw_reset(false);
    shdisp_IO_API_delay_us(10 * 1000);

    shdisp_hayabusa_external_clk_ctl(true);
    shdisp_IO_API_delay_us(1 * 1000);

    ret = shdisp_hayabusa_devicecode_read(&version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out1 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initial10);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out2 ret=%d", ret);
        return ret;
    }

    switch (version) {
    case VER_CUT1_1:
        ptr = mipi_sh_hayabusa_cmd_PowerControl_cut1_1;
        break;
    case VER_CUT2_0:
        ptr = mipi_sh_hayabusa_cmd_PowerControl_cut2_0;
        break;
    default:
        ptr = mipi_sh_hayabusa_cmd_PowerControl_cut2_0;
        SHDISP_ERR("Not Support Panel Versino=%d", version);
        break;
    }

    shdisp_hayabusa_cmds_initial20_set(ptr);
    if (IS_FLICKER_ADJUSTED(shdisp_panel_ctx.vcom_nvram)) {
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initial20);
    } else {
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initial20_flicker_unadjusted);
    }

    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out3 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initial24);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out4 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initial26);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out5 ret=%d", ret);
        return ret;
    }

    switch (version) {
    case VER_CUT1_1:
    case VER_CUT2_0:
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initialE0_cut1_1);
        break;
    default:
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initialE0);
        break;
    }
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out6 ret=%d", ret);
        return ret;
    }

    ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_initialF0);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out7 ret=%d", ret);
        return ret;
    }

    switch (version) {
    case VER_CUT1_1:
    case VER_CUT2_0:
        ret = MIPI_DSI_COMMAND_TX(mipi_sh_hayabusa_cmds_vdd_voltage);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("out8 ret=%d", ret);
            return ret;
        }
        break;
    default:
        break;
    }

    ret = MIPI_DSI_COMMAND_TX_COMMIT(mipi_sh_hayabusa_cmds_display_on_1);
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("out10 ret=%d", ret);
        return ret;
    }

    ret = shdisp_hayabusa_sleepout_wait_proc1();
    if (ret != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("Failed to shdisp_hayabusa_sleepout_wait_proc1(). (ret=%d)", ret);
        return ret;
    }

    SHDISP_TRACE("out");
    return ret;
}

#ifndef SHDISP_NOT_SUPPORT_NO_OS
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_init_phy_gmm                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_init_phy_gmm(struct shdisp_lcddr_phy_gmm_reg *phy_gmm)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int i;
    unsigned int checksum;

    SHDISP_TRACE("in");

    if (phy_gmm == NULL) {
        SHDISP_ERR("phy_gmm is NULL.");
        return SHDISP_RESULT_FAILURE;
    }

    if (phy_gmm->status != SHDISP_LCDDR_GMM_STATUS_OK) {
        SHDISP_DEBUG("gammg status invalid. status=%02x", phy_gmm->status);
        ret = SHDISP_RESULT_FAILURE;
    } else {
        checksum = phy_gmm->status;
        for (i = 0; i < SHDISP_LCDDR_PHY_GMM_BUF_MAX; i++) {
            checksum = checksum + phy_gmm->buf[i];
        }
        for (i = 0; i < SHDISP_LCDDR_APPLIED_VOLTAGE_SIZE; i++) {
            checksum = checksum + phy_gmm->applied_voltage[i];
        }
        for (i = 0; i < SHDISP_LCDDR_ADVANCED_GAMMA_SIZE; i++) {
            checksum = checksum + phy_gmm->adv_gmm[i];
        }
        if ((checksum & 0x00FFFFFF) != phy_gmm->chksum) {
            SHDISP_DEBUG("%s: gammg chksum NG. chksum=%06x calc_chksum=%06x",
                         __func__, phy_gmm->chksum, (checksum & 0x00FFFFFF));
            ret = SHDISP_RESULT_FAILURE;
        }
    }

    if (ret == SHDISP_RESULT_FAILURE) {
        phy_gmm->status = SHDISP_LCDDR_GMM_STATUS_NOT_SET;
        SHDISP_DEBUG("phy_gmm not adjusted");
        return SHDISP_RESULT_SUCCESS;
    }

    for (i = 0; i < SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET; i++) {
        mipi_sh_hayabusa_cmd_RDPSGMM_adjusted[i * 2][1] = ((phy_gmm->buf[i] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_RDPSGMM_adjusted[i * 2 + 1][1] = (phy_gmm->buf[i] & 0x00FF);
        mipi_sh_hayabusa_cmd_RDNGGMM_adjusted[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_RDNGGMM_adjusted[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET] & 0x00FF);
        mipi_sh_hayabusa_cmd_GRPSGMM_adjusted[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 2] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_GRPSGMM_adjusted[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 2] & 0x00FF);
        mipi_sh_hayabusa_cmd_GRNGGMM_adjusted[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 3] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_GRNGGMM_adjusted[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 3] & 0x00FF);
        mipi_sh_hayabusa_cmd_BLPSGMM_adjusted[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 4] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_BLPSGMM_adjusted[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 4] & 0x00FF);
        mipi_sh_hayabusa_cmd_BLNGGMM_adjusted[i * 2][1] =
            ((phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 5] >> 8) & 0x0003);
        mipi_sh_hayabusa_cmd_BLNGGMM_adjusted[i * 2 + 1][1] =
            (phy_gmm->buf[i + SHDISP_HAYABUSA_GMM_NEGATIVE_OFFSET * 5] & 0x00FF);
    }

    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGH] = phy_gmm->applied_voltage[0];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGL] = phy_gmm->applied_voltage[1];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP] = phy_gmm->applied_voltage[2];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDN] = phy_gmm->applied_voltage[3];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_GVDDP2] = phy_gmm->applied_voltage[4];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGHO] = phy_gmm->applied_voltage[5];
    mipi_sh_hayabusa_cmd_PowerControl_adjusted[SHDISP_HAYABUSA_VGLO] = phy_gmm->applied_voltage[6];

    for (i = 0; i < SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE; i++) {
        mipi_sh_hayabusa_cmd_AdvancedRDPSGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 0];
        mipi_sh_hayabusa_cmd_AdvancedRDNGGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 1];
        mipi_sh_hayabusa_cmd_AdvancedGRPSGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 2];
        mipi_sh_hayabusa_cmd_AdvancedGRNGGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 3];
        mipi_sh_hayabusa_cmd_AdvancedBLPSGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 4];
        mipi_sh_hayabusa_cmd_AdvancedBLNGGMM_adjusted[i][1] =
            phy_gmm->adv_gmm[i + SHDISP_HAYABUSA_ADVGMM_POSI_NEGA_SIZE * 5];
    }

    SHDISP_TRACE("out ret=%04x", ret);
    return ret;
}
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_external_clk_ctl                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_external_clk_ctl(int enable)
{
    int ret = 0;

    SHDISP_TRACE("in enable=%d", enable);
    if (!shdisp_hayabusa_bbclk2) {
        SHDISP_ERR("<RESULT_FAILURE> shdisp_hayabusa_bbclk2 is NULL");
        return SHDISP_RESULT_FAILURE;
    }

    if (enable) {
        ret = clk_prepare_enable(shdisp_hayabusa_bbclk2);
        if (ret) {
            SHDISP_ERR("<RESULT_FAILURE> Failed to shdisp_hayabusa_bbclk2 ON rc = %d", ret);
            return SHDISP_RESULT_FAILURE;
        }
    } else {
        clk_disable_unprepare(shdisp_hayabusa_bbclk2);
    }
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_mipierr_clear                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_mipierr_clear(void)
{
    int ret;

    char payload[][2] = {
        {0xFF, 0x10},
        {0xAD, 0x00},
    };

    struct shdisp_dsi_cmd_desc cmds[] = {
        {SHDISP_DTYPE_DCS_WRITE1, 2, payload[0], 0, 0},
        {SHDISP_DTYPE_DCS_WRITE1, 2, payload[1], 0, 0},
    };

    ret = shdisp_panel_API_mipi_dsi_cmds_tx(1, cmds, ARRAY_SIZE(cmds));

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_workqueue_handler                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_workqueue_handler(struct work_struct *work)
{
    int ret;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    SHDISP_TRACE("in");

#ifdef SHDISP_HAYABUSA_MIPIERR_DENOISE
    if (shdisp_IO_API_get_Host_gpio(shdisp_hayabusa_mipi_err_gpio)) {
#endif /* SHDISP_HAYABUSA_MIPIERR_DENOISE */
        SHDISP_ERR("MIPI Error");

#ifdef SHDISP_RESET_LOG
        err_code.mode = SHDISP_DBG_MODE_LINUX;
        err_code.type = SHDISP_DBG_TYPE_PANEL;
        err_code.code = SHDISP_DBG_CODE_ERROR_DETECT;
        err_code.subcode = SHDISP_DBG_SUBCODE_ESD_MIPI;
        shdisp_dbg_API_err_output(&err_code, 0);
        shdisp_dbg_API_set_subcode(SHDISP_DBG_SUBCODE_ESD_MIPI);
#endif /* SHDISP_RESET_LOG */

        shdisp_API_semaphore_start();
        shdisp_hayabusa_mipierr_clear();
        shdisp_API_semaphore_end();
        ret = shdisp_API_do_lcd_det_recovery();
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("recovery request error!! ret=%d", ret);
        }
#ifdef SHDISP_HAYABUSA_MIPIERR_DENOISE
    } else {
        SHDISP_DEBUG("ED: GPIO34 Low");
        enable_irq(shdisp_hayabusa_irq_gpio);
    }
#endif /* SHDISP_HAYABUSA_MIPIERR_DENOISE */

    wake_unlock(&shdisp_hayabusa_wakelock);

    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_int_isr_common                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_int_isr_common(void)
{
    int ret;

    SHDISP_TRACE("in");

    if (shdisp_API_is_lcd_det_recovering()) {
        SHDISP_WARN("now recovering...");
        goto exit;
    }

    disable_irq_nosync(shdisp_hayabusa_irq_gpio);

    if (!shdisp_wq_hayabusa) {
        SHDISP_ERR("invalid work queue. wq=%p", shdisp_wq_hayabusa);
        goto exit;
    }

    ret = shdisp_api_get_main_disp_status();
    if (ret == SHDISP_MAIN_DISP_OFF) {
        SHDISP_DEBUG("display OFF, will be exited.");
        goto exit;
    }

    wake_lock(&shdisp_hayabusa_wakelock);

    ret = queue_work(shdisp_wq_hayabusa, &shdisp_wq_hayabusa_wk);
    if (ret == 0) {
        wake_unlock(&shdisp_hayabusa_wakelock);
        SHDISP_DEBUG("failed to queue_work(). ret=%d", ret);
    }

exit:
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_int_isr                                                   */
/* ------------------------------------------------------------------------- */
static irqreturn_t shdisp_hayabusa_int_isr(int irq_num, void *data)
{
    SHDISP_TRACE("in irq=%d", irq_num);

    shdisp_hayabusa_int_isr_common();

    SHDISP_TRACE("out");
    return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_check_mipi_err                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_check_mipi_err(void)
{
    SHDISP_TRACE("in");

    if (shdisp_IO_API_get_Host_gpio(shdisp_hayabusa_mipi_err_gpio)) {
        SHDISP_ERR("MIPI_ERR is HIGH.");
        shdisp_hayabusa_int_isr_common();
        return SHDISP_RESULT_FAILURE;
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*      shdisp_hayabusa_probe                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_probe(struct platform_device *pdev)
{
    int ret = SHDISP_RESULT_SUCCESS;
#ifdef CONFIG_OF
    struct resource *res;

    SHDISP_TRACE("in pdev=0x%p.", pdev);

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!");
        } else {
            shdisp_hayabusa_irq_gpio = res->start;
            pshdisp_hayabusa_irq_port_dev = pdev;
        }
        if (&(pdev->dev) != NULL) {
            shdisp_hayabusa_rst_gpio = of_get_named_gpio(pdev->dev.of_node, "hayabusa_rst_gpio", 0);
            if (!gpio_is_valid(shdisp_hayabusa_rst_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
            shdisp_hayabusa_mipi_err_gpio = of_get_named_gpio(pdev->dev.of_node, "hayabusa_mipi_err_gpio", 0);
            if (!gpio_is_valid(shdisp_hayabusa_mipi_err_gpio)) {
                SHDISP_ERR("mipi_err gpio not specified");
            } else {
                SHDISP_DEBUG("mipi_err gpio succusess!");
            }
#ifdef SHDISP_HAYABUSA_VDD
#ifdef SHDISP_HAYABUSA_ADDITIONAL_VDD
            shdisp_hayabusa_vdd_gpio = of_get_named_gpio(pdev->dev.of_node, "hayabusa_vdd_gpio", 0);
            if (!gpio_is_valid(shdisp_hayabusa_vdd_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
#endif /* SHDISP_HAYABUSA_ADDITIONAL_VDD */
            shdisp_hayabusa_vddio = devm_regulator_get(&(pdev->dev), SHDISP_HAYABUSA_VDDIO_ID);
            if (IS_ERR(shdisp_hayabusa_vddio)) {
                SHDISP_ERR("devm_regulator_get(%s) failure. (%ld)", SHDISP_HAYABUSA_VDDIO_ID
                                                                        , PTR_ERR(shdisp_hayabusa_vddio));
                shdisp_hayabusa_vddio = NULL;
            } else {
                SHDISP_DEBUG("devm_regulator_get(%s) succusess!", SHDISP_HAYABUSA_VDDIO_ID);
            }
#endif /* SHDISP_HAYABUSA_VDD */
            shdisp_hayabusa_bbclk2 = clk_get(&(pdev->dev), SHDISP_HAYABUSA_BBCLK2_ID);
            if (IS_ERR(shdisp_hayabusa_bbclk2)) {
                SHDISP_ERR("clk_get(%s) failure. (%ld)", SHDISP_HAYABUSA_BBCLK2_ID, PTR_ERR(shdisp_hayabusa_bbclk2));
                shdisp_hayabusa_bbclk2 = NULL;
            } else {
                SHDISP_DEBUG("clk_get(%s) succusess!", SHDISP_HAYABUSA_BBCLK2_ID);
            }
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    }

    SHDISP_TRACE("out ret=%d irq=%d", ret, shdisp_hayabusa_irq_gpio);
#endif /* CONFIG_OF */
    return ret;
}


/* ------------------------------------------------------------------------- */
/*      shdisp_hayabusa_remove                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_remove(struct platform_device *pdev)
{
    SHDISP_TRACE("in.");
#ifdef SHDISP_HAYABUSA_VDD
    if (shdisp_hayabusa_vddio) {
        regulator_put(shdisp_hayabusa_vddio);
        shdisp_hayabusa_vddio = NULL;
    }
#endif /* SHDISP_HAYABUSA_VDD */
    if (shdisp_hayabusa_bbclk2) {
        clk_put(shdisp_hayabusa_bbclk2);
        shdisp_hayabusa_bbclk2 = NULL;
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_hayabusa_dt_match[] = {
    { .compatible = "sharp,shdisp_hayabusa", },
    {}
};
#else
#define shdisp_hayabusa_dt_match NULL;
#endif /* CONFIG_OF */

static struct platform_driver shdisp_hayabusa_driver = {
    .probe = shdisp_hayabusa_probe,
    .remove = shdisp_hayabusa_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_hayabusa",
        .of_match_table = shdisp_hayabusa_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/*      shdisp_hayabusa_register_driver                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_register_driver(void)
{
    SHDISP_TRACE("in.");
    SHDISP_TRACE("out");
    return platform_driver_register(&shdisp_hayabusa_driver);
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_dump_reg_sub                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_hayabusa_dump_reg_sub(int size, struct shdisp_dsi_cmd_desc *ptr)
{
    int i, j;
    short dlen;
    unsigned char addr, page, read_data;
    unsigned char read_buf[SHDISP_LCDDR_BUF_MAX];

    for (i = 0; i < size; i++) {
        addr = *(ptr->payload);
        if (addr == 0xFF) {
            page = *(ptr->payload + 1);
            shdisp_hayabusa_API_diag_write_reg(addr, &page, 1);
            printk("[SHDISP] PANEL_PARA_DUMP PAGE: %02X\n", page);
        } else {
            switch (ptr->dtype) {
            case SHDISP_DTYPE_DCS_WRITE1:
                read_data = 0x00;
                shdisp_hayabusa_API_diag_read_reg(addr, &read_data, 1);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: %02X\n", addr, read_data);
                break;
            case SHDISP_DTYPE_DCS_LWRITE:
                memset(read_buf, 0x00, sizeof(read_buf));
                dlen = ptr->dlen - 1;
                if (dlen > sizeof(read_buf)) {
                    dlen = sizeof(read_buf);
                }
                shdisp_hayabusa_API_diag_read_reg(addr, read_buf, (unsigned char)dlen);
                printk("[SHDISP] PANEL_PARA_DUMP 0x%02X: ", addr);
                for (j = 0; j < dlen; j++) {
                    printk("%02X,", read_buf[j]);
                }
                printk("\n");
                break;
            default:
                break;
            }
        }
        ptr++;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_hayabusa_dump_reg                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_hayabusa_dump_reg(void)
{
    int arraysize;
    struct shdisp_dsi_cmd_desc *dumpptr;

    printk("[SHDISP] PANEL PARAMETER INFO ->>\n");

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initial10);
    dumpptr = mipi_sh_hayabusa_cmds_initial10;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initial20);
    dumpptr = mipi_sh_hayabusa_cmds_initial20;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initial24);
    dumpptr = mipi_sh_hayabusa_cmds_initial24;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initial26);
    dumpptr = mipi_sh_hayabusa_cmds_initial26;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initialE0);
    dumpptr = mipi_sh_hayabusa_cmds_initialE0;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_initialF0);
    dumpptr = mipi_sh_hayabusa_cmds_initialF0;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_MFR_Setting_DispOn);
    dumpptr = mipi_sh_hayabusa_cmds_MFR_Setting_DispOn;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    if (shdisp_SYS_API_check_diag_mode()) {
        arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_MFR_Setting);
        dumpptr = mipi_sh_hayabusa_cmds_MFR_Setting;
        shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);
    } else {
        arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_MFR_Update);
        dumpptr = mipi_sh_hayabusa_cmds_MFR_Update;
        shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);
    }

    arraysize = ARRAY_SIZE(mipi_sh_hayabusa_cmds_gmm);
    dumpptr = mipi_sh_hayabusa_cmds_gmm;
    shdisp_hayabusa_dump_reg_sub(arraysize, dumpptr);

    printk("[SHDISP] PANEL PARAMETER INFO <<-\n");
    return SHDISP_RESULT_SUCCESS;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
