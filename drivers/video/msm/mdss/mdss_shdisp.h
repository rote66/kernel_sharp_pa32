/* drivers/video/msm/mdss/mdss_shdisp.h  (Display Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

#ifndef MDSS_SHDISP_H
#define MDSS_SHDISP_H

#include <linux/types.h>
#include "mdss_panel.h"
#include "mdss_fb.h"
#include <linux/leds.h>
#include <linux/msm_mdp.h>
#include <sharp/shdisp_kerl.h>

struct mdss_dsi_ctrl_pdata;

extern void mdss_shdisp_lock_recovery(void);
extern void mdss_shdisp_unlock_recovery(void);
extern void mdss_shdisp_lock_display(void);
extern void mdss_shdisp_unlock_display(void);
extern void mdss_shdisp_lock_blank(void);
extern void mdss_shdisp_unlock_blank(void);

extern bool mdss_shdisp_get_disp_status(void);

extern void mdss_shdisp_dsi_panel_power_on(struct mdss_panel_data *pdata);
extern void mdss_shdisp_dsi_panel_power_off(struct mdss_panel_data *pdata);
extern void mdss_shdisp_dsi_panel_on(struct mdss_panel_data *pdata);
extern void mdss_shdisp_dsi_panel_off(struct mdss_panel_data *pdata);
extern void mdss_shdisp_dsi_panel_start_display(void);
extern void mdss_shdisp_dsi_panel_post_video_start(void);
extern void mdss_shdisp_set_dsi_ctrl(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

extern void mdss_shdisp_bkl_ctl(u32 bl_level);
extern bool mdss_shdisp_is_disp_on(void);
extern void mdss_shdisp_tri_led_set_color(char red, char green, char blue);

#ifdef CONFIG_USES_SHLCDC
extern int mdss_shdisp_pll_ctl(int ctl);
#endif /* CONFIG_USES_SHLCDC */

extern bool mdss_shdisp_get_upper_unit_is_connected(void);
extern bool mdss_shdisp_get_bdic_is_exist(void);
extern void mdss_shdisp_shutdown(int seq);
extern void mdss_shdisp_video_transfer_ctrl_set_flg(struct msm_fb_data_type *mfd, bool change);
extern int mdss_shdisp_video_transfer_ctrl_kickoff(struct msm_fb_data_type *mfd, int onoff);
extern int mdss_shdisp_display_done(void);
extern struct mdss_mdp_ctl* mdss_shdisp_get_mdpctrl(int fbinx);

#ifdef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_shdisp_dsi_mipi_err_clear(void);
extern int mdss_shdisp_dsi_mipi_err_ctrl(bool enable);
extern void mdss_shdisp_dsi_panel_det_recovery(struct mdss_dsi_ctrl_pdata *ctrl, int status);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

extern void mdss_shdisp_mdp_cmd_kickoff(void);
extern void mdss_shdisp_clk_ctrl(bool onoff);
extern struct shdisp_gc_lut *mdss_shdisp_api_get_gc_lut(void);
extern struct shdisp_igc_lut *mdss_shdisp_api_get_igc_lut(void);

extern int mdss_shdisp_mdp_cmd_wait4ppdone(void);
extern void mdss_shdisp_set_atomic_seq(bool seq);

#if defined(CONFIG_ANDROID_ENGINEERING)
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...) \
                pr_debug(",[SHDISP_PERFORM]" fmt, ## args);
#else /* CONFIG_ANDROID_ENGINEERING */
    #define SHDISP_VIDEO_PERFORMANCE(fmt, args...)
#endif /* CONFIG_ANDROID_ENGINEERING */

enum {
    MDSS_PRE_SHUTDOWN,
    MDSS_POST_SHUTDOWN,
};

#endif /* MDSS_SHDISP_H */
