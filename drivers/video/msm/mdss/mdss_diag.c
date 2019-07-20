/* drivers/video/msm/mdss/mdss_diag.c  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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
#include <mdss_shdisp.h>
#include <mdss_diag.h>
#include <linux/types.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/shdisp_dsi.h>
#include "mdss_fb.h"
#include <mdss_dsi.h>
#include <mdss_mdp.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include "mdss_debug.h"

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
#define MDSS_DIAG_MIPI_CHECK_ENABLE
#define MDSS_DIAG_MIPI_CLKCHG_ENABLE
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

#define MDSS_DIAG_MIPI_CHECK_AMP_OFF		(0x0164)
#define MDSS_DIAG_WAIT_1FRAME_US		(16666)

#define DSI_TIMING_8996_CKLN_DATA_WIDTH		(NUM_MDP_MIPI_CLKCHG_CKLN_TIMING)
#define DSI_TIMING_8996_DLN_NUM				(4)
#define DSI_TIMING_8996_DLN_DATA_WIDTH		(NUM_MDP_MIPI_CLKCHG_DLN_TIMING + 1)
#define DSI_TIMING_8996_CKLN_OFFSET			(DSI_TIMING_8996_DLN_NUM * DSI_TIMING_8996_DLN_DATA_WIDTH)

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
static uint8_t mdss_diag_mipi_check_amp_data;
static uint8_t mdss_diag_mipi_check_rec_sens_data_master;
static uint8_t mdss_diag_mipi_check_rec_sens_data_slave;
static int mdss_diag_mipi_check_exec_state = false;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
static int mdss_diag_mipi_check_exec(uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param *mipi_check_param, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv_master, uint8_t sensitiv_slave, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave, struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_mipi_check_test(uint8_t *result, uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_mipi_check_test_video(uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_mipi_check_test_cmd(uint8_t *result, uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl);
static int mdss_diag_read_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave);
static int mdss_diag_write_sensitiv(uint8_t sensitiv_master, uint8_t sensitiv_slave);
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl);
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl);
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl);
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata);
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata);
#if defined(CONFIG_SHDISP_PANEL_ANDY)
static int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl);
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) */
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param);
static int mdss_diag_reconfig(struct mdss_panel_data *pdata);
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

#if defined(CONFIG_SHDISP_PANEL_ANDY)
extern int shdisp_api_set_freq_param(mdp_mipi_clkchg_panel_t *freq);
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) */
#ifndef SHDISP_DET_DSI_MIPI_ERROR
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,int enable);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
extern void mdss_shdisp_video_transfer_ctrl(int onoff, int commit);
extern int mdss_shdisp_host_dsi_tx(int commit,
		struct shdisp_dsi_cmd_desc *shdisp_cmds, int size);
extern int mdss_shdisp_host_dsi_rx(struct shdisp_dsi_cmd_desc *cmds,
		unsigned char *rx_data, int rx_size);
extern void mdss_dsi_hs_clk_lane_enable(bool enable);
extern void mdss_dsi_8996_phy_timing_config(struct mdss_dsi_ctrl_pdata *ctrl);

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check_get_exec_state(void)
{
#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
	return mdss_diag_mipi_check_exec_state;
#else  /* MDSS_DIAG_MIPI_CHECK_ENABLE */
	return false;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_check(struct mdp_mipi_check_param *mipi_check_param, struct mdss_panel_data *pdata)
{
#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
	int ret;
	u32 isr;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_dsi_ctrl_pdata *sctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	if (pdata->next) {
		sctrl_pdata = container_of(pdata->next, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	pr_debug("%s: in master=%p slave=%p\n", __func__, ctrl_pdata, sctrl_pdata);

	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is NULL.\n", __func__);
		return -ENXIO;
	}

	if (sctrl_pdata && (ctrl_pdata->panel_mode != DSI_CMD_MODE)) {
		pr_err("%s: not support video mode.\n", __func__);
		return -EINVAL;
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 0);
	}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	mdss_diag_mipi_check_exec_state = true;

	mdss_diag_dsi_cmd_bta_sw_trigger(ctrl_pdata);
	if (sctrl_pdata) {
		mdss_diag_dsi_cmd_bta_sw_trigger(sctrl_pdata);
	}

	mdss_diag_mipi_check_amp_data = 0;
	mdss_diag_mipi_check_rec_sens_data_master = 0;
	mdss_diag_mipi_check_rec_sens_data_slave  = 0;
	mdss_diag_mipi_check_get_param(
			&mdss_diag_mipi_check_amp_data,
			&mdss_diag_mipi_check_rec_sens_data_master,
			&mdss_diag_mipi_check_rec_sens_data_slave,
			ctrl_pdata);

	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(false, false);
	}

	ret = mdss_diag_mipi_check_manual(mipi_check_param, ctrl_pdata, sctrl_pdata);

	mdss_diag_mipi_check_set_param(
			mdss_diag_mipi_check_amp_data,
			mdss_diag_mipi_check_rec_sens_data_master,
			mdss_diag_mipi_check_rec_sens_data_slave,
			ctrl_pdata, sctrl_pdata);

	if (ctrl_pdata->panel_mode == DSI_VIDEO_MODE) {
		mdss_shdisp_video_transfer_ctrl(true, true);
	}

	mdss_diag_mipi_check_exec_state = false;

	/* MMSS_DSI_0_INT_CTRL */
	isr = MIPI_INP(ctrl_pdata->ctrl_base + 0x0110);
	MIPI_OUTP(ctrl_pdata->ctrl_base + 0x0110, isr);
	if (sctrl_pdata) {
		isr = MIPI_INP(sctrl_pdata->ctrl_base + 0x0110);
		MIPI_OUTP(sctrl_pdata->ctrl_base + 0x0110, isr);
	}

#ifndef SHDISP_DET_DSI_MIPI_ERROR
	mdss_dsi_err_intr_ctrl(ctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	if (sctrl_pdata) {
		mdss_dsi_err_intr_ctrl(sctrl_pdata, DSI_INTR_ERROR_MASK, 1);
	}
#endif /* SHDISP_DET_DSI_MIPI_ERROR */

	pr_debug("%s: out\n", __func__);

	return ret;
#else  /* MDSS_DIAG_MIPI_CHECK_ENABLE */
	return 0;
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
int mdss_diag_mipi_clkchg(struct mdp_mipi_clkchg_param *mipi_clkchg_param)
{
#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
	int ret = 0;
	struct mdss_mdp_ctl *pctl;
	struct mdss_panel_data *pdata;
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	struct shdisp_freq_params freq;
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	pr_debug("%s: called\n", __func__);
	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("LCDERR:[%s] mdpctrl is NULL.\n", __func__);
		return -EIO;
	}
	MDSS_XLOG(mipi_clkchg_param->host.clock_rate, XLOG_FUNC_ENTRY);
	pdata = pctl->panel_data;

	mdss_shdisp_lock_recovery();

	pr_debug("%s: flush_work called\n", __func__);
	ret = mdss_shdisp_mdp_cmd_wait4ppdone();
	pr_debug("%s: flush_work end ret=%d\n", __func__, ret);

	mdss_diag_mipi_clkchg_param_log(mipi_clkchg_param);

	mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);

#if defined(CONFIG_SHDISP_PANEL_ANDY)
	shdisp_api_set_freq_param(&mipi_clkchg_param->panel);
#endif  /* defined(CONFIG_SHDISP_PANEL_ANDY) */
#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	freq.internal_osc = mipi_clkchg_param->internal_osc;
	mutex_lock(&pctl->rsrc_lock);
	shdisp_api_set_freq_param(&freq);
	mutex_unlock(&pctl->rsrc_lock);
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	if (mdss_shdisp_is_disp_on()) {
		ret = mdss_diag_mipi_clkchg_setparam(mipi_clkchg_param, pctl);
	} else {
		ret = mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	}

	mdss_shdisp_unlock_recovery();

	pr_debug("%s: end ret(%d)\n", __func__, ret);
	MDSS_XLOG(XLOG_FUNC_EXIT);

	return ret;
#else  /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
	return 0;
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */
}

#ifdef MDSS_DIAG_MIPI_CHECK_ENABLE
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_manual(struct mdp_mipi_check_param *mipi_check_param, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	uint8_t result[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};
	uint8_t dummy[2] = {MDSS_MIPICHK_RESULT_OK, MDSS_MIPICHK_RESULT_OK};

	pr_debug("%s: in\n", __func__);

	if ((mipi_check_param->amp & ~0x07) != 0) {
		pr_err("%s: out of range. amp=0x%02X\n", __func__, mipi_check_param->amp);
		return -ENXIO;
	}

	if ((mipi_check_param->sensitiv & ~0x0F) != 0) {
		pr_err("%s: out of range. sensitiv=0x%02X\n", __func__, mipi_check_param->sensitiv);
		return -ENXIO;
	}

	ret = mdss_diag_mipi_check_exec(result, mipi_check_param->frame_cnt, mipi_check_param->amp, mipi_check_param->sensitiv, ctrl, sctrl);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
		result[1] = MDSS_MIPICHK_RESULT_NG;
	}

	if ((result[0] != MDSS_MIPICHK_RESULT_OK) || (result[1] != MDSS_MIPICHK_RESULT_OK)) {
		pr_debug("%s: recovery display.\n", __func__);
		mdss_diag_mipi_check_exec(dummy, 1, MDSS_MIPICHK_AMP_NUM - 1, MDSS_MIPICHK_SENSITIV_NUM - 1, ctrl, sctrl);
	}

	mipi_check_param->result_master = result[0];
	mipi_check_param->result_slave  = result[1];

	pr_debug("%s: out master=%d slave=%d\n", __func__, mipi_check_param->result_master, mipi_check_param->result_slave);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_exec(uint8_t *result, uint8_t frame_cnt, uint8_t amp, uint8_t sensitiv, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	uint8_t set_amp;
	uint8_t set_sensitiv_master, set_sensitiv_slave;

	static const uint8_t amp_tbl[MDSS_MIPICHK_AMP_NUM] = {
		0x03,
		0x02,
		0x00,
		0x01,
		0x04,
		0x05,
		0x06,
		0x07
	};

	pr_debug("%s: in frame_cnt=0x%02X amp=0x%02X sensitiv=0x%02X\n", __func__, frame_cnt, amp, sensitiv);

	set_amp = (amp_tbl[amp] << 1) | 1;

	set_sensitiv_master  = sensitiv << 4;
	set_sensitiv_master |= mdss_diag_mipi_check_rec_sens_data_master & 0x0F;
	set_sensitiv_slave   = sensitiv << 4;
	set_sensitiv_slave  |= mdss_diag_mipi_check_rec_sens_data_slave  & 0x0F;

	mdss_diag_mipi_check_set_param(set_amp, set_sensitiv_master, set_sensitiv_slave, ctrl, sctrl);

	ret = mdss_diag_mipi_check_test(result, frame_cnt, ctrl, sctrl);

	pr_debug("%s: out ret=%d\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_set_param(uint8_t amp, uint8_t sensitiv_master, uint8_t sensitiv_slave, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X sensitiv_slave=0x%02X\n", __func__, amp, sensitiv_master, sensitiv_slave);

#ifndef CONFIG_SHDISP /* CUST_ID_00034 */
	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	MIPI_OUTP((ctrl->phy_io.base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF, amp);
	if (sctrl) {
		MIPI_OUTP((sctrl->phy_io.base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF, amp);
	}
	wmb();
#endif /* CONFIG_SHDISP */

	mdss_diag_write_sensitiv(sensitiv_master, sensitiv_slave);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_check_get_param(uint8_t *amp, uint8_t *sensitiv_master, uint8_t *sensitiv_slave, struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;

#ifndef CONFIG_SHDISP /* CUST_ID_00034 */
	/* MMSS_DSI_0_PHY_REG_DSIPHY_REGULATOR_CTRL_0 */
	*amp = MIPI_INP((ctrl->phy_io.base) + MDSS_DIAG_MIPI_CHECK_AMP_OFF);
#endif /* CONFIG_SHDISP */

	ret = mdss_diag_read_sensitiv(sensitiv_master, sensitiv_slave);

	pr_debug("%s: amp=0x%02X sensitiv_master=0x%02X sensitiv_slave=0x%02X ret=%d\n", __func__, *amp, *sensitiv_master, *sensitiv_slave, ret);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test(uint8_t *result, uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	char mode;

	mode = ctrl->panel_mode;

	if (mode == DSI_VIDEO_MODE) {
		result[0] = mdss_diag_mipi_check_test_video(frame_cnt, ctrl);
	} else if (mode == DSI_CMD_MODE) {
		ret = mdss_diag_mipi_check_test_cmd(result, frame_cnt, ctrl, sctrl);
	} else {
		pr_err("%s: invalid panel_mode=%d\n", __func__, mode);
		ret = -EINVAL;
	}

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_video(uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	uint32_t sleep;

	sleep = frame_cnt * MDSS_DIAG_WAIT_1FRAME_US;
	pr_debug("%s: frame_cnt=%d sleep=%d\n", __func__, frame_cnt, sleep);

	mdss_shdisp_video_transfer_ctrl(true, true);

	usleep_range(sleep,sleep);

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		ret = MDSS_MIPICHK_RESULT_NG;
	} else {
		ret = MDSS_MIPICHK_RESULT_OK;
	}

	mdss_shdisp_video_transfer_ctrl(false, false);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_check_test_cmd(uint8_t *result, uint8_t frame_cnt, struct mdss_dsi_ctrl_pdata *ctrl, struct mdss_dsi_ctrl_pdata *sctrl)
{
	int ret = 0;
	int ret2 = 0;
	int i;
	struct mdss_mdp_ctl *pctl;
	struct mdss_mdp_ctl *sctl;

	pr_debug("%s: in\n", __func__);

	pctl = mdss_shdisp_get_mdpctrl(0);
	if (!pctl) {
		pr_err("%s: pctl is NULL.\n", __func__);
		return -EINVAL;
	}

	if (!pctl->ops.display_fnc) {
		pr_err("%s: display_fnc is NULL.\n", __func__);
		return -EINVAL;
	}

	sctl = mdss_mdp_get_split_ctl(pctl);

	for (i = 0; i < frame_cnt; i++) {
		pr_debug("%s: frame=%d\n", __func__, i);

		mutex_lock(&pctl->lock);

		mdss_mdp_ctl_perf_set_transaction_status(pctl, PERF_SW_COMMIT_STATE, PERF_STATUS_BUSY);
		if (sctl) {
			mdss_mdp_ctl_perf_set_transaction_status(sctl, PERF_SW_COMMIT_STATE, PERF_STATUS_BUSY);
		}

		mdss_mdp_ctl_perf_update_ctl(pctl, 1);

		if (pctl->ops.wait_pingpong) {
			ret2 = pctl->ops.wait_pingpong(pctl, NULL);
			if(ret2){
				pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
			}
		}

		if (sctl && sctl->ops.wait_pingpong) {
			ret2 = sctl->ops.wait_pingpong(sctl, NULL);
			if(ret2){
				pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
			}
		}

		ret = pctl->ops.display_fnc(pctl, NULL);
		if (ret) {
			pr_err("%s: failed to display_fnc. ret=%d\n", __func__, ret);
			mutex_unlock(&pctl->lock);
			return ret;
		}

		mutex_unlock(&pctl->lock);
	}

	if (pctl->ops.wait_pingpong) {
		ret2 = pctl->ops.wait_pingpong(pctl, NULL);
		if(ret2){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
		}
	}

	if (sctl && sctl->ops.wait_pingpong) {
		ret2 = sctl->ops.wait_pingpong(sctl, NULL);
		if(ret2){
			pr_err("%s: failed to wait_pingpong. ret=%d\n", __func__, ret2);
		}
	}

	ret = mdss_diag_dsi_cmd_bta_sw_trigger(ctrl);
	if (ret) {
		result[0] = MDSS_MIPICHK_RESULT_NG;
	} else {
		result[0] = MDSS_MIPICHK_RESULT_OK;
	}

	if (sctrl) {
		ret = mdss_diag_dsi_cmd_bta_sw_trigger(sctrl);
		if (ret) {
			result[1] = MDSS_MIPICHK_RESULT_NG;
		} else {
			result[1] = MDSS_MIPICHK_RESULT_OK;
		}
	}

	pr_debug("%s: out\n", __func__);

	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_read_sensitiv(uint8_t *sensitiv_master, uint8_t *sensitiv_slave)
{
	int ret = 0;

	char payload_sensitiv[3][2] = {
		{0xFF, 0xE0},
		{0x7E, 0x00},
		{0x97, 0x00},
	};

	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[0]},
		{SHDISP_DTYPE_DCS_READ,   1, payload_sensitiv[1]},
		{SHDISP_DTYPE_DCS_READ,   1, payload_sensitiv[2]},
	};

	mdss_shdisp_host_dsi_tx(1, &cmds_sensitiv[0], 1);

	ret = mdss_shdisp_host_dsi_rx(&cmds_sensitiv[1], sensitiv_master, 1);
	ret = mdss_shdisp_host_dsi_rx(&cmds_sensitiv[2], sensitiv_slave, 1);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_write_sensitiv(uint8_t sensitiv_master, uint8_t sensitiv_slave)
{
	int ret = 0;

	char payload_sensitiv[3][2] = {
		{0xFF, 0xE0},
		{0x7E, 0x00},
		{0x97, 0x00},
	};

	struct shdisp_dsi_cmd_desc cmds_sensitiv[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[1]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, payload_sensitiv[2]},
	};

	payload_sensitiv[1][1] = sensitiv_master;
	payload_sensitiv[2][1] = sensitiv_slave;

	ret = mdss_shdisp_host_dsi_tx(1, cmds_sensitiv, ARRAY_SIZE(cmds_sensitiv));

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_cmd_bta_sw_trigger(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;
	u32 status;
	int timeout_us = 35000;

	pr_debug("%s: in\n", __func__);

	if (ctrl == NULL) {
		pr_err("%s: ctrl is NULL.\n", __func__);
		return -EINVAL;
	}

	/* CMD_MODE_BTA_SW_TRIGGER */
	MIPI_OUTP((ctrl->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us)) {
		pr_info("%s: timeout. status=0x%08x\n", __func__, status);
		return -EIO;
	}

	ret = mdss_diag_dsi_ack_err_status(ctrl);

	pr_debug("%s: out status=0x%08x ret=%d\n", __func__, status, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_dsi_ack_err_status(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 status;
	unsigned char *base;
	u32 ack = 0x10000000;

	base = ctrl->ctrl_base;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */
	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		/* Writing of an extra 0 needed to clear error bits */
		MIPI_OUTP(base + 0x0068, 0);

		status &= ~(ack);
		if(status){
			pr_err("%s: status=0x%08x\n", __func__, status);
			return -EIO;
		}
	}

	return 0;
}
#endif /* MDSS_DIAG_MIPI_CHECK_ENABLE */

#ifdef MDSS_DIAG_MIPI_CLKCHG_ENABLE
/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_setparam(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);
	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_suspend(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
		mdss_shdisp_video_transfer_ctrl(false, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	} else {
		mutex_lock(&pctl->rsrc_lock);
	}

#if defined(CONFIG_SHDISP_PANEL_ANDY)
	ret |= mdss_diag_mipi_clkchg_panel(mipi_clkchg_param, pctl);
#endif
	ret |= mdss_diag_mipi_clkchg_host(pctl);

	if (pctl->is_video_mode) {
#ifndef SHDISP_DISABLE_HR_VIDEO
		ret |= mdss_mdp_hr_video_resume(pctl, false);
#else  /* SHDISP_DISABLE_HR_VIDEO */
		mdss_shdisp_video_transfer_ctrl(true, false);
#endif /* SHDISP_DISABLE_HR_VIDEO */
	} else {
		mutex_unlock(&pctl->rsrc_lock);
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void __mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	int i,j;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	char *dst_timing = pinfo->mipi.dsi_phy_db.timing_8996;
	char *src_timing = mipi_clkchg_param->host.timing_ctrl;

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);

	pinfo->clk_rate = mipi_clkchg_param->host.clock_rate;
	pinfo->xres = mipi_clkchg_param->host.display_width;
	pinfo->yres = mipi_clkchg_param->host.display_height;
	pinfo->lcdc.h_pulse_width = mipi_clkchg_param->host.hsync_pulse_width;
	pinfo->lcdc.h_back_porch = mipi_clkchg_param->host.h_back_porch;
	pinfo->lcdc.h_front_porch = mipi_clkchg_param->host.h_front_porch;
	pinfo->lcdc.v_pulse_width = mipi_clkchg_param->host.vsync_pulse_width;
	pinfo->lcdc.v_back_porch = mipi_clkchg_param->host.v_back_porch;
	pinfo->lcdc.v_front_porch = mipi_clkchg_param->host.v_front_porch;
	pinfo->mipi.t_clk_post = mipi_clkchg_param->host.t_clk_post;
	pinfo->mipi.t_clk_pre = mipi_clkchg_param->host.t_clk_pre;

	j = DSI_TIMING_8996_CKLN_OFFSET;
	for ( i=MDP_MIPI_CLKCHG_CKLN_TIMING_START; i < MDP_MIPI_CLKCHG_CKLN_TIMING_END+1; i++ ) {
		dst_timing[j] = src_timing[i];
		j++;
	}
	j = 0;
	for ( i=MDP_MIPI_CLKCHG_DLN_TIMING_START; i < MDP_MIPI_CLKCHG_DLN_TIMING_END+1; i++ ) {
		dst_timing[j] = src_timing[i];
		dst_timing[j+(DSI_TIMING_8996_DLN_DATA_WIDTH * 1)] = src_timing[i];
		dst_timing[j+(DSI_TIMING_8996_DLN_DATA_WIDTH * 2)] = src_timing[i];
		dst_timing[j+(DSI_TIMING_8996_DLN_DATA_WIDTH * 3)] = src_timing[i];
		if (j >= DSI_TIMING_8996_CKLN_DATA_WIDTH)
			dst_timing[j+(DSI_TIMING_8996_CKLN_OFFSET)] = src_timing[i];
		j++;
	}

	pr_debug("%s: end\n", __func__);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_host_data(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_panel_data *pdata)
{
	pr_debug("%s: called\n", __func__);

	__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata);
	if (pdata->next) {
		__mdss_diag_mipi_clkchg_host_data(mipi_clkchg_param, pdata->next);
	}

	pr_debug("%s: end\n", __func__);
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl, struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called panel name = %s\n", __func__, pdata->panel_info.panel_name);
	ret = mdss_diag_reconfig(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_stop_clkln_hs(struct mdss_panel_data *pdata) 
{
	int cnt = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	cnt = ctrl_pdata->clk_lane_cnt;
	if (cnt) {
		pr_debug("%s: ctrl_pdata(0x%p), clk_lane_cnt = %d\n", __func__, 
				ctrl_pdata, cnt);
		mdss_dsi_hs_clk_lane_enable(false);
	}
	return cnt;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_start_clkln_hs(struct mdss_panel_data *pdata, int cnt)
{
	if (cnt) {
		pr_debug("%s: clk_lane_cnt = %d\n", __func__,  cnt);
		mdss_dsi_hs_clk_lane_enable(true);
	}
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_host(struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	struct mdss_panel_data *pdata = pctl->panel_data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int mpdata_clk_ln_hs_cnt = 0;

	pr_debug("%s: called\n", __func__);
	
	ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata);
	if (pdata->next) {
		ret |= mdss_diag_mipi_clkchg_panel_clk_update(pdata->next);
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mpdata_clk_ln_hs_cnt = mdss_diag_mipi_stop_clkln_hs(pdata);

	ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_host(pctl, pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_host err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	mdss_diag_mipi_start_clkln_hs(pdata, mpdata_clk_ln_hs_cnt);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_update(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int __mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_panel_info *pinfo = &(pdata->panel_info);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u64 clk_rate;

	pr_debug("%s: called\n", __func__);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	ret = mdss_dsi_clk_div_config(pinfo, pinfo->mipi.frame_rate);
	if (ret) {
		pr_err("LCDERR:[%s] mdss_dsi_clk_div_config err.\n", __func__);
		return ret;
	}
	ctrl_pdata->pclk_rate =
		pinfo->mipi.dsi_pclk_rate;
	clk_rate = pinfo->clk_rate;
	do_div(clk_rate, 8U);
	ctrl_pdata->byte_clk_rate = (u32) clk_rate;

	ret = mdss_dsi_clk_set_link_rate(ctrl_pdata->dsi_clk_handle,
		MDSS_DSI_LINK_BYTE_CLK, ctrl_pdata->byte_clk_rate,
		MDSS_DSI_CLK_UPDATE_CLK_RATE_AT_ON);
	if (ret) {
		pr_err("%s: dsi_byte_clk - clk_set_rate failed\n",
				__func__);
		return ret;
	}

	ret= mdss_dsi_clk_set_link_rate(ctrl_pdata->dsi_clk_handle,
		MDSS_DSI_LINK_PIX_CLK, ctrl_pdata->pclk_rate,
		MDSS_DSI_CLK_UPDATE_CLK_RATE_AT_ON);
	if (ret) {
		pr_err("%s: dsi_pixel_clk - clk_set_rate failed\n",
			__func__);
		return ret;
	}
	pr_debug("%s: pclk_rate = %d byte_clk_rate = %d ctrl_pdata=0x%p DSI_%d\n", __func__
	, ctrl_pdata->pclk_rate, ctrl_pdata->byte_clk_rate, ctrl_pdata, ctrl_pdata->ndx);

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_mipi_clkchg_panel_clk_data(struct mdss_panel_data *pdata)
{
	int ret = 0;

	pr_debug("%s: called\n", __func__);

	ret = __mdss_diag_mipi_clkchg_panel_clk_data(pdata);
	if (ret) {
		pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
			, __func__, pdata->panel_info.panel_name);
	}
	if (pdata->next) {
		ret |= __mdss_diag_mipi_clkchg_panel_clk_data(pdata->next);
		if (ret) {
			pr_err("LCDERR:[%s] __mdss_diag_mipi_clkchg_panel_clk_data err panel name = %s.\n"
				, __func__, pdata->next->panel_info.panel_name);
		}
	}

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static int mdss_diag_reconfig(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *pinfo;

	if (!pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s, start\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
		panel_data);
	pinfo = &pdata->panel_info.mipi;

	/* reset DSI */
	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_ON);
	mdss_dsi_sw_reset(ctrl_pdata, true);
	mdss_dsi_8996_phy_timing_config(ctrl_pdata);
	mdss_dsi_ctrl_setup(ctrl_pdata);
	mdss_dsi_controller_cfg(true, pdata);
	mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS, MDSS_DSI_CLK_OFF);

	pr_debug("%s: end\n", __func__);
	
	return 0;
}

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
#if defined(CONFIG_SHDISP_PANEL_ANDY)
static int mdss_diag_mipi_clkchg_panel(struct mdp_mipi_clkchg_param *mipi_clkchg_param, struct mdss_mdp_ctl *pctl)
{
	int ret = 0;
	static char mipi_sh_hayabusa_cmds_clkchgSetting[6][2] = {
		{0xFF, 0x05 },
		{0x90, 0x00 },
		{0x9B, 0x00 },
		{0xFF, 0x00 },
		{0xD3, 0x00 },
		{0xD4, 0x00 }
	};
	static struct shdisp_dsi_cmd_desc mipi_sh_hayabusa_cmds_clkchg[] = {
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[0]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[1]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[2]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[3]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[4]},
		{SHDISP_DTYPE_DCS_WRITE1, 2, mipi_sh_hayabusa_cmds_clkchgSetting[5]},
	};

	pr_debug("%s: called\n", __func__);

	mipi_sh_hayabusa_cmds_clkchgSetting[1][1] = mipi_clkchg_param->panel.hayabusa.rtn;

	mipi_sh_hayabusa_cmds_clkchgSetting[2][1] = mipi_clkchg_param->panel.hayabusa.gip;

	mipi_sh_hayabusa_cmds_clkchgSetting[4][1] = mipi_clkchg_param->panel.hayabusa.vbp;

	mipi_sh_hayabusa_cmds_clkchgSetting[5][1] = mipi_clkchg_param->panel.hayabusa.vfp;

	ret = mdss_shdisp_host_dsi_tx(1, mipi_sh_hayabusa_cmds_clkchg, ARRAY_SIZE(mipi_sh_hayabusa_cmds_clkchg));

	pr_debug("%s: end ret(%d)\n", __func__, ret);

	return ret;
}
#endif	/* CONFIG_SHDISP_PANEL_ANDY */

/* ----------------------------------------------------------------------- */
/*                                                                         */
/* ----------------------------------------------------------------------- */
static void mdss_diag_mipi_clkchg_param_log(struct mdp_mipi_clkchg_param *mdp_mipi_clkchg_param)
{
	int i;
	pr_debug("[%s]param->host.clock_rate         = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.clock_rate          );
	pr_debug("[%s]param->host.display_width      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_width       );
	pr_debug("[%s]param->host.display_height     = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.display_height      );
	pr_debug("[%s]param->host.hsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.hsync_pulse_width   );
	pr_debug("[%s]param->host.h_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_back_porch        );
	pr_debug("[%s]param->host.h_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.h_front_porch       );
	pr_debug("[%s]param->host.vsync_pulse_width  = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.vsync_pulse_width   );
	pr_debug("[%s]param->host.v_back_porch       = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_back_porch        );
	pr_debug("[%s]param->host.v_front_porch      = %10d\n"  , __func__, mdp_mipi_clkchg_param->host.v_front_porch       );
	pr_debug("[%s]param->host.t_clk_post         = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_post          );
	pr_debug("[%s]param->host.t_clk_pre          = 0x%02X\n", __func__, mdp_mipi_clkchg_param->host.t_clk_pre           );
	for (i = 0; i < NUM_MDP_MIPI_CLKCHG_TIMING; i++) {
		pr_debug("[%s]param->host.timing_ctrl[%02d]    = 0x%02X\n", __func__, i, mdp_mipi_clkchg_param->host.timing_ctrl[i]);
	}

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
	pr_debug("[%s]param->internal_osc            = %10d\n"  , __func__, mdp_mipi_clkchg_param->internal_osc             );
#endif  /* CONFIG_SHDISP_PANEL_HAYABUSA */

	return;
}
#endif /* MDSS_DIAG_MIPI_CLKCHG_ENABLE */

