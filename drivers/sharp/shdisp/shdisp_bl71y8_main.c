/* drivers/sharp/shdisp/shdisp_bl71y8_main.c  (Display Driver)
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
#if !defined(SHDISP_APPSBL) && defined(CONFIG_ARCH_LYNX_GP11D)
#include "./data/shdisp_bl71y8_main_ctrl_gp11d.h"
#else /* !defined(SHDISP_APPSBL) && defined(CONFIG_ARCH_LYNX_GP11D) */
#include "./data/shdisp_bl71y8_main_ctrl.h"
#endif /* !defined(SHDISP_APPSBL) && defined(CONFIG_ARCH_LYNX_GP11D) */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_version_check(unsigned char version);
static int shdisp_bdic_LD_hw_init(void);
static int shdisp_bdic_PD_set_active(void);
static int shdisp_bdic_PD_set_standby(void);

static void shdisp_bdic_PD_hw_reset(void);
static int shdisp_bdic_PD_shutdown(void);
static int shdisp_bdic_PD_set_init(int *bdic_chipver);
static int shdisp_bdic_chk_access(void);
static int shdisp_bdic_get_chipver(int *bdic_chipver);
static int shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size);
static void shdisp_bdic_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj);

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
static int shdisp_bdic_IO_bank_set(unsigned char val);
static int shdisp_bdic_IO_chk_write_reg(unsigned char reg, unsigned char val);
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
static int shdisp_bdic_IO_psals_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_psals_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask);
static int shdisp_bdic_IO_psals_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_psals_burst_write_reg(unsigned char *wval, unsigned char dataNum);
static int shdisp_bdic_IO_psals_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum);

static int shdisp_bdic_PD_slave_transfer(struct shdisp_bdic_i2c_msg *msg);
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

static int shdisp_bdic_register_driver(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static int rst_gpio = 0;

#define SHDISP_BANK_RETRY (3)

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_boot_init(int *chipver)
{
    int ret = 0;
    int ret2 = 0;
    int ver = 0;

    SHDISP_TRACE("in")

    ret = shdisp_bdic_chk_access();
    if (ret == SHDISP_BDIC_IS_EXIST) {
        ret2 = shdisp_bdic_get_chipver(&ver);
        if (ret2 != SHDISP_RESULT_SUCCESS) {
            goto exist_err;
        }
    } else {
        goto exist_err;
    }
    if (chipver != NULL) {
        *chipver = ver;
    }
    SHDISP_TRACE("out")
    return SHDISP_BDIC_IS_EXIST;

exist_err:
    shdisp_IO_API_Host_gpio_free(rst_gpio);
    SHDISP_ERR("bdic not exist.");
    return SHDISP_BDIC_IS_NOT_EXIST;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init_2nd                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_boot_init_2nd(void)
{
    SHDISP_TRACE("in")
    SHDISP_BDIC_REGSET(shdisp_bdic_init2);

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA)
    SHDISP_BDIC_REGSET(shdisp_bdic_init3_cut11);
#else  /* defined(CONFIG_SHDISP_PANEL_HAYABUSA) */
    SHDISP_BDIC_REGSET(shdisp_bdic_init3);
#endif /* defined(CONFIG_SHDISP_PANEL_HAYABUSA) */
    SHDISP_BDIC_REGSET(shdisp_bdic_init4);

    SHDISP_TRACE("out")
    return SHDISP_BDIC_IS_EXIST;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_kerl_init                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_kerl_init(void)
{
    shdisp_bdic_register_driver();
    shdisp_IO_API_Host_gpio_request(rst_gpio, "BL_RST_N");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_shutdown                                                  */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_shutdown(void)
{
    SHDISP_TRACE("in");

    SHDISP_BDIC_REGSET(shdisp_bdic_shutdown);
    shdisp_IO_API_usleep(100);
    shdisp_bdic_PD_shutdown();
    shdisp_IO_API_usleep(100);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_active                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_active(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_set_active();
    SHDISP_TRACE("out");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_standby                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in");
    ret = shdisp_bdic_PD_set_standby();
    SHDISP_TRACE("out");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_correct_proh_val                                          */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_bdic_API_correct_proh_val(unsigned char value)
{
    static struct shdisp_bdic_prohibit_value {
        unsigned char prohibit;
        unsigned char correct;
    } const shdisp_bdic_prohibit_tbl[] = {
        { 0x68, 0x67 },
        { 0x69, 0x6A },
    };

    unsigned int cnt = 0;
    for (; cnt < ARRAY_SIZE(shdisp_bdic_prohibit_tbl); cnt++) {
        if (shdisp_bdic_prohibit_tbl[cnt].prohibit == value) {
            SHDISP_DEBUG(": change to %X from %X",
                shdisp_bdic_prohibit_tbl[cnt].correct, value);
            value = shdisp_bdic_prohibit_tbl[cnt].correct;
            break;
        }
    }
    return value;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_als_sensor_adjust                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_als_sensor_adjust(struct shdisp_photo_sensor_adj *adj)
{
    shdisp_bdic_als_sensor_adjust[0].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[1].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[2].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[3].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[4].data = (adj->als_adjust[1].als_shift & 0x1F);
    shdisp_bdic_als_sensor_adjust[5].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[6].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[7].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[8].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[9].data = (adj->als_adjust[1].als_shift & 0x1F);
    shdisp_bdic_als_sensor_adjust[10].data = (unsigned char)(adj->als_adjust[1].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[11].data = (unsigned char)(adj->als_adjust[1].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[12].data = (unsigned char)(adj->als_adjust[1].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[13].data = (unsigned char)(adj->als_adjust[1].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[14].data = (adj->als_adjust[1].als_shift & 0x1F);

    shdisp_bdic_als_sensor_adjust[15].data = (unsigned char)(adj->als_adjust[0].als_adj0 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[16].data = (unsigned char)(adj->als_adjust[0].als_adj0 >> 8);
    shdisp_bdic_als_sensor_adjust[17].data = (unsigned char)(adj->als_adjust[0].als_adj1 & 0x00FF);
    shdisp_bdic_als_sensor_adjust[18].data = (unsigned char)(adj->als_adjust[0].als_adj1 >> 8);
    shdisp_bdic_als_sensor_adjust[19].data = (adj->als_adjust[0].als_shift & 0x1F);

    shdisp_bdic_IO_bank_set(0x01);
    shdisp_IO_API_delay_us(10 * 1000);
    SHDISP_BDIC_REGSET(shdisp_bdic_als_sensor_adjust);
    shdisp_bdic_IO_bank_set(0x00);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_check_sensor_param                                        */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in,
                                        struct shdisp_photo_sensor_adj *adj_out)
{
    struct shdisp_photo_sensor_adj tmp_adj;
    int err_flg = 0;
    unsigned long chksum;

    SHDISP_TRACE("in")
    memcpy(&tmp_adj, adj_in, sizeof(struct shdisp_photo_sensor_adj));

    SHDISP_INFO(" ---before---");
    SHDISP_INFO(" chksum        = 0x%08x", (unsigned int)tmp_adj.chksum);
    SHDISP_INFO(" status        = 0x%02x", tmp_adj.status);
    SHDISP_INFO(" als_adj0      = 0x%04x", (unsigned short)tmp_adj.als_adjust[0].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x", (unsigned short)tmp_adj.als_adjust[0].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x", tmp_adj.als_adjust[0].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x", tmp_adj.als_adjust[0].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x", tmp_adj.als_adjust[0].ir_offset);
    SHDISP_INFO(" als_adj0      = 0x%04x", (unsigned short)tmp_adj.als_adjust[1].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x", (unsigned short)tmp_adj.als_adjust[1].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x", tmp_adj.als_adjust[1].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x", tmp_adj.als_adjust[1].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x", tmp_adj.als_adjust[1].ir_offset);
    SHDISP_INFO(" key_backlight = 0x%02x", tmp_adj.key_backlight);

    if (tmp_adj.status != SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED) {
        err_flg = 1;
        SHDISP_DEBUG(": status check error.");
    } else if (tmp_adj.als_adjust[0].als_shift > 0x1F) {
        err_flg = 2;
        SHDISP_ERR(": als_shift check error.");
    } else if (tmp_adj.als_adjust[1].als_shift > 0x1F) {
        err_flg = 3;
        SHDISP_ERR(": als_shift check error.");
    } else {
        chksum = (unsigned long)tmp_adj.status
                  + (unsigned long)tmp_adj.key_backlight
                  + tmp_adj.als_adjust[0].als_adj0
                  + tmp_adj.als_adjust[0].als_adj1
                  + (unsigned long)tmp_adj.als_adjust[0].als_shift
                  + (unsigned long)tmp_adj.als_adjust[0].clear_offset
                  + (unsigned long)tmp_adj.als_adjust[0].ir_offset
                  + tmp_adj.als_adjust[1].als_adj0
                  + tmp_adj.als_adjust[1].als_adj1
                  + (unsigned long)tmp_adj.als_adjust[1].als_shift
                  + (unsigned long)tmp_adj.als_adjust[1].clear_offset
                  + (unsigned long)tmp_adj.als_adjust[1].ir_offset;
        if (tmp_adj.chksum != chksum) {
            err_flg = 9;
            SHDISP_ERR(": chksum check error.");
            SHDISP_ERR(" chksum = 0x%08x", (unsigned int)tmp_adj.chksum);
            SHDISP_ERR(" result = 0x%08x", (unsigned int)chksum);
        }
    }
    if (err_flg != 0) {
        shdisp_bdic_set_default_sensor_param(&tmp_adj);
        tmp_adj.status = (unsigned char)err_flg;
    }
    memcpy(adj_out, &tmp_adj, sizeof(struct shdisp_photo_sensor_adj));

    SHDISP_INFO("---after---");
    SHDISP_INFO(" chksum        = 0x%08x", (unsigned int)tmp_adj.chksum);
    SHDISP_INFO(" status        = 0x%02x", tmp_adj.status);
    SHDISP_INFO(" als_adj0      = 0x%04x", (unsigned short)tmp_adj.als_adjust[0].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x", (unsigned short)tmp_adj.als_adjust[0].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x", tmp_adj.als_adjust[0].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x", tmp_adj.als_adjust[0].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x", tmp_adj.als_adjust[0].ir_offset);
    SHDISP_INFO(" als_adj0      = 0x%04x", (unsigned short)tmp_adj.als_adjust[1].als_adj0);
    SHDISP_INFO(" als_adj1      = 0x%04x", (unsigned short)tmp_adj.als_adjust[1].als_adj1);
    SHDISP_INFO(" als_shift     = 0x%02x", tmp_adj.als_adjust[1].als_shift);
    SHDISP_INFO(" clear_offset  = 0x%02x", tmp_adj.als_adjust[1].clear_offset);
    SHDISP_INFO(" ir_offset     = 0x%02x", tmp_adj.als_adjust[1].ir_offset);
    SHDISP_INFO(" key_backlight = 0x%02x", tmp_adj.key_backlight);
    SHDISP_TRACE("out")

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_seq_regset                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size)
{
    int ret = 0;

    ret = shdisp_bdic_seq_regset(regtable, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_write_reg                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_write_reg(reg,val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_multi_write_reg                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_write_reg(reg, wval, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_read_reg                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_multi_read_reg                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_set_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_set_bit_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_clr_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_clr_bit_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_bank_set                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_bank_set(unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_bank_set(val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_msk_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret = 0;

    ret = shdisp_bdic_IO_msk_bit_reg(reg, val, msk);
    return ret;
}

#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_i2c_transfer                                           */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_i2c_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret = 0;

    ret = shdisp_bdic_PD_slave_transfer(msg);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_psals_write_reg                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_psals_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_psals_write_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_psals_read_reg                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_psals_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_psals_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_psals_msk_bit_reg                                      */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_psals_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask)
{
    int ret = 0;

    ret = shdisp_bdic_IO_psals_msk_bit_reg(reg, val, mask);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_psals_burst_write_reg                                  */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_psals_burst_write_reg(unsigned char *wval, unsigned char dataNum)
{
    int ret = 0;

    ret = shdisp_bdic_IO_psals_burst_write_reg(wval, dataNum);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_psals_burst_read_reg                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_psals_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum)
{
    int ret = 0;

    ret = shdisp_bdic_IO_psals_burst_read_reg(reg, rval, dataNum);
    return ret;
}
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_chk_access                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_chk_access(void)
{
    int ret;

    ret = shdisp_bdic_LD_hw_init();
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_IO_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
        return SHDISP_BDIC_IS_NOT_EXIST;
    }
    return SHDISP_BDIC_IS_EXIST;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_get_chipver                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_get_chipver(int *bdic_chipver)
{
    int ret = SHDISP_RESULT_SUCCESS;
    ret = shdisp_bdic_PD_set_init(bdic_chipver);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_version_check                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_version_check(unsigned char version)
{
    if ((version & SHDISP_BDIC_VERSION71) != SHDISP_BDIC_VERSION71) {
        return SHDISP_RESULT_FAILURE;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_hw_init                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_LD_hw_init(void)
{
    int ret;
    unsigned char version;

    shdisp_bdic_PD_hw_reset();

    ret = shdisp_IO_API_bdic_i2c_read(BDIC_REG_VERSION, &version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_bdic_LD_version_check(version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* Phygical Driver                                                           */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_hw_reset                                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_hw_reset(void)
{
    shdisp_IO_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_IO_API_delay_us(100);
    shdisp_IO_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_HIGH);
    shdisp_IO_API_delay_us(100);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_shutdown                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_shutdown(void)
{
    shdisp_IO_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_IO_API_Host_gpio_free(rst_gpio);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_init                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_init(int *bdic_chipver)
{
    int ret;
    unsigned char version = 0;

    SHDISP_TRACE("in");

    SHDISP_BDIC_REGSET(shdisp_bdic_init1);
    ret = shdisp_bdic_IO_read_reg(BDIC_REG_VERSION, &version);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_FAILURE;
    }
    if (bdic_chipver != NULL) {
        *bdic_chipver = (int)SHDISP_BDIC_GET_CHIPVER(version);
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_init12);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/* shdisp_bdic_seq_regset                                                    */
/*---------------------------------------------------------------------------*/
static int shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size)
{
    int i;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t *tbl;
    unsigned char top_addr_bdic = 0x00;
    int cnt_bdic = 0;
    unsigned char bBuf_bdic[16];
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
    int cnt_als = 0;
    unsigned char bBuf_als[16];
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

    tbl = (shdisp_bdicRegSetting_t *)regtable;
    for (i = 0; i < size; i++) {
        if (((cnt_bdic > 0) && (tbl->flg != SHDISP_BDIC_STRM)) || (cnt_bdic == sizeof(bBuf_bdic))) {
            ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            cnt_bdic = 0;
            top_addr_bdic = 0x00;
        }
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
        if (((cnt_als > 0) && (tbl->flg != SHDISP_ALS_STRM)) || (cnt_als == sizeof(bBuf_als))) {
            ret = shdisp_bdic_IO_psals_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
            cnt_als = 0;
        }
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */
        switch (tbl->flg) {
        case SHDISP_BDIC_STR:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CHKWR:
            ret = shdisp_bdic_IO_chk_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
            ret = shdisp_bdic_IO_set_bit_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CLR:
            ret = shdisp_bdic_IO_clr_bit_reg(tbl->addr, tbl->mask);
            break;
        case SHDISP_BDIC_RMW:
            ret = shdisp_bdic_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BDIC_STRM:
            if (cnt_bdic == 0) {
                top_addr_bdic = tbl->addr;
            }
            bBuf_bdic[cnt_bdic] = tbl->data;
            cnt_bdic++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            break;
        case SHDISP_BDIC_BANK:
            ret = shdisp_bdic_IO_bank_set(tbl->data);
            break;
        case SHDISP_BDIC_WAIT:
            ret = SHDISP_RESULT_SUCCESS;
            break;
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
        case SHDISP_ALS_STR:
            ret = shdisp_bdic_IO_psals_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_ALS_STRM:
        case SHDISP_ALS_STRMS:
            if (cnt_als == 0) {
                bBuf_als[cnt_als] = tbl->addr;
                cnt_als++;
            }
            bBuf_als[cnt_als] = tbl->data;
            cnt_als++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_psals_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
            }
            break;
        case SHDISP_ALS_RMW:
            ret = shdisp_bdic_IO_psals_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic R/W Error addr=%02X, data=%02X, mask=%02X", tbl->addr, tbl->data, tbl->mask);
            return ret;
        }
        if (tbl->wait > 0) {
            if ((cnt_bdic > 0) && (tbl->flg == SHDISP_BDIC_STRM)) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
            if ((cnt_als > 0) && (tbl->flg == SHDISP_ALS_STRM)) {
                ret = shdisp_bdic_IO_psals_burst_write_reg(bBuf_als, (unsigned char)cnt_als);
                cnt_als = 0;
            }
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */
            shdisp_IO_API_delay_us(tbl->wait);
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_active                                                 */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_active(void)
{
    SHDISP_TRACE("in");
    SHDISP_BDIC_REGSET(shdisp_bdic_active);
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_standby                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_set_standby(void)
{
    SHDISP_TRACE("in");
    SHDISP_BDIC_REGSET(shdisp_bdic_standby);
    SHDISP_TRACE("out");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_set_default_sensor_param                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_set_default_sensor_param(struct shdisp_photo_sensor_adj *tmp_adj)
{
    unsigned short tmp_param1;
    unsigned short tmp_param2;

    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ0_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ0_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ1_H_DEFAULT_A << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ1_L_DEFAULT_A;
    tmp_adj->als_adjust[0].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[0].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_A;
    tmp_adj->als_adjust[0].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_A;
    tmp_adj->als_adjust[0].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_A;

    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ0_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ0_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj0     = tmp_param1 | tmp_param2;
    tmp_param1 = ((unsigned short)BDIC_REG_ALS_ADJ1_H_DEFAULT_B << 8);
    tmp_param2 = (unsigned short)BDIC_REG_ALS_ADJ1_L_DEFAULT_B;
    tmp_adj->als_adjust[1].als_adj1     = tmp_param1 | tmp_param2;
    tmp_adj->als_adjust[1].als_shift    = BDIC_REG_ALS_SHIFT_DEFAULT_B;
    tmp_adj->als_adjust[1].clear_offset = BDIC_REG_CLEAR_OFFSET_DEFAULT_B;
    tmp_adj->als_adjust[1].ir_offset    = BDIC_REG_IR_OFFSET_DEFAULT_B;

    return;
}

/* ------------------------------------------------------------------------- */
/* Input/Output                                                              */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;

#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> ");
        return SHDISP_RESULT_SUCCESS;
    }
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

    ret = shdisp_IO_API_bdic_i2c_write(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_IO_API_bdic_i2c_write.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_IO_API_bdic_i2c_write.");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret;

#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> ");
        return SHDISP_RESULT_SUCCESS;
    }
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

    ret = shdisp_IO_API_bdic_i2c_multi_write(reg, wval, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_IO_API_bdic_i2c_multi_write.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_IO_API_bdic_i2c_multi_write.");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> ");
        return SHDISP_RESULT_SUCCESS;
    }
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

    ret = shdisp_IO_API_bdic_i2c_read(reg, val);
    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;
    int maxreg;

    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.");
        return SHDISP_RESULT_FAILURE;
    }

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).", size);
        return SHDISP_RESULT_FAILURE;
    }

    maxreg = (int)reg + (size - 1);
    if (maxreg > BDIC_REG_BANKSEL) {
        SHDISP_ERR("<OTHER> register address overflow.");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_IO_API_bdic_i2c_multi_read(reg, val, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_IO_API_bdic_i2c_multi_read.");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_IO_API_bdic_i2c_multi_read.");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_IO_API_bdic_i2c_mask_write(reg, val, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_IO_API_bdic_i2c_mask_write(reg, 0x00, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_bank_set                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_bank_set(unsigned char val)
{
    int ret;

    ret = shdisp_bdic_IO_write_reg(BDIC_REG_BANKSEL, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret;

    ret = shdisp_IO_API_bdic_i2c_mask_write(reg, val, msk);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_chk_write_reg                                              */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_chk_write_reg(unsigned char reg, unsigned char val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned char read_val = 0;
    int bank_retry = 0;

    for (bank_retry = 0; bank_retry < SHDISP_BANK_RETRY; bank_retry++) {
        ret = shdisp_IO_API_bdic_i2c_write(reg, val);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> i2c_write.")
            return ret;
        }

        ret = shdisp_IO_API_bdic_i2c_dummy_read(reg, &read_val);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> i2c_read.")
            return ret;
        }

        if (val == read_val) {
            return SHDISP_RESULT_SUCCESS;
        }
        SHDISP_ERR("<BANK1_ERROR> addr:0x%02X, write_data:0x%02X, read_data:0x%02X, retry:%d", reg, val, read_val, bank_retry);
    }

    SHDISP_ERR("<RESULT_FAILURE> chk_write.")
    return SHDISP_RESULT_FAILURE;
}


#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_psals_write_reg                                            */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_psals_write_reg(unsigned char reg, unsigned char val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[2];

    wbuf[0] = reg;
    wbuf[1] = val;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = 2;
    msg.rlen = 0;
    msg.wbuf = &wbuf[0];
    msg.rbuf = NULL;

    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_psals_read_reg                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_psals_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];
    unsigned char rbuf[1];

    wbuf[0] = reg;
    rbuf[0] = 0x00;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = 1;
    msg.wbuf = &wbuf[0];
    msg.rbuf = &rbuf[0];

    ret = shdisp_bdic_PD_slave_transfer(&msg);
    if (ret == SHDISP_RESULT_SUCCESS) {
        *val = rbuf[0];
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_psals_msk_bit_reg                                          */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_psals_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask)
{
    int ret;
    unsigned char rval;
    unsigned char wval;

    ret = shdisp_bdic_IO_psals_read_reg(reg, &rval);
    if (ret != SHDISP_RESULT_SUCCESS) {
        return ret;
    }

    wval = (rval & ~mask) | (val & mask);

    if (rval == wval) {
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_bdic_IO_psals_write_reg(reg, wval);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_psals_burst_write_reg                                      */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_psals_burst_write_reg(unsigned char *wval, unsigned char dataNum)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;

    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_W;
    msg.wlen = dataNum;
    msg.rlen = 0;
    msg.wbuf = wval;
    msg.rbuf = NULL;
    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_psals_burst_read_reg                                       */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_IO_psals_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum)
{
    int ret = SHDISP_RESULT_SUCCESS;
    struct shdisp_bdic_i2c_msg msg;
    unsigned char wbuf[1];

    wbuf[0] = reg;
    msg.addr = SHDISP_BDIC_SENSOR_SLAVE_ADDR;
    msg.mode = SHDISP_BDIC_I2C_M_R;
    msg.wlen = 1;
    msg.rlen = dataNum;
    msg.wbuf = &wbuf[0];
    msg.rbuf = rval;

    ret = shdisp_bdic_PD_slave_transfer(&msg);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_slave_transfer                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_slave_transfer(struct shdisp_bdic_i2c_msg *msg)
{
    int ret;
    switch (msg->mode) {
    case SHDISP_BDIC_I2C_M_W:
        ret = shdisp_IO_API_Host_i2c_send(msg->addr, (unsigned char *)msg->wbuf, msg->wlen);
        break;
    case SHDISP_BDIC_I2C_M_R:
        ret = shdisp_IO_API_Host_i2c_recv(msg->addr, (unsigned char *)msg->wbuf, msg->wlen, msg->rbuf, msg->rlen);
        break;
    case SHDISP_BDIC_I2C_M_R_MODE1:
    case SHDISP_BDIC_I2C_M_R_MODE2:
    case SHDISP_BDIC_I2C_M_R_MODE3:
        ret = SHDISP_RESULT_SUCCESS;
        break;
    default:
        SHDISP_ERR("<INVALID_VALUE> msg->mode(%d).", msg->mode);
        ret = SHDISP_RESULT_FAILURE;
        break;
    }
    return ret;
}
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_probe                                                         */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    int rc = SHDISP_RESULT_SUCCESS;

    SHDISP_TRACE("in pdev = 0x%p", pdev);

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!");
            rc = 0;
            goto probe_done;
        } else {
            shdisp_SYS_API_set_irq_port(res->start, pdev);
        }
        if (&(pdev->dev) != NULL) {
            rst_gpio = of_get_named_gpio(pdev->dev.of_node, "bdic_rst_gpio", 0);
            if (!gpio_is_valid(rst_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    } else {
        SHDISP_ERR("pdev is NULL");
    }
probe_done:
    SHDISP_TRACE("out rc = %d", rc);

    return rc;
#else   /* CONFIG_OF */
    return SHDISP_RESULT_SUCCESS;
#endif  /* CONFIG_OF */
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_bdic_dt_match[] = {
    { .compatible = "sharp,shdisp_bdic", },
    {}
};
#else   /* CONFIG_OF */
#define shdisp_bdic_dt_match NULL
#endif  /* CONFIG_OF */

static struct platform_driver shdisp_bdic_driver = {
    .probe = shdisp_bdic_probe,
    .remove = NULL,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_bdic",
        .of_match_table = shdisp_bdic_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_register_driver                                               */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_register_driver(void)
{
    return platform_driver_register(&shdisp_bdic_driver);
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
