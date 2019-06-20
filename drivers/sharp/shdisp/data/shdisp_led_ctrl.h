/* drivers/sharp/shdisp/data/shdisp_led_ctrl.h  (Display Driver)
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

#ifndef SHDISP_LED_CTRL_H
#define SHDISP_LED_CTRL_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_bl71y8_cmn.h"
#if defined(SHDISP_PA)
#include "shdisp_bl71y8_led_pa.h"
#elif defined(SHDISP_DL)
#include "shdisp_bl71y8_led_dl.h"
#elif defined(SHDISP_AL)
#include "shdisp_bl71y8_led_al.h"
#else  /* SHDISP_PA */
#include "shdisp_bl71y8_led_default.h"
#endif /* SHDISP_PA */
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
#define SHDISP_LED_FIX_ONR        (1)
#define SHDISP_LED_FIX_ONG        (2)
#define SHDISP_LED_FIX_ONB        (3)

static const shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_CH0_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH1_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH2_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
#ifndef SHDISP_COLOR_LED_TWIN
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0}
#endif /* SHDISP_COLOR_LED_TWIN */
};

#ifndef SHDISP_COLOR_LED_TWIN
static shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_TIMER,               SHDISP_BDIC_RMW,    0xF7,                       0xF7,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0},
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x01,                       0xF3,   6000}
};
#ifdef SHDISP_ANIME_COLOR_LED
static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on_triple_color[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x07,                       0x07,      0},
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_RMW,    0x01,                       0xF3,   6000}
};
#endif  /* SHDISP_ANIME_COLOR_LED */
#endif /* SHDISP_COLOR_LED_TWIN */

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_enable[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_SET,    0x10,                       0x10,      0}
};

#ifdef SHDISP_ANIME_COLOR_LED
static const shdisp_bdicRegSetting_t shdisp_bdic_led_high_speed_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_standard_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_breath_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_long_breath_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_wave_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_flash_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_aurora_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_rainbow_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x46,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_emopattern_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_illumi_triple_color_1st[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_TIMER,               SHDISP_BDIC_RMW,    0x30,                       0xF0,      0}
};

#ifdef SHDISP_EXTEND_COLOR_LED
static const shdisp_bdicRegSetting_t shdisp_bdic_led_pattern1_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_pattern2_on[] = {
     {BDIC_REG_CH0_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH0_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH1_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET1,            SHDISP_BDIC_STRM,   0x46,                       0xFF,      0}
    ,{BDIC_REG_CH2_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};
#endif  /* SHDISP_EXTEND_COLOR_LED */

#ifdef SHDISP_COLOR_LED_TWIN
static const shdisp_bdicRegSetting_t shdisp_bdic_led_high_speed_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x41,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x30,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_standard_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x42,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x22,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_breath_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_long_breath_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_wave_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_flash_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x00,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_aurora_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_rainbow_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x02,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x06,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x07,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x46,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x31,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_emopattern_on_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_STRM,   0x03,                       0xFF,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_STRM,   0x05,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_illumi_triple_color_1st_twin[] = {
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH3_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_CH4_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH4_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_CH5_SET1,            SHDISP_BDIC_RMW,    0x02,                       0x6F,      0}
    ,{BDIC_REG_CH5_SET2,            SHDISP_BDIC_RMW,    0x17,                       0xF7,      0}
    ,{BDIC_REG_TIMER2,              SHDISP_BDIC_RMW,    0x30,                       0xF0,      0}
};
#endif  /* SHDISP_COLOR_LED_TWIN */
#endif  /* SHDISP_ANIME_COLOR_LED */

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
#ifdef SHDISP_COLOR_LED_TWIN
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_STR,    0x00,                       0xFF,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x3F,   5500}
#else  /* SHDISP_COLOR_LED_TWIN */
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_CLR,    0x00,                       0xF3,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
#endif /* SHDISP_COLOR_LED_TWIN */
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_lposc_disable[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM1,             SHDISP_BDIC_CLR,    0x00,                       0x10,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_led_off_fix[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
#ifdef SHDISP_COLOR_LED_TWIN
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x3F,   5500}
#else  /* SHDISP_COLOR_LED_TWIN */
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_CLR,    0x00,                       0x07,   5500}
#endif /* SHDISP_COLOR_LED_TWIN */
};

#ifdef SHDISP_COLOR_LED_TWIN
static const shdisp_bdicRegSetting_t shdisp_bdic_led_fix_on_twin[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_CH3_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH4_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH5_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH3_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH4_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH5_B,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH3_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH4_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH5_C,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH3_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH4_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_CH5_SET1,            SHDISP_BDIC_SET,    0x20,                       0x20,      0},
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x3F,                       0x3F,      0}
};

static shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on_twin[] = {
     {BDIC_REG_BANKSEL,             SHDISP_BDIC_BANK,   0x00,                       0x00,      0},
     {BDIC_REG_TIMER,               SHDISP_BDIC_RMW,    0xF7,                       0xF7,      0},
     {BDIC_REG_TIMER2,              SHDISP_BDIC_RMW,    0xF7,                       0xF7,      0},
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x3F,                       0x3F,      0},
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_STR,    0x05,                       0xFF,   6000}
};

#ifdef SHDISP_ANIME_COLOR_LED
static const shdisp_bdicRegSetting_t shdisp_bdic_led_ani_on_triple_color_twin[] = {
     {BDIC_REG_SYSTEM5,             SHDISP_BDIC_SET,    0x3F,                       0x3F,      0},
     {BDIC_REG_SYSTEM7,             SHDISP_BDIC_STR,    0x05,                       0xFF,   6000}
};
#endif  /* SHDISP_ANIME_COLOR_LED */
#endif /* SHDISP_COLOR_LED_TWIN */

#ifdef SHDISP_SYSFS_LED
static const shdisp_bdicRegSetting_t shdisp_bdic_led_current[] = {
     {BDIC_REG_CH0_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH1_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH2_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
};

#ifdef SHDISP_COLOR_LED_TWIN
static const shdisp_bdicRegSetting_t shdisp_bdic_led_current_twin[] = {
     {BDIC_REG_CH3_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH4_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
     {BDIC_REG_CH5_A,               SHDISP_BDIC_STRM,   0x00,                       0xFF,      0},
};
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */

#endif  /* SHDISP_LED_CTRL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
