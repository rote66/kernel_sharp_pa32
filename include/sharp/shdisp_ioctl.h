/* include/sharp/shdisp_ioctl.h  (Display Driver)
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

#ifndef SHDISP_IOCTL_H
#define SHDISP_IOCTL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_context_def.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_PHOTO_SENSOR_TYPE_APP,
    SHDISP_PHOTO_SENSOR_TYPE_LUX,
    SHDISP_PHOTO_SENSOR_TYPE_CAMERA,
    SHDISP_PHOTO_SENSOR_TYPE_KEYLED,
    SHDISP_PHOTO_SENSOR_TYPE_DIAG,
    SHDISP_PHOTO_SENSOR_TYPE_SENSORHUB,
    NUM_SHDISP_PHOTO_SENSOR_TYPE
};

enum {
    SHDISP_DIAG_COG_ID_NONE,
    SHDISP_DIAG_COG_ID_MASTER,
    SHDISP_DIAG_COG_ID_SLAVE,
    SHDISP_DIAG_COG_ID_BOTH,
    NUM_SHDISP_DIAG_COG_ID
};

enum {
    SHDISP_MAIN_BKL_AUTO_OFF,
    SHDISP_MAIN_BKL_AUTO_ON,
    SHDISP_MAIN_BKL_AUTO_ECO_ON,
    NUM_SHDISP_MAIN_BKL_AUTO
};

enum {
    SHDISP_MAIN_BKL_DTV_OFF,
    SHDISP_MAIN_BKL_DTV_ON,
    NUM_SHDISP_MAIN_BKL_DTV
};

enum {
    SHDISP_MAIN_BKL_EMG_OFF,
    SHDISP_MAIN_BKL_EMG_ON_LEVEL0,
    SHDISP_MAIN_BKL_EMG_ON_LEVEL1,
    NUM_SHDISP_MAIN_BKL_EMG
};

enum {
    SHDISP_MAIN_BKL_LOWBKL_MODE_OFF,
    SHDISP_MAIN_BKL_LOWBKL_MODE_ON,
    NUM_SHDISP_MAIN_BKL_LOWBKL_MODE
};


enum {
    SHDISP_MAIN_DISP_DRIVE_FREQ_DEFAULT,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_B,
    SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_C,
    NUM_SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE
};

#if defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA)
enum {
    SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_A,
    SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE_B,
    NUM_SHDISP_MAIN_DISP_INTERNAL_OSC_TYPE
};
#endif  /* defined(CONFIG_SHDISP_PANEL_HAYABUSA) || defined(USER_CONFIG_SHDISP_PANEL_HAYABUSA) */

enum {
    SHDISP_PHOTO_SENSOR_DISABLE,
    SHDISP_PHOTO_SENSOR_ENABLE,
    NUM_SHDISP_PHOTO_SENSOR
};

enum {
    SHDISP_IRQ_NO_MASK,
    SHDISP_IRQ_MASK,
    NUM_SHDISP_IRQ_SWITCH
};

enum {
    SHDISP_MAIN_BKL_CHG_OFF,
    SHDISP_MAIN_BKL_CHG_ON,
    SHDISP_MAIN_BKL_CHG_ON_BRIGHT,
    NUM_SHDISP_MAIN_BKL_CHG
};

#define SHDISP_OPT_CHANGE_INT_1             (0x01)
#define SHDISP_OPT_CHANGE_INT_2             (0x02)

#define SHDISP_REG_WRITE        (0x01)
#define SHDISP_SAVE_VALUE       (0x02)
#define SHDISP_SAVE_VALUE_LOW   (0x04)
#define SHDISP_RESET_VALUE      (0x08)

#define SHDISP_LCDDR_BUF_MAX    (64)

enum {
    SHDISP_TRV_PARAM_OFF,
    SHDISP_TRV_PARAM_ON
};

enum {
    ILLUMI_FRAME_FIRST = 0,
    ILLUMI_FRAME_SECOND,
    ILLUMI_FRAME_THIRD,
    ILLUMI_FRAME_MAX
};

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
struct shdisp_diag_bdic_reg {
    unsigned char reg;
    unsigned char val;
};

struct shdisp_diag_bdic_reg_multi {
    unsigned char reg;
    unsigned char val[8];
    unsigned char size;
};

struct shdisp_photo_sensor_val {
    unsigned short value;
    unsigned int   lux;
    int mode;
    int result;
};

struct shdisp_photo_sensor_power_ctl {
    int type;
    int power;
};

struct shdisp_lcddr_reg {
    unsigned char address;
    unsigned char size;
    unsigned char buf[SHDISP_LCDDR_BUF_MAX];
    int      cog;
};

struct shdisp_main_bkl_auto {
    int mode;
    int param;
};

struct shdisp_ave_ado {
    unsigned char  als_range;
    unsigned short ave_als0;
    unsigned short ave_als1;
    unsigned short ave_ado;
};

struct shdisp_photo_sensor_raw_val {
    unsigned short clear;
    unsigned short ir;
    int result;
};

struct shdisp_photo_sensor_trigger {
    unsigned short level;
    unsigned short side;
    unsigned short en_hi_edge;
    unsigned short en_lo_edge;
    unsigned short enable;
};

struct shdisp_photo_sensor_int_trigger {
    struct shdisp_photo_sensor_trigger trigger1;
    struct shdisp_photo_sensor_trigger trigger2;
    int type;
    int result;
};

struct shdisp_light_info {
    unsigned int lux;
    unsigned short level;
    unsigned int clear_ir_rate;
    int result;
};

struct shdisp_led_auto_low_mode_param {
    int enable;
};

struct shdisp_trv_param {
    int            request;
    int            strength;
    int            adjust;
};

struct shdisp_rgb {
    unsigned int red;
    unsigned int green;
    unsigned int blue;
};

struct shdisp_illumi_triple_color {
    struct shdisp_rgb colors[ILLUMI_FRAME_MAX];
    int count;
};

struct shdisp_lut_info {
    unsigned short status;
    struct shdisp_gc_lut lut;
};

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_GET_CONTEXT                        _IOR  (SHDISP_IOC_MAGIC,  0, struct shdisp_to_user_context)
#define SHDISP_IOCTL_TRI_LED_SET_COLOR                  _IOW  (SHDISP_IOC_MAGIC,  2, struct shdisp_tri_led)
#define SHDISP_IOCTL_BDIC_WRITE_REG                     _IOW  (SHDISP_IOC_MAGIC,  3, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_BDIC_READ_REG                      _IOWR (SHDISP_IOC_MAGIC,  4, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_GET_LUX                            _IOWR (SHDISP_IOC_MAGIC,  5, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL               _IOW  (SHDISP_IOC_MAGIC,  8, struct shdisp_photo_sensor_power_ctl)
#define SHDISP_IOCTL_LCDDR_WRITE_REG                    _IOW  (SHDISP_IOC_MAGIC,  9, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_LCDDR_READ_REG                     _IOWR (SHDISP_IOC_MAGIC, 10, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_SET_FLICKER_PARAM                  _IOW  (SHDISP_IOC_MAGIC, 11, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_FLICKER_PARAM                  _IOWR (SHDISP_IOC_MAGIC, 12, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_BKL_SET_AUTO_MODE                  _IOW  (SHDISP_IOC_MAGIC, 13, struct shdisp_main_bkl_auto)
#define SHDISP_IOCTL_BDIC_MULTI_READ_REG                _IOWR (SHDISP_IOC_MAGIC, 14, struct shdisp_diag_bdic_reg_multi)
#define SHDISP_IOCTL_BKL_SET_DTV_MODE                   _IOW  (SHDISP_IOC_MAGIC, 15, int)
#define SHDISP_IOCTL_BKL_SET_EMG_MODE                   _IOW  (SHDISP_IOC_MAGIC, 16, int)
#define SHDISP_IOCTL_BKL_SET_LOWBKL_MODE                _IOW  (SHDISP_IOC_MAGIC, 24, int)
#define SHDISP_IOCTL_BKL_SET_CHG_MODE                   _IOW  (SHDISP_IOC_MAGIC, 26, int)
#define SHDISP_IOCTL_GET_FLICKER_LOW_PARAM              _IOWR (SHDISP_IOC_MAGIC, 27, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_LCDC_SET_DRIVE_FREQ                _IOW  (SHDISP_IOC_MAGIC, 28, struct shdisp_main_drive_freq)
#define SHDISP_IOCTL_SET_GMMTABLE_AND_VOLTAGE           _IOW  (SHDISP_IOC_MAGIC, 29, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_GET_GMMTABLE_AND_VOLTAGE           _IOWR (SHDISP_IOC_MAGIC, 30, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_SET_GMM                            _IOW  (SHDISP_IOC_MAGIC, 31, struct shdisp_diag_gamma)
#define SHDISP_IOCTL_GET_AVE_ADO                        _IOWR (SHDISP_IOC_MAGIC, 32, struct shdisp_ave_ado)
#define SHDISP_IOCTL_GET_ALS                            _IOWR (SHDISP_IOC_MAGIC, 35, struct shdisp_photo_sensor_raw_val)
#define SHDISP_IOCTL_SET_IRQ_MASK                       _IOW  (SHDISP_IOC_MAGIC, 37, int)
#define SHDISP_IOCTL_VCOM_TRACKING                      _IOW  (SHDISP_IOC_MAGIC, 41, int)
#define SHDISP_IOCTL_SET_ALSINT                         _IOWR (SHDISP_IOC_MAGIC, 42, struct shdisp_photo_sensor_int_trigger)
#define SHDISP_IOCTL_GET_ALSINT                         _IOWR (SHDISP_IOC_MAGIC, 43, struct shdisp_photo_sensor_int_trigger)
#define SHDISP_IOCTL_GET_LIGHT_INFO                     _IOWR (SHDISP_IOC_MAGIC, 44, struct shdisp_light_info)
#define SHDISP_IOCTL_SET_MFR                            _IOW  (SHDISP_IOC_MAGIC, 45, int)
#define SHDISP_IOCTL_SET_LED_AUTO_LOW_MODE              _IOW  (SHDISP_IOC_MAGIC, 46, struct shdisp_led_auto_low_mode_param)
#define SHDISP_IOCTL_SET_TRV_PARAM                      _IOW  (SHDISP_IOC_MAGIC, 47, struct shdisp_trv_param)
#define SHDISP_IOCTL_SET_ILLUMI_TRI_COLOR               _IOW  (SHDISP_IOC_MAGIC, 48, struct shdisp_illumi_triple_color)
#define SHDISP_IOCTL_GET_LUT_INFO                       _IOR  (SHDISP_IOC_MAGIC, 49, struct shdisp_lut_info)

#endif /* SHDISP_IOCTL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
