/* include/sharp/shdisp_kerl.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_KERN_H
#define SHDISP_KERN_H


/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_context_def.h"
#include "shdisp_define.h"
#include "shdisp_ioctl.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#ifndef TRUE
#define TRUE    (1)
#endif  /* TRUE */

#ifndef FALSE
#define FALSE   (0)
#endif  /* FALSE */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_BDIC_I2C_M_W,
    SHDISP_BDIC_I2C_M_R,
    SHDISP_BDIC_I2C_M_R_MODE1,
    SHDISP_BDIC_I2C_M_R_MODE2,
    SHDISP_BDIC_I2C_M_R_MODE3,
    NUM_SHDISP_BDIC_I2C_M
};

enum {
    SHDISP_PROX_SENSOR_POWER_OFF,
    SHDISP_PROX_SENSOR_POWER_ON,
    SHDISP_PROX_SENSOR_BGMODE_ON,
    SHDISP_PROX_SENSOR_BGMODE_OFF,
    NUM_SHDISP_PROX_SENSOR_POWER
};

enum {
    SHDISP_RESULT_SUCCESS,
    SHDISP_RESULT_FAILURE,
    SHDISP_RESULT_FAILURE_I2C_TMO,
#ifdef SHDISP_ALS_INT
    SHDISP_RESULT_ALS_INT_OFF,
#endif /* SHDISP_ALS_INT */
    NUM_SHDISP_RESULT
};

enum {
    SHDISP_TRI_LED_INTERVAL_NONE,
    SHDISP_TRI_LED_INTERVAL_1S,
    SHDISP_TRI_LED_INTERVAL_2S,
    SHDISP_TRI_LED_INTERVAL_3S,
    SHDISP_TRI_LED_INTERVAL_4S,
    SHDISP_TRI_LED_INTERVAL_5S,
    SHDISP_TRI_LED_INTERVAL_6S,
    SHDISP_TRI_LED_INTERVAL_7S,
    SHDISP_TRI_LED_INTERVAL_8S,
    SHDISP_TRI_LED_INTERVAL_9S,
    SHDISP_TRI_LED_INTERVAL_10S,
    SHDISP_TRI_LED_INTERVAL_11S,
    SHDISP_TRI_LED_INTERVAL_12S,
    SHDISP_TRI_LED_INTERVAL_13S,
    SHDISP_TRI_LED_INTERVAL_14S,
    SHDISP_TRI_LED_INTERVAL_15S,
    NUM_SHDISP_TRI_LED_INTERVAL
};

enum {
    SHDISP_LEDC_RGB_MODE_NORMAL,
    SHDISP_LEDC_RGB_MODE_PATTERN1,
    SHDISP_LEDC_RGB_MODE_PATTERN2,
    SHDISP_LEDC_RGB_MODE_PATTERN3,
    SHDISP_LEDC_RGB_MODE_PATTERN4,
    SHDISP_LEDC_RGB_MODE_PATTERN5,
    SHDISP_LEDC_RGB_MODE_PATTERN6,
    SHDISP_LEDC_RGB_MODE_PATTERN7,
    SHDISP_LEDC_RGB_MODE_PATTERN8,
    SHDISP_LEDC_RGB_MODE_PATTERN9,
    SHDISP_LEDC_RGB_MODE_PATTERN10,
    SHDISP_LEDC_RGB_MODE_PATTERN11,
    SHDISP_LEDC_RGB_MODE_PATTERN12,
    SHDISP_LEDC_RGB_MODE_ANIMATION1,
    SHDISP_LEDC_RGB_MODE_ANIMATION2,
    SHDISP_LEDC_RGB_MODE_ANIMATION3,
    SHDISP_LEDC_RGB_MODE_ANIMATION4,
    SHDISP_LEDC_RGB_MODE_ANIMATION5,
    SHDISP_LEDC_RGB_MODE_ANIMATION6,
    SHDISP_LEDC_RGB_MODE_ANIMATION7,
    SHDISP_LEDC_RGB_MODE_ANIMATION8,
    SHDISP_LEDC_RGB_MODE_ANIMATION9,
    SHDISP_LEDC_RGB_MODE_ANIMATION10,
    NUM_SHDISP_LEDC_RGB_MODE
};

enum {
    SHDISP_PIERCE_STATE_REMOVE = 0,
    SHDISP_PIERCE_STATE_INSERT
};

enum {
    SHDISP_PRE_SHUTDOWN,
    SHDISP_POST_SHUTDOWN,
};

struct shdisp_procfs {
    int id;
    int par[4];
};

struct shdisp_bdic_i2c_msg {
    unsigned short addr;
    unsigned char mode;
    unsigned char wlen;
    unsigned char rlen;
    const unsigned char *wbuf;
    unsigned char *rbuf;
};

struct shdisp_subscribe {
    int   irq_type;
    void (*callback)(void);
};

struct shdisp_prox_params {
    unsigned int threshold_low;
    unsigned int threshold_high;
};

struct shdisp_main_dbc {
    int mode;
    int auto_mode;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_power_on(void);
int shdisp_api_main_lcd_power_off(void);
int shdisp_api_main_lcd_disp_on(void);
int shdisp_api_main_lcd_disp_off(void);
int shdisp_api_main_lcd_start_display(void);
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl);
int shdisp_api_main_bkl_off(void);
int shdisp_api_shutdown(int seq);
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params);
int shdisp_api_get_main_disp_status(void);
void shdisp_api_get_boot_context(void);
int shdisp_api_get_boot_disp_status(void);

enum {
    SHDISP_IRQ_TYPE_ALS,
    SHDISP_IRQ_TYPE_PS,
    SHDISP_IRQ_TYPE_DET,
    SHDISP_IRQ_TYPE_I2CERR,
    SHDISP_IRQ_TYPE_ALS_REQ,
    NUM_SHDISP_IRQ_TYPE
};

enum {
    SHDISP_XLOG_RECOVERY = 1,
    SHDISP_XLOG_RECOVERING,
};


/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int shdisp_api_main_lcd_post_video_start(void);
int shdisp_api_main_display_done(void);
struct shdisp_gc_lut *shdisp_api_get_gc_lut(void);
struct shdisp_igc_lut *shdisp_api_get_igc_lut(void);
#ifdef SHDISP_DET_DSI_MIPI_ERROR
int shdisp_api_do_mipi_dsi_det_recovery(void);
#endif /* SHDISP_DET_DSI_MIPI_ERROR */
int shdisp_api_tri_led_set_color(struct shdisp_tri_led *tri_led);
void shdisp_api_pre_blank_notify(void);
int shdisp_api_set_freq_param(struct shdisp_freq_params *freq);
int shdisp_API_check_upper_unit(void);
int shdisp_API_get_bdic_is_exist(void);

int shdisp_API_get_usb_info(void);

#endif /* SHDISP_KERN_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
