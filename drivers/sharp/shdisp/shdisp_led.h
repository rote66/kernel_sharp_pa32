/* drivers/sharp/shdisp/shdisp_led.h  (Display Driver)
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
#ifndef SHDISP_LED_H
#define SHDISP_LED_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#ifdef SHDISP_SYSFS_LED
#define SYSFS_LED_SH_LED_1                          (0)
#define SYSFS_LED_SH_LED_2                          (1)
#endif /* SHDISP_SYSFS_LED */

#ifdef SHDISP_SYSFS_DIM
#define SHDISP_LED_SYSFS_DIM_OFF                    (0)
#define SHDISP_LED_SYSFS_DIM_ON                     (1)
#endif /* SHDISP_SYSFS_DIM */
/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_BDIC_REQ_TRI_LED_NONE = 0,
    SHDISP_BDIC_REQ_TRI_LED_ACTIVE,
    SHDISP_BDIC_REQ_TRI_LED_STANDBY,
    SHDISP_BDIC_REQ_TRI_LED_STOP,
    SHDISP_BDIC_REQ_TRI_LED_START,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW,
    SHDISP_BDIC_REQ_TRI_LED_SET_MODE_EMOPATTERN,
    SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,
    SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,
    SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,
    SHDISP_BDIC_REQ_ILLUMI_TRIPLE_COLOR,
    SHDISP_BDIC_REQ_LED_DIM,
};

struct shdisp_led_init_param {
    int handset_color;
    int bdic_chipver;
};

struct shdisp_led_state_str {
    int handset_color;
    int bdic_chipver;
    int bdic_clrvari_index;
};

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
enum {
    ILLUMI_STATE_STOP  = -1,
    ILLUMI_STATE_WAIT_SET_B2_AREA,
    ILLUMI_STATE_WAIT_SET_C2_AREA,
    ILLUMI_STATE_WAIT_SET_A3_AREA,
    ILLUMI_STATE_WAIT_ANIME_BREAK,
    ILLUMI_STATE_WAIT_RESTART,
    ILLUMI_STATE_MAX,
};

struct shdisp_illumi_state {
    char colors[ILLUMI_FRAME_MAX];
    int  count;
    bool running_state;
    int  illumi_state;
    struct workqueue_struct  * workqueue;
    struct delayed_work works[ILLUMI_STATE_MAX];
};
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

struct shdisp_bdic_led_color_index {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    unsigned char color;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_led_API_initialize(struct shdisp_led_init_param *init_param);
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp);

int  shdisp_bdic_API_TRI_LED_off(void);
unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led );
int  shdisp_bdic_API_TRI_LED_normal_on(unsigned char color);
#ifdef SHDISP_SYSFS_LED
int shdisp_bdic_API_LED_on(int no, struct shdisp_tri_led led);
int shdisp_bdic_API_LED_off(int no);
#endif /* SHDISP_SYSFS_LED */
#ifdef SHDISP_SYSFS_DIM
int shdisp_bdic_API_LED_set_dim(int value);
#endif /* SHDISP_SYSFS_DIM */
void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count);
void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count);
#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
void shdisp_bdic_API_TRI_LED_high_speed_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_standard_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_breath_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_long_breath_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_wave_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_flash_on(unsigned char color, int interval, int count);
void shdisp_bdic_API_TRI_LED_aurora_on(int interval, int count);
void shdisp_bdic_API_TRI_LED_rainbow_on(int interval, int count);
#endif /* SHDISP_ILLUMI_COLOR_LED */
void shdisp_bdic_API_TRI_LED_emopattern_on(int interval, int count);
#endif  /* SHDISP_ANIME_COLOR_LED */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
void shdisp_bdic_API_LED_set_illumi_triple_color(struct shdisp_illumi_triple_color illumi_triple_color);
void shdisp_bdic_API_LED_clear_illumi_triple_color(void);
bool shdisp_bdic_API_LED_is_running_illumi_triple_color(void);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

void shdisp_bdic_API_TRI_LED_exit(void);
#if defined(CONFIG_ANDROID_ENGINEERING)
void shdisp_bdic_API_TRI_LED_INFO_output(void);
void shdisp_bdic_API_TRI_LED2_INFO_output(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

#endif  /* SHDISP_LED_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
