/* drivers/sharp/shdisp/shdisp_led.c  (Display Driver)
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
#include "./data/shdisp_led_ctrl.h"
#include "shdisp_bl71y8_main.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_TRI_LED_MODE_OFF           (-1)
#define SHDISP_BDIC_TRI_LED_MODE_NORMAL         (0)
#define SHDISP_BDIC_TRI_LED_MODE_BLINK          (1)
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY        (2)
#define SHDISP_BDIC_TRI_LED_MODE_HISPEED        (3)
#define SHDISP_BDIC_TRI_LED_MODE_STANDARD       (4)
#define SHDISP_BDIC_TRI_LED_MODE_BREATH         (5)
#define SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH    (6)
#define SHDISP_BDIC_TRI_LED_MODE_WAVE           (7)
#define SHDISP_BDIC_TRI_LED_MODE_FLASH          (8)
#define SHDISP_BDIC_TRI_LED_MODE_AURORA         (9)
#define SHDISP_BDIC_TRI_LED_MODE_RAINBOW       (10)
#define SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN    (11)
#define SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR  (12)


#define NO_CURRENT_SET                          (0)
#define CURRENT_SET                             (1)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static void shdisp_led_status_init(void);
static int shdisp_bdic_seq_led_off(void);
static int shdisp_bdic_seq_led_normal_on(unsigned char color);
#ifdef SHDISP_SYSFS_LED
static int shdisp_bdic_seq_led_on(int no, struct shdisp_tri_led led);
static int shdisp_bdic_seq_leds_off(int no);
static void shdisp_bdic_seq_led_current_on(void);
#endif /* SHDISP_SYSFS_LED */
#ifdef SHDISP_SYSFS_DIM
static int shdisp_bdic_seq_leds_dim(int value);
#endif /* SHDISP_SYSFS_DIM */
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count);
#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
static void shdisp_bdic_seq_led_high_speed_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_standard_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_breath_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_long_breath_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_wave_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_flash_on(unsigned char color, int interval, int count);
static void shdisp_bdic_seq_led_aurora_on(int interval, int count);
static void shdisp_bdic_seq_led_rainbow_on(int interval, int count);
#endif /* SHDISP_ILLUMI_COLOR_LED */
static void shdisp_bdic_seq_led_emopattern_on(int interval, int count);
#endif  /* SHDISP_ANIME_COLOR_LED */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
static void shdisp_bdic_seq_illumi_cancel_and_clear(void);
static void shdisp_bdic_seq_illumi_triple_color_on(struct shdisp_illumi_triple_color illumi_triple_color);
static void shdisp_bdic_seq_illumi_triple_color(unsigned char first_color);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color);

static void shdisp_bdic_LD_set_led_on_table(unsigned char *rgb_current);
#ifdef SHDISP_COLOR_LED_TWIN
static void shdisp_bdic_LD_set_led_on_table_twin(unsigned char *rgb_current);
#endif /* SHDISP_COLOR_LED_TWIN */

#ifdef SHDISP_SYSFS_LED
static bool shdisp_bdic_is_led_current_mode(void);
static bool shdisp_bdic_is_led_current_mode_no(int no);
static void shdisp_bdic_clear_current_param(void);
static void shdisp_bdic_LD_set_led_fix_current_table(unsigned char *rgb_current);
#ifdef SHDISP_COLOR_LED_TWIN
static void shdisp_bdic_LD_set_led_fix_current_table_twin(unsigned char *rgb_current);
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */

static void shdisp_bdic_seq_bdic_active_for_led(int);
static void shdisp_bdic_seq_bdic_standby_for_led(int);
static unsigned char shdisp_bdic_get_color_index_and_reedit(struct shdisp_tri_led *tri_led);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
#ifndef SHDISP_COLOR_LED_TWIN
static void shdisp_bdic_PD_TRI_LED_anime_start(void);
#endif  /* SHDISP_COLOR_LED_TWIN */
static void shdisp_bdic_PD_TRI_LED_set_anime(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig(void);
static void shdisp_bdic_PD_TRI_LED_write_chdig(int address, unsigned char *anime_tbl[3]);
static void shdisp_bdic_PD_TRI_LED_lposc_off(void);
static int shdisp_bdic_PD_TRI_LED_get_clrvari_index(int clrvari);
#ifdef SHDISP_COLOR_LED_TWIN
static void shdisp_bdic_PD_TRI_LED_control_twin(unsigned char request, int param);
static void shdisp_bdic_PD_TRI_LED_anime_start_twin(void);
static void shdisp_bdic_PD_TRI_LED_set_anime_twin(void);
static void shdisp_bdic_PD_TRI_LED_set_chdig_twin(void);
static void shdisp_bdic_LD_set_led_fix_on_table_twin(int clr_vari, int color);
#if defined(CONFIG_ANDROID_ENGINEERING)
static void shdisp_bdic_TRI_LED_INFO_output_twin(void);
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
static void shdisp_bdic_illumi_triple_color_INFO_output(void);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif /* SHDISP_COLOR_LED_TWIN */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
static void shdisp_bdic_cancel_illumi_work(void);
static void shdisp_bdic_clear_illumi_state(void);
static void shdisp_bdic_illumi_color_set_a2(void);
static void shdisp_workqueue_handler_illumi_set_b2(struct work_struct *work);
static void shdisp_workqueue_handler_illumi_set_c2(struct work_struct *work);
static void shdisp_workqueue_handler_illumi_set_a3(struct work_struct *work);
static void shdisp_workqueue_handler_illumi_set_anime_stop(struct work_struct *work);
static void shdisp_workqueue_handler_illumi_restart(struct work_struct *work);
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_led_state_str led_state_str;

static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_before_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;
static int shdisp_bdic_tri_led_count;
#ifdef SHDISP_COLOR_LED_TWIN
static int shdisp_bdic_tri_led_mode_twin;
#endif /* SHDISP_COLOR_LED_TWIN */

#ifdef SHDISP_SYSFS_LED
static unsigned char rgb_current1[SHDISP_RGB];
#ifdef SHDISP_COLOR_LED_TWIN
static unsigned char rgb_current2[SHDISP_RGB];
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */

#ifdef SHDISP_SYSFS_DIM
static int shdisp_led_dim;
#endif /* SHDISP_SYSFS_DIM */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
struct shdisp_illumi_state illumi_state;
static int shdisp_illumi_delayed_times[ILLUMI_STATE_MAX] = {
     576730 + (1000000/HZ),
    1153460 + (1000000/HZ),
    1730190 + (1000000/HZ),
    3460380 + (1000000/HZ),
    3965900 + (1000000/HZ),
};
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_led_API_initialize                                                 */
/* ------------------------------------------------------------------------- */
int shdisp_led_API_initialize(struct shdisp_led_init_param *init_param)
{
    shdisp_led_status_init();

    led_state_str.bdic_chipver        = init_param->bdic_chipver;
    led_state_str.handset_color       = init_param->handset_color;
    led_state_str.bdic_clrvari_index  = shdisp_bdic_PD_TRI_LED_get_clrvari_index(init_param->handset_color);

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode        = tmp->led_mode;
    shdisp_bdic_tri_led_before_mode = tmp->led_mode;
    shdisp_bdic_tri_led_color       = color;
    shdisp_bdic_tri_led_ontime      = tmp->ontime;
    shdisp_bdic_tri_led_interval    = tmp->interval;
    shdisp_bdic_tri_led_count       = tmp->count;

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_tri_led_mode_twin        = tmp->led_mode;
#endif /* SHDISP_COLOR_LED_TWIN */
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_TRI_LED_off(void)
{
    int ret;
    ret = shdisp_bdic_seq_led_off();
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led)
{
    unsigned char color=0;

    color = shdisp_bdic_get_color_index_and_reedit(tri_led);

    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    int ret;
    ret = shdisp_bdic_seq_led_normal_on(color);
    return ret;
}

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_on                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_LED_on(int no, struct shdisp_tri_led led)
{
    int ret;
    ret = shdisp_bdic_seq_led_on(no, led);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_off                                                   */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_LED_off(int no)
{
    int ret;

    ret = shdisp_bdic_seq_leds_off(no);
    return ret;
}
#endif /* SHDISP_SYSFS_LED */

#ifdef SHDISP_SYSFS_DIM
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_set_dim                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_LED_set_dim(int value)
{
    int ret = SHDISP_RESULT_SUCCESS;

    ret = shdisp_bdic_seq_leds_dim(value);

    return ret;
}
#endif /* SHDISP_SYSFS_DIM */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_blink_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_firefly_on                                        */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_firefly_on(color, ontime, interval, count);
    return;
}

#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_high_speed_on                                     */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_high_speed_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_high_speed_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_HISPEED, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_standard_on                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_standard_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_standard_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_STANDARD, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_breath_on                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_breath_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_BREATH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_long_breath_on                                    */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_long_breath_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_long_breath_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_LONG_BREATH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_wave_on                                           */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_wave_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_wave_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_WAVE, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_flash_on                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_flash_on(unsigned char color, int interval, int count)
{
    shdisp_bdic_seq_led_flash_on(color, SHDISP_BDIC_TRI_LED_INTERVAL_FLASH, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_aurora_on                                         */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_aurora_on(int interval, int count)
{
    shdisp_bdic_seq_led_aurora_on(SHDISP_BDIC_TRI_LED_INTERVAL_AURORA, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_rainbow_on                                        */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_rainbow_on(int interval, int count)
{
    shdisp_bdic_seq_led_rainbow_on(SHDISP_BDIC_TRI_LED_INTERVAL_RAINBOW, count);
    return;
}
#endif /* SHDISP_ILLUMI_COLOR_LED */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_set_illumi_triple_color                               */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LED_set_illumi_triple_color(struct shdisp_illumi_triple_color illumi_triple_color)
{
    shdisp_bdic_seq_illumi_triple_color_on(illumi_triple_color);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_clear_illumi_triple_color                             */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_LED_clear_illumi_triple_color(void)
{
    shdisp_bdic_seq_illumi_cancel_and_clear();
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LED_is_running_illumi_triple_color                        */
/* ------------------------------------------------------------------------- */
bool shdisp_bdic_API_LED_is_running_illumi_triple_color(void)
{
    return illumi_state.running_state;
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_emopattern_on                                     */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_emopattern_on(int interval, int count)
{
    shdisp_bdic_seq_led_emopattern_on(SHDISP_BDIC_TRI_LED_INTERVAL_EMOPATTERN, SHDISP_BDIC_TRI_LED_COUNT_EMOPATTERN);
    return;
}
#endif  /* SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_exit                                              */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_exit(void)
{
#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    if (illumi_state.workqueue) {
        destroy_workqueue(illumi_state.workqueue);
        illumi_state.workqueue = NULL;
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
}

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_INFO_output                                       */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    pbuf = (unsigned char *)kzalloc((BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d", (BDIC_REG_CH2_C - BDIC_REG_SEQ_ANIME + 1));
        return;
    }
    p = pbuf;
    for (idx = BDIC_REG_SEQ_ANIME; idx <= BDIC_REG_CH2_C; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] TRI-LED INFO ->>\n");
    printk("[SHDISP] led_state_str.handset_color       = %d.\n", led_state_str.handset_color);
    printk("[SHDISP] led_state_str.bdic_clrvari_index  = %d.\n", led_state_str.bdic_clrvari_index);
    printk("[SHDISP] led_state_str.bdic_chipver        = %d.\n", led_state_str.bdic_chipver);
    printk("[SHDISP] shdisp_bdic_tri_led_color         = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode          = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime        = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval      = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count         = %d.\n", shdisp_bdic_tri_led_count);

#ifdef SHDISP_SYSFS_LED
    printk("[SHDISP] rgb_current1[0]                   = %d.\n", rgb_current1[0]);
    printk("[SHDISP] rgb_current1[1]                   = %d.\n", rgb_current1[1]);
    printk("[SHDISP] rgb_current1[2]                   = %d.\n", rgb_current1[2]);
#endif /* SHDISP_SYSFS_LED */

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x\n", BDIC_REG_SEQ_ANIME, *p, *(p + 1), *(p + 2));
    p += 3;
    printk("[SHDISP] BDIC_REG_LED_SETTING   0x%2X: %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH0_SET1, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED_CURRENT   0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH0_A, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7), *(p + 8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED INFO <<-\n");

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_TRI_LED_INFO_output_twin();
#endif /* SHDISP_COLOR_LED_TWIN */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    shdisp_bdic_illumi_triple_color_INFO_output();
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED2_INFO_output                                      */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_TRI_LED2_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned short   shdisp_log_lv_bk;
    size_t  size;

    size  = (BDIC_REG_TIMER2 - BDIC_REG_SEQ_ANIME + 1);
    size += (BDIC_REG_CH5_C - BDIC_REG_CH3_SET1 + 1);

    pbuf = (unsigned char *)kzalloc(size, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%ld", size);
        return;
    }

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;
    shdisp_bdic_API_IO_bank_set(0x00);

    p = pbuf;
    for (idx = BDIC_REG_SEQ_ANIME; idx <= BDIC_REG_TIMER2; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    for (idx = BDIC_REG_CH3_SET1; idx <= BDIC_REG_CH5_C; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_log_lv = shdisp_log_lv_bk;

    printk("[SHDISP] TRI-LED2 INFO ->>\n");
    printk("[SHDISP] led_state_str.handset_color       = %d.\n", led_state_str.handset_color);
    printk("[SHDISP] led_state_str.bdic_clrvari_index  = %d.\n", led_state_str.bdic_clrvari_index);
    printk("[SHDISP] led_state_str.bdic_chipver        = %d.\n", led_state_str.bdic_chipver);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x\n", BDIC_REG_SEQ_ANIME, *p, *(p + 1), *(p + 2));
    p += 3;
    printk("[SHDISP] BDIC_REG_LED2_SETTING  0x%2X: %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH3_SET1, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED2_CURRENT  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH3_A, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7), *(p + 8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED2 INFO <<-\n");
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* shdisp_led_status_init                                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_led_status_init(void)
{
    SHDISP_TRACE("in")
    shdisp_bdic_tri_led_color    = 0;
    shdisp_bdic_tri_led_mode     = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
    shdisp_bdic_tri_led_ontime   = 0;
    shdisp_bdic_tri_led_interval = 0;
    shdisp_bdic_tri_led_count    = 0;
#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_tri_led_mode_twin           = SHDISP_BDIC_TRI_LED_MODE_OFF;
#endif /* SHDISP_COLOR_LED_TWIN */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
    memset(&illumi_state, 0, sizeof(illumi_state));
    illumi_state.illumi_state = ILLUMI_STATE_STOP;
    illumi_state.running_state = false;

    illumi_state.workqueue = create_singlethread_workqueue("shdisp_illumi");
    if (!illumi_state.workqueue) {
        SHDISP_ERR("create shdisp_illumi workqueue failed.");
    } else {
        INIT_DELAYED_WORK(&illumi_state.works[ILLUMI_STATE_WAIT_SET_B2_AREA],   shdisp_workqueue_handler_illumi_set_b2);
        INIT_DELAYED_WORK(&illumi_state.works[ILLUMI_STATE_WAIT_SET_C2_AREA],   shdisp_workqueue_handler_illumi_set_c2);
        INIT_DELAYED_WORK(&illumi_state.works[ILLUMI_STATE_WAIT_SET_A3_AREA],   shdisp_workqueue_handler_illumi_set_a3);
        INIT_DELAYED_WORK(&illumi_state.works[ILLUMI_STATE_WAIT_ANIME_BREAK],   shdisp_workqueue_handler_illumi_set_anime_stop);
        INIT_DELAYED_WORK(&illumi_state.works[ILLUMI_STATE_WAIT_RESTART],       shdisp_workqueue_handler_illumi_restart);
    }
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

#ifdef SHDISP_SYSFS_DIM
    shdisp_led_dim = SHDISP_LED_SYSFS_DIM_OFF;
#endif /* SHDISP_SYSFS_DIM */
    SHDISP_TRACE("out")
    return;
}

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_on                                                    */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_led_on(int no, struct shdisp_tri_led led)
{
    bool is_led_current_mode;
    int red, green, blue;

    SHDISP_TRACE("in no:%d", no);

    is_led_current_mode = shdisp_bdic_is_led_current_mode();

    SHDISP_DEBUG("is_led_current_mode:%d", is_led_current_mode);

    red   = led.red;
    green = led.green;
    blue  = led.blue;
    SHDISP_DEBUG("RGB=0x%02X,0x%02X,0x%02X", red, green, blue);

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        red   = (int)((long)((red   + 2) * 10) / 30);
        green = (int)((long)((green + 2) * 10) / 30);
        blue  = (int)((long)((blue  + 2) * 10) / 30);
        SHDISP_DEBUG("led_dim: RGB=0x%02X,0x%02X,0x%02X", red, green, blue);
    }
#endif /* SHDISP_SYSFS_DIM */

    if (no == SYSFS_LED_SH_LED_1) {
        rgb_current1[0] = shdisp_bdic_API_correct_proh_val(red);
        rgb_current1[1] = shdisp_bdic_API_correct_proh_val(green);
        rgb_current1[2] = shdisp_bdic_API_correct_proh_val(blue);
#ifdef SHDISP_COLOR_LED_TWIN
    } else {
        rgb_current2[0] = shdisp_bdic_API_correct_proh_val(red);
        rgb_current2[1] = shdisp_bdic_API_correct_proh_val(green);
        rgb_current2[2] = shdisp_bdic_API_correct_proh_val(blue);
#endif /* SHDISP_COLOR_LED_TWIN */
    }

    if (!is_led_current_mode) {
        shdisp_bdic_seq_led_current_on();
    } else {
        if (no == SYSFS_LED_SH_LED_1) {
            shdisp_bdic_LD_set_led_fix_current_table(rgb_current1);
#ifdef SHDISP_COLOR_LED_TWIN
        } else {
            shdisp_bdic_LD_set_led_fix_current_table_twin(rgb_current2);
#endif /* SHDISP_COLOR_LED_TWIN */
        }
    }
    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_leds_off                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_leds_off(int no)
{
    SHDISP_TRACE("in no:%d", no);

    if (!shdisp_bdic_is_led_current_mode_no(no)) {
        shdisp_bdic_seq_led_off();
        shdisp_bdic_clear_current_param();
    } else {
        if (no == SYSFS_LED_SH_LED_1) {
            memset(rgb_current1, 0x00, sizeof(rgb_current1));
            shdisp_bdic_LD_set_led_fix_current_table(rgb_current1);
#ifdef SHDISP_COLOR_LED_TWIN
        } else {
            memset(rgb_current2, 0x00, sizeof(rgb_current2));
            shdisp_bdic_LD_set_led_fix_current_table_twin(rgb_current2);
#endif /* SHDISP_COLOR_LED_TWIN */
        }
    }

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_current_on                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_current_on(void)
{
    SHDISP_TRACE("in");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, 0);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, CURRENT_SET);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, 0);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, CURRENT_SET);
#endif /* SHDISP_COLOR_LED_TWIN */

    SHDISP_TRACE("out");
    return;
}

#endif /* SHDISP_SYSFS_LED */

#ifdef SHDISP_SYSFS_DIM
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_leds_dim                                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_leds_dim(int value)
{
    SHDISP_TRACE("in value:%d", value);

    if (shdisp_led_dim == value) {
        return SHDISP_RESULT_SUCCESS;
    }

    if ((value != SHDISP_LED_SYSFS_DIM_ON) && (value != SHDISP_LED_SYSFS_DIM_OFF)){
        return SHDISP_RESULT_FAILURE;
    }

    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_LED_DIM, value);


    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}
#endif /* SHDISP_SYSFS_DIM */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_off                                                   */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_led_off(void)
{
    SHDISP_TRACE("in");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_STOP, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_STOP, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_normal_on                                             */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_seq_led_normal_on(unsigned char color)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    SHDISP_TRACE("out");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_blink_on                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_firefly_on                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_firefly_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_high_speed_on                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_high_speed_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_standard_on                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_standard_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_breath_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_breath_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_long_breath_on                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_long_breath_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_wave_on                                               */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_wave_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_flash_on                                              */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_flash_on(unsigned char color, int interval, int count)
{
    SHDISP_TRACE("in color:%d", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH, color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_aurora_on                                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_aurora_on(int interval, int count)
{
    SHDISP_TRACE("in");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA, SHDISP_BDIC_TRI_LED_COLOR_WHITE);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA, SHDISP_BDIC_TRI_LED_COLOR_WHITE);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_rainbow_on                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_rainbow_on(int interval, int count)
{
    SHDISP_TRACE("in");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW, SHDISP_BDIC_TRI_LED_COLOR_WHITE);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW, SHDISP_BDIC_TRI_LED_COLOR_WHITE);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}
#endif /* SHDISP_ILLUMI_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_emopattern_on                                         */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_led_emopattern_on(int interval, int count)
{
    SHDISP_TRACE("in");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_EMOPATTERN, SHDISP_BDIC_TRI_LED_COLOR_MAGENTA);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_EMOPATTERN, SHDISP_BDIC_TRI_LED_COLOR_MAGENTA);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    if (led_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        if (interval != 0) {
            shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
        }
    }
    SHDISP_TRACE("out");
}
#endif  /* SHDISP_ANIME_COLOR_LED */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_illumi_triple_color                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_illumi_triple_color(unsigned char first_color)
{
    SHDISP_TRACE("in color:%d", first_color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_ILLUMI_TRIPLE_COLOR, first_color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_START, 0);

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_ILLUMI_TRIPLE_COLOR, first_color);
    shdisp_bdic_PD_TRI_LED_control_twin(SHDISP_BDIC_REQ_TRI_LED_START, 0);
#endif /* SHDISP_COLOR_LED_TWIN */

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_illumi_triple_color_on                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_illumi_triple_color_on(struct shdisp_illumi_triple_color illumi_triple_color)
{
    struct shdisp_tri_led tri_led;
    int i;
    int work_count;

    SHDISP_TRACE("in");
    if (!illumi_state.workqueue) {
        SHDISP_ERR("illumi_state.workqueue does not exist err.");
        return;
    }

    for (i = 0; i != ILLUMI_FRAME_MAX; i++) {
        tri_led.red   = illumi_triple_color.colors[i].red;
        tri_led.green = illumi_triple_color.colors[i].green;
        tri_led.blue  = illumi_triple_color.colors[i].blue;
        illumi_state.colors[i] = shdisp_bdic_get_color_index_and_reedit(&tri_led);
    }
    illumi_state.count = illumi_triple_color.count;
    illumi_state.running_state = true;

    shdisp_bdic_seq_illumi_triple_color(illumi_state.colors[ILLUMI_FRAME_FIRST]);

    work_count = (illumi_state.count == SHDISP_TRI_LED_COUNT_NONE) ? ILLUMI_STATE_MAX : ILLUMI_STATE_MAX - 1;
    SHDISP_DEBUG("queue delay_works = %d isonshot = %d", work_count, illumi_state.count);
    for (i = 0; i != work_count; ++i) {
        queue_delayed_work(illumi_state.workqueue, &illumi_state.works[i], usecs_to_jiffies(shdisp_illumi_delayed_times[i]));
        SHDISP_DEBUG("delay_works[%d] was queued", i);
    }

    shdisp_IO_API_msleep(10);
    shdisp_bdic_illumi_color_set_a2();
    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_illumi_cancel_and_clear                                   */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_seq_illumi_cancel_and_clear(void)
{
    SHDISP_TRACE("in");

    SHDISP_DEBUG("illumi_state.running_state = %d", illumi_state.running_state);
    if (illumi_state.running_state) {
        shdisp_API_semaphore_start();
        illumi_state.running_state = false;
        shdisp_API_semaphore_end();

        shdisp_bdic_cancel_illumi_work();

        shdisp_API_semaphore_start();
        shdisp_bdic_clear_illumi_state();
        shdisp_API_semaphore_end();
    }

    SHDISP_TRACE("out");
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_write_illumi_triple_color_top                                 */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_write_illumi_triple_color_top(char reg, int color_index)
{
    char color_rgb[3];
    int dim_index = 0;
    int clrvari = led_state_str.bdic_clrvari_index;
#ifdef SHDISP_COLOR_LED_TWIN
    char reg_twin = 0;
    char color_rgb_twin[3];
#endif

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */

#ifdef SHDISP_COLOR_LED_TWIN
    switch (reg) {
    case BDIC_REG_CH0_A:
       reg_twin = BDIC_REG_CH3_A;
       break;
    case BDIC_REG_CH0_B:
       reg_twin = BDIC_REG_CH3_B;
       break;
    case BDIC_REG_CH0_C:
    default:
       reg_twin = BDIC_REG_CH3_C;
    }
#endif

    color_rgb[0] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][0];
    color_rgb[1] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][1];
    color_rgb[2] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][2];
#ifdef SHDISP_COLOR_LED_TWIN
    color_rgb_twin[0] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][0];
    color_rgb_twin[1] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][1];
    color_rgb_twin[2] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][2];
#endif

    shdisp_bdic_API_IO_multi_write_reg(reg, color_rgb, 3);
#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_API_IO_multi_write_reg(reg_twin, color_rgb_twin, 3);
#endif
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_write_illumi_triple_color_bottom                              */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_write_illumi_triple_color_bottom(char reg, int color_index)
{
    char color_rgb[3];
#ifdef SHDISP_COLOR_LED_TWIN
    char reg_twin = 0;
#endif
    memset(color_rgb, 0, sizeof(color_rgb));

#ifdef SHDISP_COLOR_LED_TWIN
    switch (reg) {
    case BDIC_REG_CH0_A:
       reg_twin = BDIC_REG_CH3_A;
       break;
    case BDIC_REG_CH0_B:
       reg_twin = BDIC_REG_CH3_B;
       break;
    case BDIC_REG_CH0_C:
    default:
       reg_twin = BDIC_REG_CH3_C;
    }
#endif

    shdisp_bdic_API_IO_multi_write_reg(reg, color_rgb, 3);
#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_API_IO_multi_write_reg(reg_twin, color_rgb, 3);
#endif
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_cancel_illumi_work                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_cancel_illumi_work(void)
{

    int i;
    int queued_tail;
    int queued_head = illumi_state.illumi_state;

    if (queued_head == ILLUMI_STATE_STOP) {
        return;
    }

    queued_tail = (illumi_state.count == SHDISP_TRI_LED_COUNT_NONE) ? ILLUMI_STATE_MAX : ILLUMI_STATE_MAX -1;
    i = illumi_state.illumi_state;

    for (; i != queued_tail; i++) {
        SHDISP_DEBUG("cancel work[%d]", i);
        cancel_delayed_work_sync(&illumi_state.works[i]);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_clear_illumi_state                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_clear_illumi_state(void)
{
    illumi_state.colors[ILLUMI_FRAME_FIRST] = illumi_state.colors[ILLUMI_FRAME_SECOND] = illumi_state.colors[ILLUMI_FRAME_THIRD] = 0;
    illumi_state.count = SHDISP_TRI_LED_COUNT_NONE;
    illumi_state.illumi_state = ILLUMI_STATE_STOP;
    illumi_state.running_state = false;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_illumi_color_set_a2                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_illumi_color_set_a2(void)
{
    SHDISP_TRACE("in");

    shdisp_bdic_write_illumi_triple_color_top(BDIC_REG_CH0_A, illumi_state.colors[ILLUMI_FRAME_SECOND]);
    illumi_state.illumi_state = ILLUMI_STATE_WAIT_SET_B2_AREA;

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_illumi_set_b2                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_illumi_set_b2(struct work_struct *work)
{
    SHDISP_TRACE("in");

    shdisp_API_semaphore_start();

    if (!illumi_state.running_state) {
        SHDISP_DEBUG("out running_state = %d", illumi_state.running_state);
        shdisp_API_semaphore_end();
        return;
    }

    shdisp_bdic_write_illumi_triple_color_bottom(BDIC_REG_CH0_B, ILLUMI_FRAME_SECOND);

    illumi_state.illumi_state = ILLUMI_STATE_WAIT_SET_C2_AREA;

    shdisp_API_semaphore_end();

    SHDISP_TRACE("out");


}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_illumi_set_c2                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_illumi_set_c2(struct work_struct *work)
{
    SHDISP_TRACE("in");

    shdisp_API_semaphore_start();

    if (!illumi_state.running_state) {
        SHDISP_DEBUG("out running_state = %d", illumi_state.running_state);
        shdisp_API_semaphore_end();
        return;
    }

    shdisp_bdic_write_illumi_triple_color_top(BDIC_REG_CH0_C, illumi_state.colors[ILLUMI_FRAME_THIRD]);

    illumi_state.illumi_state = ILLUMI_STATE_WAIT_SET_A3_AREA;

    shdisp_API_semaphore_end();

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_illumi_set_a3                                    */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_illumi_set_a3(struct work_struct *work)
{
    SHDISP_TRACE("in");

    shdisp_API_semaphore_start();

    if (!illumi_state.running_state) {
        SHDISP_DEBUG("out running_state = %d", illumi_state.running_state);
        shdisp_API_semaphore_end();
        return;
    }

    shdisp_bdic_write_illumi_triple_color_bottom(BDIC_REG_CH0_A, ILLUMI_FRAME_THIRD);

    illumi_state.illumi_state = ILLUMI_STATE_WAIT_ANIME_BREAK;

    shdisp_API_semaphore_end();

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_illumi_set_anime_stop                            */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_illumi_set_anime_stop(struct work_struct *work)
{
    SHDISP_TRACE("in");

    shdisp_API_semaphore_start();

    if (!illumi_state.running_state) {
        SHDISP_DEBUG("out running_state = %d", illumi_state.running_state);
        shdisp_API_semaphore_end();
        return;
    }

    shdisp_bdic_API_IO_write_reg(BDIC_REG_SYSTEM7, 0x00);

    if (illumi_state.count == SHDISP_TRI_LED_COUNT_1) {
        shdisp_bdic_clear_illumi_state();
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    } else {
        illumi_state.illumi_state = ILLUMI_STATE_WAIT_RESTART;
    }

    shdisp_API_semaphore_end();

    SHDISP_TRACE("out");
}

/* ------------------------------------------------------------------------- */
/* shdisp_workqueue_handler_illumi_restart                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_workqueue_handler_illumi_restart(struct work_struct *work)
{
    int clrvari = led_state_str.bdic_clrvari_index;
    int i;
    int work_count;
    unsigned char color_rgb[9];
#ifdef SHDISP_COLOR_LED_TWIN
    unsigned char color_rgb_twin[9];
#endif
    unsigned char color_index;
    int dim_index = 0;

    SHDISP_TRACE("in");

    shdisp_API_semaphore_start();

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */

    if (!illumi_state.running_state) {
        SHDISP_DEBUG("out running_state = %d", illumi_state.running_state);
        shdisp_API_semaphore_end();
        return;
    }

    color_index = illumi_state.colors[ILLUMI_FRAME_FIRST];
    color_rgb[0] = 0;
    color_rgb[1] = 0;
    color_rgb[2] = 0;
    color_rgb[3] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][0];
    color_rgb[4] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][1];
    color_rgb[5] = shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index][2];
    color_rgb[6] = 0;
    color_rgb[7] = 0;
    color_rgb[8] = 0;
#ifdef SHDISP_COLOR_LED_TWIN
    color_rgb_twin[0] = 0;
    color_rgb_twin[1] = 0;
    color_rgb_twin[2] = 0;
    color_rgb_twin[3] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][0];
    color_rgb_twin[4] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][1];
    color_rgb_twin[5] = shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index][2];
    color_rgb_twin[6] = 0;
    color_rgb_twin[7] = 0;
    color_rgb_twin[8] = 0;
#endif

    shdisp_bdic_API_IO_multi_write_reg(BDIC_REG_CH0_A, color_rgb, 9);
#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_API_IO_multi_write_reg(BDIC_REG_CH3_A, color_rgb_twin, 9);
#endif

#ifdef SHDISP_COLOR_LED_TWIN
    shdisp_bdic_API_IO_write_reg(BDIC_REG_SYSTEM7, 0x05);
#else
    shdisp_bdic_API_IO_write_reg(BDIC_REG_SYSTEM7, 0x01);
#endif

    work_count = (illumi_state.count == SHDISP_TRI_LED_COUNT_NONE) ? ILLUMI_STATE_MAX : ILLUMI_STATE_MAX - 1;
    SHDISP_DEBUG("restart queue delay_works = %d isonshot = %d", work_count, illumi_state.count);
    for (i = 0; i != work_count; ++i) {
        queue_delayed_work(illumi_state.workqueue, &illumi_state.works[i], usecs_to_jiffies(shdisp_illumi_delayed_times[i]));
        SHDISP_DEBUG("delay_works[%d] was queued", i);
    }

    shdisp_IO_API_msleep(10);
    shdisp_bdic_illumi_color_set_a2();

    illumi_state.illumi_state = ILLUMI_STATE_WAIT_SET_B2_AREA;

    shdisp_API_semaphore_end();

    SHDISP_TRACE("out");
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_on_table                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color)
{
    unsigned char *pTriLed;
    int dim_index = 0;

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */
    pTriLed = (unsigned char *)(&(shdisp_triple_led_tbl[dim_index][clr_vari][color]));

    shdisp_bdic_LD_set_led_on_table(pTriLed);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_on_table                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_on_table(unsigned char *rgb_current)
{

    unsigned char *pTriLed;
    shdisp_bdicRegSetting_t led_fix_on[ARRAY_SIZE(shdisp_bdic_led_fix_on)];

    memcpy(led_fix_on, shdisp_bdic_led_fix_on, sizeof(shdisp_bdic_led_fix_on));

    if (rgb_current == NULL) {
        SHDISP_DEBUG("[warning] parameter is NULL");
        return;
    }

    pTriLed = (unsigned char *)rgb_current;

    led_fix_on[SHDISP_LED_FIX_ONR].data = *(pTriLed + 0);
    led_fix_on[SHDISP_LED_FIX_ONG].data = *(pTriLed + 1);
    led_fix_on[SHDISP_LED_FIX_ONB].data = *(pTriLed + 2);
    SHDISP_BDIC_REGSET(led_fix_on);
}

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_current_table                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_fix_current_table(unsigned char *rgb_current)
{
    unsigned char *pTriLed;
    shdisp_bdicRegSetting_t led_current[ARRAY_SIZE(shdisp_bdic_led_current)];

    memcpy(led_current, shdisp_bdic_led_current, sizeof(shdisp_bdic_led_current));

    if (rgb_current == NULL) {
        SHDISP_DEBUG("[warning] parameter is NULL");
        return;
    }

    pTriLed = (unsigned char *)rgb_current;

    led_current[0].data = *(pTriLed + 0);
    led_current[1].data = *(pTriLed + 1);
    led_current[2].data = *(pTriLed + 2);
    SHDISP_BDIC_REGSET(led_current);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_is_led_current_mode                                           */
/* ------------------------------------------------------------------------- */
static bool shdisp_bdic_is_led_current_mode(void)
{
    int i;

    for (i = 0; i < SHDISP_RGB; i++) {
#ifdef SHDISP_COLOR_LED_TWIN
        if (rgb_current1[i] || rgb_current2[i]) {
#else /* SHDISP_COLOR_LED_TWIN */
        if (rgb_current1[i]) {
#endif /* SHDISP_COLOR_LED_TWIN */
            return true;
        }
    }

    return false;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_is_led_current_mode_no                                        */
/* ------------------------------------------------------------------------- */
static bool shdisp_bdic_is_led_current_mode_no(int no)
{
#ifdef SHDISP_COLOR_LED_TWIN
    int i;
    unsigned char *chk_current = ((no == SYSFS_LED_SH_LED_1) ? rgb_current2 : rgb_current1);

    for (i = 0; i < SHDISP_RGB; i++) {
        if (chk_current[i]) {
            return true;
        }
    }
#else /* SHDISP_COLOR_LED_TWIN */
    if (no != SYSFS_LED_SH_LED_1)
        return true;
#endif /* SHDISP_COLOR_LED_TWIN */
    return false;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_clear_current_param                                           */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_clear_current_param(void)
{
    SHDISP_TRACE("in");
    memset(rgb_current1, 0x00, sizeof(rgb_current1));
#ifdef SHDISP_COLOR_LED_TWIN
    memset(rgb_current2, 0x00, sizeof(rgb_current2));
#endif /* SHDISP_COLOR_LED_TWIN */
    SHDISP_TRACE("out");
    return;
}
#endif /* SHDISP_SYSFS_LED */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_active_for_led                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_bdic_active_for_led(int dev_type)
{
    SHDISP_TRACE("in dev_type:%d", dev_type);
    (void)shdisp_pm_API_bdic_power_manager(dev_type, SHDISP_DEV_REQ_ON);
    SHDISP_TRACE("out");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_standby_for_led                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_seq_bdic_standby_for_led(int dev_type)
{
    (void)shdisp_pm_API_bdic_power_manager(dev_type, SHDISP_DEV_REQ_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_get_color_index_and_reedit                                    */
/* ------------------------------------------------------------------------- */
static unsigned char shdisp_bdic_get_color_index_and_reedit(struct shdisp_tri_led *tri_led)
{
    unsigned int i;
    unsigned char color = 0xFF;

#ifdef SHDISP_EXTEND_COLOR_LED
    struct shdisp_bdic_led_color_index extend_cloler_index[SHDISP_TRI_LED_EXTEND_COLOR_TBL_NUM];

    if ((tri_led->red <= 5) && (tri_led->green <= 5) && (tri_led->blue <= 5)) {
        if ((tri_led->red >= 3) || (tri_led->green >= 3) || (tri_led->blue >= 3)) {
            if (tri_led->red == 0x00) {
                memcpy(extend_cloler_index, shdisp_triple_led_extend_color_index_tbl0, sizeof(extend_cloler_index));
            } else if (tri_led->red == 0x03) {
                memcpy(extend_cloler_index, shdisp_triple_led_extend_color_index_tbl3, sizeof(extend_cloler_index));
            } else if (tri_led->red == 0x04) {
                memcpy(extend_cloler_index, shdisp_triple_led_extend_color_index_tbl4, sizeof(extend_cloler_index));
            } else if (tri_led->red == 0x05) {
                memcpy(extend_cloler_index, shdisp_triple_led_extend_color_index_tbl5, sizeof(extend_cloler_index));
            }

            for (i = 0; i < ARRAY_SIZE(extend_cloler_index); i++) {
                if (extend_cloler_index[i].green == tri_led->green   &&
                    extend_cloler_index[i].blue  == tri_led->blue) {
                    color = extend_cloler_index[i].color;
                    break;
                }
            }
        } else {
            for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
                if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                    shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                    shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue) {
                    color = shdisp_triple_led_color_index_tbl[i].color;
                    break;
                }
            }
        }
    }
#else /* SHDISP_EXTEND_COLOR_LED */
    for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
        if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
            shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
            shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue) {
            color = shdisp_triple_led_color_index_tbl[i].color;
            break;
        }
    }
#endif /* SHDISP_EXTEND_COLOR_LED */

    if (color == 0xFF) {
        if (tri_led->red > 1) {
            tri_led->red = 1;
        }
        if (tri_led->green > 1) {
            tri_led->green = 1;
        }
        if (tri_led->blue > 1) {
            tri_led->blue = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
            if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue) {
                color = shdisp_triple_led_color_index_tbl[i].color;
                break;
            }
        }
        if (color == 0xFF) {
            color = 0;
        }
    }
    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_TRI_LED_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_TRI_LED_STANDBY:
        break;

    case SHDISP_BDIC_REQ_TRI_LED_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_TRI_LED_START tri_led_mode=%d, led_before_mode=%d"
                       , shdisp_bdic_tri_led_mode, shdisp_bdic_tri_led_before_mode);
#ifdef SHDISP_SYSFS_LED
        if (param == NO_CURRENT_SET) {
            shdisp_bdic_clear_current_param();
        }
#endif /* SHDISP_SYSFS_LED */
        switch (shdisp_bdic_tri_led_before_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            if (shdisp_bdic_tri_led_mode != SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
                SHDISP_BDIC_REGSET(shdisp_bdic_led_off_fix);
            }
            break;
        case SHDISP_BDIC_TRI_LED_MODE_OFF:
            break;
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
        case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        default:
            SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            shdisp_bdic_PD_TRI_LED_lposc_off();
            break;
        }

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
#ifdef SHDISP_SYSFS_LED
            if (param == CURRENT_SET) {
                shdisp_bdic_LD_set_led_on_table(rgb_current1);
            } else
#endif /* SHDISP_SYSFS_LED */
            shdisp_bdic_LD_set_led_fix_on_table(led_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
        } else {
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            shdisp_bdic_PD_TRI_LED_set_chdig();
            shdisp_bdic_PD_TRI_LED_set_anime();
#ifndef SHDISP_COLOR_LED_TWIN
            shdisp_bdic_PD_TRI_LED_anime_start();
#endif  /* SHDISP_COLOR_LED_TWIN */
        }
        shdisp_bdic_tri_led_before_mode = shdisp_bdic_tri_led_mode;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        shdisp_bdic_tri_led_mode        = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        shdisp_bdic_PD_TRI_LED_lposc_off();
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_HISPEED;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_STANDARD;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_WAVE;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FLASH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_EMOPATTERN:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_AURORA;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_RAINBOW;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_ILLUMI_TRIPLE_COLOR:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

#endif  /* SHDISP_ANIME_COLOR_LED */

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

#ifdef SHDISP_SYSFS_DIM
    case SHDISP_BDIC_REQ_LED_DIM:
        shdisp_led_dim = param;
        break;
#endif /* SHDISP_SYSFS_DIM */
    default:
        break;
    }

    return;
}

#ifndef SHDISP_COLOR_LED_TWIN
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_anime_start                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_anime_start(void)
{
    unsigned char timer_val = 0;

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
#endif  /* SHDISP_ANIME_COLOR_LED */
        timer_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
        timer_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);
        shdisp_bdic_led_ani_on[1].data  = timer_val;
        SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);
        break;

#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on_triple_color);
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */

    default:
        break;
    }

    return;
}
#endif  /* SHDISP_COLOR_LED_TWIN */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_set_anime(void)
{
    unsigned char ch_set1_val;
    unsigned char ch_set2_val;

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set1_val = 0x46;
        if (shdisp_bdic_tri_led_ontime > SHDISP_TRI_LED_ONTIME_TYPE7) {
            ch_set2_val = SHDISP_TRI_LED_ONTIME_TYPE1 + (shdisp_bdic_tri_led_ontime - SHDISP_TRI_LED_ONTIME_TYPE7);
        } else {
            ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        }
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        ch_set1_val = 0x06;
        ch_set2_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH0_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH0_SET2, ch_set2_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH1_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH1_SET2, ch_set2_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH2_SET1, ch_set1_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH2_SET2, ch_set2_val);
        break;
#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_high_speed_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_standard_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_breath_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_long_breath_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_wave_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_flash_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_emopattern_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_aurora_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_rainbow_on);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        SHDISP_BDIC_REGSET(shdisp_bdic_illumi_triple_color_1st);
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_set_chdig(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int clrvari = led_state_str.bdic_clrvari_index;
    int dim_index = 0;
    unsigned char color_index;
    unsigned char *pAnime_tbl[3] = {NULL, NULL, NULL};

    color_index = shdisp_bdic_tri_led_color;

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */

    switch (shdisp_bdic_tri_led_mode) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        ret = SHDISP_RESULT_FAILURE;
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_tbl[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_tbl[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_tbl[dim_index][clrvari][0][color_index]));
        if ((shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_BLINK) &&
            (shdisp_bdic_tri_led_ontime > SHDISP_TRI_LED_ONTIME_TYPE7)) {
            pAnime_tbl[0] = pAnime_tbl[2] = pAnime_tbl[1];
        }
        break;

#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_high_speed_tbl[dim_index][clrvari][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl[dim_index][clrvari][2]));
        break;
    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl[dim_index][clrvari][2]));
        break;
#endif /* SHDISP_ILLUMI_COLOR_LED */

    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl[dim_index][clrvari][2]));
        break;
#endif /* SHDISP_ANIME_COLOR_LED */

    default:
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_PD_TRI_LED_write_chdig(BDIC_REG_CH0_A, pAnime_tbl);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_write_chdig                                        */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_write_chdig(int address, unsigned char *anime_tbl[3])
{
    unsigned char wBuf[9];

    memset(wBuf, 0, sizeof(wBuf));
    shdisp_bdic_API_IO_bank_set(0x00);

    if (anime_tbl[0] != NULL) {
        memcpy(&wBuf[0], anime_tbl[0], 3);
    }

    if (anime_tbl[1] != NULL) {
        memcpy(&wBuf[3], anime_tbl[1], 3);
    }

    if (anime_tbl[2] != NULL) {
        memcpy(&wBuf[6], anime_tbl[2], 3);
    }

    shdisp_bdic_API_IO_multi_write_reg(address, wBuf, 9);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_lposc_off                                          */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_lposc_off(void)
{
    SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_get_clrvari_index                                  */
/* ------------------------------------------------------------------------- */
static int shdisp_bdic_PD_TRI_LED_get_clrvari_index(int clrvari)
{
    int i = 0;

    for (i = 0; i < SHDISP_COL_VARI_KIND; i++) {
        if ((int)shdisp_clrvari_index[i] == clrvari) {
            break;
        }
    }
    if (i >= SHDISP_COL_VARI_KIND) {
        clrvari = SHDISP_COL_VARI_DEFAULT;
        for (i = 0; i < SHDISP_COL_VARI_KIND; i++) {
            if ((int)shdisp_clrvari_index[i] == clrvari) {
                break;
            }
        }
    }
    return i;
}

#ifdef SHDISP_COLOR_LED_TWIN
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control_twin                                       */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_control_twin(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_TRI_LED_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_TRI_LED_STANDBY:
        break;

    case SHDISP_BDIC_REQ_TRI_LED_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_TRI_LED_START tri_led_mode_twin=%d"
                       , shdisp_bdic_tri_led_mode_twin);

        if (shdisp_bdic_tri_led_mode_twin == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
#ifdef SHDISP_SYSFS_LED
            if (param == CURRENT_SET) {
                shdisp_bdic_LD_set_led_on_table_twin(rgb_current2);
            } else
#endif /* SHDISP_SYSFS_LED */
            shdisp_bdic_LD_set_led_fix_on_table_twin(led_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
        } else {
            shdisp_bdic_PD_TRI_LED_set_chdig_twin();
            shdisp_bdic_PD_TRI_LED_set_anime_twin();
            shdisp_bdic_PD_TRI_LED_anime_start_twin();
        }
        break;

    case SHDISP_BDIC_REQ_TRI_LED_STOP:
        shdisp_bdic_tri_led_mode_twin        = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_HISPEED:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_HISPEED;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_STANDARD:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_STANDARD;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BREATH:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_LONG_BREATH:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_WAVE:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_WAVE;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FLASH:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_FLASH;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_EMOPATTERN:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_AURORA:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_AURORA;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_RAINBOW:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_RAINBOW;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
    case SHDISP_BDIC_REQ_ILLUMI_TRIPLE_COLOR:
        shdisp_bdic_tri_led_mode_twin  = SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */
    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_anime_start_twin                                   */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_anime_start_twin(void)
{
    unsigned char timer_val = 0;

    switch (shdisp_bdic_tri_led_mode_twin) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
#endif  /* SHDISP_ANIME_COLOR_LED */
        timer_val  = (unsigned char)(shdisp_bdic_tri_led_interval << 4);
        timer_val |= (unsigned char)(shdisp_bdic_tri_led_count & 0x07);
        shdisp_bdic_led_ani_on_twin[1].data  = timer_val;
        shdisp_bdic_led_ani_on_twin[2].data  = timer_val;
        SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on_twin);
        break;

#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on_triple_color_twin);
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_anime_twin                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_set_anime_twin(void)
{
    unsigned char ch_set3_val;
    unsigned char ch_set4_val;

    switch (shdisp_bdic_tri_led_mode_twin) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        ch_set3_val = 0x46;
        if (shdisp_bdic_tri_led_ontime > SHDISP_TRI_LED_ONTIME_TYPE7) {
            ch_set4_val = SHDISP_TRI_LED_ONTIME_TYPE1 + (shdisp_bdic_tri_led_ontime - SHDISP_TRI_LED_ONTIME_TYPE7);
        } else {
            ch_set4_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        }
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH3_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH3_SET2, ch_set4_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH4_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH4_SET2, ch_set4_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH5_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH5_SET2, ch_set4_val);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        ch_set3_val = 0x06;
        ch_set4_val = (unsigned char)(shdisp_bdic_tri_led_ontime);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH3_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH3_SET2, ch_set4_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH4_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH4_SET2, ch_set4_val);
        shdisp_bdic_API_IO_msk_bit_reg(BDIC_REG_CH5_SET1, ch_set3_val, 0x6F);
        shdisp_bdic_API_IO_write_reg(BDIC_REG_CH5_SET2, ch_set4_val);
        break;

#ifdef SHDISP_ANIME_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_high_speed_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_standard_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_breath_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_long_breath_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_wave_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_flash_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_emopattern_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_aurora_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_rainbow_on_twin);
        break;

    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        SHDISP_BDIC_REGSET(shdisp_bdic_illumi_triple_color_1st_twin);
        break;
#endif  /* SHDISP_ANIME_COLOR_LED */
    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_set_chdig_twin                                     */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_PD_TRI_LED_set_chdig_twin(void)
{
    int ret = SHDISP_RESULT_SUCCESS;
    int clrvari = led_state_str.bdic_clrvari_index;
    int dim_index = 0;
    unsigned char color_index;
    unsigned char *pAnime_tbl[3] = {NULL, NULL, NULL};

    color_index = shdisp_bdic_tri_led_color;

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */

    switch (shdisp_bdic_tri_led_mode_twin) {
    case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
        ret = SHDISP_RESULT_FAILURE;
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BLINK:
    case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
    case SHDISP_BDIC_TRI_LED_MODE_TRIPLE_COLOR:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_tbl_twin[dim_index][clrvari][0][color_index]));
        if ((shdisp_bdic_tri_led_mode_twin == SHDISP_BDIC_TRI_LED_MODE_BLINK) &&
            (shdisp_bdic_tri_led_ontime > SHDISP_TRI_LED_ONTIME_TYPE7)) {
            pAnime_tbl[0] = pAnime_tbl[2] = pAnime_tbl[1];
        }
        break;

#ifdef SHDISP_ANIME_COLOR_LED
#ifdef SHDISP_ILLUMI_COLOR_LED
    case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
    case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_high_speed_tbl_twin[dim_index][clrvari][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl_twin[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl_twin[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_breath_tbl_twin[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl_twin[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl_twin[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_long_breath_tbl_twin[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl_twin[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl_twin[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_wave_tbl_twin[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl_twin[dim_index][clrvari][0][color_index]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl_twin[dim_index][clrvari][1][color_index]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_flash_tbl_twin[dim_index][clrvari][2][color_index]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl_twin[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl_twin[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_aurora_tbl_twin[dim_index][clrvari][2]));
        break;

    case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl_twin[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl_twin[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_rainbow_tbl_twin[dim_index][clrvari][2]));
        break;
#endif /* SHDISP_ILLUMI_COLOR_LED */

    case SHDISP_BDIC_TRI_LED_MODE_EMOPATTERN:
        pAnime_tbl[0] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl_twin[dim_index][clrvari][0]));
        pAnime_tbl[1] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl_twin[dim_index][clrvari][1]));
        pAnime_tbl[2] = (unsigned char *)(&(shdisp_triple_led_anime_emopattern_tbl_twin[dim_index][clrvari][2]));
        break;
#endif /* SHDISP_ANIME_COLOR_LED */

    default:
        ret = SHDISP_RESULT_FAILURE;
        break;
    }

    if (ret == SHDISP_RESULT_SUCCESS) {
        shdisp_bdic_PD_TRI_LED_write_chdig(BDIC_REG_CH3_A, pAnime_tbl);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_on_table_twin                                  */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_fix_on_table_twin(int clr_vari, int color)
{
    unsigned char *pTriLed;
    int dim_index = 0;

#ifdef SHDISP_SYSFS_DIM
    if (shdisp_led_dim == SHDISP_LED_SYSFS_DIM_ON) {
        dim_index = SHDISP_LED_SYSFS_DIM_ON;
    }
#endif /* SHDISP_SYSFS_DIM */

    pTriLed = (unsigned char *)(&(shdisp_triple_led_tbl_twin[dim_index][clr_vari][color]));

    shdisp_bdic_LD_set_led_on_table_twin(pTriLed);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_on_table_twin                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_on_table_twin(unsigned char *rgb_current)
{

    unsigned char *pTriLed;
    shdisp_bdicRegSetting_t led_fix_on[ARRAY_SIZE(shdisp_bdic_led_fix_on_twin)];

    memcpy(led_fix_on, shdisp_bdic_led_fix_on_twin, sizeof(shdisp_bdic_led_fix_on_twin));

    if (rgb_current == NULL) {
        SHDISP_DEBUG("[warning] parameter is NULL");
        return;
    }

    pTriLed = (unsigned char *)rgb_current;

    led_fix_on[SHDISP_LED_FIX_ONR].data = *(pTriLed + 0);
    led_fix_on[SHDISP_LED_FIX_ONG].data = *(pTriLed + 1);
    led_fix_on[SHDISP_LED_FIX_ONB].data = *(pTriLed + 2);
    SHDISP_BDIC_REGSET(led_fix_on);
}

#ifdef SHDISP_SYSFS_LED
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_current_table_twin                             */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_LD_set_led_fix_current_table_twin(unsigned char *rgb_current)
{
    unsigned char *pTriLed;
    shdisp_bdicRegSetting_t led_current[ARRAY_SIZE(shdisp_bdic_led_current_twin)];

    memcpy(led_current, shdisp_bdic_led_current_twin, sizeof(shdisp_bdic_led_current_twin));

    if (rgb_current == NULL) {
        SHDISP_DEBUG("[warning] parameter is NULL");
        return;
    }

    pTriLed = (unsigned char *)rgb_current;

    led_current[0].data = *(pTriLed + 0);
    led_current[1].data = *(pTriLed + 1);
    led_current[2].data = *(pTriLed + 2);
    SHDISP_BDIC_REGSET(led_current);
}
#endif /* SHDISP_SYSFS_LED */

#if defined(CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_TRI_LED_INFO_output_twin                                      */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_TRI_LED_INFO_output_twin(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned short   shdisp_log_lv_bk;
    size_t  size;

    size  = (BDIC_REG_TIMER2 - BDIC_REG_SEQ_ANIME + 1);
    size += (BDIC_REG_CH5_C - BDIC_REG_CH3_SET1 + 1);

    pbuf = (unsigned char *)kzalloc(size, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%ld", size);
        return;
    }

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;
    shdisp_bdic_API_IO_bank_set(0x00);

    p = pbuf;
    for (idx = BDIC_REG_SEQ_ANIME; idx <= BDIC_REG_TIMER2; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    for (idx = BDIC_REG_CH3_SET1; idx <= BDIC_REG_CH5_C; idx++) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_log_lv = shdisp_log_lv_bk;

    printk("[SHDISP] TRI-LED-TWIN INFO ->>\n");
    printk("[SHDISP] led_state_str.handset_color       = %d.\n", led_state_str.handset_color);
    printk("[SHDISP] led_state_str.bdic_clrvari_index  = %d.\n", led_state_str.bdic_clrvari_index);
    printk("[SHDISP] led_state_str.bdic_chipver        = %d.\n", led_state_str.bdic_chipver);
    printk("[SHDISP] shdisp_bdic_tri_led_color         = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode_twin     = %d.\n", shdisp_bdic_tri_led_mode_twin);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime        = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval      = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count         = %d.\n", shdisp_bdic_tri_led_count);

#ifdef SHDISP_SYSFS_LED
    printk("[SHDISP] rgb_current2[0]                   = %d.\n", rgb_current2[0]);
    printk("[SHDISP] rgb_current2[1]                   = %d.\n", rgb_current2[1]);
    printk("[SHDISP] rgb_current2[2]                   = %d.\n", rgb_current2[2]);
#endif /* SHDISP_SYSFS_LED */

    p = pbuf;
    printk("[SHDISP] BDIC_REG_TIMER_SETTING 0x%2X: %02x %02x %02x\n", BDIC_REG_SEQ_ANIME, *p, *(p + 1), *(p + 2));
    p += 3;
    printk("[SHDISP] BDIC_REG_LED_TWIN_SETTING  0x%2X: %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH3_SET1, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6));
    p += 7;
    printk("[SHDISP] BDIC_REG_LED_TWIN_CURRENT  0x%2X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                BDIC_REG_CH3_A, *p, *(p + 1), *(p + 2), *(p + 3), *(p + 4), *(p + 5), *(p + 6), *(p + 7), *(p + 8));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED-TWIN INFO <<-\n");
    return;
}

#if defined(SHDISP_ILLUMI_TRIPLE_COLOR_LED) && defined(SHDISP_ANIME_COLOR_LED)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_illumi_triple_color_INFO_output                               */
/* ------------------------------------------------------------------------- */
static void shdisp_bdic_illumi_triple_color_INFO_output(void)
{
    int i;

    printk("[SHDISP] illumi-triple-color INFO ->>\n");

    printk("[SHDISP] illumi_state.running_state        = %d.\n", illumi_state.running_state);
    printk("[SHDISP] illumi_state.illumi_state         = %d.\n", illumi_state.illumi_state);

    for (i = ILLUMI_FRAME_FIRST; i != ILLUMI_FRAME_MAX; i++) {
        printk("[SHDISP] illumi_state.colors[%d]            = %d.\n", i, illumi_state.colors[i]);
    }

    printk("[SHDISP] illumi_state.count                = %d.\n", illumi_state.count);

    printk("[SHDISP] illumi-triple-color INFO <<-\n");
}
#endif /* SHDISP_ILLUMI_TRIPLE_COLOR_LED && SHDISP_ANIME_COLOR_LED */
#endif /* CONFIG_ANDROID_ENGINEERING */
#endif /* SHDISP_COLOR_LED_TWIN */


/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
