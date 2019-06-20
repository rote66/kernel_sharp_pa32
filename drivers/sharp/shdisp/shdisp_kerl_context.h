/* include/sharp/shdisp_kerl_context.h  (Display Driver)
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
#ifndef SHDISP_KERL_CONTEXT_H
#define SHDISP_KERL_CONTEXT_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_boot_context.h>

struct shdisp_kernel_context {
    struct shdisp_boot_context boot_ctx;
    int driver_is_open;
    int driver_open_cnt;
    int driver_is_initialized;
    int shutdown_in_progress;
    int thermal_status;
    int usb_chg_status;
    int main_disp_status;
#ifdef SHDISP_TRV_NM2
    int trv_status;
#endif /* SHDISP_TRV_NM2 */
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
#ifdef SHDISP_SYSFS_LED
    struct shdisp_tri_led sysfs_led1;
#ifdef SHDISP_COLOR_LED_TWIN
    struct shdisp_tri_led sysfs_led2;
#endif /* SHDISP_COLOR_LED_TWIN */
#endif /* SHDISP_SYSFS_LED */
#ifdef SHDISP_LED_INT
    bool led_auto_low_enable;
#endif /* SHDISP_LED_INT */
    bool led_set_color_reject;
};

#endif /* SHDISP_KERL_CONTEXT_H */
