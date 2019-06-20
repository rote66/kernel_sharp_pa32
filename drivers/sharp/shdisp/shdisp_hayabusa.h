/* drivers/sharp/shdisp/shdisp_hayabusa.h  (Display Driver)
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
#ifndef SHDISP_HAYABUSA_H
#define SHDISP_HAYABUSA_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_HAYABUSA_VDD

enum {
    VER_CUT1_0      = 1,
    VER_CUT1_0_FIB,
    VER_CUT1_1,
    VER_CUT2_0
};

#define VER_CUT_DEFAULT     VER_CUT2_0

#define ADDR_ID41           (0x3A)
#define ADDR_WID1           (0x44)
#define VAL_ID41_1          (0x01)
#define VAL_ID41_2          (0x02)
#define VAL_ID41_3          (0x03)
#define VAL_WID1_0          (0x00)
#define VAL_WID1_1          (0x01)

#define VAL_MFR_120HZ       (0x00)
#define VAL_MFR_60HZ        (0x01)
#define VAL_MFR_15HZ        (0x07)
#define VAL_MFR_1HZ         (0x77)
#define VAL_MFR_RELEASE     (0xFF)
#define VAL_MFR_DEFAULT     VAL_MFR_60HZ

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
struct shdisp_panel_operations *shdisp_hayabusa_API_create(void);
#endif /* SHDISP_HAYABUSA_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */

