/* drivers/sharp/shdisp/shdisp_type.h  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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
#ifndef SHDISP_TYPE_H
#define SHDISP_TYPE_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_DRIVER_IS_NOT_INITIALIZED,
    SHDISP_DRIVER_IS_INITIALIZED,
    NUM_SHDISP_DRIVER_STATUS
};

enum {
    SHDISP_DISP_PIC_ADJ_MODE_00,
    SHDISP_DISP_PIC_ADJ_MODE_01,
    SHDISP_DISP_PIC_ADJ_MODE_02,
    SHDISP_DISP_PIC_ADJ_MODE_03,
    SHDISP_DISP_PIC_ADJ_MODE_04,
    SHDISP_DISP_PIC_ADJ_MODE_05,
    SHDISP_DISP_PIC_ADJ_MODE_06,
    SHDISP_DISP_PIC_ADJ_MODE_07,
    SHDISP_DISP_PIC_ADJ_MODE_08,
    NUM_SHDISP_DISP_PIC_ADJ_MODE,
    SHDISP_DISP_PIC_ADJ_MODE_NO_CONVERT
};

#endif /* SHDISP_TYPE_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
