/* include/sharp/shdisp_define.h  (Display Driver)
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

#ifndef SHDISP_DEFINE_H
#define SHDISP_DEFINE_H

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#define SHDISP_TRV_NM2
#define SHDISP_LOWBKL


#define SHDISP_SYSFS_LED
#define SHDISP_SYSFS_DIM
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_ARCH_LYNX_DL90) || defined(FEATURE_SH_MODEL_DL90)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_FPS_HIGH_ENABLE
#define SHDISP_MDP_PIC_ADJ_ENABLE
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_PA32) || defined(FEATURE_SH_MODEL_PA32)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_PA
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_FPS_HIGH_ENABLE
#define SHDISP_MDP_PIC_ADJ_ENABLE
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#elif defined(CONFIG_ARCH_DECKARD_AL40) || defined(FEATURE_SH_MODEL_AL40)
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_AL
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_FPS_HIGH_ENABLE
#define SHDISP_MDP_PIC_ADJ_ENABLE
#define SHDISP_PICADJ_USE_QDCM
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#else  /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */
#define SHDISP_COLOR_LED_TWIN
#define SHDISP_DL
#define SHDISP_MODEL_FS

#define SHDISP_ALS_INT
#define SHDISP_LED_INT
#define SHDISP_ANIME_COLOR_LED
#define SHDISP_ILLUMI_COLOR_LED

#define USER_CONFIG_SHDISP_PANEL_HAYABUSA
#define SHDISP_PICADJ_USE_QDCM
#define DISABLE_SHDISP_DRIVER
#define SHDISP_MAIN_WIDTH  1080
#define SHDISP_MAIN_HEIGHT 1920
/* ------------------------------------------------------------------------- */
#endif  /* CONFIG_ARCH_XXX || FEATURE_SH_MODEL_XXX */



#ifndef CONFIG_SHDISP_PANEL_GEMINI
#if defined(SHDISP_FACTORY_MODE_ENABLE)
#if !defined(SHDISP_NOT_SUPPORT_DET)
#define SHDISP_NOT_SUPPORT_DET
#endif  /* !defined(SHDISP_NOT_SUPPORT_DET) */
#endif  /* defined(SHDISP_FACTORY_MODE_ENABLE) */
#endif  /* CONFIG_SHDISP_PANEL_GEMINI */

#endif /* SHDISP_DEFINE_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
