/* drivers/sharp/shdisp/shdisp_bl71y8.h  (Display Driver)
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
#ifndef SHDISP_BL71Y8_H
#define SHDISP_BL71Y8_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_GPIO_COG_RESET          (0)

#define SHDISP_BDIC_GPIO_LOW                (0)
#define SHDISP_BDIC_GPIO_HIGH               (1)

#define SHDISP_BDIC_GPIO_GPOD0              (0)
#define SHDISP_BDIC_GPIO_GPOD1              (1)
#define SHDISP_BDIC_GPIO_GPOD2              (2)
#define SHDISP_BDIC_GPIO_GPOD3              (3)
#define SHDISP_BDIC_GPIO_GPOD4              (4)
#define SHDISP_BDIC_GPIO_GPOD5              (5)

#define SHDISP_BDIC_I2C_SLAVE_ADDR          (0xA8)
#define SHDISP_BDIC_I2C_WBUF_MAX            (6)
#define SHDISP_BDIC_I2C_RBUF_MAX            (6)

#define SHDISP_BDIC_INT_GFAC_GFAC0          (0x00000001)
#define SHDISP_BDIC_INT_GFAC_GFAC1          (0x00000002)
#define SHDISP_BDIC_INT_GFAC_GFAC2          (0x00000004)
#define SHDISP_BDIC_INT_GFAC_PS             (0x00000008)
#define SHDISP_BDIC_INT_GFAC_GFAC4          (0x00000010)
#define SHDISP_BDIC_INT_GFAC_ALS            (0x00000100)
#define SHDISP_BDIC_INT_GFAC_PS2            (0x00000200)
#define SHDISP_BDIC_INT_GFAC_OPTON          (0x00000400)
#define SHDISP_BDIC_INT_GFAC_CPON           (0x00000800)
#define SHDISP_BDIC_INT_GFAC_ANIME          (0x00001000)
#define SHDISP_BDIC_INT_GFAC_TEST1          (0x00002000)
#define SHDISP_BDIC_INT_GFAC_DCDC2_ERR      (0x00004000)
#define SHDISP_BDIC_INT_GFAC_TSD            (0x00008000)
#define SHDISP_BDIC_INT_GFAC_TEST2          (0x00010000)
#define SHDISP_BDIC_INT_GFAC_TEST3          (0x00020000)
#define SHDISP_BDIC_INT_GFAC_DET            (0x00040000)
#define SHDISP_BDIC_INT_GFAC_I2C_ERR        (0x00080000)
#define SHDISP_BDIC_INT_GFAC_TEST4          (0x00100000)
#define SHDISP_BDIC_INT_GFAC_OPTSEL         (0x00200000)
#define SHDISP_BDIC_INT_GFAC_TEST5          (0x00400000)
#define SHDISP_BDIC_INT_GFAC_TEST6          (0x00800000)
#define SHDISP_BDIC_INT_GFAC_ALS_TRG1       (0x01000000)
#define SHDISP_BDIC_INT_GFAC_ALS_TRG2       (0x02000000)

#define SHDISP_BDIC_SENSOR_TYPE_PHOTO       (0x01)
#define SHDISP_BDIC_SENSOR_TYPE_PROX        (0x02)
#define SHDISP_BDIC_SENSOR_SLAVE_ADDR       (0x39)

#define SHDISP_OPT_CHANGE_WAIT_TIME         (150)

#define SHDISP_BDIC_GINF3_DCDC1_OVD         (0x20)

#ifdef SHDISP_ALS_INT
#define SHDISP_BDIC_OPT_L_EDGE_EN           (0x20)
#define SHDISP_BDIC_OPT_H_EDGE_EN           (0x40)
#define SHDISP_BDIC_OPT_TH_SIDE             (0x80)

#define SHDISP_BDIC_OPT_TABLE1_IMR          (0x10)
#define SHDISP_BDIC_OPT_TABLE2_IMR          (0x20)
#endif /* SHDISP_ALS_INT */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_REQ_NONE = 0,
    SHDISP_BDIC_REQ_ACTIVE,
    SHDISP_BDIC_REQ_STANDBY,
    SHDISP_BDIC_REQ_STOP,
    SHDISP_BDIC_REQ_START,
    SHDISP_BDIC_REQ_BKL_SET_MODE_OFF,
    SHDISP_BDIC_REQ_BKL_SET_MODE_FIX,
    SHDISP_BDIC_REQ_BKL_SET_MODE_AUTO,
    SHDISP_BDIC_REQ_PHOTO_SENSOR_CONFIG,
    SHDISP_BDIC_REQ_BKL_DTV_OFF,
    SHDISP_BDIC_REQ_BKL_DTV_ON,
    SHDISP_BDIC_REQ_BKL_SET_EMG_MODE,
#ifdef SHDISP_LOWBKL
    SHDISP_BDIC_REQ_BKL_LOWBKL_OFF,
    SHDISP_BDIC_REQ_BKL_LOWBKL_ON,
#endif /* SHDISP_LOWBKL */
    SHDISP_BDIC_REQ_BKL_CHG_OFF,
    SHDISP_BDIC_REQ_BKL_CHG_ON,
#ifdef SHDISP_TRV_NM2
    SHDISP_BDIC_REQ_BKL_TRV_REQUEST,
#endif /* SHDISP_TRV_NM2 */
    SHDISP_BDIC_REQ_BKL_ON,
    SHDISP_BDIC_REQ_BKL_FIX_START,
    SHDISP_BDIC_REQ_BKL_AUTO_START,
    SHDISP_BDIC_REQ_BKL_SET_LED_VALUE,
    SHDISP_BDIC_REQ_BKL_SET_OPT_VALUE
};

enum {
    SHDISP_BDIC_PWR_STATUS_OFF,
    SHDISP_BDIC_PWR_STATUS_STANDBY,
    SHDISP_BDIC_PWR_STATUS_ACTIVE,
    NUM_SHDISP_BDIC_PWR_STATUS
};

enum {
    SHDISP_BDIC_DEV_TYPE_LCD_BKL,
    SHDISP_BDIC_DEV_TYPE_LCD_PWR,
    SHDISP_BDIC_DEV_TYPE_TRI_LED,
    SHDISP_BDIC_DEV_TYPE_TRI_LED_ANIME,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_APP,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_LUX,
    SHDISP_BDIC_DEV_TYPE_PHOTO_SENSOR_BKL,
    SHDISP_BDIC_DEV_TYPE_PROX_SENSOR,
    NUM_SHDISP_BDIC_DEV_TYPE
};

enum {
    SHDISP_BDIC_DEV_PWR_OFF,
    SHDISP_BDIC_DEV_PWR_ON,
    NUM_SHDISP_BDIC_DEV_PWR
};

enum {
    SHDISP_MAIN_BKL_DEV_TYPE_APP,
    SHDISP_MAIN_BKL_DEV_TYPE_APP_AUTO,
    NUM_SHDISP_MAIN_BKL_DEV_TYPE
};

enum {
    SHDISP_BDIC_IRQ_TYPE_NONE,
    SHDISP_BDIC_IRQ_TYPE_ALS,
    SHDISP_BDIC_IRQ_TYPE_PS,
    SHDISP_BDIC_IRQ_TYPE_DET,
    SHDISP_BDIC_IRQ_TYPE_I2C_ERR,
#ifdef SHDISP_ALS_INT
    SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER,
    SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER1,
    SHDISP_BDIC_IRQ_TYPE_ALS_TRIGGER2,
#endif /* SHDISP_ALS_INT */
    NUM_SHDISP_BDIC_IRQ_TYPE
};

enum {
    SHDISP_MAIN_BKL_ADJ_RETRY0,
    SHDISP_MAIN_BKL_ADJ_RETRY1,
    SHDISP_MAIN_BKL_ADJ_RETRY2,
    SHDISP_MAIN_BKL_ADJ_RETRY3,
    NUM_SHDISP_MAIN_BKL_ADJ
};

enum {
    SHDISP_BDIC_MAIN_BKL_OPT_LOW,
    SHDISP_BDIC_MAIN_BKL_OPT_HIGH,
    NUM_SHDISP_BDIC_MAIN_BKL_OPT_MODE
};

enum {
    SHDISP_BDIC_PHOTO_LUX_TIMER_ON,
    SHDISP_BDIC_PHOTO_LUX_TIMER_OFF,
    NUM_SHDISP_BDIC_PHOTO_LUX_TIMER_SWITCH
};

enum {
    SHDISP_BDIC_LUX_JUDGE_IN,
    SHDISP_BDIC_LUX_JUDGE_IN_CONTI,
    SHDISP_BDIC_LUX_JUDGE_OUT,
    SHDISP_BDIC_LUX_JUDGE_OUT_CONTI,
    SHDISP_BDIC_LUX_JUDGE_ERROR,
    NUM_SHDISP_BDIC_LUX_JUDGE
};

enum {
    SHDISP_BDIC_BL_PARAM_WRITE = 0,
    SHDISP_BDIC_BL_PARAM_READ,
    SHDISP_BDIC_BL_MODE_SET,
    SHDISP_BDIC_ALS_SET,
    SHDISP_BDIC_ALS_PARAM_WRITE,
    SHDISP_BDIC_ALS_PARAM_READ,
    SHDISP_BDIC_ALS_PARAM_SET,
    SHDISP_BDIC_CABC_CTL,
    SHDISP_BDIC_CABC_CTL_TIME_SET,
    SHDISP_BDIC_DEVICE_SET
};

enum {
    SHDISP_BDIC_BL_PWM_FIX_PARAM = 0,
    SHDISP_BDIC_BL_PWM_AUTO_PARAM
};

enum {
    SHDISP_BDIC_PSALS_RECOVERY_NONE = 0,
    SHDISP_BDIC_PSALS_RECOVERY_DURING,
    SHDISP_BDIC_PSALS_RECOVERY_RETRY_OVER
};

enum {
    SHDISP_BDIC_BKL_MODE_OFF = 0,
    SHDISP_BDIC_BKL_MODE_FIX,
    SHDISP_BDIC_BKL_MODE_AUTO
};

enum {
    SHDISP_BDIC_BKL_EMG_OFF,
    SHDISP_BDIC_BKL_EMG_ON_LEVEL0,
    SHDISP_BDIC_BKL_EMG_ON_LEVEL1,
};

enum {
    SHDISP_BKL_TBL_MODE_NORMAL,
    SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL0,
    SHDISP_BKL_TBL_MODE_EMERGENCY_LEVEL1,
    NUM_SHDISP_BKL_TBL_MODE
};

struct shdisp_bdic_state_str {
    int handset_color;
    int bdic_chipver;
    int bdic_clrvari_index;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
#ifdef SHDISP_TRV_NM2
    struct shdisp_trv_param trv_param;
#endif /* SHDISP_TRV_NM2 */
};

struct shdisp_bdic_bkl_ado_tbl {
    unsigned long range_low;
    unsigned long range_high;
    unsigned long param_a;
    long param_b;
};

struct shdisp_bdic_bkl_info {
    int mode;
    int param;
    int value;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_initialize(struct shdisp_bdic_state_str *state_str);
void shdisp_bdic_API_LCD_release_hw_reset(void);
void shdisp_bdic_API_LCD_set_hw_reset(void);
void shdisp_bdic_API_LCD_power_on(void);
void shdisp_bdic_API_LCD_power_off(void);
void shdisp_bdic_API_LCD_m_power_on(void);
void shdisp_bdic_API_LCD_m_power_off(void);
void shdisp_bdic_API_LCD_BKL_off(void);
void shdisp_bdic_API_LCD_BKL_fix_on(int param);
void shdisp_bdic_API_LCD_BKL_auto_on(int param);
void shdisp_bdic_API_LCD_BKL_get_param(struct shdisp_bdic_bkl_info *bkl_info);
void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp);
void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req);
void shdisp_bdic_API_LCD_BKL_dtv_on(void);
void shdisp_bdic_API_LCD_BKL_dtv_off(void);
void shdisp_bdic_API_LCD_BKL_set_emg_mode(int emg_mode);
void shdisp_bdic_API_LCD_BKL_chg_on(void);
void shdisp_bdic_API_LCD_BKL_chg_off(void);
#ifdef SHDISP_LOWBKL
void shdisp_bdic_API_LCD_BKL_lowbkl_on(void);
void shdisp_bdic_API_LCD_BKL_lowbkl_off(void);
#endif /* SHDISP_LOWBKL */
#ifdef SHDISP_TRV_NM2
int shdisp_bdic_API_LCD_BKL_trv_param(struct shdisp_trv_param param);
#endif /* SHDISP_TRV_NM2 */

int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned int *lux);
int  shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir);
#ifdef SHDISP_ALS_INT
int  shdisp_bdic_API_PHOTO_SENSOR_set_alsint(struct shdisp_photo_sensor_int_trigger *value);
int  shdisp_bdic_API_PHOTO_SENSOR_get_alsint(struct shdisp_photo_sensor_int_trigger *value);
#endif /* SHDISP_ALS_INT */
#ifdef SHDISP_LED_INT
int shdisp_bdic_API_led_auto_low_enable(bool enable);
int shdisp_bdic_API_led_auto_low_process(void);
#endif /* SHDISP_LED_INT */
int shdisp_bdic_API_PHOTO_SENSOR_get_light_info(struct shdisp_light_info *value);
int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg);
unsigned char shdisp_bdic_API_I2C_start_judge(void);
void shdisp_bdic_API_I2C_start_ctl(int flg);

int  shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val);
int  shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size);
int  shdisp_bdic_API_RECOVERY_check_restoration(void);
int  shdisp_bdic_API_RECOVERY_check_bdic_practical(void);
#if defined(CONFIG_ANDROID_ENGINEERING)
void shdisp_bdic_API_DBG_INFO_output(void);
void shdisp_bdic_API_OPT_INFO_output(void);
void shdisp_bdic_API_PSALS_INFO_output(void);
#endif /* CONFIG_ANDROID_ENGINEERING */

int  shdisp_bdic_API_IRQ_check_type(int irq_type);
void shdisp_bdic_API_IRQ_save_fac(void);
int  shdisp_bdic_API_IRQ_check_DET(void);
int  shdisp_bdic_API_IRQ_check_I2C_ERR(void);
int  shdisp_bdic_API_IRQ_check_fac(void);
int  shdisp_bdic_API_IRQ_get_fac(int iQueFac);
void shdisp_bdic_API_IRQ_Clear(void);
void shdisp_bdic_API_IRQ_i2c_error_Clear(void);
void shdisp_bdic_API_IRQ_det_fac_Clear(void);
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl);
void shdisp_bdic_API_IRQ_dbg_Clear_All(void);
void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC);
int  shdisp_bdic_API_als_sensor_pow_ctl(int dev_type, int power_mode);
int  shdisp_bdic_API_psals_power_on(void);
int  shdisp_bdic_API_psals_power_off(void);
int  shdisp_bdic_API_psals_ps_init_als_off(void);
int  shdisp_bdic_API_psals_ps_init_als_on(void);
int  shdisp_bdic_API_psals_ps_deinit_als_off(void);
int  shdisp_bdic_API_psals_ps_deinit_als_on(void);
int  shdisp_bdic_API_psals_als_init_ps_off(void);
int  shdisp_bdic_API_psals_als_init_ps_on(void);
int  shdisp_bdic_API_psals_als_deinit_ps_off(void);
int  shdisp_bdic_API_psals_als_deinit_ps_on(void);
int  shdisp_bdic_API_psals_is_recovery_successful(void);
void shdisp_bdic_API_set_prox_sensor_param(struct shdisp_prox_params *prox_params);
int  shdisp_bdic_API_get_lux_data(void);
void shdisp_bdic_API_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk);
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk);
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk);
int  shdisp_bdic_API_get_sensor_state(void);
void shdisp_bdic_API_RECOVERY_lux_data_backup(void);
void shdisp_bdic_API_RECOVERY_lux_data_restore(void);
void shdisp_bdic_API_ps_background(unsigned long state);
void shdisp_bdic_API_set_device_code(void);
int  shdisp_bdic_API_get_ave_ado(struct shdisp_ave_ado *ave_ado);
void shdisp_bdic_API_update_led_value(void);

#endif  /* SHDISP_BL71Y8_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
