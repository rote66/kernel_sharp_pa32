/* drivers/sharp/shdisp/shdisp_bl71y8_main.h  (Display Driver)
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
#ifndef SHDISP_BL71Y8_MAIN_H
#define SHDISP_BL71Y8_MAIN_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BDIC_CHIPVER_0                       (0)
#define SHDISP_BDIC_CHIPVER_1                       (1)

#define SHDISP_BDIC_VERSION71                       (0x08)
#define SHDISP_BDIC_GET_CHIPVER(version)            ((version >> 4) & 0x0F)
#define SHDISP_ALS_SENSOR_ADJUST_STATUS_COMPLETED   (0x90)
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)                               (sizeof(x) / sizeof((x)[0]))
#endif /* ARRAY_SIZE */
#define SHDISP_BDIC_REGSET(x)                       (shdisp_bdic_API_seq_regset(x, ARRAY_SIZE(x)))

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
enum {
    SHDISP_BDIC_STR,
    SHDISP_BDIC_SET,
    SHDISP_BDIC_CLR,
    SHDISP_BDIC_RMW,
    SHDISP_BDIC_STRM,
    SHDISP_BDIC_BANK,
    SHDISP_BDIC_WAIT,
    SHDISP_ALS_STR,
    SHDISP_ALS_RMW,
    SHDISP_BDIC_CHKWR,
    SHDISP_ALS_STRM,
    SHDISP_ALS_STRMS
};

typedef struct {
    unsigned char   addr;
    unsigned char   flg;
    unsigned char   data;
    unsigned char   mask;
    unsigned long   wait;
} shdisp_bdicRegSetting_t;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_boot_init(int *chipver);
int  shdisp_bdic_API_boot_init_2nd(void);
int shdisp_bdic_API_kerl_init(void);
int  shdisp_bdic_API_shutdown(void);
int  shdisp_bdic_API_set_active(void);
int  shdisp_bdic_API_set_standby(void);
unsigned char shdisp_bdic_API_correct_proh_val(unsigned char);
void shdisp_bdic_API_als_sensor_adjust(struct shdisp_photo_sensor_adj *adj);
void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, \
                                        struct shdisp_photo_sensor_adj *adj_out);
int  shdisp_bdic_API_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size);
int  shdisp_bdic_API_IO_write_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
int  shdisp_bdic_API_IO_read_reg(unsigned char reg, unsigned char *val);
int  shdisp_bdic_API_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
int  shdisp_bdic_API_IO_set_bit_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_IO_clr_bit_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_IO_bank_set(unsigned char val);
int  shdisp_bdic_API_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
#if defined(USE_LINUX) || defined(SHDISP_APPSBL)
int  shdisp_bdic_API_IO_i2c_transfer(struct shdisp_bdic_i2c_msg *msg);
int  shdisp_bdic_API_IO_psals_write_reg(unsigned char reg, unsigned char val);
int  shdisp_bdic_API_IO_psals_read_reg(unsigned char reg, unsigned char *val);
int  shdisp_bdic_API_IO_psals_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char mask);
int  shdisp_bdic_API_IO_psals_burst_write_reg(unsigned char *wval, unsigned char dataNum);
int  shdisp_bdic_API_IO_psals_burst_read_reg(unsigned char reg, unsigned char *rval, unsigned char dataNum);
#endif /* defined(USE_LINUX) || defined(SHDISP_APPSBL) */

#endif  /* SHDISP_BL71Y8_MAIN_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
