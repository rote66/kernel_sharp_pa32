/* drivers/sharp/shdisp/shdisp_system.h  (Display Driver)
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
#ifndef SHDISP_SYSTEM_H
#define SHDISP_SYSTEM_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/pinctrl/consumer.h>

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_GPIO_CTL_LOW                 (0)
#define SHDISP_GPIO_CTL_HIGH                (1)



#define SHDISP_GPIO_NUM_BL_RST_N            (14)
#define SHDISP_GPIO_NUM_PANEL_RST_N         (8)
#ifdef SHDISP_HAYABUSA_ADDITIONAL_VDD
#define SHDISP_GPIO_NUM_PANEL_VDD           (107)
#endif /* SHDISP_HAYABUSA_ADDITIONAL_VDD */

#define SHDISP_GPIO_NUM_MIPI_ERROR          (34)

enum {
    SHDISP_HW_REV_ES0,
    SHDISP_HW_REV_ES1,
    SHDISP_HW_REV_ES15,
    SHDISP_HW_REV_PP1,
    SHDISP_HW_REV_PP15,
    SHDISP_HW_REV_PP2,
    SHDISP_HW_REV_PP25,
    SHDISP_HW_REV_MP,
};

#define SHDISP_HW_REV_BIT(bit3, bit2, bit1) (((bit3) << 2) | ((bit2) << 1) | (bit1))

#define SHDISP_HW_REV_BIT_ES0               SHDISP_HW_REV_BIT(0, 0, 0)
#define SHDISP_HW_REV_BIT_ES1               SHDISP_HW_REV_BIT(0, 0, 1)
#define SHDISP_HW_REV_BIT_ES15              SHDISP_HW_REV_BIT(0, 1, 0)
#define SHDISP_HW_REV_BIT_PP1               SHDISP_HW_REV_BIT(0, 1, 1)
#define SHDISP_HW_REV_BIT_PP15              SHDISP_HW_REV_BIT(1, 0, 0)
#define SHDISP_HW_REV_BIT_PP2               SHDISP_HW_REV_BIT(1, 0, 1)
#define SHDISP_HW_REV_BIT_PP25              SHDISP_HW_REV_BIT(1, 1, 0)
#define SHDISP_HW_REV_BIT_MP                SHDISP_HW_REV_BIT(1, 1, 1)

#define SHDISP_HW_REV_TABLE_SIZE            (8)
#define SHDISP_HW_REV_TABLE_ITEMS                     \
        {SHDISP_HW_REV_BIT_ES0,  SHDISP_HW_REV_ES0},  \
        {SHDISP_HW_REV_BIT_ES1,  SHDISP_HW_REV_ES1},  \
        {SHDISP_HW_REV_BIT_ES15, SHDISP_HW_REV_ES15}, \
        {SHDISP_HW_REV_BIT_PP1,  SHDISP_HW_REV_PP1},  \
        {SHDISP_HW_REV_BIT_PP15, SHDISP_HW_REV_PP15}, \
        {SHDISP_HW_REV_BIT_PP2,  SHDISP_HW_REV_PP2},  \
        {SHDISP_HW_REV_BIT_PP25, SHDISP_HW_REV_PP25}, \
        {SHDISP_HW_REV_BIT_MP,   SHDISP_HW_REV_MP},   \

#define SHDISP_BDIC_I2C_DEVNAME             ("sharp,bdic_i2c")
#define SHDISP_SENSOR_DEVNAME               ("sharp,sensor_i2c")

#define WAIT_1FRAME_US                      (16666)

#define SHDISP_DEBUGFLG_BIT_KERNEL_LOG      (0x01)
#define SHDISP_DEBUGFLG_BIT_USER_LOG        (0x02)
#define SHDISP_DEBUGFLG_BIT_APPSBL_LOG      (0x04)
#define SHDISP_DEBUGFLG_BIT_SBL1_LOG        (0x08)
#define SHDISP_DEBUGFLG_BIT_MDP_DUMP        (0x10)

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */
#define SHDISP_IRQ_MAX_KIND                 (4)

enum {
    SHDISP_HOST_CTL_CMD_LCD_CLK_START,
    SHDISP_HOST_CTL_CMD_LCD_CLK_STOP,
    SHDISP_HOST_CTL_CMD_LCD_CLK_INIT,
    SHDISP_HOST_CTL_CMD_LCD_CLK_EXIT,
    NUM_SHDISP_HOST_CTL_CMD
};

enum {
    SHDISP_IRQ_DISABLE,
    SHDISP_IRQ_ENABLE,
    NUM_SHDISP_IRQ_CMD
};

struct shdisp_gpio_info {
    int gpio;
    struct pinctrl *pinctrl;
    struct pinctrl_state *state_active;
    struct pinctrl_state *state_suspend;
};

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev);
int  shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)(int , void *));
void shdisp_SYS_API_free_irq(void);
void shdisp_SYS_API_set_irq_init(void);
int  shdisp_SYS_API_set_irq(int enable);

unsigned char shdisp_SYS_API_get_debugflg(void);


int shdisp_SYS_API_check_diag_mode(void);

#endif /* SHDISP_SYSTEM_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
