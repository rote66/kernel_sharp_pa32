/* drivers/sharp/shdisp/shdisp_system.c  (Display Driver)
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
#include <linux/gpio.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/qpnp/qpnp-adc.h>
#include <sharp/shdisp_kerl.h>
#include <sharp/sh_smem.h>
#include "shdisp_system.h"
#include "shdisp_dbg.h"
#include <sharp/sh_boot_manager.h>
#define  SHDISP_BOOT_D      SH_BOOT_D
#define  SHDISP_BOOT_F_F    SH_BOOT_F_F


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_INT_FLAGS (IRQF_TRIGGER_LOW | IRQF_DISABLED)
/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static unsigned int shdisp_int_irq_port = 0;
static struct platform_device *shdisp_int_irq_port_pdev = NULL;
static int shdisp_int_irq_port_staus = 0;
static spinlock_t shdisp_set_irq_spinlock;

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_port                                               */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_port(int irq_port, struct platform_device *pdev)
{
    shdisp_int_irq_port = irq_port;
    shdisp_int_irq_port_pdev = pdev;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_request_irq                                                */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_request_irq(irqreturn_t (*irq_handler)(int , void *))
{
    int ret = SHDISP_RESULT_SUCCESS;
    if ((irq_handler == NULL)
    ||  (shdisp_int_irq_port_pdev == NULL)) {
        return SHDISP_RESULT_FAILURE;
    }

    ret = devm_request_irq(&shdisp_int_irq_port_pdev->dev, shdisp_int_irq_port, *irq_handler,
                        SHDISP_INT_FLAGS,   "shdisp", NULL);

    if (ret == 0) {
        disable_irq(shdisp_int_irq_port);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_free_irq                                                   */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_free_irq(void)
{
    free_irq(shdisp_int_irq_port, NULL);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq_init                                               */
/* ------------------------------------------------------------------------- */
void shdisp_SYS_API_set_irq_init(void)
{
    spin_lock_init(&shdisp_set_irq_spinlock);
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_set_irq                                                    */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_set_irq(int enable)
{
    int ret = SHDISP_RESULT_SUCCESS;
    unsigned long flags = 0;

    spin_lock_irqsave( &shdisp_set_irq_spinlock, flags);

    if (enable == shdisp_int_irq_port_staus) {
        spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
        return SHDISP_RESULT_SUCCESS;
    }

    if (enable == SHDISP_IRQ_ENABLE) {
        enable_irq_wake(shdisp_int_irq_port);
        enable_irq(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else if (enable == SHDISP_IRQ_DISABLE) {
        disable_irq_nosync(shdisp_int_irq_port);
        disable_irq_wake(shdisp_int_irq_port);
        shdisp_int_irq_port_staus = enable;
    } else {
        SHDISP_ERR("<INVALID_VALUE> enable=%d", enable);
        ret = SHDISP_RESULT_FAILURE;
    }
    spin_unlock_irqrestore( &shdisp_set_irq_spinlock, flags);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_get_debugflg                                               */
/* ------------------------------------------------------------------------- */
unsigned char shdisp_SYS_API_get_debugflg(void)
{
    sharp_smem_common_type *smem = NULL;

    smem = sh_smem_get_common_address();
    if (smem) {
        return smem->shdiag_debugflg;
    } else {
        return 0;
    }
}

/* ------------------------------------------------------------------------- */
/* SUB ROUTINE                                                               */
/* ------------------------------------------------------------------------- */



/* ------------------------------------------------------------------------- */
/* shdisp_SYS_API_check_diag_mode                                            */
/* ------------------------------------------------------------------------- */
int shdisp_SYS_API_check_diag_mode(void)
{
    sharp_smem_common_type *p_sharp_smem_common_type;
    unsigned long bootmode;

    p_sharp_smem_common_type  = sh_smem_get_common_address();
    if (p_sharp_smem_common_type != 0) {
        bootmode = p_sharp_smem_common_type->sh_boot_mode;
    } else {
        bootmode = 0;
    }
    if ((bootmode == SHDISP_BOOT_D) || (bootmode == SHDISP_BOOT_F_F)) {
        return 1;
    } else {
        return 0;
    }
}

MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
