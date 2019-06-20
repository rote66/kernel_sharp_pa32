/* include/sharp/snfc_gpio.h
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
#ifndef _LINUX_SNFC_GPIO_H
#define _LINUX_SNFC_GPIO_H

#include "../../drivers/sharp/nfc/GPIO/nfc_gpio_drv_sys.h"

#define NFC_GPIO_IOCTL_GET_NINT      _IOR ( NFC_GPIO_IOC_MAGIC, 0x15, unsigned int)
#define NFC_GPIO_IOCTL_GET_STATUS    _IOR ( NFC_GPIO_IOC_MAGIC, 0x16, unsigned int)
#define NFC_GPIO_IOCTL_REQ_CHIPRESET _IO  ( NFC_GPIO_IOC_MAGIC, 0x17)

#endif /* _LINUX_SNFC_GPIO_H */

