/* include/sharp/snfc_en.h
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
#ifndef _LINUX_SNFC_EN_H
#define _LINUX_SNFC_EN_H

#define SNFC_OFF_SEQUENCE_NFC 0
#define SNFC_ON_SEQUENCE      1
#define SNFC_OFF_SEQUENCE_SIM 2

#define NFC_SNFC_EN_IOC_MAGIC 'd'
#define NFC_SNFC_EN_IOCTL_HVDD_H      _IO ( NFC_SNFC_EN_IOC_MAGIC, 0x01)
#define NFC_SNFC_EN_IOCTL_HVDD_L      _IO ( NFC_SNFC_EN_IOC_MAGIC, 0x02)

#endif /* _LINUX_SNFC_EN_H */

