/* drivers/sharp/nfc/nfc.h (NFC Common Header)
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

#ifndef NFC_H
#define NFC_H

/* DEBUG_LOG */
#if 0
#define DEBUG_NFC_DRV
#endif

#ifdef DEBUG_NFC_DRV
#define NFC_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[NFC][%s]" fmt "\n", __func__, ## args)
#else
#define NFC_DRV_DBG_LOG(fmt, args...)
#endif
#define NFC_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[NFC][%s]ERR " fmt "\n", __func__, ## args)

#ifdef DEBUG_NFC_DRV
#define INFO_PRINT( fmt, arg... )  { printk(KERN_INFO "[NFC]%s[%d]: " fmt "\n", __FUNCTION__, __LINE__, ##arg); }
#define DEBUG_PRINT( fmt, arg... ) { printk(KERN_INFO "[NFC]%s[%d]: " fmt "\n", __FUNCTION__, __LINE__, ##arg); }
#define TRACE() DEBUG_PRINT( "[NFC]%s( %d )", __FUNCTION__, __LINE__ )
#else 
#define INFO_PRINT( fmt, arg... )
#define DEBUG_PRINT( fmt, arg... )
#define TRACE() DEBUG_PRINT()
#endif
#define ERROR_PRINT( fmt, arg... )  { printk(KERN_ERR "[NFC]%s[%d]: " fmt "\n", __FUNCTION__, __LINE__, ##arg); }

#define usleep(arg) usleep_range(arg, arg)

#endif /* NFC_H */
