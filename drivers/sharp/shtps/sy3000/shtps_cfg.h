/* drivers/sharp/shtps/sy3000/shtps_cfg.h
 *
 * Copyright (c) 2015, Sharp. All rights reserved.
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
#ifndef __SHTPS_CFG_H__
#define __SHTPS_CFG_H__
/* --------------------------------------------------------------------------- */

#if 0	/** For build test */
	#undef CONFIG_SHTPS_SY3000_PRJ_000624_1
	#undef CONFIG_SHTPS_SY3000_PRJ_000624_2
	#undef CONFIG_SHTPS_SY3000_PRJ_000624_3

	#define CONFIG_SHTPS_SY3000_PRJ_000624_1
#endif

#include <sharp/shtps_dev.h>
/* --------------------------------------------------------------------------- */
#if defined(CONFIG_SHTPS_SY3000_PRJ_000624_1)
	#include "prj-000624-1/shtps_cfg_prj-000624-1.h"

#elif defined(CONFIG_SHTPS_SY3000_PRJ_000624_2)
	#include "prj-000624-2/shtps_cfg_prj-000624-2.h"

#elif defined(CONFIG_SHTPS_SY3000_PRJ_000624_3)
	#include "prj-000624-3/shtps_cfg_prj-000624-3.h"

#else
	#include "prj-000624-1/shtps_cfg_prj-000624-1.h"

#endif

/* --------------------------------------------------------------------------- */
#endif	/* __SHTPS_CFG_H__ */

