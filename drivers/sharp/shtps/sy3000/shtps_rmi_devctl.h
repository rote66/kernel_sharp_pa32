/* drivers/sharp/shtps/sy3000/tm2945-001/shtps_rmi_devctl.h
 *
 * Copyright (c) 2013, Sharp. All rights reserved.
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

#ifndef __SHTPS_RMI_DEVCTL_H__
#define __SHTPS_RMI_DEVCTL_H__

/* -----------------------------------------------------------------------------------
 */
int shtps_device_setup(int irq, int rst);
void shtps_device_teardown(int irq, int rst);
void shtps_device_reset(int rst);
void shtps_device_sleep(struct device* dev);
void shtps_device_wakeup(struct device* dev);

void shtps_lvs1_regulator_reset(int rst);
void shtps_lvs1_regulator_get(struct device *dev);
void shtps_lvs1_regulator_put(void);
void shtps_lvs1_regulator_init(struct device *dev);
void shtps_lvs1_regulator_remove(void);

#endif /* __SHTPS_RMI_DEVCTL_H__ */
