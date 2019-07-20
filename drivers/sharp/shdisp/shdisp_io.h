/* drivers/sharp/shdisp/shdisp_io.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */
#ifndef SHDISP_IO_H
#define SHDISP_IO_H
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* DEBUG MACROS                                                              */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
void shdisp_IO_API_delay_us(unsigned long usec);
void shdisp_IO_API_msleep(unsigned int msec);
void shdisp_IO_API_usleep(unsigned int usec);
void shdisp_IO_API_Host_gpio_init(void);
void shdisp_IO_API_Host_gpio_exit(void);
int  shdisp_IO_API_Host_gpio_request(int num, char *label);
int  shdisp_IO_API_Host_gpio_free(int num);
int  shdisp_IO_API_set_Host_gpio(int num, int value);
int  shdisp_IO_API_get_Host_gpio(int num);

int  shdisp_IO_API_bdic_i2c_init(void);
int  shdisp_IO_API_bdic_i2c_exit(void);
int  shdisp_IO_API_bdic_i2c_write(unsigned char addr, unsigned char data);
int  shdisp_IO_API_bdic_i2c_mask_write(unsigned char addr, unsigned char data, unsigned char mask);
int  shdisp_IO_API_bdic_i2c_multi_write(unsigned char addr, unsigned char *wval, unsigned char size);
int  shdisp_IO_API_bdic_i2c_read(unsigned char addr, unsigned char *data);
int  shdisp_IO_API_bdic_i2c_multi_read(unsigned char addr, unsigned char *data, int size);
int  shdisp_IO_API_bdic_i2c_dummy_read(unsigned char addr, unsigned char *data);

int  shdisp_IO_API_sensor_i2c_init(void);
int  shdisp_IO_API_sensor_i2c_exit(void);

int  shdisp_IO_API_Host_i2c_send(unsigned char slaveaddr, unsigned char *sendval, unsigned char size);
int  shdisp_IO_API_Host_i2c_recv(unsigned char slaveaddr, unsigned char *sendval, unsigned char sendsize,
                                   unsigned char *recvval, unsigned char recvsize);

#endif /* SHDISP_IO_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
