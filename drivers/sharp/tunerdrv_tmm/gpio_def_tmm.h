/**************************************************************************************************/
/** 
	@file		gpio_def.h
	@brief		GPIO Definition Header
*/
/**************************************************************************************************/

#ifndef GPIO_DEF
	#define GPIO_DEF

#define PM8996_GPIO_BASE		1000
#define PM8996_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio + PM8996_GPIO_BASE)

#define DTV_DRV_PINCTRL

#include "gpio_def_ext_tmm.h"

typedef struct GPIO_DEF {
	unsigned int id;		/* GPIO Number (ID) */
	unsigned int no;		/* GPIO PORT Number */
	int direction;			/* I/O Direction */
	int out_val;			/* Initialized Value */
	int init_done;			/* GPIO Initialized ? 1:Complete (Don't Care) */
	char *label;			/* labels may be useful for diagnostics */
} stGPIO_DEF;

#define DirctionIn (0)
#define DirctionOut (1)

#define GPIO_DTVEN_PORTNO			(75)
#define GPIO_DTVRST_PORTNO			(93)
/* #define GPIO_DTVLNAEN_PORTNO		(xx) */
/* #define GPIO_DTVANTSW_PORTNO		(PM8841_GPIO_PM_TO_SYS(15)) */
/* #define GPIO_DTVMANTSL_PORTNO	(PMI8994_GPIO_PMI_TO_SYS(5)) */
/* #define GPIO_DTVUANTSL_PORTNO	(PMI8994_GPIO_PMI_TO_SYS(6)) */
/* #define GPIO_DTVCANTSL_PORTNO	(49) */
/* #define GPIO_DTV_STDBY_PORTNO	(38) */


#endif	//GPIO_DEF
