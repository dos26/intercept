/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define REVISION_ADC_CHANNEL	7		/* Revision Resistor ADC */
#define DRIVER_NAME	"instinctq-battery"

/*
 * InstinctQ Rev01 board Battery Table
 */
#if defined(CONFIG_MACH_VITAL) //kimhyuns_temp
#define BATT_CAL_REV00		180	/* 3.51V */
#endif
#define BATT_CAL		296	/* 3.51V */

#define BATT_MAXIMUM		59	/* 4.19V */
#define BATT_SAFE_RECHARGE	51	/* 4.10V */
#define BATT_FULL		40	/* 3.98V */
#define BATT_ALMOST_FULL	29	/* 3.85V */
#define BATT_HIGH		21	/* 3.75V */
#define BATT_MED		17	/* 3.71V */
#define BATT_LOW		12	/* 3.65V */
#define BATT_CRITICAL		3	/* 3.55V */
#define BATT_MINIMUM		(-2)	/* 3.49V */
#define BATT_OFF		(-8)	/* 3.40V */

/*
 * InstinctQ Rev01 board Temperature Table
 */
#if defined (CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_VITAL)
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
#ifdef ADC_DEVIDE_HALF	
	{ 117,		-50 },	/* Low block	*/
	{ 122,		0	},	/* Low recover	*/
	{ 150,		100 },
	{ 160,		200 },
	{ 170,		300 },
	{ 186,		350 },
	{ 196,		430 },	/* High recover */
	{ 205,		440 },
	{ 206,		450 },	/* High block	*/
#else
	{ 224,		-50	},	/* Low block 230 -> 225	-> 224 */
	{ 230,		0	},	/* Low recover 240 -> 234 -> 230 */
	{ 300,		100	},
	{ 320,		200	},
	{ 340,		300	},
	{ 372,		350	},
	{ 382,		430	},	/* High recover	384 -> 382 */
	{ 393,		440	},
	{ 404,		450	},	/* High block 403 -> 404 */
#endif
};
//pineone_jyseo 100414 Changed battery ADC temperature [[
#elif defined(CONFIG_MACH_MAX)
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
	{ 70,		-50	},	/* Low block	*/  //pineone_jyseo_100428
	{ 76,		0	},	/* Low recover	*/  //pineone_jyseo_100428
	{ 90,		100	},
	{ 100,		200	},
	{ 110,		300	},
	{ 117,		350	},
	{ 125,		400	},	/* High recover	*/
	{ 128,		415	},
	{ 132,		430	},	/* High block	*/
};
//pineone_jyseo 100414 Changed battery ADC temperature ]]
#else
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
	{ 235,		-50	},	/* Low block	*/
	{ 242,		0	},	/* Low recover	*/
	{ 300,		100	},
	{ 320,		200	},
	{ 340,		300	},
	{ 372,		350	},
	{ 399,		430	},	/* High recover	*/
	{ 410,		440	},
	{ 440,		450	},	/* High block	*/
};
#endif

#define TEMP_HIGH_BLOCK		temper_table[8][0]
#define TEMP_HIGH_RECOVER	temper_table[6][0]
#define TEMP_LOW_BLOCK		temper_table[0][0]
#define TEMP_LOW_RECOVER	temper_table[1][0]
#define TEMP_RCOMP			temper_table[3][1]		// 20.0C

/*
 * InstinctQ Rev00 board ADC channel
 */
typedef enum s3c_adc_channel {
	S3C_ADC_VOLTAGE = 0,
	S3C_ADC_TEMPERATURE,
	S3C_ADC_CHG_CURRENT,
	S3C_ADC_EAR,
	S3C_ADC_V_F,
	S3C_ADC_SSENS,
	S3C_ADC_NC6,
	S3C_ADC_BOARD_REV,
	ENDOFADC
} adc_channel_type;

#define IRQ_TA_CONNECTED_N	IRQ_EINT(19)
#define IRQ_TA_CHG_N		IRQ_EINT(25)

/*
 * InstinctQ GPIO for battery driver
 */
const unsigned int gpio_ta_connected	= GPIO_TA_CONNECTED_N;
const unsigned int gpio_ta_connected_af	= GPIO_TA_CONNECTED_N_AF;
const unsigned int gpio_chg_ing		= GPIO_CHG_ING_N;
const unsigned int gpio_chg_ing_af	= GPIO_CHG_ING_N_AF;
const unsigned int gpio_chg_en		= GPIO_CHG_EN;
const unsigned int gpio_chg_en_af	= GPIO_CHG_EN_AF;

/******************************************************************************
 * Battery driver features
 * ***************************************************************************/
/* #define __TEMP_ADC_VALUE__ */
/* #define __USE_EGPIO__ */
#if defined(CONFIG_MACH_MAX)
#define __VZW_AUTH_CHECK__ // Lego
#endif

#define __CHECK_BATTERY_V_F__
#define __BATTERY_COMPENSATION__
#define __CHECK_BOARD_REV__
#define __BOARD_REV_ADC__
#define __TEST_DEVICE_DRIVER__
/* #define __ALWAYS_AWAKE_DEVICE__ */
#define __TEST_MODE_INTERFACE__
#define __CHECK_CHG_CURRENT__
#define __ADJUST_RECHARGE_ADC__
#define __9BITS_RESOLUTION__
#define __REVERSE_TEMPER_ADC__
#if defined (CONFIG_MACH_VINSQ) || defined(CONFIG_MACH_MAX) || defined(CONFIG_MACH_VITAL)
#define __FUEL_GAUGES_IC__
#define __FUEL_GAUGES_IC_MAX17043__		// added by kimjh
#ifdef __FUEL_GAUGES_IC_MAX17043__
#define __FUEL_GAUGES_IC_MAX17043_SLEEP_ALERT__
#define IRQ_FUEL_ALERT_N		IRQ_EINT(12)
#endif

#endif					
#define __ANDROID_BAT_LEVEL_CONCEPT__
#define __DISABLE_CHG_ING_INTR__
/*****************************************************************************/

#define TOTAL_CHARGING_TIME	(5*60*60*1000)	/* 5 hours */
#define TOTAL_RECHARGING_TIME	(2*60*60*1000)	/* 2 hours */

#if defined(CONFIG_MACH_MAX)
#define RECHARGE_COND_VOLTAGE 4120	/* 4.12V */	
#else
#define RECHARGE_COND_VOLTAGE 4130	/* 4.13V */	
#endif
#define FULL_CHARGE_COND_VOLTAGE 4190	/* 4.19V */
#define EX_FULL_CHARGE_COND_VOLTAGE 4000	/* 4.19V */
#define FG_COMPER_COND_VOLTAGE 3550 /* 3.55V */

#ifdef __CHECK_BATTERY_V_F__
#define BATT_VF_MAX		28
#define BATT_VF_MIN		12
#endif /* __CHECK_BATTERY_V_F__ */

#ifdef __CHECK_BOARD_REV__
const unsigned int rev_to_check	= 0x10;		/* 0x10 : rev01 */
#endif /* __CHECK_BOARD_REV__ */

#ifdef __CHECK_CHG_CURRENT__
#if defined(CONFIG_MACH_VINSQ)
#define CURRENT_OF_FULL_CHG 88	
#else
#define CURRENT_OF_FULL_CHG	65
#endif
#endif /* __CHECK_CHG_CURRENT__ */

#ifdef __ADJUST_RECHARGE_ADC__
#define BATT_RECHARGE_CODE	3		/* 0.03V */
#endif /* __ADJUST_RECHARGE_ADC__ */

#ifdef __BATTERY_COMPENSATION__
#if defined (__FUEL_GAUGES_IC__)
#define COMPENSATE_VIBRATOR		0
#define COMPENSATE_CAMERA		0//20 /* 20mV */
#define COMPENSATE_MP3			0
#define COMPENSATE_VIDEO		0//20 /* 20mV */
#define COMPENSATE_VOICE_CALL_2G	0//20 /* 20mV */
#define COMPENSATE_VOICE_CALL_3G	0//20 /* 20mV */
#define COMPENSATE_DATA_CALL		0//20 /* 20mV */
#define COMPENSATE_LCD			0
#define COMPENSATE_TA			0//(-70) /* 70mV */
#define COMPENSATE_CAM_FALSH		0
#define COMPENSATE_BOOTING		0
#define COMPENSATE_BROWSER      0
#define COMPENSATE_WIMAX      0
#else
#define COMPENSATE_VIBRATOR		2
#define COMPENSATE_CAMERA		2
#define COMPENSATE_MP3			2
#define COMPENSATE_VIDEO		1
#define COMPENSATE_VOICE_CALL_2G	3
#define COMPENSATE_VOICE_CALL_3G	3
#define COMPENSATE_DATA_CALL		3
#define COMPENSATE_LCD			3
#define COMPENSATE_TA			(-9)
#define COMPENSATE_CAM_FALSH		3
#define COMPENSATE_BOOTING		9
#define COMPENSATE_BROWSER      3
#define COMPENSATE_WIMAX        3
#endif /* __FUEL_GAUGES_IC__ */
#endif /* __BATTERY_COMPENSATION__ */

