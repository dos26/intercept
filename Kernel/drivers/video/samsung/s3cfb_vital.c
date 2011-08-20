/*
 * drivers/video/samsung/s3cfb_mdj2024wv.c
 *
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <linux/i2c/maximi2c.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>

#include <mach/hardware.h>

#include "s3cfb.h"

#define BACKLIGHT_STATUS_ALC	0x100
#define BACKLIGHT_LEVEL_VALUE	0x0FF	/* 0 ~ 255 */

#define BACKLIGHT_LEVEL_MIN		0
#define BACKLIGHT_LEVEL_MAX		BACKLIGHT_LEVEL_VALUE

#define BACKLIGHT_LEVEL_DEFAULT	BACKLIGHT_LEVEL_MAX		/* Default Setting */

#define OFFSET_LCD_ON           (0x1 << 7)

#define PASSWD2			0xF1
#define DISCTL			0xF2
#define POWCTL			0xF3
#define VCMCTL			0xF4
#define SRCCTL			0xF5
#define PANELCTL1		0xF6
#define PANELCTL2		0xF7
#define PANELCTL3		0xF8
#define PANELCTL4		0xF9
#define PGAMMACTL		0xFA
#define NGAMMACTL		0xFB
#define CLKCTL3			0xB7
#define HOSTCTL1		0xB8
#define HOSTCTL2		0xB9
#define TEON			0x35
#define CASET			0x2A
#define PASET			0x2B
#define COLMOD			0x3A
#define WRCTRLD			0x53
#define SLPOUT			0x11
#define DISPON			0x29
#define DISPOFF			0x28
#define SLPIN			0x10

extern int lcd_late_resume;

/* sec_bsp_tsim 2009.08.12 : reset lcd before reboot this machine. */
void lcd_reset(void)
{
	gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);
};
EXPORT_SYMBOL(lcd_reset);

extern void s3c_bat_set_compensation_for_drv(int mode,int offset);

int lcd_power = OFF;
EXPORT_SYMBOL(lcd_power);

int lcd_power_ctrl(s32 value);
EXPORT_SYMBOL(lcd_power_ctrl);

int backlight_power = OFF;
EXPORT_SYMBOL(backlight_power);

void backlight_power_ctrl(s32 value);
EXPORT_SYMBOL(backlight_power_ctrl);

int backlight_level = BACKLIGHT_LEVEL_DEFAULT;
EXPORT_SYMBOL(backlight_level);

void backlight_level_ctrl(s32 value);
EXPORT_SYMBOL(backlight_level_ctrl);

#define S3C_FB_HFP			8 		/* Front Porch */
#define S3C_FB_HSW			16 		/* Hsync Width */
#define S3C_FB_HBP			12 		/* Back Porch */

#define S3C_FB_VFP			2 		/* Front Porch */
#define S3C_FB_VSW			2 		/* Vsync Width */
#define S3C_FB_VBP			8 		/* Back Porch */

#define S3C_FB_HRES             320 //vital.boot 320     /* Horizon pixel Resolition */
#define S3C_FB_VRES             480 //vital.boot 480     /* Vertical pixel Resolution */

#define S3C_FB_HRES_VIRTUAL     S3C_FB_HRES     /* Horizon pixel Resolition */
#define S3C_FB_VRES_VIRTUAL     S3C_FB_VRES * 2 /* Vertial pixel Resolution */
#define S3C_FB_HRES_OSD         320 //vital.boot 320     /* Horizon pixel Resolition */
#define S3C_FB_VRES_OSD         480 //vital.boot 480     /* Vertial pixel Resolution */
#define S3C_FB_HRES_OSD_VIRTUAL S3C_FB_HRES_OSD     /* Horizon pixel Resolition */
#define S3C_FB_VRES_OSD_VIRTUAL S3C_FB_VRES_OSD * 2 /* Vertial pixel Resolution */

#define S3C_FB_VFRAME_FREQ  	95		/* Frame Rate Frequency */

#define S3C_FB_PIXEL_CLOCK		(S3C_FB_VFRAME_FREQ * \
								(S3C_FB_HFP + S3C_FB_HSW + S3C_FB_HBP + S3C_FB_HRES) * \
								(S3C_FB_VFP + S3C_FB_VSW + S3C_FB_VBP + S3C_FB_VRES))

static void s3cfb_set_fimd_info(void)
{
	s3c_fimd.vidcon1	= S3C_VIDCON1_IHSYNC_INVERT |
							S3C_VIDCON1_IVSYNC_INVERT |
							S3C_VIDCON1_IVDEN_NORMAL|S3C_VIDCON1_IVCLK_RISE_EDGE;
	s3c_fimd.vidtcon0 	= S3C_VIDTCON0_VBPD(S3C_FB_VBP - 1) |
							S3C_VIDTCON0_VFPD(S3C_FB_VFP - 1) |
							S3C_VIDTCON0_VSPW(S3C_FB_VSW - 1);
	s3c_fimd.vidtcon1	= S3C_VIDTCON1_HBPD(S3C_FB_HBP - 1) |
							S3C_VIDTCON1_HFPD(S3C_FB_HFP - 1) |
							S3C_VIDTCON1_HSPW(S3C_FB_HSW - 1);
	s3c_fimd.vidtcon2	= S3C_VIDTCON2_LINEVAL(S3C_FB_VRES - 1) |
							S3C_VIDTCON2_HOZVAL(S3C_FB_HRES - 1);

	s3c_fimd.vidosd0a 	= S3C_VIDOSDxA_OSD_LTX_F(0) |
							S3C_VIDOSDxA_OSD_LTY_F(0);
	s3c_fimd.vidosd0b 	= S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES - 1) |
							S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES - 1);

	s3c_fimd.vidosd1a 	= S3C_VIDOSDxA_OSD_LTX_F(0) |
							S3C_VIDOSDxA_OSD_LTY_F(0);
	s3c_fimd.vidosd1b 	= S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES_OSD - 1) |
							S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES_OSD - 1);

	s3c_fimd.width		= 50;
	s3c_fimd.height 	= 75;
	s3c_fimd.xres 		= S3C_FB_HRES;
	s3c_fimd.yres 		= S3C_FB_VRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3c_fimd.xres_virtual = S3C_FB_HRES_VIRTUAL;
	s3c_fimd.yres_virtual = S3C_FB_VRES_VIRTUAL;
#else
	s3c_fimd.xres_virtual = S3C_FB_HRES;
	s3c_fimd.yres_virtual = S3C_FB_VRES;
#endif

	s3c_fimd.osd_width 	= S3C_FB_HRES_OSD;
	s3c_fimd.osd_height = S3C_FB_VRES_OSD;
	s3c_fimd.osd_xres 	= S3C_FB_HRES_OSD;
	s3c_fimd.osd_yres 	= S3C_FB_VRES_OSD;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3c_fimd.osd_xres_virtual = S3C_FB_HRES_OSD_VIRTUAL;
	s3c_fimd.osd_yres_virtual = S3C_FB_VRES_OSD_VIRTUAL;
#else
	s3c_fimd.osd_xres_virtual = S3C_FB_HRES_OSD;
	s3c_fimd.osd_yres_virtual = S3C_FB_VRES_OSD;
#endif

    s3c_fimd.pixclock		= S3C_FB_PIXEL_CLOCK;

	s3c_fimd.hsync_len 		= S3C_FB_HSW;
	s3c_fimd.vsync_len 		= S3C_FB_VSW;
	s3c_fimd.left_margin 	= S3C_FB_HFP;
	s3c_fimd.upper_margin 	= S3C_FB_VFP;
	s3c_fimd.right_margin 	= S3C_FB_HBP;
	s3c_fimd.lower_margin 	= S3C_FB_VBP;

	s3c_fimd.set_lcd_power		 = lcd_power_ctrl;
	s3c_fimd.set_backlight_power = backlight_power_ctrl;
	s3c_fimd.set_brightness 	 = backlight_level_ctrl;

	s3c_fimd.backlight_min = BACKLIGHT_LEVEL_MIN;
	s3c_fimd.backlight_max = BACKLIGHT_LEVEL_MAX;

}

static void lcd_gpio_init(void)
{
	/* B(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_B_0, S3C_GPIO_SFN(GPIO_LCD_B_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_1, S3C_GPIO_SFN(GPIO_LCD_B_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_2, S3C_GPIO_SFN(GPIO_LCD_B_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_3, S3C_GPIO_SFN(GPIO_LCD_B_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_4, S3C_GPIO_SFN(GPIO_LCD_B_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_5, S3C_GPIO_SFN(GPIO_LCD_B_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_6, S3C_GPIO_SFN(GPIO_LCD_B_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_7, S3C_GPIO_SFN(GPIO_LCD_B_7_AF));
	/* G(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_G_0, S3C_GPIO_SFN(GPIO_LCD_G_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_1, S3C_GPIO_SFN(GPIO_LCD_G_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_2, S3C_GPIO_SFN(GPIO_LCD_G_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_3, S3C_GPIO_SFN(GPIO_LCD_G_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_4, S3C_GPIO_SFN(GPIO_LCD_G_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_5, S3C_GPIO_SFN(GPIO_LCD_G_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_6, S3C_GPIO_SFN(GPIO_LCD_G_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_7, S3C_GPIO_SFN(GPIO_LCD_G_7_AF));
	/* R(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_R_0, S3C_GPIO_SFN(GPIO_LCD_R_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_1, S3C_GPIO_SFN(GPIO_LCD_R_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_2, S3C_GPIO_SFN(GPIO_LCD_R_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_3, S3C_GPIO_SFN(GPIO_LCD_R_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_4, S3C_GPIO_SFN(GPIO_LCD_R_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_5, S3C_GPIO_SFN(GPIO_LCD_R_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_6, S3C_GPIO_SFN(GPIO_LCD_R_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_7, S3C_GPIO_SFN(GPIO_LCD_R_7_AF));
	/* HSYNC */
	s3c_gpio_cfgpin(GPIO_LCD_HSYNC, S3C_GPIO_SFN(GPIO_LCD_HSYNC_AF));
	/* VSYNC */
	s3c_gpio_cfgpin(GPIO_LCD_VSYNC, S3C_GPIO_SFN(GPIO_LCD_VSYNC_AF));
	/* DE */
	s3c_gpio_cfgpin(GPIO_LCD_DE, S3C_GPIO_SFN(GPIO_LCD_DE_AF));
	/* CLK */
	s3c_gpio_cfgpin(GPIO_LCD_CLK, S3C_GPIO_SFN(GPIO_LCD_CLK_AF));

	/* LCD_RST_N */
	if (gpio_is_valid(GPIO_LCD_RST_N)) {
		if (gpio_request(GPIO_LCD_RST_N, S3C_GPIO_LAVEL(GPIO_LCD_RST_N))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_RST_N!\n");
		gpio_direction_output(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_RST_N, S3C_GPIO_PULL_NONE);
	/* LCD_ID */
	if (gpio_is_valid(GPIO_LCD_ID)) {
		if (gpio_request(GPIO_LCD_ID, S3C_GPIO_LAVEL(GPIO_LCD_ID))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_ID!\n");
		gpio_direction_input(GPIO_LCD_ID);
	}
	s3c_gpio_setpull(GPIO_LCD_ID, S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	if (gpio_is_valid(GPIO_LCD_SCLK)) {
		if (gpio_request(GPIO_LCD_SCLK, S3C_GPIO_LAVEL(GPIO_LCD_SCLK))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SCLK!\n");
		gpio_direction_output(GPIO_LCD_SCLK, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SCLK, S3C_GPIO_PULL_NONE);
	/* LCD_CS_N */
	if (gpio_is_valid(GPIO_LCD_CS_N)) {
		if (gpio_request(GPIO_LCD_CS_N, S3C_GPIO_LAVEL(GPIO_LCD_CS_N))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_CS_N!\n");
		gpio_direction_output(GPIO_LCD_CS_N, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_CS_N, S3C_GPIO_PULL_NONE);
	/* LCD_SDO */
#if !defined(CONFIG_MACH_VITAL)
	if (gpio_is_valid(GPIO_LCD_SDO)) {
		if (gpio_request(GPIO_LCD_SDO, S3C_GPIO_LAVEL(GPIO_LCD_SDO))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SDO!\n");
		gpio_direction_output(GPIO_LCD_SDO, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SDO, S3C_GPIO_PULL_NONE);
#endif
	/* LCD_SDI */
	if (gpio_is_valid(GPIO_LCD_SDI)) {
		if (gpio_request(GPIO_LCD_SDI, S3C_GPIO_LAVEL(GPIO_LCD_SDI))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SDI!\n");
		gpio_direction_output(GPIO_LCD_SDI, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SDI, S3C_GPIO_PULL_NONE);
}

static void backlight_gpio_init(void)
{
#if defined(FEATURE_AAT1231)
    s3c_gpio_cfgpin(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_AF);
#if 0 //kimhyuns   
    if (system_rev > 0x80)
#endif
        s3c_gpio_cfgpin(GPIO_LCD_BL_SEL, GPIO_LCD_BL_SEL_AF);

	if (gpio_is_valid(GPIO_LCD_BL_EN)) {
		if (gpio_request(GPIO_LCD_BL_EN, S3C_GPIO_LAVEL(GPIO_LCD_BL_EN))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_BL_EN!\n");
		gpio_direction_output(GPIO_LCD_BL_EN, GPIO_LEVEL_HIGH);
	}    
#if 0 //kimhyuns
    if (system_rev > 0x80)
#endif
    {
	if (gpio_is_valid(GPIO_LCD_BL_SEL)) {
		if (gpio_request(GPIO_LCD_BL_SEL, S3C_GPIO_LAVEL(GPIO_LCD_BL_SEL))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_BL_SEL!\n");
		gpio_direction_output(GPIO_LCD_BL_SEL, GPIO_LEVEL_HIGH);
	}	
    }
#else
    s3c_gpio_cfgpin(GPIO_BL_RST, 1);


	if (gpio_is_valid(GPIO_BL_RST)) {
		if (gpio_request(GPIO_BL_RST, S3C_GPIO_LAVEL(GPIO_BL_RST))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_BL_EN!\n");
		gpio_direction_output(GPIO_BL_RST, GPIO_LEVEL_HIGH);
	}   
#endif
}

#define LCD_CSX_HIGH	gpio_set_value(GPIO_LCD_CS_N, GPIO_LEVEL_HIGH);
#define LCD_CSX_LOW		gpio_set_value(GPIO_LCD_CS_N, GPIO_LEVEL_LOW);

#define LCD_SCL_HIGH	gpio_set_value(GPIO_LCD_SCLK, GPIO_LEVEL_HIGH);
#define LCD_SCL_LOW		gpio_set_value(GPIO_LCD_SCLK, GPIO_LEVEL_LOW);

#define LCD_SDI_HIGH	gpio_set_value(GPIO_LCD_SDI, GPIO_LEVEL_HIGH);
#define LCD_SDI_LOW		gpio_set_value(GPIO_LCD_SDI, GPIO_LEVEL_LOW);
#define DEFAULT_USLEEP	10	

struct setting_table {
	u8 command;	
	u8 parameters;
	u8 parameter[50];
	s32 wait;
};


static struct setting_table power_on_setting_table[] = {
	{  PASSWD2,  2, { 0x5A, 0x5A, 	},	0 },
	{   DISCTL,  9, { 0x00, 0x00, 0x82, 0x82, 0x57, 0x57, 0x10, 0x02, 0x00, },   0 },	
	{   POWCTL, 12, { 0x00, 0x10, 0x25, 0x01, 0x2D, 0x2D, 0x24, 0x2d, 0x14, 0x14, 0x12, 0x78,  },   0 },
	{   VCMCTL, 10, { 0x00, 0x23, 0x00, 0x57, 0x68, 0x00, 0x57, 0x68, 0x00, 0x00, },   0 },
	{   SRCCTL, 10, { 0x00, 0x00, 0x57, 0x00, 0x0B, 0x01, 0x14, 0x14, 0x09, 0x09, },   0 },
	{PANELCTL1,  6, { 0x02, 0x00, 0x80, 0x00, 0x44, 0x00, },   0 },
	{PANELCTL2, 17, { 0x00, 0x01, 0x00, 0xF2, 0x08, 0x00, 0x07, 0x1C, 0x07, 0x08, 0x23, 0x00, 0x07, 0x00, 0x4B, 0x00, 0x8C },   0 },
	{PANELCTL3, 17, { 0x00, 0x01, 0x00, 0xF2, 0x08, 0x00, 0x07, 0x1C, 0x07, 0x08, 0x23, 0x00, 0x07, 0x00, 0x4B, 0x00, 0x8C, },   0 },
	{PANELCTL4, 22, { 0x00, 0x08, 0x00, 0x01, 0x00, 0x05, 0x00, 0x04, 0x00, 0x0C, 0x02, 0x0F, 0x00, 0x10, 0x00, 0x11, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xC0, },   0 },
	{PGAMMACTL, 45, { 0x21, 0x37, 0x2C, 0x0F, 0x13, 0x17, 0x1A, 0x0F, 0x0B, 0x23, 0x19, 0x21, 0x2A, 0x29, 0x27, 0x19, 0x37, 0x2C, 0x0F, 0x14, 0x17, 0x1A, 0x0D, 0x09, 0x22, 0x16, 0x1A, 0x21, 0x1D, 0x1C, 0x00, 0x37, 0x2C, 0x12, 0x16, 0x17, 0x1C, 0x10, 0x0E, 0x25, 0x12, 0x10, 0x07, 0x00, 0x00, }, 0 },
	{NGAMMACTL, 45, { 0x00, 0x1B, 0x15, 0x13, 0x32, 0x3B, 0x43, 0x38, 0x32, 0x4D, 0x41, 0x44, 0x45, 0x23, 0x0D, 0x00, 0x22, 0x1F, 0x1E, 0x3B, 0x42, 0x46, 0x38, 0x30, 0x4B, 0x42, 0x43, 0x42, 0x23, 0x0D, 0x00, 0x3F, 0x3E, 0x3E, 0x55, 0x4A, 0x48, 0x38, 0x31, 0x4C, 0x41, 0x43, 0x40, 0x1F, 0x0D, }, 0 },
	{  CLKCTL3,  3, { 0x00, 0x11, 0x11, }, 0},
	{ HOSTCTL1,  2, { 0x31, 0x11, }, 0},
	{ HOSTCTL2,  2, { 0x00, 0x06, }, 0},
	{     TEON,  0, { 0, }, 0},
	{  PASSWD2,  2, { 0xA5, 0xA5,  },	0 },
	{    CASET,  4, { 0x00, 0x00, 0x01, 0x3F, }, 0 },
	{    PASET,  4, { 0x00, 0x00, 0x01, 0xDF, }, 0 },
	{   COLMOD,  2, { 0x00, 0x77, }, 0 },
	{  WRCTRLD,  1, { 0x00, }, 0 },
	// sleep out
	{   SLPOUT,  0, {0, }, 120 },
	// disp on
	{   DISPON,  0, {0, }, 0 },
//kimhyuns to prevent LCD Backlight damage by hw's asking	{    0xDA,  2, { 0x11, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 }	
};

#define POWER_ON_SETTINGS	(int)(sizeof(power_on_setting_table)/sizeof(struct setting_table))

static struct setting_table power_off_setting_table[] = {
	{  PASSWD2,  2, { 0x5A, 0x5A, 	},	0 },
	{   DISCTL,  9, { 0x00, 0x00, 0x82, 0x82, 0x57, 0x57, 0x10, 0x00, 0x00, },   0 },	
	{  PASSWD2,  2, { 0xA5, 0xA5,  },	5 },
	{   DISPOFF, 0, { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  40 },
	{   SLPIN,   0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

#define POWER_OFF_SETTINGS	(int)(sizeof(power_off_setting_table)/sizeof(struct setting_table))

#define usleep(t)  udelay(t)
static void setting_table_write(struct setting_table *table)
{
	s32 i, j;

	LCD_CSX_HIGH
	usleep(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	usleep(DEFAULT_USLEEP);

	/* Write Command */
	LCD_CSX_LOW
	usleep(DEFAULT_USLEEP);
	
	LCD_SCL_LOW 
	usleep(DEFAULT_USLEEP);		
	LCD_SDI_LOW 
	usleep(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	usleep(DEFAULT_USLEEP); 

   	for (i = 7; i >= 0; i--) { 
		LCD_SCL_LOW
		usleep(DEFAULT_USLEEP);
		if ((table->command >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		usleep(DEFAULT_USLEEP);	
		LCD_SCL_HIGH
		usleep(DEFAULT_USLEEP);	
	}

	LCD_CSX_HIGH
	usleep(DEFAULT_USLEEP);	

	/* Write Parameter */
	if ((table->parameters) > 0) {
		for (j = 0; j < table->parameters; j++) {
			LCD_CSX_LOW 
			usleep(DEFAULT_USLEEP); 	
		
			LCD_SCL_LOW 
			usleep(DEFAULT_USLEEP); 	
			LCD_SDI_HIGH 
			usleep(DEFAULT_USLEEP);
			LCD_SCL_HIGH 
			usleep(DEFAULT_USLEEP); 	

			for (i = 7; i >= 0; i--) { 
				LCD_SCL_LOW
				usleep(DEFAULT_USLEEP);	
				if ((table->parameter[j] >> i) & 0x1)
					LCD_SDI_HIGH
				else
					LCD_SDI_LOW
				usleep(DEFAULT_USLEEP);	
				LCD_SCL_HIGH
				usleep(DEFAULT_USLEEP);					
			}
		
			LCD_CSX_HIGH 
			usleep(DEFAULT_USLEEP); 	
		}
	}
	
	msleep(table->wait);
}

#if 0//defined(CONFIG_MACH_VITAL)//kimhyuns to prevent LCD Backlight damage by hw's asking
static struct setting_table device_id_readding_table[] = {
	{    0xDA,  2, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

#define DEVICE_ID_READDINGS	(int)(sizeof(device_id_readding_table)/sizeof(struct setting_table))

static void setting_table_read(struct setting_table *table)
{
	s32 i, j;
    
	LCD_CSX_HIGH
	usleep(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	usleep(DEFAULT_USLEEP);

	/* Write Command */
	LCD_CSX_LOW
	usleep(DEFAULT_USLEEP);
	
	LCD_SCL_LOW 
	usleep(DEFAULT_USLEEP);		
	LCD_SDI_LOW 
	usleep(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	usleep(DEFAULT_USLEEP); 

   	for (i = 7; i >= 0; i--) { 
		LCD_SCL_LOW
		usleep(DEFAULT_USLEEP);
		if ((table->command >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		usleep(DEFAULT_USLEEP);	
		LCD_SCL_HIGH
		usleep(DEFAULT_USLEEP);	
	}
	LCD_SCL_LOW
	usleep(DEFAULT_USLEEP);	
	LCD_SCL_HIGH
	usleep(DEFAULT_USLEEP);	

	/* Read Parameter */
	if ((table->parameters) > 0) {
		for (j = 0; j < table->parameters; j++) {
			for (i = 7; i >= 0; i--) { 
				LCD_SCL_LOW
				usleep(DEFAULT_USLEEP);	
				if (gpio_get_value(GPIO_LCD_ID))
				    table->parameter[j]=table->parameter[j]|(0x1 << i);
				usleep(DEFAULT_USLEEP);	
				LCD_SCL_HIGH
				usleep(DEFAULT_USLEEP);				
                printk("gpio_get_value(GPIO_LCD_SDO)=%d, table->param=%d \n", gpio_get_value(GPIO_LCD_ID), table->parameter[j]);				
			}
		}
	}

	LCD_CSX_HIGH 
	usleep(DEFAULT_USLEEP); 	
	msleep(table->wait);
}
#endif

/*
 *	LCD Power Handler
 */

#define MAX8698_ID		0xCC

#define ONOFF2			0x01

#define ONOFF2_ELDO6	(0x01 << 7)
#define ONOFF2_ELDO7	(0x03 << 6)

#include <linux/suspend.h>
int lcd_power_ctrl(s32 value)
{
	s32 i;	
	u8 data;
	u32 timeout = 100;
	u32 loopcnt=0;
	printk("lcd_power_ctrl(%d)\n", value);
	if (value) {
		while (timeout-- > 0) {
			if (lcd_late_resume == 1)
				break;
#ifdef CONFIG_EARLYSUSPEND
			extern suspend_state_t get_suspend_state(void);
			if(get_suspend_state()!=PM_SUSPEND_ON) {
				printk(KERN_ERR "[%s suspend state=%d] called before late resume is even requested!!!\n", __func__, get_suspend_state());
				return -1;
			}
#endif
			msleep(50);
			loopcnt++;
		}
		
		if (timeout == -1) {
			printk(KERN_ERR "lcd power control time out\n");
			return -1;
		}
		if(loopcnt>0)
			printk(KERN_ERR "[lcd_power_ctrl]!@#$ waited loopcnt=%d\n", loopcnt);
		
		printk("Lcd power on sequence start\n");
		/* Power On Sequence */
		s3c_bat_set_compensation_for_drv(1,OFFSET_LCD_ON);
#if 0 //kimhyuns_test opt timing		
		/* Reset Asseert */
		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);
	
		/* Power Enable */
		if(pmic_read(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't read the status from PMIC\n");
			return -1;
		}
		data |= (ONOFF2_ELDO6 | ONOFF2_ELDO7);
			
		if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			return -1;
		}
	
		msleep(100);

		/* Reset Deasseert */
		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);
	
		msleep(120);
#else
		/* Reset Asseert */
//		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW); //removed by kimhyuns for hw Kim mijin's asking
		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH); //added by kimhyuns for hw Kim mijin's asking		
		msleep(1); //added by kimhyuns for hw Kim mijin's asking			
		/* Power Enable */
		if(pmic_read(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't read the status from PMIC\n");
			return -1;
		}
		data |= (ONOFF2_ELDO6);
			
		if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			return -1;
		}
		msleep(1);

		data |= (ONOFF2_ELDO7);
			
		if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			return -1;
		}
		msleep(1);

		/* Reset Deasseert */
		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW); //added by kimhyuns for hw Kim mijin's asking
		msleep(1);	 //added by kimhyuns for hw Kim mijin's asking		

		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);
		msleep(10);		
#endif
        struct setting_table *p_power_set_tbl = power_on_setting_table;
#if 0 //kimhyuns
        struct setting_table *p_disp_set_tbl = disp_on_setting_table;
#endif
        int tblsize = POWER_ON_SETTINGS;
#if 0 //kimhyuns        
        int disp_tblsize = DISP_ON_SETTINGS;
#endif

            // Power setting - Initializing - Ram addr set & data write
		for (i = 0; i < tblsize; i++)
			setting_table_write(&p_power_set_tbl[i]);

		printk("Lcd power on sequence end\n");
	}
	else {
		printk("Lcd power off sequence start\n");

		s3c_bat_set_compensation_for_drv(0,OFFSET_LCD_ON);
#if 1 //kimhyuns_test to follow the spec doc
        int power_off_tblsize = POWER_OFF_SETTINGS;       
   		for (i = 0; i < power_off_tblsize; i++)
   			setting_table_write(&power_off_setting_table[i]);
#endif
		/* Reset Assert */
		gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);

		/* Power Disable */
		if(pmic_read(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't read the status from PMIC\n");
			return -1;
		}
		data &= ~(ONOFF2_ELDO7);
	
		if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			return -1;
		}
		msleep(1);
		data &= ~(ONOFF2_ELDO6);
	
		if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
			printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			return -1;
		}
		msleep(1);
		printk("Lcd power off sequence end\n");
	}
	
	lcd_power = value;
#if 1
		gpio_set_value(GPIO_TOUCH_EN, 0);  // TOUCH EN
		msleep(1);	  
		gpio_set_value(GPIO_TOUCH_EN, 1);  // TOUCH EN
#endif
	return 0;
}

static s32 old_level = 0;
#ifdef FEATURE_AAT1231
static void AAT1231_backlight_ctrl(int count)
{
    int i = 0;
    unsigned long flags;
    printk("kimhyuns ext_lcd_dim_pulse(%d)\n", count);
    local_irq_save(flags);
#if 0//kimhyuns
    if(system_rev >= 0x80)
#endif
    {
        // Level 0 ~ 25
        if(count <= 10)
        {
            gpio_set_value( GPIO_LCD_BL_SEL,  GPIO_LEVEL_HIGH);
            printk("ext_lcd_pulse(%d), HIGH\n", count);
        }
        else
        {
            count = count - 10;
            gpio_set_value( GPIO_LCD_BL_SEL,  GPIO_LEVEL_LOW);
            printk("ext_lcd_pulse(%d), LOW\n", count);
        }

    }
#if 0 //kimhyuns
    else
    {
        gpio_set_value( GPIO_LCD_BL_EN,  GPIO_LEVEL_HIGH); //INITIAL High (>= 200us)
    }
#endif
    udelay(500);
//    INTLOCK(); //INTR_LOCK(); //mskim_yl19 for preventing  from interrupt
    for(i = 0 ; i< count; i++)
    {
        gpio_set_value( GPIO_LCD_BL_EN,  GPIO_LEVEL_LOW); // LOW ( 500ns ~ 500us)
        udelay(2);

        gpio_set_value( GPIO_LCD_BL_EN,  GPIO_LEVEL_HIGH); // HIGH (  >= 500ns )
        udelay(2);
    }
    udelay(500);
//    INTFREE(); //INTR_UNLOCK(); //mskim_yl19 for preventing  from interrupt
    local_irq_restore(flags);
}
#endif

void backlight_ctrl(s32 value)
{
	extern void bd60910gu_backlight_ctrl(s32 value);

#if 0//defined(CONFIG_MACH_VITAL)//kimhyuns to prevent LCD Backlight damage by hw's asking
    u8 data;
    setting_table_read(device_id_readding_table);
    if ((device_id_readding_table[0].parameter[0]!=0x11) || (device_id_readding_table[0].parameter[1]!=0x22))
        return 0;
        
    printk("device_id_readding_table 1 is %d, 2 is %d\n", device_id_readding_table[0].parameter[0], device_id_readding_table[0].parameter[1]);
#endif	

	printk("backlight_ctrl(%d)\n", value);
	bd60910gu_backlight_ctrl(value);

}

void backlight_level_ctrl(s32 value)
{
	if ((value < BACKLIGHT_LEVEL_MIN) ||	/* Invalid Value */
		(value > BACKLIGHT_LEVEL_MAX) ||
		(value == backlight_level))	/* Same Value */
		return;

	if (lcd_late_resume == 0) {
		printk(KERN_ERR "backlight control is not allowed after early suspend\n");
	   	return;
	}

	if (backlight_power)
		backlight_ctrl(value);
	
	backlight_level = value;	
}

void backlight_power_ctrl(s32 value)
{
	printk(KERN_ERR "kimhyuns backlight_power_ctrl !!!!!!!!!!!!!!!!!!\n");
	if ((value < OFF) ||	/* Invalid Value */
		(value > ON))
		return;

	backlight_ctrl((value ? backlight_level : OFF));	
	
	backlight_power = (value ? ON : OFF);
}
#if defined(CONFIG_MACH_VITAL)
#define BD60910GU_DEFAULT_BACKLIGHT_BRIGHTNESS		255

static s32 bd60910gu_backlight_off;
static s32 bd60910gu_backlight_brightness = BD60910GU_DEFAULT_BACKLIGHT_BRIGHTNESS;
static u8 bd60910gu_backlight_last_level = 33;
static DEFINE_MUTEX(bd60910gu_backlight_lock);


struct bd60910gu_info
{
    struct i2c_client *client;
    struct mutex io_lock;
};
static struct bd60910gu_info bd60910gu_data;
//sylee
static inline int bd60910gu_i2c_read(struct bd60910gu_info *info, u8 reg)
{
    int regdata;
    regdata=i2c_smbus_read_byte_data(info->client, reg);
	return regdata;
}

static int bd60910gu_reg_read(struct bd60910gu_info *info, u8 reg)
{
    int err;

    mutex_lock(&info->io_lock);

    err = bd60910gu_i2c_read(info, reg);
    if (err < 0)
        dev_err(&info->client->dev, "read for reg 0x%x failed\n", reg);

    mutex_unlock(&info->io_lock);
    return err;
}
//sylee

static inline int bd60910gu_i2c_write(struct bd60910gu_info *info, u8 reg, u8 val)
{
    return i2c_smbus_write_byte_data(info->client, reg, val);
}

static int bd60910gu_reg_write(struct bd60910gu_info *info, u8 reg, u8 val)
{
    int err;

    mutex_lock(&info->io_lock);

    err = bd60910gu_i2c_write(info, reg, val);
    if (err < 0)
        dev_err(&info->client->dev, "Write for reg 0x%x failed\n", reg);

    mutex_unlock(&info->io_lock);
    return err;
}

void bd60910gu_backlight_ctrl(s32 value)
{
	static int prev_level = 0;
	int valuedevide = 0;
	int a=0;
	int b=0;
#if defined(CONFIG_MACH_VITAL)//kimhyuns to prevent LCD Backlight damage by hw's asking
    static int result=-97;
    int old_cfg, old_pull;
    if (result==-97)
    {    
        old_cfg=s3c_gpio_get_cfgpin(GPIO_PS_VOUT);
        old_pull=s3c_gpio_getpull(GPIO_PS_VOUT);
    //    gpio_set_value(GPIO_PS_VOUT,GPIO_PS_VOUT_AF);
    	s3c_gpio_cfgpin(GPIO_PS_VOUT, 0);    
        s3c_gpio_setpull(GPIO_PS_VOUT, S3C_GPIO_PULL_DOWN);	
        msleep(10);	
        result=gpio_get_value(GPIO_PS_VOUT);	
        msleep(10);
        printk("result2 is %d, old_cfg = %d, old_pull = %d\n", result,old_cfg,old_pull);  
        s3c_gpio_cfgpin(GPIO_PS_VOUT, S3C_GPIO_SFN(old_cfg));
        s3c_gpio_setpull(GPIO_PS_VOUT, old_pull);
    }
    if(result==0)
        value=0;
#endif
	
    //printk("bd60910gu_backlight_ctrl(%d)\n", value);

    if(value == 0)
    {
		bd60910gu_reg_write(&bd60910gu_data, 0x01, 0x00);
		gpio_set_value(GPIO_BL_RST, GPIO_LEVEL_LOW);
		printk("value==0\n");
		udelay(10);		
        lcd_power_ctrl(OFF);
    }
    else
    {
    	if (lcd_power == OFF) {
    		lcd_power_ctrl(ON);
			printk("lcd_on\n");						
    	}
		if(prev_level == 0) {
			// reset and enable 
			gpio_set_value(GPIO_BL_RST, GPIO_LEVEL_LOW);
			udelay(10);
			gpio_set_value(GPIO_BL_RST, GPIO_LEVEL_HIGH);
			printk("prev_level==0\n");
		}
		bd60910gu_reg_write(&bd60910gu_data, 0x01, 0x01); //register mode
		bd60910gu_reg_write(&bd60910gu_data, 0x08, 0x33);
		if(value>74){		  
		  valuedevide = 74;
		}else if(value<0){
		  valuedevide = 0;
		}else{
		  valuedevide = value;
		}			
		printk("bd60910gu_backlight_ctrl_devide(%d)\n", valuedevide);
		//vital.boot.temp 
        bd60910gu_reg_write(&bd60910gu_data, 0x03, valuedevide&0x7f);
        #if 0
        a=bd60910gu_reg_read(&bd60910gu_data, 0x03);
        printk("a=(%d)\n", a);
        b=bd60910gu_reg_read(&bd60910gu_data, 0x01);
        printk("b=(%d)\n", b);
        #endif
    }

	prev_level = value;
}


static void bd60910gu_set_backlight_level(u8 level)
{	
	if (backlight_level == level)
		return;

	bd60910gu_backlight_ctrl(level);

	backlight_level = level;
}

static void bd60910gu_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	mutex_lock(&bd60910gu_backlight_lock);
	bd60910gu_backlight_brightness = value;
	bd60910gu_set_backlight_level(bd60910gu_backlight_brightness);
	mutex_unlock(&bd60910gu_backlight_lock);
}

static struct led_classdev bd60910gu_backlight_led  = {
	.name		= "lcd-backlight",
	.brightness = BD60910GU_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = bd60910gu_brightness_set,
};


int bd60910gu_backlight_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    mutex_init(&bd60910gu_data.io_lock);
    printk("bd60910gu_backlight_probe start&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
    /* common for all regulators */
    bd60910gu_data.client = client;

    if (!i2c_check_functionality(client->adapter,
                                 I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;

    i2c_set_clientdata(client, &bd60910gu_data);

	
	led_classdev_register(&client->dev, &bd60910gu_backlight_led);
    printk("bd60910gu_backlight_probe end^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
	return 0;
}

static int bd60910gu_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&bd60910gu_backlight_led); 
	return 0;
}

static const struct i2c_device_id bd60910gu_id[] =
{
    {.name = "bd60910gu-backlight", 0},
    { },
};
MODULE_DEVICE_TABLE(i2c, bd60910gu_id);

static struct i2c_driver bd60910gu_i2c_driver =
{
    .driver = {
        .name = "bd60910gu-backlight",
    },
    .probe = bd60910gu_backlight_probe,
    .remove = __devexit_p(bd60910gu_backlight_remove),
    .id_table = bd60910gu_id,
};



static int __init bd60910gu_backlight_init(void)
{
	int ret=0;
	ret=i2c_add_driver(&bd60910gu_i2c_driver);
    printk("kimhyuns $$$$$$$$$$$$$$$$$$$$$$$$ bd60910gu_backlight_init ret=%d\n",ret);
    return ret;//i2c_add_driver(&bd60910gu_i2c_driver);
}

static void __exit bd60910gu_backlight_exit(void)
{
	i2c_del_driver(&bd60910gu_i2c_driver); 
}
module_init(bd60910gu_backlight_init);
module_exit(bd60910gu_backlight_exit);
#else
#define AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS		255

static s32 ams320fs01_backlight_off;
static s32 ams320fs01_backlight_brightness = AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS;
static u8 ams320fs01_backlight_last_level = 33;
static DEFINE_MUTEX(ams320fs01_backlight_lock);

static void ams320fs01_set_backlight_level(u8 level)
{	
	if (backlight_level == level)
		return;

	backlight_ctrl(level);

	backlight_level = level;
}

static void ams320fs01_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	mutex_lock(&ams320fs01_backlight_lock);
	ams320fs01_backlight_brightness = value;
	ams320fs01_set_backlight_level(ams320fs01_backlight_brightness);
	mutex_unlock(&ams320fs01_backlight_lock);
}

static struct led_classdev ams320fs01_backlight_led  = {
	.name		= "lcd-backlight",
	.brightness = AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = ams320fs01_brightness_set,
};

static int ams320fs01_backlight_probe(struct platform_device *pdev)
{
	led_classdev_register(&pdev->dev, &ams320fs01_backlight_led);
	return 0;
}

static int ams320fs01_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&ams320fs01_backlight_led); 
	return 0;
}

static struct platform_driver ams320fs01_backlight_driver = {
	.probe		= ams320fs01_backlight_probe,
	.remove		= ams320fs01_backlight_remove,
	.driver		= {
		.name		= "ams320fs01-backlight",
		.owner		= THIS_MODULE,
	},
};

static int __init ams320fs01_backlight_init(void)
{
	return platform_driver_register(&ams320fs01_backlight_driver);
}

static void __exit ams320fs01_backlight_exit(void)
{
	platform_driver_unregister(&ams320fs01_backlight_driver); 
}
module_init(ams320fs01_backlight_init);
module_exit(ams320fs01_backlight_exit);
#endif


void s3cfb_init_hw(void)
{
	s3cfb_set_fimd_info();

	s3cfb_set_gpio();
#ifdef CONFIG_FB_S3C_LCD_INIT	
	lcd_gpio_init();
	
	backlight_gpio_init();

	lcd_power_ctrl(ON);

	backlight_level_ctrl(BACKLIGHT_LEVEL_DEFAULT);

	backlight_power_ctrl(ON); 
#else
	lcd_gpio_init();
	
	backlight_gpio_init();
	
	lcd_power = ON;

	backlight_level = BACKLIGHT_LEVEL_DEFAULT;

	backlight_power = ON;
#endif
}

#define LOGO_MEM_BASE		(0x50000000 + 0x0D000000 - 0x100000)	/* SDRAM_BASE + SRAM_SIZE(208MB) - 1MB */
#if defined (CONFIG_FB_S3C_BPP_24) //vital.24bpp 2->4
#define LOGO_MEM_SIZE		(S3C_FB_HRES * S3C_FB_VRES * 4)
#else
#define LOGO_MEM_SIZE		(S3C_FB_HRES * S3C_FB_VRES * 2)
#endif
void s3cfb_display_logo(int win_num)
{
	s3c_fb_info_t *fbi = &s3c_fb_info[0];
	u8 *logo_virt_buf;
	
	logo_virt_buf = ioremap_nocache(LOGO_MEM_BASE, LOGO_MEM_SIZE);

	memcpy(fbi->map_cpu_f1, logo_virt_buf, LOGO_MEM_SIZE);	

	iounmap(logo_virt_buf);
}


#include "s3cfb_progress.h"
#define BAR_DST_VRES          416
#define DELTIMER_THRESHOLD    118

static int progress = 0;

static int progress_flag = OFF;

static struct timer_list progress_timer;

static void progress_timer_handler(unsigned long data)
{
	s3c_fb_info_t *fbi = &s3c_fb_info[1];
	unsigned short *bar_src, *bar_dst;	
	int	i, j, p;

	/* 1 * 12 R5G5B5 BMP (Aligned 4 Bytes) */
/* ECIM-2702128 : 41 -> 18 */
	bar_dst = (unsigned short *)(fbi->map_cpu_f1 + (((S3C_FB_HRES * BAR_DST_VRES) + 41) * 2));
	bar_src = (unsigned short *)(progress_bar + sizeof(progress_bar) - 4);

	for (i = 0; i < 12; i++) {
		for (j = 0; j < 2; j++) {
			p = ((S3C_FB_HRES * i) + (progress * 2) + j);
			*(bar_dst + p) = (*(bar_src - (i * 2)) | 0x8000);
		}
	}	

	progress++;

	if (progress > DELTIMER_THRESHOLD) {
		del_timer(&progress_timer);
	}
	else {
        progress_timer.expires = (get_jiffies_64() + (HZ/28)); //25->28
		progress_timer.function = progress_timer_handler; 
		add_timer(&progress_timer);
	}
}

static unsigned int new_wincon1; 
static unsigned int old_wincon1; 

void s3cfb_start_progress(void)
{
	s3c_fb_info_t *fbi = &s3c_fb_info[1];
	unsigned short *bg_src, *bg_dst;	
	int	i, j, p;
	
	memset(fbi->map_cpu_f1, 0x00, LOGO_MEM_SIZE);	

	/* S3C_FB_HRES * 25 R5G5B5 BMP */
	bg_dst = (unsigned short *)(fbi->map_cpu_f1 + ((S3C_FB_HRES * (BAR_DST_VRES-6)) * 2));
	bg_src = (unsigned short *)(progress_bg + sizeof(progress_bg) - 2);

	for (i = 0; i < 25; i++) {
		for (j = 0; j < S3C_FB_HRES; j++) {
			p = ((S3C_FB_HRES * i) + j);
			if ((*(bg_src - p) & 0x7FFF) == 0x0000)
				*(bg_dst + p) = (*(bg_src - p) & ~0x8000);
			else
				*(bg_dst + p) = (*(bg_src - p) | 0x8000);
		}
	}	

	old_wincon1 = readl(S3C_WINCON1);

	new_wincon1 = S3C_WINCONx_ENLOCAL_DMA | S3C_WINCONx_BUFSEL_0 | S3C_WINCONx_BUFAUTOEN_DISABLE | \
	           S3C_WINCONx_BITSWP_DISABLE | S3C_WINCONx_BYTSWP_DISABLE | S3C_WINCONx_HAWSWP_ENABLE | \
	           S3C_WINCONx_BURSTLEN_16WORD | S3C_WINCONx_BLD_PIX_PIXEL | S3C_WINCONx_BPPMODE_F_16BPP_A555 | \
	           S3C_WINCONx_ALPHA_SEL_0 | S3C_WINCONx_ENWIN_F_ENABLE,

	writel(new_wincon1, S3C_WINCON1);

	init_timer(&progress_timer);
	progress_timer.expires = (get_jiffies_64() + (HZ/10)); 
	progress_timer.function = progress_timer_handler; 
	add_timer(&progress_timer);

	progress_flag = ON;
}

void s3cfb_stop_progress(void)
{
	if (progress_flag == OFF)
		return;

	del_timer(&progress_timer);
	
#ifdef CONFIG_FB_S3C_BPP_24
	writel(s3c_fimd.wincon0,    S3C_WINCON0);
  	s3cfb_onoff_win(&s3c_fb_info[0], ON);
#endif	
	writel(old_wincon1, S3C_WINCON1);
	
	progress_flag = OFF;
}
