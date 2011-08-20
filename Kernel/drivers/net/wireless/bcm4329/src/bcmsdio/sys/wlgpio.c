/* 
 * Copyright (c) 2008 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Revision History
 * ===============
 * 0.0 Initial version WLAN power wakeup
 * 0.1 Second version for Aries platform
 * 0.2 Third version for VinsQ platform
 */
#include <linux/delay.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#include <plat/sdhci.h>

#ifdef CONFIG_MACH_SPICA
#include <mach/spica.h>
#elif CONFIG_MACH_SATURN
#include <mach/saturn.h>
#elif CONFIG_MACH_CYGNUS
#include <mach/cygnus.h>
#elif CONFIG_MACH_INSTINCTQ
#include <mach/instinctq.h>
#elif CONFIG_MACH_BONANZA
#include <mach/bonanza.h>
#endif

#include <plat/devs.h>
#include <linux/spinlock.h>
#include <linux/mmc/host.h>

#if defined(CONFIG_MACH_BONANZA)
#define GPIO_BT_WLAN_REG_ON GPIO_BT_WLAN_EN
#endif

#ifdef CUSTOMER_HW_SAMSUNG

#define WLGPIO_INFO(x) printk x
#define WLGPIO_DEBUG(x)

static unsigned long flags = 0;
static spinlock_t regon_lock = SPIN_LOCK_UNLOCKED;

static void s3c_WLAN_SDIO_on(void)
{
#if defined(CONFIG_MACH_INSTINCTQ)
	s3c_gpio_cfgpin(GPIO_SDIO_CLK, S3C_GPIO_SFN(GPIO_SDIO_CLK_AF));
	s3c_gpio_cfgpin(GPIO_SDIO_CMD, S3C_GPIO_SFN(GPIO_SDIO_CMD_AF));
#else
	s3c_gpio_cfgpin(GPIO_WLAN_CLK, S3C_GPIO_SFN(GPIO_WLAN_CLK_AF));
	s3c_gpio_cfgpin(GPIO_WLAN_CMD, S3C_GPIO_SFN(GPIO_WLAN_CMD_AF));
#endif
	s3c_gpio_cfgpin(GPIO_WLAN_D_0, S3C_GPIO_SFN(GPIO_WLAN_D_0_AF));
	s3c_gpio_cfgpin(GPIO_WLAN_D_1, S3C_GPIO_SFN(GPIO_WLAN_D_1_AF));
	s3c_gpio_cfgpin(GPIO_WLAN_D_2, S3C_GPIO_SFN(GPIO_WLAN_D_2_AF));
	s3c_gpio_cfgpin(GPIO_WLAN_D_3, S3C_GPIO_SFN(GPIO_WLAN_D_3_AF));
#if defined(CONFIG_MACH_INSTINCTQ)
	s3c_gpio_setpull(GPIO_SDIO_CLK, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_SDIO_CMD, S3C_GPIO_PULL_NONE);
#else
	s3c_gpio_setpull(GPIO_WLAN_CLK, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_CMD, S3C_GPIO_PULL_NONE);
#endif
	s3c_gpio_setpull(GPIO_WLAN_D_0, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_1, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_2, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_3, S3C_GPIO_PULL_NONE);
}

static void s3c_WLAN_SDIO_off(void)
{
#if defined(CONFIG_MACH_INSTINCTQ)
	s3c_gpio_cfgpin(GPIO_SDIO_CLK, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_SDIO_CMD, S3C_GPIO_INPUT);
#else
	s3c_gpio_cfgpin(GPIO_WLAN_CLK, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_WLAN_CMD, S3C_GPIO_INPUT);
#endif
	s3c_gpio_cfgpin(GPIO_WLAN_D_0, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_WLAN_D_1, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_WLAN_D_2, S3C_GPIO_INPUT);
	s3c_gpio_cfgpin(GPIO_WLAN_D_3, S3C_GPIO_INPUT);
#if defined(CONFIG_MACH_INSTINCTQ)
	s3c_gpio_setpull(GPIO_SDIO_CLK, S3C_GPIO_PULL_DOWN);
	s3c_gpio_setpull(GPIO_SDIO_CMD, S3C_GPIO_PULL_NONE);
#else
	s3c_gpio_setpull(GPIO_WLAN_CLK, S3C_GPIO_PULL_DOWN);
	s3c_gpio_setpull(GPIO_WLAN_CMD, S3C_GPIO_PULL_NONE);
#endif
	s3c_gpio_setpull(GPIO_WLAN_D_0, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_1, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_2, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(GPIO_WLAN_D_3, S3C_GPIO_PULL_NONE);
}
enum condition {WIFI_POWER = 1, WIFI_RESET};
void bcm_wlan_power_on(int flag)
{
	if (flag == WIFI_POWER) {
		/* WLAN_REG_ON control */

		WLGPIO_INFO(("[WIFI] Device powering ON\n"));

		/* Enable sdio pins and configure it */
		s3c_WLAN_SDIO_on();

		spin_lock_irqsave(&regon_lock, flags);

		/* Power on WLAN chip */ 
		s3c_gpio_cfgpin(GPIO_BT_WLAN_REG_ON, S3C_GPIO_OUTPUT);
		s3c_gpio_cfgpin(GPIO_WLAN_RST_N, S3C_GPIO_OUTPUT);
		/* Set REG ON as High & configure it for sleep mode */
		gpio_set_value(GPIO_BT_WLAN_REG_ON, 1);
		WLGPIO_INFO(("[WIFI] GPIO_BT_WLAN_REG_ON = %d\n", gpio_get_value(GPIO_BT_WLAN_REG_ON)));

		msleep(100);

		/* Active low pin: Make it high for NO RESET & configure it for sleep mode */
		gpio_set_value(GPIO_WLAN_RST_N, 1);
		WLGPIO_INFO(("[WIFI] GPIO_WLAN_RST_N = %d\n", gpio_get_value(GPIO_WLAN_RST_N)));

		s3c_gpio_slp_cfgpin(GPIO_BT_WLAN_REG_ON, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_BT_WLAN_REG_ON, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_RST_N, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_RST_N, S3C_GPIO_PULL_NONE);

		spin_unlock_irqrestore(&regon_lock, flags);

		/*gpio_free(GPIO_BT_WLAN_REG_ON);
		gpio_free(GPIO_WLAN_RST_N);*/

		msleep(100);

#if defined(CONFIG_MACH_QUATTRO)
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc1);
#else
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc2);
#endif
	}
	else if (flag == WIFI_RESET) {
        /* Active low pin: Make it high for NO RESET */
		WLGPIO_INFO(("[WIFI] RESET is HIGH \n"));
		gpio_set_value(GPIO_WLAN_RST_N, 1); 
	}
	else
	{
		WLGPIO_DEBUG(("bcm_wlan_power_on: flag=%d - skip\n", flag));
	}
}


void bcm_wlan_power_off(int flag)
{
	if (flag == WIFI_POWER) {
		/* WLAN_REG_ON control */

		WLGPIO_INFO(("[WIFI] Device powering OFF\n"));

		spin_lock_irqsave(&regon_lock, flags);
		
		/* Active Low: Assert Reset line unconditionally while turning off WIFI*/
		gpio_set_value(GPIO_WLAN_RST_N, 0);
		WLGPIO_INFO(("[WIFI] GPIO_WLAN_RST_N = %d \n", gpio_get_value(GPIO_WLAN_RST_N)));

		s3c_gpio_slp_cfgpin(GPIO_WLAN_RST_N, S3C_GPIO_SLP_OUT0);
		s3c_gpio_slp_setpull_updown(GPIO_WLAN_RST_N, S3C_GPIO_PULL_NONE);

		if(gpio_get_value(GPIO_BT_RST_N) == 0)
		{
			/* Set REG ON as low, only if BT is not operational */
			gpio_set_value(GPIO_BT_WLAN_REG_ON, 0);

			s3c_gpio_slp_cfgpin(GPIO_BT_WLAN_REG_ON, S3C_GPIO_SLP_OUT0);
			s3c_gpio_slp_setpull_updown(GPIO_BT_WLAN_REG_ON, S3C_GPIO_PULL_NONE);

			WLGPIO_INFO(("[WIFI] GPIO_BT_WLAN_REG_ON = %d \n", gpio_get_value(GPIO_BT_WLAN_REG_ON)));
		}

		spin_unlock_irqrestore(&regon_lock, flags);

		/* Disable SDIO pins */
		s3c_WLAN_SDIO_off();

		/* gpio_free(GPIO_BT_WLAN_REG_ON);
		gpio_free(GPIO_WLAN_RST_N); */

		msleep(100);
	    
#if defined(CONFIG_MACH_QUATTRO)
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc1);
#else
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc2);
#endif
	}
	else if (flag == WIFI_RESET) {
		/* Active Low: Assert Reset line */
		WLGPIO_INFO(("[WIFI] RESET is LOW \n"));
		gpio_set_value(GPIO_WLAN_RST_N, 0);
	} 
	else {
		WLGPIO_DEBUG(("bcm_wlan_power_off: flag=%d - skip\n", flag));
	}
}

#endif /* CUSTOMER_HW_SAMSUNG */