/*
 * TP-LINK EAP245 v1 board support
 * 
 * Based on EAP120 from Henryk Heisig <hyniu@o2.pl>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 * 
 * EAP245 v1 board support wifi AC1750 and gigabit LAN:
 * 	- QCA9563-AL3A MIPS 74kc and 2.4GHz wifi
 *  - QCA9880-BR4A 5 GHz wifi ath10k
 *  - AR8033-AL1A 1 gigabit lan port
 * 	- 25Q128CSIG SPI NOR
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/mdio-gpio.h>
#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/ar71xx_regs.h>
#include <linux/platform_data/phy-at803x.h>

#include "common.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-wmac.h"
#include "machtypes.h"
#include "pci.h"

/* GPIO
 * GPIO4 reset the boad when direction is changed, be careful
 * GPIO5 si a master switch for all leds */
#define EAP245_V1_GPIO_LED_RED		1
#define EAP245_V1_GPIO_LED_YEL		9
#define EAP245_V1_GPIO_LED_GRN		7
#define EAP245_V1_GPIO_LED_ALL		5
#define EAP245_V1_GPIO_BTN_RESET	2

#define EAP245_V1_KEYS_POLL_INTERVAL		20	/* msecs */
#define EAP245_V1_KEYS_DEBOUNCE_INTERVAL	(3 * EAP245_V1_KEYS_POLL_INTERVAL)

#define EAP245_V1_GPIO_SMI_MDC		8
#define EAP245_V1_GPIO_SMI_MDIO		10

#define EAP245_V1_LAN_PHYADDR		4

static struct gpio_led eap245_v1_leds_gpio[] __initdata = {
	{
		.name = "eap245-v1:red:system",
		.gpio = EAP245_V1_GPIO_LED_RED,
	}, {
		.name = "eap245-v1:yellow:system",
		.gpio = EAP245_V1_GPIO_LED_YEL,
	}, {
		.name ="eap245-v1:green:system",
		.gpio = EAP245_V1_GPIO_LED_GRN,
	},
};

static struct gpio_keys_button eap245_v1_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = EAP245_V1_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= EAP245_V1_GPIO_BTN_RESET,
		.active_low = 1,
	},
};

static struct mdio_gpio_platform_data eap245_v1_mdio = {
	.mdc		= EAP245_V1_GPIO_SMI_MDC,
	.mdio		= EAP245_V1_GPIO_SMI_MDIO,
	.phy_mask	= ~BIT(EAP245_V1_LAN_PHYADDR),
};

static struct at803x_platform_data eap245_v1_ar8033_data = {
	.disable_smarteee		= 0,
	.enable_rgmii_rx_delay	= 1,
	.enable_rgmii_tx_delay	= 0,
	.fixup_rgmii_tx_delay	= 1,
};

static struct platform_device eap245_v1_phy_device = {
	.name	= "mdio-gpio",
	.id		= 0,
	.dev	= {
		.platform_data = &eap245_v1_mdio, &eap245_v1_ar8033_data
	},
};

static void __init eap245_v1_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f030008);
	u8 *ee = (u8 *) KSEG1ADDR(0x1fff1000);
	u8 wmac_addr[ETH_ALEN];

	ath79_register_leds_gpio(-1, ARRAY_SIZE(eap245_v1_leds_gpio),
								eap245_v1_leds_gpio);

	ath79_register_gpio_keys_polled(-1, EAP245_V1_KEYS_POLL_INTERVAL,
									ARRAY_SIZE(eap245_v1_gpio_keys),
									eap245_v1_gpio_keys);

	ath79_register_m25p80(NULL);

	platform_device_register(&eap245_v1_phy_device);
	ath79_setup_qca956x_eth_cfg(QCA956X_ETH_CFG_SW_PHY_SWAP |
								QCA956X_ETH_CFG_SW_PHY_ADDR_SWAP);

	printk(KERN_DEBUG "Read mac address %pK: %pM\n", mac, mac);
	/* Set 2.4 GHz to mac + 1 */
	/* TODO: if these two line are moved at the end of function lan
	 * iface timeout on TX. Find why, need a sleep, bad pll ? */
	ath79_init_mac(wmac_addr, mac, 1);
	ath79_register_wmac(ee, wmac_addr);

	/* Set lan port to eepromm mac address */
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_eth0_data.mii_bus_dev = &eap245_v1_phy_device.dev;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_RGMII;
	ath79_eth0_data.phy_mask = BIT(EAP245_V1_LAN_PHYADDR);
	ath79_eth0_pll_data.pll_1000 = 0x0e000000;
	ath79_eth0_pll_data.pll_100 = 0x00000101;
	ath79_eth0_pll_data.pll_10 = 0x00001313;
	ath79_register_eth(0);

	/* Initializer pci bus for ath10k (5GHz) */
	ath79_register_pci();
}

MIPS_MACHINE(ATH79_MACH_EAP245_V1, "EAP245-V1", "TP-LINK EAP245 v1",
		eap245_v1_setup);
