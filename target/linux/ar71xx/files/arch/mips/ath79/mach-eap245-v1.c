
/*
 * TP-LINK EAP245 v1 board support
 * Based on TP-Link GPL code for Linux 3.3.8 (hack of mach-ap152.c)
 *
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 * Copyright (c) 2012 Gabor Juhos <juhosg@openwrt.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * EAP245 v1 board support wifi AC1750 and gigabit LAN:
 *  - QCA9563-AL3A MIPS 74kc and 2.4GHz wifi
 *  - QCA9880-BR4A 5 GHz wifi ath10k
 *  - AR8033-AL1A 1 gigabit lan port
 *  - 25Q128CSIG SPI NOR
 */

#include <linux/platform_device.h>
#include <linux/ath9k_platform.h>
#include <asm/mach-ath79/ar71xx_regs.h>
#include <linux/delay.h>
#include <linux/platform_data/phy-at803x.h>
#include <linux/platform_data/mdio-gpio.h>
#include <ath79.h>

#include "common.h"
#include "dev-m25p80.h"
#include "machtypes.h"
#include "pci.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-spi.h"
#include "dev-usb.h"
#include "dev-wmac.h"
#include "956x.h"

#define QCA956X_RESET_EXTERNAL		BIT(28)
#define QCA956X_RESET_GE1_MDIO		BIT(23)
#define QCA956X_RESET_GE0_MDIO		BIT(22)
#define QCA956X_RESET_GE1_MAC		BIT(13)
#define QCA956X_RESET_SGMII_ANALOG	BIT(12)
#define QCA956X_RESET_GE0_MAC		BIT(9)
#define QCA956X_RESET_SGMII		BIT(8)
#define QCA956X_RESET_SWITCH_ANALOG	BIT(2)
#define QCA956X_RESET_SWITCH		BIT(0)

#define QCA956X_PLL_REG_SGMII_SERDES			0x4c
#define QCA956X_PLL_SGMII_SERDES_EN_LOCK_DETECT		BIT(2)
#define QCA956X_PLL_SGMII_SERDES_EN_PLL			BIT(1)

#define QCA956X_MAC_CFG1		0xb9000000
#define QCA956X_MAC_CFG1_SOFT_RST	BIT(31)
#define QCA956X_MAC_CFG1_RX_RST		BIT(19)
#define QCA956X_MAC_CFG1_TX_RST		BIT(18)
#define QCA956X_MAC_CFG1_LOOPBACK	BIT(8)
#define QCA956X_MAC_CFG1_RX_EN		BIT(2)
#define QCA956X_MAC_CFG1_TX_EN		BIT(0)

#define QCA956X_MAC_CFG2		0xb9000004
#define QCA956X_MAC_CFG2_IF_1000	BIT(9)
#define QCA956X_MAC_CFG2_IF_10_100	BIT(8)
#define QCA956X_MAC_CFG2_HUGE_FRAME_EN	BIT(5)
#define QCA956X_MAC_CFG2_LEN_CHECK	BIT(4)
#define QCA956X_MAC_CFG2_PAD_CRC_EN	BIT(2)
#define QCA956X_MAC_CFG2_FDX		BIT(0)

#define QCA956X_MAC_FIFO_CFG_0		0xb9000048
#define QCA956X_MAC_FIFO_CFG_1		0xb900004c
#define QCA956X_MAC_FIFO_CFG_2		0xb9000050
#define QCA956X_MAC_FIFO_CFG_3		0xb9000054
#define QCA956X_MAC_FIFO_CFG_4		0xb9000058
#define QCA956X_MAC_FIFO_CFG_5		0xb900005c

#define QCA956X_SGMII_SERDES_VCO_SLOW_SHIFT	10
#define QCA956X_SGMII_SERDES_VCO_SLOW_MASK	0x1

#define QCA956X_SGMII_SERDES_VCO_FAST_SHIFT	9
#define QCA956X_SGMII_SERDES_VCO_FAST_MASK	0x1

#define QCA956X_SGMII_SERDES_RES_CALIBRATION_SHIFT	23
#define QCA956X_SGMII_SERDES_RES_CALIBRATION_MASK	0xf

#if 0
#define SGMII_SERDES_ADDRESS                                         (QCA956X_GMAC_SGMII_BASE + 0x18) 
#define SGMII_SERDES_LOCK_DETECT_STATUS_MASK                         0x00008000


#define SGMII_SERDES_CDR_BW_MASK                                     0x00000006
#define SGMII_SERDES_CDR_BW_SET(x)                                   (((x) << 1) & SGMII_SERDES_CDR_BW_MASK)

#define SGMII_SERDES_TX_DR_CTRL_MASK                                 0x00000070
#define SGMII_SERDES_TX_DR_CTRL_SET(x)                               (((x) << 4) & SGMII_SERDES_TX_DR_CTRL_MASK)

#define SGMII_SERDES_PLL_BW_MASK                                     0x00000100
#define SGMII_SERDES_PLL_BW_SET(x)                                   (((x) << 8) & SGMII_SERDES_PLL_BW_MASK)

#define SGMII_SERDES_EN_SIGNAL_DETECT_MASK                           0x00010000
#define SGMII_SERDES_EN_SIGNAL_DETECT_SET(x)                         (((x) << 16) & SGMII_SERDES_EN_SIGNAL_DETECT_MASK)

#define SGMII_SERDES_FIBER_SDO_MASK                                  0x00020000
#define SGMII_SERDES_FIBER_SDO_SET(x)                                (((x) << 17) & SGMII_SERDES_FIBER_SDO_MASK)

#define SGMII_SERDES_VCO_REG_MASK                                    0x78000000
#define SGMII_SERDES_VCO_REG_SET(x)                                  (((x) << 27) & SGMII_SERDES_VCO_REG_MASK)
#endif

/* GPIO
 * GPIO4 reset the boad when direction is changed, be careful
 * GPIO5 si a master switch for all leds */
#define EAP245_V1_GPIO_LED_RED         1
#define EAP245_V1_GPIO_LED_YEL         9
#define EAP245_V1_GPIO_LED_GRN         7
#define EAP245_V1_GPIO_LED_ALL         5
#define EAP245_V1_GPIO_BTN_RESET       2
#define EAP245_V1_KEYS_POLL_INTERVAL        20	/* msecs */
#define EAP245_V1_KEYS_DEBOUNCE_INTERVAL    (3 * EAP245_V1_KEYS_POLL_INTERVAL)

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
		.name = "eap245-v1:green:system",
		.gpio = EAP245_V1_GPIO_LED_GRN,
	},
};

static struct gpio_keys_button eap245_v1_gpio_keys[] __initdata = {
	{
		.desc = "Reset button",
		.type = EV_KEY,
		.code = KEY_RESTART,
		.debounce_interval = EAP245_V1_KEYS_DEBOUNCE_INTERVAL,
		.gpio = EAP245_V1_GPIO_BTN_RESET,
		.active_low = 1,
	 },
};

static struct at803x_platform_data eap245_v1_ar8033_data = {
	.disable_smarteee = 0,
	.enable_rgmii_rx_delay = 1,
	.enable_rgmii_tx_delay = 0,
	.fixup_rgmii_tx_delay = 1,
};

static struct mdio_gpio_platform_data eap245_v1_mdio = {
	.mdc		= EAP245_V1_GPIO_SMI_MDC,
	.mdio		= EAP245_V1_GPIO_SMI_MDIO,
	.phy_mask	= ~BIT(EAP245_V1_LAN_PHYADDR),
};

static struct platform_device eap245_v1_phy_device = {
	.name	= "mdio-gpio",
	.id	= 0,
	.dev	= {
		.platform_data = &eap245_v1_mdio, &eap245_v1_ar8033_data
	},
};

/* MDIO initialization code from TP-Link GPL code */
typedef unsigned int ath_reg_t;

#define ath_reg_rd(_phys)	(*(volatile ath_reg_t *)KSEG1ADDR(_phys))

#define ath_reg_wr_nf(_phys, _val) \
	((*(volatile ath_reg_t *)KSEG1ADDR(_phys)) = (_val))
#define ath_reg_wr(_phys, _val) do {	\
	ath_reg_wr_nf(_phys, _val);	\
	ath_reg_rd(_phys);		\
} while(0)
#define ath_reg_rmw_set(_reg, _mask)	do {			\
	ath_reg_wr((_reg), (ath_reg_rd((_reg)) | (_mask)));	\
	ath_reg_rd((_reg));					\
} while(0)
#define ath_reg_rmw_clear(_reg, _mask) do {			\
	ath_reg_wr((_reg), (ath_reg_rd((_reg)) & ~(_mask)));	\
	ath_reg_rd((_reg));					\
} while(0)

static void __init athrs_sgmii_res_cal(void)
{
	unsigned int read_data;
	unsigned int reversed_sgmii_value;
	unsigned int i = 0;
	unsigned int vco_fast, vco_slow;
	unsigned int startValue = 0, endValue = 0;

	ath79_pll_wr(QCA956X_PLL_REG_SGMII_SERDES,
		   QCA956X_PLL_SGMII_SERDES_EN_LOCK_DETECT |
		   QCA956X_PLL_SGMII_SERDES_EN_PLL);

	read_data = ath_reg_rd(SGMII_SERDES_ADDRESS);

	vco_fast = (read_data >> QCA956X_SGMII_SERDES_VCO_FAST_SHIFT) & QCA956X_SGMII_SERDES_VCO_FAST_MASK;
	vco_slow = (read_data >> QCA956X_SGMII_SERDES_VCO_SLOW_SHIFT) & QCA956X_SGMII_SERDES_VCO_SLOW_MASK;
	/* set resistor Calibration from 0000 -> 1111 */
	for (i = 0; i < 0x10; i++) {
		read_data = ath_reg_rd(SGMII_SERDES_ADDRESS);
		read_data &= ~(QCA956X_SGMII_SERDES_RES_CALIBRATION_MASK << QCA956X_SGMII_SERDES_RES_CALIBRATION_SHIFT);
		read_data |= (i & QCA956X_SGMII_SERDES_RES_CALIBRATION_MASK) << QCA956X_SGMII_SERDES_RES_CALIBRATION_SHIFT;
		ath_reg_wr(SGMII_SERDES_ADDRESS, read_data);

		udelay(50);

		read_data = ath_reg_rd(SGMII_SERDES_ADDRESS);
		if(	(vco_fast != (read_data >> QCA956X_SGMII_SERDES_VCO_FAST_SHIFT) & QCA956X_SGMII_SERDES_VCO_FAST_MASK) ||
			(vco_slow != (read_data >> QCA956X_SGMII_SERDES_VCO_SLOW_SHIFT) & QCA956X_SGMII_SERDES_VCO_SLOW_MASK)) {
			if (startValue == 0) {
				startValue = endValue = i;
			} else {
				endValue = i;
			}
		}
		vco_fast = (read_data >> QCA956X_SGMII_SERDES_VCO_FAST_SHIFT) & QCA956X_SGMII_SERDES_VCO_FAST_MASK;
		vco_slow = (read_data >> QCA956X_SGMII_SERDES_VCO_SLOW_SHIFT) & QCA956X_SGMII_SERDES_VCO_SLOW_MASK;
	}

	if (startValue == 0) {
		/* No boundary found, use middle value for resistor calibration value */
		reversed_sgmii_value = 0x7;
	} else {
		/* get resistor calibration from the middle of boundary */
		reversed_sgmii_value = (startValue + endValue) / 2;
	}

	read_data = ath_reg_rd(SGMII_SERDES_ADDRESS);
	read_data &= ~(QCA956X_SGMII_SERDES_RES_CALIBRATION_MASK << QCA956X_SGMII_SERDES_RES_CALIBRATION_SHIFT);
	read_data |= (reversed_sgmii_value & QCA956X_SGMII_SERDES_RES_CALIBRATION_MASK) << QCA956X_SGMII_SERDES_RES_CALIBRATION_SHIFT;
	ath_reg_wr(SGMII_SERDES_ADDRESS, read_data);

	ath79_pll_wr(QCA956X_PLL_REG_SGMII_SERDES,
		   QCA956X_PLL_SGMII_SERDES_EN_LOCK_DETECT |
		   QCA956X_PLL_SGMII_SERDES_EN_PLL);

	ath_reg_rmw_set(SGMII_SERDES_ADDRESS,
			SGMII_SERDES_CDR_BW_SET(3) |
			SGMII_SERDES_TX_DR_CTRL_SET(1) |
			SGMII_SERDES_PLL_BW_SET(1) |
			SGMII_SERDES_EN_SIGNAL_DETECT_SET(1) |
			SGMII_SERDES_FIBER_SDO_SET(1) |
			SGMII_SERDES_VCO_REG_SET(3));

	ath79_device_reset_clear(QCA956X_RESET_SGMII_ANALOG);
	udelay(25);
	ath79_device_reset_clear(QCA956X_RESET_SGMII);
	while (!
	       (ath_reg_rd(SGMII_SERDES_ADDRESS) &
		SGMII_SERDES_LOCK_DETECT_STATUS_MASK)) ;
}

#define QCA956X_SGMII_RESET_RX_CLK_N_RESET	0x0
#define QCA956X_SGMII_RESET_HW_RX_125M_N	BIT(4)
#define QCA956X_SGMII_RESET_TX_125M_N		BIT(3)
#define QCA956X_SGMII_RESET_RX_125M_N		BIT(2)
#define QCA956X_SGMII_RESET_TX_CLK_N		BIT(1)
#define QCA956X_SGMII_RESET_RX_CLK_N		BIT(0)

#define QCA956X_MR_AN_CONTROL_PHY_RESET		BIT(15)
#define QCA956X_MR_AN_CONTROL_AN_ENABLE		BIT(12)

#define QCA956X_SGMII_LINK_WAR_MAX_TRY		10
#define QCA956X_SGMII_RESET_ADDRESS		(QCA956X_GMAC_SGMII_BASE + 0x14)
#define QCA956X_SGMII_CONFIG_ADDRESS		(QCA956X_GMAC_SGMII_BASE + 0x34)
#define QCA956X_SGMII_DEBUG_ADDRESS		(QCA956X_GMAC_SGMII_BASE + 0x58)
#define QCA956X_MR_AN_CONTROL_ADDRESS		0x1807001c

#define QCA956X_SGMII_CONFIG_MODE_CTRL_SHIFT	0
#define QCA956X_SGMII_CONFIG_MODE_CTRL_MASK     0x7

static void __init athrs_sgmii_set_up(void)
{
	uint32_t status = 0, count = 0, t;

	ath_reg_wr(AR71XX_GE0_BASE, 0x3f);
	udelay(10);
	ath_reg_wr(AR71XX_MII_BASE, 0x3c041);

	t = (2 & QCA956X_SGMII_CONFIG_MODE_CTRL_MASK) << QCA956X_SGMII_CONFIG_MODE_CTRL_SHIFT;
	ath_reg_wr(QCA956X_SGMII_CONFIG_ADDRESS, t);

	ath_reg_wr(QCA956X_MR_AN_CONTROL_ADDRESS, QCA956X_MR_AN_CONTROL_AN_ENABLE |
		   QCA956X_MR_AN_CONTROL_PHY_RESET);

	/*
	 * SGMII reset sequence suggested by systems team.
	 */
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, QCA956X_SGMII_RESET_RX_CLK_N_RESET);

	t = QCA956X_SGMII_RESET_HW_RX_125M_N;
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, t);

	t |= QCA956X_SGMII_RESET_RX_125M_N;
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, t);

	t |= QCA956X_SGMII_RESET_TX_125M_N;
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, t);

	t |= QCA956X_SGMII_RESET_RX_CLK_N;
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, t);

	t |= QCA956X_SGMII_RESET_TX_CLK_N;
	ath_reg_wr(QCA956X_SGMII_RESET_ADDRESS, t);

	ath_reg_rmw_clear(QCA956X_MR_AN_CONTROL_ADDRESS,
			  QCA956X_MR_AN_CONTROL_PHY_RESET);
	/*
	 * WAR::Across resets SGMII link status goes to weird
	 * state.
	 * if 0xb8070058 (SGMII_DEBUG register) reads other then 0x1f or 0x10
	 * for sure we are in bad  state.
	 * Issue a PHY reset in QCA956X_MR_AN_CONTROL_ADDRESS to keep going.
	 */
	status = (ath_reg_rd(QCA956X_SGMII_DEBUG_ADDRESS) & 0xff);
	while (!(status == 0xf || status == 0x10)) {

		ath_reg_rmw_set(QCA956X_MR_AN_CONTROL_ADDRESS,
				QCA956X_MR_AN_CONTROL_PHY_RESET);
		udelay(100);
		ath_reg_rmw_clear(QCA956X_MR_AN_CONTROL_ADDRESS,
				  QCA956X_MR_AN_CONTROL_PHY_RESET);
		if (count++ == QCA956X_SGMII_LINK_WAR_MAX_TRY) {
			printk("Max resets limit reached exiting...\n");
			break;
		}
		status = (ath_reg_rd(QCA956X_SGMII_DEBUG_ADDRESS) & 0xff);
	}

}

static void __init ath_gmac_mii_setup(void)
{
	unsigned int mgmt_cfg_val;
	ath_reg_wr(SWITCH_CLOCK_SPARE_ADDRESS, 0xc5200);

	mgmt_cfg_val = 7;
	ath_reg_wr(ETH_CFG_ADDRESS, ETH_CFG_ETH_RXDV_DELAY_SET(3) |
		   ETH_CFG_ETH_RXD_DELAY_SET(3) |
		   ETH_CFG_RGMII_GE0_SET(1) | ETH_CFG_GE0_SGMII_SET(1));
	ath_reg_wr(ETH_XMII_ADDRESS, ETH_XMII_TX_INVERT_SET(1) |
		   ETH_XMII_RX_DELAY_SET(2) |
		   ETH_XMII_TX_DELAY_SET(1) | ETH_XMII_GIGE_SET(1));
	udelay(1000);
	ath_reg_wr(ATH_MAC_MII_MGMT_CFG, mgmt_cfg_val | BIT(31));
	ath_reg_wr(ATH_MAC_MII_MGMT_CFG, mgmt_cfg_val);
}


static void __init ath_gmac_hw_start(void)
{
	ath_reg_wr(QCA956X_MAC_CFG2, (QCA956X_MAC_CFG2_PAD_CRC_EN |
				  QCA956X_MAC_CFG2_LEN_CHECK |
				  QCA956X_MAC_CFG2_IF_10_100));
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_0, 0x1f00);
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_1, 0x10ffff);
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_2, 0xAAA0555);
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_4, 0x3ffff);
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_5, 0x7eccf);
	ath_reg_wr(QCA956X_MAC_FIFO_CFG_3, 0x1f00140);
}

static void __init ath_gmac_enet_initialize(void)
{
	unsigned int mask;
	athrs_sgmii_res_cal();

	ath79_device_reset_set(QCA956X_RESET_SGMII_ANALOG);
	mdelay(100);
	ath79_device_reset_clear(QCA956X_RESET_SGMII_ANALOG);
	mdelay(100);
	mask = QCA956X_RESET_SGMII | QCA956X_RESET_SGMII_ANALOG
	    | QCA956X_RESET_EXTERNAL | QCA956X_RESET_SWITCH_ANALOG
	    | QCA956X_RESET_SWITCH;
	ath79_device_reset_set(mask);
	mdelay(100);
	mask = QCA956X_RESET_SGMII | QCA956X_RESET_SGMII_ANALOG
	    | QCA956X_RESET_EXTERNAL;
	ath79_device_reset_clear(mask);
	mdelay(100);
	ath_reg_rmw_set(QCA956X_MAC_CFG1,
			QCA956X_MAC_CFG1_SOFT_RST | QCA956X_MAC_CFG1_RX_RST |
			QCA956X_MAC_CFG1_TX_RST);

	mask = QCA956X_RESET_GE0_MAC | QCA956X_RESET_GE1_MAC | QCA956X_RESET_GE0_MDIO | QCA956X_RESET_GE1_MDIO;
	ath79_device_reset_set(mask);
	mdelay(100);
	ath79_device_reset_clear(mask);
	mdelay(100);
	mdelay(10);

	ath_gmac_mii_setup();

	athrs_sgmii_set_up();

	ath_gmac_hw_start();
}

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

	/* This board need initialization code for LAN */
	ath_gmac_enet_initialize();
	platform_device_register(&eap245_v1_phy_device);
        ath79_setup_qca956x_eth_cfg(QCA956X_ETH_CFG_SW_PHY_SWAP | QCA956X_ETH_CFG_SW_PHY_ADDR_SWAP);

	printk(KERN_DEBUG "Read mac address %pK: %pM\n", mac, mac);
	/* Set 2.4 GHz to mac + 1 */
	ath79_init_mac(wmac_addr, mac, 1);
	ath79_register_wmac(ee, wmac_addr);

	/* Initializer pci bus for ath10k (5GHz) */
	ath79_register_pci();

	/* Set lan port to eepromm mac address */
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);
	ath79_eth0_data.mii_bus_dev = &ath79_mdio0_device.dev;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_SGMII;
	ath79_eth0_data.speed = SPEED_1000;
	ath79_eth0_data.duplex = DUPLEX_FULL;
	ath79_eth0_data.phy_mask = BIT(EAP245_V1_LAN_PHYADDR);
	ath79_register_eth(0);
}

MIPS_MACHINE(ATH79_MACH_EAP245_V1, "EAP245-V1", "TP-LINK EAP245 v1",
	     eap245_v1_setup);
