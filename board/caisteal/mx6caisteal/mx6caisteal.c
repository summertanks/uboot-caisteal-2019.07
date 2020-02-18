// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;


//--------------------------------------------------------------
// Pad Configuration

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (			                \
        PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
        PAD_CTL_DSE_40ohm | PAD_CTL_HYS |                       \
        PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define JTAG_PAD (PAD_CTL_PKE | PAD_CTL_PUE |                   \
        PAD_CTL_PUS_47K_UP  | PAD_CTL_DSE_60ohm)

#define JTAG_PAD_TDO (PAD_CTL_PKE | PAD_CTL_PUS_100K_UP |       \
        PAD_CTL_DSE_60ohm   | PAD_CTL_SRE_FAST)

#define JTAG_PAD_MOD (PAD_CTL_PKE | PAD_CTL_PUE |               \
        PAD_CTL_PUS_100K_UP  | PAD_CTL_DSE_60ohm)

#define UNUSED_PULLUP ( PAD_CTL_PUS_100K_UP  |                  \
		PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |         \
		PAD_CTL_HYS | PAD_CTL_SRE_SLOW )

//--------------------------------------------------------------
//Pad Declarations

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RTS_B__UART1_DCE_RTS  | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_UART1_CTS_B__UART1_DCE_CTS  | MUX_PAD_CTRL(UART_PAD_CTRL),

};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	
	/* CLK 	TODO: Check Pad CTRL - No pullup in the design 
	 * 	TODO: 22 Ohm serial impedence */
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	
	/* CMD */
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	
	/* Data */
	MX6_PAD_SD1_DATA0__USDHC1_DATA0  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_READY_B__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE1_B__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CLE__USDHC1_DATA7   | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* VSELECT - Not Connected */
	MX6_PAD_ENET1_RX_EN__USDHC1_VSELECT | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* CD - Not Connected */
	MX6_PAD_CSI_DATA05__USDHC1_CD_B     | MUX_PAD_CTRL(NO_PAD_CTRL),
	
	/* RST_B */
	MX6_PAD_NAND_WP_B__USDHC1_RESET_B   | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* WP - Not connected */
	MX6_PAD_CSI_DATA04__USDHC1_WP       | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* LCTL - Not Connected */
	MX6_PAD_ENET1_RX_DATA0__USDHC1_LCTL | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const qspi1_pads[] = {

	/* SS0 TODO: pullup 10K */
	MX6_PAD_NAND_WE_B__QSPI_B_SS0_B     | MUX_PAD_CTRL(NO_PAD_CTRL),
	
	/* SS1 TODO: Not Connected */
	MX6_PAD_NAND_DATA00__QSPI_B_SS1_B   | MUX_PAD_CTRL(NO_PAD_CTRL),
	
	/* DQS TODO: Not Connected */
	MX6_PAD_NAND_DATA01__QSPI_B_DQS     | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* DATA */
	MX6_PAD_NAND_DATA02__QSPI_B_DATA00  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_DATA03__QSPI_B_DATA01  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_DATA04__QSPI_B_DATA02  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NAND_DATA05__QSPI_B_DATA03  | MUX_PAD_CTRL(NO_PAD_CTRL),

};

static iomux_v3_cfg_t const i2c1_pads[] = {

	MX6_PAD_UART4_RX_DATA__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
	MX6_PAD_UART4_TX_DATA__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

static iomux_v3_cfg_t const jtag_pads[] = {

	MX6_PAD_JTAG_TMS__SJC_TMS      | MUX_PAD_CTRL(JTAG_PAD),
	MX6_PAD_JTAG_TRST_B__SJC_TRSTB | MUX_PAD_CTRL(JTAG_PAD),
	MX6_PAD_JTAG_TDI__SJC_TDI      | MUX_PAD_CTRL(JTAG_PAD),
	MX6_PAD_JTAG_TDO__SJC_TDO      | MUX_PAD_CTRL(JTAG_PAD_TDO),
	MX6_PAD_JTAG_TCK__SJC_TCK      | MUX_PAD_CTRL(JTAG_PAD),
	MX6_PAD_JTAG_MOD__SJC_MOD      | MUX_PAD_CTRL(JTAG_PAD_MOD),

};

static iomux_v3_cfg_t const boot_cfg_pads[] = {

	MX6_PAD_LCD_DATA00__SRC_BT_CFG00 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA01__SRC_BT_CFG01 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA02__SRC_BT_CFG02 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA03__SRC_BT_CFG03 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA04__SRC_BT_CFG04 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA05__SRC_BT_CFG05 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA06__SRC_BT_CFG06 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA07__SRC_BT_CFG07 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA08__SRC_BT_CFG08 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA09__SRC_BT_CFG09 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA10__SRC_BT_CFG10 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA11__SRC_BT_CFG11 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA12__SRC_BT_CFG12 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA13__SRC_BT_CFG13 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA14__SRC_BT_CFG14 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA15__SRC_BT_CFG15 | MUX_PAD_CTRL(UNUSED_PULLUP),

        MX6_PAD_LCD_DATA16__SRC_BT_CFG24 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA17__SRC_BT_CFG25 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA18__SRC_BT_CFG26 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA19__SRC_BT_CFG27 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA20__SRC_BT_CFG28 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA21__SRC_BT_CFG29 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA22__SRC_BT_CFG30 | MUX_PAD_CTRL(UNUSED_PULLUP),
        MX6_PAD_LCD_DATA23__SRC_BT_CFG31 | MUX_PAD_CTRL(UNUSED_PULLUP),

};

//--------------------------------------------------------------------------
// iomux setup

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_usdhc(void)
{
        imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
}

static void setup_iomux_qspi(void)
{
        imx_iomux_v3_setup_multiple_pads(qspi1_pads, ARRAY_SIZE(qspi1_pads));
}

static void setup_iomux_i2c(void)
{
        imx_iomux_v3_setup_multiple_pads(i2c1_pads, ARRAY_SIZE(i2c1_pads));
}

static void setup_iomux_jtag(void)
{
        imx_iomux_v3_setup_multiple_pads(jtag_pads, ARRAY_SIZE(jtag_pads));
}

static void setup_iomux_boot_cfg(void)
{
        imx_iomux_v3_setup_multiple_pads(boot_cfg_pads, ARRAY_SIZE(boot_cfg_pads));
}

//--------------------------------------------------------------------------
//

#ifdef CONFIG_DM_PMIC
int power_init_board(void)
{
        struct udevice *dev;
        int ret, dev_id, rev_id;
        unsigned int reg;

        // TODO: Confirm the PMIC and change Device name
        ret = pmic_get("pfuze3000", &dev);
        if (ret == -ENODEV)
                return 0;
        if (ret != 0)
                return ret;

        dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
        rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
        printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

        // TODO : Confirm Voltage Selection
        /* disable Low Power Mode during standby mode */
        reg = pmic_reg_read(dev, PFUZE3000_LDOGCTL);
        reg |= 0x1;
        pmic_reg_write(dev, PFUZE3000_LDOGCTL, reg);

        /* SW1B step ramp up time from 2us to 4us/25mV */
        pmic_reg_write(dev, PFUZE3000_SW1BCONF, 0x40);

        /* SW1B mode to APS/PFM */
        pmic_reg_write(dev, PFUZE3000_SW1BMODE, 0xc);

        /* SW1B standby voltage set to 0.975V */
        pmic_reg_write(dev, PFUZE3000_SW1BSTBY, 0xb);

        return 0;
}
#endif

int dram_init(void)
{
        gd->ram_size = imx_ddr_size();
        return 0;
}


#ifdef CONFIG_FSL_QSPI
static int board_qspi_init(void)
{
	/* Set the clock */
	enable_qspi_clk(0);

	return 0;
}
#endif

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)
#define USDHC1_PWR_GPIO	IMX_GPIO_NR(1, 9)
#define USDHC2_CD_GPIO	IMX_GPIO_NR(4, 5)
#define USDHC2_PWR_GPIO	IMX_GPIO_NR(4, 10)


#ifdef CONFIG_USB_EHCI_MX6
#ifndef CONFIG_DM_USB

#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

/* At default the 3v3 enables the MIC2026 for VBUS power */
static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif
#endif

int board_early_init_f(void)
{
	setup_iomux_boot_cfg();
	setup_iomux_uart();
	setup_iomux_jtag();
	setup_iomux_qspi();
	setup_iomux_usdhc();
	setup_iomux_i2c();
	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;


#ifdef CONFIG_USB_EHCI_MX6
#ifndef CONFIG_DM_USB
	setup_usb();
#endif
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif


	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "CAISTEAL");
	env_set("board_rev", "14X14");
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: Caisteal MX6UL 14x14 \n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <linux/libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>


static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
#ifdef CONFIG_TARGET_MX6UL_9X9_EVK
	.grp_ddr_type = 0x00080000,
#else
	.grp_ddr_type = 0x000c0000,
#endif
};

#ifdef CONFIG_TARGET_MX6UL_9X9_EVK
static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000000,
	.dram_odt1 = 0x00000000,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00003030,
	.dram_sdqs1 = 0x00003030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x20000000,
	.p0_mprddlctl = 0x4040484f,
	.p0_mpwrdlctl = 0x40405247,
	.mpzqlp2ctl = 0x1b4700c7,
};

static struct mx6_lpddr2_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 2,
	.width = 16,
	.banks = 4,
	.rowaddr = 14,
	.coladdr = 10,
	.trcd_lp = 1500,
	.trppb_lp = 1500,
	.trpab_lp = 2000,
	.trasmin = 4250,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 18,
	.ncs = 1,
	.cs1_mirror = 0,
	.walat = 0,
	.ralat = 5,
	.mif3_mode = 3,
	.bi_on = 1,
	.rtt_wr = 0,        /* LPDDR2 does not need rtt_wr rtt_nom */
	.rtt_nom = 0,
	.sde_to_rst = 0,    /* LPDDR2 does not need this field */
	.rst_to_cke = 0x10, /* JEDEC value for LPDDR2: 200us */
	.ddr_type = DDR_TYPE_LPDDR2,
	.refsel = 0,	/* Refresh cycles at 64KHz */
	.refr = 3,	/* 4 refresh commands per refresh cycle */
};

#else
static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x41570155,
	.p0_mprddlctl = 0x4040474A,
	.p0_mpwrdlctl = 0x40405550,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 0,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
	.refsel = 0,	/* Refresh cycles at 64KHz */
	.refr = 1,	/* 2 refresh commands per refresh cycle */
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	mx6ul_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&ddr_sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
