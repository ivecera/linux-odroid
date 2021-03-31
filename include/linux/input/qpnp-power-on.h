/* Copyright (c) 2012-2015, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef QPNP_PON_H
#define QPNP_PON_H

#include <linux/errno.h>

/**
 * enum pon_trigger_source: List of PON trigger sources
 * %PON_SMPL:		PON triggered by SMPL - Sudden Momentary Power Loss
 * %PON_RTC:		PON triggered by RTC alarm
 * %PON_DC_CHG:		PON triggered by insertion of DC charger
 * %PON_USB_CHG:	PON triggered by insertion of USB
 * %PON_PON1:		PON triggered by other PMIC (multi-PMIC option)
 * %PON_CBLPWR_N:	PON triggered by power-cable insertion
 * %PON_KPDPWR_N:	PON triggered by long press of the power-key
 */
enum pon_trigger_source {
	PON_SMPL = 1,
	PON_RTC,
	PON_DC_CHG,
	PON_USB_CHG,
	PON_PON1,
	PON_CBLPWR_N,
	PON_KPDPWR_N,
};

/**
 * enum pon_power_off_type: Possible power off actions to perform
 * %PON_POWER_OFF_RESERVED:          Reserved, not used
 * %PON_POWER_OFF_WARM_RESET:        Reset the MSM but not all PMIC peripherals
 * %PON_POWER_OFF_SHUTDOWN:          Shutdown the MSM and PMIC completely
 * %PON_POWER_OFF_HARD_RESET:        Reset the MSM and all PMIC peripherals
 */
enum pon_power_off_type {
	PON_POWER_OFF_RESERVED		= 0x00,
	PON_POWER_OFF_WARM_RESET	= 0x01,
	PON_POWER_OFF_SHUTDOWN		= 0x04,
	PON_POWER_OFF_HARD_RESET	= 0x07,
	PON_POWER_OFF_MAX_TYPE		= 0x10,
};

enum pon_restart_reason {
	/* 0 ~ 31 for common defined features */
	PON_RESTART_REASON_UNKNOWN		= 0x00,
	PON_RESTART_REASON_RECOVERY		= 0x01,
	PON_RESTART_REASON_BOOTLOADER		= 0x02,
	PON_RESTART_REASON_RTC			= 0x03,
	PON_RESTART_REASON_DMVERITY_CORRUPTED	= 0x04,
	PON_RESTART_REASON_DMVERITY_ENFORCE	= 0x05,
	PON_RESTART_REASON_KEYS_CLEAR		= 0x06,

	/* 32 ~ 63 for OEMs/ODMs secific features */
	PON_RESTART_REASON_OEM_MIN		= 0x20,
	PON_RESTART_REASON_OEM_MAX		= 0x3f,
};

enum pon_power_off_reason {
	/* Software */
	PON_POWER_OFF_REASON_SOFT				= 0,
	/* PS_HOLD/MSM controlled shutdown */
	PON_POWER_OFF_REASON_PS_HOLD				= 1,
	/* PMIC watchdog */
	PON_POWER_OFF_REASON_PMIC_WD				= 2,
	/* Keypad_Reset1 */
	PON_POWER_OFF_REASON_GP1				= 3,
	/* Keypad_Reset2 */
	PON_POWER_OFF_REASON_GP2				= 4,
	/* Simultaneous power key and reset line */
	PON_POWER_OFF_REASON_KPDPWR_AND_RESIN			= 5,
	/* Reset line/Volume Down Key */
	PON_POWER_OFF_REASON_RESIN_N				= 6,
	/* Long Power Key hold */
	PON_POWER_OFF_REASON_KPDPWR_N				= 7,
	/* Charger ENUM_TIMER, BOOT_DONE */
	PON_POWER_OFF_REASON_CHARGER				= 11,
	/* Thermal Fault Tolerance */
	PON_POWER_OFF_REASON_TFT				= 12,
	/* Under Voltage Lock Out */
	PON_POWER_OFF_REASON_UVLO				= 13,
	/* Overtemp */
	PON_POWER_OFF_REASON_OTST3				= 14,
	/* Stage 3 reset */
	PON_POWER_OFF_REASON_STAGE3				= 15,
	/* GP_FAULT0 */
	PON_POWER_OFF_REASON_GP_FAULT0				= 16,
	/* GP_FAULT1 */
	PON_POWER_OFF_REASON_GP_FAULT1				= 17,
	/* GP_FAULT2 */
	PON_POWER_OFF_REASON_GP_FAULT2				= 18,
	/* GP_FAULT3 */
	PON_POWER_OFF_REASON_GP_FAULT3				= 19,
	/* MBG_FAULT */
	PON_POWER_OFF_REASON_MBG_FAULT				= 20,
	/* Over Voltage Lock Out */
	PON_POWER_OFF_REASON_OVLO				= 21,
	/* Under Voltage Lock Out */
	PON_POWER_OFF_REASON_GEN2_UVLO				= 22,
	/* AVDD_RB */
	PON_POWER_OFF_REASON_AVDD_RB				= 23,
	/* FAULT_FAULT_N */
	PON_POWER_OFF_REASON_FAULT_FAULT_N			= 27,
	/* FAULT_PBS_WATCHDOG_TO */
	PON_POWER_OFF_REASON_FAULT_PBS_WATCHDOG_TO		= 28,
	/* FAULT_PBS_NACK */
	PON_POWER_OFF_REASON_FAULT_PBS_NACK			= 29,
	/* FAULT_RESTART_PON */
	PON_POWER_OFF_REASON_FAULT_RESTART_PON			= 30,
	/* Overtemp */
	PON_POWER_OFF_REASON_GEN2_OTST3				= 31,
	/* S3_RESET_FAULT_N */
	PON_POWER_OFF_REASON_S3_RESET_FAULT_N			= 36,
	/* S3_RESET_PBS_WATCHDOG_TO */
	PON_POWER_OFF_REASON_S3_RESET_PBS_WATCHDOG_TO		= 37,
	/* S3_RESET_PBS_NACK */
	PON_POWER_OFF_REASON_S3_RESET_PBS_NACK			= 38,
	/* power key and/or reset line */
	PON_POWER_OFF_REASON_S3_RESET_KPDPWR_ANDOR_RESIN	= 39,
};

#ifdef CONFIG_INPUT_QPNP_POWER_ON
int qpnp_pon_system_pwr_off(enum pon_power_off_type type);
int qpnp_pon_is_warm_reset(void);
int qpnp_pon_trigger_config(enum pon_trigger_source pon_src, bool enable);
int qpnp_pon_wd_config(bool enable);
int qpnp_pon_set_restart_reason(enum pon_restart_reason reason);
bool qpnp_pon_check_hard_reset_stored(void);
int qpnp_pon_power_off_reason(void);

#else
static int qpnp_pon_system_pwr_off(enum pon_power_off_type type)
{
	return -ENODEV;
}
static inline int qpnp_pon_is_warm_reset(void) { return -ENODEV; }
static inline int qpnp_pon_trigger_config(enum pon_trigger_source pon_src,
							bool enable)
{
	return -ENODEV;
}
int qpnp_pon_wd_config(bool enable)
{
	return -ENODEV;
}
static inline int qpnp_pon_set_restart_reason(enum pon_restart_reason reason)
{
	return -ENODEV;
}
static inline bool qpnp_pon_check_hard_reset_stored(void)
{
	return false;
}
static inline int qpnp_pon_power_off_reason(void)
{
	return -ENODEV;
}
#endif

#endif
