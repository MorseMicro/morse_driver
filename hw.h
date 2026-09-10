#ifndef _MORSE_HW_H_
#define _MORSE_HW_H_

/*
 * Copyright 2017-2022 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/gpio.h>

#include "chip_if.h"
#include "morse_commands.h"
#include "led.h"

/* To be moved to sdio.c */
#define MORSE_REG_ADDRESS_BASE		0x10000
#define MORSE_REG_ADDRESS_WINDOW_0	MORSE_REG_ADDRESS_BASE
#define MORSE_REG_ADDRESS_WINDOW_1	(MORSE_REG_ADDRESS_BASE + 1)
#define MORSE_REG_ADDRESS_CONFIG	(MORSE_REG_ADDRESS_BASE + 2)

#define MORSE_SDIO_RW_ADDR_BOUNDARY_MASK ((u32)0xFFFF0000)

#define MORSE_CONFIG_ACCESS_1BYTE	0
#define MORSE_CONFIG_ACCESS_2BYTE	1
#define MORSE_CONFIG_ACCESS_4BYTE	2

/* Generates IRQ to RISC */
#define MORSE_REG_TRGR_BASE(mors)	((mors)->cfg->regs->trgr_base_address)
#define MORSE_REG_TRGR1_STS(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x00)
#define MORSE_REG_TRGR1_SET(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x04)
#define MORSE_REG_TRGR1_CLR(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x08)
#define MORSE_REG_TRGR1_EN(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x0C)
#define MORSE_REG_TRGR2_STS(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x10)
#define MORSE_REG_TRGR2_SET(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x14)
#define MORSE_REG_TRGR2_CLR(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x18)
#define MORSE_REG_TRGR2_EN(mors)	(MORSE_REG_TRGR_BASE(mors) + 0x1C)

#define MORSE_REG_INT_BASE(mors)	((mors)->cfg->regs->irq_base_address)
#define MORSE_REG_INT1_STS(mors)	(MORSE_REG_INT_BASE(mors) + 0x00)
#define MORSE_REG_INT1_SET(mors)	(MORSE_REG_INT_BASE(mors) + 0x04)
#define MORSE_REG_INT1_CLR(mors)	(MORSE_REG_INT_BASE(mors) + 0x08)
#define MORSE_REG_INT1_EN(mors)		(MORSE_REG_INT_BASE(mors) + 0x0C)
#define MORSE_REG_INT2_STS(mors)	(MORSE_REG_INT_BASE(mors) + 0x10)
#define MORSE_REG_INT2_SET(mors)	(MORSE_REG_INT_BASE(mors) + 0x14)
#define MORSE_REG_INT2_CLR(mors)	(MORSE_REG_INT_BASE(mors) + 0x18)
#define MORSE_REG_INT2_EN(mors)		(MORSE_REG_INT_BASE(mors) + 0x1C)

#define MORSE_REG_MEM_ACCESS_BASE(mors)		((mors)->cfg->regs->mem_access_req_base_addr)
#define MORSE_REG_MEM_ACCESS_ADDR(mors)		(MORSE_REG_MEM_ACCESS_BASE(mors) + 0x00)
#define MORSE_REG_MEM_ACCESS_SIZE(mors)		(MORSE_REG_MEM_ACCESS_BASE(mors) + 0x0C)
#define MORSE_REG_MEM_ACCESS_STATUS(mors)	(MORSE_REG_MEM_ACCESS_BASE(mors) + 0x18)
#define MORSE_REG_MEM_ACCESS_STATE(mors)	(MORSE_REG_MEM_ACCESS_BASE(mors) + 0x24)

#define MORSE_REG_CHIP_ID(mors)	((mors)->cfg->chip_id_address)
#define MORSE_REG_OTP_DATA_WORD(mors, word) \
		((mors)->cfg->regs->otp_data_base_address + (4 * (word)))

#define MORSE_REG_MSI(mors)		((mors)->cfg->regs->msi_address)
#define MORSE_REG_MSI_HOST_INT(mors)	((mors)->cfg->regs->msi_value)

#define MORSE_REG_HOST_MAGIC_VALUE(mors)	\
					((mors)->cfg->regs->magic_num_value)

#define MORSE_REG_RESET(mors)		((mors)->cfg->regs->cpu_reset_address)
#define MORSE_REG_RESET_VALUE(mors)	((mors)->cfg->regs->cpu_reset_value)

#define MORSE_REG_HOST_MANIFEST_PTR(mors)	\
				((mors)->cfg->regs->manifest_ptr_address)

#define MORSE_REG_EARLY_CLK_CTRL_VALUE(morse)	\
				((mors)->cfg->regs->early_clk_ctrl_value)

#define MORSE_REG_CLK_CTRL(mors)	((mors)->cfg->regs->clk_ctrl_address)
#define MORSE_REG_CLK_CTRL_VALUE(mors)	((mors)->cfg->regs->clk_ctrl_value)

#define MORSE_REG_BOOT_ADDR(mors)	((mors)->cfg->regs->boot_address)
#define MORSE_REG_BOOT_ADDR_VALUE(mors)	\
					((mors)->cfg->regs->boot_value)

#define MORSE_REG_AON_ADDR(mors)	((mors)->cfg->regs->aon)
#define MORSE_REG_AON_COUNT(mors)	((mors)->cfg->regs->aon_count)
#define MORSE_REG_AON_LATCH_ADDR(mors)	\
					((mors)->cfg->regs->aon_latch)
#define MORSE_REG_AON_LATCH_MASK(mors)	\
					((mors)->cfg->regs->aon_latch_mask)
#define MORSE_REG_AON_USB_RESET(mors) \
					((mors)->cfg->regs->aon_reset_usb_value)
#define MORSE_REG_MTIME_UPPER(mors)	((mors)->cfg->regs->mtime_upper)
#define MORSE_REG_MTIME_LOWER(mors)	((mors)->cfg->regs->mtime_lower)

#define MORSE_PAGER_BYPASS_TX_STATUS_IRQ_NUM		(15)
#define MORSE_PAGER_IRQ_BYPASS_TX_STATUS_AVAILABLE	BIT(MORSE_PAGER_BYPASS_TX_STATUS_IRQ_NUM)

/** Bit 17 to 24 reserved for the beacon VIF 0 to 7 interrupts */
#define MORSE_INT_BEACON_VIF_MASK_ALL		(GENMASK(24, 17))
#define MORSE_INT_BEACON_BASE_NUM			(17)

/** BIT 25 and 26 are unused */

/** Bit 27 Chip to Host stop notify */
#define MORSE_INT_HW_STOP_NOTIFICATION_NUM	(27)
#define MORSE_INT_HW_STOP_NOTIFICATION		BIT(MORSE_INT_HW_STOP_NOTIFICATION_NUM)

/** Bit 28 Chip to Host attach done notify */
#define MORSE_HW_HEADLESS_DONE_NUM	(28)
#define MORSE_HW_HEADLESS_DONE		BIT(MORSE_HW_HEADLESS_DONE_NUM)

#define MORSE_PAGER_BYPASS_CMD_RESP_IRQ_NUM		(29)
#define MORSE_PAGER_IRQ_BYPASS_CMD_RESP_AVAILABLE	BIT(MORSE_PAGER_BYPASS_CMD_RESP_IRQ_NUM)

/** Bits 0-13 and bypass bits are all observed by the chip interface */
#define MORSE_CHIP_IF_IRQ_MASK_ALL (GENMASK(13, 0) | \
				    MORSE_PAGER_IRQ_BYPASS_TX_STATUS_AVAILABLE | \
				    MORSE_PAGER_IRQ_BYPASS_CMD_RESP_AVAILABLE)

#ifdef CONFIG_MORSE_ENABLE_TEST_MODES
/** Bit 30 for bus irq self test ('Chip' to host, in reality it is host to host) */
#define MORSE_INT_BUS_IRQ_SELF_TEST_NUM		(30)
#define MORSE_INT_BUS_IRQ_SELF_TEST		BIT(MORSE_INT_BUS_IRQ_SELF_TEST_NUM)
#endif

/** Host to Chip memory access reuqest interrupt */
#define MORSE_MEM_ACCESS_TRGR_SET(_m)		MORSE_REG_TRGR2_SET(_m)
#define MORSE_MEM_ACCESS_IRQ_BIT		BIT(0)

/** Bit 28 and 29 Host to Chip detach and attach interrupts */
#define MORSE_HW_DETACH_TRGR_SET(_m)		MORSE_REG_TRGR1_SET(_m)
#define MORSE_HW_DETACH_IRQ_BIT			BIT(28)
#define MORSE_HW_ATTACH_TRGR_SET(_m)		MORSE_REG_TRGR1_SET(_m)
#define MORSE_HW_ATTACH_IRQ_BIT			BIT(29)

/* OTP Bootrom XTAL wait bits[89:86] for MM610x */
#define MM610X_OTP_DATA2_XTAL_WAIT_POS	GENMASK(25, 22)

/* OTP Supplemental Chip ID */
#define MM610X_OTP_DATA2_SUPPLEMENTAL_CHIP_ID GENMASK(23, 16)

/* OTP 8MHz support bit[48] for MM610x */
#define MM610X_OTP_DATA1_8MHZ_SUPPORT	BIT(18)

#define CHIP_TYPE_SILICON	0x0
#define CHIP_TYPE_FPGA		0x1

#define MM6108XX_ID 0x6

/* Chip ID for MM6108 */
#define MM6108A0_ID MORSE_DEVICE_ID(MM6108XX_ID, 2, CHIP_TYPE_SILICON)
#define MM6108A1_ID MORSE_DEVICE_ID(MM6108XX_ID, 3, CHIP_TYPE_SILICON)
#define MM6108A2_ID MORSE_DEVICE_ID(MM6108XX_ID, 4, CHIP_TYPE_SILICON)

/* Chip Id */
#define MM8108XX_ID 0x9

/* Chip Rev */
#define MM8108B0_REV 0x6
#define MM8108B1_REV 0x7
#define MM8108B2_REV 0x8
#define MM8108B3_REV 0x9

/* Chip Rev String */
#define MM8108B_STRING "b"
#define MM8108B0_REV_STRING MM8108B_STRING "0"
#define MM8108B1_REV_STRING MM8108B_STRING "1"
#define MM8108B2_REV_STRING MM8108B_STRING "2"
#define MM8108B3_REV_STRING MM8108B_STRING "3"

/* Chip ID for MM8108 */
#define MM8108B0_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B0_REV, CHIP_TYPE_SILICON)
#define MM8108B1_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B1_REV, CHIP_TYPE_SILICON)
#define MM8108B2_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B2_REV, CHIP_TYPE_SILICON)
#define MM8108B3_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B3_REV, CHIP_TYPE_SILICON)

/* Chip ID for MM8108 - FPGA */
#define MM8108B0_FPGA_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B0_REV, CHIP_TYPE_FPGA)
#define MM8108B1_FPGA_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B1_REV, CHIP_TYPE_FPGA)
#define MM8108B2_FPGA_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B2_REV, CHIP_TYPE_FPGA)
#define MM8108B3_FPGA_ID MORSE_DEVICE_ID(MM8108XX_ID, MM8108B3_REV, CHIP_TYPE_FPGA)


#define MEM_ACCESS_FIXED_REGION_COUNT	2

#define FW_RAM_ONLY_STRING ""
#define FW_ROM_LINKED_STRING "-rl"
#define FW_ROM_ALL_STRING "-ro"

enum morse_coredump_method;

enum host_table_firmware_flags {
	/** Firmware supports S1G */
	MORSE_FW_FLAGS_SUPPORT_S1G = BIT(0),
	/** BUSY GPIO pin is active low */
	MORSE_FW_FLAGS_BUSY_ACTIVE_LOW = BIT(1),
	/** Firmware reports beacon tx completion status to host */
	MORSE_FW_FLAGS_REPORTS_TX_BEACON_COMPLETION = BIT(2),
	/** FW has HW scan support */
	MORSE_FW_FLAGS_SUPPORT_HW_SCAN = BIT(3),
	/** Supports hostsync chip halting */
	MORSE_FW_FLAGS_SUPPORT_CHIP_HALT_IRQ = BIT(4),
	/** FW will toggle busy pin upon wake up from wake pin */
	MORSE_FW_FLAGS_TOGGLES_BUSY_PIN_ON_WAKE_PIN = BIT(8),
	/** Supports HW reattach */
	MORSE_FW_FLAGS_SUPPORT_HW_REATTACH = BIT(9),
	/** Firmware can report TX status for any frame (fullmac only) */
	MORSE_FW_FLAGS_SUPPORT_FULLMAC_REPORT = BIT(10),
	/** BCF was not loaded - firmware is running in failsafe mode */
	MORSE_FW_FLAGS_FAILSAFE_MODE = BIT(11),
};

enum morse_hw_state {
	/** HW is off, or firmware is not loaded. */
	MORSE_HW_STATE_OFF = 0,
	/** HW is on, firmware is loaded and HW is running. */
	MORSE_HW_STATE_ON,
	/** HW has stopped, supress all host interaction with HW. */
	MORSE_HW_STATE_STOPPED,
	/** HW is being restarted after a stop event. */
	MORSE_HW_STATE_RESTARTING,
};

struct host_table {
	__le32 magic_number;
	__le32 fw_version_number;
	__le32 host_flags;
	__le32 firmware_flags;
	__le32 memcmd_cmd_addr;
	__le32 memcmd_resp_addr;
	__le32 extended_host_table_addr;
	struct morse_chip_if_host_table chip_if;
} __packed;

/**
 * struct morse_hw_memory - On chip memory address space
 *
 * Identifies memory space on chip.
 * Used to optimise chip access
 *
 * @start: Start address of memory space
 * @end: End address of memory space
 */
struct morse_hw_memory {
	u32 start;
	u32 end;
};

struct mem_access_cfg {
	/**
	 * @expand_cache_access: Region for which access through cache can be expanded
	 */
	struct morse_hw_memory expand_cache_access;

	/**
	 * @fixed_regions: Fixed memory regions to which bus access is always unrestricted
	 */
	struct morse_hw_memory fixed_regions[MEM_ACCESS_FIXED_REGION_COUNT];
};

struct morse_hw_regs {
	u32 irq_base_address;
	u32 trgr_base_address;
	u32 mem_access_req_base_addr;
	u32 cpu_reset_address;
	u32 cpu_reset_value;
	u32 msi_address;
	u32 msi_value;
	u32 manifest_ptr_address;
	u32 host_table_address;
	u32 magic_num_value;
	u32 clk_ctrl_address;
	u32 clk_ctrl_value;
	u32 early_clk_ctrl_value;
	u32 boot_address;
	u32 boot_value;
	u32 otp_data_base_address;
	u32 pager_base_address;
	u32 test_mode_mem_base_address;
	u32 aon_latch;
	u32 aon_latch_mask;
	u32 aon_reset_usb_value;
	u32 aon;
	u8 aon_count;
	u32 mtime_upper;
	u32 mtime_lower;
};

struct morse_hw_clock {
	/* Value returned by the ops->read_clock() operation (microseconds) */
	u64 val;
	/* Kernel system time of when the hw clock was read (nanosecond accuracy) */
	ktime_t reference;
	/* Used for freeing the data on update */
	struct rcu_head rcu;
};

/**
 * struct morse_gpios - GPIO assignments for SDIO/SPI attached devices.
 * @reset: Host-controlled GPIO used to reset the device - required for SPI.
 * @wake: Host-controlled GPIO used to wake the device from low power state.
 * @busy: Device-controlled GPIO indicating the device is busy.
 * @spi_irq: Device-controlled GPIO interrupt - required for SPI.
 */
struct morse_gpios {
	int reset;
	int wake;
	int busy;
	int spi_irq;
};

struct morse_hw_cfg {
	const struct morse_hw_regs *regs;

	/**
	 * @chip_id_address: The address where we can read about the type of chip.
	 * Should not change for a family of chipset.
	 */
	const u32 chip_id_address;

	const struct morse_firmware *fw;
	const struct chip_if_ops *ops;

	/**
	 * Get hardware version
	 *
	 * @chip_id: Registered chip ID when loading the driver
	 */
	const char *(*get_hw_version)(u32 chip_id);

	/**
	 * Get warm boot time depending on chip id
	 *
	 * @chip_id: Registered chip ID when loading the driver
	 */
	 u8 (*get_warm_boot_time_ms)(u32 chip_id);

	/**
	 * Get cold  boot time depending on chip id
	 *
	 * @chip_id: Registered chip ID when loading the driver
	 */
	 u32 (*get_cold_boot_time_ms)(u32 chip_id);

	/**
	 * Get firmware trigger delay/wait time depending on chip id
	 *
	 * @chip_id: Registered chip ID when loading the driver
	 */
	u32 (*get_firmware_trigger_delay_ms)(u32 chip_id);

	/**
	 * Get FW path depending on chip id
	 *
	 * @chip_id: Registered chip ID when loading the driver
	 */
	 char *(*get_fw_path)(u32 chip_id);

	/**
	 * Enable SDIO burst mode
	 *
	 * @mors: Morse context object
	 * @burst_enabled: Bursting enabled or not
	 *
	 * @return error code
	 */
	int (*enable_sdio_burst_mode)(struct morse *mors, bool burst_enabled);

	/**
	 * Get burst mode inter block delay
	 *
	 * @burst_enabled: Bursting enabled or not
	 *
	 * @return Inter block delay in nano seconds
	 */
	u32 (*get_spi_inter_block_delay_ns)(bool burst_enabled);

	/**
	 * Perform necessary actions to prepare the chip before firmware load
	 *
	 * @return error code
	 */
	int (*pre_load_prepare)(struct morse *mors);

	/**
	 * Perform a digital reset
	 *
	 * @return error code
	 */
	int (*digital_reset)(struct morse *mors);

	/**
	 * Return the board type burnt into OTP
	 *
	 * @return the board type if available, else -EINVAL
	 */
	int (*get_board_type)(struct morse *mors);

	/**
	 * Return the region code burnt into OTP
	 *
	 * @return the region code if available, else an error code
	 */
	int (*get_encoded_country)(struct morse *mors);

	/**
	 * Set the slow clock source selection mode
	 *
	 * @return error code
	 */
	int (*set_slow_clock_mode)(struct morse *mors, enum morse_cmd_slow_clock_mode mode);

	/**
	 * Invoke prior to initiating a coredump to prepare the chip
	 *
	 * @return error code
	 */
	int (*pre_coredump_hook)(struct morse *mors, enum morse_coredump_method method);

	/**
	 * Invoke after creating coredump to restore chip settings.
	 *
	 * @return error code
	 */
	int (*post_coredump_hook)(struct morse *mors, enum morse_coredump_method method);

	/**
	 * Invoke prior to performing non-destructive-reset of the chip.
	 *
	 */
	void (*pre_firmware_ndr)(struct morse *mors);

	/**
	 * Invoke after performing non-destructive-reset of the chip.
	 *
	 */
	void (*post_firmware_ndr)(struct morse *mors);

	/**
	 * enable_ext_xtal_delay: Enable external XTAL wait delays during bus transfers.
	 *
	 * @param mors: Morse context object
	 * @param enable: true if delay is required
	 */
	void (*enable_ext_xtal_delay)(struct morse *mors, bool enable);

	/**
	 * Initialise gpio for output
	 *
	 * @return error code
	 */
	int (*gpio_enable_output)(struct morse *mors, int pin_num, bool enable);

	/**
	 * Set or clear gpio output
	 */
	void (*gpio_write_output)(struct morse *mors, int pin_num, bool value);

	/**
	 * Set Security Manifest Pointer address
	 *
	 *
	 * Write the chip memory address where the security manifest has been loaded
	 * into the Security Manifest Pointer register, so that the boot loader can
	 * find it and verify the firmware.
	 *
	 * @mors Morse context object
	 * @addr Load address of the security manifest
	 *
	 * @return error code
	 */
	int (*set_security_manifest_ptr)(struct morse *mors, uint32_t addr);

	/**
	 * Retrieve the secure boot status.
	 *
	 * Reads the secure boot status register and stores the value
	 * in the provided status pointer.
	 *
	 * @mors	Morse context object.
	 * @status	Variable where the secure boot status will be stored.
	 *
	 * @return	0 on success, -EOPNOTSUPP if the chip does not support secure boot,
	 * or an error code if the register read failed.
	 */
	int (*get_secureboot_status)(struct morse *mors, uint32_t *status);

	/**
	 * @led_group: Contains information pertaining to gpio-attached LEDs
	 */
	struct morse_led_group led_group;

	/**
	 * @bus_double_read: Decide if the bus workaround is required to recover
	 * the page header repeated words
	 */
	bool bus_double_read;

	/**
	 * @mem_access_cfg: Restricted mem access config
	 */
	const struct mem_access_cfg *mem_access_cfg;

	/**
	 * @xtal_init_bus_trans_delay_ms : An additional delay incurred if the device
	 * requires external (host) xtal initialisation. Once the XTAL is initialised,
	 * this variable will get cleared to zero (no delay).
	 */
	unsigned int xtal_init_bus_trans_delay_ms;

	/**
	 * @enable_short_bcn_as_dtim : Indicate if DTIM beacon should be a long beacon.
	 */
	bool enable_short_bcn_as_dtim;

	/**
	 * @enable_amsdu_support: Indicate if A-MSDU to be enabled when airtime fairness is enabled
	 */
	bool enable_amsdu_support;

	/** Host GPIO config */
	struct morse_gpios gpios;

	u32 board_type_max_value;
	u32 fw_count;
	u32 host_table_ptr;
};

struct morse_chip_series {
	/**
	 * @cfg: The hardware config for the chip
	 */
	struct morse_hw_cfg *cfg;

	/**
	 * Check whether the given chip ID matches a chip from this series
	 *
	 * @chip_id: Chip id to be matched
	 *
	 * Return: True if the chip ID matches.
	 */
	bool (*chip_id_matches)(u32 chip_id);
};

int morse_hw_irq_enable(struct morse *mors, u32 irq, bool enable);
int morse_hw_irq_handle(struct morse *mors);
int morse_hw_irq_clear(struct morse *mors);
int morse_hw_toggle_aon_latch(struct morse *mors);

enum sdio_burst_mode {
	SDIO_WORD_BURST_DISABLE = 0,	/* Intentionally duplicate to make it clear it's disabled */
	SDIO_WORD_BURST_SIZE_0 = 0,	/* 000: no bursting (single 32bit word) */
	SDIO_WORD_BURST_SIZE_2 = 1,	/* 001: bursts of 2 words */
	SDIO_WORD_BURST_SIZE_4 = 2,	/* 010: bursts of 4 words */
	SDIO_WORD_BURST_SIZE_8 = 3,	/* 011: bursts of 8 words */
	SDIO_WORD_BURST_SIZE_16 = 4,	/* 100: bursts of 16 words */
	SDIO_WORD_BURST_MASK = 7,
};

/* MM6108 */
extern struct morse_chip_series mm61xx_chip_series;
extern struct morse_hw_cfg mm6108_cfg;
/* MM8108 */
extern struct morse_chip_series mm81xx_chip_series;
extern struct morse_hw_cfg mm8108_cfg;

/**
 * morse_hw_reset - performs a hardware reset on the chip by toggling
 *  the reset gpio.
 * @reset_pin: RPi GPIO pin number connected to chip reset pin
 *
 * Return: int return code.
 */
int morse_hw_reset(int reset_pin);

/**
 * is_otp_xtal_wait_supported - checks the xtal wait bit.
 * @mors: morse struct containing the config
 *
 * Return: true if the check is successful or
 *  false if any step fails
 */
bool is_otp_xtal_wait_supported(struct morse *mors);

/**
 * morse_hw_enable_stop_notifications - Enable/Disable chip->host notification on stop.
 *
 * @mors: morse config struct
 * @enable: true to enable
 *
 * Return: int return code.
 */
int morse_hw_enable_stop_notifications(struct morse *mors, bool enable);

/**
 * morse_hw_stop_work - Work to handle a hardware stop.
 *
 * @work: Work item
 */
void morse_hw_stop_work(struct work_struct *work);

/**
 * morse_chip_cfg_set_and_validate - Set the config and validate
 * the chip id
 *
 * @mors: morse struct containing bus_ops and pointers to regs and cfgs
 * @mors_chip_series: Chip family struct containing config to set
 *
 * Return: int error code. Returns 0 if successful.
 */
int morse_chip_cfg_set_and_validate(struct morse *mors, struct morse_chip_series *mors_chip_series);

/**
 * morse_chip_cfg_init - Assign the chip-specific cfg and regs based on chip_id.
 *
 * @mors: morse struct containing bus_ops and pointers to regs and cfgs
 * @chip_id: Chip ID
 *
 * Return: int error code. Returns 0 if successful.
 */
int morse_chip_cfg_init(struct morse *mors, u32 chip_id);

/**
 * morse_hw_clock_update - Read internal HW clock and capture local kernel time as a reference.
 *                         Note; Should be called by chip_if work only.
 *
 * @mors: morse struct
 *
 * @return 0 if success else error code
 */
int morse_hw_clock_update(struct morse *mors);

/**
 * morse_hw_clock_trigger_update - Trigger a update/read operation on the internal HW clock.
 *
 * @mors: morse struct
 * @wait: true if operation should wait until completion
 *
 * @return 0 if success else error code
 */
int morse_hw_clock_trigger_update(struct morse *mors, bool wait);

/**
 * morse_hw_clock_now - Current interpolated value of the HW's clock (microseconds).
 *                      Note; Requires the HW clock to have been read at least once.
 *
 * @mors: mors struct
 * @now: Field in which to store the current (interpolated) value of the HW clock.
 *
 * @return 0 if success else error code
 */
int morse_hw_clock_now(const struct morse *mors, u64 *now);


/**
 * morse_hw_is_already_loaded - Check if the hardware is already loaded
 *
 * @mors: morse struct
 *
 * @return true if hardware is already loaded else false
 */
bool morse_hw_is_already_loaded(struct morse *mors);

/**
 * morse_hw_headless_done_irq_enable - Enable or disable 'HW headless done' interrupt
 *
 * @mors: morse struct
 * @enable: enable or disable the interrupt
 *
 * @return 0 on success, else an error code
 */
int morse_hw_headless_done_irq_enable(struct morse *mors, bool enable);

/**
 * morse_hw_headless_reset - Reset software tracking of headless mode (i.e. due to HW reset)
 *
 * @mors: morse struct
 */
void morse_hw_headless_reset(struct morse *mors);

/**
 * morse_hw_attach - Attach to running hardware
 *
 * @mors: morse struct
 * @cfg: headless configuration (see morse_cmd_headless_cfg_option)
 *
 * @return 0 on success, else an error code
 */
int morse_hw_attach(struct morse *mors, int cfg);

/**
 * morse_hw_detach - Detach from hardware
 *
 * @mors: morse struct
 * @cfg: headless configuration (see morse_cmd_headless_cfg_option)
 *
 * @return 0 on success, else an error code
 */
int morse_hw_detach(struct morse *mors, int cfg);

/**
 * morse_hw_should_reattach - Check if reattaching to hardware
 *
 * @return true if reattaching to hardware else false
 */
bool morse_hw_should_reattach(void);

/**
 * morse_hw_can_detach - Check if host can detach from hardware
 *
 * @mors: morse struct
 *
 * @return true if detaching is allowed
 */
bool morse_hw_can_detach(const struct morse *mors);

/**
 * morse_hw_headless_init - Initialise headless data structure in mors struct
 *
 * @mors: morse struct
 */
void morse_hw_headless_init(struct morse *mors);

/**
 * morse_hw_trigger_detach - Send detach interrupt to hardware
 *
 * @mors: morse struct
 *
 * @return 0 on success, else an error code
 */
int morse_hw_trigger_detach(struct morse *mors);

/**
 * morse_hw_headless_work - Work function to handle headless ON/OFF requests
 *
 * @work: work struct
 */
void morse_hw_headless_work(struct work_struct *work);

/**
 * morse_hw_headless_is_off - Is headless mode off
 *
 * @mors: morse struct
 *
 * @return true when headless is off
 */
bool morse_hw_headless_is_off(struct morse *mors);

/**
 * morse_hw_set_state - Set HW state
 *
 * @mors: morse struct
 * @next: state to set
 */
void morse_hw_set_state(struct morse *mors, enum morse_hw_state next);

/**
 * morse_hw_is_restarting - Is morse HW being restarted
 *
 * @mors: morse struct
 *
 * @return true if restarting
 */
bool morse_hw_is_restarting(const struct morse *mors);

/**
 * morse_hw_restart() - Request a HW restart.
 * @mors: morse struct
 */
void morse_hw_restart(struct morse *mors);

/**
 * morse_hw_is_stopped - Is morse HW stopped
 *
 * @morse: morse struct
 *
 * @return true if stopped
 */
bool morse_hw_is_stopped(const struct morse *mors);

/**
 * morse_hw_is_stopped_notification_set - Check if the hardware stop notification is set
 *
 * @morse: morse struct
 *
 * @return true if hardware indicates stop
 */
bool morse_hw_is_stopped_notification_set(struct morse *mors);

/**
 * morse_hw_is_on - Is morse HW on and running
 *
 * @morse: morse struct
 *
 * @return true if on
 */
bool morse_hw_is_on(const struct morse *mors);

/**
 * morse_hw_ps_gpios_are_supported - Indicate if a hardware config supports power save through
 * hardware GPIOs
 * @gpios: Morse hardware GPIO structure
 */
static inline bool morse_hw_ps_gpios_are_supported(struct morse_gpios *gpios)
{
	return gpio_is_valid(gpios->wake) && gpio_is_valid(gpios->busy);
}

#endif /* !_MORSE_HW_H_ */
