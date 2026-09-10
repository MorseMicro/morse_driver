/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <net/mac80211.h>
#include <linux/elf.h>
#include <linux/crc32.h>
#include <linux/completion.h>
#include <linux/types.h>

#include "morse.h"
#include "morse_commands.h"
#include "bus.h"
#include "debug.h"
#include "firmware.h"
#include "mac.h"
#include "vendor.h"
#include "coredump.h"
#include "mem_access.h"

#define MAX_NUM_REGDOMS		 32
#define MAX_FW_BIN_FILE_NAME_LEN 30

struct fw_init_params {
	bool download_fw;
	bool get_host_table_ptr;
	bool verify_fw;
};

/* The last mac address octet value */
u8 macaddr_octet = 0xFF;
module_param(macaddr_octet, byte, 0644);
MODULE_PARM_DESC(macaddr_octet,
		"MAC address octet 6 (0xFF for a random value) - ignored if hardware MAC address present");

/* When setting the mac address, the special value 00 will randomise the last 3 octets */
#define MORSE_RANDOMISE_OCTETS "00:00:00"

/* The last 3 mac address octet values */
static char macaddr_suffix[9] = "00:00:00";
module_param_string(macaddr_suffix, macaddr_suffix, ARRAY_SIZE(macaddr_suffix), 0644);
MODULE_PARM_DESC(macaddr_suffix,
	"MAC address octets 4, 5, and 6 (the default 00:00:00 randomises the value) - ignored if hardware MAC address present");

uint sdio_reset_time = CONFIG_MORSE_SDIO_RESET_TIME;
module_param(sdio_reset_time, uint, 0644);
MODULE_PARM_DESC(sdio_reset_time, "Time to wait (in ms) after SDIO reset");

static char fw_bin_file[MAX_FW_BIN_FILE_NAME_LEN];
module_param_string(fw_bin_file, fw_bin_file, sizeof(fw_bin_file), 0644);
MODULE_PARM_DESC(fw_bin_file, "Firmware binary filename to load");



static int get_file_header(const u8 *data, morse_elf_ehdr *ehdr)
{
	morse_elf_ehdr *p = (morse_elf_ehdr *)data;

	/* Magic check */
	if (p->e_ident[EI_MAG0] != ELFMAG0 ||
	    p->e_ident[EI_MAG1] != ELFMAG1 ||
	    p->e_ident[EI_MAG2] != ELFMAG2 ||
	    p->e_ident[EI_MAG3] != ELFMAG3)
		return -1;

	/* elf32 and little endian */
	if (p->e_ident[EI_DATA] != ELFDATA2LSB || p->e_ident[EI_CLASS] != ELFCLASS32)
		return -1;

	ehdr->e_phoff = le32_to_cpu((__force __le32)p->e_phoff);
	ehdr->e_phentsize = le16_to_cpu((__force __le16)p->e_phentsize);
	ehdr->e_phnum = le16_to_cpu((__force __le16)p->e_phnum);
	ehdr->e_shoff = le32_to_cpu((__force __le32)p->e_shoff);
	ehdr->e_shentsize = le16_to_cpu((__force __le16)p->e_shentsize);
	ehdr->e_shnum = le16_to_cpu((__force __le16)p->e_shnum);
	ehdr->e_shstrndx = le16_to_cpu((__force __le16)p->e_shstrndx);
	ehdr->e_entry = le32_to_cpu((__force __le32)p->e_entry);

	return 0;
}

static void parse_firmware_info(struct morse *mors, const u8 *data, int length)
{
	const struct morse_fw_info_tlv *tlv = (const struct morse_fw_info_tlv *)data;
	const u8 *end = data + length;

	/* Reset the coredump memory descriptor information */
	morse_coredump_remove_memory_regions(mors);
	/* Fallback for firmware metadata that does not provide BCF size. */
	mors->bcf.size = BCF_DEFAULT_SIZE;
	mors->bcf.address = 0;

	while (((const u8 *)tlv + sizeof(*tlv)) <= end) {
		u16 tlv_len = le16_to_cpu(tlv->length);

		if (((const u8 *)tlv + sizeof(*tlv) + tlv_len) > end) {
			MORSE_ERR(mors, "Malformed .fw_info TLV (type:%u, len:%u)\n",
				  le16_to_cpu(tlv->type), tlv_len);
			break;
		}

		switch (le16_to_cpu(tlv->type)) {
		case MORSE_FW_INFO_TLV_BCF_ADDR:
			if (tlv_len != sizeof(__le32)) {
				MORSE_ERR(mors, "Invalid BCF metadata length %u\n", tlv_len);
				break;
			}

			mors->bcf.address = le32_to_cpu(get_unaligned((__force __le32 *)tlv->val));
			break;
		case MORSE_FW_INFO_TLV_BCF_SIZE:
			if (tlv_len != sizeof(__le32)) {
				MORSE_ERR(mors, "Invalid BCF size TLV length %u\n", tlv_len);
				break;
			}

			mors->bcf.size = le32_to_cpu(get_unaligned((__force __le32 *)tlv->val));
			if (!mors->bcf.size) {
				MORSE_ERR(mors, "Invalid BCF metadata size 0x%x\n",
					  mors->bcf.size);
				mors->bcf.size = BCF_DEFAULT_SIZE;
				break;
			}

			break;
		case MORSE_FW_INFO_TLV_COREDUMP_MEM_REGION:
			morse_coredump_add_memory_region(mors,
					(struct morse_fw_info_tlv_coredump_mem *)tlv->val);
			break;
		default:
			/* Just skip unknown types */
			break;
		}
		tlv = (const struct morse_fw_info_tlv *)((u8 *)tlv + tlv_len + sizeof(*tlv));
	}
}

/**
 * @brief Fill a section header from the buffered ELF
 *
 * @param data Buffer where the ELF resides
 * @param ehdr Header of the ELF File
 * @param shdr Header of the target section
 * @param i Index of the section header table index
 * @return int
 */
static int get_section_header(const u8 *data, morse_elf_ehdr *ehdr, morse_elf_shdr *shdr, int i)
{
	morse_elf_shdr *p;

	if (i < 0 || i >= ehdr->e_shnum)
		return -ENOENT;

	p = (morse_elf_shdr *)(data + ehdr->e_shoff + (i * ehdr->e_shentsize));

	shdr->sh_name = le32_to_cpu((__force __le32)p->sh_name);
	shdr->sh_type = le32_to_cpu((__force __le32)p->sh_type);
	shdr->sh_offset = le32_to_cpu((__force __le32)p->sh_offset);
	shdr->sh_addr = le32_to_cpu((__force __le32)p->sh_addr);
	shdr->sh_size = le32_to_cpu((__force __le32)p->sh_size);
	shdr->sh_flags = le32_to_cpu((__force __le32)p->sh_flags);

	return 0;
}

/**
 * morse_set_boot_addr() - Initialize boot address resgister with the passed value.
 *
 * @mors: Global driver context.
 * @addr: Boot address to which the core jumps after loading firware.
 *
 * Return: Returns the status of reg write operation.
 *
 **/
static int morse_set_boot_addr(struct morse *mors, uint32_t addr)
{
	int status = 0;

	MORSE_INFO(mors, "Overwriting boot address to 0x%x\n", addr);
	morse_claim_bus(mors);

	if (mem_access_is_restriction_enforced(mors)) {
		MORSE_DBG(mors,
			  "Memory access restriction is enabled. %s not permitted\n",
			  __func__);
		goto exit;
	}

	status = morse_reg32_write(mors, MORSE_REG_BOOT_ADDR(mors), addr);

exit:
	morse_release_bus(mors);
	return status;
}

static int extract_firmware_info(struct morse *mors, const struct firmware *fw)
{
	int i;
	int ret = 0;
	morse_elf_ehdr ehdr;
	morse_elf_shdr shdr;
	morse_elf_shdr sh_strtab;
	const char *sh_strs;

	if (get_file_header(fw->data, &ehdr) != 0) {
		MORSE_ERR(mors, "Wrong file format\n");
		return -1;
	}

	if (get_section_header(fw->data, &ehdr, &sh_strtab, ehdr.e_shstrndx) != 0) {
		MORSE_ERR(mors, "Invalid firmware. Missing string table\n");
		return -1;
	}

	sh_strs = (const char *)fw->data + sh_strtab.sh_offset;

	for (i = 0; i < ehdr.e_shnum; i++) {
		if (get_section_header(fw->data, &ehdr, &shdr, i) != 0)
			continue;

		/* This is the firmware info. Parse it */
		if (strncmp(sh_strs + shdr.sh_name, ".fw_info", sizeof(".fw_info")) == 0) {
			parse_firmware_info(mors, fw->data + shdr.sh_offset, shdr.sh_size);
			break;
		}
	}

	return ret;
}

static int morse_firmware_load(struct morse *mors, const struct firmware *fw)
{
	int i;
	int ret = 0;
	morse_elf_ehdr ehdr;
	morse_elf_phdr phdr;
	morse_elf_shdr shdr;
	morse_elf_shdr sh_strtab;
	const char *sh_strs;
	static const char sec_manifest_id[] = ".security_manifest";

	u8 *fw_buf = devm_kmalloc(mors->dev, ROUND_BYTES_TO_WORD(fw->size), GFP_KERNEL);

	if (!fw_buf)
		return -ENOMEM;

	if (get_file_header(fw->data, &ehdr) != 0) {
		MORSE_ERR(mors, "Wrong file format\n");
		return -1;
	}

	if (get_section_header(fw->data, &ehdr, &sh_strtab, ehdr.e_shstrndx) != 0) {
		MORSE_ERR(mors, "Invalid firmware. Missing string table\n");
		return -1;
	}

	sh_strs = (const char *)fw->data + sh_strtab.sh_offset;

	for (i = 0; i < ehdr.e_phnum; i++) {
		int status;
		int address;

		morse_elf_phdr *p =
			(morse_elf_phdr *)(fw->data + ehdr.e_phoff + i * ehdr.e_phentsize);

		phdr.p_type = le32_to_cpu((__force __le32)p->p_type);
		phdr.p_offset = le32_to_cpu((__force __le32)p->p_offset);
		phdr.p_paddr = le32_to_cpu((__force __le32)p->p_paddr);
		phdr.p_filesz = le32_to_cpu((__force __le32)p->p_filesz);
		phdr.p_memsz = le32_to_cpu((__force __le32)p->p_memsz);

		/* In current design, the iflash/dflash are only used in self-hosted mode. For
		 * hosted mode, if the sections are found in the combined image, driver
		 * needs to skip them.
		 */
		address = phdr.p_paddr;
		if (address == IFLASH_BASE_ADDR || address == DFLASH_BASE_ADDR)
			continue;

		if (phdr.p_type != PT_LOAD || !phdr.p_memsz)
			continue;

		if (phdr.p_filesz && phdr.p_offset &&
					(phdr.p_offset + phdr.p_filesz) < fw->size) {
			u32 padded_size = ROUND_BYTES_TO_WORD(phdr.p_filesz);

			memcpy(fw_buf, fw->data + phdr.p_offset, padded_size);
			/* Set padding to 0xff */
			memset(fw_buf + phdr.p_filesz, 0xff, padded_size - phdr.p_filesz);
			morse_claim_bus(mors);
			status = morse_dm_write(mors, address, fw_buf, padded_size);
			morse_release_bus(mors);
			if (status) {
				ret = -1;
				break;
			}
		}
	}

	for (i = 0; i < ehdr.e_shnum; i++) {
		if (get_section_header(fw->data, &ehdr, &shdr, i) != 0)
			continue;

		if (strncmp(sh_strs + shdr.sh_name, sec_manifest_id,
				sizeof(sec_manifest_id)) == 0) {
			if (mors->cfg->set_security_manifest_ptr)
				mors->cfg->set_security_manifest_ptr(mors, shdr.sh_addr);
		}
	}

	{
		if (ehdr.e_entry != 0)
			ret = morse_set_boot_addr(mors, ehdr.e_entry);
	}
	devm_kfree(mors->dev, fw_buf);
	return ret;
}

static int add_regdom_to_buffer(struct morse *mors,
				char regdom_buff[MAX_NUM_REGDOMS][3],
				const char *cc)
{
	int r;

	if (!mors || !regdom_buff || !cc)
		return -EINVAL;

	if (mors->regdoms.count >= MAX_NUM_REGDOMS)
		return -ENOMEM;

	r = strscpy(regdom_buff[mors->regdoms.count], cc, 3);
	if (r != 2)
		return -EINVAL;

	mors->regdoms.count++;
	return 0;
}

static const char *secure_boot_status_to_str(enum morse_secure_boot_status status)
{
	switch (status) {
	case MORSE_SECURE_BOOT_MANIFEST_ERROR:
		return "invalid security manifest";
	case MORSE_SECURE_BOOT_VERSION_ERROR:
		return "invalid security version number";
	case MORSE_SECURE_BOOT_OTP_ERROR:
		return "OTP access error";
	case MORSE_SECURE_BOOT_DIGEST_MISMATCH:
		return "image digest mismatch";
	case MORSE_SECURE_BOOT_SIGN_VERIFY_ERROR:
		return "signature verification failed";
	case MORSE_SECURE_BOOT_CERT_VERIFY_ERROR:
		return "certificate verification failed";
	case MORSE_SECURE_BOOT_CERT_ERROR:
		return "invalid certificate";
	case MORSE_SECURE_BOOT_PARSE_ERROR:
		return "parsing error";
	case MORSE_SECURE_BOOT_PUBLIC_KEY_ERROR:
		return "public key error";
	case MORSE_SECURE_BOOT_SIGNATURE_ERROR:
		return "invalid signature";
	case MORSE_SECURE_BOOT_SHA_ERROR:
		return "SHA computation error";
	case MORSE_SECURE_BOOT_KEY_REVOKE_ERROR:
		return "key revocation error";
	case MORSE_SECURE_BOOT_KEY_UPDATE_ERROR:
		return "key update error";
	case MORSE_SECURE_BOOT_PROVISION_ERROR:
		return "provisioning error";
	case MORSE_SECURE_BOOT_STATE_ERROR:
		return "invalid secure boot state";
	case MORSE_SECURE_BOOT_MANIFEST_TLV_ERROR:
		return "malformed manifest TLV";
	case MORSE_SECURE_BOOT_PROGRAM_TYPE_ERROR:
		return "invalid program type";
	default:
		return "unknown secure-boot error";
	}
}

static int store_regdom_info(struct morse *mors, char regdom_buff[MAX_NUM_REGDOMS][3])
{
	if (mors->regdoms.count) {
		if (mors->regdoms.list) {
			/* It's possible for this function to be run more than once
			 * if the firmware load is retried
			 */
			devm_kfree(mors->dev, mors->regdoms.list);
		}
		mors->regdoms.list = devm_kmalloc_array(mors->dev,
							mors->regdoms.count,
							sizeof(*mors->regdoms.list),
							GFP_KERNEL);
		if (!mors->regdoms.list)
			return -ENOMEM;

		memcpy(mors->regdoms.list,
		       regdom_buff,
		       mors->regdoms.count * sizeof(*mors->regdoms.list));
	}
	return 0;
}

struct morse_bcf_fallback_entry {
	const char *country;
	const char *fallback_cc;
};

static const struct morse_bcf_fallback_entry morse_bcf_fallback_table[] = {
	{ "MX", "US" },
	{ "AR", "AU" },
	{ "CL", "AU" },
	{ "CO", "AU" },
};

/**
 * Returns a fallback BCF country code if the given countries regdom section
 * is not present in the BCF, or NULL if there is no fallback.
 */
static const char *morse_bcf_country_fallback(const char *country)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(morse_bcf_fallback_table); i++) {
		if (!strncmp(country, morse_bcf_fallback_table[i].country, 2))
			return morse_bcf_fallback_table[i].fallback_cc;
	}
	return NULL;
}

/**
 * @brief Copy a section of the BCF file to a buffer
 *
 * @param bcf BCF file
 * @param dest Destination address to copy section to
 * @param shdr ELF section header
 * @param max_len Maximum length to copy
 *
 * @returns Total length copied in bytes, or negative value on failure
 */
static int copy_bcf_section(const struct firmware *bcf,
			    u8 *dest,
			    morse_elf_shdr shdr,
			    int max_len)
{
	int padded_len = ROUND_BYTES_TO_WORD(shdr.sh_size);

	if (shdr.sh_offset > bcf->size || shdr.sh_size > (bcf->size - shdr.sh_offset))
		return -EINVAL;

	if (padded_len > max_len)
		return -ENOMEM;

	memcpy(dest, bcf->data + shdr.sh_offset, shdr.sh_size);
	memset(dest + shdr.sh_size, 0xff, padded_len - shdr.sh_size);

	return padded_len;
}

static int add_fallback_regdoms_to_buffer(struct morse *mors,
					  char regdom_buff[MAX_NUM_REGDOMS][3],
					  const char *regdom_cc)
{
	int j;
	int ret;

	for (j = 0; j < ARRAY_SIZE(morse_bcf_fallback_table); j++) {
		if (!strncmp(regdom_cc, morse_bcf_fallback_table[j].fallback_cc, 2)) {
			ret = add_regdom_to_buffer(mors, regdom_buff,
						   morse_bcf_fallback_table[j].country);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int morse_bcf_load(struct morse *mors, const struct firmware *bcf,
			  unsigned int bcf_address)
{
	int i = 0;
	int ret = 0;
	morse_elf_ehdr ehdr;
	morse_elf_shdr shdr;
	morse_elf_shdr sh_strtab;
	const char *sh_strs;
	const char *section_name;
	int config_len = -1;
	int regdom_len = -1;
	const char *reg_prefix = ".regdom_";
	u8 *bcf_buf;
	const char *regdom_cc = NULL;
	char regdom_buff[MAX_NUM_REGDOMS][3] = { 0 };
	bool fill_regdom_info_from_bcf = true;
	int bcf_buf_len = mors->bcf.size ? mors->bcf.size : BCF_DEFAULT_SIZE;

	if (bcf_buf_len <= 0) {
		MORSE_ERR(mors, "Invalid BCF buffer length %d\n", bcf_buf_len);
		return -EINVAL;
	}

	bcf_buf = devm_kmalloc(mors->dev, bcf_buf_len, GFP_KERNEL);
	if (!bcf_buf)
		return -ENOMEM;

	ret = get_file_header(bcf->data, &ehdr);
	if (ret) {
		MORSE_ERR(mors, "Wrong file format\n");
		goto exit;
	}

	ret = get_section_header(bcf->data, &ehdr, &sh_strtab, ehdr.e_shstrndx);
	if (ret) {
		MORSE_ERR(mors, "Invalid BCF - missing string table\n");
		goto exit;
	}

	sh_strs = (const char *)bcf->data + sh_strtab.sh_offset;

	mors->regdoms.count = 0;
	if (test_bit(MORSE_STATE_FLAG_REGDOM_SET_BY_OTP, &mors->state_flags)) {
		/* If OTP set, just report this (even if BCF does not define
		 * this regdom); other places will show appropriate errors
		 */
		fill_regdom_info_from_bcf = false;
		ret = add_regdom_to_buffer(mors, regdom_buff, mors->country);
		if (ret)
			goto exit;
	}

	/* Find and copy the board config section */
	for (i = 0; i < ehdr.e_shnum; i++) {
		if (get_section_header(bcf->data, &ehdr, &shdr, i) != 0)
			continue;

		section_name = sh_strs + shdr.sh_name;

		if (strncmp(section_name, ".board_config", sizeof(".board_config")) == 0) {
			ret = copy_bcf_section(bcf, bcf_buf, shdr, bcf_buf_len);
			MORSE_INFO(mors, "BCF section %s, size %d", section_name, ret);
			if (ret < 0)
				goto exit;
			config_len = ret;
			break;
		}
	}

	if (config_len < 0) {
		MORSE_ERR(mors, "Board config section not found in BCF");
		ret = -ENOENT;
		goto exit;
	}

	/* Find and copy the configured country's regdom section */
	for (; i < ehdr.e_shnum; i++) {
		if (get_section_header(bcf->data, &ehdr, &shdr, i) != 0)
			continue;

		if (strncmp(sh_strs + shdr.sh_name, reg_prefix, strlen(reg_prefix)) != 0)
			continue;	/* Not a regdom section */

		/* Name and country code of current regdom section */
		section_name = sh_strs + shdr.sh_name;
		regdom_cc = section_name + strlen(reg_prefix);

		if (fill_regdom_info_from_bcf) {
			ret = add_regdom_to_buffer(mors, regdom_buff, regdom_cc);
			if (ret)
				goto exit;

			ret = add_fallback_regdoms_to_buffer(mors, regdom_buff, regdom_cc);
			if (ret)
				goto exit;
		}

		if ((strncmp(regdom_cc, mors->country, 2) == 0) && regdom_len < 0) {
			ret = copy_bcf_section(bcf, bcf_buf + config_len, shdr,
					       bcf_buf_len - config_len);
			MORSE_INFO(mors, "BCF section %s, size %d", section_name, ret);
			if (ret < 0)
				goto exit;
			regdom_len = ret;

			/* Do not break if parsing all regdoms to save to sysfs */
			if (!fill_regdom_info_from_bcf)
				break;
		}
	}

	if (regdom_len < 0) {
		const char *fallback = morse_bcf_country_fallback(mors->country);

		if (fallback) {
			for (i = 0; i < ehdr.e_shnum; i++) {
				if (get_section_header(bcf->data, &ehdr, &shdr, i) != 0)
					continue;
				section_name = sh_strs + shdr.sh_name;
				if (strncmp(section_name, reg_prefix, strlen(reg_prefix)) != 0)
					continue;
				regdom_cc = section_name + strlen(reg_prefix);
				if (strncmp(regdom_cc, fallback, 2) != 0)
					continue;
				ret = copy_bcf_section(bcf, bcf_buf + config_len, shdr,
						       bcf_buf_len - config_len);
				MORSE_INFO(mors, "BCF section %s, size %d", section_name, ret);
				if (ret < 0)
					goto exit;
				regdom_len = ret;
				if (fill_regdom_info_from_bcf) {
					ret = add_regdom_to_buffer(mors, regdom_buff,
								   mors->country);
					if (ret)
						goto exit;
				}
				break;
			}
		}
	}

	if (regdom_len < 0) {
		MORSE_ERR(mors, "Country code %s not found in BCF", mors->country);
		ret = -ENOENT;
		goto exit;
	}

	if ((config_len + regdom_len) > bcf_buf_len) {
		MORSE_ERR(mors,
			  "BCF sections exceed allocated buffer (%d > %d)\n",
			  config_len + regdom_len, bcf_buf_len);
		ret = -EOVERFLOW;
		goto exit;
	}

	/* Store list of regdoms to mors->regdoms, to expose it to sysfs */
	ret = store_regdom_info(mors, regdom_buff);
	if (ret)
		goto exit;

	/* Write BCF sections to hw */
	morse_claim_bus(mors);
	ret = morse_dm_write(mors, bcf_address, bcf_buf, config_len + regdom_len);
	morse_release_bus(mors);

exit:
	devm_kfree(mors->dev, bcf_buf);
	if (ret)
		MORSE_ERR(mors, "%s failed (ret:%d)\n", __func__, ret);
	return ret;
}

static int morse_firmware_reset(struct morse *mors)
{
	return mors->cfg->digital_reset(mors);
}

static void morse_firmware_clear_aon(struct morse *mors)
{
	int idx;
	u8 count = MORSE_REG_AON_COUNT(mors);
	u32 address = MORSE_REG_AON_ADDR(mors);

	if (mem_access_is_restriction_enforced(mors)) {
		MORSE_DBG(mors,
			  "Memory access restriction is enabled. %s not permitted\n",
			  __func__);
		return;
	}

	if (address)
		for (idx = 0; idx < count; idx++, address += 4) {
			if (mors->bus_type == MORSE_HOST_BUS_TYPE_USB && idx == 0)
				/* Keep the USB power domain enabled in AON. */
				morse_reg32_write(mors, address, MORSE_REG_AON_USB_RESET(mors));
			else
				/* clear AON in case there is any latched sleeps */
				morse_reg32_write(mors, address, 0x0);
		}

	morse_hw_toggle_aon_latch(mors);
}

static int morse_firmware_trigger(struct morse *mors)
{
	morse_claim_bus(mors);
	/*
	 * If not coming from a full reset, some AON flags may be latched.
	 * Make sure to clear any hanging AON bits (can affect booting).
	 */
	morse_firmware_clear_aon(mors);

	if (MORSE_REG_CLK_CTRL(mors) != 0)
		morse_reg32_write(mors, MORSE_REG_CLK_CTRL(mors), MORSE_REG_CLK_CTRL_VALUE(mors));


	morse_reg32_write(mors, MORSE_REG_MSI(mors), MORSE_REG_MSI_HOST_INT(mors));
	mem_access_reset_cache(mors);
	morse_release_bus(mors);

	/* Give the chip a chance to boot / prepare for driver interaction after triggering
	 * the MSI register. This time will be longer for some targets than others
	 */
	mdelay(mors->cfg->get_firmware_trigger_delay_ms(mors->chip_id));

	return 0;
}

int morse_firmware_magic_verify(struct morse *mors)
{
	int ret = 0;
	int magic = ~MORSE_REG_HOST_MAGIC_VALUE(mors);	/* not the magic value */

	morse_claim_bus(mors);

	morse_reg32_read(mors, mors->cfg->host_table_ptr +
			 offsetof(struct host_table, magic_number), &magic);

	if (magic != MORSE_REG_HOST_MAGIC_VALUE(mors)) {
		MORSE_ERR(mors, "FW magic mismatch 0x%08x:0x%08x\n",
			  MORSE_REG_HOST_MAGIC_VALUE(mors), magic);
		ret = -EIO;
	}
	morse_release_bus(mors);

	return ret;
}

enum host_table_firmware_flags morse_firmware_get_fw_flags(struct morse *mors)
{
	int ret = 0;
	int fw_flags = 0;

	morse_claim_bus(mors);

	ret = morse_reg32_read(mors, mors->cfg->host_table_ptr +
			       offsetof(struct host_table, firmware_flags), &fw_flags);

	mors->firmware_flags = fw_flags;

	morse_release_bus(mors);

	return ret;
}

int morse_firmware_check_compatibility(struct morse *mors)
{
	int ret = 0;
	u32 fw_version = 0;
	u32 major;
	u32 minor;
	u32 patch;

	morse_claim_bus(mors);

	ret = morse_reg32_read(mors, mors->cfg->host_table_ptr +
			       offsetof(struct host_table, fw_version_number), &fw_version);

	morse_release_bus(mors);

	major = MORSE_SEMVER_GET_MAJOR(fw_version);
	minor = MORSE_SEMVER_GET_MINOR(fw_version);
	patch = MORSE_SEMVER_GET_PATCH(fw_version);

	/* Firmware on device must be recent enough for driver */
	if (ret == 0 && major != MORSE_CMD_SEMVER_MAJOR) {
		MORSE_ERR(mors,
			  "Incompatible FW version: (Driver) %d.%d.%d, (Chip) %d.%d.%d\n",
			  MORSE_CMD_SEMVER_MAJOR,
			  MORSE_CMD_SEMVER_MINOR,
			  MORSE_CMD_SEMVER_PATCH, major, minor, patch);
		ret = -EPERM;
	} else if (ret == 0 && minor != MORSE_CMD_SEMVER_MINOR) {
		MORSE_WARN(mors,
			"FW version mismatch, some features might not be supported: (Driver) %d.%d.%d, (Chip) %d.%d.%d\n",
			MORSE_CMD_SEMVER_MAJOR,
			MORSE_CMD_SEMVER_MINOR,
			MORSE_CMD_SEMVER_PATCH, major, minor, patch);
	}

	return ret;
}

static int morse_firmware_invalidate_host_ptr(struct morse *mors)
{
	int ret;

	mors->cfg->host_table_ptr = 0;
	morse_claim_bus(mors);
	ret = morse_reg32_write(mors, MORSE_REG_HOST_MANIFEST_PTR(mors), 0);
	morse_release_bus(mors);
	return ret;
}

int morse_firmware_get_host_table_ptr(struct morse *mors)
{
	int ret = 0;
	u32 wait_time_ms = 0;
	unsigned long timeout;
	unsigned long start;

	wait_time_ms = mors->cfg->get_cold_boot_time_ms(mors->chip_id);
	morse_claim_bus(mors);

	/*
	 * HW will take a variable length of time to cold boot. Poll until the host table pointer
	 * has been set.
	 */
	start = jiffies;
	timeout = start + msecs_to_jiffies(wait_time_ms);
	while (1) {
		ret = morse_reg32_read(mors,
				       MORSE_REG_HOST_MANIFEST_PTR(mors),
				       &mors->cfg->host_table_ptr);
		if (mors->cfg->host_table_ptr != 0)
			break;

		if (time_after(jiffies, timeout)) {
			ret = -EIO;
			break;
		}
		usleep_range(5000, 10000);
	}

	if (!ret)
		MORSE_INFO(mors, "HW booted in %u ms\n", jiffies_to_msecs(jiffies - start));

	morse_release_bus(mors);

	return ret;
}

static int morse_firmware_read_ext_host_table(struct morse *mors,
					      struct extended_host_table **ext_host_table)
{
	int ret = 0;
	u32 host_tbl_ptr = mors->cfg->host_table_ptr;
	u32 ext_host_tbl_ptr;
	u32 ext_host_tbl_ptr_addr = host_tbl_ptr
	    + offsetof(struct host_table, extended_host_table_addr);
	u32 ext_host_tbl_len;
	u32 ext_host_tbl_len_ptr_addr;
	struct extended_host_table *host_tbl = NULL;

	morse_claim_bus(mors);
	ret = morse_reg32_read(mors, ext_host_tbl_ptr_addr, &ext_host_tbl_ptr);
	if (ret)
		goto err;

	/* check if this fw populated the extended host table */
	if (ext_host_tbl_ptr == 0) {
		ret = -ENXIO;
		goto err;
	}

	ext_host_tbl_len_ptr_addr = ext_host_tbl_ptr +
	    offsetof(struct extended_host_table, extended_host_table_length);

	/* read the length of the extended host table */
	ret = morse_reg32_read(mors, ext_host_tbl_len_ptr_addr, &ext_host_tbl_len);
	if (ret)
		goto err;

	/* Round up to the nearest word, as dm reads must be multiples of word size */
	ext_host_tbl_len = ROUND_BYTES_TO_WORD(ext_host_tbl_len);

	if (WARN_ON(ext_host_tbl_len == 0 || ext_host_tbl_len > INT_MAX)) {
		ret = -EINVAL;
		goto err;
	}

	host_tbl = kmalloc(ext_host_tbl_len, GFP_KERNEL);
	if (!host_tbl) {
		ret = -ENOMEM;
		goto err;
	}

	ret = morse_dm_read(mors, ext_host_tbl_ptr, (u8 *)host_tbl, (int)ext_host_tbl_len);
	if (ret)
		goto err;

	morse_release_bus(mors);

	*ext_host_table = host_tbl;

	return ret;

err:
	morse_release_bus(mors);
	kfree(host_tbl);
	MORSE_ERR(mors, "%s failed %d\n", __func__, ret);
	return ret;
}

/**
 * @brief Set the MAC address based on 1) chip config if set, 2) user value or
 *        3) fall back to a value prefixed with the morse OUI.
 *
 *        MAC address can be overridden entirely using `iw wlanX hw ether`.
 *
 * @param fw_mac_addr The mac address read from the fw manifest table
 * @param mors The global morse config object
 */
static void set_mac_addr(struct morse *mors, u8 *fw_mac_addr)
{
	char *token;
	int res;
	int i;
	bool use_user_mac = false;
	bool use_fw_mac = false;
	char macaddr_suffix_cpy[9];
	char *user_mac = &macaddr_suffix_cpy[0];
	u8 mac_addr_unset[ETH_ALEN];
	u8 macaddr[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	memcpy(macaddr_suffix_cpy, macaddr_suffix, sizeof(macaddr_suffix_cpy));
	memset(mac_addr_unset, 0x00, sizeof(mac_addr_unset));

	/* Set the first 3 octets to the Morse Micro OUI */
	memcpy(macaddr, morse_oui, sizeof(morse_oui));

	use_fw_mac = !!memcmp(mac_addr_unset, fw_mac_addr, sizeof(mac_addr_unset));

	use_user_mac = use_fw_mac ? false :
	    !!strncmp(macaddr_suffix, MORSE_RANDOMISE_OCTETS, strlen(macaddr_suffix));

	if (use_user_mac && !use_fw_mac) {
		/* Parse the user MAC suffix */
		for (i = 3; i < ETH_ALEN; i++) {
			token = strsep(&user_mac, ":");
			if (token) {
				if (!kstrtoint(token, 16, &res)) {
					macaddr[i] = (u8)res;
					continue;
				}
			}
			use_user_mac = false;
			break;
		}
	}

	if (!use_user_mac && !use_fw_mac) {
		eth_random_addr(macaddr);
		/* handle the user passing just the last octet */
		if (macaddr_octet != 0xFF) {
			macaddr[sizeof(macaddr) - 1] = macaddr_octet;
			MORSE_INFO(mors,
				   "Last octet set from macaddr_octet, MAC address is %pM\n",
				   macaddr);
		} else {
			MORSE_INFO(mors,
				   "Using randomly generated MAC address, %pM\n",
				   macaddr);
		}
	} else if (use_fw_mac) {
		memcpy(macaddr, fw_mac_addr, sizeof(macaddr));
	}
	memcpy(&mors->macaddr, &macaddr, sizeof(mors->macaddr));
}

static void update_capabilities_from_ext_host_table(struct morse *mors,
						    struct extended_host_table_capabilities_s1g
						    *caps)
{
	int i;

	for (i = 0; i < FW_CAPABILITIES_FLAGS_WIDTH; i++) {
		mors->capabilities.flags[i] = le32_to_cpu(caps->flags[i]);
		MORSE_INFO(mors, "Firmware Manifest Flags%d: 0x%x", i, le32_to_cpu(caps->flags[i]));
	}
	mors->capabilities.ampdu_mss = caps->ampdu_mss;
	mors->capabilities.morse_mmss_offset = caps->morse_mmss_offset;
	mors->capabilities.beamformee_sts_capability = caps->beamformee_sts_capability;
	mors->capabilities.maximum_ampdu_length_exponent = caps->maximum_ampdu_length;
	mors->capabilities.number_sounding_dimensions = caps->number_sounding_dimensions;

	MORSE_INFO(mors, "\tAMPDU Minimum start spacing: %u\n", caps->ampdu_mss);
	MORSE_INFO(mors, "\tMorse Minimum Start Spacing offset: %u\n", caps->morse_mmss_offset);
	MORSE_INFO(mors, "\tBeamformee STS Capability: %u\n", caps->beamformee_sts_capability);
	MORSE_INFO(mors, "\tNumber of Sounding Dimensions: %u\n", caps->number_sounding_dimensions);
	MORSE_INFO(mors, "\tMaximum AMPDU Length Exponent: %u\n", caps->maximum_ampdu_length);
}

static void update_pager_bypass_tx_status_addr(struct morse *mors,
					       struct extended_host_table_pager_bypass_tx_status
					       *bypass)
{
	mors->chip_if->bypass.tx_sts.location = le32_to_cpu(bypass->tx_status_buffer_addr);
	MORSE_INFO(mors, "TX Status pager bypass enabled: buffer addr 0x%08x\n",
		   mors->chip_if->bypass.tx_sts.location);
}

static void update_pager_bypass_cmd_resp_addr(struct morse *mors,
					       struct extended_host_table_pager_bypass_cmd_resp
					       *bypass)
{
	mors->chip_if->bypass.cmd_resp.location = le32_to_cpu(bypass->cmd_resp_buffer_addr);
	MORSE_INFO(mors, "CMD response pager bypass enabled: buffer addr 0x%08x\n",
		   mors->chip_if->bypass.cmd_resp.location);
}

static void update_headless_config_addr(struct morse *mors,
					struct extended_host_table_headless_cfg_addr
					*headless_cfg)

{
	mors->chip_if->headless_cfg_addr = le32_to_cpu(headless_cfg->headless_cfg_addr);
	MORSE_DBG(mors, "Headless configuration addr 0x%08x\n",
		   mors->chip_if->headless_cfg_addr);
}

static void update_max_ap_num_sta(struct morse *mors,
					struct extended_host_table_max_ap_num_sta
					*max_ap_num_sta)
{
	mors->max_ap_num_sta = le32_to_cpu(max_ap_num_sta->max_ap_num_sta);
	MORSE_DBG(mors, "Maximum number of STAs per AP VIF %d\n",
			mors->max_ap_num_sta);
}

static void update_validate_skb_checksum(struct morse *mors,
					 struct extended_host_table_insert_skb_checksum
					 *validate_checksum)
{
	mors->chip_if->validate_skb_checksum = validate_checksum->insert_and_validate_checksum;
	MORSE_DBG(mors, "Validate checksum inserted by fw %s\n",
		  validate_checksum->insert_and_validate_checksum ? "enabled" : "disabled");
}

static void update_pager_pkt_memory(struct morse *mors,
	struct extended_host_table_pager_pkt_memory *pkt_memory)
{
	if (!mors || !mors->chip_if) {
		MORSE_WARN_ON_ONCE(FEATURE_ID_DEFAULT, 1);
		return;
	}

	mors->chip_if->pkt_memory.base_addr = le32_to_cpu(pkt_memory->base_addr);
	mors->chip_if->pkt_memory.page_len = pkt_memory->page_len;
	mors->chip_if->pkt_memory.page_len_reserved = pkt_memory->page_len_reserved;
	mors->chip_if->pkt_memory.num = pkt_memory->num;
}

const char *morse_firmware_boot_code_to_str(enum morse_cmd_boot_code boot_code)
{
	switch (boot_code) {
	case MORSE_CMD_BOOT_CODE_NONE:
		return "Boot not started";
	case MORSE_CMD_BOOT_CODE_BCF_INIT:
		return "BCF init failed";
	case MORSE_CMD_BOOT_CODE_VALID_BCF_NOT_FOUND:
		return "BCF not found";
	case MORSE_CMD_BOOT_CODE_BCF_LEN_INVALID:
		return "Invalid BCF length";
	case MORSE_CMD_BOOT_CODE_BCF_CRC_INVALID:
		return "Invalid BCF CRC";
	case MORSE_CMD_BOOT_CODE_BCF_UNSUPPORTED_VERSION:
		return "BCF version not supported";
	case MORSE_CMD_BOOT_CODE_BCF_REGDOM_LEN_INVALID:
		return "BCF regdom length invalid";
	case MORSE_CMD_BOOT_CODE_BCF_REGDOM_CRC_INVALID:
		return "BCF regdom CRC invalid";
	case MORSE_CMD_BOOT_CODE_BCF_PARSE_FAIL:
		return "BCF parse fail";
	case MORSE_CMD_BOOT_CODE_COMPLETE:
		return "Boot completed successfully";
	}

	return "Unknown boot code";
}

int morse_firmware_parse_extended_host_table(struct morse *mors)
{
	int ret;
	u8 *head;
	u8 *end;
	struct extended_host_table *ext_host_table = NULL;

	mors->boot_code = MORSE_CMD_BOOT_CODE_NONE;

	ret = morse_firmware_get_fw_flags(mors);
	if (ret)
		goto exit;

	ret = morse_firmware_read_ext_host_table(mors, &ext_host_table);
	if (ret || !ext_host_table)
		goto exit;

	MORSE_INFO(mors, "Firmware Manifest MAC: %pM", ext_host_table->dev_mac_addr);
	set_mac_addr(mors, ext_host_table->dev_mac_addr);

	/* Parse the TLVs */
	head = ext_host_table->ext_host_table_data_tlvs;
	end = ((u8 *)ext_host_table) + le32_to_cpu(ext_host_table->extended_host_table_length);

	while (head < end) {
		struct extended_host_table_tlv_hdr *hdr =
		    (struct extended_host_table_tlv_hdr *)head;

		switch (le16_to_cpu(hdr->tag)) {
		case MORSE_FW_HOST_TABLE_TAG_S1G_CAPABILITIES:
			update_capabilities_from_ext_host_table(mors,
					(struct extended_host_table_capabilities_s1g *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_PAGER_BYPASS_TX_STATUS:
			update_pager_bypass_tx_status_addr(mors,
					(struct extended_host_table_pager_bypass_tx_status *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_INSERT_SKB_CHECKSUM:
			update_validate_skb_checksum(mors,
					(struct extended_host_table_insert_skb_checksum *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_YAPS_TABLE:
			morse_yaps_hw_read_table(mors,
						 &((struct extended_host_table_yaps_table *)
						   hdr)->yaps_table);
			break;

		case MORSE_FW_HOST_TABLE_TAG_PAGER_PKT_MEMORY:
			update_pager_pkt_memory(mors,
					(struct extended_host_table_pager_pkt_memory *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_PAGER_BYPASS_CMD_RESP:
			update_pager_bypass_cmd_resp_addr(mors,
					(struct extended_host_table_pager_bypass_cmd_resp *)hdr);
			break;
		case MORSE_FW_HOST_TABLE_TAG_HEADLESS_CFG_ADDR:
			update_headless_config_addr(mors,
					(struct extended_host_table_headless_cfg_addr *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_MAX_AP_NUM_STA:
			update_max_ap_num_sta(mors,
						(struct extended_host_table_max_ap_num_sta *)hdr);
			break;

		case MORSE_FW_HOST_TABLE_TAG_BOOT_CODE:
			mors->boot_code = ((struct extended_host_table_boot_code *)hdr)->code;
			break;

		default:
			break;
		}

		head += le16_to_cpu(hdr->length);
		if (hdr->length == 0) {
			MORSE_WARN(mors, "Found a 0 length TLV in the extended host table\n");
			break;
		}
	}

	kfree(ext_host_table);

	if (mors->firmware_flags & MORSE_FW_FLAGS_FAILSAFE_MODE)
		dev_err(mors->dev, "ERROR: %s firmware is in failsafe mode due to: %s\n",
			mors->cfg->get_hw_version(mors->chip_id),
			morse_firmware_boot_code_to_str(mors->boot_code));

	return ret;
exit:
	MORSE_ERR(mors, "%s failed %d\n", __func__, ret);
	return ret;
}

/* Caller must kfree() the returned value. */
char *morse_firmware_build_fw_path(struct morse *mors)
{
	if (fw_bin_file[0] == '\0')
		return mors->cfg->get_fw_path(mors->chip_id);
	else
		return kasprintf(GFP_KERNEL, MORSE_FW_DIR "/%s", fw_bin_file);
}

static int morse_firmware_get_init_params(uint test_mode, struct fw_init_params *init_params)
{
	bool download_fw = true;
	bool get_host_table_ptr = true;
	bool verify_fw = true;

	switch (test_mode) {
	case MORSE_CONFIG_TEST_MODE_DISABLED:
		download_fw = true;
		get_host_table_ptr = true;
		verify_fw = true;
		break;
	case MORSE_CONFIG_TEST_MODE_DOWNLOAD_ONLY:
		download_fw = true;
		get_host_table_ptr = false;
		verify_fw = false;
		break;
	case MORSE_CONFIG_TEST_MODE_DOWNLOAD_AND_GET_HOST_TBL_PTR:
		download_fw = true;
		get_host_table_ptr = true;
		verify_fw = false;
		break;
	case MORSE_CONFIG_TEST_MODE_GET_HOST_TBL_PTR_ONLY:
		download_fw = false;
		get_host_table_ptr = true;
		verify_fw = false;
		break;
	case MORSE_CONFIG_TEST_MODE_RESET:
		download_fw = false;
		get_host_table_ptr = false;
		verify_fw = false;
		break;
	case MORSE_CONFIG_TEST_MODE_BUS:
		download_fw = false;
		get_host_table_ptr = false;
		verify_fw = false;
		break;
	case MORSE_CONFIG_TEST_MODE_BUS_PROFILE:
		download_fw = false;
		get_host_table_ptr = false;
		verify_fw = false;
		break;
	default:
		/* Not a valid case */
		return -EINVAL;
	}

	init_params->download_fw = download_fw;
	init_params->get_host_table_ptr = get_host_table_ptr;
	init_params->verify_fw = verify_fw;

	return 0;
}

static int morse_firmware_init_try(struct morse *mors,
				   const struct firmware *fw,
				   const struct firmware *bcf,
				   struct fw_init_params *init_params)
{
	int ret = 0;

	if (mors->chip_was_reset) {
		MORSE_WARN(mors, "%s: Chip was already reset", __func__);
	} else {
		ret = morse_firmware_reset(mors);
		if (ret)
			return ret;
	}

	/* Ensure the chip gets reset if we need to retry init */
	mors->chip_was_reset = false;

	if (mors->cfg->pre_load_prepare) {
		ret = mors->cfg->pre_load_prepare(mors);
		if (ret)
			return ret;
	}

	if (init_params->download_fw) {
		ret = morse_firmware_invalidate_host_ptr(mors);
		if (ret)
			return ret;

		ret = morse_firmware_load(mors, fw);
		if (ret)
			return ret;

		/*
		 * Load the BCF into the firmware. Load failures are ignored because the firmware
		 * will still start, in failsafe mode.
		 */
		if (bcf)
			morse_bcf_load(mors, bcf, mors->bcf.address);

		ret = morse_firmware_trigger(mors);
		if (ret)
			return ret;
	}

	if (mors->cfg->get_secureboot_status) {
		u32 secureboot_status = MORSE_SECURE_BOOT_SUCCESS;

		ret = mors->cfg->get_secureboot_status(mors, &secureboot_status);

		if (!ret && secureboot_status != MORSE_SECURE_BOOT_SUCCESS) {
			MORSE_ERR(mors, "Firmware secure boot failed: %s (0x%x)\n",
					secure_boot_status_to_str(secureboot_status),
					secureboot_status);
			return -EIO;
		}

		if (ret && ret != -EOPNOTSUPP)
			return ret;
	}

	if (init_params->get_host_table_ptr) {
		ret = morse_firmware_get_host_table_ptr(mors);
		if (ret) {
			MORSE_ERR(mors, "FW manifest pointer not set (ret:%d)\n", ret);
			return ret;
		}
	}

	if (init_params->verify_fw) {
		ret = morse_firmware_magic_verify(mors);
		if (ret)
			return ret;
	}

	return 0;
}

static int morse_firmware_init(struct morse *mors,
			       const struct firmware *fw,
			       const struct firmware *bcf,
			       enum morse_config_test_mode test_mode)
{
	int ret;
	int retries = 3;
	struct fw_init_params init_params;

	ret = morse_firmware_get_init_params(test_mode, &init_params);
	if (ret)
		return ret;

	while (retries--) {
		ret = morse_firmware_init_try(mors, fw, bcf, &init_params);
		if (ret == 0)
			break;
	}

	if (ret)
		return ret;

	if (init_params.verify_fw)
		ret = morse_firmware_check_compatibility(mors);

	return ret;
}

static uint32_t binary_crc(const struct firmware *fw)
{
	return ~crc32_le(~0, (unsigned char const *)fw->data, fw->size) & 0xffffffff;
}

static int morse_firmware_request(struct morse *mors, const struct firmware **fw_out,
				  bool use_full_path)
{
	int ret;
	char *fw_path = NULL;
	const char *fw_name;
	const struct firmware *fw = NULL;
	char *p;

	fw_path = morse_firmware_build_fw_path(mors);
	if (!fw_path)
		return -ENOMEM;
	fw_name = fw_path;

	if (!use_full_path) {
		p = strrchr(fw_name, '/');
		if (p)
			fw_name = p + 1;
	}

	ret = request_firmware(&fw, fw_name, mors->dev);
	if (ret != 0) {
		if (ret == -ENOENT)
			dev_err(mors->dev, "Firmware %s not found\n", fw_name);
		goto exit;
	}

	dev_info(mors->dev, "Loaded firmware from %s, size %zu, crc32 0x%08x\n",
		fw_name, fw->size, binary_crc(fw));

	/* Clear out extra ACK timeout; its value is unknown */
	mors->extra_ack_timeout_us = -1;

	/* store the fw binary string into our coredump */
	morse_coredump_set_fw_binary_str(mors, fw_name);

	*fw_out = fw;

exit:
	kfree(fw_path);

	return ret;
}

static void morse_bcf_request(struct morse *mors, const struct firmware **bcf_out,
			     bool use_full_path)
{
	int n;
	int ret;
	char bcf_path[MAX_BCF_NAME_LEN];
	const char *bcf_name = bcf_path;
	const struct firmware *bcf = NULL;
	char *p;

	if (strlen(board_config_file) > 0) {
		n = snprintf(bcf_path, sizeof(bcf_path), "%s/%s", MORSE_FW_DIR, board_config_file);
	} else if (strlen(mors->board_serial) > 0) {
		if (memcmp(mors->board_serial, "default", sizeof("default")) == 0 &&
		    (mors->board_id > 0 && mors->board_id < mors->cfg->board_type_max_value)) {
			/* Use board ID read from chip if it's non-zero and the board serial is
			 * default.
			 */
			MORSE_INFO(mors, "Using board type 0x%04x from OTP\n", mors->board_id);
			n = snprintf(bcf_path, sizeof(bcf_path),
				     "%s/bcf_boardtype_%04x.bin", MORSE_FW_DIR, mors->board_id);
		} else {
			/* fallback to the old style */
			n = snprintf(bcf_path, sizeof(bcf_path),
				     "%s/bcf_%s.bin", MORSE_FW_DIR, mors->board_serial);
		}
	} else {
		MORSE_ERR(mors, "%s: BCF or Serial parameters are not defined\n", __func__);
		return;
	}

	if (n < 0 || n > (sizeof(bcf_path) - 1)) {
		MORSE_ERR(mors, "%s: Failed to create a BCF path\n", __func__);
		return;
	}

	if (!use_full_path) {
		p = strrchr(bcf_name, '/');
		if (p)
			bcf_name = p + 1;
	}

	ret = request_firmware(&bcf, bcf_name, mors->dev);
	if (ret != 0) {
		if (ret == -ENOENT)
			MORSE_ERR(mors, "BCF %s not found\n", bcf_name);
		return;
	}

	/* Calculate CRC to match the crc32 line command */
	dev_info(mors->dev, "Loaded BCF from %s, size %zu, crc32 0x%08x\n",
		bcf_name, bcf->size, binary_crc(bcf));

	*bcf_out = bcf;
}

int morse_firmware_prepare(struct morse *mors, bool reset_hw, bool reattach_hw)
{
	int ret = 0;
	bool is_hw_loaded = false;
	const struct firmware *fw = NULL;
	const struct firmware *bcf = NULL;
#ifdef CONFIG_ANDROID
	bool use_full_path = false; /* Use filenames only - Android sets the path */
#else
	bool use_full_path = true;
#endif

	lockdep_assert_held(&mors->lock);

	if (enable_otp_check) {
		if (mors->cfg->get_board_type)
			mors->board_id = mors->cfg->get_board_type(mors);

		if (mors->cfg->get_encoded_country) {
			if (mors->cfg->get_encoded_country(mors) == 0)
				set_bit(MORSE_STATE_FLAG_REGDOM_SET_BY_OTP, &mors->state_flags);
		}
	}

	ret = morse_firmware_request(mors, &fw, use_full_path);
	if (ret)
		return ret;

	morse_bcf_request(mors, &bcf, use_full_path);

	ret = extract_firmware_info(mors, fw);
	if (ret)
		goto exit;

	if (!reset_hw && reattach_hw) {
		if (!morse_test_mode_enabled(test_mode)) {
			is_hw_loaded = morse_hw_is_already_loaded(mors);
			MORSE_DBG(mors, "HW is %s loaded\n", is_hw_loaded ? "already" : "not yet");
		}

		if (is_hw_loaded) {
			char *stop = NULL;
			bool is_stopped = morse_hw_is_stopped(mors) ||
					  morse_hw_is_stopped_notification_set(mors);

			/* Attempt to get stop information string from HW */
			morse_coredump_get_stop_info(mors, &stop);
			is_stopped |= ((bool)stop);

			if (is_stopped) {
				ret = morse_coredump_new(mors,
							 MORSE_COREDUMP_REASON_STOP_ON_REATTACH);
				if (!ret)
					ret = morse_coredump(mors);

				/* Coredump failed, log stop string if present */
				if (stop && ret)
					MORSE_ERR(mors, "stop at %s\n", stop);

				kfree(stop);

				/* Stopped HW requires a full digital reset before downloading
				 * the firmware
				 */
				reset_hw = true;
			} else {
				/* Tell caller HW is ready to attach */
				ret = -EALREADY;
				goto exit;
			}
		}
	}

	if (reset_hw) {
		if (mors->cfg->pre_firmware_ndr)
			mors->cfg->pre_firmware_ndr(mors);

		morse_claim_bus(mors);
		morse_firmware_clear_aon(mors);
		morse_release_bus(mors);
	}

	ret = morse_firmware_init(mors, fw, bcf, test_mode);
	if (ret == 0) {
		morse_hw_headless_reset(mors);
		if (reset_hw && mors->cfg->post_firmware_ndr)
			mors->cfg->post_firmware_ndr(mors);
	}

	if (ret)
		MORSE_ERR(mors, "%s failed: %d\n", __func__, ret);
	else
		MORSE_INFO(mors, "Firmware initialized\n");

exit:
	release_firmware(fw);
	release_firmware(bcf);

	return ret;
}
