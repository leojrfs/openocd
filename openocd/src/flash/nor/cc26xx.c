/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "cc26xx.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>

#define FLASH_TIMEOUT 8000

struct cc26xx_bank {
	const char *family_name;
	uint32_t icepick_id;
	uint32_t user_id;
	uint32_t device_type;
	uint32_t sector_length;
	uint32_t sram_size;
	bool probed;
	struct working_area *working_area;
	struct armv7m_algorithm armv7m_info;
	const uint8_t *algo_code;
	uint32_t algo_size;
	uint32_t algo_working_size;
	uint32_t algo_entry;
	uint32_t algo_buffer[2];
	uint32_t algo_params[2];
	uint32_t algo_status[2];
};

static int cc26xx_auto_probe(struct flash_bank *bank);

static uint32_t cc26xx_device_type(uint32_t icepick_id, uint32_t user_id)
{
	uint32_t device_type = 0;
	
	switch (icepick_id & ICEPICK_ID_MASK) {
		case CC26X0_ICEPICK_ID:
			device_type = CC26X0_TYPE;
			break;
		case CC26X1_ICEPICK_ID:
			device_type = CC26X1_TYPE;
			break;
		case CC13X0_ICEPICK_ID:
			device_type = CC13X0_TYPE;
			break;			
		case CC13X2_CC26X2_ICEPICK_ID:
		default:
			if ((user_id & USER_ID_CC13_MASK) != 0)
				device_type = CC13X2_TYPE;
			else
				device_type = CC26X2_TYPE;
			break;
	}
	
	return device_type;
}

static uint32_t cc26xx_sram_size(uint32_t icepick_id, uint32_t size_code)
{
	uint32_t sram_size;
	
	switch (icepick_id & ICEPICK_ID_MASK) {
		case CC26X0_ICEPICK_ID:
		case CC26X1_ICEPICK_ID:
		case CC13X0_ICEPICK_ID:
			/* Chameleon family device */
			switch (icepick_id & ICEPICK_REV_MASK) {
				case 0x00000000:
				case 0x10000000:
					/* PG1 silicon had less SRAM available */
					switch (size_code) {
						case 0:
							sram_size = 0x800;
							break;
						case 1:
							sram_size = 0x1000;
							break;
						case 2:
							sram_size = 0x2000;
							break;
						case 3:
						default:
							sram_size = 0x4000;
							break;
					}
					break;
				default:
					/* All other revisions are PG2 or later */
					switch (size_code) {
						case 0:
							sram_size = 0x1000;
							break;
						case 1:
							sram_size = 0x2800;
							break;
						case 2:
							sram_size = 0x4000;
							break;
						case 3:
						default:
							sram_size = 0x5000;
							break;
					}
					break;
			}
			break;
		case CC13X2_CC26X2_ICEPICK_ID:
		default:
			/* Agama family device */
			switch (size_code) {
				case 0:
					sram_size = 0x8000;
					break;
				case 1:
					sram_size = 0xc000;
					break;
				case 2:
					sram_size = 0x10000;
					break;
				case 3:
				default:
					sram_size = 0x14000;
					break;
			}
			break;
	}
	
	return sram_size;
}

static uint32_t cc26xx_sector_length(uint32_t icepick_id)
{
	uint32_t sector_length;
	
	switch (icepick_id & ICEPICK_ID_MASK) {
		case CC26X0_ICEPICK_ID:
		case CC26X1_ICEPICK_ID:
		case CC13X0_ICEPICK_ID:
			/* Chameleon family device */
			sector_length = CC26XX_CHAMELEON_SECTOR_LENGTH;
			break;
		case CC13X2_CC26X2_ICEPICK_ID:
		default:
			/* Agama family device */
			sector_length = CC26XX_AGAMA_SECTOR_LENGTH;
			break;
	}
	
	return sector_length;
}

static int cc26xx_wait_algo_done(struct flash_bank *bank, uint32_t status_addr)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;

	uint32_t status = CC26XX_BUFFER_FULL;
	long long start_ms;
	long long elapsed_ms;

	int retval = ERROR_OK;

	start_ms = timeval_ms();
	while (CC26XX_BUFFER_FULL == status) {
		retval = target_read_buffer(target, status_addr, sizeof(status), 
					(uint8_t *)&status);
		if (ERROR_OK != retval)
			return retval;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > FLASH_TIMEOUT)
			break;
	};

	if (CC26XX_BUFFER_EMPTY != status) {
		LOG_ERROR("%s: Flash operation failed", cc26xx_bank->family_name);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cc26xx_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	
	int retval;
	
	/* Make sure we've probed the flash to get the device and size */
	retval = cc26xx_auto_probe(bank);
	if (ERROR_OK != retval)
		return retval;

	/* Check for working area to use for flash helper algorithm */
	if (0 != cc26xx_bank->working_area)
		target_free_working_area(target, cc26xx_bank->working_area);
	retval = target_alloc_working_area(target, cc26xx_bank->algo_working_size,
				&cc26xx_bank->working_area);
	if (ERROR_OK != retval)
		return retval;

	/* Confirm the defined working address is the area we need to use */
	if (CC26XX_ALGO_BASE_ADDRESS != cc26xx_bank->working_area->address)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, CC26XX_ALGO_BASE_ADDRESS,
				cc26xx_bank->algo_size, cc26xx_bank->algo_code);
	if (ERROR_OK != retval)
		return retval;

	/* Initialize the ARMv7 specific info to run the algorithm */
	cc26xx_bank->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	cc26xx_bank->armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Begin executing the flash helper algorithm */
	retval = target_start_algorithm(target, 0, 0, 0, 0,
				cc26xx_bank->algo_entry, 0, &cc26xx_bank->armv7m_info);
	if (ERROR_OK != retval) {
		LOG_ERROR("%s: Failed to start flash helper algorithm",
			cc26xx_bank->family_name);
		return retval;
	}

	/*
	 * At this point, the algorithm is running on the target and
	 * ready to receive commands and data to flash the target
	 */

	/* Mark erased status of sectors as "unknown" */
	if (0 != bank->sectors) {
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = -1;
	}

	return retval;
}

static int cc26xx_quit(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;

	int retval;

	/* Regardless of the algo's status, attempt to halt the target */
	(void)target_halt(target);

	/* Now confirm target halted and clean up from flash helper algorithm */
	retval = target_wait_algorithm(target, 0, 0, 0, 0, 0, FLASH_TIMEOUT,
				&cc26xx_bank->armv7m_info);

	target_free_working_area(target, cc26xx_bank->working_area);
	cc26xx_bank->working_area = 0;

	return retval;
}

static int cc26xx_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	struct cc26xx_algo_params algo_params;

	int retval;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = cc26xx_init(bank);
	if (ERROR_OK != retval)
		return retval;
	
	/* Initialize algorithm parameters */
	algo_params.address = 0;
	algo_params.length = 4;
	algo_params.command = CC26XX_CMD_ERASE_ALL;
	algo_params.status = CC26XX_BUFFER_FULL;
	algo_params.buffer = cc26xx_bank->algo_buffer[0];

	/* Issue the erase all command to the flash helper algorithm */
	retval = target_write_buffer(target, cc26xx_bank->algo_params[0],
				sizeof(algo_params), (uint8_t *)&algo_params);

	/* Wait for command to complete */
	if (ERROR_OK == retval) 
		retval = cc26xx_wait_algo_done(bank, cc26xx_bank->algo_status[0]);
	
	/* Regardless of errors, try to close down algo */
	(void)cc26xx_quit(bank);
	
	/* If no errors, mark sectors as erased */
	if (ERROR_OK == retval)
		if (0 != bank->sectors)			
			for (int i = 0; i < bank->num_sectors; i++) 
				bank->sectors[i].is_erased = 1;

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(cc26xx_flash_bank_command)
{
	struct cc26xx_bank *cc26xx_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cc26xx_bank = malloc(sizeof(struct cc26xx_bank));
	if (0 == cc26xx_bank)
		return ERROR_FAIL;

	/* Initialize private flash information */
	memset((void *)cc26xx_bank, 0x00, sizeof(struct cc26xx_bank));
	cc26xx_bank->family_name = "cc26xx";
	cc26xx_bank->device_type = CC26XX_NO_TYPE;
	cc26xx_bank->sector_length = 0x1000;
	
	/* Finish initialization of bank */
	bank->driver_priv = cc26xx_bank;
	bank->next = 0;

	return ERROR_OK;
}

static int cc26xx_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	struct cc26xx_algo_params algo_params[2];

	uint32_t index;
	uint32_t value;
	int retval;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Do a mass erase if user requested all sectors of main flash */
	if ((0 == bank->bank_number) && (first == 0) &&
		(last == (bank->num_sectors - 1))) {
		/* Request mass erase of main flash */
		return cc26xx_mass_erase(bank);
	}

	retval = cc26xx_init(bank);
	if (ERROR_OK != retval)
		return retval;

	/* Initialize buffers to write a word of all ones */
	value = 0xffffffff;
	retval = target_write_buffer(target, cc26xx_bank->algo_buffer[0],
				4, (uint8_t *)&value);
	if (ERROR_OK != retval)
		return retval;
	if ((last - first) > 0) {
		retval = target_write_buffer(target, cc26xx_bank->algo_buffer[1],
					4, (uint8_t *)&value);
		if (ERROR_OK != retval)
			return retval;
	}
	
	/* Initialize algorithm parameters to default values */
	algo_params[0].length = 4;
	algo_params[0].command = CC26XX_CMD_ERASE_AND_PROGRAM;
	algo_params[0].buffer = cc26xx_bank->algo_buffer[0];
	algo_params[1].length = 4;
	algo_params[1].command = CC26XX_CMD_ERASE_AND_PROGRAM;
	algo_params[1].buffer = cc26xx_bank->algo_buffer[1];

	/* Erase requested sectors one by one */
	index = 0;
	for (int i = first; i <= last; i++) {

		/* Convert sector number to starting address of sector */
		algo_params[index].address =
			bank->base + (i * cc26xx_bank->sector_length);
		
		/* Reset sector status to flag we've started a new command */
		algo_params[index].status = CC26XX_BUFFER_FULL;

		/* Issue the sector erase command to the flash helper algorithm */
		retval = target_write_buffer(target, cc26xx_bank->algo_params[index],
					sizeof(algo_params[index]), (uint8_t *)&algo_params[index]);
		if (ERROR_OK != retval)
			break;

		/* Wait for next ping pong buffer to be ready */
		index ^= 1;
		retval = cc26xx_wait_algo_done(bank, cc26xx_bank->algo_status[index]);
		if (ERROR_OK != retval)
			break;
	}

	/* If no error yet, wait for last sector to finish */
	if (ERROR_OK == retval) {
		index ^= 1;
		retval = cc26xx_wait_algo_done(bank, cc26xx_bank->algo_status[index]);
	}
	
	/* Regardless of errors, try to close down algo */
	(void)cc26xx_quit(bank);
	
	/* If no errors, mark sectors as erased */
	if (ERROR_OK == retval)
		if (0 != bank->sectors)			
			for (int i = first; i <= last; i++) 
				bank->sectors[i].is_erased = 1;

	return retval;
}

static int cc26xx_protect(struct flash_bank *bank, int set, int first,
	int last)
{
	return ERROR_OK;
}

static int cc26xx_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	struct cc26xx_algo_params algo_params[2];
	uint32_t size = 0;
	long long start_ms;
	long long elapsed_ms;
	uint32_t address;
	uint32_t end_address = offset + count - 1;
	uint32_t sector;

	uint32_t index;
	int retval;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = cc26xx_init(bank);
	if (ERROR_OK != retval)
		return retval;

	/* Initialize algorithm parameters to default values */
	algo_params[0].command = CC26XX_CMD_PROGRAM;
	algo_params[0].buffer = cc26xx_bank->algo_buffer[0];
	algo_params[1].command = CC26XX_CMD_PROGRAM;
	algo_params[1].buffer = cc26xx_bank->algo_buffer[1];
	
	/* Write requested data, ping ponging between two buffers */
	index = 0;
	start_ms = timeval_ms();
	address = bank->base + offset;
	while (count > 0) {
		
		if (count > cc26xx_bank->sector_length)
			size = cc26xx_bank->sector_length;
		else
			size = count;

		/* Put next block of data to flash into buffer */
		retval = target_write_buffer(target, cc26xx_bank->algo_buffer[index],
					size, buffer);
		if (ERROR_OK != retval) {
			LOG_ERROR("Unable to write data to target memory");
			break;
		}
		
		/* Update algo parameters for next block */
		algo_params[index].address = address;
		algo_params[index].length = size;
		algo_params[index].status = CC26XX_BUFFER_FULL;

		/* Issue the sector erase command to the flash helper algorithm */
		retval = target_write_buffer(target, cc26xx_bank->algo_params[index],
					sizeof(algo_params[index]), (uint8_t *)&algo_params[index]);
		if (ERROR_OK != retval)
			break;
		
		/* Wait for next ping pong buffer to be ready */
		index ^= 1;
		retval = cc26xx_wait_algo_done(bank, cc26xx_bank->algo_status[index]);
		if (ERROR_OK != retval)
			break;
		
		count -= size;
		buffer += size;
		address += size;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
	}

	/* If no error yet, wait for last buffer to finish */
	if (ERROR_OK == retval) {
		index ^= 1;
		retval = cc26xx_wait_algo_done(bank, cc26xx_bank->algo_status[index]);
	}

	/* Regardless of errors, try to close down algo */
	(void)cc26xx_quit(bank);

	/* If no errors, mark sectors as not erased */
	if (ERROR_OK == retval)
		if (0 != bank->sectors)
			while (offset <= end_address) {
				sector = offset / cc26xx_bank->sector_length;
				bank->sectors[sector].is_erased = 0;
				offset += cc26xx_bank->sector_length;
			}

	return retval;
}

static int cc26xx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	
	uint32_t sector_length;
	uint32_t value;
	int num_sectors;
	
	int retval;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	retval = target_read_u32(target, FCFG1_ICEPICK_ID, &value);
	if (ERROR_OK != retval)
		return retval;
	cc26xx_bank->icepick_id = value;
	
	retval = target_read_u32(target, FCFG1_USER_ID, &value);
	if (ERROR_OK != retval)
		return retval;
	cc26xx_bank->user_id = value;
	
	cc26xx_bank->device_type = cc26xx_device_type(cc26xx_bank->icepick_id,
		cc26xx_bank->user_id);
	
	sector_length = cc26xx_sector_length(cc26xx_bank->icepick_id);
	
	retval = target_read_u32(target, CC26XX_SRAM_SIZE_INFO, &value);
	if (ERROR_OK != retval)
		return retval;
	cc26xx_bank->sram_size = cc26xx_sram_size(cc26xx_bank->icepick_id, value);
	
	retval = target_read_u32(target, CC26XX_FLASH_SIZE_INFO, &value);
	if (ERROR_OK != retval)
		return retval;
	num_sectors = value & 0xff;
	if (num_sectors > CC26XX_MAX_SECTOR_COUNT)
		num_sectors = CC26XX_MAX_SECTOR_COUNT;

	/* Set up appropriate flash helper algorithm */
	switch (cc26xx_bank->icepick_id & ICEPICK_ID_MASK) {
		case CC26X0_ICEPICK_ID:
		case CC26X1_ICEPICK_ID:
		case CC13X0_ICEPICK_ID:
			/* Chameleon family device */
			cc26xx_bank->algo_code = cc26xx_chameleon_algo;
			cc26xx_bank->algo_size = sizeof(cc26xx_chameleon_algo);
			cc26xx_bank->algo_working_size = CC26XX_CHAMELEON_WORKING_SIZE;
			cc26xx_bank->algo_entry = CC26XX_CHAMELEON_ALGO_ENTRY;
			cc26xx_bank->algo_buffer[0] = CC26XX_CHAMELEON_ALGO_BUFFER_0;
			cc26xx_bank->algo_buffer[1] = CC26XX_CHAMELEON_ALGO_BUFFER_1;
			cc26xx_bank->algo_params[0] = CC26XX_CHAMELEON_ALGO_PARAMS_0;
			cc26xx_bank->algo_params[1] = CC26XX_CHAMELEON_ALGO_PARAMS_1;
			cc26xx_bank->algo_status[0] = CC26XX_CHAMELEON_ALGO_STATUS_0;
			cc26xx_bank->algo_status[1] = CC26XX_CHAMELEON_ALGO_STATUS_1;
			break;
		case CC13X2_CC26X2_ICEPICK_ID:
		default:
			/* Agama family device */
			cc26xx_bank->algo_code = cc26xx_agama_algo;
			cc26xx_bank->algo_size = sizeof(cc26xx_agama_algo);
			cc26xx_bank->algo_working_size = CC26XX_AGAMA_WORKING_SIZE;
			cc26xx_bank->algo_entry = CC26XX_AGAMA_ALGO_ENTRY;
			cc26xx_bank->algo_buffer[0] = CC26XX_AGAMA_ALGO_BUFFER_0;
			cc26xx_bank->algo_buffer[1] = CC26XX_AGAMA_ALGO_BUFFER_1;
			cc26xx_bank->algo_params[0] = CC26XX_AGAMA_ALGO_PARAMS_0;
			cc26xx_bank->algo_params[1] = CC26XX_AGAMA_ALGO_PARAMS_1;
			cc26xx_bank->algo_status[0] = CC26XX_AGAMA_ALGO_STATUS_0;
			cc26xx_bank->algo_status[1] = CC26XX_AGAMA_ALGO_STATUS_1;
			break;
	}
	
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (0 == bank->sectors)
		return ERROR_FAIL;

	bank->base = CC26XX_FLASH_BASE_ADDR;
	bank->num_sectors = num_sectors;
	bank->size = num_sectors * sector_length;
	cc26xx_bank->sector_length = sector_length;

	for (int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * sector_length;
		bank->sectors[i].size = sector_length;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* We've successfully determined the stats on the flash bank */
	cc26xx_bank->probed = true;

	/* If we fall through to here, then all went well */

	return ERROR_OK;
}

static int cc26xx_auto_probe(struct flash_bank *bank)
{
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;

	int retval = ERROR_OK;

	if (bank->bank_number != 0) {
		/* Invalid bank number somehow */
		return ERROR_FAIL;
	}

	if (!cc26xx_bank->probed)
		retval = cc26xx_probe(bank);

	return retval;
}

static int cc26xx_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int cc26xx_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct cc26xx_bank *cc26xx_bank = bank->driver_priv;
	int printed = 0;

	switch (cc26xx_bank->device_type) {
		case CC26X0_TYPE:
			printed = snprintf(buf, buf_size, "CC26x0");
			break;
		case CC26X1_TYPE:
			printed = snprintf(buf, buf_size, "CC26x1");
			break;
		case CC13X0_TYPE:
			printed = snprintf(buf, buf_size, "CC13x0");
			break;
		case CC13X2_TYPE:
			printed = snprintf(buf, buf_size, "CC13x2");
			break;
		case CC26X2_TYPE:
			printed = snprintf(buf, buf_size, "CC26x2");
			break;
		case CC26XX_NO_TYPE:
		default:
			printed = snprintf(buf, buf_size, "Unrecognized");
			break;
	}
	
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, 
		" device: ICEPick ID 0x%08x, USER ID 0x%08x\n",
		cc26xx_bank->icepick_id, cc26xx_bank->user_id);
	
	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size, 
		"flash size = 0x%x, SRAM size = 0x%x",
		bank->num_sectors * cc26xx_bank->sector_length, cc26xx_bank->sram_size);
	
	buf += printed;
	buf_size -= printed;
	
	if (0 > buf_size)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}

struct flash_driver cc26xx_flash = {
	.name = "cc26xx",
	.flash_bank_command = cc26xx_flash_bank_command,
	.erase = cc26xx_erase,
	.protect = cc26xx_protect,
	.write = cc26xx_write,
	.read = default_flash_read,
	.probe = cc26xx_probe,
	.auto_probe = cc26xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cc26xx_protect_check,
	.info = cc26xx_info,
};
