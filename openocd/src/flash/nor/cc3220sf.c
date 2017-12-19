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
#include "cc3220sf.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_TIMEOUT 5000

struct cc3220sf_bank {
	bool probed;
	struct working_area *working_area;
	struct armv7m_algorithm armv7m_info;
};

static int cc3220sf_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	bool done;
	long long start_ms;
	long long elapsed_ms;
	uint32_t value;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Set starting address to erase to zero */
	value = 0;
	retval = target_write_buffer(target, FMA_REGISTER_ADDR,
				sizeof(value), (uint8_t *)&value);
	if (ERROR_OK != retval)
		return retval;

	/* Write the MERASE bit of the FMC register */
	value = FMC_DEFAULT_VALUE | FMC_MERASE_BIT;
	retval = target_write_buffer(target, FMC_REGISTER_ADDR,
				sizeof(value), (uint8_t *)&value);
	if (ERROR_OK != retval)
		return retval;

	/* Poll the MERASE bit until the mass erase is complete */
	done = false;
	start_ms = timeval_ms();
	while (!done) {
		retval = target_read_buffer(target, FMC_REGISTER_ADDR,
					sizeof(value), (uint8_t *)&value);
		if (ERROR_OK != retval)
			return retval;

		if ((value & FMC_MERASE_BIT) == 0) {
			/* Bit clears when mass erase is finished */
			done = true;
		} else {
			elapsed_ms = timeval_ms() - start_ms;
			if (elapsed_ms > 500)
				keep_alive();
			if (elapsed_ms > FLASH_TIMEOUT)
				break;
		}
	}

	if (!done) {
		/* Mass erase timed out waiting for confirmation */
		return ERROR_FAIL;
	}

	/* Mark all sectors erased */
	if (0 != bank->sectors) {
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;
	}

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(cc3220sf_flash_bank_command)
{
	struct cc3220sf_bank *cc3220sf_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cc3220sf_bank = malloc(sizeof(struct cc3220sf_bank));
	if (0 == cc3220sf_bank)
		return ERROR_FAIL;

	/* Initialize private flash information */
	cc3220sf_bank->probed = false;
	cc3220sf_bank->working_area = 0;

	/* Finish initialization of flash bank */
	bank->driver_priv = cc3220sf_bank;
	bank->next = 0;

	return ERROR_OK;
}

static int cc3220sf_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	bool done;
	long long start_ms;
	long long elapsed_ms;
	uint32_t address;
	uint32_t value;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Do a mass erase if user requested all sectors of flash */
	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		/* Request mass erase of flash */
		return cc3220sf_mass_erase(bank);
	}

	/* Erase requested sectors one by one */
	for (int i = first; i <= last; i++) {

		/* Determine address of sector to erase */
		address = FLASH_BASE_ADDR + i * FLASH_SECTOR_SIZE;

		/* Set starting address to erase */
		retval = target_write_buffer(target, FMA_REGISTER_ADDR,
					sizeof(address), (uint8_t *)&address);
		if (ERROR_OK != retval)
			return retval;

		/* Write the ERASE bit of the FMC register */
		value = FMC_DEFAULT_VALUE | FMC_ERASE_BIT;
		retval = target_write_buffer(target, FMC_REGISTER_ADDR,
					sizeof(value), (uint8_t *)&value);
		if (ERROR_OK != retval)
			return retval;

		/* Poll the ERASE bit until the erase is complete */
		done = false;
		start_ms = timeval_ms();
		while (!done) {
			retval = target_read_buffer(target, FMC_REGISTER_ADDR,
						sizeof(value), (uint8_t *)&value);
			if (ERROR_OK != retval)
				return retval;

			if ((value & FMC_ERASE_BIT) == 0) {
				/* Bit clears when mass erase is finished */
				done = true;
			} else {
				elapsed_ms = timeval_ms() - start_ms;
				if (elapsed_ms > 500)
					keep_alive();
				if (elapsed_ms > FLASH_TIMEOUT)
					break;
			}
		}

		if (!done) {
			/* Sector erase timed out waiting for confirmation */
			return ERROR_FAIL;
		}

		/* Mark the sector as erased */
		if (0 != bank->sectors)
			bank->sectors[i].is_erased = 1;
	}

	return retval;
}

static int cc3220sf_protect(struct flash_bank *bank, int set, int first,
	int last)
{
	return ERROR_OK;
}

static int cc3220sf_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;
	struct reg_param reg_params[3];
	uint32_t address;
	uint32_t remaining;
	uint32_t words;
	uint32_t end_address = offset + count - 1;
	uint32_t sector;
	uint32_t result;

	int retval = ERROR_OK;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check for working area to use for flash helper algorithm */
	if (0 != cc3220sf_bank->working_area)
		target_free_working_area(target, cc3220sf_bank->working_area);
	retval = target_alloc_working_area(target, ALGO_WORKING_SIZE,
				&cc3220sf_bank->working_area);
	if (ERROR_OK != retval)
		return retval;

	/* Confirm the defined working address is the area we need to use */
	if (ALGO_BASE_ADDR != cc3220sf_bank->working_area->address)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Write flash helper algorithm into target memory */
	retval = target_write_buffer(target, ALGO_BASE_ADDR,
				sizeof(cc3220sf_algo), cc3220sf_algo);
	if (ERROR_OK != retval)
		return retval;

	/* Initialize the ARMv7m specific info to run the algorithm */
	cc3220sf_bank->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	cc3220sf_bank->armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Initialize register params for flash helper algorithm */
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);

	/* Prepare to write to flash */
	address = FLASH_BASE_ADDR + offset;
	remaining = count;

	while (remaining > 0) {
		/* Helper parameters are passed in registers R0-R2 */
		/* Set address to write to and start of data buffer */
		buf_set_u32(reg_params[0].value, 0, 32, ALGO_BUFFER_ADDR);
		buf_set_u32(reg_params[1].value, 0, 32, address);

		/* Download data to write into memory buffer */
		if (remaining >= ALGO_BUFFER_SIZE) {
			/* Fill up buffer with data to flash */
			retval = target_write_buffer(target, ALGO_BUFFER_ADDR,
						ALGO_BUFFER_SIZE, buffer);
			if (ERROR_OK != retval)
				break;

			/* Count to write is in 32-bit words */
			words = ALGO_BUFFER_SIZE / 4;

			/* Bump variables to next data */
			address += ALGO_BUFFER_SIZE;
			buffer += ALGO_BUFFER_SIZE;
			remaining -= ALGO_BUFFER_SIZE;
		} else {
			/* Fill buffer with what's left of the data */
			retval = target_write_buffer(target, ALGO_BUFFER_ADDR,
						remaining, buffer);
			if (ERROR_OK != retval)
				break;

			/* Calculate the final word count to write */
			words = remaining / 4;
			if (0 != (remaining % 4))
				words++;

			/* All done after this last buffer */
			remaining = 0;
		}

		/* Set number of words to write */
		buf_set_u32(reg_params[2].value, 0, 32, words);

		/* Execute the flash helper algorithm */
		retval = target_run_algorithm(target, 0, 0, 3, reg_params,
					ALGO_BASE_ADDR, 0, FLASH_TIMEOUT,
					&cc3220sf_bank->armv7m_info);
		if (ERROR_OK != retval) {
			LOG_ERROR("cc3220sf: Flash algorithm failed to run");
			break;
		}

		/* Check that all words were written to flash */
		result = buf_get_u32(reg_params[2].value, 0, 32);
		if (0 != result) {
			retval = ERROR_FAIL;
			LOG_ERROR("cc3220sf: Flash operation failed");
			break;
		}
	}

	/* Free resources  */
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	target_free_working_area(target, cc3220sf_bank->working_area);
	cc3220sf_bank->working_area = 0;

	if (ERROR_OK == retval) {
		/* Mark flashed sectors as "not erased" */
		while (offset <= end_address) {
			sector = offset / FLASH_SECTOR_SIZE;
			bank->sectors[sector].is_erased = 0;
			offset += FLASH_SECTOR_SIZE;
		}
	}

	return retval;
}

static int cc3220sf_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;

	uint32_t base;
	uint32_t size;
	int num_sectors;
	int bank_id;

	bank_id = bank->bank_number;

	if (TARGET_HALTED != target->state) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (0 == bank_id) {
		base = FLASH_BASE_ADDR;
		size = FLASH_NUM_SECTORS * FLASH_SECTOR_SIZE;
		num_sectors = FLASH_NUM_SECTORS;
	} else {
		/* Invalid bank number somehow */
		return ERROR_FAIL;
	}

	if (0 != bank->sectors) {
		free(bank->sectors);
		bank->sectors = 0;
	}

	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (0 == bank->sectors)
		return ERROR_FAIL;

	bank->base = base;
	bank->size = size;
	bank->num_sectors = num_sectors;

	for (int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	/* We've successfully determined the stats on this flash bank */
	cc3220sf_bank->probed = true;

	/* If we fall through to here, then all went well */

	return ERROR_OK;
}

static int cc3220sf_auto_probe(struct flash_bank *bank)
{
	struct cc3220sf_bank *cc3220sf_bank = bank->driver_priv;

	int retval = ERROR_OK;

	if (0 != bank->bank_number) {
		/* Invalid bank number somehow */
		return ERROR_FAIL;
	}

	if (!cc3220sf_bank->probed)
		retval = cc3220sf_probe(bank);

	return retval;
}

static int cc3220sf_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int cc3220sf_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed = 0;

	printed = snprintf(buf, buf_size, "CC3220SF with 1MB internal flash\n");

	buf += printed;
	buf_size -= printed;

	if (0 > buf_size)
		return ERROR_BUF_TOO_SMALL;

	return ERROR_OK;
}

struct flash_driver cc3220sf_flash = {
	.name = "cc3220sf",
	.flash_bank_command = cc3220sf_flash_bank_command,
	.erase = cc3220sf_erase,
	.protect = cc3220sf_protect,
	.write = cc3220sf_write,
	.read = default_flash_read,
	.probe = cc3220sf_probe,
	.auto_probe = cc3220sf_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cc3220sf_protect_check,
	.info = cc3220sf_info,
};
