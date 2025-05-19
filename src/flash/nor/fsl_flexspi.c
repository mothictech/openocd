// SPDX-License-Identifier: GPL-2.0-or-later


/***************************************************************************
 *   Copyright (C) 2022 by Brandon Martin and Mothic Technologies LLC      *
 *   martinbv@mothictech.com                                               *
 ***************************************************************************/

/*
 * Based on the U-Boot fsl-qspi driver (C) 2013-2020 Freescale/NXP and others
 * and stmqspi OpenOCD driver (C) 2016-2019 Andreas Bolsch / (C) 2010 Antonio
 * Borneo
 */

/*
 * The Freescale (NXP) FlexSPI controller found in the i.MXRT "Crossover" MCU
 * line is somewhat similar to the QSPI controller found in the larger i.MX
 * parts and is a typical controller focused on SPI memories providing
 * programmatic/transactional access to all of the connected device functions
 * as well as memory-mapped linear access to the memory.  Unlike some
 * controllers, writing to the device is supported in the linear memory-mapped
 * mode with some restrictions though this driver does not use this
 * functionality.
 *
 * In general, once the bank is probed, the connected memory device will be
 * available via memory-mapped linear access any time another operation is not
 * outstanding.
 *
 * Limitations of this driver:
 *
 * - Only supports a single attached memory device which must be on CS A1
 *   (which is mandatory for it to be the boot device, anyway).  Parallel mode
 *   combining two QSPI flash memories into a single 8-bit wide memory is not
 *   supported nor is combination mode for a single 8-bit wide device
 *   (octo-SPI).
 *
 * - Does not support DDR mode even if the attached flash device does.  Usually
 *   these devices also support SDR, so the only consequence is potentially
 *   slower access.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#include <helper/align.h>
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#include "fsl_flexspi.h"

#define TIMEOUT_EXEC_IPCMD			100
#define TIMEOUT_RESET				100
#define TIMEOUT_FLASH_ERASE_SECTOR	1000
#define TIMEOUT_FLASH_ERASE_CHIP	90000
#define TIMEOUT_FLASH_PROGRAM		250

/*
 * Macro for constructing the LUT entries with the following
 * register layout:
 *
 *  -----------------------------------------------------
 *  | OPCODE1 | PAD1 | OPRND1 | OPCODE0 | PAD0 | OPRND0 |
 *  -----------------------------------------------------
 */
#define LUT_DEF(idx, opc, pad, opr)					\
	((((opc) << 10) | ((pad) << 8) | (opr)) << (((idx) % 2) * 16))

struct fsl_flexspi_flash_bank {
	bool probed;
	struct flash_device dev;
	uint32_t io_base;
};

static int lut_pad(int x)
{
	switch (x) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 4:
		return 2;
	case 8:
		return 3;
	default:
		return -1;
	}
}


static int poll_reg(struct target *target, uint32_t value, uint32_t mask, uint32_t io_addr, unsigned int timeout)
{
	long long endtime;
	uint32_t regval;

	endtime = timeval_ms() + timeout;

	do {
		int ret;

		ret = target_read_u32(target, io_addr, &regval);
		if (ret != ERROR_OK)
			return ret;

		if ((regval & mask) == value) {
			LOG_DEBUG("Polled register at %08x done: regval=%08x", io_addr, regval);
			return ERROR_OK;
		}

		LOG_DEBUG("Polling register at %08x: regval=%08x", io_addr, regval);
		alive_sleep(1);
	} while (timeval_ms() < endtime);

	LOG_ERROR("Timeout while polling register at %08" PRIx32 ": val=%08" PRIx32
			", want=%08" PRIx32 "/%08" PRIx32, io_addr, regval, value, mask);

	return ERROR_TIMEOUT_REACHED;
}

static int read_rxfifo(struct flash_bank *bank, uint8_t *buf, uint32_t datalen)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t rx_fifo = flexspi_info->io_base + REG_RFDR;
	uint32_t io_base = flexspi_info->io_base;
	int ret;
	uint32_t nbytes;
	uint8_t bytes[4];

	LOG_DEBUG("want %" PRIx32 "B", datalen);

	ret = target_write_u32(target, io_base + REG_IPRXFCR, IPRXFCR_RXWMRK(datalen / 8 - 1));
	if (ret != ERROR_OK)
		goto err;

	nbytes = ALIGN_DOWN(datalen, 4);
	ret = target_read_memory(target, rx_fifo, 4, nbytes / 4, buf);

	if (nbytes < datalen) {
		ret = target_read_memory(target, rx_fifo + nbytes, 4, 1, bytes);
		memcpy(buf + nbytes, bytes, datalen - nbytes);
		if (ret != ERROR_OK)
			goto err;
	}

	ret = target_write_u32(target, io_base + REG_INTR, INTR_IPRXWA);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int fill_txfifo(struct flash_bank *bank, const uint8_t *buf, uint32_t datalen)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t tx_fifo = flexspi_info->io_base + REG_TFDR;
	uint32_t io_base = flexspi_info->io_base;
	int ret;
	uint32_t nbytes;
	uint8_t bytes[4] = {};

	ret = target_write_u32(target, io_base + REG_IPTXFCR, IPTXFCR_TXWMRK(datalen / 8 - 1));
	if (ret != ERROR_OK)
		goto err;

	ret = poll_reg(target, INTR_IPTXWE, INTR_IPTXWE, io_base + REG_INTR, TIMEOUT_EXEC_IPCMD);
	if (ret != ERROR_OK)
		goto err;

	nbytes = ALIGN_DOWN(datalen, 4);
	ret = target_write_memory(target, tx_fifo, 4, nbytes / 4, buf);
	if (ret != ERROR_OK)
		goto err;

	if (nbytes < datalen) {
		memcpy(&bytes, buf + nbytes, datalen - nbytes);
		ret = target_write_memory(target, tx_fifo + nbytes, 4, 1, bytes);
		if (ret != ERROR_OK)
			goto err;
	}

	ret = target_write_u32(target, io_base + REG_INTR, INTR_IPTXWE);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int write_lut_memory(struct flash_bank *bank, const uint32_t *lutval, uint8_t lutnum)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	int ret;

	LOG_DEBUG("LUT %02x/lutval[0:%08x, 1:%08x, 2:%08x, 3:%08x]",
		lutnum, lutval[0], lutval[1], lutval[2], lutval[3]);

	/* Unlock LUT memory */
	ret = target_write_u32(target, io_base + REG_LUTKEY, LUTKEY_MAGIC);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_LUTCR, LUTCR_UNLOCK);
	if (ret != ERROR_OK)
		goto err;

	/* Fill LUT sequence */
	for (int i = 0; i < 4; i++) {
		ret = target_write_u32(target, io_base + REG_LUT(lutnum * 4 + i), lutval[i]);
		if (ret != ERROR_OK)
			goto err;
	}

	/* Lock LUT memory */
	ret = target_write_u32(target, io_base + REG_LUTKEY, LUTKEY_MAGIC);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_LUTCR, LUTCR_LOCK);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int execute_ipcmd_read_with_lutnum(struct flash_bank *bank, uint8_t opcode,
		void *buf, uint32_t datalen, uint32_t lutnum)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	uint32_t lutval[4] = {};
	int lutidx = 0;
	uint32_t intr, sts1;
	int ret;

	LOG_DEBUG("with opcode 0x%02x", opcode);

	/* Prepare sequence LUT */
	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), opcode);
	lutidx++;

	if (datalen > 0) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_READ, lut_pad(1), datalen);
		lutidx++;
	}

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
	lutidx++;

	ret = write_lut_memory(bank, lutval, lutnum);
	if (ret != ERROR_OK)
		goto err;

	/* Reset FIFO states */
	ret = target_write_u32(target, io_base + REG_IPRXFCR, IPRXFCR_CLRIPRXF);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPTXFCR, IPTXFCR_CLRIPTXF);
	if (ret != ERROR_OK)
		goto err;

	/* Clear any stray pending status */
	ret = target_write_u32(target,
						   io_base + REG_INTR,
						   INTR_AHBCMDERR | INTR_IPCMDERR | INTR_AHBCMDGE | INTR_IPCMDGE);

	/* Execute the command */
	ret = target_write_u32(target, io_base + REG_IPCR0, IPCR0_SFAR(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_IPCR1,
						   IPCR1_IDATASZ(datalen) | IPCR1_ISEQID(lutnum) | IPCR1_ISEQNUM(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPCMD, IPCMD_TRG);
	if (ret != ERROR_OK)
		goto err;

	/* Wait for completion */
	ret = poll_reg(target, INTR_IPCMDDONE, INTR_IPCMDDONE, io_base + REG_INTR, TIMEOUT_EXEC_IPCMD);
	if (ret != ERROR_OK)
		goto err;

	/* Check for error */
	ret = target_read_u32(target, io_base + REG_INTR, &intr);
	if (ret != ERROR_OK)
		goto err;

	if (intr & INTR_IPCMDERR) {
		ret = target_read_u32(target, io_base + REG_STS1, &sts1);
		if (ret != ERROR_OK)
			goto err;

		LOG_ERROR("Error executing IP command. sts1=%08" PRIx32, sts1);
		ret = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Read result */
	if (datalen > 0) {
		ret = read_rxfifo(bank, buf, datalen);
		if (ret != ERROR_OK)
			goto err;
	}

	/* Clear status */
	ret = target_write_u32(target, io_base + REG_INTR, INTR_IPCMDDONE | INTR_IPRXWA | INTR_IPTXWE);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int execute_ipcmd_read(struct flash_bank *bank, uint8_t opcode,
		void *buf, uint32_t datalen)
{
	return execute_ipcmd_read_with_lutnum(
			bank, opcode, buf, datalen, LUTNUM_DRV);
}

static int execute_ipcmd_write(struct flash_bank *bank, uint8_t opcode,
		const void *buf, uint32_t datalen)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	uint32_t lutval[4] = {};
	int lutidx = 0;
	uint32_t intr, sts1;
	int ret;

	LOG_DEBUG("%s with opcode 0x%02x", __func__, opcode);

	/* Prepare sequence LUT */
	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), opcode);
	lutidx++;

	if (datalen > 0) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_WRITE, lut_pad(1), datalen);
		lutidx++;
	}

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
	lutidx++;

	ret = write_lut_memory(bank, lutval, LUTNUM_DRV);
	if (ret != ERROR_OK)
		goto err;

	/* Reset FIFO states */
	ret = target_write_u32(target, io_base + REG_IPRXFCR, IPRXFCR_CLRIPRXF);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPTXFCR, IPTXFCR_CLRIPTXF);
	if (ret != ERROR_OK)
		goto err;

	/* Clear any stray pending status */
	ret = target_write_u32(target,
						   io_base + REG_INTR,
						   INTR_AHBCMDERR | INTR_IPCMDERR | INTR_AHBCMDGE | INTR_IPCMDGE);

	/* Write data to FIFO */
	if (datalen > 0) {
		ret = fill_txfifo(bank, buf, datalen);
		if (ret != ERROR_OK)
			goto err;
	}

	/* Execute the command */
	ret = target_write_u32(target, io_base + REG_IPCR0, IPCR0_SFAR(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_IPCR1,
						   IPCR1_IDATASZ(datalen) | IPCR1_ISEQID(LUTNUM_DRV) | IPCR1_ISEQNUM(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPCMD, IPCMD_TRG);
	if (ret != ERROR_OK)
		goto err;

	/* Wait for completion */
	ret = poll_reg(target, INTR_IPCMDDONE, INTR_IPCMDDONE, io_base + REG_INTR, TIMEOUT_EXEC_IPCMD);
	if (ret != ERROR_OK)
		goto err;

	/* Check for error */
	ret = target_read_u32(target, io_base + REG_INTR, &intr);
	if (ret != ERROR_OK)
		goto err;

	if (intr & INTR_IPCMDERR) {
		ret = target_read_u32(target, io_base + REG_STS1, &sts1);
		if (ret != ERROR_OK)
			goto err;

		LOG_ERROR("Error executing IP command. sts1=%08" PRIx32, sts1);
		ret = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Clear status */
	ret = target_write_u32(target, io_base + REG_INTR, INTR_IPCMDDONE | INTR_IPRXWA | INTR_IPTXWE);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	int ret;

	LOG_DEBUG("reached");

	*id = 0;

	ret = execute_ipcmd_read(bank, SPIFLASH_READ_ID, id, sizeof(*id));
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int read_status_reg(struct flash_bank *bank, uint8_t *statusreg)
{
	int ret;

	LOG_DEBUG("reached");

	*statusreg = 0;

	ret = execute_ipcmd_read_with_lutnum(bank, SPIFLASH_READ_STATUS, statusreg, sizeof(*statusreg), LUTNUM_READ_STATUS);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int write_enable(struct flash_bank *bank)
{
	int ret;

	LOG_DEBUG("reached");

	ret = execute_ipcmd_read_with_lutnum(bank, SPIFLASH_WRITE_ENABLE, NULL, 0, LUTNUM_WRITE_ENABLE);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int poll_flash_status(struct flash_bank *bank, uint8_t value, uint8_t mask, int timeout)
{
	uint8_t status;
	int ret;
	long long endtime;

	LOG_DEBUG("reached");

	endtime = timeval_ms() + timeout;
	do {
		/* Read flash status register(s) */
		ret = read_status_reg(bank, &status);
		if (ret != ERROR_OK)
			return ret;

		if ((status & mask) == value)
			return ret;

		alive_sleep(25);
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_TIMEOUT_REACHED;
}

static int setup_write_lut(struct flash_bank *bank, uint8_t opcode,
		uint32_t writelen)
{
	LOG_DEBUG("opcode: %02x, writelen: %u", opcode, writelen);

	uint32_t lutval[4] = {};
	int lutidx = 0;

	/* Prepare sequence LUT */
	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), opcode);
	lutidx++;

	uint8_t naddrbytes = 1;
	if (bank->size > (1 << 8))
		naddrbytes++;
	if (bank->size > (1 << 16))
		naddrbytes++;
	if (bank->size > (1 << 24))
		naddrbytes++;

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_RADDR, lut_pad(1), naddrbytes * 8);
	lutidx++;

	if (writelen > 0) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_WRITE, lut_pad(1), writelen);
		lutidx++;
	}

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
	lutidx++;

	return write_lut_memory(bank, lutval, LUTNUM_DRV);
}

static int program_page(struct flash_bank *bank, uint8_t opcode,
		const void *buf, uint32_t flashaddr, uint32_t datalen)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	uint8_t status;
	int ret;

	LOG_DEBUG("%" PRIu32 "B to 0x%08" PRIx32, datalen, flashaddr);

	ret = write_enable(bank);
	if (ret != ERROR_OK)
		goto err;

	ret = setup_write_lut(bank, opcode, datalen);
	if(ret != ERROR_OK)
		return ret;

	/* Clear any stray pending status */
	ret = target_write_u32(target,
						   io_base + REG_INTR,
						   INTR_AHBCMDERR | INTR_IPCMDERR | INTR_AHBCMDGE | INTR_IPCMDGE);

	/* Reset FIFO states */
	ret = target_write_u32(target, io_base + REG_IPRXFCR, IPRXFCR_CLRIPRXF);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPTXFCR, IPTXFCR_CLRIPTXF);
	if (ret != ERROR_OK)
		goto err;

	/* Fill the TX FIFO with the data */
	ret = fill_txfifo(bank, buf, datalen);
	if (ret != ERROR_OK)
		goto err;

	/* Execute the command */
	ret = target_write_u32(target, io_base + REG_IPCR0, IPCR0_SFAR(flashaddr));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_IPCR1,
						   IPCR1_IDATASZ(datalen) | IPCR1_ISEQID(LUTNUM_DRV) | IPCR1_ISEQNUM(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPCMD, IPCMD_TRG);
	if (ret != ERROR_OK)
		goto err;

	/* Wait for controller completion */
	ret = poll_reg(target, INTR_IPCMDDONE, INTR_IPCMDDONE, io_base + REG_INTR, TIMEOUT_EXEC_IPCMD);
	if (ret != ERROR_OK)
		goto err;

	/*
	 * Check status register to see if the flash device appears to have
	 * accepted the command and is now executing it.  If BSY and WE are clear,
	 * we assume program already completed.  If BSY is clear but WE is still
	 * set, the command wasn't accepted.
	 */
	ret = read_status_reg(bank, &status);
	if (ret != ERROR_OK)
		goto err;

	if (((status & SPIFLASH_BSY_BIT) == 0) && ((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Page program command not accepted by flash. status=0x%02x", status);
		ret = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Poll flash device for end of internally timed Page Program operation */
	ret = poll_flash_status(bank, 0, SPIFLASH_BSY_BIT, TIMEOUT_FLASH_PROGRAM);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

/* Kinetis Program-LongWord Microcodes */
static const uint8_t flexspi_flash_blockwrite_algo[] = {
#include "../../../contrib/loaders/flash/fsl_flexspi_blockwrite.inc"
};

static int program_block(struct flash_bank *bank, uint8_t opcode,
		const void *buf, uint32_t flashaddr, uint32_t datalen, uint32_t blksize)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t progdata_size;
	struct working_area *algo;
	struct working_area *progdata;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	uint32_t end_address;
	uint32_t blkcount;
	int ret;
	uint8_t statusreg;

	/* Set up driver LUT for block write */
	ret = setup_write_lut(bank, opcode, blksize);
	if(ret != ERROR_OK)
		return ret;

	/* Run an extra read status command to ensure LUTNUM_READ_STATUS is set up */
	ret = read_status_reg(bank, &statusreg);
	if (ret != ERROR_OK)
		return ret;

	/* Likewise for write enable */
	ret = write_enable(bank);
	if (ret != ERROR_OK)
		return ret;

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(flexspi_flash_blockwrite_algo),
			&algo) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto out_noalgo;
	}

	ret = target_write_buffer(target, algo->address,
		sizeof(flexspi_flash_blockwrite_algo), flexspi_flash_blockwrite_algo);
	if (ret != ERROR_OK)
		goto out_noprogdata;

	/* memory buffer, size must be (multiple of blksize) plus 8 */
	progdata_size = target_get_working_area_avail(target) & ~(sizeof(uint32_t) - 1);
	if (progdata_size < (blksize * 2 + 8)) {
		LOG_WARNING("large enough working area not available for block writes");
		ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto out_noprogdata;
	}

	/* Anything bigger than 32k seems superfluous */
	progdata_size = MIN(progdata_size, 32768 + 8);

	/* Ensure it's a multiple of the blksize plus the extra 8 bytes for the rp/wp */
	progdata_size = ALIGN_DOWN(progdata_size, blksize) + 8;

	if (target_alloc_working_area(target, progdata_size, &progdata) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		ret = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto out_noprogdata;
	}

	blkcount = datalen / blksize;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* flash address */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* block count */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* block length */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* progdata start */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* progdata end */
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);		/* FLEXSPI base */

	buf_set_u32(reg_params[0].value, 0, 32, flashaddr);
	buf_set_u32(reg_params[1].value, 0, 32, blkcount);
	buf_set_u32(reg_params[2].value, 0, 32, blksize / sizeof(uint32_t));
	buf_set_u32(reg_params[3].value, 0, 32, progdata->address);
	buf_set_u32(reg_params[4].value, 0, 32,
			progdata->address + progdata->size);
	buf_set_u32(reg_params[5].value, 0, 32, flexspi_info->io_base);

	ret = target_run_flash_async_algorithm(target, buf, blkcount, blksize,
						0, NULL, ARRAY_SIZE(reg_params), reg_params,
						progdata->address, progdata->size,
						algo->address, 0, &armv7m_info);

	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		end_address = buf_get_u32(reg_params[0].value, 0, 32);

		LOG_ERROR("Error writing flash at %08" PRIx32, end_address);
	} else if (ret != ERROR_OK) {
		LOG_ERROR("Error executing flash block programming algorithm");
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	target_free_working_area(target, progdata);
out_noprogdata:
	target_free_working_area(target, algo);
out_noalgo:

	return ret;
}


static int erase_sector(struct flash_bank *bank, unsigned int sector)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	uint32_t flashaddr = bank->sectors[sector].offset;
	uint32_t lutval[4] = {};
	int lutidx = 0;
	uint8_t status;
	int ret;

	LOG_DEBUG("number %u", sector);

	ret = write_enable(bank);
	if (ret != ERROR_OK)
		goto err;

	/* Prepare sequence LUT for Erase Sector */
	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), flexspi_info->dev.erase_cmd);
	lutidx++;

	uint8_t naddrbytes = 1;
	if (bank->size > (1 << 8))
		naddrbytes++;
	if (bank->size > (1 << 16))
		naddrbytes++;
	if (bank->size > (1 << 24))
		naddrbytes++;

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_RADDR, lut_pad(1), naddrbytes * 8);
	lutidx++;

	lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
	lutidx++;

	ret = write_lut_memory(bank, lutval, LUTNUM_DRV);
	if (ret != ERROR_OK)
		goto err;

	/* Execute the command */
	ret = target_write_u32(target, io_base + REG_IPCR0, IPCR0_SFAR(flashaddr));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_IPCR1,
						   IPCR1_IDATASZ(0) | IPCR1_ISEQID(LUTNUM_DRV) | IPCR1_ISEQNUM(0));
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_IPCMD, IPCMD_TRG);
	if (ret != ERROR_OK)
		goto err;

	/* Wait for completion of FLEXSPI command cycle */
	ret = poll_reg(target, INTR_IPCMDDONE, INTR_IPCMDDONE, io_base + REG_INTR, TIMEOUT_EXEC_IPCMD);
	if (ret != ERROR_OK)
		goto err;

	/*
	 * Check status register to see if the flash device appears to have
	 * accepted the command and is now executing it.  If BSY and WE are clear,
	 * we assume erase already completed.  If BSY is clear but WE is still set,
	 * the command wasn't accepted.
	 */
	ret = read_status_reg(bank, &status);
	if (ret != ERROR_OK)
		goto err;

	if (((status & SPIFLASH_BSY_BIT) == 0) && ((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Sector erase command not accepted by flash. status=0x%02x", status);
		ret = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	/* Erase takes a long time, so some sort of progress message is a good idea */
	LOG_DEBUG("erasing sector %4u", sector);

	/* Poll the flash device for end of internally timed Sector Erase operation */
	ret = poll_flash_status(bank, 0, SPIFLASH_BSY_BIT, TIMEOUT_FLASH_ERASE_SECTOR);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

static int default_setup(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	uint32_t io_base = flexspi_info->io_base;
	int ret;
	uint32_t mcr0;

	LOG_DEBUG("reached");

	/*
	 * There are a bunch of non-zero default/reserved values that are to be
	 * preserved in MCR0
	 */
	ret = target_read_u32(target, io_base + REG_MCR0, &mcr0);
	if (ret != ERROR_OK)
		goto err;

	/* Reset the module */
	ret = target_write_u32(target, io_base + REG_MCR0, (mcr0 | MCR0_SWRESET) & (~MCR0_MDIS));
	if (ret != ERROR_OK)
		goto err;

	ret = poll_reg(target, 0, MCR0_SWRESET, io_base + REG_MCR0, TIMEOUT_RESET);
	if (ret != ERROR_OK)
		goto err;

	/* Disable the module during configuration */
	ret = target_write_u32(target, io_base + REG_MCR0, mcr0 | MCR0_MDIS);
	if (ret != ERROR_OK)
		goto err;

	/*
	 * AHB buffer 3 is the fallback buffer - configure it and deactivate
	 * all the others by setting their size to 0.
	 */
	ret = target_write_u32(target, io_base + REG_AHBRXBUF0CR0, 0);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_AHBRXBUF1CR0, 0);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_AHBRXBUF2CR0, 0);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_AHBRXBUF3CR0,
						   AHBRXBUFNCR0_BUFSZ(FLEXSPI_AHB_RX_BUFSZ / 8) |
								AHBRXBUFNCR0_REGIONEN |
								AHBRXBUFNCR0_PREFETCHEN);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target,
						   io_base + REG_AHBCR,
						   AHBCR_PREFETCHEN |
								AHBCR_BUFFERABLEEN |
								AHBCR_CACHABLEEN |
								AHBCR_READADDROPT);
	if (ret != ERROR_OK)
		goto err;

	/* Set up Flash A1 as non-zero size so we can probe it */
	ret = target_write_u32(target, io_base + REG_FLSHA1CR0, FLSHNNCR0_FLASHSZ(64));
	if (ret != ERROR_OK)
		goto err;

	/* Enable the module and set FIFO access to IPB */
	ret = target_write_u32(target, io_base + REG_MCR0, (mcr0 & ~(MCR0_MDIS | MCR0_ARDFEN | MCR0_ATDFEN)));
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

COMMAND_HANDLER(fsl_flexspi_handle_mass_erase_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct fsl_flexspi_flash_bank *flexspi_info;
	unsigned int sector;
	uint8_t status;
	long long starttime;
	int ret;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ret = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ret != ERROR_OK)
		goto err;

	flexspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		ret = ERROR_TARGET_NOT_HALTED;
		goto err;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		ret = ERROR_FLASH_BANK_NOT_PROBED;
		goto err;
	}

	if (flexspi_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
		goto err;
	}

	for (sector = 0; sector < bank->num_sectors; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			ret = ERROR_FLASH_PROTECTED;
			goto err;
		}
	}

	ret = write_enable(bank);
	if (ret != ERROR_OK)
		goto err;

	starttime = timeval_ms();

	ret = execute_ipcmd_read(bank, flexspi_info->dev.chip_erase_cmd, NULL, 0);
	if (ret != ERROR_OK)
		goto err;

	/*
	 * Check status register to see if the flash device appears to have
	 * accepted the command and is now executing it.  If BSY and WE are clear,
	 * we assume erase already completed.  If BSY is clear but WE is still set,
	 * the command wasn't accepted.
	 */
	ret = read_status_reg(bank, &status);
	if (ret != ERROR_OK)
		goto err;

	if (((status & SPIFLASH_BSY_BIT) == 0) && ((status & SPIFLASH_WE_BIT) != 0)) {
		LOG_ERROR("Chip erase command not accepted by flash. status=0x%02x", status);
		ret = ERROR_FLASH_OPERATION_FAILED;
		goto err;
	}

	LOG_INFO("Waiting for chip erase to finish");

	ret = poll_flash_status(bank, 0, SPIFLASH_BSY_BIT, TIMEOUT_FLASH_ERASE_CHIP);
	if (ret != ERROR_OK)
		goto err;

	LOG_INFO("Chip erase completed in %lldms", timeval_ms() - starttime);

	return ERROR_OK;

err:
	return ret;
}

COMMAND_HANDLER(fsl_flexspi_handle_exec_flashcmd_write_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct fsl_flexspi_flash_bank *flexspi_info;
	int ret;
	uint8_t flashcmd;
	uint8_t wrdata[32];
	int nwrdata = CMD_ARGC - 2;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 1 + 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC > 1 + 1 + 32) {
		LOG_ERROR("Too much data");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	ret = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ret != ERROR_OK)
		goto err;

	flexspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		ret = ERROR_TARGET_NOT_HALTED;
		goto err;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		ret = ERROR_FLASH_BANK_NOT_PROBED;
		goto err;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], flashcmd);

	for (int i = 0; i < nwrdata; i++)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i + 2], wrdata[i]);

	ret = execute_ipcmd_write(bank, flashcmd, wrdata, nwrdata);
	if (ret != ERROR_OK)
		goto err;

	return ERROR_OK;

err:
	return ret;
}

COMMAND_HANDLER(fsl_flexspi_handle_exec_flashcmd_read_command)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct fsl_flexspi_flash_bank *flexspi_info;
	int ret;
	uint8_t flashcmd;
	uint32_t nrddata;
	uint8_t *rddata;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ret = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ret != ERROR_OK)
		goto err;

	flexspi_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		ret = ERROR_TARGET_NOT_HALTED;
		goto err;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		ret = ERROR_FLASH_BANK_NOT_PROBED;
		goto err;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], flashcmd);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], nrddata);

	rddata = calloc(1, nrddata);

	if (!rddata) {
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto err;
	}

	ret = execute_ipcmd_read(bank, flashcmd, rddata, nrddata);
	if (ret != ERROR_OK)
		goto err_dealloc;

	command_print(CMD, "Read data (hex):");
	for (uint32_t i = 0; i < nrddata; i++)
		command_print_sameline(CMD, " %02x", rddata[i]);

	free(rddata);

	return ERROR_OK;

err_dealloc:
	free(rddata);
err:
	return ret;
}

static int fsl_flexspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	unsigned int sector;
	int ret;

	LOG_DEBUG("from sector %u to sector %u", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (flexspi_info->dev.erase_cmd == 0x00) {
		LOG_ERROR("Sector erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (last < first || last >= bank->num_sectors) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	for (sector = first; sector <= last; sector++) {
		ret = erase_sector(bank, sector);
		if (ret != ERROR_OK)
			break;
		alive_sleep(10);
		keep_alive();
	}

	if (ret != ERROR_OK)
		LOG_ERROR("Flash sector_erase failed on sector %u", sector);

	return ret;
}

static int fsl_flexspi_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	unsigned int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	if (set)
		LOG_WARNING("setting soft protection only, not related to flash's hardware write protection");

	return ERROR_OK;
}

static int fsl_flexspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	unsigned int sector;
	int ret;

	LOG_DEBUG("offset=0x%08" PRIx32 " count=0x%08" PRIx32, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Write beyond end of flash. Extra data discarded.");
		count = bank->size - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size)) &&
			((offset + count - 1) >= bank->sectors[sector].offset) &&
			bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	/* Write the lesser of one page or the TX FIFO size at a time */
	uint32_t wrsize = (flexspi_info->dev.pagesize < FLEXSPI_IP_TX_FIFOSZ) ?
		flexspi_info->dev.pagesize :
		FLEXSPI_IP_TX_FIFOSZ;

	/* Attempt to program the entire block using on-chip algorithm */
	uint32_t blockwrlen = ALIGN_DOWN(count, wrsize);
	LOG_DEBUG("Block program %" PRIu32 "B at 0x%08" PRIx32,blockwrlen, offset);
	ret = program_block(bank, flexspi_info->dev.pprog_cmd, buffer, offset,
			blockwrlen, wrsize);
	if (ret == ERROR_OK) {
		count -= blockwrlen;
		offset += blockwrlen;
		buffer += blockwrlen;
	} else {
		LOG_WARNING("Can't use block program; falling back to page program");
		return ret;
	}

	/* Program by page whatever we couldn't program as a block */
	while (count > 0) {
		if (count < wrsize)
			wrsize = count;

		ret = program_page(bank, flexspi_info->dev.pprog_cmd, buffer, offset,
				wrsize);
		if (ret != ERROR_OK)
			goto err;

		count -= wrsize;
		offset += wrsize;
		buffer += wrsize;
	}

	return ERROR_OK;

err:
	return ret;
}

static int fsl_flexspi_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;

	LOG_DEBUG("offset=0x%08" PRIx32 " count=0x%08" PRIx32, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(flexspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > bank->size) {
		LOG_WARNING("Read beyond end of flash. Extra data to be ignored.");
		count = bank->size - offset;
	}

	/* Simply read from the linear AHB port */
	return target_read_buffer(target, bank->base + offset, count, buffer);
}

static int fsl_flexspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;
	struct flash_sector *sectors = NULL;
	uint32_t io_base = flexspi_info->io_base;
	uint32_t lutkey;
	uint32_t id;
	uint32_t lutval[4] = {};
	int lutidx = 0;
	int ret;

	LOG_DEBUG("reached");

	if (flexspi_info->probed) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	flexspi_info->probed = false;

	ret = target_read_u32(target, io_base + REG_LUTKEY, &lutkey);
	if (ret == ERROR_OK && lutkey == LUTKEY_MAGIC) {
		LOG_DEBUG("Found FLEXSPI at 0x%08" PRIx64 " with io_base 0x%08" PRIx32,
			bank->base, io_base);
	} else {
		LOG_ERROR("Did not find FLEXSPI at io_base 0x%08" PRIx32 " (LUTKEY=0x%08" PRIx32 ")",
			io_base, lutkey);
		return ERROR_FAIL;
	}

	/* Put the controller into a sane/default configuration */
	ret = default_setup(bank);
	if (ret != ERROR_OK)
		goto err;

	/* Attempt to read the ID of the device on CS A1 */
	ret = read_flash_id(bank, &id);
	if (ret != ERROR_OK)
		goto err;

	LOG_DEBUG("CS A1 flash id %06" PRIx32, id);

	/* Match with OpenOCD's list of known SPI flash memories */
	for (const struct flash_device *p = flash_devices; id && p->name ; p++) {
		if (p->device_id == id) {
			memcpy(&flexspi_info->dev, p, sizeof(flexspi_info->dev));
			if (p->size_in_bytes / 4096)
				LOG_INFO("flash A1 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					" KiB", p->name, id, p->size_in_bytes / 1024);
			else
				LOG_INFO("flash A1 \'%s\' id = 0x%06" PRIx32 " size = %" PRIu32
					" B", p->name, id, p->size_in_bytes);
			break;
		}
	}

	if (!flexspi_info->dev.name) {
		/* Chip could not be identified by ID */
		LOG_WARNING("Unknown flash A1 device id = 0x%06" PRIx32, id);
		goto err;
	}

	/* Configure flash size */
	bank->size = flexspi_info->dev.size_in_bytes;

	ret = target_write_u32(target, io_base + REG_FLSHA1CR0, FLSHNNCR0_FLASHSZ(flexspi_info->dev.size_in_bytes / 1024));
	if (ret != ERROR_OK)
		goto err;

	/* Configure the AHB read LUT */
	if (flexspi_info->dev.qread_cmd) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), flexspi_info->dev.qread_cmd);
		lutidx++;

		uint8_t naddrbytes = 1;
		if (bank->size > (1 << 8))
			naddrbytes++;
		if (bank->size > (1 << 16))
			naddrbytes++;
		if (bank->size > (1 << 24))
			naddrbytes++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_RADDR, lut_pad(4), naddrbytes * 8);
		lutidx++;

		LOG_INFO("Assuming 2 mode cycles (0xF0) + 4 dummy cycles for qread cmd 0x%02X", flexspi_info->dev.qread_cmd);

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_MODE8, lut_pad(4), 0xF0);
		lutidx++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_DUMMY, lut_pad(4), 4);
		lutidx++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_READ, lut_pad(4), 1);
		lutidx++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
		lutidx++;
	} else if (flexspi_info->dev.read_cmd) {
		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_CMD, lut_pad(1), flexspi_info->dev.read_cmd);
		lutidx++;

		uint8_t naddrbytes = 1;
		if (bank->size > (1 << 8))
			naddrbytes++;
		if (bank->size > (1 << 16))
			naddrbytes++;
		if (bank->size > (1 << 24))
			naddrbytes++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_RADDR, lut_pad(1), naddrbytes * 8);
		lutidx++;

		LOG_INFO("Assuming no dummy cycles for single-io read cmd 0x%02X", flexspi_info->dev.read_cmd);

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_READ, lut_pad(1), 1);
		lutidx++;

		lutval[lutidx / 2] |= LUT_DEF(lutidx, OPCODE_STOP, 0, 0);
		lutidx++;
	}

	ret = write_lut_memory(bank, lutval, LUTNUM_AHB_READ);
	if (ret != ERROR_OK)
		goto err;

	ret = target_write_u32(target, io_base + REG_FLSHA1CR2,
			FLSHNNCR2_CLRINSTRPTR | FLSHNNCR2_ARDSEQNUM(0) | FLSHNNCR2_ARDSEQID(LUTNUM_AHB_READ));
	if (ret != ERROR_OK)
		goto err;

	/* Create and fill sectors array */
	bank->num_sectors = flexspi_info->dev.size_in_bytes / flexspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto err;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * flexspi_info->dev.sectorsize;
		sectors[sector].size = flexspi_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	flexspi_info->probed = true;

	return ERROR_OK;

err:
	LOG_ERROR("Error communicating to target FLEXSPI module: %d", ret);
	return ret;
}


static int fsl_flexspi_auto_probe(struct flash_bank *bank)
{
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;

	if (flexspi_info->probed)
		return ERROR_OK;

	return fsl_flexspi_probe(bank);
}

static int fsl_flexspi_protect_check(struct flash_bank *bank)
{
	/* Not relevant - controller has no protection features */
	return ERROR_OK;
}

static int fsl_flexspi_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct fsl_flexspi_flash_bank *flexspi_info = bank->driver_priv;

	if (!(flexspi_info->probed)) {
		command_print_sameline(cmd, "\nFLEXSPI flash bank not probed yet\n");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print_sameline(cmd, "flash A1 \'%s\', device id = 0x%06" PRIx32
			", flash size = %" PRIu32 "%sB\n(page size = %" PRIu32
			", read = 0x%02" PRIx8 ", qread = 0x%02" PRIx8
			", pprog = 0x%02" PRIx8 ", mass_erase = 0x%02" PRIx8
			", sector size = %" PRIu32 " %sB, sector_erase = 0x%02" PRIx8 ")",
			flexspi_info->dev.name, flexspi_info->dev.device_id,
			bank->size / 4096 ? bank->size / 1024 : bank->size,
			bank->size / 4096 ? "Ki" : "", flexspi_info->dev.pagesize,
			flexspi_info->dev.read_cmd, flexspi_info->dev.qread_cmd,
			flexspi_info->dev.pprog_cmd, flexspi_info->dev.chip_erase_cmd,
			flexspi_info->dev.sectorsize / 4096 ?
				flexspi_info->dev.sectorsize / 1024 : flexspi_info->dev.sectorsize,
			flexspi_info->dev.sectorsize / 4096 ? "Ki" : "",
			flexspi_info->dev.erase_cmd);

	return ERROR_OK;
}


FLASH_BANK_COMMAND_HANDLER(fsl_flexspi_flash_bank_command)
{
	struct fsl_flexspi_flash_bank *flexspi_info;

	LOG_DEBUG("reached");

	flexspi_info = calloc(1, sizeof(struct fsl_flexspi_flash_bank));
	if (!flexspi_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = flexspi_info;
	flexspi_info->io_base = FLEXSPI_DEFAULT_IOBASE;

	return ERROR_OK;
}

static const struct command_registration fsl_flexspi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = fsl_flexspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "exec_flashcmd_write",
		.handler = fsl_flexspi_handle_exec_flashcmd_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id cmdbyte [databytes...]",
		.help = "Execute an arbitrary flash command with written optional arguments",
	},
	{
		.name = "exec_flashcmd_read",
		.handler = fsl_flexspi_handle_exec_flashcmd_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id cmdbyte numbytes",
		.help = "Execute an arbitrary flash command with subsequent data read",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration fsl_flexspi_command_handlers[] = {
	{
		.name = "fsl_flexspi",
		.mode = COMMAND_ANY,
		.help = "fsl_flexspi flash command group",
		.usage = "",
		.chain = fsl_flexspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver fsl_flexspi_flash = {
	.name = "fsl_flexspi",
	.commands = fsl_flexspi_command_handlers,
	.flash_bank_command = fsl_flexspi_flash_bank_command,
	.erase = fsl_flexspi_erase,
	.protect = fsl_flexspi_protect,
	.write = fsl_flexspi_write,
	.read = fsl_flexspi_read,
	.probe = fsl_flexspi_probe,
	.auto_probe = fsl_flexspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = fsl_flexspi_protect_check,
	.info = fsl_flexspi_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
