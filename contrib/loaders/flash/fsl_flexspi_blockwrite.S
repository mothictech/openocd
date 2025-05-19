/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by Brandon Martin (Mothic Technologies LLC)        *
 *   martinbv@mothictech.com                                               *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m7
	.thumb
	.thumb_func

	/* 
	 * Params:
	 * r0 = flash destination address [in/out]
	 * r1 = blkcount - 0 indicates success [in/out]
	 * r2 = blksize in 32-bit words (must be <= IP TX FIFO length)
	 * r3 = workarea (streaming FIFO) start address
	 * r4 = workarea (streaming FIFO) end address
	 * r5 = FLEXSPI register base
	 */

#include "../../../src/flash/nor/fsl_flexspi.h"

	.macro	wait_ipcmd
0:
	ldr	r7, [r5, #REG_INTR]
	ands	r6, r7, #INTR_IPCMDERR
	bne	error
	ands	r6, r7, #INTR_IPCMDDONE
	beq	0b
	.endm

	.macro clear_intr
	mov	r6, #(INTR_AHBCMDERR | INTR_IPCMDERR | INTR_AHBCMDGE | INTR_IPCMDGE | INTR_IPCMDDONE | INTR_IPRXWA | INTR_IPTXWE)
	str	r6, [r5, #REG_INTR]
	.endm

	.align	4

start:
	cpsid	i
	mov	r6, #0
	ldr	r7, wdog1
	strh	r6, [r7, #8]
	mov	r6, #0x0030
	strh	r6, [r7, #0]
	ldr	r7, wdog2
	mov	r6, #0
	strh	r6, [r7, #8]
	mov	r6, #0x0030
	strh	r6, [r7, #0]
	ldr	r7, rtwdog
	ldr	r6, [r7]
	ands	r6, #BIT(7)
	beq	rtwdog_disabled
	ldr	r6, rtwdog_unlock
	str	r6, [r7, #4]
	mov	r6, #0x00002120
	str	r6, [r7]
rtwdog_disabled:
	ldr	r7, src
	ldr	r6, src_scr_disable_wdogs
	str	r6, [r7]
	mov	r6, #0x1ff
	str	r6, [r7, #8]
	ldr	r7, systick
	mov	r6, #0
	str	r6, [r7]

next_block:
	/* Write enable */
	clear_intr
	str	r0, [r5, #REG_IPCR0]	/* flash address */
	mov	r6, #(IPCR1_IDATASZ(0) | IPCR1_ISEQID(LUTNUM_WRITE_ENABLE) | IPCR1_ISEQNUM(0))
	str	r6, [r5, #REG_IPCR1]	/* data length and LUT program info */
	mov	r6, #IPCMD_TRG
	str	r6, [r5, #REG_IPCMD]	/* initiate command */
	wait_ipcmd

	/* Configure the TX FIFO for a block */
	lsr	r6, r2, #1
	sub	r6, r6, #1
	lsl	r6, r6, #2
	orr	r6, r6, #1
	str	r6, [r5, #REG_IPTXFCR]

	/* Load TX FIFO with this block's data */
	mov	r8, #0			/* block word count [r8] */
	add	r5, r5, #REG_TFDR
	ldr 	r6, [r3, #4]		/* read rp [r6] */
wait_fifo:
	ldr 	r7, [r3, #0]		/* read wp [r7] */
	cmp 	r7, #0			/* abort if wp == 0 */
	beq 	exit

	cmp 	r6, r7			/* wait until rp != wp */
	beq 	wait_fifo

	ldr	r7, [r6]		/* read from rp */
	str	r7, [r5, r8, lsl #2]	/* store to IP TX FIFO */

	add	r6, #4			/* rp += 4 */
	cmp     r6, r4			/* Wrap? */
	bcc     no_wrap
	mov	r6, r3
	add   	r6, r6, #8
no_wrap:
	add	r8, r8, #1
	cmp	r2, r8			/* need more data for block? */
	bne	wait_fifo

	/* Entire block written to TX FIFO */
	str     r6, [r3, #4]		/* Store rp */
	sub	r5, r5, #REG_TFDR

	/* Send write command + data */
	clear_intr
	str	r0, [r5, #REG_IPCR0]	/* flash address */
	mov	r6, #(IPCR1_ISEQID(LUTNUM_DRV) | IPCR1_ISEQNUM(0))
	orr	r6, r6, r2, lsl #2
	str	r6, [r5, #REG_IPCR1]	/* data length and LUT program info */
	mov	r6, #IPCMD_TRG
	str	r6, [r5, #REG_IPCMD]	/* initiate command */
	wait_ipcmd

	/* Read status and wait for completion */
check_busy:
	clear_intr
	mov	r6, #IPRXFCR_RXWMRK(1) | IPRXFCR_CLRIPRXF
	str	r6, [r5, #REG_IPRXFCR]
	mov	r6, #0
	str	r6, [r5, #REG_IPCR0]
	ldr	r6, wait_busy_ipcr1
	str	r6, [r5, #REG_IPCR1]
	mov	r6, #IPCMD_TRG
	str	r6, [r5, #REG_IPCMD]	/* initiate command */

wait_status:
	wait_ipcmd

	ldr	r7, [r5, #REG_RFDR]
	ands	r6, r7, #SPIFLASH_BSY_BIT
	bne	check_busy
	ands	r6, r7, #SPIFLASH_WE_BIT
	bne	error

	/* Block is complete */
	add	r0, r0, r2, lsl #2	/* flash address += blksize - increment after error check since it's output */

	subs	r1, #1			/* blkcount-- */
	bne	next_block
	b	exit

error:
	mov	r5, #0
	str     r5, [r3, #4]		/* set rp = 0 on error */

exit:
	bkpt    #0

wait_busy_ipcr1:
	.word	(IPCR1_IDATASZ(4) | IPCR1_ISEQID(LUTNUM_READ_STATUS) | IPCR1_ISEQNUM(0))
wdog1:
	.word	0x400b8000
wdog2:
	.word	0x400d0000
rtwdog:
	.word	0x400bc000
rtwdog_unlock:
	.word	0xd928c520
systick:
	.word	0xe000e010
src:
	.word	0x400f8000
src_scr_disable_wdogs:
	.word	0x504802a0
