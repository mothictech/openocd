/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Brandon Martin and Mothic Technologies LLC      *
 *   martinbv@mothictech.com                                               *
 ***************************************************************************/

#include "spi.h"

/* Make the BIT macro available from assembly */
#ifndef BIT
# define BIT(x)						(1 << (x))
#endif

/* These are the base addresses used on the IMXRT1020 */
#define FLEXSPI_DEFAULT_IOBASE		0x402A8000
#define FLEXSPI_LINEAR_AHBASE		0x60000000
#define FLEXSPI_TX_FIFO_AHBASE		0x7F800000
#define FLEXSPI_RX_FIFO_AHBASE		0x7FC00000

/* The documentation implies that these may be chip-specific tweakables */
#define FLEXSPI_AHB_TX_BUFSZ		64
#define FLEXSPI_AHB_RX_BUFSZ		1024
#define FLEXSPI_IP_TX_FIFOSZ		128
#define FLEXSPI_IP_RX_FIFOSZ		128

/* On-chip registers and fields */
#define REG_MCR0					0x00
#define MCR0_SWRESET				BIT(0)
#define MCR0_MDIS					BIT(1)
#define MCR0_ARDFEN					BIT(6)
#define MCR0_ATDFEN					BIT(7)

#define REG_AHBCR					0x0C
#define AHBCR_ALIGNMENT(x)			(((x) & 0x03) << 20)
#define AHBCR_READSZALIGN			BIT(10)
#define AHBCR_READADDROPT			BIT(6)
#define AHBCR_PREFETCHEN			BIT(5)
#define AHBCR_BUFFERABLEEN			BIT(4)
#define AHBCR_CACHABLEEN			BIT(3)
#define AHBCR_APAREN				BIT(0)

#define REG_INTR					0x14
#define INTR_IPCMDDONE				BIT(0)
#define INTR_IPCMDGE				BIT(1)
#define INTR_AHBCMDGE				BIT(2)
#define INTR_IPCMDERR				BIT(3)
#define INTR_AHBCMDERR				BIT(4)
#define INTR_IPRXWA					BIT(5)
#define INTR_IPTXWE					BIT(6)
#define INTR_SCKSTOPBYRD			BIT(8)
#define INTR_SCKSTOPBYWR			BIT(9)
#define INTR_AHBBUSTTIMEOUT			BIT(10)
#define INTR_SEQTIMEOUT				BIT(11)
#define INTR_IPCMDSECUREVIO			BIT(16)

#define REG_LUTKEY					0x18
#define LUTKEY_MAGIC				0x5AF05AF0

#define REG_LUTCR					0x1C
#define LUTCR_LOCK					BIT(0)
#define LUTCR_UNLOCK				BIT(1)
#define LUTCR_PROTECT				BIT(2)

#define REG_AHBRXBUF0CR0			0x20
#define REG_AHBRXBUF1CR0			0x24
#define REG_AHBRXBUF2CR0			0x28
#define REG_AHBRXBUF3CR0			0x2C
#define AHBRXBUFNCR0_BUFSZ(x)		(((x) & 0xFF) << 0)
#define AHBRXBUFNCR0_MSTRID(x)		(((x) & 0x0F) << 16)
#define AHBRXBUFNCR0_PRIORITY(x)	(((x) & 0x03) << 24)
#define AHBRXBUFNCR0_REGIONEN		BIT(30)
#define AHBRXBUFNCR0_PREFETCHEN		BIT(31)

#define REG_FLSHA1CR0				0x60
#define FLSHNNCR0_FLASHSZ(x)		(((x) & 0x7FFFFF) << 0)

#define REG_FLSHA1CR1				0x70
#define FLSHNNCR1_TCSS(x)			(((x) & 0x1F) << 0)
#define FLSHNNCR1_TCSH(x)			(((x) & 0x1F) << 5)
#define FLSHNNCR1_WA				BIT(10)
#define FLSHNNCR1_CAS(x)			(((x) & 0x0F) << 11)
#define FLSHNNCR1_CSINTERVALUNIT	BIT(15)
#define FLSHNNCR1_CSINTERVAL(x)		(((x) & 0xFFFF) << 16)

#define REG_FLSHA1CR2				0x80
#define FLSHNNCR2_ARDSEQID(x)		(((x) & 0x0F) << 0)
#define FLSHNNCR2_ARDSEQNUM(x)		(((x) & 0x07) << 5)
#define FLSHNNCR2_AWRSEQID(x)		(((x) & 0x0F) << 8)
#define FLSHNNCR2_AWRSEQNUM(x)		(((x) & 0x07) << 13)
#define FLSHNNCR2_AWRWAIT(x)		(((x) & 0x0FFF) << 16)
#define FLSHNNCR2_AWRWAITUNIT(x)	(((x) & 0x07) << 28)
#define AWRWAITUNIT_2AHB			0
#define AWRWAITUNIT_8AHB			1
#define AWRWAITUNIT_32AHB			2
#define AWRWAITUNIT_128AHB			3
#define AWRWAITUNIT_512AHB			4
#define AWRWAITUNIT_2048AHB			5
#define AWRWAITUNIT_8192AHB			6
#define AWRWAITUNIT_32768AHB		7
#define FLSHNNCR2_CLRINSTRPTR		BIT(31)

#define REG_FLSHCR4					0x94
#define FLSHCR4_WMOPT1				BIT(0)
#define FLSHCR4_WMENA				BIT(2)
#define FLSHCR4_WMENB				BIT(3)
#define FLSHCR4_PAR_WM(x)			(((x) & 0x03) << 9)
#define FLSHCR4_PAR_ADDR_ADJ_DIS	BIT(11)

#define REG_IPCR0					0xA0
#define IPCR0_SFAR(x)				(x)

#define REG_IPCR1					0xA4
#define IPCR1_IDATASZ(x)			(((x) & 0xFFFF) << 0)
#define IPCR1_ISEQID(x)				(((x) & 0x0F) << 16)
#define IPCR1_ISEQNUM(x)			(((x) & 0x0F) << 24)
#define IPCR1_IPAREN				BIT(31)

#define REG_IPCMD					0xB0
#define IPCMD_TRG					BIT(0)

#define REG_IPRXFCR					0xB8
#define IPRXFCR_CLRIPRXF			BIT(0)
#define IPRXFCR_RXDMAEN				BIT(1)
#define IPRXFCR_RXWMRK(x)			(((x) & 0x0F) << 2)

#define REG_IPTXFCR					0xBC
#define IPTXFCR_CLRIPTXF			BIT(0)
#define IPTXFCR_TXDMAEN				BIT(1)
#define IPTXFCR_TXWMRK(x)			(((x) & 0x0F) << 2)

#define REG_STS1					0xE4

#define REG_RFDR					0x100
#define REG_TFDR					0x180

#define REG_LUT_BASE				0x200
#define REG_LUT(x)					(REG_LUT_BASE + (x) * 4)

/*
 * This LUT sequence ID is used during probe to read flash IDs and then is
 * left configured so that the FLEXSPI can provide the usual linear
 * memory-mapped access to the attached serial flash
 *
 * The LUT sequence ID 0 is the default for FlexSPI NOR (and NAND?) boot
 * and is used by every example from Freescale/NXP that I've ever seen for AHB
 * reads, though it doesn't strictly have to be.
 */
#define LUTNUM_AHB_READ				0

/*
 * This LUT sequence ID is used to read flash status since it's a common
 * thing to have to do interleaved with other actions
 *
 * The LUT sequence ID 1 is pretty much universally used for this purpose
 * throughout Freescale/NXP's example/HAL code, and we at least attempt to use
 * it in a compatible way.
 */
#define LUTNUM_READ_STATUS			1

/*
 * This LUT sequence ID is used to perform write enable since it's also a
 * common thing to have to do interleaved with other actions
 *
 * Like with read status, LUT ID 3 seems to be used for this pretty much
 * everywhere at least for NOR flash.
 */
#define LUTNUM_WRITE_ENABLE			3

/*
 * The usual LUT sequence ID to be used by the driver after probing for
 * general activities other than reading status and performing write enable
 *
 * Other LUT sequence numbers will not be touched and can be used by
 * application code without impeding operating by the driver without controller
 * re-initialization if some other basic conditions are met.
 *
 * The LUT sequence number 10 is not used by boot procedures nor the Freescale
 * sample code
 */
#define LUTNUM_DRV					10

/* Instruction set for the LUT register (SDR only) */
#define OPCODE_STOP					0
#define OPCODE_CMD					1
#define OPCODE_RADDR				2
#define OPCODE_CADDR				3
#define OPCODE_MODE1				4
#define OPCODE_MODE2				5
#define OPCODE_MODE4				6
#define OPCODE_MODE8				7
#define OPCODE_WRITE				8
#define OPCODE_READ					9
#define OPCODE_LEARN				10
#define OPCODE_DATSZ				11
#define OPCODE_DUMMY				12
#define OPCODE_JMP_ON_CS			31

