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

#ifndef OPENOCD_FLASH_NOR_CC3220SF_H
#define OPENOCD_FLASH_NOR_CC3220SF_H

/* CC3220SF device types */
#define CC3220_NO_TYPE 0 /* Device type not determined yet */
#define CC3220_OTHER   1 /* CC3220 variant without flash */
#define CC3220SF       2 /* CC3220SF variant with flash */

/* Flash parameters */
#define FLASH_BASE_ADDR   0x01000000
#define FLASH_SECTOR_SIZE 2048
#define FLASH_NUM_SECTORS 512

/* CC2200SF flash registers */
#define FMA_REGISTER_ADDR 0x400FD000
#define FMC_REGISTER_ADDR 0x400FD008
#define FMC_DEFAULT_VALUE 0xA4420000
#define FMC_ERASE_BIT     0x00000002
#define FMC_MERASE_BIT    0x00000004

/* Flash helper algorithm addresses and values */
#define ALGO_BASE_ADDR    0x20000000
#define ALGO_BUFFER_ADDR  0x20000400
#define ALGO_BUFFER_SIZE  0x1000
#define ALGO_WORKING_SIZE 0x1400

/* Flash helper algorithm for CC3220SF (assembled by hand) */
const uint8_t cc3220sf_algo[] = {
							/*		; flash programming key					*/
	0xdf, 0xf8, 0x7c, 0xa0,	/*	1:	ldr		r10, =0xa4420001				*/
							/*		; base of FWB							*/
	0xdf, 0xf8, 0x7c, 0xb0,	/*		ldr		r11, =0x400fd100				*/
							/*		; base of flash regs					*/
	0xdf, 0xf8, 0x7c, 0xc0,	/*		ldr		r12, =0x400fd000				*/
							/*		; is the dest address 32-bit aligned?	*/
	0x01, 0xf0, 0x7f, 0x03,	/*		and		r3, r1, #0x7f					*/
	0x00, 0x2b,				/*		cmp		r3, #0							*/
							/*		; if not aligned do one word at a time	*/
	0x1e, 0xd1,				/*		bne		%6								*/

							/*		; program using the write buffers		*/
							/*		; start the buffer word counter at 0	*/
	0x4f, 0xf0, 0x00, 0x04,	/*		ldr		r4, =0							*/
							/*		; store the dest addr in FMA			*/
	0xcc, 0xf8, 0x00, 0x10,	/*		str		r1, [r12]						*/
							/*		; get the word to write to FWB			*/
	0x03, 0x68,				/*	2:	ldr		r3, [r0]						*/
							/*		; store the word in the FWB				*/
	0xcb, 0xf8, 0x00, 0x30,	/*		str		r3, [r11]						*/
							/*		; increment the FWB pointer				*/
	0x0b, 0xf1, 0x04, 0x0b,	/*		add		r11, r11, #4					*/
							/*		; increment the source pointer			*/
	0x00, 0xf1, 0x04, 0x00,	/*		add		r0, r0, #4						*/
							/*		; decrement the total word counter		*/
	0xa2, 0xf1, 0x01, 0x02,	/*		sub		r2, r2, #1						*/
							/*		; increment the buffer word counter		*/
	0x04, 0xf1, 0x01, 0x04,	/*		add		r4, r4, #1						*/
							/*		; increment the dest pointer			*/
	0x01, 0xf1, 0x04, 0x01,	/*		add		r1, r1, #4						*/
							/*		; is the total word counter now 0?		*/
	0x00, 0x2a,				/*		cmp		r2, #0							*/
							/*		; go to end if total word counter is 0	*/
	0x01, 0xd0,				/*		beq		%3								*/
							/*		; is the buffer word counter now 32?	*/
	0x20, 0x2c,				/*		cmp		r4, #32							*/
							/*		; go to continue to fill buffer			*/
	0xee, 0xd1,				/*		bne		%2								*/
							/*		; store the key and write bit to FMC2	*/
	0xcc, 0xf8, 0x20, 0xa0,	/*	3:	str		r10, [r12, #0x20]				*/
							/*		; read FMC2								*/
	0xdc, 0xf8, 0x20, 0x30,	/*	4:	ldr		r3, [r12, #0x20]				*/
							/*		; see if the write bit is cleared		*/
	0x13, 0xf0, 0x01, 0x0f,	/*		tst		r3, #1							*/
							/*		; go to read FMC2 if bit not cleared	*/
	0xfa, 0xd1,				/*		bne		%4								*/
							/*		; is the total word counter now 0?		*/
	0x00, 0x2a,				/*		cmp		r2, #0							*/
							/*		; go if there is more to program		*/
	0xd7, 0xd1,				/*		bne		%1								*/
	0x13, 0xe0,				/*		b		%5								*/

							/*		; program 1 word at a time				*/
							/*		; store the dest addr in FMA			*/
	0xcc, 0xf8, 0x00, 0x10,	/*	6:	str		r1, [r12]						*/
							/*		; get the word to write to FMD			*/
	0x03, 0x68,				/*		ldr		r3, [r0]						*/
							/*		; store the word in FMD					*/
	0xcc, 0xf8, 0x04, 0x30,	/*		str		r3, [r12, #0x4]					*/
							/*		; store the key and write bit to FMC	*/
	0xcc, 0xf8, 0x08, 0xa0,	/*		str		r10, [r12, #0x8]				*/
							/*		; read FMC								*/
	0xdc, 0xf8, 0x08, 0x30,	/*	7:	ldr		r3, [r12, #0x8]					*/
							/*		; see if the write bit is cleared		*/
	0x13, 0xf0, 0x01, 0x0f,	/*		tst		r3, #1							*/
							/*		; go to read FMC if bit not cleared		*/
	0xfa, 0xd1,				/*		bne		%7								*/
							/*		; decrement the total word counter		*/
	0xa2, 0xf1, 0x01, 0x02,	/*		sub		r2, r2, #1						*/
							/*		; increment the source pointer			*/
	0x00, 0xf1, 0x04, 0x00,	/*		add		r0, r0, #4						*/
							/*		; increment the dest pointer			*/
	0x01, 0xf1, 0x04, 0x01,	/*		add		r1, r1, #4						*/
							/*		; is the total word counter now 0		*/
	0x00, 0x2a,				/*		cmp		r2, #0							*/
							/*		; go if there is more to program		*/
	0xc2, 0xd1,				/*		bne		%1								*/
							/*		; end									*/
	0x00, 0xbe,				/*	5:	bkpt	#0								*/
	0x01, 0xbe,				/*		bkpt	#1								*/
	0xfc, 0xe7,				/*		b		%5								*/

							/*		; flash register address constants		*/
	0x01, 0x00, 0x42, 0xa4,	/*		.word	0xa4420001						*/
	0x00, 0xd1, 0x0f, 0x40,	/*		.word	0x400fd100						*/
	0x00, 0xd0, 0x0f, 0x40	/*		.word	0x400fd000						*/
};

#endif /* OPENOCD_FLASH_NOR_CC3220SF_H */
