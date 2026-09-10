/********************************** (C) COPYRIGHT *******************************
 * File Name          : portASM.S
 * Author             : WCH
 * Version            : V2.0
 * Date               : 2026/01/12
 * Description        : WCH Qingke V4C FreeRTOS
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#define portCONTEXT_SIZE ( 32 * 4 )

.global xPortStartFirstTask
.global pxPortInitialiseStack
.global SW_Handler
.extern pxCurrentTCB
.extern vTaskSwitchContext
.extern xISRStackTop

.section	.highcode.SW_Handler,"ax",@progbits
.option arch, +zicsr
.align 2
.func
SW_Handler:
	/*
	   Adjust stack pointer.
	   We need space for:
	   - 30 General Purpose Registers (x1, x4-x31, excluding x0, x2, x3)
	   - 1  MEPC
	   - 1  MSTATUS
	   Total: 32 words * 4 bytes = 128 bytes.
	   (Make sure portCONTEXT_SIZE in your config matches this, or hardcode 128)
	*/
	addi sp, sp, -portCONTEXT_SIZE

	/* Save General Purpose Registers */
	sw x1,  1 * 4( sp )
	sw x5,  2 * 4( sp )
	sw x6,  3 * 4( sp )
	sw x7,  4 * 4( sp )
	sw x8,  5 * 4( sp )
	sw x9,  6 * 4( sp )
	sw x10, 7 * 4( sp )
	sw x11, 8 * 4( sp )
	sw x12, 9 * 4( sp )
	sw x13, 10 * 4( sp )
	sw x14, 11 * 4( sp )
	sw x15, 12 * 4( sp )
	sw x16, 13 * 4( sp )
	sw x17, 14 * 4( sp )
	sw x18, 15 * 4( sp )
	sw x19, 16 * 4( sp )
	sw x20, 17 * 4( sp )
	sw x21, 18 * 4( sp )
	sw x22, 19 * 4( sp )
	sw x23, 20 * 4( sp )
	sw x24, 21 * 4( sp )
	sw x25, 22 * 4( sp )
	sw x26, 23 * 4( sp )
	sw x27, 24 * 4( sp )
	sw x28, 25 * 4( sp )
	sw x29, 26 * 4( sp )
	sw x30, 27 * 4( sp )
	sw x31, 28 * 4( sp )
	sw x4,  29 * 4( sp )

	/* WCH Specific: Handle Hardware Stacking/Nesting bits if required */
	addi a1, x0, 0x20
	csrs 0x804, a1

	/* Save Critical CSRs: MEPC and MSTATUS */
	csrr t0, mepc
	sw   t0, 0 * 4( sp )   /* Save MEPC at offset 0 */

	csrr t0, mstatus
	sw   t0, 30 * 4( sp )  /* Save MSTATUS at offset 30 (or any free slot) */

	/* Save Stack Pointer to current TCB */
	lw  t0, pxCurrentTCB
	sw  sp, 0( t0 )

	/* Switch Tasks */
	jal vTaskSwitchContext

	/* Restore Stack Pointer from new TCB */
	lw  t1, pxCurrentTCB
	lw  sp, 0( t1 )

	/* Restore Critical CSRs */
	lw  t0, 30 * 4( sp )   /* Load MSTATUS */
	csrw mstatus, t0       /* Restore it BEFORE returning */

	lw  t0, 0 * 4( sp )    /* Load MEPC */
	csrw mepc, t0

	/* Restore General Purpose Registers */
	lw  x1,  1 * 4( sp )
	lw  x5,  2 * 4( sp )
	lw  x6,  3 * 4( sp )
	lw  x7,  4 * 4( sp )
	lw  x8,  5 * 4( sp )
	lw  x9,  6 * 4( sp )
	lw  x10, 7 * 4( sp )
	lw  x11, 8 * 4( sp )
	lw  x12, 9 * 4( sp )
	lw  x13, 10 * 4( sp )
	lw  x14, 11 * 4( sp )
	lw  x15, 12 * 4( sp )
	lw  x16, 13 * 4( sp )
	lw  x17, 14 * 4( sp )
	lw  x18, 15 * 4( sp )
	lw  x19, 16 * 4( sp )
	lw  x20, 17 * 4( sp )
	lw  x21, 18 * 4( sp )
	lw  x22, 19 * 4( sp )
	lw  x23, 20 * 4( sp )
	lw  x24, 21 * 4( sp )
	lw  x25, 22 * 4( sp )
	lw  x26, 23 * 4( sp )
	lw  x27, 24 * 4( sp )
	lw  x28, 25 * 4( sp )
	lw  x29, 26 * 4( sp )
	lw  x30, 27 * 4( sp )
	lw  x31, 28 * 4( sp )
	lw  x4,  29 * 4( sp )

	addi sp, sp, portCONTEXT_SIZE
	mret
	.endfunc
/*-----------------------------------------------------------*/

.section	.text,"ax",@progbits
.align 2
.func
xPortStartFirstTask:

	lw  sp, pxCurrentTCB			/* Load pxCurrentTCB. */
	lw  sp, 0( sp )				 	/* Read sp from first TCB member. */

	lw  x1, 0( sp ) /* Note for starting the scheduler the exception return address is used as the function return address. */

	lw  x6, 3 * 4( sp )		/* t1 */
	lw  x7, 4 * 4( sp )		/* t2 */
	lw  x8, 5 * 4( sp )		/* s0/fp */
	lw  x9, 6 * 4( sp )		/* s1 */
	lw  x10, 7 * 4( sp )	/* a0 */
	lw  x11, 8 * 4( sp )	/* a1 */
	lw  x12, 9 * 4( sp )	/* a2 */
	lw  x13, 10 * 4( sp )	/* a3 */
	lw  x14, 11 * 4( sp )	/* a4 */
	lw  x15, 12 * 4( sp )	/* a5 */
	lw  x16, 13 * 4( sp )	/* a6 */
	lw  x17, 14 * 4( sp )	/* a7 */
	lw  x18, 15 * 4( sp )	/* s2 */
	lw  x19, 16 * 4( sp )	/* s3 */
	lw  x20, 17 * 4( sp )	/* s4 */
	lw  x21, 18 * 4( sp )	/* s5 */
	lw  x22, 19 * 4( sp )	/* s6 */
	lw  x23, 20 * 4( sp )	/* s7 */
	lw  x24, 21 * 4( sp )	/* s8 */
	lw  x25, 22 * 4( sp )	/* s9 */
	lw  x26, 23 * 4( sp )	/* s10 */
	lw  x27, 24 * 4( sp )	/* s11 */
	lw  x28, 25 * 4( sp )	/* t3 */
	lw  x29, 26 * 4( sp )	/* t4 */
	lw  x30, 27 * 4( sp )	/* t5 */
	lw  x31, 28 * 4( sp )	/* t6 */

	/* Initial mstatus into x5 (t0). NOTE: pxPortInitialiseStack below stores this at
	   byte offset 120 (30 * 4), matching SW_Handler's own save/restore slot - not
	   offset 29 * 4 (116), which is the x4/tp slot (always 0 for a fresh task). */
	lw  x5, 30 * 4( sp )
	addi x5, x5, 0x08						/* Set MIE bit so the first task starts with interrupts enabled - required as returns with ret not eret. */
	csrrw  x0, mstatus, x5					/* Interrupts enabled from here! */
	lw  x5, 2 * 4( sp )		/* Initial x5 (t0) value. */

	addi	sp, sp, portCONTEXT_SIZE
	ret
	.endfunc
/*-----------------------------------------------------------*/

.section	.text,"ax",@progbits
.align 2
.func
pxPortInitialiseStack:
	/* a0 = pxTopOfStack (We are growing downwards) */
	/* a1 = pxCode (Entry Function) */
	/* a2 = pvParameters */

	/* 1. Allocate space for 32 registers (128 bytes) */
	addi a0, a0, -128

	/* 2. Prepare MSTATUS value (MPIE=1, MPP=11 -> 0x1880) */
	/* We hardcode this to ensure interrupts are enabled when the task starts */
	addi t0, x0, 0x188
	slli t0, t0, 4

	/* 3. Store MSTATUS at offset 120 (30 * 4) */
	sw t0, 120(a0)

	/* 4. Store MEPC (Task Entry Point) at offset 0 */
	sw a1, 0(a0)

	/* 5. Store pvParameters (x10) at offset 28 (7 * 4) */
	/* mapping: x1(1), x5(2), x6(3), x7(4), x8(5), x9(6), x10(7) */
	sw a2, 28(a0)

	/* 6. Store Return Address (x1) at offset 4 */
	/* We set this to 0 (or a specific error handler address if you have one) */
	sw x0, 4(a0)

	/* 7. Initialize other registers to 0 for cleanliness (Optional but good) */
	sw x0, 116(a0) /* x4 / tp */

	/* Return the new Stack Pointer (a0) */
	ret
	.endfunc
/*-----------------------------------------------------------*/
