/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef __MSP430WARE_MPU_H__
#define __MSP430WARE_MPU_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_MPU__

//*****************************************************************************
//
//The following are values that can be passed to the MPU_createTwoSegments()
//and MPU_createThreeSegments  function as the seg1accmask, seg2accmask and
//seg3accmask parameter.
//
//*****************************************************************************

#define MPU_READ		MPUSEG1RE
#define MPU_WRITE		MPUSEG1WE
#define MPU_EXEC		MPUSEG1XE

//*****************************************************************************
//
//The following are values that can be passed to the MPU_createPUConViolation()
//function as the segment parameter.
//
//*****************************************************************************
#define MPU_FIRST_SEG		MPUSEG1VS
#define MPU_SECOND_SEG		MPUSEG2VS
#define MPU_THIRD_SEG		MPUSEG3VS


//*****************************************************************************
//
//The following values are used to by createTwoSegments, createThreeSegments
//to check the user has passed a valid segmentation value. This value was
//obtained from the User's Guide
//
//*****************************************************************************
#define MPU_MAX_SEG_VALUE			0x20

#define MPU_SEG_1_ACCESS_VIOLATION 		MPUSEG1IFG
#define MPU_SEG_2_ACCESS_VIOLATION 		MPUSEG2IFG
#define MPU_SEG_3_ACCESS_VIOLATION 		MPUSEG3IFG

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void
MPU_createTwoSegments
(	unsigned int baseAddress,
	unsigned int seg2boundary,
	unsigned char seg1accmask,
	unsigned char seg2accmask
	);

extern void
MPU_createThreeSegments
(	unsigned int baseAddress,
	unsigned int seg1boundary,
	unsigned int seg2boundary,
	unsigned char seg1accmask,
	unsigned char seg2accmask,
	unsigned char seg3accmask
	);

extern void
MPU_start
(	unsigned int baseAddress
	);

extern void
MPU_enablePUCOnViolation
(	unsigned int baseAddress,
	unsigned int segment
	);

extern
void
MPU_disablePUCOnViolation(unsigned int baseAddress,
		unsigned int segment
		);

extern unsigned short
MPU_getInterruptStatus
(	unsigned int baseAddress,
	unsigned int memAccFlag
	);

extern unsigned short
MPU_clearInterruptFlag
(	unsigned int baseAddress,
	unsigned int memAccFlag
	);

unsigned short
MPU_clearAllInterruptFlags
(	unsigned int baseAddress
	);

#endif
