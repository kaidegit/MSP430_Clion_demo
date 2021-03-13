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
#ifndef __MSP430WARE_FRAM_H__
#define __MSP430WARE_FRAM_H__


//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_FRAM_FR5XX__


//*****************************************************************************
//
//The following are values that can be passed to the FRAM_enableInterrupt,
//FRAM_disableInterrupt, API as the mask parameter.
//
//*****************************************************************************

#define FRAM_PUC_ON_UNCORRECTABLE_BIT		UBDRSTEN
#define FRAM_UNCORRECTABLE_BIT_INTERUPT		UBDIEN
#define FRAM_CORRECTABLE_BIT_INTERRUPT		CBDIEN
#define FRAM_ACCESS_VIOLATION_INTERRUPT		ACCVIE
#define FRAM_ACCESS_TIME_ERROR_INTERUPT		ACCTEIE
#define FRAM_PUC_ON_DOUBLE_BIT_ERROR		SBDRSTEN
#define FRAM_DOUBLE_BIT_ERROR_INTERUPT		DBDIEN
#define FRAM_SINGLE_BIT_ERROR_INTERUPT		SBDIEN


//*****************************************************************************
//
//The following are values that can be passed to the FRAM_getInterruptStatus
//API as the mask parameter.
//
//*****************************************************************************

#define FRAM_ACCESS_TIME_ERROR_FLAG			ACCTEIFG
#define FRAM_UNCORRECTABLE_BIT_FLAG			UBDIFG
#define FRAM_CORRECTABLE_BIT_FLAG			CBDIFG
#define FRAM_ACCESS_VIOLATION_FLAG			ACCVIFG
#define FRAM_SINGLE_BIT_ERROR_FLAG			SBDIFG
#define FRAM_DOUBLE_BIT_ERROR_FLAG			DBDIFG

//*****************************************************************************
//
//The following are values that are returned by FRAM_isBusy().
//
//*****************************************************************************
#define FRAM_NOTBUSY (0x0)
#define FRAM_BUSY    (0x1)

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************

extern void FRAM_write8(unsigned int baseAddress,
		unsigned char *dataPtr,
		unsigned char *framPtr,
		unsigned int numberOfBytes);

extern void FRAM_write16(unsigned int baseAddress,
		 unsigned int *dataPtr,
		 unsigned int *framPtr,
		 unsigned int numberOfBytes);

extern void FRAM_write32(unsigned int baseAddress,
		unsigned long *dataPtr,
		unsigned long *framPtr,
		unsigned int numberOfBytes);

extern void FRAM_memoryFill32(unsigned int baseAddress,
		 unsigned long value,
		 unsigned long *framPtr,
		 unsigned int count);

extern void FRAM_enableInterrupt (unsigned int baseAddress,
		unsigned char interruptMask);

extern unsigned char FRAM_getInterruptStatus(unsigned int baseAddress,
		unsigned int interruptFlagMask);

extern void FRAM_disableInterrupt(unsigned int baseAddress,
		unsigned int interruptMask);

extern unsigned short FRAM_isBusy(unsigned int baseAddress);

#endif
