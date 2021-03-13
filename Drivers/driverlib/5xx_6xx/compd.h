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
#ifndef __MSP430WARE_COMPD_H__
#define __MSP430WARE_COMPD_H__

#define __MSP430_HAS_COMPD__

//*****************************************************************************
//
//The following are values that can be passed to Comp_init(),
//Comp_disableInputBuffer(), and Comp_enableInputBuffer() in the
//positiveTerminalInput, negativeTerminalInput, and inputPort parameters.
//
//*****************************************************************************
#define COMPD_INPUT0  (CDIPSEL_0)
#define COMPD_INPUT1  (CDIPSEL_1)
#define COMPD_INPUT2  (CDIPSEL_2)
#define COMPD_INPUT3  (CDIPSEL_3)
#define COMPD_INPUT4  (CDIPSEL_4)
#define COMPD_INPUT5  (CDIPSEL_5)
#define COMPD_INPUT6  (CDIPSEL_6)
#define COMPD_INPUT7  (CDIPSEL_7)
#define COMPD_INPUT8  (CDIPSEL_8)
#define COMPD_INPUT9  (CDIPSEL_9)
#define COMPD_INPUT10 (CDIPSEL_10)
#define COMPD_INPUT11 (CDIPSEL_11)
#define COMPD_INPUT12 (CDIPSEL_12)
#define COMPD_INPUT13 (CDIPSEL_13)
#define COMPD_INPUT14 (CDIPSEL_14)
#define COMPD_INPUT15 (CDIPSEL_15)
#define COMPD_VREF    (0x10)

//*****************************************************************************
//
//The following are values that can be passed to Comp_init() in the
//outputFilterEnableAndDelayLevel parameter.
//
//*****************************************************************************
#define COMPD_FILTEROUTPUT_OFF     0x00
#define COMPD_FILTEROUTPUT_DLYLVL1 (CDF + CDFDLY_0)
#define COMPD_FILTEROUTPUT_DLYLVL2 (CDF + CDFDLY_1)
#define COMPD_FILTEROUTPUT_DLYLVL3 (CDF + CDFDLY_2)
#define COMPD_FILTEROUTPUT_DLYLVL4 (CDF + CDFDLY_3)

//*****************************************************************************
//
//The following are values that can be passed to Comp_init() in the
//invertedOutputPolarity parameter.
//
//*****************************************************************************
#define COMPD_NORMALOUTPUTPOLARITY   ( !(CDOUTPOL) )
#define COMPD_INVERTEDOUTPUTPOLARITY (CDOUTPOL)

//*****************************************************************************
//
//The following are values that can be passed to Comp_setReferenceVoltage() in
//the supplyVoltageBase parameter.
//
//*****************************************************************************
#define COMPD_REFERENCE_AMPLIFIER_DISABLED (CDREFL_0)
#define COMPD_VREFBASE1_5V (CDREFL_1)
#define COMPD_VREFBASE2_0V (CDREFL_2)
#define COMPD_VREFBASE2_5V (CDREFL_3)
//deprecated
#define COMPD_VREFBASE_VCC (CDREFL_0)

//*****************************************************************************
//
//The following are values that can be passed to Comp_setEdgeDirection() in
//the edgeDirection parameter.
//
//*****************************************************************************
#define COMPD_FALLINGEDGE ( !(CDIES) )
#define COMPD_RISINGEDGE  (CDIES)

//*****************************************************************************
//
//The following are values that returned by COMPD_outputValue().
//
//*****************************************************************************
#define COMPD_LOW  (0x0)
#define COMPD_HIGH (0x1)

//*****************************************************************************
//
//The following are values that can be passed to COMPD_enableInterrupt().
// COMPD_disableInterrupt()
//
//*****************************************************************************
#define COMPD_INTERRUPT_ENABLE CDIE
#define COMPD_INTERRUPT_ENABLE_INVERTED_POLARITY CDIIE

//*****************************************************************************
//
//The following are values that can be passed to COMPD_clearInterrupt().
// and returned from COMPD_getInterruptStatus()
//
//*****************************************************************************
#define COMPD_INTERRUPT_FLAG CDIFG
#define COMPD_INTERRUPT_FLAG_INVERTED_POLARITY CDIIFG

//*****************************************************************************
//API
//*****************************************************************************

extern unsigned short COMPD_init(unsigned int baseAddress,
		unsigned char positiveTerminalInput,
		unsigned char negativeTerminalInput,
		unsigned char outputFilterEnableAndDelayLevel,
		unsigned short invertedOutputPolarity);

extern void COMPD_setReferenceVoltage(unsigned int baseAddress,
		unsigned int supplyVoltageReferenceBase,
		unsigned int lowerLimitSupplyVoltageFractionOf32,
		unsigned int upperLimitSupplyVoltageFractionOf32);

extern void COMPD_enableInterrupt(unsigned int baseAddress, unsigned int mask);

extern void COMPD_disableInterrupt(unsigned int baseAddress, unsigned int mask);

extern void COMPD_clearInterrupt(unsigned int baseAddress, unsigned int mask);

extern unsigned char COMPD_getInterruptStatus(unsigned int baseAddress,
		unsigned int mask);

extern void COMPD_interruptSetEdgeDirection(unsigned int baseAddress,
		unsigned short edgeDirection);

extern void COMPD_interruptToggleEdgeDirection(unsigned int baseAddress);

extern void COMPD_enable(unsigned int baseAddress);

extern void COMPD_disable(unsigned int baseAddress);

extern void COMPD_shortInputs (unsigned int baseAddress);

extern void COMPD_unshortInputs (unsigned int baseAddress);

extern void COMPD_disableInputBuffer(unsigned int baseAddress,
		unsigned char inputPort);

extern void COMPD_enableInputBuffer(unsigned int baseAddress,
		unsigned char inputPort);

extern void COMPD_IOSwap(unsigned int baseAddress);

extern unsigned short COMPD_outputValue(unsigned int baseAddress);

#endif

