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
#ifndef __MSP430WARE_CS_H__
#define __MSP430WARE_CS_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_SFR__
#define __MSP430_HAS_CS__

//*****************************************************************************
//
//The following are values that can be passed to the CS_clockSignalInit() API
//as the selectedClockSignal parameter.
//
//*****************************************************************************
#define CS_ACLK                0x01
#define CS_MCLK                0x02
#define CS_SMCLK               0x04

//*****************************************************************************
//
//The following along with CS_ACLK, CS_MCLK, CS_SMCLK may be passed to the
//CS_clockRequestEnable() and CS_clockRequestDisable() API
//as the selectClock parameter.
//
//*****************************************************************************
#define CS_MODOSC              MODCLKREQEN

//*****************************************************************************
//
//The following are values that can be passed to the CS_clockSignalInit() API
//as the clockSource parameter. CS_VLOCLK_SELECT may not be used for
//selectedClockSignal CS_FLLREF
//
//*****************************************************************************
#define CS_XT1CLK_SELECT        SELM__XT1CLK
#define CS_VLOCLK_SELECT        SELM__VLOCLK
#define CS_DCOCLK_SELECT        SELM__DCOCLK
#define CS_XT2CLK_SELECT        SELM__XT2CLK
#define CS_MODOSC_SELECT		SELM__MODOSC

//*****************************************************************************
//
//The following are values that can be passed to the CS_clockSignalInit() API
//as the clockSourceDivider parameter.
//
//*****************************************************************************
#define CS_CLOCK_DIVIDER_1     DIVM__1
#define CS_CLOCK_DIVIDER_2     DIVM__2
#define CS_CLOCK_DIVIDER_4     DIVM__4
#define CS_CLOCK_DIVIDER_8     DIVM__8
#define CS_CLOCK_DIVIDER_16    DIVM__16
#define CS_CLOCK_DIVIDER_32    DIVM__32

//*****************************************************************************
//
//The following are values that can be passed to the CS_LFXT1Start(),
//CS_HFXT1Start(), CS_LFXT1StartWithTimeout(), CS_HFXT1StartWithTimeout()
//as the xt1drive parameter.
//
//*****************************************************************************
#define CS_XT1_DRIVE0	XT1DRIVE_0
#define CS_XT1_DRIVE1	XT1DRIVE_1
#define CS_XT1_DRIVE2	XT1DRIVE_2
#define CS_XT1_DRIVE3	XT1DRIVE_3

//*****************************************************************************
//
//The following are values that can be passed to the CS_SetDCOFreq() API
//as the dcorsel parameter.
//
//*****************************************************************************
#define CS_DCORSEL_0	DCOFSEL_0
#define CS_DCORSEL_1	DCORSEL


//*****************************************************************************
//
//The following are values that can be passed to the CS_SetDCOFreq() API
//as the dcofsel parameter.
//
//*****************************************************************************
#define CS_DCOFSEL_0	DCOFSEL_0
#define CS_DCOFSEL_1	DCOFSEL_1
#define CS_DCOFSEL_2	DCOFSEL_2
#define CS_DCOFSEL_3	DCOFSEL_3

//*****************************************************************************
//
//FOR FR57xx DEVICES. The following are values can be passed to CS_XT2_Start
//and CS_XT2_StartWithTimeOut as the xt2drive parameter.
//
//*****************************************************************************
#define CS_XT2DRIVE_4MHZ_8MHZ       XT2DRIVE_0
#define CS_XT2DRIVE_8MHZ_16MHZ      XT2DRIVE_1
#define CS_XT2DRIVE_16MHZ_24MHZ     XT2DRIVE_2
#define CS_XT2DRIVE_24MHZ_32MHZ     XT2DRIVE_3

//*****************************************************************************
//
//The following are values that can be passed to the CS_faultFlagStatus() and
//CS_clearFaultFlag API as the mask parameter.
//
//*****************************************************************************
#define CS_XT2OFFG   XT2OFFG
#define CS_XT1OFFG   XT1OFFG

//*****************************************************************************
//
//The following are values that can be passed to the
// CS_LFXT1Start() and CS_LFXT1StartWithTimeout() API as the xt1drive parameter.
//
//*****************************************************************************
#define CS_XT1DRIVE_0	XT1DRIVE_0
#define CS_XT1DRIVE_1	XT1DRIVE_1
#define CS_XT1DRIVE_2	XT1DRIVE_2
#define CS_XT1DRIVE_3	XT1DRIVE_3

//*****************************************************************************
//
//Internal very low power VLOCLK, low frequency oscillator with
//10 kHz typical frequency
//Internal low-power oscillator MODCLK with 5 MHz typical 
//frequency and LFMODCLK is MODCLK divided by 128.
//
//
//*****************************************************************************
#define CS_VLOCLK_FREQUENCY	10000
#define CS_MODCLK_FREQUENCY	5000000
#define CS_LFMODCLK_FREQUENCY	39062

//*****************************************************************************
//
//The following value is used by CS_XT1Start, CS_bypassXT1,
//CS_XT1StartWithTimeout, CS_bypassXT1WithTimeout on FR57xx devices to properly
//set the XTS bit. This frequnecy threshold is specified in the FR5xx
//User's Guide
//
//*****************************************************************************

#define XT1_FREQUENCY_THRESHOLD		50000


//*****************************************************************************
//
//The following value is used by CS_getACLK, CS_getSMCLK, CS_getMCLK to
//determine the operating frequency based on the available DCO frequencies for
//FR57xx devices
//
//*****************************************************************************
#define CS_DCO_FREQ_1 	5330000
#define CS_DCO_FREQ_2 	6670000
#define CS_DCO_FREQ_3 	8000000
#define CS_DCO_FREQ_4 	16000000
#define CS_DCO_FREQ_5 	20000000
#define CS_DCO_FREQ_6 	24000000

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void
CS_setExternalClockSource (unsigned int baseaddress,
    unsigned long XT1_LFXT_CLK_frequency,
    unsigned long XT2_HFXT_CLK_frequency
    );
extern void
CS_clockSignalInit ( unsigned int baseaddress,
    unsigned char selectedClockSignal,
    unsigned int clockSource,
    unsigned int clockSourceDivider
    );

extern void
CS_XT1Start ( unsigned int baseAddress,
    unsigned int xt1drive
    );

extern void
CS_bypassXT1 ( unsigned int baseAddress
    );

extern unsigned short
CS_bypassXT1WithTimeout (
    unsigned int baseAddress,
    unsigned long timeout
    );

extern unsigned short
CS_XT1StartWithTimeout (
    unsigned int baseAddress,
    unsigned int xt1drive,
    unsigned long timeout
    );
extern void
CS_XT1Off (unsigned int baseAddress);

extern void CS_XT2Start (  unsigned int baseAddress,
    unsigned int xt2drive
    );

extern void CS_bypassXT2 (  unsigned int baseAddress );

extern unsigned short
CS_XT2StartWithTimeout ( unsigned int baseAddress,
    unsigned int xt2drive,
    unsigned long timeout
    );

extern unsigned short
CS_bypassXT2WithTimeout ( unsigned int baseAddress,
    unsigned long timeout
    );

extern void
CS_XT2Off (unsigned int baseAddress);

extern void CS_enableClockRequest (    unsigned int baseAddress,
    unsigned char selectClock
    );
extern void CS_disableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    );
extern unsigned char CS_faultFlagStatus (
    unsigned int baseAddress,
    unsigned char mask
    );

extern void CS_clearFaultFlag (
    unsigned int baseAddress,
    unsigned char mask
    );

extern unsigned long
CS_getACLK (
		unsigned int baseAddress);

extern unsigned long
CS_getSMCLK (
		unsigned int baseAddress);

extern unsigned long
CS_getMCLK (
		unsigned int baseAddress);

extern unsigned int
CS_clearAllOscFlagsWithTimeout(
		unsigned int baseAddress,
        unsigned long timeout);

extern void
CS_setDCOFreq(
		unsigned int baseAddress,
		unsigned int dcorsel,
		unsigned int dcofsel);

#endif
