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
#ifndef __MSP430WARE_CSA_A_H__
#define __MSP430WARE_CSA_A_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_SFR__
#define __MSP430_HAS_CS_A__

//*****************************************************************************
//
//The following are values that can be passed to the CSA_clockSignalInit() API
//as the selectedClockSignal parameter.
//
//*****************************************************************************
#define CSA_ACLK                0x01
#define CSA_MCLK                0x02
#define CSA_SMCLK               0x04

//*****************************************************************************
//
//The following along with CSA_ACLK, CSA_MCLK, CSA_SMCLK may be passed to the
//CSA_clockRequestEnable() and CSA_clockRequestDisable() API
//as the selectClock parameter.
//
//*****************************************************************************
#define CSA_MODOSC              MODCLKREQEN

//*****************************************************************************
//
//The following are values that can be passed to the CSA_clockSignalInit() API
//as the clockSource parameter. CSA_VLOCLK_SELECT may not be used for
//selectedClockSignal CSA_FLLREF
//
//*****************************************************************************
#define CSA_VLOCLK_SELECT        SELM__VLOCLK
#define CSA_DCOCLK_SELECT        SELM__DCOCLK
#define CSA_LFXTCLK_SELECT		SELM__LFXTCLK
#define CSA_HFXTCLK_SELECT		SELM__HFXTCLK
#define CSA_LFMODOSC_SELECT		SELM__LFMODOSC
#define CSA_MODOSC_SELECT		SELM__MODOSC

//*****************************************************************************
//
//The following are values that can be passed to the CSA_clockSignalInit() API
//as the clockSourceDivider parameter.
//
//*****************************************************************************
#define CSA_CLOCK_DIVIDER_1     DIVM__1
#define CSA_CLOCK_DIVIDER_2     DIVM__2
#define CSA_CLOCK_DIVIDER_4     DIVM__4
#define CSA_CLOCK_DIVIDER_8     DIVM__8
#define CSA_CLOCK_DIVIDER_16    DIVM__16
#define CSA_CLOCK_DIVIDER_32    DIVM__32


//*****************************************************************************
//
//The following are values that can be passed to the CSA_LFXTStart(),
//CSA_LFXTStartWithTimeout(), as the lfxtdrive parameter.
//
//*****************************************************************************
#define CSA_LFXT_DRIVE0	LFXTDRIVE_0
#define CSA_LFXT_DRIVE1	LFXTDRIVE_1
#define CSA_LFXT_DRIVE2	LFXTDRIVE_2
#define CSA_LFXT_DRIVE3	LFXTDRIVE_3


//*****************************************************************************
//
//The following are values that can be passed to the CSA_SetDCOFreq() API
//as the dcorsel parameter.
//
//*****************************************************************************
#define CSA_DCORSEL_0	0x00
#define CSA_DCORSEL_1	DCORSEL


//*****************************************************************************
//
//The following are values that can be passed to the CSA_SetDCOFreq() API
//as the dcofsel parameter.
//
//*****************************************************************************
#define CSA_DCOFSEL_0	DCOFSEL_0
#define CSA_DCOFSEL_1	DCOFSEL_1
#define CSA_DCOFSEL_2	DCOFSEL_2
#define CSA_DCOFSEL_3	DCOFSEL_3
#define CSA_DCOFSEL_4	DCOFSEL_4
#define CSA_DCOFSEL_5	DCOFSEL_5
#define CSA_DCOFSEL_6	DCOFSEL_6

//*****************************************************************************
//
//FOR FR57xx DEVICES. The following are values can be passed to CSA_XT2_Start
//and CSA_XT2_StartWithTimeOut as the hfxtdtive parameter.
//
//*****************************************************************************
#define CSA_HFXTDRIVE_4MHZ_8MHZ       HFXTDRIVE_0
#define CSA_HFXTDRIVE_8MHZ_16MHZ      HFXTDRIVE_1
#define CSA_HFXTDRIVE_16MHZ_24MHZ     HFXTDRIVE_2
#define CSA_HFXTDRIVE_24MHZ_32MHZ     HFXTDRIVE_3


//*****************************************************************************
//
// The following are values can be passed to CSA_HFXT_Start
//and CSA_HFXT_StartWithTimeOut as the hfxtdtive parameter.
//
//*****************************************************************************
#define CSA_HFXTDRIVE_0     HFXTDRIVE_0
#define CSA_HFXTDRIVE_1     HFXTDRIVE_1
#define CSA_HFXTDRIVE_2     HFXTDRIVE_2
#define CSA_HFXTDRIVE_3     HFXTDRIVE_3


//*****************************************************************************
//
//The following are values that can be passed to the CSA_faultFlagStatus() and
//CSA_clearFaultFlag API as the mask parameter.
//
//*****************************************************************************
#define CSA_XT2OFFG   XT2OFFG
#define CSA_XT1OFFG   XT1OFFG

//*****************************************************************************
//
//FOR WOLVERINE DEVICES. The following are values that can be passed to the
//CSA_faultFlagStatus and CSA_clearFaultFlag as the mask parameter.
//
//*****************************************************************************
#define CSA_LFXTOFFG		LFXTOFFG
#define CSA_HFXTOFFG		HFXTOFFG

//*****************************************************************************
//
//FOR FR57xx devices. The following are values that can be passed to the
// CSA_LFXT1Start() and CSA_LFXT1StartWithTimeout() API as the lfxtdrive parameter.
//
//*****************************************************************************
#define CSA_XT1DRIVE_0	XT1DRIVE_0
#define CSA_XT1DRIVE_1	XT1DRIVE_1
#define CSA_XT1DRIVE_2	XT1DRIVE_2
#define CSA_XT1DRIVE_3	XT1DRIVE_3

//*****************************************************************************
//
//FOR FR57xx devices. The following are values that can be passed to the
// CSA_LFXT1Start() and CSA_LFXT1StartWithTimeout() API as the lfxtdrive parameter.
//
//*****************************************************************************
#define CSA_LFXTDRIVE_0	LFXTDRIVE_0
#define CSA_LFXTDRIVE_1	LFXTDRIVE_1
#define CSA_LFXTDRIVE_2	LFXTDRIVE_2
#define CSA_LFXTDRIVE_3	LFXTDRIVE_3

//*****************************************************************************
//
//Internal very low power VLOCLK, low frequency oscillator with
//10 kHz typical frequency
//Internal low-power oscillator MODCLK with 5 MHz typical 
//frequency and LFMODCLK is MODCLK divided by 128.
//
//
//*****************************************************************************
#define CSA_VLOCLK_FREQUENCY	10000
#define CSA_MODCLK_FREQUENCY	5000000
#define CSA_LFMODCLK_FREQUENCY	39062


//*****************************************************************************
//
//The following value is used by CSA_XT1Start, CSA_bypassXT1,
//CSA_XT1StartWithTimeout, CSA_bypassXT1WithTimeout on FR57xx devices to properly
//set the XTS bit. This frequnecy threshold is specified in the FR5xx
//User's Guide
//
//*****************************************************************************

#define LFXT_FREQUENCY_THRESHOLD		50000


//*****************************************************************************
//
//The following value is used by CSA_getACLK, CSA_getSMCLK, CSA_getMCLK to
//determine the operating frequency based on the available DCO frequencies for
//FR58xx_FR59xx devices
//
//*****************************************************************************
#define CSA_DCO_FREQ_1 	1000000
#define CSA_DCO_FREQ_2 	2670000
#define CSA_DCO_FREQ_3 	3330000
#define CSA_DCO_FREQ_4 	4000000
#define CSA_DCO_FREQ_5 	5330000
#define CSA_DCO_FREQ_6 	6670000
#define CSA_DCO_FREQ_7 	8000000
#define CSA_DCO_FREQ_8 	16000000

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void
CSA_setExternalClockSource (unsigned int baseaddress,
    unsigned long LFXT_LFXT_CLK_frequency,
    unsigned long HFXT_HFXT_CLK_frequency
    );
extern void
CSA_clockSignalInit ( unsigned int baseaddress,
    unsigned char selectedClockSignal,
    unsigned int clockSource,
    unsigned int clockSourceDivider
    );

extern void
CSA_XT1Start ( unsigned int baseAddress,
    unsigned int lfxtdrive
    );

extern void
CSA_LFXTStart(
		unsigned int baseAddress,
	    unsigned int lfxtdrive
		);

extern void
CSA_bypassXT1 ( unsigned int baseAddress
    );

extern void
CSA_bypassLFXT ( unsigned int baseAddress
    );


extern unsigned short
CSA_bypassXT1WithTimeout (
    unsigned int baseAddress,
    unsigned long timeout
    );

extern unsigned short
CSA_bypassLFXTWithTimeout (
    unsigned int baseAddress,
    unsigned long timeout
    );

extern unsigned short
CSA_XT1StartWithTimeout (
    unsigned int baseAddress,
    unsigned int lfxtdrive,
    unsigned long timeout
    );

unsigned short
CSA_LFXTStartWithTimeout(
		unsigned int baseAddress,
	    unsigned int lfxtdrive,
	    unsigned long timeout
		);

extern void
CSA_XT1Off (unsigned int baseAddress);

extern void
CSA_LFXTOff (unsigned int baseAddress);

extern void CSA_XT2Start (  unsigned int baseAddress,
    unsigned int hfxtdtive
    );

extern void CSA_HFXTStart (  unsigned int baseAddress,
    unsigned int hfxtdtive
    );

extern void CSA_bypassXT2 (  unsigned int baseAddress );

extern void CSA_bypassHFXT (  unsigned int baseAddress );

extern unsigned short
CSA_XT2StartWithTimeout ( unsigned int baseAddress,
    unsigned int hfxtdtive,
    unsigned long timeout
    );

extern unsigned short
CSA_HFXTStartWithTimeout ( unsigned int baseAddress,
    unsigned int hfxtdrive,
    unsigned long timeout
    );

extern unsigned short
CSA_bypassXT2WithTimeout ( unsigned int baseAddress,
    unsigned long timeout
    );

extern unsigned short
CSA_bypassHFXTWithTimeout ( unsigned int baseAddress,
    unsigned long timeout
    );

extern void
CSA_XT2Off (unsigned int baseAddress);

extern void
CSA_HFXTOff (unsigned int baseAddress);


extern void CSA_enableClockRequest (    unsigned int baseAddress,
    unsigned char selectClock
    );
extern void CSA_disableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    );
extern unsigned char CSA_faultFlagStatus (
    unsigned int baseAddress,
    unsigned char mask
    );

extern void CSA_clearFaultFlag (
    unsigned int baseAddress,
    unsigned char mask
    );

extern unsigned long
CSA_getACLK (
		unsigned int baseAddress);

extern unsigned long
CSA_getSMCLK (
		unsigned int baseAddress);

extern unsigned long
CSA_getMCLK (
		unsigned int baseAddress);

extern unsigned int
CSA_clearAllOscFlagsWithTimeout(
		unsigned int baseAddress,
        unsigned long timeout);

extern void
CSA_VLOoff(unsigned int baseAddress);

extern void
CSA_setDCOFreq(
		unsigned int baseAddress,
		unsigned int dcorsel,
		unsigned int dcofsel);

#endif
