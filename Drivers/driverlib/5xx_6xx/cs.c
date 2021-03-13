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
//******************************************************************************
//
//cs.c - Driver for the CS Module.
//
//******************************************************************************

#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/cs.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
#include "driverlib/5xx_6xx/debug.h"
#include "inc/sfr_sys_baseAddress.h"


//******************************************************************************
//
//The XT1 crystal frequency. Should be set with
//CS_externalClockSourceInit if XT1 is used and user intends to invoke
//CS_getSMCLK, CS_getMCLK, CS_getACLK and
//XT1Start, XT1ByPass, XT1StartWithTimeout, XT1ByPassWithTimeout.
//
//******************************************************************************
unsigned long CS_XT1ClockFrequency = 0;

//******************************************************************************
//
//The XT2 crystal frequency. Should be set with
//CS_externalClockSourceInit if XT2 is used and user intends to invoke
//CS_getSMCLK, CS_getMCLK, CS_getACLK and
//
//******************************************************************************
unsigned long CS_XT2ClockFrequency = 0;

//******************************************************************************
//
//! This function sets the external clock sources XT1 and XT2 crystal
//! oscillator frequency values. This function must be called if an external
//! crystal XT1 or XT2 is used and the user intends to call
//! CS_getMCLK, CS_getSMCLK, CS_getACLK and
//! XT1Start, XT1ByPass, XT1StartWithTimeout, XT1ByPassWithTimeout.
//!
//! \param baseAddress is the base address of the CS module.
//! \param XT1CLK_frequency is the XT1 crystal frequencies in Hz
//! \param XT2CLK_frequency is the XT2 crystal frequencies in Hz
//!
//! \return None
//
//******************************************************************************
void
CS_setExternalClockSource (unsigned int baseAddress,
    unsigned long XT1CLK_frequency,
    unsigned long XT2CLK_frequency
    )
{
    CS_XT1ClockFrequency = XT1CLK_frequency;
    CS_XT2ClockFrequency = XT2CLK_frequency;
}


//******************************************************************************
//
//! This function initializes each of the clock signals. The user must ensure
//! that this function is called for each clock signal. If not, the default
//! state is assumed for the particular clock signal. Refer MSP430ware
//! documentation for CS module or Device Family User's Guide for details of
//! default clock signal states. IMPORTANT: User must call CS_init function
//! before calling this API. Otherwise CS register will remain with default
//! values.
//!
//! \param baseAddress is the base address of the CS module.
//! \param selectedClockSignal - Valid values are
//!           \b CS_ACLK,
//!           \b CS_MCLK,
//!           \b CS_SMCLK,
//! \param clockSource is Clock source for the selectedClock Signal
//!            \e Valid values for FR57xx devices
//!            \b CS_XT1CLK_SELECT,
//!            \b CS_VLOCLK_SELECT,
//!            \b CS_DCOCLK_SELECT,
//!            \b CS_XT2CLK_SELECT
//!            \e Valid values for FR58xx/FR59xx devices
//!            \b CS_LFXTCLK_SELECT,
//!            \b CS_VLOCLK_SELECT,
//!            \b CS_DCOCLK_SELECT,		[Not available for ACLK]
//!            \b CS_HFXTCLK_SELECT,	[Not available for ACLK]
//!            \b CS_LFMODOSC_SELECT,
//!            \b CS_MODOSC_SELECT		[Not available for ACLK]
//! \param clockSourceDivider - selected the clock divider to calculate
//!         clock signal from clock source. Valid values are
//!           \b CS_CLOCK_DIVIDER_1,	[Default for ACLK]	
//!           \b CS_CLOCK_DIVIDER_2,
//!           \b CS_CLOCK_DIVIDER_4,
//!           \b CS_CLOCK_DIVIDER_8,	[Default for SMCLK and MCLK]
//!           \b CS_CLOCK_DIVIDER_16,
//!           \b CS_CLOCK_DIVIDER_32
//!
//! Modified registers are \b CSCTL0, \b CSCTL2, \b CSCTL3
//!
//! \return NONE
//
//******************************************************************************
void
CS_clockSignalInit ( unsigned int baseAddress,
    unsigned char selectedClockSignal,
    unsigned int clockSource,
    unsigned int clockSourceDivider
    )
{


	//Verify User has selected a valid Frequency divider
	ASSERT(
		(CS_CLOCK_DIVIDER_1 == clockSourceDivider) ||
		(CS_CLOCK_DIVIDER_2 == clockSourceDivider) ||
		(CS_CLOCK_DIVIDER_4 == clockSourceDivider) ||
		(CS_CLOCK_DIVIDER_8 == clockSourceDivider) ||
		(CS_CLOCK_DIVIDER_16 == clockSourceDivider) ||
		(CS_CLOCK_DIVIDER_32 == clockSourceDivider)
		);


	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	switch (selectedClockSignal){
		case CS_ACLK:
			//verify User has selected a valid clock source for ACLK
			ASSERT(
					(CS_XT1CLK_SELECT == clockSource) ||
					(CS_VLOCLK_SELECT == clockSource) ||
					(CS_DCOCLK_SELECT == clockSource) ||
					(CS_XT2CLK_SELECT == clockSource)
					);
			clockSourceDivider = clockSourceDivider << 8;
			clockSource = clockSource << 8;

			HWREG(baseAddress + OFS_CSCTL2) &= ~(SELA_7);
			HWREG(baseAddress + OFS_CSCTL2) |= (clockSource);
			HWREG(baseAddress + OFS_CSCTL3) &= ~(DIVA0 + DIVA1 + DIVA2);
			HWREG(baseAddress + OFS_CSCTL3) |= clockSourceDivider;
			break;
		case CS_SMCLK:
			ASSERT(
				(CS_XT1CLK_SELECT == clockSource) ||
				(CS_VLOCLK_SELECT == clockSource) ||
				(CS_DCOCLK_SELECT == clockSource) ||
				(CS_XT2CLK_SELECT == clockSource) ||
				(CS_LFMODOSC_SELECT== clockSource)||
				(CS_MODOSC_SELECT == clockSource)
				);

			clockSource = clockSource << 4;
			clockSourceDivider = clockSourceDivider << 4;

			HWREG(baseAddress + OFS_CSCTL2) &= ~(SELS_7);
			HWREG(baseAddress + OFS_CSCTL2) |= clockSource;
			HWREG(baseAddress + OFS_CSCTL3) &= ~(DIVS0 + DIVS1 + DIVS2);
			HWREG(baseAddress + OFS_CSCTL3) |= clockSourceDivider;
			break;
		case CS_MCLK:
			ASSERT(
			(CS_XT1CLK_SELECT == clockSource) ||
			(CS_VLOCLK_SELECT == clockSource) ||
			(CS_DCOCLK_SELECT == clockSource) ||
			(CS_XT2CLK_SELECT == clockSource) ||
			(CS_LFMODOSC_SELECT== clockSource)||
			(CS_MODOSC_SELECT == clockSource)
			);

			HWREG(baseAddress + OFS_CSCTL2) &= ~(SELM_7);
			HWREG(baseAddress + OFS_CSCTL2) |= clockSource;
			HWREG(baseAddress + OFS_CSCTL3) &= ~(DIVM0 + DIVM1 + DIVM2);
			HWREG(baseAddress + OFS_CSCTL3) |= clockSourceDivider;
			break;
	}

}

//******************************************************************************
//
//! Initializes the XT1 crystal oscillator in low frequency mode. Loops
//! until all oscillator fault flags are cleared, with no timeout. See the
//! device-specific data sheet for appropriate drive settings. IMPORTANT: User
//! must call CS_setExternalClockSource function to set frequency of external
//! clocks before calling this function is call.
//!
//! \param baseAddress is the base address of the CS module.
//! \param xt1drive is the target drive strength for the XT1 crystal oscillator.
//!         Valid values:
//!         \b CS_XT1_DRIVE0,
//!         \b CS_XT1_DRIVE1,
//!         \b CS_XT1_DRIVE2,
//!         \b CS_XT1_DRIVE3.	[Default]
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5 and \b SFRIFG1
//!
//! \return None
//
//******************************************************************************
void
CS_XT1Start ( unsigned int baseAddress,
    unsigned int xt1drive
    )
{
	ASSERT(CS_XT1ClockFrequency !=0)

    ASSERT((xt1drive == CS_XT1_DRIVE0 ) ||
        (xt1drive == CS_XT1_DRIVE1 ) ||
        (xt1drive == CS_XT1_DRIVE2 ) ||
        (xt1drive == CS_XT1_DRIVE3 ));

	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

    //If the drive setting is not already set to maximum
    //Set it to max for LFXT startup
    if ((HWREG(baseAddress + OFS_CSCTL4) & XT1DRIVE_3) != XT1DRIVE_3){
        //Highest drive setting for XT1startup
        HWREG(baseAddress + OFS_CSCTL4_L) |= XT1DRIVE1_L + XT1DRIVE0_L;
    }

	//If user is using XT1 in HF mode
	if(CS_XT1ClockFrequency>XT1_FREQUENCY_THRESHOLD){
		HWREG(baseAddress + OFS_CSCTL4) |= XTS;
		HWREG(baseAddress + OFS_CSCTL4) &= ~XT1BYPASS;
	}else{
		HWREG(baseAddress + OFS_CSCTL4) &= ~(XTS+XT1BYPASS);
	}

    //Wait for Crystal to stabilize
    while (HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG)
    {
        //Clear OSC flaut Flags fault flags
        HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG);

        //Clear OFIFG fault flag
        HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
    }


    //set requested Drive mode
    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
                                         ~(XT1DRIVE_3)
                                         ) |
                                       (xt1drive);

    //Switch ON XT1 oscillator
    HWREG(baseAddress + OFS_CSCTL4) &= ~XT1OFF;

}
//******************************************************************************
//
//! Bypasses the XT1 crystal oscillator. Loops until all oscillator fault
//! flags are cleared, with no timeout. IMPORTANT: User must call
//! CS_setExternalClockSource function to set frequency of external clocks this
//! function is call.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5, \b SFRIFG1
//! \return None
//
//******************************************************************************
void
CS_bypassXT1 ( unsigned int baseAddress
    )
{
	//Verify user has set frequency of XT1 with SetExternalClockSource
	ASSERT(CS_XT1ClockFrequency!=0)

	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	if(CS_XT1ClockFrequency>XT1_FREQUENCY_THRESHOLD){
		// Set XT1 in HF mode switch off XT1 Oscillator and enable BYPASS mode
		HWREG(baseAddress + OFS_CSCTL4) |= (XTS + XT1BYPASS +XT1OFF);
	}else {
		// Set XT1 in LF mode Switch off XT1 oscillator and enable BYPASS mode
		HWREG(baseAddress + OFS_CSCTL4) &= ~XTS;
		HWREG(baseAddress + OFS_CSCTL4) |= (XT1BYPASS + XT1OFF);
	}

		// Set XT1 in LF mode Switch off XT1 oscillator and enable BYPASS mode
	HWREG(baseAddress + OFS_CSCTL4) |= (XT1BYPASS + XT1OFF);


	//Wait until XT1 stabilizes
	while (HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG)
	{
		//Clear OSC flaut Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG);

		// Clear the global fault flag. In case the XT1 caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
	}
}

//******************************************************************************
//
//! Initializes the XT1 crystal oscillator in low frequency mode with timeout.
//! Loops until all oscillator fault flags are cleared or until a timeout
//! counter is decremented and equals to zero. See the device-specific
//! datasheet for appropriate drive settings. IMPORTANT: User
//! must call CS_setExternalClockSource function to set frequency of external
//! clocks before calling this function is call.
//!
//! \param baseAddress is the base address of the CS module.
//! \param xt1drive is the target drive strength for the XT1 crystal oscillator.
//!        Valid values are
//!         \b CS_XT1_DRIVE0,
//!         \b CS_XT1_DRIVE1,
//!         \b CS_XT1_DRIVE2,
//!         \b CS_XT1_DRIVE3	[Default Value]
//!
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5 and \b SFRIFG1
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//******************************************************************************
unsigned short
CS_XT1StartWithTimeout (
    unsigned int baseAddress,
    unsigned int xt1drive,
    unsigned long timeout
    )
{
	ASSERT(CS_XT1ClockFrequency !=0)

	ASSERT((xt1drive == CS_XT1_DRIVE0 ) ||
		(xt1drive == CS_XT1_DRIVE1 ) ||
		(xt1drive == CS_XT1_DRIVE2 ) ||
		(xt1drive == CS_XT1_DRIVE3 ));

	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	//If the drive setting is not already set to maximum
	//Set it to max for LFXT startup
	if ((HWREG(baseAddress + OFS_CSCTL4) & XT1DRIVE_3) != XT1DRIVE_3){
		//Highest drive setting for XT1startup
		HWREG(baseAddress + OFS_CSCTL4_L) |= XT1DRIVE1_L + XT1DRIVE0_L;
	}

	//If user is using XT1 in HF mode
	if(CS_XT1ClockFrequency>XT1_FREQUENCY_THRESHOLD){
		HWREG(baseAddress + OFS_CSCTL4) |= XTS;
		HWREG(baseAddress + OFS_CSCTL4) &= ~XT1BYPASS;
	}else{
	//If user is using XT1 in HF mode
		HWREG(baseAddress + OFS_CSCTL4) &= ~(XTS+XT1BYPASS);
	}

	while ((HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG);

		// Clear the global fault flag. In case the XT1 caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}

	if(timeout){

	    //set requested Drive mode
		HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
												 ~(XT1DRIVE_3)
												 ) |
											   (xt1drive);
		//Switch ON XT1 oscillator
		HWREG(baseAddress + OFS_CSCTL4) &= ~XT1OFF;

		return (STATUS_SUCCESS);
	} else   {

		return (STATUS_FAIL);
	}
}


//******************************************************************************
//
//! Bypasses the XT1 crystal oscillator with time out. Loops until all
//! oscillator fault flags are cleared or until a timeout counter is
//! decremented and equals to zero.IMPORTANT: User must call
//! CS_setExternalClockSource to set frequency of external clocks
//! before calling this function
//!
//! \param baseAddress is the base address of the CS module.
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//******************************************************************************
unsigned short
CS_bypassXT1WithTimeout (
    unsigned int baseAddress,
    unsigned long timeout
    )
{
    ASSERT(CS_XT1ClockFrequency !=0);

	if(CS_XT1ClockFrequency>XT1_FREQUENCY_THRESHOLD){
		// Set XT1 in HF mode switch off XT1 Oscillator and enable BYPASS mode
		HWREG(baseAddress + OFS_CSCTL4) |= (XTS + XT1BYPASS +XT1OFF);
	}else {
		// Set XT1 in LF mode Switch off XT1 oscillator and enable BYPASS mode
		HWREG(baseAddress + OFS_CSCTL4) &= ~XTS;
		HWREG(baseAddress + OFS_CSCTL4) |= (XT1BYPASS + XT1OFF);
	}

	while ((HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG);

		// Clear the global fault flag. In case the XT1 caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}



    if (timeout){
        return (STATUS_SUCCESS);
    } else {
        return (STATUS_FAIL);
    }
}

//******************************************************************************
//
//! Stops the XT1 oscillator using the XT1OFF bit.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL4
//!
//! \return NONE
//
//******************************************************************************
void
CS_XT1Off (unsigned int baseAddress)
{
    //Switch off XT1 oscillator
    HWREG(baseAddress + OFS_CSCTL4) |= XT1OFF;
}

//******************************************************************************
//
//! Initializes the XT2 crystal oscillator, which supports crystal frequencies
//! between 4 MHz and 32 MHz, depending on the selected drive strength. Loops
//! until all oscillator fault flags are cleared, with no timeout. See the
//! device-specific data sheet for appropriate drive settings. NOTE: User must
//! call CS_setExternalClockSource to set frequency of external clocks
//! before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//! \param xt2drive is the target drive strength for the XT2 crystal oscillator.
//!      Valid values are
//!     \b CS_XT2DRIVE_4MHZ_8MHZ,
//!     \b CS_XT2DRIVE_8MHZ_16MHZ,
//!     \b CS_XT2DRIVE_16MHZ_24MHZ,
//!     \b CS_XT2DRIVE_24MHZ_32MHZ. [Default]
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return NONE
//
//******************************************************************************
void
CS_XT2Start (  unsigned int baseAddress,
    unsigned int xt2drive
    )
{
	ASSERT(CS_XT2ClockFrequency !=0)

    ASSERT((xt2drive == CS_XT2DRIVE_4MHZ_8MHZ  ) ||
    	   (xt2drive == CS_XT2DRIVE_8MHZ_16MHZ ) ||
           (xt2drive == CS_XT2DRIVE_16MHZ_24MHZ )||
           (xt2drive == CS_XT2DRIVE_24MHZ_32MHZ ));

    //Disable XT2BYPASS mode and Switch on XT2 oscillator
    HWREG(baseAddress + OFS_CSCTL4) &= ~XT2BYPASS;

    while (HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG){
     //Clear OSC flaut Flags
     HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT2OFFG);

     //Clear OFIFG fault flag
     HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
    }

    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
                                         ~(CS_XT2DRIVE_24MHZ_32MHZ)
                                         ) | (xt2drive);

    HWREG(baseAddress + OFS_CSCTL4) &= ~XT2OFF;

}

//******************************************************************************
//
//! Bypasses the XT2 crystal oscillator, which supports crystal frequencies
//! between 4 MHz and 32 MHz. Loops until all oscillator fault flags are
//! cleared, with no timeout. NOTE: User must call CS_setExternalClockSource
//! to set frequency of external clocks before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return NONE
//
//******************************************************************************
void
CS_bypassXT2 (  unsigned int baseAddress )
{
	//Verify user has initialized value of XT2Clock
	ASSERT(CS_XT2ClockFrequency !=0)

    //Switch off XT2 oscillator and set it to BYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) |= ( XT2BYPASS + XT2OFF );

	 while (HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG){
     //Clear OSC fault Flags
     HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT2OFFG);

     //Clear OFIFG fault flag
     HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
    }
}
//******************************************************************************
//
//! Initializes the XT2 crystal oscillator, which supports crystal frequencies
//! between 4 MHz and 32 MHz, depending on the selected drive strength. Loops
//! until all oscillator fault flags are cleared or until a timeout counter is
//! decremented and equals to zero. See the device-specific data sheet for
//! appropriate drive settings. NOTE: User must call CS_setExternalClockSource
//! to set frequency of external clocks before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//! \param xt2drive is the target drive strength for the XT2 crystal oscillator.
//!        Valid values are
//!        \b CS_XT2_4MHZ_8MHZ,
//!        \b CS_XT2_8MHZ_16MHZ,
//!        \b CS_XT2_16MHZ_24MHZ
//!        \b CS_XT2_24MHZ_32MHZ [Default Value]
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//******************************************************************************
unsigned short
CS_XT2StartWithTimeout ( unsigned int baseAddress,
    unsigned int xt2drive,
    unsigned long timeout
    )
{
	//Verify user has initialized value of XT2Clock
	ASSERT(CS_XT2ClockFrequency !=0)

	// Disable XT2BYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) &= ~XT2BYPASS;

	while ((HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT2OFFG);

		// Clear the global fault flag. In case the XT1 caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}

    if (timeout){
		//Set drive strength for XT2
	    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
	                                         ~(CS_XT2DRIVE_24MHZ_32MHZ)
	                                         ) |
	                                       (xt2drive);

	    //Switch on XT2 oscillator
	    HWREG(baseAddress + OFS_CSCTL4) &= ~XT2OFF;

        return (STATUS_SUCCESS);
    } else   {
        return (STATUS_FAIL);
    }
}

//******************************************************************************
//
//! Bypasses the XT2 crystal oscillator, which supports crystal frequencies
//! between 4 MHz and 32 MHz. Loops until all oscillator fault flags are
//! cleared or until a timeout counter is decremented and equals to zero.
//! NOTE: User must call CS_setExternalClockSource to set frequency of external
//! clocks before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1G
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//******************************************************************************
unsigned short
CS_bypassXT2WithTimeout ( unsigned int baseAddress,
    unsigned long timeout
    )
{
	//Verify user has initialized value of XT2Clock
	ASSERT(CS_XT2ClockFrequency !=0)

	//Switch off XT2 oscillator and enable BYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) |= (XT2BYPASS + XT2OFF );


	while ((HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT2OFFG);

		// Clear the global fault flag. In case the XT1 caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}

    if (timeout){
        return (STATUS_SUCCESS);
    } else   {
        return (STATUS_FAIL);
    }
}


//******************************************************************************
//
//! Stops the XT2 oscillator using the XT2OFF bit.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL4
//!
//! \return NONE
//
//******************************************************************************
void
CS_XT2Off (unsigned int baseAddress)
{
    //Switch off XT2 oscillator
    HWREG(baseAddress + OFS_CSCTL4) |= XT2OFF;
}

//******************************************************************************
//
//! Enables conditional module requests
//!
//! \param baseAddress is the base address of the CS module.
//! \param selectClock selects specific request enables. Valid values are
//!        \b CS_ACLK,
//!        \b CS_SMCLK,
//!        \b CS_MCLK,
//!        \b CS_MODOSC
//!
//! Modified registers are \b CSCTL6
//!
//! \return NONE
//
//******************************************************************************
void
CS_enableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    )
{
	ASSERT(
			(CS_ACLK  == selectClock )||
			(CS_SMCLK == selectClock )||
			(CS_MCLK  == selectClock )||
			(CS_MODOSC== selectClock ));

    HWREGB(baseAddress + OFS_CSCTL6) |= selectClock;
}

//******************************************************************************
//
//! Disables conditional module requests
//!
//! \param baseAddress is the base address of the CS module.
//! \param selectClock selects specific request enables. Valid values are
//!        \b CS_ACLK,
//!        \b CS_SMCLK,
//!        \b CS_MCLK,
//!        \b CS_MODOSC
//!
//! Modified registers are \b CSCTL6
//!
//! \return NONE
//
//******************************************************************************
void
CS_disableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    )
{
	ASSERT(
			(CS_ACLK  == selectClock )||
			(CS_SMCLK == selectClock )||
			(CS_MCLK  == selectClock )||
			(CS_MODOSC== selectClock ));

    HWREGB(baseAddress + OFS_CSCTL6) &= ~selectClock;
}

//******************************************************************************
//
//! Gets the current CS fault flag status.
//!
//! \param baseAddress is the base address of the CS module.
//! \param mask is the masked interrupt flag status to be returned.
//!      Mask parameter can be either any of the following selection.
//!         - \b CS_XT2OFFG - XT2 oscillator fault flag
//!         - \b CS_XT1OFFG - XT1 oscillator fault flag (HF mode)
//! Modified registers are \b CSCTL5
//!
//! \return The current flag status for the corresponding masked bit
//
//******************************************************************************
unsigned char
CS_faultFlagStatus (
    unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(mask <= CS_XT2OFFG );
    return (HWREGB(baseAddress + OFS_CSCTL5) & mask);
}

//******************************************************************************
//
//! Clears the current CS fault flag status for the masked bit.
//!
//! \param baseAddress is the base address of the CS module.
//! \param mask is the masked interrupt flag status to be returned.
//!         mask parameter can be any one of the following
//!         - \b CS_XT2OFFG - XT2 oscillator fault flag
//!         - \b CS_XT1OFFG - XT1 oscillator fault flag (HF mode)
//!
//! Modified registers are \b CSCTL5
//!
//! \return NONE
//
//******************************************************************************
void
CS_clearFaultFlag (
    unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(mask <= CS_XT2OFFG );
    HWREGB(baseAddress + OFS_CSCTL5) &= ~mask;
}

//******************************************************************************
//
//Compute the clock frequency when clock is source from DCO
//
//\param baseAddress is the base address of the CS module.
//\param clockdivider is clock source for FLL reference. Valid values are:
//           \b CS_CLOCK_DIVIDER_1,
//           \b CS_CLOCK_DIVIDER_2,
//           \b CS_CLOCK_DIVIDER_4,
//           \b CS_CLOCK_DIVIDER_8,
//           \b CS_CLOCK_DIVIDER_16,
//           \b CS_CLOCK_DIVIDER_32
//
//\return Calculated clock frequency in Hz
//
//******************************************************************************
unsigned long
privateCSSourceClockFromDCO ( unsigned int baseAddress,
		unsigned char clockdivider)
{
    unsigned long CLKFrequency;

	if (HWREG(baseAddress + OFS_CSCTL1)& 0x0080) {
		switch (HWREG(baseAddress + OFS_CSCTL1)& DCOFSEL_3) {
			case DCOFSEL_3:
				CLKFrequency=CS_DCO_FREQ_6/clockdivider;
				break;
			case DCOFSEL_2:
				CLKFrequency=CS_DCO_FREQ_5/clockdivider;
				break;
			default:
				CLKFrequency=CS_DCO_FREQ_4/clockdivider;
				break;
		}

	} else {
		switch (HWREG(baseAddress + OFS_CSCTL1)& DCOFSEL_3) {
			case DCOFSEL_3:
				CLKFrequency=CS_DCO_FREQ_3/clockdivider;
				break;
			case DCOFSEL_2:
				CLKFrequency=CS_DCO_FREQ_2/clockdivider;
				break;
			default:
				CLKFrequency=CS_DCO_FREQ_1/clockdivider;
				break;

		}
	}

    return (CLKFrequency);
}


//******************************************************************************
//
//Compute the clock frequency given the clock source and divider
//
//\param baseAddress is the base address of the CS module.
//\param CLKSource is the clock source. Valid values are:
//		\b SELM__XT1CLK (SELM__LFCLK),
//		\b SELM__VLOCLK,
//		\b SELM__XT2CLK (SELM__HFCLK),
//		\b SELM__DCOCLK,
//		\b SELM__XT2CLK (SELM__HFCLK),
//		\b SELM__DCOCLK,
//\param CLKSourceDivider is the Clock source divider
//
//\return Calculated clock frequency in Hz
//
//******************************************************************************
unsigned long
privateCSComputeCLKFrequency ( unsigned int baseAddress,
    unsigned int CLKSource,
    unsigned int CLKSourceDivider
    )
{
    unsigned long CLKFrequency;
    unsigned char CLKSourceFrequencyDivider = 1;
    unsigned char i = 0;

    // Determine Frequency divider
    for ( i = 0; i < CLKSourceDivider; i++){
        CLKSourceFrequencyDivider *= 2;
    }

    // Determine clock source based on CLKSource
    switch (CLKSource){

    	// If XT1 is selected as clock source
        case SELM__XT1CLK:
            CLKFrequency = (CS_XT1ClockFrequency /
                            CLKSourceFrequencyDivider);

            //Check if XT1OFFG is not set. If fault flag is set
            //VLO is used as source clock
            if (HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG){
                HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG);
                //Clear OFIFG fault flag
                HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

                if (HWREGB(baseAddress + OFS_CSCTL5) & XT1OFFG){
                	CLKFrequency = CS_VLOCLK_FREQUENCY;
                }
            }
            break;

        case SELM__VLOCLK:
            CLKFrequency =
                (CS_VLOCLK_FREQUENCY / CLKSourceFrequencyDivider);
            break;

        case SELM__DCOCLK:
        	CLKFrequency =
        	privateCSSourceClockFromDCO(baseAddress, CLKSourceFrequencyDivider);

            break;

        case SELM__XT2CLK:
            CLKFrequency =
                (CS_XT2ClockFrequency / CLKSourceFrequencyDivider);

            if (HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG){

              HWREGB(baseAddress + OFS_CSCTL5) &=  ~XT2OFFG;
              //Clear OFIFG fault flag
              HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
            }

            if (HWREGB(baseAddress + OFS_CSCTL5) & XT2OFFG){
                CLKFrequency = CS_MODCLK_FREQUENCY;
            }
            break;
    }

    return (CLKFrequency) ;
}

//******************************************************************************
//
//! Get the current ACLK frequency.
//!
//! If a oscillator fault is set, the frequency returned will be based on the
//! fail safe mechanism of CS module. The user of this API must ensure that
//! CS_externalClockSourceInit API was invoked before in case XT1 or
//! XT2 is being used. User must call CS_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current ACLK frequency in Hz
//
//******************************************************************************
unsigned long
CS_getACLK (unsigned int baseAddress)
{
	//Find ACLK source
	unsigned int ACLKSource = (HWREG(baseAddress + OFS_CSCTL2) & SELA_7);
	ACLKSource = ACLKSource >> 8;

	//Find ACLK frequency divider
	unsigned int ACLKSourceDivider =  HWREG(baseAddress + OFS_CSCTL3) & SELA_7;
	ACLKSourceDivider = ACLKSourceDivider >>8;

	return (privateCSComputeCLKFrequency(baseAddress,
					ACLKSource,
					ACLKSourceDivider));
}

//******************************************************************************
//
//! Get the current SMCLK frequency.
//!
//! If a oscillator fault is set, the frequency returned will be based on the
//! fail safe mechanism of CS module. The user of this API must ensure that
//! CS_externalClockSourceInit API was invoked before in case XT1 or
//! XT2 is being used. User must call CS_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current SMCLK frequency in Hz
//
//******************************************************************************
unsigned long
CS_getSMCLK (unsigned int baseAddress)
{
	//Find SMCLK source
	unsigned int SMCLKSource = HWREGB(baseAddress + OFS_CSCTL2) & SELS_7 ;

	SMCLKSource = SMCLKSource >> 4;

	//Find SMCLK frequency divider
	unsigned int SMCLKSourceDivider = HWREG(baseAddress + OFS_CSCTL3) & SELS_7;
	SMCLKSourceDivider = SMCLKSourceDivider >> 4;

	return (privateCSComputeCLKFrequency(baseAddress,
				SMCLKSource,
				SMCLKSourceDivider )
			);
}

//******************************************************************************
//
//! Get the current MCLK frequency.
//!
//! If a oscillator fault is set, the frequency returned will be based on the
//! fail safe mechanism of CS module. The user of this API must ensure that
//! CS_externalClockSourceInit API was invoked before in case XT1 or
//! XT2 is being used. User must call CS_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current MCLK frequency in Hz
//
//******************************************************************************
unsigned long
CS_getMCLK (unsigned int baseAddress)
{

	//Find MCLK source
	unsigned int MCLKSource = (HWREG(baseAddress + OFS_CSCTL2) & SELM_7);
	//Find MCLK frequency divider
	unsigned int MCLKSourceDivider =  HWREG(baseAddress + OFS_CSCTL3) & SELM_7;

	return (privateCSComputeCLKFrequency(baseAddress,
				MCLKSource,
				MCLKSourceDivider )
			);
}

//******************************************************************************
//
//! Clears all the Oscillator Flags
//!
//! \param baseAddress is the base address of the CS module.
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL5, \b SFRIFG1G
//!
//! \return the mask of the oscillator flag status.
//
//******************************************************************************
unsigned int CS_clearAllOscFlagsWithTimeout(unsigned int baseAddress,
                                             unsigned long timeout
                                             )
{
    do {
      // Clear all osc fault flags
      HWREGB(baseAddress + OFS_CSCTL5) &= ~(XT1OFFG + XT2OFFG );

      // Clear the global osc fault flag.
      HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

      // Check XT1 fault flags
    } while ((HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1)) && --timeout);

    return (HWREGB(baseAddress + OFS_CSCTL5) & (XT1OFFG + XT2OFFG));
}

//******************************************************************************
//
//! Set DCO frequency
//!
//! Before invoking the this function, the user must call CS_init.
//!
//! \param baseAddress is the base address of the CS module.
//! \param dcofsel selects valid frequency options based on dco frequency range
//!			selection (dcorsel). Valid options are:
//!			\e For FR57xx devices:
//!			\b CS_DCOFSEL_0 - Low frequency option 5.33MHZ. High frequency
//!				option 16MHz. 
//!			\b CS_DCOFSEL_1 - Low frequency option 5.33MHZ. High frequency
//!				option 16MHz. 
//!			\b CS_DCOFSEL_2 - Low frequency option 6.67MHZ. High frequency
//!				option 20MHz. 
//!			\b CS_DCOFSEL_3 - Low frequency option 8MHZ. High frequency
//!				option 24MHz. [Default]
//!			\e For FR58xx/FR59xx devices:
//!			\b CS_DCOFSEL_0 - Low frequency option 1 MHZ. High frequency
//!				option 1 MHz.
//!			\b CS_DCOFSEL_1 - Low frequency option 2.67 MHZ. High frequency
//!				option 5.33 MHz.
//!			\b CS_DCOFSEL_2 Low frequency option 3.33 MHZ. High frequency
//!				option 6.67 MHz.
//!			\b CS_DCOFSEL_3 Low frequency option 4 MHZ. High frequency
//!				option 8 MHz.
//!			\b CS_DCOFSEL_4 Low frequency option 5.33 MHZ. High frequency
//!				option 16 MHz.
//!\param dcorsel selects frequency range option. Valid options are:
//!			\b CS_DCORSEL_0	[Default]
//!			\b CS_DCORSEL_1
//!
//! \return NONE
//
//******************************************************************************
void
CS_setDCOFreq(
		unsigned int baseAddress,
		unsigned int dcorsel,
		unsigned int dcofsel)
{


	//Verify User has selected a valid frequency option
	ASSERT(
			(dcofsel==CS_DCOFSEL_0)||
			(dcofsel==CS_DCOFSEL_1)||
			(dcofsel==CS_DCOFSEL_2)||
			(dcofsel==CS_DCOFSEL_3));
	//Verify User has selected a valid frequency option

	//Verify user has selected a valid DCO Frequency Range option
	ASSERT(
			(dcorsel==CS_DCORSEL_0)||
			(dcorsel==CS_DCORSEL_1));

	//Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	// Set user's frequency selection for DCO
	HWREG(baseAddress + OFS_CSCTL1) = (dcorsel + dcofsel);

}

//******************************************************************************
//
//Close the Doxygen group.
//! @}
//
//******************************************************************************

