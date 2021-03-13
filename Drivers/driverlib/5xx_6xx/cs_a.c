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
#include "driverlib/5xx_6xx/cs_a.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
#include "driverlib/5xx_6xx/debug.h"
#include "inc/sfr_sys_baseAddress.h"


//******************************************************************************
//
// LFXT for FR58xx/FR59xx crystal frequency. Should be set with
//CSA_externalClockSourceInit if LFXT is used and user intends to invoke
//CSA_getSMCLK, CSA_getMCLK, CSA_getACLK and
//CSA_LFXTStart, CSA_LFXTByPass, CSA_LFXTStartWithTimeout,
//CSA_LFXTByPassWithTimeout.
//
//******************************************************************************
unsigned long CSA_LFXTClockFrequency = 0;

//******************************************************************************
//
//The HFXT (or HFXT for FR58xx/FR59xx) crystal frequency. Should be set with
//CSA_externalClockSourceInit if HFXT is used and user intends to invoke
//CSA_getSMCLK, CSA_getMCLK, CSA_getACLK,
//CSA_LFXTStart, CSA_LFXTByPass, CSA_LFXTStartWithTimeout,
//CSA_LFXTByPassWithTimeout.
//
//******************************************************************************
unsigned long CSA_HFXTClockFrequency = 0;

//******************************************************************************
//
//! This function sets the external clock sources LFXT and HFXT crystal
//! oscillator frequency values. This function must be called if an external
//! crystal LFXT or HFXT is used and the user intends to call
//! CSA_getMCLK, CSA_getSMCLK, CSA_getACLK and
//! CSA_LFXTStart, CSA_LFXTByPass, CSA_LFXTStartWithTimeout,
//! CSA_LFXTByPassWithTimeout, CSA_HFXTStart, CSA_HFXTByPass,
//! CSA_HFXTStartWithTimeout, CSA_HFXTByPassWithTimeout.
//!
//! \param baseAddress is the base address of the CSA module.
//! \param LFXTCLK_frequency is the LFXT crystal frequencies in Hz
//! \param HFXTCLK_frequency is the HFXT crystal frequencies in Hz
//!
//! \return None
//
//******************************************************************************
void
CSA_setExternalClockSource (unsigned int baseAddress,
    unsigned long LFXTCLK_frequency,
    unsigned long HFXTCLK_frequency
    )
{
    CSA_LFXTClockFrequency = LFXTCLK_frequency;
    CSA_HFXTClockFrequency = HFXTCLK_frequency;
}


//******************************************************************************
//
//! This function initializes each of the clock signals. The user must ensure
//! that this function is called for each clock signal. If not, the default
//! state is assumed for the particular clock signal. Refer MSP430ware
//! documentation for CSA module or Device Family User's Guide for details of
//! default clock signal states.
//!
//! \param baseAddress is the base address of the CSA module.
//! \param selectedClockSignal - Valid values are
//!           \b CSA_ACLK,
//!           \b CSA_MCLK,
//!           \b CSA_SMCLK,
//! \param clockSource is Clock source for the selectedClock Signal
//!            \b CSA_LFXTCLK_SELECT,
//!            \b CSA_VLOCLK_SELECT,
//!            \b CSA_DCOCLK_SELECT,	[Not available for ACLK]
//!            \b CSA_HFXTCLK_SELECT,	[Not available for ACLK]
//!            \b CSA_LFMODOSC_SELECT,
//!            \b CSA_MODOSC_SELECT		[Not available for ACLK]
//! \param clockSourceDivider - selected the clock divider to calculate
//!         clock signal from clock source. Valid values are
//!           \b CSA_CLOCK_DIVIDER_1,	[Default for ACLK]
//!           \b CSA_CLOCK_DIVIDER_2,
//!           \b CSA_CLOCK_DIVIDER_4,
//!           \b CSA_CLOCK_DIVIDER_8,	[Default for SMCLK and MCLK]
//!           \b CSA_CLOCK_DIVIDER_16,
//!           \b CSA_CLOCK_DIVIDER_32
//!
//! Modified registers are \b CSCTL0, \b CSCTL2, \b CSCTL3
//!
//! \return NONE
//
//******************************************************************************
void
CSA_clockSignalInit ( unsigned int baseAddress,
    unsigned char selectedClockSignal,
    unsigned int clockSource,
    unsigned int clockSourceDivider
    )
{

	//Verify User has selected a valid Frequency divider
	ASSERT(
			(CSA_CLOCK_DIVIDER_1 == clockSourceDivider) ||
			(CSA_CLOCK_DIVIDER_2 == clockSourceDivider) ||
			(CSA_CLOCK_DIVIDER_4 == clockSourceDivider) ||
			(CSA_CLOCK_DIVIDER_8 == clockSourceDivider) ||
			(CSA_CLOCK_DIVIDER_16 == clockSourceDivider) ||
			(CSA_CLOCK_DIVIDER_32 == clockSourceDivider)
			);


		// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	switch (selectedClockSignal){
		case CSA_ACLK:
				ASSERT(
					(CSA_LFXTCLK_SELECT == clockSource)  ||
					(CSA_VLOCLK_SELECT == clockSource)   ||
					(CSA_LFMODOSC_SELECT == clockSource) ||
					);

			clockSourceDivider = clockSourceDivider << 8;
			clockSource = clockSource << 8;

			HWREG(baseAddress + OFS_CSCTL2) &= ~(SELA_7);
			HWREG(baseAddress + OFS_CSCTL2) |= (clockSource);
			HWREG(baseAddress + OFS_CSCTL3) &= ~(DIVA0 + DIVA1 + DIVA2);
			HWREG(baseAddress + OFS_CSCTL3) |= clockSourceDivider;
			break;
		case CSA_SMCLK:
			ASSERT(
				(CSA_LFXTCLK_SELECT == clockSource) ||
				(CSA_VLOCLK_SELECT == clockSource) ||
				(CSA_DCOCLK_SELECT == clockSource) ||
				(CSA_HFXTCLK_SELECT == clockSource) ||
				(CSA_LFMODOSC_SELECT== clockSource)||
				(CSA_MODOSC_SELECT == clockSource)
				);

			clockSource = clockSource << 4;
			clockSourceDivider = clockSourceDivider << 4;

			HWREG(baseAddress + OFS_CSCTL2) &= ~(SELS_7);
			HWREG(baseAddress + OFS_CSCTL2) |= clockSource;
			HWREG(baseAddress + OFS_CSCTL3) &= ~(DIVS0 + DIVS1 + DIVS2);
			HWREG(baseAddress + OFS_CSCTL3) |= clockSourceDivider;
			break;
		case CSA_MCLK:
			ASSERT(
			(CSA_LFXTCLK_SELECT == clockSource) ||
			(CSA_VLOCLK_SELECT == clockSource) ||
			(CSA_DCOCLK_SELECT == clockSource) ||
			(CSA_HFXTCLK_SELECT == clockSource) ||
			(CSA_LFMODOSC_SELECT== clockSource)||
			(CSA_MODOSC_SELECT == clockSource)
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
//! Initializes the LFXT crystal oscillator in low frequency mode. Loops
//! until all oscillator fault flags are cleared, with no timeout. See the
//! device-specific data sheet for appropriate drive settings. IMPORTANT: User
//! must call CSA_setExternalClockSource function to set frequency of external
//! clocks this function is call.
//!
//! \param baseAddress is the base address of the CS module.
//! \param lfxtdrive is the target drive strength for the LFXT crystal oscillator.
//!         Valid values:
//!         \b CSA_LFXT_DRIVE0,
//!         \b CSA_LFXT_DRIVE1,
//!         \b CSA_LFXT_DRIVE2,
//!         \b CSA_LFXT_DRIVE3.	[Default]
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5 and \b SFRIFG1
//!
//! \return None
//
//******************************************************************************
void
CSA_LFXTStart ( unsigned int baseAddress,
    unsigned int lfxtdrive
    )
{
	ASSERT(CSA_LFXTClockFrequency !=0)

	    ASSERT((lfxtdrive == CSA_LFXT_DRIVE0 ) ||
	        (lfxtdrive == CSA_LFXT_DRIVE1 ) ||
	        (lfxtdrive == CSA_LFXT_DRIVE2 ) ||
	        (lfxtdrive == CSA_LFXT_DRIVE3 ));

		// Unlock CS control register
		HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	    //If the drive setting is not already set to maximum
	    //Set it to max for LFXT startup
	    if ((HWREG(baseAddress + OFS_CSCTL4) & LFXTDRIVE_3) != LFXTDRIVE_3){
	        //Highest drive setting for LFXTstartup
	        HWREG(baseAddress + OFS_CSCTL4_L) |= LFXTDRIVE1_L + LFXTDRIVE0_L;
	    }

	    HWREG(baseAddress + OFS_CSCTL4) &= ~LFXTBYPASS;

	    //Wait for Crystal to stabilize
	    while (HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG)
	    {
	        //Clear OSC flaut Flags fault flags
	        HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG);

	        //Clear OFIFG fault flag
	        HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
	    }


	    //set requested Drive mode
	    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
	                                         ~(LFXTDRIVE_3)
	                                         ) |
	                                       (lfxtdrive);

	    //Switch ON LFXT oscillator
	    HWREG(baseAddress + OFS_CSCTL4) &= ~LFXTOFF;

}

//******************************************************************************
//
//! Bypasses the LFXT crystal oscillator. Loops until all oscillator fault
//! flags are cleared, with no timeout. IMPORTANT: User must call
//! CSA_setExternalClockSource function to set frequency of external clocks this
//! function is call.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5, \b SFRIFG1
//! \return None
//
//******************************************************************************
void
CSA_bypassLFXT ( unsigned int baseAddress
    )
{
	//Verify user has set frequency of LFXT with SetExternalClockSource
	ASSERT(CSA_LFXTClockFrequency!=0)

	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;


	ASSERT(CSA_LFXTClockFrequency<LFXT_FREQUENCY_THRESHOLD);

	// Set LFXT in LF mode Switch off LFXT oscillator and enable BYPASS mode
	HWREG(baseAddress + OFS_CSCTL4) |= (LFXTBYPASS + LFXTOFF);

	//Wait until LFXT stabilizes
	while (HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG)
	{
		//Clear OSC flaut Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG);

		// Clear the global fault flag. In case the LFXT caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
	}
}

//******************************************************************************
//
//! Initializes the LFXT crystal oscillator in low frequency mode with timeout.
//! Loops until all oscillator fault flags are cleared or until a timeout
//! counter is decremented and equals to zero. See the device-specific
//! datasheet for appropriate drive settings. IMPORTANT: User
//! must call CSA_setExternalClockSource to set frequency of external clocks
//! before calling this function
//!
//! \param baseAddress is the base address of the CS module.
//! \param lfxtdrive is the target drive strength for the LFXT crystal oscillator.
//!         Valid values are
//!         \b CSA_LFXTDRIVE0,
//!         \b CSA_LFXTDRIVE1,
//!         \b CSA_LFXTDRIVE2,
//!         \b CSA_LFXTDRIVE3.	[Default]
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL0, \b CSCTL4, \b CSCTL5 and \b SFRIFG1
//!
//! \return None
//
//******************************************************************************
unsigned short
CSA_LFXTStartWithTimeout (
		unsigned int baseAddress,
		unsigned int lfxtdrive,
		unsigned long timeout
    )
{
	ASSERT(CSA_LFXTClockFrequency !=0)

	ASSERT((lfxtdrive == CSA_LFXT_DRIVE0 ) ||
		(lfxtdrive == CSA_LFXT_DRIVE1 ) ||
		(lfxtdrive == CSA_LFXT_DRIVE2 ) ||
		(lfxtdrive == CSA_LFXT_DRIVE3 ));

	// Unlock CS control register
	HWREG(baseAddress + OFS_CSCTL0) = CSKEY;

	//If the drive setting is not already set to maximum
	//Set it to max for LFXT startup
	if ((HWREG(baseAddress + OFS_CSCTL4) & LFXTDRIVE_3) != LFXTDRIVE_3){
		//Highest drive setting for LFXTstartup
		HWREG(baseAddress + OFS_CSCTL4_L) |= LFXTDRIVE1_L + LFXTDRIVE0_L;
	}

	HWREG(baseAddress + OFS_CSCTL4) &= ~LFXTBYPASS;

	while ((HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG);

		// Clear the global fault flag. In case the LFXT caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}

	if(timeout){

	    //set requested Drive mode
		HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
												 ~(LFXTDRIVE_3)
												 ) |
											   (lfxtdrive);
		//Switch ON LFXT oscillator
		HWREG(baseAddress + OFS_CSCTL4) &= ~LFXTOFF;

		return (STATUS_SUCCESS);
	} else   {

		return (STATUS_FAIL);
	}
}


//******************************************************************************
//
//! Bypasses the LFXT crystal oscillator with time out. Loops until all
//! oscillator fault flags are cleared or until a timeout counter is
//! decremented and equals to zero. NOTE: User must call
//! CSA_setExternalClockSource to set frequency of external clocks
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
CSA_bypassLFXTWithTimeout (
    unsigned int baseAddress,
    unsigned long timeout
    )
{
    ASSERT(CSA_LFXTClockFrequency !=0);

	ASSERT(CSA_LFXTClockFrequency<LFXT_FREQUENCY_THRESHOLD);

	// Set LFXT in LF mode Switch off LFXT oscillator and enable BYPASS mode
	HWREG(baseAddress + OFS_CSCTL4) |= (LFXTBYPASS + LFXTOFF);

	while ((HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG);

		// Clear the global fault flag. In case the LFXT caused the global fault
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
//! Stops the LFXT oscillator using the LFXTOFF bit.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL4
//!
//! \return NONE
//
//******************************************************************************
void
CSA_LFXTOff (unsigned int baseAddress)
{
    //Switch off LFXT oscillator
    HWREG(baseAddress + OFS_CSCTL4) |= LFXTOFF;
}
//******************************************************************************
//
//! Initializes the HFXT crystal oscillator, which supports crystal frequencies
//! between 0 MHz and 24 MHz, depending on the selected drive strength. Loops
//! until all oscillator fault flags are cleared, with no timeout. See the
//! device-specific data sheet for appropriate drive settings. NOTE: User must
//! call CSA_setExternalClockSource to set frequency of external clocks
//! before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//! \param hfxtdrive is the target drive strength for the HFXT crystal oscillator.
//!      Valid values are
//!     \b CSA_HFXTDRIVE_0,
//!     \b CSA_HFXTDRIVE_1,
//!     \b CSA_HFXTDRIVE_2,
//!     \b CSA_HFXTDRIVE_3	[Default]
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return NONE
//
//******************************************************************************
void
CSA_HFXTStart (  unsigned int baseAddress,
    unsigned int hfxtdrive
    )
{
	ASSERT(CSA_HFXTClockFrequency !=0)

    ASSERT((hfxtdrive == CSA_HFXTDRIVE_4MHZ_8MHZ  ) ||
    	   (hfxtdrive == CSA_HFXTDRIVE_8MHZ_16MHZ ) ||
           (hfxtdrive == CSA_HFXTDRIVE_16MHZ_24MHZ )||
           (hfxtdrive == CSA_HFXTDRIVE_24MHZ_32MHZ ));

    //Disable HFXTBYPASS mode and Switch on HFXT oscillator
    HWREG(baseAddress + OFS_CSCTL4) &= ~HFXTBYPASS;

	//If HFFrequency is 16MHz or above
	if (CSA_HFXTClockFrequency>16000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_3;
	}
	//If HFFrequency is between 8MHz and 16MHz
	else if (CSA_HFXTClockFrequency>8000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_2;
	}
	//If HFFrequency is between 0MHz and 4MHz
	else if (CSA_HFXTClockFrequency<4000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_0;
	}
	//If HFFrequency is between 4MHz and 8MHz
	else{
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_1;
	}

    while (HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG){
     //Clear OSC flaut Flags
     HWREGB(baseAddress + OFS_CSCTL5) &= ~(HFXTOFFG);

     //Clear OFIFG fault flag
     HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
    }

    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
                                         ~(CSA_HFXTDRIVE_24MHZ_32MHZ)
                                         ) |
                                       (hfxtdrive);

    HWREG(baseAddress + OFS_CSCTL4) &= ~HFXTOFF;

}

//******************************************************************************
//
//! Bypasses the HFXT crystal oscillator, which supports crystal frequencies
//! between 0 MHz and 24 MHz. Loops until all oscillator fault flags are
//! cleared, with no timeout.NOTE: User must call CSA_setExternalClockSource
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
CSA_bypassHFXT (  unsigned int baseAddress )
{
	//Verify user has initialized value of HFXTClock
	ASSERT(CSA_HFXTClockFrequency !=0)

    //Switch off HFXT oscillator and set it to BYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) |= ( HFXTBYPASS + HFXTOFF );


	//Set correct HFFREQ bit for FR58xx/FR59xx devices

	//If HFFrequency is 16MHz or above
	if (CSA_HFXTClockFrequency>16000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_3;
	}
	//If HFFrequency is between 8MHz and 16MHz
	else if (CSA_HFXTClockFrequency>8000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_2;
	}
	//If HFFrequency is between 0MHz and 4MHz
	else if (CSA_HFXTClockFrequency<4000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_0;
	}
	//If HFFrequency is between 4MHz and 8MHz
	else{
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_1;
	}

    while (HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG){
     //Clear OSC fault Flags
     HWREGB(baseAddress + OFS_CSCTL5) &= ~(HFXTOFFG);

     //Clear OFIFG fault flag
     HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
    }
}
//******************************************************************************
//
//! Initializes the HFXT crystal oscillator, which supports crystal frequencies
//! between 0 MHz and 24 MHz, depending on the selected drive strength. Loops
//! until all oscillator fault flags are cleared or until a timeout counter is
//! decremented and equals to zero. See the device-specific data sheet for
//! appropriate drive settings. NOTE: User must call CSA_setExternalClockSource
//! to set frequency of external clocks before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//! \param hfxtdrive is the target drive strength for the HFXT crystal oscillator.
//!        Valid values are
//!        \b CSA_HFXTDRIVE_0,
//!        \b CSA_HFXTDRIVE_1,
//!        \b CSA_HFXTDRIVE_2,
//!        \b CSA_HFXTDRIVE_3	[Default]
//! \param timeout is the count value that gets decremented every time the loop
//!         that clears oscillator fault flags gets executed.
//!
//! Modified registers are \b CSCTL4, \b CSCTL5, \b SFRIFG1
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//******************************************************************************
unsigned short
CSA_HFXTStartWithTimeout ( unsigned int baseAddress,
    unsigned int hfxtdrive,
    unsigned long timeout
    )
{
	//Verify user has initialized value of HFXTClock
	ASSERT(CSA_HFXTClockFrequency !=0)

	// Disable HFXTBYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) &= ~HFXTBYPASS;

	//Set correct HFFREQ bit for FR58xx/FR59xx devices based
	//on HFXTClockFrequency

	//If HFFrequency is 16MHz or above
	if (CSA_HFXTClockFrequency>16000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_3;
	}
	//If HFFrequency is between 8MHz and 16MHz
	else if (CSA_HFXTClockFrequency>8000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_2;
	}
	//If HFFrequency is between 0MHz and 4MHz
	else if (CSA_HFXTClockFrequency<4000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_0;
	}
	//If HFFrequency is between 4MHz and 8MHz
	else{
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_1;
	}


	while ((HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(HFXTOFFG);

		// Clear the global fault flag. In case the LFXT caused the global fault
		// flag to get set this will clear the global error condition. If any
		// error condition persists, global flag will get again.
		HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

	}

    if (timeout){
		//Set drive strength for HFXT
	    HWREG(baseAddress + OFS_CSCTL4) = ( HWREG(baseAddress + OFS_CSCTL4) &
	                                         ~(CSA_HFXTDRIVE_24MHZ_32MHZ)
	                                         ) |
	                                       (hfxtdrive);

	    //Switch on HFXT oscillator
	    HWREG(baseAddress + OFS_CSCTL4) &= ~HFXTOFF;

        return (STATUS_SUCCESS);
    } else   {
        return (STATUS_FAIL);
    }
}


//******************************************************************************
//
//! Bypasses the HFXT crystal oscillator, which supports crystal frequencies
//! between 0 MHz and 24 MHz. Loops until all oscillator fault flags are
//! cleared or until a timeout counter is decremented and equals to zero.
//! NOTE: User must call CSA_setExternalClockSource to set frequency of external
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

extern unsigned short
CSA_bypassHFXTWithTimeout ( unsigned int baseAddress,
    unsigned long timeout
    )
{
	//Verify user has initialized value of HFXTClock
	ASSERT(CSA_HFXTClockFrequency !=0)

	//If HFFrequency is 16MHz or above
	if (CSA_HFXTClockFrequency>16000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_3;
	}
	//If HFFrequency is between 8MHz and 16MHz
	else if (CSA_HFXTClockFrequency>8000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_2;
	}
	//If HFFrequency is between 0MHz and 4MHz
	else if (CSA_HFXTClockFrequency<4000000) {
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_0;
	}
	//If HFFrequency is between 4MHz and 8MHz
	else{
		HWREG(baseAddress + OFS_CSCTL4)=HFFREQ_1;
	}

    //Switch off HFXT oscillator and enable BYPASS mode
    HWREG(baseAddress + OFS_CSCTL4) |= (HFXTBYPASS + HFXTOFF );


	while ((HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG) && --timeout)
	{
		//Clear OSC fault Flags fault flags
		HWREGB(baseAddress + OFS_CSCTL5) &= ~(HFXTOFFG);

		// Clear the global fault flag. In case the LFXT caused the global fault
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
//! Stops the HFXT oscillator using the HFXTOFF bit.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! Modified registers are \b CSCTL4
//!
//! \return NONE
//
//******************************************************************************
void
CSA_HFXTOff (unsigned int baseAddress)
{
	CSA_HFXTOff(baseAddress);
}
//******************************************************************************
//
//! Enables conditional module requests
//!
//! \param baseAddress is the base address of the CS module.
//! \param selectClock selects specific request enables. Valid values are
//!        \b CSA_ACLK,
//!        \b CSA_SMCLK,
//!        \b CSA_MCLK,
//!        \b CSA_MODOSC
//!
//! Modified registers are \b CSCTL6
//!
//! \return NONE
//
//******************************************************************************
void
CSA_enableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    )
{
	ASSERT(
			(CSA_ACLK  == selectClock )||
			(CSA_SMCLK == selectClock )||
			(CSA_MCLK  == selectClock )||
			(CSA_MODOSC== selectClock ));

    HWREGB(baseAddress + OFS_CSCTL6) |= selectClock;
}

//******************************************************************************
//
//! Disables conditional module requests
//!
//! \param baseAddress is the base address of the CS module.
//! \param selectClock selects specific request enables. Valid values are
//!        \b CSA_ACLK,
//!        \b CSA_SMCLK,
//!        \b CSA_MCLK,
//!        \b CSA_MODOSC
//!
//! Modified registers are \b CSCTL6
//!
//! \return NONE
//
//******************************************************************************
void
CSA_disableClockRequest (
    unsigned int baseAddress,
    unsigned char selectClock
    )
{
	ASSERT(
			(CSA_ACLK  == selectClock )||
			(CSA_SMCLK == selectClock )||
			(CSA_MCLK  == selectClock )||
			(CSA_MODOSC== selectClock ));

    HWREGB(baseAddress + OFS_CSCTL6) &= ~selectClock;
}

//******************************************************************************
//
//! Gets the current CS fault flag status.
//!
//! \param baseAddress is the base address of the CS module.
//! \param mask is the masked interrupt flag status to be returned.
//!      Mask parameter can be either any of the following selection.
//!         - \b CSA_HFXTOFFG - HFXT oscillator fault flag
//!         - \b CSA_LFXTOFFG - LFXT oscillator fault flag

//! Modified registers are \b CSCTL5
//!
//! \return The current flag status for the corresponding masked bit
//
//******************************************************************************
unsigned char
CSA_faultFlagStatus (
    unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(
    			(CSA_HFXTOFFG  == mask )||
    			(CSA_LFXTOFFG == mask )
    			);
    return (HWREGB(baseAddress + OFS_CSCTL5) & mask);
}

//******************************************************************************
//
//! Clears the current CS fault flag status for the masked bit.
//!
//! \param baseAddress is the base address of the CS module.
//! \param mask is the masked interrupt flag status to be returned.
//!         mask parameter can be any one of the following
//!         - \b CSA_HFXTOFFG - HFXT oscillator fault flag
//!         - \b CSA_LFXTOFFG - LFXT oscillator fault flag

//!
//! Modified registers are \b CSCTL5
//!
//! \return NONE
//
//******************************************************************************
void
CSA_clearFaultFlag (
    unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(
			(CSA_HFXTOFFG  == mask )||
			(CSA_LFXTOFFG == mask )
			);
    HWREGB(baseAddress + OFS_CSCTL5) &= ~mask;
}

//******************************************************************************
//
//Compute the clock frequency when clock is source from DCO
//
//\param baseAddress is the base address of the CS module.
//\param clockdivider is clock source for FLL reference. Valid values are:
//           \b CSA_CLOCK_DIVIDER_1,
//           \b CSA_CLOCK_DIVIDER_2,
//           \b CSA_CLOCK_DIVIDER_4,
//           \b CSA_CLOCK_DIVIDER_8,
//           \b CSA_CLOCK_DIVIDER_16,
//           \b CSA_CLOCK_DIVIDER_32
//
//\return Calculated clock frequency in Hz
//
//******************************************************************************
unsigned long
privateCSASourceClockFromDCO ( unsigned int baseAddress,
		unsigned char clockdivider)
{
    unsigned long CLKFrequency;

   	if (HWREG(baseAddress + OFS_CSCTL1)& 0x0080) {
			switch(HWREG(baseAddress + OFS_CSCTL1)& DCOFSEL_7){
				case DCOFSEL_0:
					CLKFrequency=CSA_DCO_FREQ_1/clockdivider;
					break;
				case DCOFSEL_1:
					CLKFrequency=CSA_DCO_FREQ_5/clockdivider;
					break;
				case DCOFSEL_2:
					CLKFrequency=CSA_DCO_FREQ_6/clockdivider;
					break;
				case DCOFSEL_3:
					CLKFrequency=CSA_DCO_FREQ_7/clockdivider;
					break;
				case DCOFSEL_4:
					CLKFrequency=CSA_DCO_FREQ_8/clockdivider;
					break;
				default:
					CLKFrequency=0;
					break;
			}
		}else{
			switch(HWREG(baseAddress + OFS_CSCTL1)& DCOFSEL_7){
				case DCOFSEL_0:
					CLKFrequency=CSA_DCO_FREQ_1/clockdivider;
					break;
				case DCOFSEL_1:
					CLKFrequency=CSA_DCO_FREQ_2/clockdivider;
					break;
				case DCOFSEL_2:
					CLKFrequency=CSA_DCO_FREQ_3/clockdivider;
					break;
				case DCOFSEL_3:
					CLKFrequency=CSA_DCO_FREQ_4/clockdivider;
					break;
				case DCOFSEL_4:
					CLKFrequency=CSA_DCO_FREQ_5/clockdivider;
					break;
				default:
					CLKFrequency=0;
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
//		\b SELM__LFXTCLK (SELM__LFCLK),
//		\b SELM__VLOCLK,
//		\b SELM__HFXTCLK (SELM__HFCLK),
//		\b SELM__DCOCLK,
//		\b SELM__HFXTCLK (SELM__HFCLK),
//		\b SELM__DCOCLK,
//		\b SELM__LFMODOSC	[Available for FR58xx/FR59xx]
//		\b SELM__MODOSC		[Available for FR58xx/FR59xx]
//\param CLKSourceDivider is the Clock source divider
//
//\return Calculated clock frequency in Hz
//
//******************************************************************************
unsigned long
privateCSAComputeCLKFrequency ( unsigned int baseAddress,
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

    	// If LFXT is selected as clock source
        case SELM__LFXTCLK:
            CLKFrequency = (CSA_LFXTClockFrequency /
                            CLKSourceFrequencyDivider);

            //Check if LFXTOFFG is not set. If fault flag is set
            //VLO is used as source clock
            if (HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG){
                HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG);
                //Clear OFIFG fault flag
                HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

                if (HWREGB(baseAddress + OFS_CSCTL5) & LFXTOFFG){
                			CLKFrequency = CSA_LFMODCLK_FREQUENCY;
				}
            }
            break;

        case SELM__VLOCLK:
            CLKFrequency =
                (CSA_VLOCLK_FREQUENCY / CLKSourceFrequencyDivider);
            break;

        case SELM__LFMODOSC:
            CLKFrequency =
                (CSA_LFMODCLK_FREQUENCY / CLKSourceFrequencyDivider);

            break;

        case SELM__DCOCLK:
        	CLKFrequency =
        	privateCSASourceClockFromDCO(baseAddress, CLKSourceFrequencyDivider);

            break;

        case SELM__MODOSC:
            CLKFrequency =
                (CSA_MODCLK_FREQUENCY / CLKSourceFrequencyDivider);

            break;

        case SELM__HFXTCLK:
            CLKFrequency =
                (CSA_HFXTClockFrequency / CLKSourceFrequencyDivider);

            if (HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG){

              HWREGB(baseAddress + OFS_CSCTL5) &=  ~HFXTOFFG;
              //Clear OFIFG fault flag
              HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;
            }

            if (HWREGB(baseAddress + OFS_CSCTL5) & HFXTOFFG){
                CLKFrequency = CSA_MODCLK_FREQUENCY;
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
//! CSA_externalClockSourceInit API was invoked before in case LFXT or
//! HFXT is being used. User must call CSA_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current ACLK frequency in Hz
//
//******************************************************************************
unsigned long
CSA_getACLK (unsigned int baseAddress)
{

	//Find ACLK source
	unsigned int ACLKSource = (HWREG(baseAddress + OFS_CSCTL2) & SELA_7);
	ACLKSource = ACLKSource >> 8;

	//Find ACLK frequency divider
	unsigned int ACLKSourceDivider =  HWREG(baseAddress + OFS_CSCTL3) & SELA_7;
	ACLKSourceDivider = ACLKSourceDivider >>8;

	return (privateCSAComputeCLKFrequency(baseAddress,
				ACLKSource,
				ACLKSourceDivider));

}

//******************************************************************************
//
//! Get the current SMCLK frequency.
//!
//! If a oscillator fault is set, the frequency returned will be based on the
//! fail safe mechanism of CS module. The user of this API must ensure that
//! CSA_externalClockSourceInit API was invoked before in case LFXT or
//! HFXT is being used. User must call CSA_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current SMCLK frequency in Hz
//
//******************************************************************************
unsigned long
CSA_getSMCLK (unsigned int baseAddress)
{
		//Find SMCLK source
		unsigned int SMCLKSource = HWREGB(baseAddress + OFS_CSCTL2) & SELS_7 ;

		SMCLKSource = SMCLKSource >> 4;

		//Find SMCLK frequency divider
		unsigned int SMCLKSourceDivider = HWREG(baseAddress + OFS_CSCTL3) & SELS_7;
		SMCLKSourceDivider = SMCLKSourceDivider >> 4;

		return (privateCSAComputeCLKFrequency(baseAddress,
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
//! CSA_externalClockSourceInit API was invoked before in case LFXT or
//! HFXT is being used. User must call CSA_init before calling this function.
//!
//! \param baseAddress is the base address of the CS module.
//!
//! \return Current MCLK frequency in Hz
//
//******************************************************************************
unsigned long
CSA_getMCLK (unsigned int baseAddress)
{
		//Find MCLK source
		unsigned int MCLKSource = (HWREG(baseAddress + OFS_CSCTL2) & SELM_7);
		//Find MCLK frequency divider
		unsigned int MCLKSourceDivider =  HWREG(baseAddress + OFS_CSCTL3) & SELM_7;

		return (privateCSAComputeCLKFrequency(baseAddress,
					MCLKSource,
					MCLKSourceDivider )
				);
}

//******************************************************************************
//
//! Turns off VLO
//!
//! USer must call CSA_init function before invoking this function.
//! Modified registers are \b CSCTL4
//!
//
//******************************************************************************

void
CSA_VLOoff(unsigned int baseAddress)
{
	HWREG(baseAddress + OFS_CSCTL4) |= VLOOFF;
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
unsigned int CSA_clearAllOscFlagsWithTimeout(unsigned int baseAddress,
                                             unsigned long timeout
                                             )
{
    do {
      // Clear all osc fault flags
      HWREGB(baseAddress + OFS_CSCTL5) &= ~(LFXTOFFG + HFXTOFFG );

      // Clear the global osc fault flag.
      HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1) &= ~OFIFG;

      // Check LFXT fault flags
    } while ((HWREGB(SFR_BASEADDRESS + OFS_SFRIFG1)) && --timeout);

    return (HWREGB(baseAddress + OFS_CSCTL5) & (LFXTOFFG + HFXTOFFG));
}

//******************************************************************************
//
//! Set DCO frequency
//!
//! Before invoking the this function, the user must call CSA_init.
//!
//! \param baseAddress is the base address of the CS module.
//! \param dcofsel selects valid frequency options based on dco frequency range
//!			selection (dcorsel). Valid options are:
//!			\e For FR58xx/FR59xx devices:
//!			\b CSA_DCOFSEL_0 - Low frequency option 1 MHZ. High frequency
//!				option 1 MHz.
//!			\b CSA_DCOFSEL_1 - Low frequency option 2.67 MHZ. High frequency
//!				option 5.33 MHz.
//!			\b CSA_DCOFSEL_2 Low frequency option 3.33 MHZ. High frequency
//!				option 6.67 MHz.
//!			\b CSA_DCOFSEL_3 Low frequency option 4 MHZ. High frequency
//!				option 8 MHz.
//!			\b CSA_DCOFSEL_4 Low frequency option 5.33 MHZ. High frequency
//!				option 16 MHz.
//!			\b CSA_DCOFSEL_5 Low frequency option 6.67 MHZ. High frequency
//!				option 20 MHz.
//!			\b CSA_DCOFSEL_6 Low frequency option 8 MHZ. High frequency
//!				option 24 MHz.
//!\param dcorsel selects frequency range option. Valid options are:
//!			\b CSA_DCORSEL_0	[Default - Low Frequency Option]
//!			\b CSA_DCORSEL_1	[High Frequency Option]
//!
//! \return NONE
//
//******************************************************************************
void
CSA_setDCOFreq(
		unsigned int baseAddress,
		unsigned int dcorsel,
		unsigned int dcofsel)
{
		ASSERT(
					(dcofsel==CSA_DCOFSEL_0)||
					(dcofsel==CSA_DCOFSEL_1)||
					(dcofsel==CSA_DCOFSEL_2)||
					(dcofsel==CSA_DCOFSEL_3)||
					(dcofsel==CSA_DCOFSEL_4)||
					(dcofsel==CSA_DCOFSEL_5)||
					(dcofsel==CSA_DCOFSEL_6)
					);

		//Verify user has selected a valid DCO Frequency Range option
		ASSERT(
				(dcorsel==CSA_DCORSEL_0)||
				(dcorsel==CSA_DCORSEL_1));

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

