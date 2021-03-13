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
//mpu_a.c - Driver for the MPU Module.
//
//******************************************************************************

#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/mpu_a.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
#include "driverlib/5xx_6xx/debug.h"
//******************************************************************************
//
//!This function creates two memory segmentations in FRAM allowing the user
//!to set access right to each segment. To set the correct value
//!for seg1boundary, the user must consult the Device Family User's Guide and
//!provide the MPUSBx value corresponding to the memory address where the user
//!wants to create the partition.
//!For FR57xx device consult Table 6-1 in Section 6.2.2 of the User's Guide to
//!find the options available for MPUSBx.
//!For FR58xx/Fr59xx devices consult the "Segment Border Setting" section in the
//!User's Guide to find the options available for MPUSBx
//!
//!
//! \param baseAddress is the base address of the MPU module.
//! \param seg1boundary - Valid values can be found on the deviceFamily
//!				User's Guide
//!            \e Valid values
//!				Value in the following range 0x0400-0x13C0
//! \param seg1accmask is the bit mask of access right for memory segment 1.
//!		   the following bit masks are available:
//!				\b MPUA_EXEC|MPUA_READ|MPUA_WRITE.- For execution,read and write access rights
//!					[Default]
//!				\b MPUA_EXEC|MPUA_READ.- For execution and read access rights.
//!				\b READ|MPUA_WRITE.- For read and write access rights
//!				\b READ.- For read only access rights
//! \param seg2accmask is the bit mask of access right for memory segment 2.
//!		   the following bit masks are available:
//!				\b MPUA_EXEC|MPUA_READ|MPUA_WRITE.- For execution,read and write access rights
//!					[Default]
//!				\b MPUA_EXEC|MPUA_READ.- For execution and read access rights.
//!				\b READ|MPUA_WRITE.- For read and write access rights
//!				\b READ.- For read only access rights
//!
//! Modified registers are \b MPUCTL0, \b MPUSEG, \b MPUSAM
//!
//! \return NONE
//
//******************************************************************************

void
MPUA_createTwoSegments(unsigned int baseAddress,
		unsigned int seg1boundary,
		unsigned char seg1accmask,
		unsigned char seg2accmask
		)
{

	// Verify access right mask for segment1 is a valid selection
	ASSERT((seg1accmask<MPUSEG1VS)&&((seg1accmask==(MPUA_EXEC|MPUA_READ|MPUA_WRITE))||
									 (seg1accmask==(MPUA_EXEC|MPUA_READ))||
									 (seg1accmask==(MPUA_READ|MPUA_WRITE))||
									 (seg1accmask==(MPUA_READ))));

	// Verify access right mask for segment2 is a valid selection
	ASSERT((seg2accmask<MPUSEG1VS)&&((seg2accmask==(MPUA_EXEC|MPUA_READ|MPUA_WRITE))||
									 (seg2accmask==(MPUA_EXEC|MPUA_READ))||
									 (seg2accmask==(MPUA_READ|MPUA_WRITE))||
									 (seg2accmask==(MPUA_READ))));


	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;

	// Verify segment1 boundary is valid selection
	ASSERT(seg1boundary<MPUA_MAX_SEG_VALUE);

	// Create two memory segmentations
	HWREG(baseAddress + OFS_MPUSEGB1) = seg1boundary;
	HWREG(baseAddress + OFS_MPUSEGB2) = seg1boundary;

	// Set access rights based on user's selection for segment1
	switch (seg1accmask) {
		case MPUA_EXEC|MPUA_READ:
			HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG1WE;
			break;
		case MPUA_READ|MPUA_WRITE:
			HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG1XE;
			break;
		case MPUA_READ:
			HWREG(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE +MPUSEG1WE);
			break;
		default:
			HWREG(baseAddress + OFS_MPUSAM) |= (MPUSEG1XE +MPUSEG1WE+ MPUSEG1RE);
			break;
	}

	// Set access rights based on user's selection for segment2
	switch (seg2accmask) {
		case MPUA_EXEC|MPUA_READ:
			HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG3WE;
			break;
		case MPUA_READ|MPUA_WRITE:
			HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG3XE;
			break;
		case MPUA_READ:
			HWREG(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE +MPUSEG3WE);
			break;
		default:
			HWREG(baseAddress + OFS_MPUSAM) |= (MPUSEG3XE +MPUSEG3WE+ MPUSEG3RE);
			break;
	}
}

//******************************************************************************
//
//!This function creates three memory segmentations in FRAM allowing the user
//!to set access right to each segment. To set the correct value
//!for seg1boundary, the user must consult the Device Family User's Guide and
//!provide the MPUSBx value corresponding to the memory address where the user
//!wants to create the partition.
//!Consult the "Segment Border Setting" section in the
//!User's Guide to find the options available for MPUSBx
//!
//! \param baseAddress is the base address of the MPU module.
//! \param seg1boundary - Valid values can be found on the deviceFamily
//!				User's Guide
//!            \e Valid values
//!				Value in the following range 0x0400-0x13C0
//! \param seg2boundary - Valid values can be found on the deviceFamily
//!				User's Guide
//!            \e Valid values
//!				Value in the following range 0x0400-0x13C0
//! \param seg1accmask is the bit mask of access right for memory segment 1.
//!		   the following bit masks are available:
//!				\b MPUA_EXEC|MPUA_READ|MPUA_WRITE.- For execution,read and
//!					write access rights [Default]
//!				\b MPUA_EXEC|MPUA_READ.- For execution and read access rights.
//!				\b READ|MPUA_WRITE.- For read and write access rights
//!				\b READ.- For read only access rights
//! \param seg2accmask is the bit mask of access right for memory segment 2.
//!		   the following bit masks are available:
//!				\b MPUA_EXEC|MPUA_READ|MPUA_WRITE.- For execution,read and
//!					write access rights [Default]
//!				\b MPUA_EXEC|MPUA_READ.- For execution and read access rights.
//!				\b READ|MPUA_WRITE.- For read and write access rights
//!				\b READ.- For read only access rights
//! \param seg3accmask is the bit mask of access right for memory segment 1.
//!		   the following bit masks are available:
//!				\b MPUA_EXEC|MPUA_READ|MPUA_WRITE.- For execution,read and
//!					write access rights [Default]
//!				\b MPUA_EXEC|MPUA_READ.- For execution and read access rights.
//!				\b READ|MPUA_WRITE.- For read and write access rights
//!				\b READ.- For read only access rights
//!
//! Modified registers are \b MPUCTL0, \b MPUSEG, \b MPUSAM
//!
//! \return NONE
//
//******************************************************************************

void
MPUA_createThreeSegments(unsigned int baseAddress,
		unsigned int seg1boundary,
		unsigned int seg2boundary,
		unsigned char seg1accmask,
		unsigned char seg2accmask,
		unsigned char seg3accmask
		)
{


	// Verify access right mask for segment1 is a valid selection
	ASSERT((seg1accmask<MPUSEG1VS)&&((seg1accmask==(MPUA_EXEC|MPUA_READ|MPUA_WRITE))||
									 (seg1accmask==(MPUA_EXEC|MPUA_READ))||
									 (seg1accmask==(MPUA_READ|MPUA_WRITE))||
									 (seg1accmask==(MPUA_READ))));

	// Verify access right mask for segment2 is a valid selection
	ASSERT((seg2accmask<MPUSEG1VS)&&((seg2accmask==(MPUA_EXEC|MPUA_READ|MPUA_WRITE))||
									 (seg2accmask==(MPUA_EXEC|MPUA_READ))||
									 (seg2accmask==(MPUA_READ|MPUA_WRITE))||
									 (seg2accmask==(MPUA_READ))));

	// Verify access right mask for segment3 is a valid selection
	ASSERT((seg3accmask<MPUSEG1VS)&&((seg3accmask==(MPUA_EXEC|MPUA_READ|MPUA_WRITE))||
									 (seg3accmask==(MPUA_EXEC|MPUA_READ))||
									 (seg3accmask==(MPUA_READ|MPUA_WRITE))||
									 (seg3accmask==(MPUA_READ))));



	// Write MPU password to allow MPU register configuration
	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;

				// Verify segment1 boundary is valid selection
		ASSERT(seg1boundary<MPUA_MAX_SEG_VALUE);

		// Create two memory segmentations
		HWREG(baseAddress + OFS_MPUSEGB1) = seg1boundary;
		HWREG(baseAddress + OFS_MPUSEGB2) = seg2boundary;

		// Set access rights based on user's selection for segment1
		switch (seg1accmask) {
			case MPUA_EXEC|MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG1WE;
				break;
			case MPUA_READ|MPUA_WRITE:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG1XE;
				break;
			case MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~(MPUSEG1XE +MPUSEG1WE);
				break;
			default:
				HWREG(baseAddress + OFS_MPUSAM) |= (MPUSEG1XE +MPUSEG1WE+ MPUSEG1RE);
				break;
		}

		// Set access rights based on user's selection for segment2
		switch (seg2accmask) {
			case MPUA_EXEC|MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG2WE;
				break;
			case MPUA_READ|MPUA_WRITE:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG2XE;
				break;
			case MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~(MPUSEG2XE +MPUSEG2WE);
				break;
			default:
				HWREG(baseAddress + OFS_MPUSAM) |= (MPUSEG2XE +MPUSEG2WE+ MPUSEG2RE);
				break;
		}

		// Set access rights based on user's selection for segment2
		switch (seg3accmask) {
			case MPUA_EXEC|MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG3WE;
				break;
			case MPUA_READ|MPUA_WRITE:
				HWREG(baseAddress + OFS_MPUSAM) &= ~MPUSEG3XE;
				break;
			case MPUA_READ:
				HWREG(baseAddress + OFS_MPUSAM) &= ~(MPUSEG3XE +MPUSEG3WE);
				break;
			default:
				HWREG(baseAddress + OFS_MPUSAM) |= (MPUSEG3XE +MPUSEG3WE+ MPUSEG3RE);
				break;
		}

}

//******************************************************************************
//
//!The following function enables the NMI Event if a Segment violation has
//!occurred.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified register is \b MPUCTL0
//!
//!\return NONE
//!
//
//******************************************************************************
void
MPUA_enableNMIevent(unsigned int baseAddress)
{
	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW + MPUSEGIE;
}


//******************************************************************************
//
//!The following function enables the MPU module in the device.
//!
//!This function needs to be called once all memory segmentation has been
//!done. If this function is not called the MPU module will not be activated.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified register is \b MPUCTL0
//!
//!\return NONE
//!
//
//******************************************************************************
void
MPUA_start(unsigned int baseAddress)
{
	if(HWREG(baseAddress + OFS_MPUCTL0) & MPUSEGIE)
	{
		HWREG(baseAddress + OFS_MPUCTL0) = MPUPW  | MPUSEGIE| MPUENA;
	}else
	{
		HWREG(baseAddress + OFS_MPUCTL0) = MPUPW  | MPUENA;
	}
}


//******************************************************************************
//
//!The following function enables PUC generation when an access violation has
//!Occurred on the memory segment selected by the user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param segment is the bit mask of memory segment that will generate a PUC
//!			when an access violation occurs. Valid values is the bit mask of any
//!			of the following values:
//!			\b MPUA_FIRST_SEG.- enables PUC generation on first memory segment.
//!			\b MPUA_SECOND_SEG.-  enables PUC generation on second memory segment.
//!			\b MPUA_THIRD_SEG.-  enables PUC generation on third memory segment.
//!
//! Modified registers are \b MPUCTL0, \b MPUSAM
//!
//! \return NONE
//!
//
//*****************************************************************************

void
MPUA_enablePUCOnViolation(unsigned int baseAddress,
		unsigned int segment
		)
{
	// Verify user has selected a valid memory segment
	ASSERT(0x00 &= ^(MPUA_FIRSTSEG + MPUA_SECOND_SEG + MPUA_THIRD_SEG));

	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;
	HWREG(baseAddress + OFS_MPUSAM) |= segment;

}


//******************************************************************************
//
//!The following function isables PUC generation when an access violation has
//!Occurred on the memory segment selected by the user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param segment is the bit mask of memory segment that will NOT generate a PUC
//!			when an access violation occurs. Valid values is the bit mask of any
//!			of the following values:
//!			\b MPUA_FIRST_SEG.- enables PUC generation on first memory segment.
//!			\b MPUA_SECOND_SEG.-  enables PUC generation on second memory segment.
//!			\b MPUA_THIRD_SEG.-  enables PUC generation on third memory segment.
//!
//! Modified registers are \b MPUCTL0, \b MPUSAM
//!
//! \return NONE
//!
//
//*****************************************************************************

void
MPUA_disablePUCOnViolation(unsigned int baseAddress,
		unsigned int segment
		)
{
	// Verify user has selected a valid memory segment
	ASSERT(0x00 &= ^(MPU_FIRSTSEG + MPU_SECOND_SEG + MPU_THIRD_SEG));

	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;
	HWREG(baseAddress + OFS_MPUSAM) &= ~segment;

}

//******************************************************************************
//
//!
//!Returns the memory segment violation flag status requested by the user.
//!
//! \param baseAddress is the base address of the MPU module.
//! \param memAccFlag is the is the memory access violation flag. Valid value
//! 	is the bit mask of any of the following value:
//!			\b MPUA_SEG_1_ACCESS_VIOLATION.- is set if an access violation in
//!					Main Memory Segment 1 is detected.
//!			\b MPUA_SEG_2_ACCESS_VIOLATION.- is set if an access violation in
//!					Main Memory Segment 2 is detected.
//!			\b MPUA_SEG_3_ACCESS_VIOLATION.- is set if an access violation in
//!					Main Memory Segment 3 is detected.
//!
//! \return The current interrupt status as the mask of the set flags.
//!
//
//******************************************************************************

unsigned short
MPUA_getInterruptStatus(unsigned int baseAddress,
		unsigned int memAccFlag
		)
{
	return (HWREG(baseAddress + OFS_MPUCTL1) & memAccFlag);
}


//******************************************************************************
//
//!
//!Returns the memory segment violation flag status requested by the user or
//!if user is providing a bit mask value, the function will return a value
//!indicating if all flags were cleared
//!
//! \param baseAddress is the base address of the MPU module.
//! \param memAccFlag is the is the memory access violation flag. Valid values
//! 	for this function are:
//!			\b MPUSEG1IFG is set if an access violation in Main Memory Segment 1
//!					is detected.
//!			\b MPUSEG2IFG is set if an access violation in Main Memory Segment 2
//!					is detected.
//!			\b MPUSEG3IFG is set if an access violation in Main Memory Segment 3
//!					is detected.
//!
//! \return The current interrupt status as the mask of the set flags.
//!
//
//******************************************************************************

unsigned short
MPUA_clearInterruptFlag(unsigned int baseAddress,
		unsigned int memAccFlag
	)
{

	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;
	HWREG(baseAddress + OFS_MPUCTL1) &= ~memAccFlag;

	return (HWREG(baseAddress + OFS_MPUCTL1) & memAccFlag);
}

//******************************************************************************
//
//!
//!Clears all Memory Segment Access Violation Interrupt Flags.
//!
//! \param baseAddress is the base address of the MPU module.
//!
//! Modified registers are MPUCTL1
//!
//! \return The current interrupt status as the mask of the set flags.
//!
//
//******************************************************************************

unsigned short
MPUA_clearAllInterruptFlags(unsigned int baseAddress
	)
{

	HWREG(baseAddress + OFS_MPUCTL0) = MPUPW;
	HWREG(baseAddress + OFS_MPUCTL1) &= ~(MPUSEG1IFG + MPUSEG2IFG + MPUSEG3IFG);

	return (HWREG(baseAddress + OFS_MPUCTL1) & (MPUSEG1IFG + MPUSEG2IFG + MPUSEG3IFG));
}


//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************








