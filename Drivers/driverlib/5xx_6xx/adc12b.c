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
//*****************************************************************************
//
//adc12b.c - Driver for the ADC12B Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/debug.h"
#include "driverlib/5xx_6xx/adc12b.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
//*****************************************************************************
//
//! Initializes the ADC12B Module.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param sampleHoldSignalSourceSelect is the signal that will trigger a
//!       sample-and-hold for an input signal to be converted.
//!        Valid values are
//!        \b ADC12B_SAMPLEHOLDSOURCE_SC [Default]
//!        \b ADC12B_SAMPLEHOLDSOURCE_1
//!        \b ADC12B_SAMPLEHOLDSOURCE_2
//!        \b ADC12B_SAMPLEHOLDSOURCE_3
//!        \b ADC12B_SAMPLEHOLDSOURCE_4
//!        \b ADC12B_SAMPLEHOLDSOURCE_5
//!        \b ADC12B_SAMPLEHOLDSOURCE_6
//!        \b ADC12B_SAMPLEHOLDSOURCE_7
//!        This parameter is device specific and sources should be found in the
//!        device's datasheet.
//!        Modified bits are \b ADC12SHSx of \b ADC12CTL1 register.
//! \param clockSourceSelect selects the clock that will be used by the ADC12B
//!       core, and the sampling timer if a sampling pulse mode is enabled.
//!        Valid values are
//!        \b ADC12B_CLOCKSOURCE_ADC12OSC - MODOSC 5 MHz oscillator from the UCS
//!            [Default]
//!        \b ADC12B_CLOCKSOURCE_ACLK     - The Auxilary Clock
//!        \b ADC12B_CLOCKSOURCE_MCLK     - The Master Clock
//!        \b ADC12B_CLOCKSOURCE_SMCLK    - The Sub-Master Clock
//!        Modified bits are \b ADC12SSELx of \b ADC12CTL1 register.
//! \param clockSourceDivider selects the amount that the clock will be divided.
//!        Valid values are
//!        \b ADC12B_CLOCKDIVIDER_1 [Default]
//!        \b ADC12B_CLOCKDIVIDER_2
//!        \b ADC12B_CLOCKDIVIDER_3
//!        \b ADC12B_CLOCKDIVIDER_4
//!        \b ADC12B_CLOCKDIVIDER_5
//!        \b ADC12B_CLOCKDIVIDER_6
//!        \b ADC12B_CLOCKDIVIDER_7
//!        \b ADC12B_CLOCKDIVIDER_8
//!        Modified bits are \b ADC12DIVx of \b ADC12CTL1 register
//! \param clockSourcePredivider selects the amount that the clock will be predivided.
//!		   Valid values are
//!        \b ADC12B_CLOCKPREDIVIDER__1 [Default]
//!        \b ADC12B_CLOCKPREDIVIDER__4
//!        \b ADC12B_CLOCKPREDIVIDER__32
//!        \b ADC12B_CLOCKPREDIVIDER__64
//!        Modified bits are \b ADC12PDIV of \b ADC12CTL1 register.
//! \param internalChannelMap selects what internal channel to map for ADC
//!        input channels
//!		   Valid values are
//!        \b ADC12B_MAPINTCH3
//!        \b ADC12B_MAPINTCH2
//!        \b ADC12B_MAPINTCH1
//!        \b ADC12B_MAPINTCH0
//!        \b ADC12B_TEMPSENSEMAP
//!        \b ADC12B_BATTMAP
//!        Modified bits are \b ADC12ICH3MA, \b ADC12ICH2MA, \b ADC12ICH1MA, \b ADC12ICH0MA,
//!							 	\b ADC12TCMAP and \b ADC12BATMAP of
//!        						\b ADC12CTL3 register
//!
//! This function initializes the ADC module to allow for analog-to-digital
//! conversions. Specifically this function sets up the sample-and-hold signal
//! and clock sources for the ADC core to use for conversions. Upon successful
//! completion of the initialization all of the ADC control registers will be
//! reset, excluding the memory controls and reference module bits, the given
//! parameters will be set, and the ADC core will be turned on (Note, that the
//! ADC core only draws power during conversions and remains off when not
//! converting).Note that sample/hold signal sources are device dependent. Note
//! that if re-initializing the ADC after starting a conversion with the
//! startConversion() function, the disableConversion() must be called BEFORE
//! this function can be called.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the initialization process.
//
//*****************************************************************************
unsigned short ADC12B_init (unsigned int baseAddress,
    unsigned int sampleHoldSignalSourceSelect,
    unsigned char clockSourceSelect,
    unsigned int clockSourceDivider,
    unsigned int clockSourcePredivider,
    unsigned int internalChannelMap)
{

    ASSERT(sampleHoldSignalSourceSelect <= ADC12B_SAMPLEHOLDSOURCE_3);
    ASSERT(clockSourceSelect <= ADC12B_CLOCKSOURCE_SMCLK);
    ASSERT(clockSourceDivider <= ADC12B_CLOCKDIVIDER_32);
    ASSERT(internalChannelMap <= ADC12B_BATMAP);
    ASSERT(sequenceStartAddress <= START_AT_ADC12MEM31);
    //Make sure the ENC bit is cleared before initializing the ADC12
    HWREGB(baseAddress + OFS_ADC12CTL0_L) &= ~ADC12ENC;

    unsigned char retVal = STATUS_SUCCESS;

    //Turn OFF ADC12B Module & Clear Interrupt Registers
    HWREG(baseAddress + OFS_ADC12CTL0) &= ~(ADC12ON + ADC12ENC + ADC12SC);
    HWREG(baseAddress + OFS_ADC12IER0)  &= 0x0000; //Reset ALL interrupt enables
    HWREG(baseAddress + OFS_ADC12IER1)  &= 0x0000;
    HWREG(baseAddress + OFS_ADC12IER2)  &= 0x0000;
    HWREG(baseAddress + OFS_ADC12IFGR0)  &= 0x0000; //Reset ALL interrupt flags
    HWREG(baseAddress + OFS_ADC12IFGR1)  &= 0x0000;
    HWREG(baseAddress + OFS_ADC12IFGR2)  &= 0x0000;

    //Set ADC12B Control 1
    HWREG(baseAddress + OFS_ADC12CTL1) =
        sampleHoldSignalSourceSelect //Setup the Sample-and-Hold Source
        + (clockSourceDivider & ADC12DIV_7) //Set Clock Divider
        + (clockSourcePredivider & ADC12PDIV__64)
        + clockSourceSelect; //Setup Clock Source

    //Set ADC12B Control 2
    HWREG(baseAddress + OFS_ADC12CTL2) =
        ADC12RES_2; //Default resolution to 12-bits

    //Set ADC12B Control 3
    HWREG(baseAddress + OFS_ADC12CTL3) =
    	internalChannelMap; // Map internal channels

    return (retVal) ;
}

//*****************************************************************************
//
//! Enables the ADC12B block.
//!
//! \param baseAddress is the base address of the ADC12B module.
//!
//! This will enable operation of the ADC12B block.
//! Modified bits are \b ADC12ON of \b ADC12CTL0 register.
//!
//! \return None.
//
//*****************************************************************************
void ADC12B_enable (unsigned int baseAddress)
{
    //Enable the ADC12B Module
    HWREGB(baseAddress + OFS_ADC12CTL0_L) |= ADC12ON;
}

//*****************************************************************************
//
//! Disables the ADC12B block.
//!
//! \param baseAddress is the base address of the ADC12B module.
//!
//! This will disable operation of the ADC12B block.
//! Modified bits are \b ADC12ON of \b ADC12CTL0 register.
//!
//! \return None.
//
//*****************************************************************************
void ADC12B_disable (unsigned int baseAddress)
{
    //Disable ADC12B module
    HWREGB(baseAddress + OFS_ADC12CTL0_L) &= ~ADC12ON;
}


//*****************************************************************************
//
//! Sets up and enables the Sampling Timer Pulse Mode.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param clockCycleHoldCountLowMem sets the amount of clock cycles to
//!       sample-and-hold for the higher memory buffers 0-7.
//!        Valid values are
//!        \b ADC12B_CYCLEHOLD_4_CYCLES [Default]
//!        \b ADC12B_CYCLEHOLD_8_CYCLES
//!        \b ADC12B_CYCLEHOLD_16_CYCLES
//!        \b ADC12B_CYCLEHOLD_32_CYCLES
//!        \b ADC12B_CYCLEHOLD_64_CYCLES
//!        \b ADC12B_CYCLEHOLD_96_CYCLES
//!        \b ADC12B_CYCLEHOLD_128_CYCLES
//!        \b ADC12B_CYCLEHOLD_192_CYCLES
//!        \b ADC12B_CYCLEHOLD_256_CYCLES
//!        \b ADC12B_CYCLEHOLD_384_CYCLES
//!        \b ADC12B_CYCLEHOLD_512_CYCLES
//!        \b ADC12B_CYCLEHOLD_768_CYCLES
//!        \b ADC12B_CYCLEHOLD_1024_CYCLES
//!        Modified bits are \b ADC12SHT0x of \b ADC12CTL0 register.
//! \param clockCycleHoldCountHighMem sets the amount of clock cycles to
//!       sample-and-hold for the higher memory buffers 8-15.
//!        Valid values are
//!        \b ADC12B_CYCLEHOLD_4_CYCLES [Default]
//!        \b ADC12B_CYCLEHOLD_8_CYCLES
//!        \b ADC12B_CYCLEHOLD_16_CYCLES
//!        \b ADC12B_CYCLEHOLD_32_CYCLES
//!        \b ADC12B_CYCLEHOLD_64_CYCLES
//!        \b ADC12B_CYCLEHOLD_96_CYCLES
//!        \b ADC12B_CYCLEHOLD_128_CYCLES
//!        \b ADC12B_CYCLEHOLD_192_CYCLES
//!        \b ADC12B_CYCLEHOLD_256_CYCLES
//!        \b ADC12B_CYCLEHOLD_384_CYCLES
//!        \b ADC12B_CYCLEHOLD_512_CYCLES
//!        \b ADC12B_CYCLEHOLD_768_CYCLES
//!        \b ADC12B_CYCLEHOLD_1024_CYCLES
//!        Modified bits are \b ADC12SHT1x of \b ADC12CTL0 register.
//! \param multipleSamplesEnabled allows multiple conversions to start
//!       without a trigger signal from the sample/hold signal
//!        Valid values are
//!        \b ADC12B_MULTIPLESAMPLESDISABLE - a timer trigger will be needed to
//!                  start every ADC conversion. [Default]
//!        \b ADC12B_MULTIPLESAMPLESENABLE  - during a sequenced and/or repeated
//!                  conversion mode, after the first conversion, no sample/hold
//!                  signal is necessary to start subsequent sample/hold and
//!                  convert processes.
//!        Modified bits are \b ADC12MSC of \b ADC12CTL0 register.
//!
//! This function sets up the sampling timer pulse mode which allows the
//! sample/hold signal to trigger a sampling timer to sample-and-hold an input
//! signal for a specified number of clock cycles without having to hold the
//! sample/hold signal for the entire period of sampling. Note that if a
//! conversion has been started with the startConversion() function, then a call
//! to disableConversions() is required before this function may be called.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_setupSamplingTimer (unsigned int baseAddress,
    unsigned int clockCycleHoldCountLowMem,
    unsigned int clockCycleHoldCountHighMem,
    unsigned short multipleSamplesEnabled)
{
    //Make sure the ENC bit is cleared before setting up sampling pulse mode
    ASSERT( !(HWREGB(baseAddress + OFS_ADC12CTL0_L) & ADC12ENC) );

    ASSERT(clockCycleHoldCountLowMem <= ADC12B_CYCLEHOLD_1024_CYCLES);
    ASSERT(clockCycleHoldCountHighMem <= ADC12B_CYCLEHOLD_1024_CYCLES);

    HWREG(baseAddress + OFS_ADC12CTL1) |= ADC12SHP;

    //Reset clock cycle hold counts and msc bit before setting them
    HWREG(baseAddress + OFS_ADC12CTL0) &=
        ~(ADC12SHT0_15 + ADC12SHT1_15 + ADC12MSC);

    //Set clock cycle hold counts and msc bit
    HWREG(baseAddress + OFS_ADC12CTL0) |= clockCycleHoldCountLowMem
                                          + (clockCycleHoldCountHighMem << 4)
                                          + multipleSamplesEnabled;
}

//*****************************************************************************
//
//! Disables Sampling Timer Pulse Mode.
//!
//! \param baseAddress is the base address of the ADC12B module.
//!
//! Disables the Sampling Timer Pulse Mode. Note that if a conversion has been
//! started with the startConversion() function, then a call to
//! disableConversions() is required before this function may be called.
//!
//! Modified bits are \b ADC12SHP of \b ADC12CTL1 register.
//! \return NONE
//
//*****************************************************************************
void ADC12B_disableSamplingTimer (unsigned int baseAddress)
{
    //Make sure the ENC bit is cleared before disabling sampling pulse mode
    ASSERT( !(HWREGB(baseAddress + OFS_ADC12CTL0_L) & ADC12ENC) );

    HWREG(baseAddress + OFS_ADC12CTL1) &= ~(ADC12SHP);
}

//*****************************************************************************
//
//! Configures the controls of the selected memory buffer.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param memoryBufferControlIndex is the selected memory buffer to set the
//!       configuration for.
//!        Valid values are
//!        \b ADC12B_MEMORY_0  (0x00)
//!        \b ADC12B_MEMORY_1  (0x02)
//!        \b ADC12B_MEMORY_2  (0x04)
//!        \b ADC12B_MEMORY_3  (0x06)
//!        \b ADC12B_MEMORY_4  (0x08)
//!        \b ADC12B_MEMORY_5  (0x0A)
//!        \b ADC12B_MEMORY_6  (0x0C)
//!        \b ADC12B_MEMORY_7  (0x0E)
//!        \b ADC12B_MEMORY_8  (0x10)
//!        \b ADC12B_MEMORY_9  (0x12)
//!        \b ADC12B_MEMORY_10 (0x14)
//!        \b ADC12B_MEMORY_11 (0x16)
//!        \b ADC12B_MEMORY_12 (0x18)
//!        \b ADC12B_MEMORY_13 (0x1A)
//!        \b ADC12B_MEMORY_14 (0x1C)
//!        \b ADC12B_MEMORY_15 (0x1E)
//!        \b ADC12B_MEMORY_16 (0x20)
//!        \b ADC12B_MEMORY_17 (0x22)
//!        \b ADC12B_MEMORY_18 (0x24)
//!        \b ADC12B_MEMORY_19 (0x26)
//!        \b ADC12B_MEMORY_20 (0x28)
//!        \b ADC12B_MEMORY_21 (0x2A)
//!        \b ADC12B_MEMORY_22 (0x2C)
//!        \b ADC12B_MEMORY_23 (0x2E)
//!        \b ADC12B_MEMORY_24 (0x30)
//!        \b ADC12B_MEMORY_25 (0x32)
//!        \b ADC12B_MEMORY_26 (0x34)
//!        \b ADC12B_MEMORY_27 (0x36)
//!        \b ADC12B_MEMORY_28 (0x38)
//!        \b ADC12B_MEMORY_29 (0x3A)
//!        \b ADC12B_MEMORY_30 (0x3C)
//!        \b ADC12B_MEMORY_31 (0x3E)
//! \param inputSourceSelect is the input that will store the converted data
//!       into the specified memory buffer.
//!        Valid values are
//!        \b ADC12B_INPUT_A0	[Default]
//!        \b ADC12B_INPUT_A1
//!        \b ADC12B_INPUT_A2
//!        \b ADC12B_INPUT_A3
//!        \b ADC12B_INPUT_A4
//!        \b ADC12B_INPUT_A5
//!        \b ADC12B_INPUT_A6
//!        \b ADC12B_INPUT_A7
//!        \b ADC12B_INPUT_A8
//!        \b ADC12B_INPUT_A9
//!        \b ADC12B_INPUT_A10
//!        \b ADC12B_INPUT_A11
//!        \b ADC12B_INPUT_A12
//!        \b ADC12B_INPUT_A13
//!        \b ADC12B_INPUT_A14
//!        \b ADC12B_INPUT_A15
//!        \b ADC12B_INPUT_A16
//!        \b ADC12B_INPUT_A17
//!        \b ADC12B_INPUT_A18
//!        \b ADC12B_INPUT_A19
//!        \b ADC12B_INPUT_A20
//!        \b ADC12B_INPUT_A21
//!        \b ADC12B_INPUT_A22
//!        \b ADC12B_INPUT_A23
//!        \b ADC12B_INPUT_A24
//!        \b ADC12B_INPUT_A25
//!        \b ADC12B_INPUT_A26
//!        \b ADC12B_INPUT_A27
//!        \b ADC12B_INPUT_A28
//!        \b ADC12B_INPUT_A29
//!        \b ADC12B_INPUT_TCMAP
//!        \b ADC12B_INPUT_BATMAP
//!        Modified bits are \b ADC12INCHx of \b ADC12MCTLx register.
//! \param refVoltageSourceSelect is the reference voltage source to set
//!       as the upper/lower limits for the conversion stored in the specified memory.
//!        Valid values are
//!        \b ADC12B_VREFPOS_AVCC_VREFNEG_VSS  		[Default]
//!        \b ADC12B_VREFPOS_INTBUF_VREFNEG_VSS
//!        \b ADC12B_VREFPOS_EXTNEG_VREFNEG_VSS
//!        \b ADC12B_VREFPOS_EXTBUF_VREFNEG_VSS
//!        \b ADC12B_VREFPOS_EXTPOS_VREFNEG_VSS
//!        \b ADC12B_VREFPOS_AVCC_VREFNEG_EXTBUF
//!        \b ADC12B_VREFPOS_AVCC_VREFNEG_EXTPOS
//!        \b ADC12B_VREFPOS_INTBUF_VREFNEG_EXTPOS
//!        \b ADC12B_VREFPOS_AVCC_VREFNEG_INTBUF
//!        \b ADC12B_VREFPOS_EXTPOS_VREFNEG_INTBUF
//!        \b ADC12B_VREFPOS_AVCC_VREFNEG_EXTNEG
//!        \b ADC12B_VREFPOS_INTBUF_VREFNEG_EXTNEG
//!        \b ADC12B_VREFPOS_EXTPOS_VREFNEG_EXTNEG
//!        \b ADC12B_VREFPOS_EXTBUF_VREFNEG_EXTNEG
//!        Modified bits are \b ADC12VRSEL of \b ADC12MCTLx register.
//! \param endOfSequence indicates that the specified memory buffer will be
//!       the end of the sequence if a sequenced conversion mode is selected
//!        Valid values are
//!        \b ADC12B_NOTENDOFSEQUENCE - The specified memory buffer will NOT be the
//!                  end of the sequence OR a sequenced conversion mode is not
//!                  selected. [Default]
//!        \b ADC12B_ENDOFSEQUENCE  - The specified memory buffer will be the end
//!                  of the sequence.
//!        Modified bits are \b ADC12EOS of \b ADC12MCTLx register.
//!
//! Maps an input signal conversion into the selected memory buffer, as
//! well as the positive and negative reference voltages for each conversion
//! being stored into this memory buffer. If the internal reference is used for
//! the positive reference voltage, the internal REF module must be used to
//! control the voltage level. Note that if a conversion has been started with
//! the startConversion() function, then a call to disableConversions() is
//! required before this function may be called.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_memoryConfigure (unsigned int baseAddress,
    unsigned char memoryBufferControlIndex,
    unsigned char inputSourceSelect,
    unsigned int refVoltageSourceSelect,
    unsigned short endOfSequence)
{
    //Make sure the ENC bit is cleared before configuring a Memory Buffer Control
    ASSERT( !(HWREGB(baseAddress + OFS_ADC12CTL0_L) & ADC12ENC) );

    ASSERT(memoryBufferControlIndex <= ADC12B_MEMORY_31);
    ASSERT(inputSourceSelect <= ADC12B_INPUT_BATMAP);
    ASSERT(refVoltageSourceSelect <= ADC12B_VREFPOS_EXTBUF_VREFNEG_EXTNEG);

    //Set the offset in respect to ADC12MCTL0
    unsigned int memoryBufferControlOffset =
        (OFS_ADC12MCTL0 + memoryBufferControlIndex);

    //Reset the memory buffer control and Set the input source
    HWREG(baseAddress + memoryBufferControlOffset) =
        inputSourceSelect //Set Input Source
        + refVoltageSourceSelect //Set Vref+/-
        + endOfSequence; //Set End of Sequence
}

//*****************************************************************************
//
//! Enables selected ADC12B interrupt sources.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param interruptMask0 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be enabled. If the desired interrupt is not available in
//!			the selection for interruptMask0, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_IE0
//!        \b ADC12B_IE1
//!        \b ADC12B_IE2
//!        \b ADC12B_IE3
//!        \b ADC12B_IE4
//!        \b ADC12B_IE5
//!        \b ADC12B_IE6
//!        \b ADC12B_IE7
//!        \b ADC12B_IE8
//!        \b ADC12B_IE9
//!        \b ADC12B_IE10
//!        \b ADC12B_IE11
//!        \b ADC12B_IE12
//!        \b ADC12B_IE13
//!        \b ADC12B_IE14
//!        \b ADC12B_IE15
//! \param interruptMask1 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be enabled.  If the desired interrupt is not available in
//!			the selection for interruptMask1, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_IE16
//!        \b ADC12B_IE17
//!        \b ADC12B_IE18
//!        \b ADC12B_IE19
//!        \b ADC12B_IE20
//!        \b ADC12B_IE21
//!        \b ADC12B_IE22
//!        \b ADC12B_IE23
//!        \b ADC12B_IE24
//!        \b ADC12B_IE25
//!        \b ADC12B_IE26
//!        \b ADC12B_IE27
//!        \b ADC12B_IE28
//!        \b ADC12B_IE29
//!        \b ADC12B_IE30
//!        \b ADC12B_IE31
//! \param interruptMask2 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be enabled.  If the desired interrupt is not available in
//!			the selection for interruptMask2, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_INIE - Interrupt enable for a conversion in the result register
//!							is either greater than the ADC12LO or lower than the ADC12HI
//!							threshold.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_LOIE - Interrupt enable for the falling short of the lower
//!							limit interrupt of the window comparator for the result register.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_HIIE - Interrupt enable for the exceeding the upper limit
//!							of the window comparator for the result register.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_OVIE - Interrupt enable for a conversion that is about to
//!                  		save to a memory buffer that has not been read out yet.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_TOVIE -Interrupt enable for a conversion that is about
//!                  		to start before the previous conversion has been completed.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_RDYIE -Interrupt enable for the local buffered reference ready signal.
//!							GIE bit must be set to enable the interrupt.
//!
//! Enables the indicated ADC12B interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_enableInterrupt (unsigned int baseAddress,
    unsigned int interruptMask0,
    unsigned int interruptMask1,
    unsigned int interruptMask2)
{
    ASSERT(interruptMask0 <= (
			ADC12B_IE0 + ADC12B_IE1 + ADC12B_IE2 + ADC12B_IE3 + ADC12B_IE4 + ADC12B_IE5 +
			ADC12B_IE6 + ADC12B_IE7 + ADC12B_IE8 + ADC12B_IE9 + ADC12B_IE10 + ADC12B_IE11 +
			ADC12B_IE12 + ADC12B_IE13 + ADC12B_IE14 + ADC12B_IE15
			));

    ASSERT(interruptMask1 <= (
    		ADC12B_IE16 + ADC12B_IE17 + ADC12B_IE18 + ADC12B_IE19 + ADC12B_IE20 + ADC12B_IE21 +
    		ADC12B_IE22 + ADC12B_IE23 + ADC12B_IE24 + ADC12B_IE25 + ADC12B_IE26 + ADC12B_IE27 +
    		ADC12B_IE28 + ADC12B_IE29 + ADC12B_IE30 + ADC12B_IE31
    		));

    ASSERT(interruptMask2 <= (
			ADC12B_INIE + ADC12B_LOIE + ADC12B_HIIE + ADC12B_OVIE +	ADC12B_TOVIE + ADC12B_RDYIE
			));

    HWREG(baseAddress + OFS_ADC12IFGR0) &= ~interruptMask0;
    HWREG(baseAddress + OFS_ADC12IFGR1) &= ~interruptMask1;
    HWREG(baseAddress + OFS_ADC12IFGR2) &= ~interruptMask2;
    HWREG(baseAddress + OFS_ADC12IER0) |= interruptMask0;
    HWREG(baseAddress + OFS_ADC12IER1) |= interruptMask1;
    HWREG(baseAddress + OFS_ADC12IER2) |= interruptMask2;
}

//*****************************************************************************
//
//! Disables selected ADC12B interrupt sources.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param interruptMask0 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be disabled.  If the desired interrupt is not available in
//!			the selection for interruptMask0, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_IE0
//!        \b ADC12B_IE1
//!        \b ADC12B_IE2
//!        \b ADC12B_IE3
//!        \b ADC12B_IE4
//!        \b ADC12B_IE5
//!        \b ADC12B_IE6
//!        \b ADC12B_IE7
//!        \b ADC12B_IE8
//!        \b ADC12B_IE9
//!        \b ADC12B_IE10
//!        \b ADC12B_IE11
//!        \b ADC12B_IE12
//!        \b ADC12B_IE13
//!        \b ADC12B_IE14
//!        \b ADC12B_IE15
//! \param interruptMask1 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be disabled.  If the desired interrupt is not available in
//!			the selection for interruptMask1, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_IE16
//!        \b ADC12B_IE17
//!        \b ADC12B_IE18
//!        \b ADC12B_IE19
//!        \b ADC12B_IE20
//!        \b ADC12B_IE21
//!        \b ADC12B_IE22
//!        \b ADC12B_IE23
//!        \b ADC12B_IE24
//!        \b ADC12B_IE25
//!        \b ADC12B_IE26
//!        \b ADC12B_IE27
//!        \b ADC12B_IE28
//!        \b ADC12B_IE29
//!        \b ADC12B_IE30
//!        \b ADC12B_IE31
//! \param interruptMask2 is the bit mask of the memory buffer and overflow
//!       interrupt sources to be disabled.  If the desired interrupt is not available in
//!			the selection for interruptMask2, then simply pass in a '0' for this value.
//!        Valid values are
//!        \b ADC12B_INIE - Interrupt enable for a conversion in the result register
//!							is either greater than the ADC12LO or lower than the ADC12HI
//!							threshold.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_LOIE - Interrupt enable for the falling short of the lower
//!							limit interrupt of the window comparator for the result register.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_HIIE - Interrupt enable for the exceeding the upper limit
//!							of the window comparator for the result register.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_OVIE - Interrupt enable for a conversion that is about to
//!                  		save to a memory buffer that has not been read out yet.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_TOVIE -Interrupt enable for a conversion that is about
//!                  		to start before the previous conversion has been completed.
//!							GIE bit must be set to enable the interrupt.
//!        \b ADC12B_RDYIE -Interrupt enable for the local buffered reference ready signal.
//!							GIE bit must be set to enable the interrupt.
//! Disables the indicated ADC12B interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! Modified registers are \b ADC12CTL0 and \b ADC12IE.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_disableInterrupt (unsigned int baseAddress,
    unsigned int interruptMask0,
    unsigned int interruptMask1,
    unsigned int interruptMask2)
{
    ASSERT(interruptMask0 <= (
			ADC12B_IE0 + ADC12B_IE1 + ADC12B_IE2 + ADC12B_IE3 + ADC12B_IE4 + ADC12B_IE5 +
			ADC12B_IE6 + ADC12B_IE7 + ADC12B_IE8 + ADC12B_IE9 + ADC12B_IE10 + ADC12B_IE11 +
			ADC12B_IE12 + ADC12B_IE13 + ADC12B_IE14 + ADC12B_IE15
			));

    ASSERT(interruptMask1 <= (
    		ADC12B_IE16 + ADC12B_IE17 + ADC12B_IE18 + ADC12B_IE19 + ADC12B_IE20 + ADC12B_IE21 +
    		ADC12B_IE22 + ADC12B_IE23 + ADC12B_IE24 + ADC12B_IE25 + ADC12B_IE26 + ADC12B_IE27 +
    		ADC12B_IE28 + ADC12B_IE29 + ADC12B_IE30 + ADC12B_IE31
    		));

    ASSERT(interruptMask2 <= (
			ADC12B_INIE + ADC12B_LOIE + ADC12B_HIIE + ADC12B_OVIE +	ADC12B_TOVIE + ADC12B_RDYIE
			));


    HWREG(baseAddress + OFS_ADC12IER0) &= ~(interruptMask0);
    HWREG(baseAddress + OFS_ADC12IER1) &= ~(interruptMask1);
    HWREG(baseAddress + OFS_ADC12IER2) &= ~(interruptMask2);
}

//*****************************************************************************
//
//! Clears ADC12B selected interrupt flags.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param interruptRegisterChoice is either 0, 1, or 2, to choose the correct
//!       interrupt register to update
//! \param memoryInterruptFlagMask is the bit mask of the memory buffer and overflow
//!       interrupt flags to be cleared.
//!        Valid values are
//!        \b ADC12B_IFG0		interruptRegisterChoice = 0
//!        \b ADC12B_IFG1
//!        \b ADC12B_IFG2
//!        \b ADC12B_IFG3
//!        \b ADC12B_IFG4
//!        \b ADC12B_IFG5
//!        \b ADC12B_IFG6
//!        \b ADC12B_IFG7
//!        \b ADC12B_IFG8
//!        \b ADC12B_IFG9
//!        \b ADC12B_IFG10
//!        \b ADC12B_IFG11
//!        \b ADC12B_IFG12
//!        \b ADC12B_IFG13
//!        \b ADC12B_IFG14
//!        \b ADC12B_IFG15
//!        \b ADC12B_IFG16		interruptRegisterChoice = 1
//!        \b ADC12B_IFG17
//!        \b ADC12B_IFG18
//!        \b ADC12B_IFG19
//!        \b ADC12B_IFG20
//!        \b ADC12B_IFG21
//!        \b ADC12B_IFG22
//!        \b ADC12B_IFG23
//!        \b ADC12B_IFG24
//!        \b ADC12B_IFG25
//!        \b ADC12B_IFG26
//!        \b ADC12B_IFG27
//!        \b ADC12B_IFG28
//!        \b ADC12B_IFG29
//!        \b ADC12B_IFG30
//!        \b ADC12B_IFG31
//!        \b ADC12B_INIFG		interruptRegisterChoice = 2
//!        \b ADC12B_LOIFG
//!        \b ADC12B_HIIFG
//!        \b ADC12B_OVIFG
//!        \b ADC12B_TOVIFG
//!        \b ADC12B_RDYIFG
//! The selected ADC12B interrupt flags are cleared, so that it no longer asserts.
//! The memory buffer interrupt flags are only cleared when the memory buffer is
//! accessed. Note that the overflow interrupts do not have an interrupt flag to
//! clear; they must be accessed directly from the interrupt vector.
//!
//! Modified registers are \b ADC12IFG.
//
//! \return NONE
//
//*****************************************************************************
void ADC12B_clearInterrupt (unsigned int baseAddress,
	unsigned char interruptRegisterChoice,
    unsigned int memoryInterruptFlagMask)
{
	HWREG(baseAddress + OFS_ADC12IFGR0 + 2*interruptRegisterChoice) &= memoryInterruptFlagMask;

}

//*****************************************************************************
//
//! Returns the status of the selected memory interrupt flags.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param interruptRegisterChoice is either 0, 1, or 2, to choose the correct
//!       interrupt register to update
//! \param memoryInterruptFlagMask is the bit mask of the memory buffer and overflow
//!       interrupt flags to be cleared.
//!        Valid values are
//!        \b ADC12B_IFG0		interruptRegisterChoice = 0
//!        \b ADC12B_IFG1
//!        \b ADC12B_IFG2
//!        \b ADC12B_IFG3
//!        \b ADC12B_IFG4
//!        \b ADC12B_IFG5
//!        \b ADC12B_IFG6
//!        \b ADC12B_IFG7
//!        \b ADC12B_IFG8
//!        \b ADC12B_IFG9
//!        \b ADC12B_IFG10
//!        \b ADC12B_IFG11
//!        \b ADC12B_IFG12
//!        \b ADC12B_IFG13
//!        \b ADC12B_IFG14
//!        \b ADC12B_IFG15
//!        \b ADC12B_IFG16		interruptRegisterChoice = 1
//!        \b ADC12B_IFG17
//!        \b ADC12B_IFG18
//!        \b ADC12B_IFG19
//!        \b ADC12B_IFG20
//!        \b ADC12B_IFG21
//!        \b ADC12B_IFG22
//!        \b ADC12B_IFG23
//!        \b ADC12B_IFG24
//!        \b ADC12B_IFG25
//!        \b ADC12B_IFG26
//!        \b ADC12B_IFG27
//!        \b ADC12B_IFG28
//!        \b ADC12B_IFG29
//!        \b ADC12B_IFG30
//!        \b ADC12B_IFG31
//!        \b ADC12B_INIFG		interruptRegisterChoice = 2
//!        \b ADC12B_LOIFG
//!        \b ADC12B_HIIFG
//!        \b ADC12B_OVIFG
//!        \b ADC12B_TOVIFG
//!        \b ADC12B_RDYIFG
//! Returns the status of the selected memory interrupt flags. Note that the
//! overflow interrupts do not have an interrupt flag to clear; they must be
//! accessed directly from the interrupt vector.
//!
//! \return The current interrupt flag status for the corresponding mask.
//
//*****************************************************************************
unsigned char ADC12B_getInterruptStatus (unsigned int baseAddress,
	unsigned char interruptRegisterChoice,
	unsigned int memoryInterruptFlagMask)
{
    return ( HWREG(baseAddress + OFS_ADC12IFGR0 + 2*interruptRegisterChoice)
    		& memoryInterruptFlagMask );
}

//*****************************************************************************
//
//! Enables/Starts an Analog-to-Digital Conversion.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param startingMemoryBufferIndex is the memory buffer that will hold the
//!       first or only conversion.
//!        Valid values are
//!        \b START_AT_ADC12MEM0	[Default]
//!        \b START_AT_ADC12MEM1
//!        \b START_AT_ADC12MEM2
//!        \b START_AT_ADC12MEM3
//!        \b START_AT_ADC12MEM4
//!        \b START_AT_ADC12MEM5
//!        \b START_AT_ADC12MEM6
//!        \b START_AT_ADC12MEM7
//!        \b START_AT_ADC12MEM8
//!        \b START_AT_ADC12MEM9
//!        \b START_AT_ADC12MEM10
//!        \b START_AT_ADC12MEM11
//!        \b START_AT_ADC12MEM12
//!        \b START_AT_ADC12MEM13
//!        \b START_AT_ADC12MEM14
//!        \b START_AT_ADC12MEM15
//!        \b START_AT_ADC12MEM16
//!        \b START_AT_ADC12MEM17
//!        \b START_AT_ADC12MEM18
//!        \b START_AT_ADC12MEM19
//!        \b START_AT_ADC12MEM20
//!        \b START_AT_ADC12MEM21
//!        \b START_AT_ADC12MEM22
//!        \b START_AT_ADC12MEM23
//!        \b START_AT_ADC12MEM24
//!        \b START_AT_ADC12MEM25
//!        \b START_AT_ADC12MEM26
//!        \b START_AT_ADC12MEM27
//!        \b START_AT_ADC12MEM28
//!        \b START_AT_ADC12MEM29
//!        \b START_AT_ADC12MEM30
//!        \b START_AT_ADC12MEM31
//!        Modified bits are \b ADC12STARTADDx of \b ADC12CTL1 register.
//! \param conversionSequenceModeSelect determines the ADC operating mode.
//!        Valid values are
//!        \b ADC12B_SINGLECHANNEL - one-time conversion of a single channel into
//!           a single memory buffer. [Default]
//!        \b ADC12B_SEQOFCHANNELS - one time conversion of multiple channels
//!           into the specified starting memory buffer and each subsequent
//!           memory buffer up until the conversion is stored in a memory buffer
//!           dedicated as the end-of-sequence by the memory's control register.
//!        \b ADC12B_REPEATED_SINGLECHANNEL - repeated conversions of one channel
//!           into a single memory buffer.
//!        \b ADC12B_REPEATED_SEQOFCHANNELS - repeated conversions of multiple
//!           channels into the specified starting memory buffer and each
//!           subsequent memory buffer up until the conversion is stored in a
//!           memory buffer dedicated as the end-of-sequence by the memory's
//!           control register.
//!        Modified bits are \b ADC12CONSEQx of \b ADC12CTL1 register.
//!
//! This function  enables/starts the conversion process of the ADC.
//! If the sample/hold signal source chosen during initialization was
//! ADC12OSC, then the conversion is started immediately, otherwise the chosen
//! sample/hold signal source starts the conversion by a rising edge of the
//! signal. Keep in mind when selecting conversion modes, that for sequenced
//! and/or repeated modes, to keep the sample/hold-and-convert process
//! continuing without a trigger from the sample/hold signal source, the
//! multiple samples must be enabled using the ADC12B_setupSamplingTimer()
//! function. Note that after this function is called, the
//! ADC12B_stopConversions() has to be called to re-initialize the ADC,
//! reconfigure a memory buffer control, enable/disable the sampling timer, or
//! to change the internal reference voltage.
//!
//! Modified registers are \b ADC12CTL0 and \b ADC12CTL1.
//! \return NONE
//
//*****************************************************************************
void ADC12B_startConversion (unsigned int baseAddress,
    unsigned int startingMemoryBufferIndex,
    unsigned char conversionSequenceModeSelect)
{
    ASSERT(startingMemoryBufferIndex <= ADC12B_MEMORY_15);
    ASSERT(conversionSequenceModeSelect <= ADC12B_REPEATED_SEQOFCHANNELS);

    //Reset the ENC bit to set the starting memory address and conversion mode
    //sequence
    HWREGB(baseAddress + OFS_ADC12CTL0_L) &= ~(ADC12ENC);
    //Reset the bits about to be set
    HWREG(baseAddress + OFS_ADC12CTL3) &= ~(ADC12CSTARTADD_31);
    HWREG(baseAddress + OFS_ADC12CTL1) &= ~(ADC12CONSEQ_3);

    HWREGB(baseAddress + OFS_ADC12CTL3) |= startingMemoryBufferIndex;
    HWREGB(baseAddress + OFS_ADC12CTL1) |= conversionSequenceModeSelect;
    HWREGB(baseAddress + OFS_ADC12CTL0_L) |= ADC12ENC + ADC12SC;
}

//*****************************************************************************
//
//! Disables the ADC from converting any more signals.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param preempt specifies if the current conversion should be preemptively
//!       stopped before the end of the conversion.
//!        Valid values are
//!        \b ADC12B_COMPLETECONVERSION - Allows the ADC12B to end the current
//!                  conversion before disabling conversions.
//!        \b ADC12B_PREEMPTCONVERSION - Stops the ADC12B immediately, with
//!                  unpredictable results of the current conversion.
//!
//! Disables the ADC from converting any more signals.
//! If there is a conversion in progress, this function can stop it immediately
//! if the preempt parameter is set as TRUE, by changing the conversion mode to
//! single-channel, single-conversion and disabling conversions. If the
//! conversion mode is set as single-channel, single-conversion and this
//! function is called without preemption, then the ADC core conversion status
//! is polled until the conversion is complete before disabling conversions to
//! prevent unpredictable data. If the ADC12B_startConversion() has been called,
//! then this function has to be called to re-initialize the ADC, reconfigure a
//! memory buffer control, enable/disable the sampling pulse mode, or change the
//! internal reference voltage.
//!
//! Modified registers are \b ADC12CTL0 and \b ADC12CTL1.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_disableConversions (unsigned int baseAddress, unsigned short preempt)
{
    if (ADC12B_PREEMPTCONVERSION == preempt){
        HWREGB(baseAddress + OFS_ADC12CTL1_L) &= ~(ADC12CONSEQ_3);
        //Reset conversion sequence mode to single-channel, single-conversion
    } else if (~(HWREGB(baseAddress + OFS_ADC12CTL1_L) & ADC12CONSEQ_3)){
        //To prevent preemption of a single-channel, single-conversion we must
        //wait for the ADC core to finish the conversion.
        while (ADC12B_isBusy(baseAddress)) ;
    }

    HWREGB(baseAddress + OFS_ADC12CTL0_L) &= ~(ADC12ENC);
}

//*****************************************************************************
//
//! Returns the raw contents of the specified memory buffer.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param memryBufferIndex is the specified Memory Buffer to read.
//!        Valid values are
//!        \b ADC12B_MEMORY_0
//!        \b ADC12B_MEMORY_1
//!        \b ADC12B_MEMORY_2
//!        \b ADC12B_MEMORY_3
//!        \b ADC12B_MEMORY_4
//!        \b ADC12B_MEMORY_5
//!        \b ADC12B_MEMORY_6
//!        \b ADC12B_MEMORY_7
//!        \b ADC12B_MEMORY_8
//!        \b ADC12B_MEMORY_9
//!        \b ADC12B_MEMORY_10
//!        \b ADC12B_MEMORY_11
//!        \b ADC12B_MEMORY_12
//!        \b ADC12B_MEMORY_13
//!        \b ADC12B_MEMORY_14
//!        \b ADC12B_MEMORY_15
//!        \b ADC12B_MEMORY_16
//!        \b ADC12B_MEMORY_17
//!        \b ADC12B_MEMORY_18
//!        \b ADC12B_MEMORY_19
//!        \b ADC12B_MEMORY_20
//!        \b ADC12B_MEMORY_21
//!        \b ADC12B_MEMORY_22
//!        \b ADC12B_MEMORY_23
//!        \b ADC12B_MEMORY_24
//!        \b ADC12B_MEMORY_25
//!        \b ADC12B_MEMORY_26
//!        \b ADC12B_MEMORY_27
//!        \b ADC12B_MEMORY_28
//!        \b ADC12B_MEMORY_29
//!        \b ADC12B_MEMORY_30
//!        \b ADC12B_MEMORY_31
//!
//! Returns the raw contents of the specified memory buffer. The format of the
//! content depends on the read-back format of the data: if the data is in
//! signed 2's complement format then the contents in the memory buffer will be
//! left-justified with the least-significant bits as 0's, whereas if the data
//! is in unsigned format then the contents in the memory buffer will be
//! right-justified with the most-significant bits as 0's.
//!
//! \return A Signed Integer of the contents of the specified memory buffer.
//
//*****************************************************************************
int ADC12B_getResults (unsigned int baseAddress, unsigned char memoryBufferIndex)
{
    ASSERT(memoryBufferIndex <= ADC12B_MEMORY_31);

    return ( HWREG(baseAddress + (0x60 + memoryBufferIndex)) );
    //(0x60 + memoryBufferIndex) == offset of ADC12MEMx
}

//*****************************************************************************
//
//! Use to change the resolution of the converted data.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param resolutionSelect determines the resolution of the converted data.
//!        Valid values are
//!        \b ADC12B_RESOLUTION_8BIT
//!        \b ADC12B_RESOLUTION_10BIT
//!        \b ADC12B_RESOLUTION_12BIT [Default]
//!        Modified bits are \b ADC12RESx of \b ADC12CTL2 register.
//!
//! This function can be used to change the resolution of the converted data
//! from the default of 12-bits.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_setResolution (unsigned int baseAddress,
    unsigned char resolutionSelect)
{
    ASSERT(resolutionSelect <= ADC12B_RESOLUTION_12BIT);

    HWREGB(baseAddress + OFS_ADC12CTL2_L) &= ~(ADC12RES_3);
    HWREGB(baseAddress + OFS_ADC12CTL2_L) |= resolutionSelect;
}

//*****************************************************************************
//
//! Use to invert or un-invert the sample/hold signal.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param invertedSignal set if the sample/hold signal should be inverted
//!        Valid values are
//!        \b ADC12B_NONINVERTEDSIGNAL - a sample-and-hold of an input signal for
//!                  conversion will be started on a rising edge of the
//!                  sample/hold signal. [Default]
//!        \b ADC12B_INVERTEDSIGNAL  - a sample-and-hold of an input signal for
//!                  conversion will be started on a falling edge of the
//!                  sample/hold signal.
//!        Modified bits are \b ADC12ISSH of \b ADC12CTL1 register.
//!
//! This function can be used to invert or un-invert the sample/hold signal.
//! Note that if a conversion has been started with the startConversion()
//! function, then a call to disableConversions() is required before this
//! function may be called.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_setSampleHoldSignalInversion (unsigned int baseAddress,
    unsigned int invertedSignal)
{
    //Make sure the ENC bit is cleared before using this function
    ASSERT( !(HWREGB(baseAddress + OFS_ADC12CTL0_L) & ADC12ENC) );

    HWREG(baseAddress + OFS_ADC12CTL1) &= ~(ADC12ISSH);
    HWREG(baseAddress + OFS_ADC12CTL1) |= invertedSignal;
}

//*****************************************************************************
//
//! Use to set the read-back format of the converted data.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param readBackFormat is the specified format to store the conversions in
//!       the memory buffer.
//!        Valid values are
//!        \b ADC12B_UNSIGNED_BINARY 	[Default]
//!        \b ADC12B_SIGNED_2SCOMPLEMENT
//!        Modified bits are \b ADC12DF of \b ADC12CTL2 register.
//!
//! Sets the format of the converted data: how it will be stored into the
//! memory buffer, and how it should be read back. The format can be set as
//! right-justified (default), which indicates that the number will be unsigned,
//! or left-justified, which indicates that the number will be signed in
//! 2's complement format. This change affects all memory buffers for subsequent
//! conversions.
//!
//! \return NONE
//
//*****************************************************************************
void ADC12B_setDataReadBackFormat (unsigned int baseAddress,
    unsigned short readBackFormat)
{
    ASSERT(readBackFormat <= ADC12B_SIGNED_2SCOMPLEMENT);

    HWREGB(baseAddress + OFS_ADC12CTL2_L) &= ~(ADC12DF);
    HWREGB(baseAddress + OFS_ADC12CTL2_L) |= readBackFormat;
}

//*****************************************************************************
//
//! Use to set the ADC's power conservation mode if the sampling rate is
//!	at 50-ksps or less.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param powerMode is the specified maximum sampling rate.
//!        Valid values are
//!        \b ADC12B_REGULARPOWERMODE 	- If sampling rate is greater than 50-ksps,
//!										there is no power saving feature available. [Default]
//!        \b ADC12B_LOWPOWERMODE		- If sampling rate is less than or equal
//!										to 50-ksps, select this value to save power
//!        Modified bits are \b ADC12SR of \b ADC12CTL2 register.
//!
//! Sets ADC's power mode. If the user has a sampling rate greater than 50-ksps, then
//! he/she can only enable ADC12B_REGULARPOWERMODE.  If the sampling rate is 50-ksps or less,
//! the user can enable ADC12B_LOWPOWERMODE granting additional power savings.
//! \return NONE
//
//*****************************************************************************
void ADC12B_setAdcPowerMode (unsigned int baseAddress,
    unsigned short powerMode)
{
    ASSERT(powerMode <= ADC12B_MAXSAMPLINGRATE_50KSPS);

    HWREGB(baseAddress + OFS_ADC12CTL2_L) &= ~(ADC12PWRMD);
    HWREGB(baseAddress + OFS_ADC12CTL2_L) |= powerMode;
}

//*****************************************************************************
//
//! Returns the address of the specified memory buffer for the DMA module.
//!
//! \param baseAddress is the base address of the ADC12B module.
//! \param memoryIndex is the memory buffer to return the address of.
//!        Valid values are
//!        \b ADC12B_MEMORY_0  (0x00)
//!        \b ADC12B_MEMORY_1  (0x02)
//!        \b ADC12B_MEMORY_2  (0x04)
//!        \b ADC12B_MEMORY_3  (0x06)
//!        \b ADC12B_MEMORY_4  (0x08)
//!        \b ADC12B_MEMORY_5  (0x0A)
//!        \b ADC12B_MEMORY_6  (0x0C)
//!        \b ADC12B_MEMORY_7  (0x0E)
//!        \b ADC12B_MEMORY_8  (0x10)
//!        \b ADC12B_MEMORY_9  (0x12)
//!        \b ADC12B_MEMORY_10 (0x14)
//!        \b ADC12B_MEMORY_11 (0x16)
//!        \b ADC12B_MEMORY_12 (0x18)
//!        \b ADC12B_MEMORY_13 (0x1A)
//!        \b ADC12B_MEMORY_14 (0x1C)
//!        \b ADC12B_MEMORY_15 (0x1E)
//!        \b ADC12B_MEMORY_16 (0x20)
//!        \b ADC12B_MEMORY_17 (0x22)
//!        \b ADC12B_MEMORY_18 (0x24)
//!        \b ADC12B_MEMORY_19 (0x26)
//!        \b ADC12B_MEMORY_20 (0x28)
//!        \b ADC12B_MEMORY_21 (0x2A)
//!        \b ADC12B_MEMORY_22 (0x2C)
//!        \b ADC12B_MEMORY_23 (0x2E)
//!        \b ADC12B_MEMORY_24 (0x30)
//!        \b ADC12B_MEMORY_25 (0x32)
//!        \b ADC12B_MEMORY_26 (0x34)
//!        \b ADC12B_MEMORY_27 (0x36)
//!        \b ADC12B_MEMORY_28 (0x38)
//!        \b ADC12B_MEMORY_29 (0x3A)
//!        \b ADC12B_MEMORY_30 (0x3C)
//!        \b ADC12B_MEMORY_31 (0x3E)
//!
//! Returns the address of the specified  memory buffer. This can be used in
//! conjunction with the DMA to store the converted data directly to memory.
//!
//! \return address of the specified memory buffer
//
//*****************************************************************************
unsigned long ADC12B_getMemoryAddressForDMA (unsigned int baseAddress,
    unsigned char memoryIndex)
{
    return ( baseAddress + (0x60 + memoryIndex) );
    //(0x60 + memoryIndex) == offset of ADC12MEMx
}

//*****************************************************************************
//
//! Returns the busy status of the ADC12B core.
//!
//! \param baseAddress is the base address of the ADC12B module.
//!
//! Returns the status of the ADC core if there is a conversion currently taking
//! place.
//!
//! \return ADC12B_BUSY or ADC12B_NOTBUSY dependent if there is a conversion
//!        currently taking place.
//
//*****************************************************************************
unsigned short ADC12B_isBusy (unsigned int baseAddress)
{
    if (HWREGB(baseAddress + OFS_ADC12CTL1_L) & ADC12BUSY){
        return ( ADC12B_BUSY) ;
    } else   {
        return ( ADC12B_NOTBUSY) ;
    }
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
