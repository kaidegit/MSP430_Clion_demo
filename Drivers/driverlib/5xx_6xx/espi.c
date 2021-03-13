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
//eUSCI_spi.c - Driver for the SPI Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/debug.h"
#include "driverlib/5xx_6xx/espi.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! Initializes the SPI Master block.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource selects Clock source. Valid values are
//!         \b eSPI_CLOCKSOURCE_ACLK
//!         \b eSPI_CLOCKSOURCE_SMCLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//! \param msbFirst controls the direction of the receive and transmit shift
//!      register. Valid values are
//!         \b eSPI_MSB_FIRST
//!         \b eSPI_LSB_FIRST [Default Value]
//! \param clockPhase is clock phase select.
//!         Valid values are
//!         \b eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT  [Default Value]
//!         \b eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity_InactivityHIGH is clock polarity select.
//!         Valid values are
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_LOW  [Default Value]
//! \param spiMode is SPI mode select.
//!         Valid values are
//!         \b eSPI_3PIN [Default Value]
//!         \b eSPI_4PIN_UCxSTE_ACTIVE_HIGH
//!         \b eSPI_4PIN_UCxSTE_ACTIVE_LOW
//! Upon successful initialization of the SPI master block, this function
//! will have set the bus speed for the master, but the SPI Master block
//! still remains disabled and must be enabled with eSPI_enable()
//!
//! Modified bits are \b UCCKPH, \b UCCKPL, \b UC7BIT, \b UCMSB,\b UCSSELx, 
//! \b UCSWRST bits of \b UCAxCTLW0 register
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned short eSPI_masterInit (unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned long clockSourceFrequency,
    unsigned long desiredSpiClock,
    unsigned int msbFirst,
    unsigned int clockPhase,
    unsigned int clockPolarity,
    unsigned char spiMode
    )
{
    ASSERT(
        (eSPI_CLOCKSOURCE_ACLK == selectClockSource) ||
        (eSPI_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    ASSERT(  (eSPI_MSB_FIRST == msbFirst) ||
        (eSPI_LSB_FIRST == msbFirst)
        );

    ASSERT(  (eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
        (eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
        );

    ASSERT(  (eSPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
        (eSPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
        );
    
    ASSERT(
        (eSPI_3PIN == spiMode) ||
        (eSPI_4PIN_UCxSTE_ACTIVE_HIGH == spiMode)
        (eSPI_4PIN_UCxSTE_ACTIVE_LOW == spiMode)
        );

    //Disable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Reset OFS_UCAxCTLW0 values
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB);


    //Reset OFS_UCAxCTLW0 values
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCSSEL_3);

    //Select Clock
    HWREG(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;


    HWREG(baseAddress + OFS_UCAxBRW) =
        (unsigned int)(clockSourceFrequency / desiredSpiClock);


    /*
     * Configure as SPI master mode.
     * Clock phase select, polarity, msb
     * UCMST = Master mode
     * UCSYNC = Synchronous mode
     * UCMODE_0 = 3-pin SPI
     */
    HWREG(baseAddress + OFS_UCAxCTLW0) |= (
        msbFirst +
        clockPhase +
        clockPolarity +
        UCMST +
        UCSYNC +
        UCMODE_0
        );
    //No modulation
    HWREG(baseAddress + OFS_UCAxMCTLW) = 0;

    return ( STATUS_SUCCESS) ;
}

//*****************************************************************************
//
//! Selects 4Pin Functionality
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param select4PinFunctionality selects Clock source. Valid values are
//!         \b eSPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS
//!         \b eSPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE
//! This function should be invoked only in 4-wire mode. Invoking this function 
//! has no effect in 3-wire mode.
//!
//! Modified bits are \b UCSTEM bit of \b UCAxCTLW0 register
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
void eSPI_select4PinFunctionality (unsigned int baseAddress,
    unsigned char select4PinFunctionality
    )
{
  ASSERT(  (eSPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS == select4PinFunctionality) ||
           (eSPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE == select4PinFunctionality)
        );
      
  HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCSTEM;
  HWREG(baseAddress + OFS_UCAxCTLW0) |= select4PinFunctionality;
}

//*****************************************************************************
//
//! Initializes the SPI Master clock.At the end of this function call, SPI 
//! module is left enabled.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//!
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register and 
//! \b UCAxBRW register
//!
//! \return None
//
//*****************************************************************************
void eSPI_masterChangeClock (unsigned int baseAddress,
    unsigned long clockSourceFrequency,
    unsigned long desiredSpiClock
    )
{
  //Disable the USCI Module
  HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

  HWREG(baseAddress + OFS_UCAxBRW) =
        (unsigned int)(clockSourceFrequency / desiredSpiClock);
  
  //Reset the UCSWRST bit to enable the USCI Module
  HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Initializes the SPI Slave block.
//!
//! \param baseAddress is the base address of the SPI Slave module.
//! \param msbFirst controls the direction of the receive and transmit shift
//!      register. Valid values are
//!         \b eSPI_MSB_FIRST
//!         \b eSPI_LSB_FIRST [Default Value]
//! \param clockPhase is clock phase select.
//!         Valid values are
//!         \b eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT [Default Value]
//!         \b eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity_InactivityHIGH is clock polarity select.
//!         Valid values are
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_LOW [Default Value]
//! Upon successful initialization of the SPI slave block, this function
//! will have initailized the slave block, but the SPI Slave block
//! still remains disabled and must be enabled with eSPI_enable()
//!
//! Modified bits are \b UCMSB, \b UC7BIT, \b UCMST, \b UCCKPL, \b UCCKPH, 
//! \b UCMODE_3, \b UCSWRST bits of \b UCAxCTLW0
//!
//! \return STATUS_SUCCESS
//*****************************************************************************
unsigned short eSPI_slaveInit (unsigned int baseAddress,
    unsigned int msbFirst,
    unsigned int clockPhase,
    unsigned int clockPolarity
    )
{
    ASSERT(
        (eSPI_MSB_FIRST == msbFirst) ||
        (eSPI_LSB_FIRST == msbFirst)
        );

    ASSERT(
        (eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT == clockPhase) ||
        (eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT == clockPhase)
        );

    ASSERT(
        (eSPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
        (eSPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
        );

    //Disable USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0)  |= UCSWRST;

    //Reset OFS_UCAxCTLW0 register
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCMSB +
                                            UC7BIT +
                                            UCMST +
                                            UCCKPL +
                                            UCCKPH +
                                            UCMODE_3
                                            );


    //Clock polarity, phase select, msbFirst, SYNC, Mode0
    HWREG(baseAddress + OFS_UCAxCTLW0) |= ( clockPhase +
                                            clockPolarity +
                                            msbFirst +
                                            UCSYNC +
                                            UCMODE_0
                                            );


    return ( STATUS_SUCCESS) ;
}


//*****************************************************************************
//
//! Changes the SPI colock phase and polarity.At the end of this function call, 
//! SPI module is left enabled.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param clockPhase is clock phase select.
//!         Valid values are
//!         \b eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT  [Default Value]
//!         \b eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity_InactivityHIGH is clock polarity select.
//!         Valid values are
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!         \b eSPI_CLOCKPOLARITY_INACTIVITY_LOW  [Default Value]
//!
//! Modified bits are \b UCSWRST, \b UCCKPH, \b UCCKPL, \b UCSWRST bits of 
//! \b UCAxCTLW0
//!
//! \return None
//
//*****************************************************************************
void eSPI_changeClockPhasePolarity (unsigned int baseAddress,
    unsigned char clockPhase,
    unsigned char clockPolarity
    )
{
  
   ASSERT(  (eSPI_CLOCKPOLARITY_INACTIVITY_HIGH == clockPolarity) ||
        (eSPI_CLOCKPOLARITY_INACTIVITY_LOW == clockPolarity)
        );

  //Disable the USCI Module
  HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
  
  HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCCKPH + UCCKPL);

  HWREG(baseAddress + OFS_UCAxCTLW0) |= (
        clockPhase +
        clockPolarity
          );
  
  //Reset the UCSWRST bit to enable the USCI Module
  HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Transmits a byte from the SPI Module.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param transmitData data to be transmitted from the SPI module
//!
//! This function will place the supplied data into SPI trasmit data register
//! to start transmission
//!
//! Modified register is \b UCAxTXBUF
//@
//! \return None.
//
//*****************************************************************************
void eSPI_transmitData ( unsigned int baseAddress,
    unsigned char transmitData
    )
{
    HWREG(baseAddress + OFS_UCAxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the SPI Module.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! This function reads a byte of data from the SPI receive data Register.
//!
//! \return Returns the byte received from by the SPI module, cast as an
//! unsigned char.
//
//*****************************************************************************
unsigned char eSPI_receiveData (unsigned int baseAddress)
{
    return ( HWREG(baseAddress + OFS_UCAxRXBUF)) ;
}

//*****************************************************************************
//
//! Enables individual SPI interrupt sources.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated SPI interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b eSPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b eSPI_TRANSMIT_INTERRUPT - Transmit interrupt
//!
//! Modified registers are \b UCAxIFG and \b UCAxIE
//!
//! \return None.
//
//*****************************************************************************
void eSPI_enableInterrupt (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT( (eSPI_RECEIVE_INTERRUPT == mask) ||
        (eSPI_TRANSMIT_INTERRUPT == mask)
        );

    HWREG(baseAddress + OFS_UCAxIFG) &=  ~mask;
    HWREG(baseAddress + OFS_UCAxIE) |= mask;
}

//*****************************************************************************
//
//! Disables individual SPI interrupt sources.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated SPI interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b eSPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b eSPI_TRANSMIT_INTERRUPT - Transmit interrupt
//!
//! Modified register is \b UCAxIE
//!
//! \return None.
//
//*****************************************************************************
void eSPI_disableInterrupt (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(  (eSPI_RECEIVE_INTERRUPT == mask) ||
        (eSPI_TRANSMIT_INTERRUPT == mask)
        );

    HWREG(baseAddress + OFS_UCAxIE) &= ~mask;
}

//*****************************************************************************
//
//! Gets the current SPI interrupt status.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the SPI  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b eSPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b eSPI_TRANSMIT_INTERRUPT - Transmit interrupt
//!
//! Modified registers are \b UCAxIFG.
//!
//! \return The current interrupt status as the mask of the set flags
//
//*****************************************************************************
unsigned char eSPI_getInterruptStatus (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(
        (eSPI_RECEIVE_INTERRUPT == mask) ||
        (eSPI_TRANSMIT_INTERRUPT == mask)
        );

    return ( HWREG(baseAddress + OFS_UCAxIFG) & mask );
}

//*****************************************************************************
//
//! Clears the selected SPI interrupt status flag.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the masked interrupt flag to be cleared.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b eSPI_RECEIVE_INTERRUPT -Receive interrupt
//! - \b eSPI_TRANSMIT_INTERRUPT - Transmit interrupt
//! Modified registers are \b UCAxIFG.
//!
//! \return None
//
//*****************************************************************************
void eSPI_clearInterruptFlag (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT(
        (eSPI_RECEIVE_INTERRUPT == mask) ||
        (eSPI_TRANSMIT_INTERRUPT == mask)
        );

    HWREG(baseAddress + OFS_UCAxIFG) &=  ~mask;
}

//*****************************************************************************
//
//! Enables the SPI block.
//!
//! \param baseAddress is the base address of the USCI SPI module.
//!
//! This will enable operation of the SPI block.
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register.
//!
//! \return None.
//
//*****************************************************************************
void eSPI_enable (unsigned int baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the SPI block.
//!
//! \param baseAddress is the base address of the USCI SPI module.
//!
//! This will disable operation of the SPI block.
//!
//! Modified bits are \b UCSWRST bit of \b UCAxCTLW0 register.
//!
//! \return None.
//
//*****************************************************************************
void eSPI_disable (unsigned int baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the SPI for the DMA module.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! Returns the address of the SPI RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
unsigned long eSPI_getReceiveBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCAxRXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the SPI for the DMA module.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! Returns the address of the SPI TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
unsigned long eSPI_getTransmitBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCAxTXBUF );
}

//*****************************************************************************
//
//! Indicates whether or not the SPI bus is busy.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! This function returns an indication of whether or not the SPI bus is
//! busy.This function checks the status of the bus via UCBBUSY bit
//!
//! \return eSPI_BUSY if the SPI module trasmitting or receiving
//! is busy; otherwise, returns eSPI_NOT_BUSY.
//
//*****************************************************************************
unsigned char eSPI_isBusy (unsigned int baseAddress)
{
    //Return the bus busy status.
    return (HWREG(baseAddress + OFS_UCAxSTATW) & UCBBUSY);
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************

