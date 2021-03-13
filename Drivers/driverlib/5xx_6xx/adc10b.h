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
#ifndef __MSP430WARE_ADC10B_H__
#define __MSP430WARE_ADC10B_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_ADC10_B__

//*****************************************************************************
//
//The following are values that can be passed to ADC10_B_init() in the
//sampleTimerSourceSelect parameter.
//
//*****************************************************************************
#define ADC10B_SAMPLEHOLDSOURCE_SC (ADC10SHS_0)
#define ADC10B_SAMPLEHOLDSOURCE_1  (ADC10SHS_1)
#define ADC10B_SAMPLEHOLDSOURCE_2  (ADC10SHS_2)
#define ADC10B_SAMPLEHOLDSOURCE_3  (ADC10SHS_3)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_init() in the
//clockSourceSelect parameter.
//
//*****************************************************************************
#define ADC10B_CLOCKSOURCE_ADC10OSC (ADC10SSEL_0)
#define ADC10B_CLOCKSOURCE_ACLK     (ADC10SSEL_1)
#define ADC10B_CLOCKSOURCE_MCLK     (ADC10SSEL_2)
#define ADC10B_CLOCKSOURCE_SMCLK    (ADC10SSEL_3)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_init() in the
//clockSourceDivider parameter.
//Note: Bit 8-9 determines if the pre-divider is 1, 4, or 64
//Bits 5-7 determine the the post-divider 1-8 (0x00 - 0xE0)
//
//*****************************************************************************
#define ADC10B_CLOCKDIVIDER_1   (ADC10DIV_0 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_2   (ADC10DIV_1 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_3   (ADC10DIV_2 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_4   (ADC10DIV_3 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_5   (ADC10DIV_4 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_6   (ADC10DIV_5 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_7   (ADC10DIV_6 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_8   (ADC10DIV_7 + ADC10PDIV_0)
#define ADC10B_CLOCKDIVIDER_12  (ADC10DIV_2 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_16  (ADC10DIV_3 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_20  (ADC10DIV_4 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_24  (ADC10DIV_5 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_28  (ADC10DIV_6 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_32  (ADC10DIV_7 + ADC10PDIV_1)
#define ADC10B_CLOCKDIVIDER_64  (ADC10DIV_0 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_128 (ADC10DIV_1 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_192 (ADC10DIV_2 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_256 (ADC10DIV_3 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_320 (ADC10DIV_4 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_384 (ADC10DIV_5 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_448 (ADC10DIV_6 + ADC10PDIV_2)
#define ADC10B_CLOCKDIVIDER_512 (ADC10DIV_7 + ADC10PDIV_2)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_setupSamplingTimer() in
//the clockCycleHoldCount parameter.
//
//*****************************************************************************
#define ADC10B_CYCLEHOLD_4_CYCLES    (ADC10SHT_0)
#define ADC10B_CYCLEHOLD_8_CYCLES    (ADC10SHT_1)
#define ADC10B_CYCLEHOLD_16_CYCLES   (ADC10SHT_2)
#define ADC10B_CYCLEHOLD_32_CYCLES   (ADC10SHT_3)
#define ADC10B_CYCLEHOLD_64_CYCLES   (ADC10SHT_4)
#define ADC10B_CYCLEHOLD_96_CYCLES   (ADC10SHT_5)
#define ADC10B_CYCLEHOLD_128_CYCLES  (ADC10SHT_6)
#define ADC10B_CYCLEHOLD_192_CYCLES  (ADC10SHT_7)
#define ADC10B_CYCLEHOLD_256_CYCLES  (ADC10SHT_8)
#define ADC10B_CYCLEHOLD_384_CYCLES  (ADC10SHT_9)
#define ADC10B_CYCLEHOLD_512_CYCLES  (ADC10SHT_10)
#define ADC10B_CYCLEHOLD_768_CYCLES  (ADC10SHT_11)
#define ADC10B_CYCLEHOLD_1024_CYCLES (ADC10SHT_12)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_setupSamplingTimer() in
//the multipleSamplesEnabled parameter.
//
//*****************************************************************************
#define ADC10B_MULTIPLESAMPLESDISABLE ( !(ADC10MSC) )
#define ADC10B_MULTIPLESAMPLESENABLE  (ADC10MSC)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_memoryConfigure() in the
//inputSourceSelect parameter.
//
//*****************************************************************************
#define ADC10B_INPUT_A0              (ADC10INCH_0)
#define ADC10B_INPUT_A1              (ADC10INCH_1)
#define ADC10B_INPUT_A2              (ADC10INCH_2)
#define ADC10B_INPUT_A3              (ADC10INCH_3)
#define ADC10B_INPUT_A4              (ADC10INCH_4)
#define ADC10B_INPUT_A5              (ADC10INCH_5)
#define ADC10B_INPUT_A6              (ADC10INCH_6)
#define ADC10B_INPUT_A7              (ADC10INCH_7)
#define ADC10B_INPUT_VEREF_P         (ADC10INCH_8)
#define ADC10B_INPUT_VEREF_N         (ADC10INCH_9)
#define ADC10B_INPUT_TEMPSENSOR      (ADC10INCH_10)
#define ADC10B_INPUT_BATTERYMONITOR  (ADC10INCH_11)
#define ADC10B_INPUT_A12             (ADC10INCH_12)
#define ADC10B_INPUT_A13             (ADC10INCH_13)
#define ADC10B_INPUT_A14             (ADC10INCH_14)
#define ADC10B_INPUT_A15             (ADC10INCH_15)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_memoryConfigure() in the
//positiveRefVoltageSourceSelect parameter.
//
//*****************************************************************************
#define ADC10B_VREFPOS_AVCC ( !(ADC10SREF0 + ADC10SREF1) )
#define ADC10B_VREFPOS_INT  (ADC10SREF0)
#define ADC10B_VREFPOS_EXT  (ADC10SREF1)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_memoryConfigure() in the
//negativeRefVoltageSourceSelect parameter.
//
//*****************************************************************************
#define ADC10B_VREFNEG_AVSS ( !(ADC10SREF2) )
#define ADC10B_VREFNEG_EXT  (ADC10SREF2)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_startConversion() in the
//conversionSequenceModeSelect parameter.
//
//*****************************************************************************
#define ADC10B_SINGLECHANNEL          (ADC10CONSEQ_0)
#define ADC10B_SEQOFCHANNELS          (ADC10CONSEQ_1)
#define ADC10B_REPEATED_SINGLECHANNEL (ADC10CONSEQ_2)
#define ADC10B_REPEATED_SEQOFCHANNELS (ADC10CONSEQ_3)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_disableConversions() in
//the preempt parameter.
//
//*****************************************************************************
#define ADC10B_COMPLETECONVERSION (0x0)
#define ADC10B_PREEMPTCONVERSION  (0x1)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_setDataReadBackFormat()
//in the readBackFormat parameter.
//
//*****************************************************************************
#define ADC10B_UNSIGNED_BINARY     ( !(ADC10DF) )
#define ADC10B_SIGNED_2SCOMPLEMENT (ADC10DF)

//*****************************************************************************
//
//The following are values that can be passed to ADC10B_setResolution() in the
//resolutionSelect parameter.
//
//*****************************************************************************
#define ADC10B_RESOLUTION_8BIT  ( !(ADC10RES) )
#define ADC10B_RESOLUTION_10BIT (ADC10RES)

//*****************************************************************************
//
//The following are values that can be passed to
//ADC10B_setSampleHoldSignalInversion() in the invertedSignal parameter.
//
//*****************************************************************************
#define ADC10B_NONINVERTEDSIGNAL ( !(ADC10ISSH) )
#define ADC10B_INVERTEDSIGNAL    (ADC10ISSH)

//*****************************************************************************
//
//The following are values that can be passed to
//ADC10B_setReferenceBufferSamplingRate() in the samplingRateSelect parameter.
//
//*****************************************************************************
#define ADC10B_MAXSAMPLINGRATE_200KSPS ( !(ADC10SR) )
#define ADC10B_MAXSAMPLINGRATE_50KSPS  (ADC10SR)

//*****************************************************************************
//
//The following are values that are returned by ADC10B_isBusy().
//
//*****************************************************************************
#define ADC10B_NOTBUSY (0x0)
#define ADC10B_BUSY    (0x1)


//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************

extern unsigned short ADC10B_init (unsigned int baseAddress,
    unsigned int sampleHoldSourceSelect,
    unsigned char clockSourceSelect,
    unsigned int clockSourceDivider);

extern void ADC10B_enable (unsigned int baseAddress);

extern void ADC10B_disable (unsigned int baseAddress);

extern void ADC10B_setupSamplingTimer (unsigned int baseAddress,
    unsigned int clockCycleHoldCount,
    unsigned short multipleSamplesEnabled);

extern void ADC10B_disableSamplingTimer (unsigned int baseAddress);

extern void ADC10B_memoryConfigure (unsigned int baseAddress,
    unsigned char inputSourceSelect,
    unsigned char positiveRefVoltageSourceSelect,
    unsigned char negativeRefVoltageSourceSelect);

extern void ADC10B_enableInterrupt (unsigned int baseAddress,
    unsigned char interruptMask);

extern void ADC10B_disableInterrupt (unsigned int baseAddress,
    unsigned char interruptMask);

extern void ADC10B_clearInterrupt (unsigned int baseAddress,
    unsigned char interruptFlagMask);

extern unsigned char ADC10B_getInterruptStatus (unsigned int baseAddress,
    unsigned char interruptFlagMask);

extern void ADC10B_startConversion (unsigned int baseAddress,
    unsigned char conversionSequenceModeSelect);

void ADC10B_disableConversions (unsigned int baseAddress,
    unsigned short preempt);

extern int ADC10B_getResults (unsigned int baseAddress);

void ADC10B_setResolution (unsigned int baseAddress,
    unsigned char resolutionSelect);

extern void ADC10B_setSampleHoldSignalInversion (unsigned int baseAddress,
    unsigned int invertedSignal);

extern void ADC10B_setDataReadBackFormat (unsigned int baseAddress,
    unsigned short readBackFormat);

extern void ADC10B_enableReferenceBurst (unsigned int baseAddress);

extern void ADC10B_disableReferenceBurst (unsigned int baseAddress);

extern void ADC10B_setReferenceBufferSamplingRate (unsigned int baseAddress,
    unsigned short samplingRateSelect);

extern void ADC10B_setWindowComp (unsigned int baseAddress,
    unsigned int highThreshold,
    unsigned int lowThreshold);

extern unsigned long ADC10B_getMemoryAddressForDMA (unsigned int baseAddress);

extern unsigned short ADC10B_isBusy (unsigned int baseAddress);

#endif
