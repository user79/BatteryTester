// based on  MSP430F552x Demo - ADC12, Sample A0, Set P1.0 if A0 > 0.5*AVcc demo
// see BSD license for that starter code below

#include "driverlib/MSP430F5xx_6xx/pmm.h"
#include "driverlib/MSP430F5xx_6xx/sfr.h"
#include "driverlib/MSP430F5xx_6xx/ucs.h"
#include "driverlib/MSP430F5xx_6xx/wdt_a.h"
#include "intrinsics.h"
#include "msp430f5529.h"
#include "types.h"
#include <msp430.h>


#include "BCUart.h" // Include the backchannel UART "library"
#include "hal.h"    // Modify hal.h to select your hardware

// Global variables
WORD rxByteCount; // Momentarily stores the number of bytes received
BYTE buf_bcuartToUsb[BC_RXBUF_SIZE]; // Same size as the UART's rcv buffer
BYTE buf_usbToBcuart[128];           // This can be any size
volatile BYTE batteryInserted;
volatile BYTE batteryRemoved;



long takeSample(void) 
{
    ADC12CTL0 |= ADC12SC; // Start sampling/conversion
    __bis_SR_register(LPM0_bits + GIE);
    long temp = ADC12MEM0 * 4 / 5;
    return temp;
}

void SendToUART(long num) 
{
      // send to the backchannel uart ("MSP Application UART" in device manager)
      buf_usbToBcuart[0] = (num / 1000) + '0';
      buf_usbToBcuart[1] = (num / 100) % 10 + '0';
      buf_usbToBcuart[2] = (num / 10) % 10 + '0';
      buf_usbToBcuart[3] = (num) % 10 + '0';
      buf_usbToBcuart[4] = '\r';
      buf_usbToBcuart[5] = '\n';
      bcUartSend(buf_usbToBcuart, 6);
}

long takeSampleAndSendToUART(void) 
{
    ADC12CTL0 |= ADC12SC; // Start sampling/conversion
    __bis_SR_register(LPM0_bits + GIE);

     // bcUartSend("Hello world", 11); //buf_usbToBcuart, rxByteCount); // debug
      long calibAdcVal = ADC12MEM0 * 4 / 5; // approx conversion to millivolts

      // send to the backchannel uart ("MSP Application UART" in device manager)
      buf_usbToBcuart[0] = (calibAdcVal / 1000) + '0';
      buf_usbToBcuart[1] = (calibAdcVal / 100) % 10 + '0';
      buf_usbToBcuart[2] = (calibAdcVal / 10) % 10 + '0';
      buf_usbToBcuart[3] = (calibAdcVal) % 10 + '0';
      buf_usbToBcuart[4] = '\r';
      buf_usbToBcuart[5] = '\n';
      bcUartSend(buf_usbToBcuart, 6);

      return calibAdcVal;
}




int main(void) {
  WDTCTL = WDTPW + WDTHOLD;         // Stop WDT
  ADC12CTL0 = ADC12SHT02 + ADC12ON; // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;             // Use sampling timer
  ADC12IE = 0x01;                   // Enable interrupt
  ADC12CTL0 |= ADC12ENC;
  P6SEL |= 0x01; // P6.0 ADC option select
  P1DIR |= 0x01; // P1.0 output
  //buf_usbToBcuart[4] = '\r';
  //buf_usbToBcuart[5] = '\n';
  //int counter = 0;
  batteryInserted = 0;
  batteryRemoved = 0;

  // MSP430 USB requires a Vcore setting of at least 2.  2 is high enough
  // for 8MHz MCLK, below.
  PMM_setVCore(PMM_CORE_LEVEL_2);
  initPorts();         // Config all the GPIOS for low-power (output low)
  initClocks(8000000); // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
  bcUartInit();        // Init the back-channel UART



  // pseudocode: 
  // take conversion
  // if battery voltage level detected, then
  //    do 3 cycles of 3 readings:
  //         - 1k load only (always connected)
  //         - with added 100 ohm load (~15mA)
  //         - with added 10  ohm load (~150mA)
  //    then send the data out to UART backchannel, 
  //       and turn off the readings until voltage disconnected and reconnected

  while (1) {
    //ADC12CTL0 |= ADC12SC; // Start sampling/conversion
    //__bis_SR_register(LPM0_bits + GIE); // LPM0, ADC12_ISR will force exit
    //takeSample();

    if ((P2IN & BIT1) == 0) { // if pushbutton is pressed
      const unsigned long int SAMPLEDELAY = 400000;
      const int numSamples = 16;
      int i=0;
      long cumsum = 0;

      // first take the "unloaded" samples
      for(i =0; i< numSamples; i++)
      {
         __delay_cycles(SAMPLEDELAY);
         //takeSampleAndSendToUART();
         cumsum += takeSample();          
      }
      SendToUART(cumsum/numSamples) ;

      
      // next take the "loaded---100 ohm" samples
      P1OUT |= BIT6;
      cumsum = 0;
      for(i =0; i< numSamples; i++)
      {
         __delay_cycles(SAMPLEDELAY);
         //takeSampleAndSendToUART();
         cumsum += takeSample();          
      }
      P1OUT &= ~BIT6;
      SendToUART(cumsum/numSamples) ;


      // next take the "loaded---10 ohm" samples
      P1OUT |= BIT4;
      cumsum = 0;
      for(i =0; i< numSamples; i++)
      {
         __delay_cycles(SAMPLEDELAY);
         //takeSampleAndSendToUART();
         cumsum += takeSample();          
      }
      P1OUT &= ~BIT4;
      SendToUART(cumsum/numSamples) ;


      //batteryInserted = 0; // wait until the next battery insertion

      // P1OUT |= BIT6;
      //__delay_cycles(800000);
      // P1OUT &= ~BIT6;
      //__delay_cycles(8000000);

      //counter++;
      //if (counter >= 6) {
      //  counter = 0;
      //  P1OUT ^= BIT6;
      //}

    }


    __delay_cycles(1000000); // 1/8 second event loop pacing
    __no_operation();        // For debugger
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(ADC12_VECTOR))) ADC12_ISR(void)
#else
#error Compiler not supported!
#endif
{
  switch (__even_in_range(ADC12IV, 34)) {
  case 0:
    break; // Vector  0:  No interrupt
  case 2:
    break; // Vector  2:  ADC overflow
  case 4:
    break;                  // Vector  4:  ADC timing overflow
  case 6:                   // Vector  6:  ADC12IFG0
    if (ADC12MEM0 >= 0x1ff) // ADC12MEM = A0 > 0.5AVcc?
    {
       P1OUT |= BIT0;                        // P1.0 = 1
      //if (batteryRemoved == 1)
     //     batteryInserted = 1;
      //batteryRemoved = 0;
      //__delay_cycles(1000000); // todo: remove delay from ISR, replace with timer
    } else {
       P1OUT &= ~BIT0;                       // P1.0 = 0
     // batteryRemoved = 1;
      //batteryInserted = 0;
      //__delay_cycles(1000000); // todo: remove delay from ISR, replace with timer
    }

    __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
  case 8:
    break; // Vector  8:  ADC12IFG1
  case 10:
    break; // Vector 10:  ADC12IFG2
  case 12:
    break; // Vector 12:  ADC12IFG3
  case 14:
    break; // Vector 14:  ADC12IFG4
  case 16:
    break; // Vector 16:  ADC12IFG5
  case 18:
    break; // Vector 18:  ADC12IFG6
  case 20:
    break; // Vector 20:  ADC12IFG7
  case 22:
    break; // Vector 22:  ADC12IFG8
  case 24:
    break; // Vector 24:  ADC12IFG9
  case 26:
    break; // Vector 26:  ADC12IFG10
  case 28:
    break; // Vector 28:  ADC12IFG11
  case 30:
    break; // Vector 30:  ADC12IFG12
  case 32:
    break; // Vector 32:  ADC12IFG13
  case 34:
    break; // Vector 34:  ADC12IFG14
  default:
    break;
  }
}

/* --COPYRIGHT--,BSD_EX
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
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//   MSP430F552x Demo - ADC12, Sample A0, Set P1.0 if A0 > 0.5*AVcc
//
//   Description: A single sample is made on A0 with reference to AVcc.
//   Software sets ADC12SC to start sample and conversion - ADC12SC
//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
//
//                MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//     Vin -->|P6.0/CB0/A0  P1.0|--> LED
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************