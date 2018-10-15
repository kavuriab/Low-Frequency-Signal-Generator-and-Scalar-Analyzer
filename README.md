# Low-Frequency-Signal-Generator-and-Scalar-Analyzer
In this work involved designing a system capable of capturing analog signals and generating various waveforms. It had a command line interface capable of controlling the system and providing measurement data back to user.


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
	while(PUSH_BUTTON);
}
// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

   // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 , 5 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0xB0;                       // set drive strength to 2mA
 	GPIO_PORTB_AFSEL_R |= 0xB0;                      // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK| GPIO_PCTL_PB5_SSI2FSS; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xB0;                        // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                        // must be enabled when SPO=1

     // Configure the SSI2 as a SPI master, mode 3, 16bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 20;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2



}






// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

float frequency, amplitude, frequency2, frequency1, someValue=0, raw=0, ampl, instVolt, freq3=0;
uint32_t lookupTable[4096], DeltaPhase,acc=0, i, itteration=0;
char F1[30], F2[30], volt[30], v[30];


int16_t readadcslow()
{

    // Configure AN0 as an analog input
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
	GPIO_PORTE_AFSEL_R |= 0x04;                      // select alternative functions for AN0 (PE2)
    GPIO_PORTE_DEN_R &= ~0x04;                       // turn off digital operation on pin PE2
    GPIO_PORTE_AMSEL_R |= 0x04;                      // turn on analog operation on pin PE2
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 1;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}
int16_t readAdc0Ss3()
{

    ADC0_PSSI_R |= ADC_PSSI_SS3;
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);
    instVolt=(raw*5)/4096;
    raw=ADC0_SSFIFO3_R;
    return raw;
}


void Timer1Isr()
{


			    acc=acc+DeltaPhase;
			    SSI2_DR_R=lookupTable[acc>>20];
	            TIMER1_ICR_R=TIMER_ICR_TATOCINT;
	            //while (SSI2_SR_R & SSI_SR_BSY);

}


uint16_t checkWhichCommand(char checkString[],uint16_t whichCommand) {
	//char check[]="dc\0";
	 //putsUart0(checkString);
  // Checking if it is a regular wave form, or dc or sweep
  if (strcmp(checkString, "sine") == 0 ) {
    return 1;
  }if (strcmp(checkString, "square") == 0){
	  return 2;
  }if (strcmp(checkString, "sawtooth") == 0){
	  return 3;
  }if (strcmp(checkString, "dc") == 0) {
	return 4;
  }  if (strcmp(checkString, "sweep") == 0) {
    return 5;
  } if (strcmp(checkString, "reset") == 0){
	  return 6;
  } if (strcmp(checkString, "voltage") == 0){
	  return 7;
  }else {
    return whichCommand;
  }
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{


    //Initialize everything as string
    char wave[10], amp[10], freq[10], totalString[50]={"\0"}, freq2[10];
    char enteredCharacter, echar[10];
    uint16_t  flag=0,totalCharacters = 0, sequence = 0, totalStringLength = 0, backspaceSequence = 0, i, whichCommand = 0;
    uint16_t amplitudeInFloat = 0.00 , frequencyInFloat = 0.00 ;
    // Initialize hardware
    initHw();


     RED_LED=1;
     waitMicrosecond(10000);
     RED_LED=0;

    // Display greeting
    putsUart0("\n\rEnter Input\r\n");
    putsUart0(totalString);

	while(1)
	{

    // Wait for PB press
    waitPbPress();


        //For continuous iteration
        while (1) {
            enteredCharacter = getcUart0();
            if (enteredCharacter > 31) {
                /* Entered a valid character */
                if ((enteredCharacter >= 'a' && enteredCharacter <= 'z') || (enteredCharacter >='A' && enteredCharacter <= 'Z')) {
                    /* Entered an alphabet */
                    totalString[totalStringLength++] = enteredCharacter;
                    putcUart0(enteredCharacter);
                    continue;
                } else if (enteredCharacter == 32 && totalStringLength > 0) {
                    /* Entered a space */
                    //Checking what type of wave is generated
                	//putsUart0(totalString);
                    whichCommand = checkWhichCommand(totalString, whichCommand);
                    totalString[totalStringLength++] = enteredCharacter;
                    putcUart0(enteredCharacter);
                    continue;
                } else if (((enteredCharacter >= 48 && enteredCharacter <= 57) || enteredCharacter==46) ) {
                    /* Entered a number */
                    if (sequence == 1) {
                        totalString[totalStringLength++] = enteredCharacter;
                        putcUart0(enteredCharacter);
                        continue;
                    } else {
                        /* Entered number for frequency */
                        totalString[totalStringLength++] = enteredCharacter;
                        putcUart0(enteredCharacter);
                        continue;
                    }
                }else {
                	totalString[totalStringLength++]=' ';
                	putcUart0(enteredCharacter);
                }
            } else if (enteredCharacter == 13) {
                    totalString[totalStringLength] = '\0';
                    break;
            } else if (enteredCharacter == 8) {
                /* Entered a backspace */
                if (totalStringLength > 0) {
                    totalString[totalStringLength--] = '\0';
                } else {
                    continue;
                }
            }
        }
          if (whichCommand == 1) {
            /* sine */
            for (i = 0; ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {

                	  break;
                  }

                  flag = 1;
                  wave[totalCharacters++] = totalString[i];
              }
              flag = 0;
              wave[totalCharacters] = '\0';
              totalCharacters = 0;

              for (; ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                      break;
                  }

                  flag = 1;
                  freq[totalCharacters++] = totalString[i];
                  frequency=atof(freq);
              }

              flag = 0;
              freq[totalCharacters] = '\0';
              totalCharacters = 0;

              for (;totalString[i] != 0 ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                      break;
                  }

                  flag = 1;
                  amp[totalCharacters++] = totalString[i];
                  amplitude=atof(amp);
              }
              flag = 0;
              amp[totalCharacters] = '\0';
              totalCharacters = 0;


              putsUart0("\n\r Wave : ");
              putsUart0(wave);
              putsUart0("\n\r Frequency : ");
              putsUart0(freq);
              putsUart0("\n\r Amplitude :  ");
              putsUart0(amp);

             /* for(i=0;i<strlen(totalString);i++){
              		totalString[i]='\0';
                        		}*/
                 //putsUart0(totalString);



              	while(itteration <4096)

              	{

              		if (amplitude <= 0)
              		{
              			lookupTable[itteration] = (0x3000+0x7DF+(((0xFFF-0x7DF)/4.9)*(amplitude/1.05)*sin((2*3.14*itteration)/4096)));
              		}
              		if (amplitude > 0)
              		{
              			lookupTable[itteration] = (0x3000+0x7DF+((0x7DF/5.04)*(amplitude/1.05)*sin((2*3.14*itteration)/4096)));
              		}


              		itteration++;
              	}

              	DeltaPhase=pow(2,32)*frequency/100000;


                SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // turn-on timer
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
                TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
                TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
                TIMER1_TAILR_R = 0x190; // set load value to 40e6 for 100k Hz interrupt rate
                TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts
                NVIC_EN0_R |= 1 << (INT_TIMER1A-16); // turn-on interrupt 37 (TIMER1A)
              	 TIMER1_CTL_R |= TIMER_CTL_TAEN; // turn-on timer
              //

               waitMicrosecond(10000);
            } else if (whichCommand == 2) {
                //square
                for (i = 0; ; ++i) {
                      if (totalString[i] == 32 && flag == 0) {
                          continue;
                      }
                      if (totalString[i] == 32 && flag == 1) {

                    	  break;
                      }

                      flag = 1;
                      wave[totalCharacters++] = totalString[i];
                  }
                  flag = 0;
                  wave[totalCharacters] = '\0';
                  totalCharacters = 0;

                  for (; ; ++i) {
                      if (totalString[i] == 32 && flag == 0) {
                          continue;
                      }
                      if (totalString[i] == 32 && flag == 1) {
                          break;
                      }

                      flag = 1;
                      freq[totalCharacters++] = totalString[i];
                      frequency=atof(freq);
                  }

                  flag = 0;
                  freq[totalCharacters] = '\0';
                  totalCharacters = 0;

                  for (;totalString[i] != 0 ; ++i) {
                      if (totalString[i] == 32 && flag == 0) {
                          continue;
                      }
                      if (totalString[i] == 32 && flag == 1) {
                          break;
                      }

                      flag = 1;
                      amp[totalCharacters++] = totalString[i];
                      amplitude=atof(amp);
                  }
                  flag = 0;
                  amp[totalCharacters] = '\0';
                  totalCharacters = 0;

                  putsUart0("\n\r Wave : ");
                  putsUart0(wave);
                  putsUart0("\n\r Frequency : ");
                  putsUart0(freq);
                  putsUart0("\n\r Amplitude :  ");
                  putsUart0(amp);
                                       while(itteration<4096){
                               		for(itteration=0;itteration<2049;itteration++)
                               		{
                               			lookupTable[itteration] = (0x3000+650+((-(amplitude)+5)*4096/5));
                               		}
                               		for(itteration=2049;itteration<=4095;itteration++)
                               		{
                               			lookupTable[itteration] = (0x3000+500-((-(amplitude)+5)*4096/24));
                               		}

                                     }
                               	DeltaPhase=pow(2,32)*frequency/100000;


                                 SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // turn-on timer
                                 TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
                                 TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
                                 TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
                                 TIMER1_TAILR_R = 0x190; // set load value to 40e6 for 100k Hz interrupt rate
                                 TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts
                                 NVIC_EN0_R |= 1 << (INT_TIMER1A-16); // turn-on interrupt 37 (TIMER1A)
                               	 TIMER1_CTL_R |= TIMER_CTL_TAEN; // turn-on timer
                               //

                }else if (whichCommand == 3) {
                        //sawtooth
                        for (i = 0; ; ++i) {
                              if (totalString[i] == 32 && flag == 0) {
                                  continue;
                              }
                              if (totalString[i] == 32 && flag == 1) {

                            	  break;
                              }

                              flag = 1;
                              wave[totalCharacters++] = totalString[i];
                          }
                          flag = 0;
                          wave[totalCharacters] = '\0';
                          totalCharacters = 0;

                          for (; ; ++i) {
                              if (totalString[i] == 32 && flag == 0) {
                                  continue;
                              }
                              if (totalString[i] == 32 && flag == 1) {
                                  break;
                              }

                              flag = 1;
                              freq[totalCharacters++] = totalString[i];
                              frequency=atof(freq);
                          }

                          flag = 0;
                          freq[totalCharacters] = '\0';
                          totalCharacters = 0;

                          for (;totalString[i] != 0 ; ++i) {
                              if (totalString[i] == 32 && flag == 0) {
                                  continue;
                              }
                              if (totalString[i] == 32 && flag == 1) {
                                  break;
                              }

                              flag = 1;
                              amp[totalCharacters++] = totalString[i];
                              amplitude=atof(amp);
                          }
                          flag = 0;
                          amp[totalCharacters] = '\0';
                          totalCharacters = 0;

                          putsUart0("\n\r Wave : ");
                          putsUart0(wave);
                          putsUart0("\n\r Frequency : ");
                          putsUart0(freq);
                          putsUart0("\n\r Amplitude :  ");
                          putsUart0(amp);


                                       	/*while(itteration <4096)

                                       	{

                                       		if (amplitude <= 0)
                                       		{
                                       			lookupTable[itteration] = (0x3000+0x7AF-(((0xFFF-0x7DF)/4.4)*amplitude*atan(tan((2*3.14*itteration)/4096))));
                                       		}
                                       		if (amplitude > 0)
                                       		{
                                       			lookupTable[itteration] = (0x3000+0x7AF-((0x7AF/5.02)*(amplitude)*atan(tan((2*3.14*itteration)/4096))));
                                       		}


                                       		itteration++;
                                       	}*/
                          while(itteration <4096){
                        	  if(amplitude <= 0){
                        		  lookupTable[itteration]= (0x3000+0x7DF+(((0xFFF-0x7DF)/4.9)*(amplitude)*itteration/2048));
                        	  }
                        	  if(amplitude > 0){

                        		  lookupTable[itteration]= (0x3000+0x7DF+((0x7DF/5.04)*(amplitude)*itteration/2048));
                        	  }
                        	  itteration++;
                          }

                                       	DeltaPhase=pow(2,32)*frequency/100000;


                                         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // turn-on timer
                                         TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
                                         TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
                                         TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
                                         TIMER1_TAILR_R = 0x190; // set load value to 40e6 for 100k Hz interrupt rate
                                         TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts
                                         NVIC_EN0_R |= 1 << (INT_TIMER1A-16); // turn-on interrupt 37 (TIMER1A)
                                       	 TIMER1_CTL_R |= TIMER_CTL_TAEN; // turn-on timer
                                       //

                        }else if(whichCommand == 4) {
                  for (i = 0; ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                    break;
                  }
                  flag = 1;
                  wave[totalCharacters++] = totalString[i];
              }

           flag = 0;
              wave[totalCharacters] = '\0';
              totalCharacters = 0;

              for (; ; ++i) {
                              if (totalString[i] == 32 && flag == 0) {
                                  continue;
                              }
                              if (totalString[i] == 32 && flag == 1) {
                                break;
                              }
                              flag = 1;
                              amp[totalCharacters++] = totalString[i];
                              amplitude = atof(amp);
                          }
          	   putsUart0("\n\r wave = DC ");
               putsUart0("\n\r Amplitude: ");
               putsUart0(amp);
               TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
               waitMicrosecond(1000);
               //while(itteration < 4096){
               SSI2_DR_R= 0X3000+(((-(amplitude)+5)/10)*4095);
               //itteration++;
               //}



//               while(itteration<4096){
//            	   lookupTable[itteration] = 0x3000 + 0x7CF + (((0x7CF/5.04) * amplitude)/4096);
//            	   itteration++;
               /*if(amplitude >= -5 && amplitude <= 5){
                        lookupTable[itteration]=0X3000+(((-(amplitude)+5)/10)*4095);
                        itteration++;
               }else{
            	   putsUart0("check the input");
               }*/
               //}
            //   flag = 0;
             //  wave[totalCharacters] = '\0';
               //totalCharacters = 0;
              } else if (whichCommand == 5) {
                    //sweep
              for (i = 0; ; ++i) {
                               if (totalString[i] == 32 && flag == 0) {
                                   continue;
                               }
                               if (totalString[i] == 32 && flag == 1) {
                                 break;
                               }
                               flag = 1;
                               wave[totalCharacters++] = totalString[i];
                           }

                        flag = 0;
                           wave[totalCharacters] = '\0';
                           totalCharacters = 0;
                           putsUart0("\n\r command:");
                           putsUart0(wave);

              for (; ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                    break;
                  }
                  flag = 1;
                  freq[totalCharacters++] = totalString[i];
                  frequency1=atof(freq);
              }


              flag=0;
              freq[totalCharacters] = '\0';
              totalCharacters = 0;
              putsUart0("\n\r Frequency1 ");
              putsUart0(freq);

                for (; totalString[i] != 0 ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                      break;
                  }

                  flag = 1;
                  freq2[totalCharacters++] = totalString[i];
              }

              frequency2=atof(freq2);
              flag=0;
              freq2[totalCharacters] = '\0';
              totalCharacters = 0;

              putsUart0("\n\r Frequency2 ");
              putsUart0(freq2);

              for (;totalString[i] != 0 ; ++i) {
                  if (totalString[i] == 32 && flag == 0) {
                      continue;
                  }
                  if (totalString[i] == 32 && flag == 1) {
                      break;
                  }

                  flag = 1;
                  amp[totalCharacters++] = totalString[i];
                  amplitude=atof(amp);
              }
              flag = 0;
              amp[totalCharacters] = '\0';
              totalCharacters = 0;
              putsUart0("\n\r Amplitude :  ");
              putsUart0(amp);
             /* putsUart0("\n\r Enter the amplitude:");
              int i =0;
              echar[i] = getcUart0();
              i++;
              echar[i] = '\0';

              ampl = atoi(echar);*/



uint32_t z;

                           //someValue= (frequency2-frequency1)/20;
                           //freq3=frequency1;
                          while(frequency1<frequency2)
                          {
                            //  someValue= frequency2+200;
                           	while(itteration <4096)

                           	{

                           		if (amplitude <= 0)
                           		{
                           			lookupTable[itteration] = (0x3000+0x7DF+(((0xFFF-0x7DF)/4.9)*(amplitude)*sin((2*3.14*itteration)/4096)));
                           		}
                           		if (amplitude > 0)
                           		{
                           			lookupTable[itteration] = (0x3000+0x7DF+((0x7DF/5.04)*(amplitude)*sin((2*3.14*itteration)/4096)));
                           		}


                           		itteration++;
                          	}

                           	DeltaPhase=pow(2,32)*frequency1/100000;


                           readadcslow();
                           readAdc0Ss3();

                           SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // turn-on timer
                            TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // turn-off timer before reconfiguring
                            TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; // configure as 32-bit timer (A+B)
                            TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
                            TIMER1_TAILR_R = 0x190; // set load value to 40e6 for 100k Hz interrupt rate
                            TIMER1_IMR_R = TIMER_IMR_TATOIM; // turn-on interrupts
                            NVIC_EN0_R |= 1 << (INT_TIMER1A-16); // turn-on interrupt 37 (TIMER1A)
                          	 TIMER1_CTL_R |= TIMER_CTL_TAEN; // turn-on timer
                          //

                             waitMicrosecond(1000000);
                            sprintf(F1,"%f",frequency1);
                            putsUart0("\n\r");
                            putsUart0(F1);
                            putsUart0(":");
                            frequency1= frequency1+100;




                            sprintf(v,"%f",instVolt);
                            putsUart0("\n\r");
                            putsUart0(v);
                            putsUart0(":");


              }

            }


            else if(whichCommand == 7){

            	readAdc0Ss3();
            	instVolt=(raw*5)/4096;
            	sprintf(volt,"%f",instVolt);
            	putsUart0("\n\r");
            	putsUart0(volt);
            	putsUart0(":");
            }
            else if(whichCommand == 6){
        	  putsUart0("\n\r System is reset ");
        	  ResetISR();
          }
            else{

            	putsUart0("\nUnsupported value");
            	//putsUart0(totalString);
            }

      	for(i=0;i<strlen(totalString);i++){
               totalString[i]='\0';

                   		}






	}


}









