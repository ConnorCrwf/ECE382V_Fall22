// Seeed_HC12main.c
// Runs on MSP432
// Lab 1 start project for ECE382V, Technology for Embedded IoT
// Use the SysTick timer to request interrupts at a particular period.
// Jonathan Valvano
// August 12, 2022

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2022, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/


// built-in LED1 connected to P1.0
// P1.0, P2.0 are an output to profiling scope/logic analyzer
// UART output to PC for debugging
//   UCA2RXD (VCP receive) connected to P3.2
//   UCA2TXD (VCP transmit) connected to P3.3
// HC12 pin connections
//   5 SET to GPIO      J2.18 from LaunchPad to HC12  (GPIO output){MSP432 P3.0}
//   4 SO to serial RxD J1.3 from HC12 to LaunchPad (UCA2TXD RxD){MSP432 P3.2}
//   3 SI to serial TxD J1.4 from LaunchPad to HC12  (UCA2TXD TxD){MSP432 P3.3}
//   2 GND
//   1 VCC 1N4004 diode to +5V Vin (plus side of diode on +5V)
//   Power pin, the requirements of 3.2V to 5.5V
//   DC power supply, the supply current is not less
//   than 200mA. Note: If the module is to work
//   for a long time in the transmit state, it is
//   recommended that the power supply voltage
//   of more than 4.5V when connected to a
//   1N4007/1N4004 diode, to avoid the module built-in
//   LDO fever.
// SSD1306 LCD connections to I2CB1
//   GND    ground
//   VIN    3.3V
//   SDA    SSD1306 SDA <> P6.4 I2C data SDA_S
//   SCL    SSD1306 SCL <- P6.5 I2C clock SCL_S with 1.5k pullup to 3.3V


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Clock.h"
#include "..\inc\UART0.h"
#include "..\inc\UART1.h"
#include "..\inc\SSD1306_I2C.h"
#include "..\inc\TimerA0.h"
#include "..\inc\GPIO.h"
#include "Seeed_HC12main.h"


volatile uint32_t Time, MainCount;
volatile uint32_t Time2, PrintFlag;
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
// P3->OUT is 8-bit port at 0x4000.4C22
// I/O address be 0x4000.0000+n, and let b represent the bit 0 to 7.
// n=0x4C22, b=0
// bit banded address is 0x4200.0000 + 32*n + 4*b
#define SET (*((volatile uint8_t *)(0x42098440)))
char HC12data;
uint32_t Message;
int Flag; // semaphore

void HC12_ReadAllInput(void){uint8_t in;
// flush receiver buffer
  in = UART1_InCharNonBlock();
  while(in){
    UART0_OutChar(in);
    in = UART1_InCharNonBlock();
  }
}
void HC12_Init(uint32_t baud){
  P3->SEL0 &= ~0x01;
  P3->SEL1 &= ~0x01;    // configure P3.0 as GPIO
  P3->DIR |= 0x01;      // make P3.0 out
  UART1_InitB(baud);    // serial port to HC12
  HC12data = '1';
  //************ configure the HC12 module**********************
  SET = 0;       // enter AT command mode
  Clock_Delay1ms(40);
  UART1_OutString("AT+B9600\n");  // UART baud rate set to 9600
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+C007\n");   // channel 7 selected (001 to 100 valid)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+P8\n");    // highest power level (1 to 8)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+RF\n");    // read FU transmission mode (FU3)
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  UART1_OutString("AT+V\n");    // read firmware
  Clock_Delay1ms(50);
  HC12_ReadAllInput();
  SET = 1;  // exit AT command mode
  Clock_Delay1ms(200);
  HC12_ReadAllInput(); // remove any buffered input
  //************ configuration ended***********************
  printf("\nRF_XMT initialization done\n");
}

// ******************************************************************************
// ******************************************************************************
// ******************************************************************************
void SysTick_Handler_1(void){
  uint8_t in;

  LEDOUT ^= 0x01;       // toggle P1.0
  LEDOUT ^= 0x01;       // toggle P1.0

  Time = Time + 1;
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if(ThisInput){
    if((Time%100) == 0){ // 1 Hz
      P1->OUT ^= 0x80;      // toggle P1.7
      P1->OUT ^= 0x80;      // toggle P1.7
      HC12data = HC12data^0x01; // toggle '0' to '1'
      if(HC12data == 0x31){
        Message = 1; // S1
        Flag = 1;    // signal
       }else{
        Message = 0; // S0
        Flag = 1;    // signal
      }
      UART1_OutChar(HC12data);
      P1->OUT ^= 0x80;      // toggle P1.7
    }
  }
  in = UART1_InCharNonBlock();
  if(in){
    P1->OUT ^= 0x80;      // toggle P1.7
    P1->OUT ^= 0x80;      // toggle P1.7
    switch(in){
      case '0':
        Message = 2; // R0
        Flag = 1;    // signal
        LaunchPad_Output(0); // off
        break;
      case '1':
        Message = 3; // R1
        Flag = 1;    // signal
        LaunchPad_Output(BLUE);
        break;
    }
    P1->OUT ^= 0x80;      // toggle P1.7
  }

  LEDOUT ^= 0x01;       // toggle P1.0
}

/**
 * main.c
 */
void main_1(void){int num=0;
  Clock_Init48MHz();        // running on crystal
  Time = MainCount = 0;
  SysTick_Init(480000,2);   // set up SysTick for 100 Hz interrupts
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  GPIO_Init();              // P1.7 is GPIO output
  UART0_Initprintf();       // serial port to PC for debugging
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_OutClear();
  SSD1306_SetCursor(0,0);
  SSD1306_OutString("----- ECE382V ------\n");
  SSD1306_OutString(" Lab 1\n");
  SSD1306_OutString(" Seeed_HC12 example\n");
  SSD1306_OutString(" Valvano\n");
  EnableInterrupts();
  printf("\nSeeed_HC12 example -Valvano\n");
  HC12_Init(UART1_BAUD_9600);
  SSD1306_OutString(" RF_XMT init done\n");
  SSD1306_OutString("\nHold switch for 1s\n");
  while(1){ // USER Output in main (not ISR)
    WaitForInterrupt();
     // foreground thread
    MainCount++;
    if(Flag){ // wait on semaphore
      num++;
      if(num == 1){
        SSD1306_OutClear();
      }
      SSD1306_OutUDec16(num);
      SSD1306_OutChar(' ');
      Flag = 0; //
      switch(Message){
        case 0:
          printf("S0\n");
          SSD1306_OutString("S0\n");
          break;
        case 1:
          printf("S1\n");
          SSD1306_OutString("S1\n");
          break;
        case 2:
          printf("R0\n");
          SSD1306_OutString("R0\n");
          break;
        case 3:
          printf("R1\n");
          SSD1306_OutString("R1\n");
          break;
      }
    }
  }
}

// ******************************************************************************
// ******************************************************************************
// ******************************************************************************
uint8_t KnownSequence[] = {'A', 'B', 'C', 'D', 'E'};
uint8_t InputSequence[5];

//********SysTick_Handler*****************
// ISR for SysTick timer.
// Checks UART1 for communication through HC12 wireless module.
// Sends data when button is pressed.
// Inputs: none
// Outputs: none
void SysTick_Handler_5E(void) {
  uint8_t i, j;
  uint8_t in;

  LEDOUT ^= 0x01;       // toggle P1.0
  LEDOUT ^= 0x01;       // toggle P1.0
  Time = Time + 1;

  // Clear input message from previous run
  InputSequence[0] = '\0';
  InputSequence[1] = '\0';
  InputSequence[2] = '\0';
  InputSequence[3] = '\0';
  InputSequence[4] = '\0';

  // Send data on button press at a rate of 1 Hz
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if (ThisInput) {
    if ((Time % 100) == 0) {  // 1 Hz
      Message = 1;
      Flag = 1;    // signal
      for (i = 0; i < sizeof(KnownSequence); i++) {
        UART1_OutChar(KnownSequence[i]);
      }
    }
  }

  // Receive data and compare to known sequence
  in = UART1_InCharNonBlock();
  j = 0;
  while (in) {
    Message = 2;
    Flag = 1;
    InputSequence[j] = in;
    in = UART1_InCharNonBlock();
    j = (j + 1) % 5;
  }
  // Check for correct sequence received
  if (Flag && Message == 2) {
    if (memcmp(KnownSequence, InputSequence, 5) != 0)
      Message = 3;
      while (UART1_InCharNonBlock()) {}
  }

  LEDOUT ^= 0x01;       // toggle P1.0
}

//********Main_5E*****************
// Entry point of the application for section 5E
// Inputs: none
// Outputs: none
void main_5E(void) {
  int num = -1;
  DisableInterrupts();  // Prevent interrupts during initialization

  // Standard initialization sequence
  Clock_Init48MHz();        // running on crystal
  SysTick_Init(480000,2);   // set up SysTick for 100 Hz interrupts
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  UART0_Initprintf();       // serial port to PC for debugging
  SSD1306_Init(SSD1306_SWITCHCAPVCC);

  // Print initial message to display
  SSD1306_OutClear();
  SSD1306_SetCursor(0,0);
  SSD1306_OutString("----- ECE382V ------\n");
  SSD1306_OutString(" Lab 1\n");
  SSD1306_OutString(" Section 5E\n");
  EnableInterrupts();
  printf("\nSection 5E\n");

  // Initialize wireless module (may take some time)
  HC12_Init(UART1_BAUD_9600);
  SSD1306_OutString(" RF_XMT init done\n");
  SSD1306_OutString("\nHold switch for 1s\n");

  EnableInterrupts();

  // Application, loop forever
  while (1) {
    WaitForInterrupt();
    MainCount++;

    if (Flag) {   // wait on semaphore
      num++;
      if (num == 1) {
        SSD1306_OutClear(); // Clear display
      }

      Flag = 0;   // Data transmission has been handled
      SSD1306_OutUDec16(num);
      SSD1306_OutChar(' ');
      switch (Message) {
        case 1:   // Send sequence
          printf("S\n");
          SSD1306_OutString("S               \n");
          break;
        case 2:
          printf("R\n");
          SSD1306_OutString("R ");
          SSD1306_OutChar(InputSequence[0]);
          SSD1306_OutChar(InputSequence[1]);
          SSD1306_OutChar(InputSequence[2]);
          SSD1306_OutChar(InputSequence[3]);
          SSD1306_OutChar(InputSequence[4]);
          SSD1306_OutString("\n");
          break;
        case 3:
          printf("L\n");
          SSD1306_OutString("L ");
          SSD1306_OutChar(InputSequence[0]);
          SSD1306_OutChar(InputSequence[1]);
          SSD1306_OutChar(InputSequence[2]);
          SSD1306_OutChar(InputSequence[3]);
          SSD1306_OutChar(InputSequence[4]);
          SSD1306_OutString("\n");
          break;
      }
    }
  }
}


// ******************************************************************************
// ******************************************************************************
// ******************************************************************************
uint32_t TIMER_FREQ = 24000000;
uint32_t MICROSEC_UNITS = 2;
uint32_t INTERRUPT_RATE_TIMER = 9600;   // 9.6kHz
message_t OutPacket;
uint8_t Len = 0;

//********SysTick_Handler*****************
// ISR for SysTick timer.
// Sends data when button is pressed.
// Inputs: none
// Outputs: none
void SysTick_Handler_5F(void) {
  Time = Time + 1;

  // Send data on button press
  uint8_t ThisInput = LaunchPad_Input();   // either button
  if (ThisInput) {
    if ((Time % 100) == 0) {  // 1 Hz at a time
      // Only send one packet
      if (G_UNSENT_BYTES > 0) {   // Message already in transit
        return;
      }
      else {
        G_UNSENT_BYTES = sizeof(OutMessage) + 4;  // Length of data + 4 metadata
        memcpy(G_SEND_BUFF, &OutPacket, G_UNSENT_BYTES); 
        G_SEND_PTR = G_SEND_BUFF;
      }
    }
  }
}

//********Main_5F*****************
// Entry point of the application for section 5F
// Inputs: none
// Outputs: none
void main_5F(void) {
  uint8_t count = 0;
  uint8_t flag = 0;
  uint8_t in, error;
  uint8_t profile_flag = 1;

  DisableInterrupts();  // Prevent interrupts during initialization

  // Standard initialization sequence
  Clock_Init48MHz();        // running on crystal
  SysTick_Init(480000, 1);  // SysTick interrupts at 100Hz, priority 1
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  GPIO_Init();              // P1.7 is GPIO output
  UART0_Initprintf();       // serial port to PC for debugging
  SSD1306_Init(SSD1306_SWITCHCAPVCC);

  // Print initial message to display
  SSD1306_OutClear();
  SSD1306_SetCursor(0,0);
  SSD1306_OutString("----- ECE382V ------\n");
  SSD1306_OutString(" Lab 1\n");
  SSD1306_OutString(" Section 5F\n");
  EnableInterrupts();
  printf("\nSection 5F\n");

  // Initialize wireless module (may take some time)
  HC12_Init(UART1_BAUD_9600);
  SSD1306_OutString(" RF_XMT init done\n");
  SSD1306_OutString("\nHold switch for 1s\n");

  // Create output message
  OutPacket.header = HEADER;
  OutPacket.address = DEST_ID;
  OutPacket.length = sizeof(OutMessage);
  memcpy(OutPacket.data, OutMessage, OutPacket.length);
  OutPacket.error = OutPacket.length ^ OutPacket.data[0];

  // Dynamic ID allocation
  while (count < 5) {
    if (!flag) {
      UART1_OutChar(MY_ID);     // Broadcast my ID
      flag = 1;
    }
    
    // Check if someone else is awake, up to 5sec
    if (UART1_InStatus()) {
      in = UART1_InCharNonBlock();
      // Check to see if it's an "I'm awake msg"
      if (in == '1') {
        MY_ID++;  // increase my ID
        count = 0;
      }
      else {
        // Clear message
        while (UART1_InCharNonBlock()) {
              Clock_Delay1ms(10);
            }
      }
    }
    else {
      Clock_Delay1ms(1000);
      count++;
    }
  }

  // IDs sssigned, continue with application
  SSD1306_OutClear();
  SSD1306_SetCursor(0,0);
  SSD1306_OutChar(MY_ID);
  LaunchPad_Output(RED);
  Clock_Delay1ms(3000);
  LaunchPad_Output(0);
  EnableInterrupts();

  // Application, loop forever
  while (1) {
    // Check for outgoing data that needs to be sent out
    if (G_UNSENT_BYTES > 0) {
      if (profile_flag) {
        // Profile
        P1->OUT ^= 0x80;      // toggle P1.7
        P1->OUT ^= 0x80;      // toggle P1.7
        profile_flag = 0;
      }

      LaunchPad_Output(BLUE);
      UART1_OutChar(*G_SEND_PTR);

      UART0_OutUDec(*G_SEND_PTR);
      UART0_OutString("\n\r");

      G_SEND_PTR++;
      G_UNSENT_BYTES--;
      if (G_UNSENT_BYTES == 1) {
        P1->OUT ^= 0x80;      // toggle P1.7
        profile_flag = 1;
      }
    }

    // Check UART1 for incoming data
    if (UART1_InStatus()) {
      in = UART1_InChar();
      // Check recv buffer for incoming data that needs to be processed
      if (G_UNRECV_BYTES > 0) {
        LaunchPad_Output(GREEN);
        *G_RECV_PTR = in;
        
        UART0_OutUDec(*G_RECV_PTR);
        UART0_OutString("\n\r");


        G_UNRECV_BYTES--;

        // Once recv buffer reaches 0, check for error
        if (G_UNRECV_BYTES == 0) {
          // Profile
          P1->OUT ^= 0x80;      // toggle P1.7

          // Calculate error from msg
          error = Len ^ G_RECV_BUFF[0];
          // Compare to transmitted error
          if (error != in) {
            LaunchPad_Output(RED);
          }
          else {
            SSD1306_SetCursor(0,1);
            SSD1306_OutString(G_RECV_BUFF);
          }
        }

        G_RECV_PTR++;
      }
      else {
        // Parse incoming message and set recv buffer
        if (in == HEADER) {       // Check correct header
          in = UART1_InChar();
          if (in == MY_ID)  {     // Check message is for me
            // Profile
            P1->OUT ^= 0x80;      // toggle P1.7
            P1->OUT ^= 0x80;      // toggle P1.7

            Len = UART1_InChar();    // Retrieve length
            G_UNRECV_BYTES = Len + 1;   // Size of data + error byte
            G_RECV_PTR = G_RECV_BUFF;   // Pointer to start of buffer
          }
          else {          // Message not for me
            // Clear message
            while (UART1_InCharNonBlock()) {
              Clock_Delay1ms(10);
            }
            G_UNRECV_BYTES = 0;
          }
        }
        else {          // Incorrect header
            // Clear message
            while (UART1_InCharNonBlock()) {
              Clock_Delay1ms(10);
            }
            G_UNRECV_BYTES = 0;
        }
      }
    }

    // Turn LED off when not sending or receiving
    if (G_UNSENT_BYTES == 0 && G_UNRECV_BYTES == 0) {
      Clock_Delay1ms(100);
      LaunchPad_Output(0);
    }

    MainCount++;
  }
}


// ******************************************************************************
// ******************************************************************************
// ******************************************************************************
#define trampoline  2

//********SysTick_Handler*****************
// ISR for SysTick timer, runs every 10ms.
// Trampoline for selecting handler.
// Inputs: none
// Outputs: none
void SysTick_Handler(void) {
  switch (trampoline) {
    case 0:
      SysTick_Handler_1();
      break;
    case 1:
      SysTick_Handler_5E();
      break;
    case 2:
      SysTick_Handler_5F();
      break;
  }
}

//********Trampoline for selecting main*****************
void main() {
  switch (trampoline) {
    case 0:
      main_1();
      break;
    case 1:
      main_5E();
      break;
    case 2:
      main_5F();
      break;
  }
}
