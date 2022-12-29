//*****************************************************************************
// RSLK-MAX test main with CC2650 BoosterPack
// MSP432 with RSLK-MAX
// Starter code for Lab2 ECE382V Technology for IoT
// Daniel and Jonathan Valvano
// August 18, 2022
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2022
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
/* Hardware requirements
    1- CC2650 BoosterPack, BOOSTXL-CC2650MA (this lab 2)
    1- CC3100 BoosterPack, CC3100BOOST (for Lab 3)
    1- RSLK robot with MSP432, TIRSLK-EVM
    1- Some robots have OPT3101 time of flight sensor, some do not
    6- 1.2V AA NiMH batteries and charger
 */
// ******************************************************************************************
// NOTE: For MSP432+CC2560BP+RSLK, change 153 in GPIO.h to leave in #define DEFAULT 1       *
// ******************************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/LPF.h"
#include "../inc/TimerA1.h"
//#include "../inc/TA3InputCapture.h"  //already included in Tachometer.h
#include "../inc/blinker.h"
#include "../inc/TExaS.h"
#include "../inc/Reflectance.h"
#include "../inc/Tachometer.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/SysTickInts.h"
#include "../inc/SSD1306.h"
#include "../inc/I2CB1.h"
#include "../inc/opt3101.h"
// OPT3101=0 if no OPT3101, OPT3101=1 if robot has OPT3101
#define ROBOT 1

bool extra_features;

uint16_t Switch1;       // 16-bit notify data from Button 1
//processed from python client script
int32_t DriveLeft;
int32_t DriveRight;
// LED memory to be written to
uint16_t LED; 
uint32_t time_global=0;

uint8_t NPI_GATTSetDeviceNameJacki19[] = {
  SOF,15,0x00,    // length = 15 (3 + length of phrase)
//  SOF,18,0x00,    // length = 18
  0x35,0x8C,      // SNP Set GATT Parameter (0x8C)
  0x01,           // Generic Access Service
  0x00,0x00,      // Device Name
//  'J','a','c','k','i',' ','A','S','E','E',' ','0',
  'J','a','c','k','i',' ','R','S','L','K',' ','0',
//    'R','S','L','K',' ','E','x','p','l','o','r','e','r',' ','0',
  0x77};          // FCS (calculated by AP_SendMessageResponse)
uint8_t NPI_SetAdvertisementDataJacki19[] = {
  SOF,24,0x00,    // length = 24
//  SOF,27,0x00,    // length = 27
  0x55,0x43,      // SNP Set Advertisement Data
  0x00,           // Scan Response Data
  13,0x09,        // length, type=LOCAL_NAME_COMPLETE
//  'J','a','c','k','i',' ','A','S','E','E',' ','0',
  'J','a','c','k','i',' ','R','S','L','K',' ','0',
//    'R','S','L','K',' ','E','x','p','l','o','r','e','r',' ','0',

// connection interval range
  0x05,           // length of this data
  0x12,           // GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE
  0x50,0x00,      // DEFAULT_DESIRED_MIN_CONN_INTERVAL
  0x20,0x03,      // DEFAULT_DESIRED_MAX_CONN_INTERVAL
// Tx power level
  0x02,           // length of this data
  0x0A,           // GAP_ADTYPE_POWER_LEVEL
  0x00,           // 0dBm
  0x77};          // FCS (calculated by AP_SendMessageResponse)
#define RECVSIZE 128
extern uint8_t RecvBuf[RECVSIZE];
extern const uint8_t NPI_SetAdvertisement1[];
extern const uint8_t NPI_StartAdvertisement[];


//************ RSLK variables ********************
uint16_t LeftSpeed;            //  rotations per minute
uint16_t RightSpeed;           // rotations per minute
uint16_t LeftTach;             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;    // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;             // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach;            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;   // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;            // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

volatile uint32_t Time;    // ms
volatile uint8_t Data;     // QTR-8RC
volatile int32_t Position; // -332 to +332 mm from line

//************ RSLK functions ********************
void SysTick_Handler(void){ // every 1ms
  Time = Time + 1;
  if(Time%10==1){
    Reflectance_Start(); // start every 10ms
  }
  if(Time%10==2){
    Data = Reflectance_End(); // finish 1ms later
    Position = Reflectance_Position(Data);
  }
}
void PrintBump(void){int i;
  printf("Bump switches: ");
  uint32_t in = Bump_Read();
  uint32_t mask = 0x01;
  for(i=0;i<6;i++){
    if(mask&in) printf("%d ",i);
    mask = mask<<1;
  }
  printf("pressed\n\r");
}

//************ BT functions ********************

// printf (to PC) used for debugging

//*************AP_StartAdvertisementJacki**************
// Start advertisement for Jacki
// Input:  num is robot number 0 to 99
// Output: APOK if successful,
//         APFAIL if notification not configured, or if SNP failure
int AP_StartAdvertisementJacki19(uint8_t num){volatile int r;
  if(num<10){
    NPI_GATTSetDeviceNameJacki19[18] = ' ';
    NPI_GATTSetDeviceNameJacki19[19] = '0'+num%10;
    NPI_SetAdvertisementDataJacki19[18] = ' ';
    NPI_SetAdvertisementDataJacki19[19] = '0'+num%10;
//    NPI_GATTSetDeviceNameJacki19[21] = ' ';
//    NPI_GATTSetDeviceNameJacki19[22] = '0'+num%10;
//    NPI_SetAdvertisementDataJacki19[21] = ' ';
//    NPI_SetAdvertisementDataJacki19[22] = '0'+num%10;
  }else{
    num = num%100; // 0 to 99
    NPI_GATTSetDeviceNameJacki19[18] = '0'+num/10;
    NPI_GATTSetDeviceNameJacki19[19] = '0'+num%10;
    NPI_SetAdvertisementDataJacki19[18] = '0'+num/10;
    NPI_SetAdvertisementDataJacki19[19] = '0'+num%10;
//    NPI_GATTSetDeviceNameJacki19[21] = '0'+num/10;
//    NPI_GATTSetDeviceNameJacki19[22] = '0'+num%10;
//    NPI_SetAdvertisementDataJacki19[21] = '0'+num/10;
//    NPI_SetAdvertisementDataJacki19[22] = '0'+num%10;
  }
  r =AP_SendMessageResponse((uint8_t*)NPI_GATTSetDeviceNameJacki19,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisement1,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_SetAdvertisementDataJacki19,RecvBuf,RECVSIZE);
  r =AP_SendMessageResponse((uint8_t*)NPI_StartAdvertisement,RecvBuf,RECVSIZE);
  return r;
}

// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}

void Bluetooth_DriveXCmd(void){
   UART0_OutString("\n\r");
   UART0_OutString("\n\rDriveX Cmd is ");
//   UART0_OutSDec(DriveXCmd);
   UART0_OutString("\n\r");
  // OutValue("\n\rDriveYCmd Data=",DriveYCmd);
}

void Bluetooth_DriveYCmd(void){
   UART0_OutString("\n\r");
   UART0_OutString("\n\rDriveY Cmd is ");
//   UART0_OutSDec(DriveYCmd);
   UART0_OutString("\n\r");
  // OutValue("\n\rDriveYCmd Data=",DriveYCmd);
}

void Bluetooth_DriveCmd(void){
   UART0_OutString("\n\r");
   UART0_OutString("\n\rDriveLeft Cmd is ");
   UART0_OutSDec(DriveLeft);
   UART0_OutString(", ");
   UART0_OutString("\n\rDriveRight Cmd is ");
   UART0_OutSDec(DriveRight);
   UART0_OutString("\n\r");
  // OutValue("\n\rDriveYCmd Data=",DriveYCmd);
}

void Bluetooth_LED(void){
  LaunchPad_Output(LED);  // set LEDs with bottom bits
  OutValue("\n\rLED Data=",LED);
}

void Bluetooth_Switch1(void){ // called on SNP CCCD Updated Indication
   OutValue("\n\rSwitch 1 CCCD=",AP_GetNotifyCCCD(0));
 }

void BLE_Init(uint8_t num){volatile int r;
  UART0_Init();
  EnableInterrupts();
  UART0_OutString("\n\rJacki test project - MSP432-CC2650\n\r");
  r = AP_Init();
  AP_GetStatus();  // optional
  AP_GetVersion(); // optional
  AP_AddService(0xFFF0);  //TODO I guess it's ok to have both devices under the same service?

  //Characteristics (Values written by Python Client)
  if(extra_features) AP_AddCharacteristic(0xFFF6,2,&LED,0x02,0x08,"LED",0,&Bluetooth_LED);
  AP_AddCharacteristic(0xFFFE,4,&DriveLeft,0x02,0x08,"DriveLeft",0,&Bluetooth_DriveCmd);
  AP_AddCharacteristic(0xFFFF,4,&DriveRight,0x02,0x08,"DriveRight",0,&Bluetooth_DriveCmd);

  //Notify Characteristics (Publishing to Python Client)
  if(extra_features) AP_AddNotifyCharacteristic(0xFFFD,2,&Switch1,"Button1",&Bluetooth_Switch1);
  AP_RegisterService();
  AP_StartAdvertisementJacki19(num);
  AP_GetStatus(); // optional
}


// #define OPT3101=0 if no OPT3101, OPT3101=1 if robot has OPT3101
int main(void){
uint32_t time2 = 0;    // incremented with every pass through main loop
int i=0;
uint8_t last;
DisableInterrupts();
Clock_Init48MHz();
UART0_Initprintf();   // serial port channel to PC, with printf
LaunchPad_Init();   // built-in switches and LEDs
//RSLK Init Functions (Begin)
Bump_Init();
Tachometer_Init();
Blinker_Init();       // Blinker LED
Motor_Init();
Reflectance_Init();    // line sensor
SysTick_Init(48000,3); // set up SysTick for 1kHz interrupts
//RSLK Init Functions (End)
BLE_Init(ROBOT);
EnableInterrupts();
last = LaunchPad_Input();
while(Bump_Read() == 0){
  time_global++;
  AP_BackgroundProcess();  // handle incoming SNP frames
  time2 = time2 + 1;

  //******RSLK Startup Functions (Begin) **************
  Motor_Stop();
  //******RSLK Startup Functions (END)**************
  //******RSLK Operation Functions (BEGIN)**************
  LaunchPad_Output(GREEN);           // green LED is in bit 1
  Motor_Forward(DriveLeft, DriveRight);
  i++;
  if((i%25)==0){ // slow down output
    Tachometer_Get(&LeftTach, &LeftDir, &LeftSteps, &RightTach, &RightDir, &RightSteps);
  // (1/tach step/cycles) * (12,000,000 cycles/sec) * (60 sec/min) * (1/360 rotation/step)
    LeftSpeed = 2000000/LeftTach;
    RightSpeed = 2000000/RightTach;
    // printf("Tachometer speed Left=%d, Right=%d rps\n\r",LeftSpeed,RightSpeed);
  }
  Clock_Delay1ms(62);              // delay ~0.062 sec at 48 MHz
  //******RSLK Operation Functions (END)**************
  
  //******BT Operation Functions (BEGIN)**************
  // the constant 177770 determines the period between notifications
  if(extra_features) {
    if(time2 >= 177770){         // calibration value is basically a guess to get about 1 Hz
      time2 = 0;
      if(AP_GetNotifyCCCD(0)){
        Switch1 = LaunchPad_Input()&0x01;   // Button 1 the & masks everything but the 1st bit, shouldn't it be 0x02 then?
      //      OutValue("\n\rNotify Bumpers=",JackiBumpSensor);
        AP_SendNotification(0);
      }
      // take IR distance measurements
      // LaunchPad_Output((i&0x01)<<2);     // toggle the blue LED
      // print distance average
      }
  }
    //******BT Operation Functions (END)**************
  }
}








