// MK_Fitness_BLEmain.c
// Runs on either MSP432 or TM4C123
// Starter code for Lab2 ECE382V Technology for IoT
// This version operates with the RTOS built in ECE445M Lab 3
// Data streamed via Bluetooth to phone
// Take sensor readings, process the data,
// and output the results.  Specifically, this program will
// measure steps using the accelerometer, audio sound amplitude using
// the microphone, temperature using the TMP006, and light using the
// OPT3001.
// Daniel and Jonathan Valvano
// see GPIO.c file for hardware connections 
// August 14, 2022

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2022

   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2022

   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2022

   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2022

 Copyright 2022 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
/* Hardware
 * One MSP-EXP432P401R LaunchPad
 * One LAUNCHXL-CC2650 LaunchPad
 * One BOOSTXL-EDUMKII BoosterPack
 */

// ******************************************************************************************
// NOTE: For MSP432+CC2560LP+MKII, change 153 in GPIO.h to comment out #define DEFAULT 1    *
// ******************************************************************************************

#include <stdint.h>
#include "../inc/CortexM.h"
#include "../inc/UART0.h"
#include "../inc/Profile.h"
#include "os.h"
#include "Texas.h"
#include "../inc/AP.h"
#include "AP_Lab6.h"
#include "../inc/LaunchPad.h"
//**************WARNING************************************
//***Edit GPIO.h line 153 to match your BLE option*********
//*********************************************************
uint32_t sqrt32(uint32_t s);
#define THREADFREQ 1000   // frequency in Hz of round robin scheduler

//---------------- Global variables shared between tasks ----------------
uint32_t Time;              // elasped time in 100 ms units
uint32_t Steps;             // number of steps counted

// New variables
uint16_t Switch1;       // 16-bit notify data from Button 1
uint16_t LED;           // 16-bit write-only data to LED

// semaphores
int32_t NewData;  // true when new numbers to display on top of LCD
int32_t LCDmutex; // exclusive access to LCD
int32_t I2Cmutex; // exclusive access to I2C
int SendFlag=0;
int Send1Flag_Joy=0;


// *********Task1*********
// collects data from accelerometer and switch
// Inputs:  none
// Outputs: none
void Task1(void){uint32_t squared;
  TExaS_Task1();     // records system time in array, toggles virtual logic analyzer
  Profile_Toggle1(); // viewed by a real logic analyzer to know Task1 started

  Switch1 = LaunchPad_Input()&0x01;   // Button 1 the & masks everything but the 1st bit, shouldn't it be 0x02 then?
  Time++; // in 100ms units
}

/* ****************************************** */
/*          End of Task1 Section              */
/* ****************************************** */





//---------------- Task7 dummy function ----------------
// *********Task7*********
// Main thread scheduled by OS round robin preemptive scheduler
// Task7 checks for Bluetooth incoming frames but never blocks or sleeps
// Inputs:  none
// Outputs: none
uint32_t Count7;
void Task7(void){
  Count7 = 0;
  while(1){
    Count7++;
    AP_BackgroundProcess();
    if(SendFlag){
      //should I plot value of LED?
      if(AP_GetNotifyCCCD(0)) {
        // OutValue("\n\rLight data=",LightData);
        AP_SendNotification(0);
      }
    SendFlag=0;
    }
    
    //Could this line be an issue?
    OS_Sleep(100);
    // WaitForInterrupt();
  }
}
/* ****************************************** */
/*          End of Task7 Section              */
/* ****************************************** */

void Task2(void){
}

void Task3(void){
}

void Task4(void){
}

void Task5(void){
}

void Task6(void){
}


void Task8(void){
    SendFlag = 1;
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

//Write Characteristics (to be written to memory on this device)

//Changes LED based on global variable that is shared through Bluetooth
void Bluetooth_LED(void){
  LaunchPad_Output(LED);  // set LEDs with bottom bits
  OutValue("\n\rLED Data=",LED);
}

// Notify Characteristics

 void Bluetooth_Switch1(void){ // called on SNP CCCD Updated Indication
   OutValue("\n\rSwitch 1 CCCD=",AP_GetNotifyCCCD(0));
 }

extern uint16_t edXNum; // actual variable within TExaS
void Bluetooth_Init(void){volatile int r;
  EnableInterrupts();
  UART0_OutString("\n\rLab 6 Application Processor\n\r");
  r = AP_Init(); 
  Lab6_GetStatus();  // optional
  Lab6_GetVersion(); // optional
  //this is a server, servers have services. python script is a client. they're connected. Can't connect two servers together
  Lab6_AddService(0xFFF0); 
  //Characteristics (Values read by Client)

  // Notify Characteristics (Values subscribed to by Client based on publish rate of each task)
  Lab6_AddNotifyCharacteristic(0xFFFD,2,&Switch1,"Button1",&Bluetooth_Switch1);


  Lab6_RegisterService();
  Lab6_StartAdvertisement("Simple Device");
  Lab6_GetStatus();
  DisableInterrupts(); // optional
}
//---------------- Step 6 ----------------
// Step 6 is to implement the fitness device by combining the
// OS functions that were implemented and tested in the earlier
// steps with the user tasks in this file.  Completing this
// step will give you your grade, so remember to change the
// second parameter in TExaS_Init() to your 4-digit number.
// Task   Purpose        When to Run
// Task0  microphone     periodically exactly every 1 ms
// Task1  accelerometer  periodically exactly every 100 ms
// Task2  plot on LCD    after Task1 finishes
// Task3  switch/buzzer  periodically every 10 ms
// Task4  temperature    periodically every 1 sec
// Task5  numbers on LCD after Task0 runs SOUNDRMSLENGTH times (every 1 sec effectively)
// Task6  light          periodically every 800 ms, comes from line OS_Sleep(800);
// Task7  dummy          no timing requirement
// Remember that you must have exactly one main() function, so
// to work on this step, you must rename all other main()
// functions in this file.
int main(void){
  OS_Init();
  LaunchPad_Init();         // P1.0 is red LED on LaunchPad
  LaunchPad_Output(0);
  Profile_Init();  // initialize the 7 hardware profiling pins

  Time = 0;
  OS_InitSemaphore(&NewData, 0);  // 0 means no data
  OS_InitSemaphore(&LCDmutex, 1); // 1 means free
  OS_InitSemaphore(&I2Cmutex, 1); // 1 means free
  OS_FIFO_Init();                 // initialize FIFO used to send data between Task1 and Task2
  // Task 1 should run every 100ms
  OS_AddPeriodicEventThread(&Task1, 100);
  // // Task 8 should run every second
  OS_AddPeriodicEventThread(&Task8, 1000);
  // Task2, Task3, Task4, Task5, Task6, Task7 are main threads
  OS_AddThreads(&Task2, &Task3, &Task4, &Task5, &Task6, &Task7);
  UART0_Init();
  Bluetooth_Init();
  UART0_OutString("\n\rBluetooth Initialized\n\r");
  //This is what controls the execution of the threads, they're not periodic. They are round-robin (one after the other)
  uint32_t ClockFrequency = 3000000;
  OS_Launch(ClockFrequency/THREADFREQ); // doesn't return, interrupts enabled in here
  UART0_OutString("\n\rEnd of Program\n\r");
  return 0;             // this never executes
}



