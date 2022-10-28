
// SIM7600main.c
// Runs on either MSP432
// Starter code for Lab5 ECE382V Technology for IoT
// This version operates with the RTOS built in ECE445M Lab 3
// Data streamed via SIM7600 GSM SMS module to phone
// The phone number for this Embedded IoT object is 737-529-6744

// Pranav Rama, Daniel Valvano, and Jonathan Valvano
// Sept 14, 2022

/*
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
 * One COM.LTE 4G MODULE SIM7600G, Digikey 2221-M031-A-ND
 * One BP-BASSENSORSMKII BoosterPack
 */
// built-in LED1 connected to P1.0
// P1.0, P2.0 are an output to profiling scope/logic analyzer
// Current sense amplifier connect to ADC A13, P4.0

// SIM7600G pin connection
// TX      , MSP432 UCA2RXD P3.2
// RX      , MSP432 UCA2TXD P3.3
// 5V      , LaunchPad 5V
// GND     , LaunchPad GND
//***********BP-BASSENSORSMKII****************
// P4.1 INT1      BMI160 IMU
// P5.0 INT2      BMI160 IMU
// P4.3 HDC_V+    HDC2080
// P6.1 HDC_INT   HDC2080
// P4.5 OPT_V+    OPT3001 VDD   pin 1 <- P4.5 power to sensor
// P6.4 SDA       OPT3001 SDA   pin 6 <> P6.4 I2C data SDA
// P6.5 SCL       OPT3001 SCL   pin 4 <- P6.5 I2C clock SCL
// P4.2 OPT_INT   OPT3001 INT   pin 5 -> P4.2 interrupt
// P4.7 TMP_V+    TMP117 V+    pin 5 <- P4.7 power to sensor
// P6.4 SDA       TMP117 SDA   pin 6 <> P6.4 I2C data SDA
// P6.5 SCL       TMP117 SCL   pin 1 <- P6.5 I2C clock SCL
// P4.4 TMP_ALERT TMP117 ALERT pin 3 -> P4.4 alert
/*
<table>
<caption id="BMI160">BMI160 on BP-BASSENSORSMKII connected to MSP432</caption>
<tr><th>Signal <th> BMI160       <th> MSP432
<tr><td>SDO    <td>BMI160 pin 1  <td> 3.3V (I2C address = 0x69)
<tr><td>ASDx   <td>BMI160 pin 2  <td> P6.4 I2C data SDA_S
<tr><td>ASCx   <td>BMI160 pin 3  <td> P6.5 I2C clock SCL_S
<tr><td>INT1   <td>BMI160 pin 4  <td> P4.1 input from the interrupt pin
<tr><td>VDDIO  <td>BMI160 pin 5  <td> 3.3V
<tr><td>GNDIO  <td>BMI160 pin 6  <td> ground
<tr><td>GND    <td>BMI160 pin 7  <td> ground
<tr><td>VDD    <td>BMI160 pin 8  <td> 3.3V
<tr><td>INT2   <td>BMI160 pin 9  <td> P5.0
<tr><td>OCSB   <td>BMI160 pin 10 <td> not connected (unused SPI for slave)
<tr><td>OSDO   <td>BMI160 pin 11 <td> not connected (unused SPI for slave)
<tr><td>CSB    <td>BMI160 pin 12 <td> 3.3V (protocol select = I2C)
<tr><td>SCX    <td>BMI160 pin 13 <td> I2C clock to BMM150 Three-Axis Geomagnetic Sensor slave (I2C address = 0x13)
<tr><td>SDX    <td>BMI160 pin 14 <td> I2C data to BMM150 Three-Axis Geomagnetic Sensor slave (I2C address = 0x13)
</table>
<table>
<caption id="HDC2080">HDC2080 on BP-BASSENSORSMKII connected to MSP432</caption>
<tr><th>Signal  <th> HDC2080             <th> MSP432
<tr><td>GND     <td> ground        pin 2 <td> ground
<tr><td>HDC_V+  <td> Q2 powers VDD pin 5 <td> P4.3, output 0 to power the sensor
<tr><td>SDA     <td> HDC2080 SDA   pin 1 <td> P6.4 I2C data SDA
<tr><td>SCL     <td> HDC2080 SCL   pin 6 <td> P6.5 I2C clock SCL
<tr><td>HDC_INT <td> HDC2080 DRDY/INT pin 4 <td> P6.1 alert
</table>
<table>
<caption id="OPT3001">OPT3001 on BP-BASSENSORSMKII connected to MSP432</caption>
<tr><th>Signal    <th> OPT3001             <th>MSP432
<tr><td>GND       <td> ground              <td> ground
<tr><td>TMP_V+    <td> OPT3001 V+    pin 5 <td> P4.7 power to sensor
<tr><td>SDA       <td> OPT3001 SDA   pin 6 <td> P6.4 I2C data SDA
<tr><td>SCL       <td> OPT3001 SCL   pin 1 <td> P6.5 I2C clock SCL
<tr><td>TMP_ALERT <td> OPT3001 ALERT pin 3 <td> P4.4 open drain alert
</table>
<table>
<caption id="TMP117">TMP117 on BP-BASSENSORSMKII connected to MSP432</caption>
<tr><th>Signal    <th> TMP117 <th>MSP432
<tr><td>GND       <td> ground             <td> ground
<tr><td>TMP_V+    <td> TMP117 V+    pin 5 <td> P4.7 power to sensor
<tr><td>SDA       <td> TMP117 SDA   pin 6 <td> P6.4 I2C data SDA
<tr><td>SCL       <td> TMP117 SCL   pin 1 <td> P6.5 I2C clock SCL
<tr><td>TMP_ALERT <td> TMP117 ALERT pin 3 <td> P4.4 open drain alert
</table>
*/
// Connect logic analyzer to these bits
// Task 0 Port 1.0 Output
// Task 1 Port 2.0 Output
// Task 2 Port 5.4 Output
// Task 3 Port 5.5 Output
// Task 4 Port 3.0 Output
// Task 5 Port 6.6 Output
// Task 6 Port 2.3 Output
#include <stdint.h>
#include <string.h> 
#include "../inc/CortexM.h"
#include "../inc/Clock.h"
#include "../inc/UART0.h"
#include "../inc/Profile.h"
#include "../inc/SIM7600G.h"
#include "../inc/LaunchPad.h"
#include "../inc/ADC14.h"
#include "../inc/I2CB1.h"
#include "../inc/HDC2080.h"
#include "../inc/OPT3001.h"
#include "../inc/TMP117.h"
#include "../inc/bmi160.h"
#include "../inc/bmm150.h"
#include "os.h"
#include "msp.h"

// #define VALVANO (uint8_t *)"+15129682240"
// #define PRANAV (uint8_t *)"+17138085920"
#define USER (uint8_t *)"+18326640684"

#define COMMAND_MESSAGE "Available Commads:\n\r \
    C: temperature\n\r \
    C: humidity\n\r \
    C: light\n\r \
    C: imu\n\r \
    C: battery\n\r"


// Parse buffer to decipher application commands
// Available commands defined by COMMAND_MESSAGE
int Message_Parser(char *buffer) {
  char t[150] = "";
  char response[300] = "";

  if (strstr(buffer, "C:temperature") != NULL) {
    sprintf(t, "Temperature: %d\n\r", 111);
    strcat(response, t);
  }
  if (strstr(buffer, "C:humidity") != NULL) {
    sprintf(t, "Humidity: %d\n\r", 222);
    strcat(response, t);
  }
  if (strstr(buffer, "C:light") != NULL) {
    sprintf(t, "Light: %d\n\r", 333);
    strcat(response, t);
  }
  if (strstr(buffer, "C:imu") != NULL) {
    sprintf(t, "IMU: (%d,%d,%d) (%d,%d,%d) (%d,%d,%d)\n\r", -11, -22, -33, 11, 22, 33, -11, -22, -33);
    strcat(response, t);
  }
  if (strstr(buffer, "C:battery") != NULL) {
    sprintf(t, "Battery: %d\n\r", 444);
    strcat(response, t);
  }

  UART0_OutString(response);

  return 0;
}


// test main for phone
int testmain(){int32_t n;
  Clock_Init48MHz();
  UART0_Initprintf();
  LaunchPad_Init();
  UART0_OutString("Embedded IoT SIM7600 project\n\r");

  // Initializa SIM7600G module (restart required)
  SIM7600G_Init(SMS_SIM7600G); // more space on flash in SIM7600G
  // UART0_OutString("Power down\n\r");
  // SIM7600G_PowerDown();
  // Clock_Delay1ms(40000); // give it time to shut down
  // UART0_OutString("Restarting\n\r");
  // SIM7600G_Restart(SMS_SIM7600G); // more space on flash in SIM7600G
  UART0_OutString("***Initialization Complete***\n\r");

  // Clear text messages in buffer
  UART0_OutString("***Deleting texts***\n\r");
  n = SIM7600G_GetNumSMS();
  for(int i=0 ; i<n; i++){
    SIM7600G_DeleteSMS(i);
  }
  UART0_OutString("***Deleted***\n\r");

  while(LaunchPad_Input()==0){};  // wait for user switch press
  
  // Send a text message
  // UART0_OutString("***Sending text***\n\r");
  // // SIM7600G_SendSMS(PRANAV, "Hello TA SIM7600 from Embedded IoT!");
  // // SIM7600G_SendSMS(VALVANO, "Hello professor from Embedded IoT!");
  // SIM7600G_SendSMS(USER, "Hello student from Embedded IoT!");
  // UART0_OutString("***Sent***\n\r");
  
  while(LaunchPad_Input()==0){};  // wait for user input

  // Read text and parse text messages
  char sms_read_buffer[255];
  uint32_t sms_readlen;
  char sender[100];
  UART0_OutString(COMMAND_MESSAGE);
  while(1) {
    UART0_OutString("***Reading texts***\n\r");
    n = SIM7600G_GetNumSMS();
    for (int i = 0; i < n; i++) {
      UART0_OutString("Message ");
      UART0_OutChar(i + '1');

      // Read message
      SIM7600G_ReadSMS(i, sms_read_buffer, 255, &sms_readlen);
      SIM7600G_GetSMSSender(i, sender, 100);
      
      // Parse message
      Message_Parser(sms_read_buffer);
      
      // Delete message
      SIM7600G_DeleteSMS(i);
    }
    UART0_OutString(COMMAND_MESSAGE);
    while(LaunchPad_Input()==0){};  // wait for user input
  }

  while(1){};
}



// *************RTOS version***************
int32_t I2Cmutex;         // exclusive access to I2C
OPT3001_Result Light;     // lux = 0.01*(2^Exponent)*Result
uint32_t TimeMs;          // elapsed time in ms
uint32_t Humidity;        // units 0.1% (ranges from 0 to 1000, 123 means 12.3%
int32_t Temperature;      // units 0.1 C
int16_t RawTemperature;   // 0.0078125 C
int32_t FixedTemperature; // 0.1C
int32_t Ax,Ay,Az; // acceleration
int32_t Rx,Ry,Rz; // gyro pitch, roll, yaw
int32_t Mx,My,Mz; // magnetic field strength
int32_t Battery;  // battery level
// Select whether or not to check for errors
#define ERRORCHECK 1


// Parse buffer to decipher application commands
// Available commands defined by COMMAND_MESSAGE
char* Message_Parser_RTOS(char *buffer) {
  char t[150] = "";
  static char response[255];
  sprintf(response, "");

  // Generate response message
  if (strstr(buffer, "C:temperature") != NULL) {
    sprintf(t, "Temperature: %d\n\r", Temperature);
    strcat(response, t);
  }
  if (strstr(buffer, "C:humidity") != NULL) {
    sprintf(t, "Humidity: %d\n\r", Humidity);
    strcat(response, t);
  }
  if (strstr(buffer, "C:light") != NULL) {
    sprintf(t, "Light: %d\n\r", Light);
    strcat(response, t);
  }
  if (strstr(buffer, "C:imu") != NULL) {
    sprintf(t, "IMU: (%d,%d,%d) (%d,%d,%d) (%d,%d,%d)\n\r", Ax, Ay, Az, Rx, Ry, Rz, Mx, My, Mz);
    strcat(response, t);
  }
  if (strstr(buffer, "C:battery") != NULL) {
    sprintf(t, "Battery: %d\n\r", Battery);
    strcat(response, t);
  }

  // No command recognized, generate help message
  if (response == "") {
    sprintf(response, "%s", COMMAND_MESSAGE);
  }

  return response;
}

// Transmit text message with current measurements
int Transmit_Measurements(void) {
  static char response[255] = "";

  // Update response with current measurement values
  sprintf(response, "Temperature: %d\n\r \
                     Humidity: %d\n\r \
                     Light: %d\n\r \
                     IMU: (%d,%d,%d) (%d,%d,%d) (%d,%d,%d)\n\r \
                     Battery: %d\n\r",
                     Temperature, Humidity, Light, Ax, Ay, Az, Rx, Ry, Rz, Mx, My, Mz, Battery);

  // Send text message
  SIM7600G_SendSMS(USER, response);
}


//---------------- Task0 real time periodic event ----------------
// Event thread run by OS in real time at 1000 Hz
// *********Task0_Init*********
// initializes TimeMs
// Task0 maintains time
// Inputs:  none
// Outputs: none
uint32_t SupplyCurrent; // mA measured with LTC6102
uint32_t E;             // integral of SupplyCurrent
uint32_t deltaT;        // 
// Rsense = 0.05ohm
// Rin = 100 ohm
// Rout=4999 ohm (gain=50)
void Task0_Init(void){
  TimeMs = 0;
  ADC0_InitSWTriggerCh13();
}
// *********Task0*********
// Periodic event thread runs in real time at 1000 Hz
// Task0 maintains time, measures +5V supply current
// Periodic events cannot block, spin, suspend or sleep
// Periodic events can call OS_Signal but not OS_Wait
// Inputs:  none
// Outputs: none
// Warning: Execution time must be much less than 1ms
// P1->OUT is 0x4000.4C02
#define P1_0  (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*0)))  /* Port 1.0 Output */
#define Gain 1320 // 1320/16384 = 0.080582523mA/ADC value
void Task0(void){uint32_t data;
  P1_0 = P1_0^0x01; // viewed by a real logic analyzer to know Task0 started
  TimeMs = TimeMs + 1;
  data = ADC_In13();
  SupplyCurrent = (Gain*data)/16384;
  E += SupplyCurrent;
}
/* ****************************************** */
/*          End of Task0 Section              */
/* ****************************************** */

//---------------- Task1 handles text messages ----------------
// Main thread scheduled by OS round robin preemptive scheduler
//------------UART0_InCharBlocking------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char UART0_InCharBlocking(void){
  while((EUSCI_A0->IFG&0x01) == 0){
    OS_Suspend();
  }
  return((char)(EUSCI_A0->RXBUF));
}
void UART0_InStringBlocking(char *bufPt, uint16_t max) {
int length=0;
char character;
  character = UART0_InCharBlocking();
  while(character != CR){
    if(character == BS){
      if(length){
        bufPt--;
        length--;
        UART0_OutChar(BS);
      }
    }
    else if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      UART0_OutChar(character);
    }
    character = UART0_InChar();
  }
  *bufPt = 0;
}
void UART0_OutCharBlocking(char letter){
  while((EUSCI_A0->IFG&0x02) == 0){
    OS_Suspend();
  }
  EUSCI_A0->TXBUF = letter;
}
void UART0_OutStringBlocking(char *pt){
  while(*pt){
    UART0_OutCharBlocking(*pt);
    pt++;
  }
}
// P2->OUT is 0x4000.4C03
#define P2_0  (*((volatile uint8_t *)(0x42000000+32*0x4C03+4*0)))  /* Port 2.0 Output */
void Task1(void){
  // char string[64];
  int32_t n;
  char sms_read_buffer[255];
  uint32_t sms_readlen;
  char sender[100];
  char *response;

  UART0_OutStringBlocking("\n\rRTOS version for SMS7600 4G\n\r");
  UART0_OutStringBlocking("Valvano Sept 12, 2022\n\r");
  SIM7600G_Init(SMS_SIM7600G); // more space on flash in SIM7600G
  UART0_OutStringBlocking("***Initialization Complete***\n\r");
  // UART0_OutStringBlocking("\n\rType message to send text\n\r");

  while(1){
    P2_0 = P2_0^0x01; // viewed by a real logic analyzer to know Task1 started
    OS_Sleep(10000);
    LaunchPad_LED(1);

    // Restart module and transmit measurements (Automatic Mode)
    E = 0;
    SIM7600G_Restart(SMS_SIM7600G);
    Transmit_Measurements();

    // Read and parse incoming messages (Interactive Mode)
    n = SIM7600G_GetNumSMS();
    for (int i = 0; i < n; i++) {
      // Read message
      SIM7600G_ReadSMS(i, sms_read_buffer, 255, &sms_readlen);
      SIM7600G_GetSMSSender(i, sender, 100);

      // Parse and send response message
      response = Message_Parser_RTOS(sms_read_buffer);
      SIM7600G_SendSMS(USER, response);

      // Delete message
      SIM7600G_DeleteSMS(i);
    }

    // Power down to conserve energy
    SIM7600G_PowerDown();

    // UART0_OutStringBlocking("\n\rText: ");
    // UART0_InStringBlocking(string,63); // user enters a string

    
    // SIM7600G_SendSMS(VALVANO,(uint8_t *)string);
    // SIM7600G_SendSMS(USER,(uint8_t *)string);

    LaunchPad_LED(0);
  }
}
/* ****************************************** */
/*          End of Task1 Section              */
/* ****************************************** */


//---------------- Task2 measures temperature and humidity from HC2080
// Main thread scheduled by OS round robin preemptive scheduler
void Task2(void){
  OS_Wait(&I2Cmutex);
  HDC2080_Init();
  OS_Signal(&I2Cmutex);
  while(1){
    Profile_Toggle2(); // viewed by a real logic analyzer to know Task2 started
    OS_Wait(&I2Cmutex);
    Humidity = HDC2080_ReadHumidity();
    Temperature = HDC2080_ReadTemperature();
    OS_Signal(&I2Cmutex);
    OS_Sleep(2000);     // every 2 seconds
  }
}
/* ****************************************** */
/*          End of Task2 Section              */
/* ****************************************** */


//------------Task3 handles accelerator-------
/* 1 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE   250

/* Variable declarations */
struct bmi160_dev bmi;
struct bmm150_dev bmm;
uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data;
struct bmm150_mag_data mag_data;
struct bmi160_sensor_data gyro_data, accel_data;
int8_t rslt;

/* Auxiliary function definitions */
int8_t bmm150_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
 // (void) id; /* id is unused here */
  return bmi160_aux_read(reg_addr, reg_data, len, &bmi);
}

int8_t bmm150_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
//  (void) id; /* id is unused here */
  return bmi160_aux_write(reg_addr, reg_data, len, &bmi);
}

int8_t I2cGetRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
  if(len == 1){
    I2CB1_Send(dev_addr, &reg_addr, 1);
    data[0] = I2CB1_Recv1(dev_addr);
  }else{
    I2CB1_Send(dev_addr, &reg_addr, 1);
    I2CB1_Recv(dev_addr,data,len);
  }
  return BMI160_OK;
}

int8_t I2cSetRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
  if(len == 1){
    I2CB1_Send2(dev_addr, reg_addr, data[0]);
    return BMI160_OK;
  }
  if(len == 2){
    I2CB1_Send3(dev_addr, reg_addr, data);
    return BMI160_OK;
  }
  if(len == 3){
    I2CB1_Send4(dev_addr, reg_addr, data);
    return BMI160_OK;
  }
  return BMI160_E_INVALID_INPUT;
}
#ifdef ERRORCHECK
void CheckFail(char *message){
  if(rslt){
    while(1){
      P1->OUT ^= 0x01;         // profile
      Clock_Delay1ms(500);
    }
  }
}
#else
#define CheckFail(X)
#endif

// Main thread scheduled by OS round robin preemptive scheduler
// non-real-time task
// reads data from 9-axis IMU
// Inputs:  none
// Outputs: none
void Task3(void){
  OS_Wait(&I2Cmutex);
  bmi.id = BMI160_I2C_ADDR;
  bmi.read = I2cGetRegs;
  bmi.write = I2cSetRegs;
  bmi.delay_ms = Clock_Delay1ms;
  bmi.interface = BMI160_I2C_INTF;

      /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
      /* Check the pins of the BMM150 for the right I2C address */
  bmm.dev_id = BMI160_AUX_BMM150_I2C_ADDR;
  bmm.intf = BMM150_I2C_INTF;
  bmm.read = bmm150_aux_read;
  bmm.write = bmm150_aux_write;
  bmm.delay_ms = Clock_Delay1ms;

  rslt = bmi160_soft_reset(&bmi);
  CheckFail("bmi160_soft_reset");
  rslt = bmi160_init(&bmi);
  CheckFail("bmi160_init");

      /* Configure the BMI160's auxiliary interface for the BMM150 */
  bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
  bmi.aux_cfg.aux_i2c_addr = bmm.dev_id;
  bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
  bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */
  rslt = bmi160_aux_init(&bmi);
  CheckFail("bmi160_aux_init");

  rslt = bmm150_init(&bmm);
  CheckFail("bmm150_init");

      /* Configure the accelerometer */
  bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
  bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Configure the gyroscope */
  bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
  bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  rslt = bmi160_set_sens_conf(&bmi);
  CheckFail("bmi160_set_sens_conf");

     /* Configure the magnetometer. The regular preset supports up to 100Hz in Forced mode */
  bmm.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
  rslt = bmm150_set_presetmode(&bmm);
  CheckFail("bmm150_set_presetmode");

      /* It is important that the last write to the BMM150 sets the forced mode.
       * This is because the BMI160 writes the last value to the auxiliary sensor
       * after every read */
  bmm.settings.pwr_mode = BMM150_FORCED_MODE;
  rslt = bmm150_set_op_mode(&bmm);
  CheckFail("bmm150_set_op_mode");

  uint8_t bmm150_data_start = BMM150_DATA_X_LSB;
  bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;
  rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
  CheckFail("bmi160_set_aux_auto_mode");

      /* Link the FIFO memory location */
  fifo_frame.data = fifo_buff;
  fifo_frame.length = FIFO_SIZE;
  bmi.fifo = &fifo_frame;

   /* Clear all existing FIFO configurations */
  rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK , BMI160_DISABLE, &bmi);
  CheckFail("bmi160_set_aux_auto_mode");

  uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX |  BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
  rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
  CheckFail("bmi160_set_fifo_config");
  OS_Signal(&I2Cmutex);

  while(1){
    Profile_Toggle3(); // viewed by a real logic analyzer to know Task3 started

    /* It is VERY important to reload the length of the FIFO memory as after the
     * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
     * number of bytes read from the FIFO */
    OS_Wait(&I2Cmutex);
    bmi.fifo->length = FIFO_SIZE;
    rslt = bmi160_get_fifo_data(&bmi);
    /* Check rslt for any error codes */

    uint8_t aux_inst = 1, gyr_inst = 1, acc_inst = 1;
    rslt = bmi160_extract_aux(&aux_data, &aux_inst, &bmi);
    CheckFail("bmi160_extract_aux");
    rslt = bmi160_extract_gyro(&gyro_data, &gyr_inst, &bmi);
    CheckFail("bmi160_extract_gyro");
    rslt = bmi160_extract_accel(&accel_data, &acc_inst, &bmi);
    CheckFail("bmi160_extract_accel");

    rslt = bmm150_aux_mag_data(&aux_data.data[0], &bmm);
    OS_Signal(&I2Cmutex);
    CheckFail("bmm150_aux_mag_data");
        /* Copy the compensated magnetometer data */
    mag_data = bmm.data;
    Rx = gyro_data.x;
    Ry = gyro_data.y;
    Rz = gyro_data.z;
    Ax = accel_data.x;
    Ay = accel_data.y;
    Az = accel_data.z;
    Mx = mag_data.x;
    My = mag_data.y;
    Mz = mag_data.z;
    OS_Sleep(100); // 10 Hz
  }
}
/* ****************************************** */
/*          End of Task3 Section              */
/* ****************************************** */


//------------Task4 measures temperature from TMP117-------
// Main thread scheduled by OS round robin preemptive scheduler
// measures temperature
// Inputs:  none
// Outputs: none
void Task4(void){
  OS_Wait(&I2Cmutex);
  TMP117_Init();
  OS_Signal(&I2Cmutex);
  while(1){
    Profile_Toggle4(); // viewed by a real logic analyzer to know Task4 started
    OS_Wait(&I2Cmutex);
    RawTemperature = TMP117_ReadTemperature();
    OS_Signal(&I2Cmutex);
    FixedTemperature = (10*RawTemperature)/128; // 0.1C
    OS_Sleep(1000);     // every 1 second
  }
}
/* ****************************************** */
/*          End of Task4 Section              */
/* ****************************************** */


//------- Task5 measures light from OPT3001 -----------
// Main thread scheduled by OS round robin preemptive scheduler
void Task5(void){
  OS_Wait(&I2Cmutex);
  OPT3001_Init();
  OS_Signal(&I2Cmutex);
  while(1){
    Profile_Toggle5(); // viewed by a real logic analyzer to know Task5 started
    OS_Wait(&I2Cmutex);
    Light = OPT3001_ReadLight();
    OS_Signal(&I2Cmutex);
    OS_Sleep(1000);     // every 1 second
  }
}
/* ****************************************** */
/*          End of Task5 Section              */
/* ****************************************** */

//---------------- Task6 dummy task ----------------
// *********Task6*********
// This OS needs one foreground task that never blocks or sleeps
// Main thread scheduled by OS round robin preemptive scheduler
uint32_t CPUTimeAvailable; // the faster this counts the more CPU time available
void Task6(void){
  CPUTimeAvailable = 0;
  while(1){
    Profile_Toggle6(); // viewed by a real logic analyzer to know Task6 started
    CPUTimeAvailable++;
    OS_Suspend();   // release processor to other tasks
  }
}
/* ****************************************** */
/*          End of Task6 Section              */
/* ****************************************** */

// Remember that you must have exactly one main() function, so
// to work on this step, you must rename all other main()
// functions in this file.
int main(void){ //RTOS main
  Clock_Init48MHz();

  OS_Init();
  Profile_Init();
  UART0_Initprintf(); // initialize UART and printf
  LaunchPad_Init();
  UART0_OutString("Embedded IoT SIM7600 project\n\r");

  SIM7600G_Init(SMS_SIM7600G); // more space on flash in SIM7600G
  UART0_OutString("***Initialization Complete***\n\r");
  Task0_Init();       // real-time init

  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  OS_InitSemaphore(&I2Cmutex, 1); // 1 means free
  OS_FIFO_Init();                 // initialize FIFO used to send data between Task1 and Task2
  
  // Task 0 should run every 1ms
  OS_AddPeriodicEventThread(&Task0, 1);

  // Task1, Task2, Task3, Task4, Task5, Task6 are main threads
  OS_AddThreads(&Task1, &Task2, &Task3, &Task4, &Task5, &Task6);

  OS_Launch(48000); // 1ms switching, doesn't return, interrupts enabled in here
  return 0;         // this never executes
}
