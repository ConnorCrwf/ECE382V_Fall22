/*
 * SIM7600G.c
 *
*/
// Pranav Rama, Daniel Valvano, and Jonathan Valvano
// August 25, 2022

/* 


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
// SIM7600G pin connection


// TX      , MSP432 UCA2RXD P3.2
// RX      , MSP432 UCA2TXD P3.3


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "msp.h"
#include "..\inc\SIM7600G.h"
#include "..\inc\UART0.h"
#include "..\inc\UART1.h"
#include "..\inc\Clock.h"
#define LEDOUT (*((volatile uint8_t *)(0x42098040)))
// P3->OUT is 8-bit port at 0x4000.4C22
// I/O address be 0x4000.0000+n, and let b represent the bit 0 to 7.
// n=0x4C22, b=0
// bit banded address is 0x4200.0000 + 32*n + 4*b
#define RST (*((volatile uint8_t *)(0x42098440)))
#define SIM7600GDELAY 40
#ifdef SIM7600G_DEBUG
#define DebugOutString(x) UART0_OutString((char *)x)
#define DebugOutChar UART0_OutChar
#define DebugOutUHex2 UART0_OutUHex2
#else
#define DebugOutString(x)
#define DebugOutChar(x)
#define DebugOutHex2(x)
#endif

uint8_t replybuffer[255];  ///< buffer for holding replies from the module
uint8_t const useragent[] = "FONA";
uint8_t const ok_reply[] = "OK";
// *************low level private functions **************
/**
 * @brief Read all available serial input to flush pending data.
 *
 */
void flushInput(void){uint8_t in;
// flush receiver buffer
uint32_t timeoutloop = 0;
  while(timeoutloop++ < SIM7600GDELAY) {
    in = UART1_InCharNonBlock();
    while(in){
      in = UART1_InCharNonBlock();
      DebugOutChar(in);
      timeoutloop = 0; // If char was received reset the timer
    }
    Clock_Delay1ms(1); // 40 ms with no input
  }
}

void readAllInput(void){uint8_t in;
// flush receiver buffer
uint32_t timeoutloop = 0;
  while(timeoutloop++ < SIM7600GDELAY) {
    in = UART1_InCharNonBlock();
    while(in){
      DebugOutChar(in); //
      in = UART1_InCharNonBlock();
      timeoutloop = 0; // If char was received reset the timer
    }
    Clock_Delay1ms(1); // 40 ms with no input
  }
}



/**
 * @brief Read directly into the reply buffer
 *
 * @param read_length The number of bytes to return
 * @return uint16_t The number of bytes read
 */
uint32_t readRaw(uint16_t read_length) {uint8_t in;
  uint32_t idx = 0;

  while (read_length && (idx < sizeof(replybuffer) - 1)) {
    in = UART1_InCharNonBlock();

    if (in) {
      replybuffer[idx] = in;
      idx++;
      read_length--;
    }
  }
  replybuffer[idx] = 0;

  return idx;
}

/**
 * @brief Read a single line or up to 254 bytes
 *
 * @param timeout Reply timeout
 * @param multiline true: read the maximum amount. false: read up to the second
 * newline
 * @return uint8_t the number of bytes read
 */
uint8_t readline(uint32_t timeout, int multiline) {uint8_t in;
  uint16_t replyidx = 0;

  while (timeout--) {
    if (replyidx >= 254) {
      // DEBUG_PRINTLN(F("SPACE"));
      break;
    }

    while(in = UART1_InCharNonBlock()) {
      if (in == '\r'){
        continue;
      }
      if(in == 0x0A) {
        if (replyidx == 0){// the first 0x0A is ignored
          continue;
        }
        if(!multiline) {
          timeout = 0; // the second 0x0A is the end of the line
          break;
        }
      }
      replybuffer[replyidx] = in;
     // DebugOutString("0x");DebugOutUHex2(in);DebugOutChar(',');

      replyidx++;
    }

    if(timeout == 0) {
      break;
    }
  //  Clock_Delay1ms(1);
    Clock_Delay1us(1000);
  }
  replybuffer[replyidx] = 0; // null term
  return replyidx;
}
/**
 * @brief Send data and verify the response matches an expected response
 *
 * @param send Pointer to the buffer of data to send
 * @param reply Buffer with the expected reply
 * @param timeout Read timeout
 * @return true: success, false: failure
 */
/**
 * @brief Send a command and return the reply
 *
 * @param send The char* command to send
 * @param timeout Timeout for reading a  response
 * @return uint8_t The response length
 */
uint8_t getReply(uint8_t *send, uint32_t timeout) {
  flushInput();

  DebugOutString("\n\r---> ");
  DebugOutString(send);

  UART1_OutString((uint8_t *)send);UART1_OutChar(LF);

  uint32_t l = readline(timeout,0);

  DebugOutString("\n\r<--- ");
  DebugOutString(replybuffer);DebugOutChar(CR);DebugOutChar(LF);

  return l;
}
int sendCheckReply(uint8_t *send, const uint8_t *reply, uint32_t timeout) {
  if (!getReply(send, timeout))
    return 0;
  return (strcmp((const char *)replybuffer, (const char *)reply) == 0);
}

void SIM7600GCommand(uint8_t *pt){
  UART1_OutString(pt);  // send string
  readAllInput();
}
// *************high level public functions **************

void SIM7600G_Reset(void){ // reset 
 
}


void SIM7600G_Init(uint32_t n){
// baud choices defined in UART1.h
  P3->SEL0 &= ~0x01;    // P3.0 GPIO output to RST
  P3->SEL1 &= ~0x01;    // configure P3.0 as GPIO
  P3->DIR |= 0x01;      // make P3.0 out
  //
  // add code to initialize GPIO input for RI
  //
  UART1_InitB(n);       // serial port to Fona
  //************ configure the Fona module**********************
  SIM7600G_Reset();
  SIM7600GCommand("AT\n");        // initialize auto-baud'er
  SIM7600GCommand("AT\n");        // initialize auto-baud'er
  SIM7600GCommand("ATE1\n");      // Turn on echo
  SIM7600GCommand("AT+CMEE=2\n"); // Turn on verbose errors
#ifdef FONA_DEBUG
  SIM7600GCommand("ATI\n");       // Get the module name and revision
  SIM7600GCommand("AT+CCID\n");   // Report CCID for device
  SIM7600GCommand("AT+COPS?\n");  // Check that you're connected to the network
  SIM7600GCommand("AT+CSQ\n");    // Check the 'signal strength' - the first # is dB strength,
  SIM7600GCommand("AT+CBC\n");    // will return the lipo battery state.
  // The second number is the % full (in this case its 92%) and
  // the third number is the actual voltage in mV
  SIM7600GCommand("AT+CPMS=?\n"); // list of supported SMS memory devices
  SIM7600GCommand("AT+CPMS?\n");  // SMS messages
  SIM7600GCommand("AT+CSMS=?\n"); // list of supported SMS services
  SIM7600GCommand("AT+CSMS?\n");  // SMS service
  SIM7600GCommand("AT+CFGRI?\n"); // check RI configuration service
#endif
  SIM7600GCommand("ATE0\n");      // Turn off echo

  Clock_Delay1ms(200);
  readAllInput(); // remove any buffered input
  //************ configuration ended***********************
}

// Set the preferred SMS storage.
//  SMS_SIM = 1 for storage on the SIM.
//  SMS_FONA= 0 for storage on the FONA chip
void Fona_SetSMSStorage(int place){
  if(place){
    SIM7600GCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"\n"); // set SMS memory devices to SIM card
  }else{
    SIM7600GCommand("AT+CPMS=\"ME\",\"ME\",\"ME\"\n"); // set SMS memory devices to FONA chip
  }
  SIM7600GCommand("AT+CSMS?\n");  // check SMS service
}

void Fona_SetSMSInterrupt(int n){
    // configure RI pin
  if(n == 0){
    SIM7600GCommand("AT+CFGRI=0\n"); // off
  }else if(n == 1){
    SIM7600GCommand("AT+CFGRI=1\n"); // On(TCPIP,FTP and URC control RI PIN)
  }else if(n == 2){
    SIM7600GCommand("AT+CFGRI=2\n"); // On(only TCPIP control RI PIN)
  }
  SIM7600GCommand("AT+CFGRI?\n");  // check RI configuration service
}


int32_t Fona_GetNumSMS(void){
    // Get the number of SMS messages
    // "AT+CMGF=1"  get into text mode
    // "AT+CPMS?" ask how many sms are stored
    // parse response
  return -1;
}

// Reading SMS's is a bit involved so we don't use helpers that may cause delays
// or debug printouts!

/**
 * @brief Read an SMS message into a provided buffer
 *
 * @param message_index The SMS message index to retrieve
 * @param smsbuff SMS message buffer
 * @param maxlen The maximum read length
 * @param readlen The length read
 * @return 1: success, 0: failure
 */
int Fona_ReadSMS(uint32_t message_index, char *smsbuff,
                            uint32_t maxlen, uint32_t *readlen){

  return 0; // fail
}

/**
 * @brief Delete an SMS Message
 *
 * @param message_index The message to delete
 * @return true: success, false: failure
 */
int Fona_DeleteSMS(uint8_t message_index){
/*
  if (!sendCheckReply(F("AT+CMGF=1"), ok_reply))
    return false;
  // read an sms
  char sendbuff[12] = "AT+CMGD=000";
  sendbuff[8] = (message_index / 100) + '0';
  message_index %= 100;
  sendbuff[9] = (message_index / 10) + '0';
  message_index %= 10;
  sendbuff[10] = message_index + '0';

  return sendCheckReply(sendbuff, ok_reply, 2000);
*/
  return 0;
}

/**
 * @brief Retrieve the sender of the specified SMS message and copy it as a
 *    string to the sender buffer.  Up to senderlen characters of the sender
 * will be copied and a null terminator will be added if less than senderlen
 *    characters are copied to the result.
 *
 * @param message_index The SMS message index to retrieve the sender for
 * @param sender Pointer to a buffer to fill with the sender
 * @param senderlen The maximum length to read
 * @return 1: a result was successfully retrieved, 0: failure
 */
int Fona_GetSMSSender(uint32_t message_index, char *sender,
                                 int senderlen){
/* Adafruit_FONA.cpp C++ code

  // Ensure text mode and all text mode parameters are sent.
  if (!sendCheckReply(F("AT+CMGF=1"), ok_reply))
    return false;
  if (!sendCheckReply(F("AT+CSDH=1"), ok_reply))
    return false;

  DEBUG_PRINT(F("AT+CMGR="));
  DEBUG_PRINTLN(message_index);

  // Send command to retrieve SMS message and parse a line of response.
  mySerial->print(F("AT+CMGR="));
  mySerial->println(message_index);
  readline(1000);

  DEBUG_PRINTLN(replybuffer);

  // Parse the second field in the response.
  bool result = parseReplyQuoted(F("+CMGR:"), sender, senderlen, ',', 1);
  // Drop any remaining data from the response.
  flushInput();
  return result;
  */
  return 0; // fail
}


/**
 * @brief Send an SMS Message from a buffer provided
 *
 * @param phone     The SMS address buffer,
 * @param message   The SMS message buffer
 * @return true: success, false: failure
 */
int Fona_SendSMS(uint8_t *phone, uint8_t *message) {
  if (!sendCheckReply((uint8_t *)"AT+CMGF=1", ok_reply,5000)){
    DebugOutString("AT+CMGF=1 fail\n");
    return 0; // fail
  }
  uint8_t sendcmd[30] = "AT+CMGS=\"";
  strncpy((char *)(sendcmd + 9), (char *)phone,
          30 - 9 - 2); // 9 bytes beginning, 2 bytes for close quote + null
  sendcmd[strlen((char *)sendcmd)] = '\"';

  if (!sendCheckReply(sendcmd, "> ",1000)){
   DebugOutString(sendcmd);
   DebugOutString(" fail\n ");
   return 0; // fail
  }
  DebugOutString("> ");
  DebugOutString(message); DebugOutChar(CR);DebugOutChar(LF);
  UART1_OutString(message); // sending text
  UART1_OutChar(LF);
  UART1_OutChar(0x1A);     // terminate SMS

  DebugOutString("^Z\n");

  readline(10000,0); // read the +CMGS reply, wait up to 10 seconds!!!
  if (strstr((char *)replybuffer, "+CMGS") == 0) {
    return 0; // fail
  }
  readline(1000,0); // read OK

  if (strcmp((char *)replybuffer, "OK") != 0) {
    return 0; // fail
  }
  return 1; // success
}

