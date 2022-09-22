/*
 * SIM7600G.c
 *
*/
// Pranav Rama, Daniel Valvano, and Jonathan Valvano
// September 14, 2022

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
// 5V      , LaunchPad 5V
// GND     , LaunchPad GND


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "msp.h"
#include "SIM7600G.h"
#include "UART0.h"
#include "UART1.h"
#include "Clock.h"
#define SIM7600GDELAY 1000 //ms
#ifdef SIM7600G_DEBUG
#define DebugOutString(x) UART0_OutString((char *)x)
#define DebugOutChar UART0_OutChar
#define DebugOutUHex2 UART0_OutUHex2
#define DebugOutUDec UART0_OutUDec
#else
#define DebugOutString(x)
#define DebugOutChar(x)
#define DebugOutHex2(x)
#define DebugOutUDec(x)
#endif

uint8_t replybuffer[255];  ///< buffer for holding replies from the module
uint8_t const useragent[] = "SIM7600G";
uint8_t const ok_reply[] = "OK";
void SIM7600G_SetSMSStorage(int place);

// *************low level private functions **************
/**
 * @brief Read all available serial input to flush pending data.
 *
 */
void flushInput(void){uint8_t in;
// flush receiver buffer
uint32_t timeoutloop = 0;
  while(timeoutloop++ < SIM7600GDELAY){
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
  while(timeoutloop++ < SIM7600GDELAY){
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
uint32_t readRaw(uint32_t read_length){uint8_t in;
  uint32_t idx = 0;

  while (read_length && (idx < sizeof(replybuffer) - 1)){
    in = UART1_InCharNonBlock();

    if(in){
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
uint8_t readline(uint32_t timeout, int multiline){uint8_t in;
  uint32_t replyidx = 0;

  while (timeout--){
    if(replyidx >= 254){
      // DEBUG_PRINTLN(F("SPACE"));
      break;
    }

    while(in = UART1_InCharNonBlock()){
      if(in == '\r'){
        continue;
      }
      if(in == 0x0A){
        if(replyidx == 0){// the first 0x0A is ignored
          continue;
        }
        if(!multiline){
          timeout = 0; // the second 0x0A is the end of the line
          break;
        }
      }
      replybuffer[replyidx] = in;
     // DebugOutString("0x");DebugOutUHex2(in);DebugOutChar(',');

      replyidx++;
    }

    if(timeout == 0){
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
uint8_t getReply(uint8_t *send, uint32_t timeout){
  flushInput();

  DebugOutString("\n\r---> ");
  DebugOutString(send);

  UART1_OutString((uint8_t *)send);UART1_OutChar(LF);

  uint32_t l = readline(timeout,0);

  DebugOutString("\n\r<--- ");
  DebugOutString(replybuffer);DebugOutChar(CR);DebugOutChar(LF);

  return l;
}
int sendCheckReply(uint8_t *send, const uint8_t *reply, uint32_t timeout){
  if(!getReply(send, timeout))
    return 0;
  return (strcmp((const char *)replybuffer, (const char *)reply) == 0);
}

void SIM7600GCommand(uint8_t *pt){
  UART1_OutString(pt);  // send string
  readAllInput();
}
// *************high level public functions **************

void SIM7600G_Reset(void){ // reset 
  SIM7600GCommand("AT+CRESET=?\r\n");
}

void SIM7600G_PowerDown(void){ // power down
  SIM7600GCommand("AT+CPOF\r\n");
}

void SIM7600G_Init(int place){
// baud choices defined in UART1.h
  UART1_InitB(UART1_BAUD_115200);       // serial port to SIM7600G
  //************ configure the SIM7600G module**********************
  SIM7600G_Reset();
  SIM7600GCommand("AT\r");        // initialize auto-baud'er
  SIM7600GCommand("AT\r");        // initialize auto-baud'er
  SIM7600GCommand("ATE1\r");      // Turn on echo
  SIM7600GCommand("AT+CMEE=2\r"); // Turn on verbose errors
#ifdef SIM7600G_DEBUG
  SIM7600GCommand("ATI\r");       // Get the module name and revision
  SIM7600GCommand("AT+CCID\r");   // Report CCID for device
  SIM7600GCommand("AT+COPS?\r");  // Check that you're connected to the network
  SIM7600GCommand("AT+CSQ\r");    // Check the 'signal strength' - the first # is dB strength,
  SIM7600GCommand("AT+CBC\r");    // will return the lipo battery state.
  // The second number is the % full (in this case its 92%) and
  // the third number is the actual voltage in mV
  SIM7600GCommand("AT+CPMS=?\r"); // list of supported SMS memory devices
  SIM7600GCommand("AT+CPMS?\r");  // SMS messages
  SIM7600GCommand("AT+CSMS=?\r"); // list of supported SMS services
  SIM7600GCommand("AT+CSMS?\r");  // SMS service
  SIM7600GCommand("AT+CFGRI?\r"); // check RI configuration service
#endif
  SIM7600GCommand("ATE0\r");      // Turn off echo

  SIM7600G_SetSMSStorage(place); //Set storage to flash since it has greater capacity

  Clock_Delay1ms(1000);
  readAllInput(); // remove any buffered input
  //************ configuration ended***********************
}
void SIM7600G_Restart(int place){
  readAllInput(); // remove any buffered input
  //************ configure the SIM7600G module**********************
  SIM7600G_Reset();
  SIM7600GCommand("AT\r");        // initialize auto-baud'er
  SIM7600GCommand("AT\r");        // initialize auto-baud'er
  SIM7600GCommand("ATE1\r");      // Turn on echo
  SIM7600GCommand("AT+CMEE=2\r"); // Turn on verbose errors
  SIM7600GCommand("ATE0\r");      // Turn off echo
  SIM7600G_SetSMSStorage(place); //Set storage to flash since it has greater capacity
  Clock_Delay1ms(1000);
  readAllInput(); // remove any buffered input
  //************ configuration ended***********************
}

// Set the preferred SMS storage.
//  SMS_SIM = 1 for storage on the SIM (small size).
//  SMS_SIM7600G= 0 for storage on the SIM7600G chip, flash, (large size)
void SIM7600G_SetSMSStorage(int place){
  if(place){
    SIM7600GCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"\r"); // set SMS memory devices to SIM card
  }else{
    SIM7600GCommand("AT+CPMS=\"ME\",\"ME\",\"ME\"\r"); // set SMS memory devices to SIM7600G chip
  }
  SIM7600GCommand("AT+CSMS?\r");  // check SMS service storage space
}

void SIM7600G_SetSMSInterrupt(int n){
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


int32_t SIM7600G_GetNumSMS(void){
  // Get the number of SMS messages
  // "AT+CMGF=1"  get into text mode
  // "AT+CPMS?" ask how many sms are stored
  // parse response

  if(!sendCheckReply((uint8_t *)"AT+CMGF=1\r", ok_reply,5000)){
    DebugOutString("AT+CMGF=1 fail\n");
    return 0; // fail
  }
  uint8_t sendcmd[30] = "AT+CPMS?\r";

  if(!getReply(sendcmd, 5000)){
    DebugOutString("AT+CPMS fail\n");
    return 0; // fail
  }

  char* message_received = strstr((char *)replybuffer, "+CPMS");
  if(message_received == 0){
    return 0; // fail
  }

  char* pt = strtok (message_received,",");
  if(pt != NULL){
    pt = strtok (NULL, ",");
  }

  else{
    DebugOutString("AT+CPMS unexpected response; fail\n");
    return 0;
  }

  int num_messages = 0;
  if(pt == NULL){
    DebugOutString("AT+CPMS unexpected response; fail\n");
    return 0;
  }
  else{
    num_messages = atoi(pt);
  }

  readline(1000,1); // read OK

  message_received = strstr((char *)replybuffer, "OK");

  if(message_received == 0){
    return 0; // fail
  }
  //Only print out and return success if whole transaction including OK receive was complete
  else{
    DebugOutString("Num messages = ");
    DebugOutUDec(num_messages);
    DebugOutString("\n\r");
    return num_messages; // success
  }
}

// Reading SMS's is a bit involved so we don't use helpers that may cause delays
// or debug printouts!

/**
 * @brief Read an SMS message into a provided buffer
 *
 * @param message_index The SMS message index to retrieve, 0 to 255
 * @param smsbuff       Pointer to empty buffer into which message will be filled
 * @param maxlen        The maximum read length (size of smsbuff)
 * @param readlen       The length, number of characters read
 * @return 1: success, 0: failure
 */
int SIM7600G_ReadSMS(uint32_t message_index, char *smsbuff,
                            uint32_t maxlen, uint32_t *readlen){

  if(message_index > 255){
    return 0;
  }
  // select text format (not PDU)
  if(!sendCheckReply((uint8_t *)"AT+CMGF=1\r", ok_reply,5000)){
    DebugOutString("AT+CMGF=1 fail\n");
    return 0; // fail
  }
  uint8_t sendcmd[30] = "AT+CMGR=";
  int temp_len = strlen((char*)sendcmd);
  if(message_index >= 100){
    sendcmd[temp_len++] = (uint8_t)(message_index/100+'0');
    message_index = message_index/100;
    sendcmd[temp_len++] = (uint8_t)(message_index/10+'0');
    sendcmd[temp_len++] = (uint8_t)(message_index%10+'0');
    sendcmd[temp_len] = '\r';
  }else if(message_index >= 10){
    sendcmd[temp_len++] = (uint8_t)(message_index/10+'0');
    sendcmd[temp_len++] = (uint8_t)(message_index%10+'0');
    sendcmd[temp_len] = '\r';
  }else{
    sendcmd[temp_len++] = (uint8_t)(message_index+'0');
    sendcmd[temp_len] = '\r';
  }
  if(!getReply(sendcmd, 5000)){
    DebugOutString("AT+CMGR fail\n");
    return 0; // fail
  }

  char* message_received = strstr((char *)replybuffer, "+CMGR");
  if(message_received == 0){
    return 0; // fail
  }

  char actual_message[255];
  readline(10000,0); // read the actual message, wait up to 10 seconds!!!
  strcpy(actual_message, (char*)replybuffer);
  if(actual_message == 0){
    return 0; // fail
  }

  //Using multiline read here; looks a bit tricky to get the exact line position of "OK"
  //since there could be other metadata between the actual message and OK
  //metadata typically small and much less than 255 bytes
  readline(1000,1); // read OK

  message_received = strstr((char *)replybuffer, "OK");

  if(message_received == 0){
    return 0; // fail
  }
  //Only print out and return success if whole transaction including OK receive was complete
  else{
    DebugOutString(" Message received: ");
    strncpy(smsbuff, actual_message, maxlen);
    *readlen = strlen(smsbuff);
    DebugOutString(actual_message);
    DebugOutString("\n\r");
    return 1; // success
  }
}

/**
 * @brief Delete an SMS Message
 *
 * @param message_index The message to delete
 * @return 1: success, 0: failure
 */
int SIM7600G_DeleteSMS(uint32_t message_index){
    // select text format (not PDU)
  if(!sendCheckReply("AT+CMGF=1\r", ok_reply, 5000))
    return 0;
  // delete an SMS
  char sendbuff[14] = "AT+CMGD=000";
  if(message_index >= 100){
    sendbuff[8] = (message_index / 100) + '0';
    message_index %= 100;
    sendbuff[9] = (message_index / 10) + '0';
    message_index %= 10;
    sendbuff[10] = message_index + '0';
    sendbuff[11] = ',';
    sendbuff[12] = '0';
    sendbuff[13] = '\r';
  }else if(message_index >= 10){
    sendbuff[8] = (message_index / 10) + '0';
    message_index %= 10;
    sendbuff[9] = message_index + '0';
    sendbuff[10] = ',';
    sendbuff[11] = '0';
    sendbuff[12] = '\r';
  }else{
    sendbuff[8] = message_index + '0';
    sendbuff[9] = ',';
    sendbuff[10] = '0';
    sendbuff[11] = '\r';
  }

  int del_ret = sendCheckReply((uint8_t *)sendbuff, ok_reply, 2000);
  if(del_ret == 0){
      DebugOutString("Message delete failed \n\r");
      return 0;
  }
  DebugOutString("Message successfully deleted \n\r");
  return 1;

}

/**
 * @brief Retrieve the sender phone number of the specified SMS message and
 *    copy it as a string to the sender buffer.
 *    Up to senderlen characters will be copied
 *    and a null terminator will be added if less than senderlen
 *    characters are copied to the result.
 *
 * @param message_index The SMS message index to retrieve the sender phone number
 * @param sender        Pointer to an empty buffer into which to fill with the sender phone number
 * @param senderlen     The maximum length to read (size of empty buffer)
 * @return 1: a result was successfully retrieved, 0: failure
 */
int SIM7600G_GetSMSSender(uint32_t message_index, char *sender,
                                 int senderlen){
  if(message_index > 255){
    return 0;
  }
  if(!sendCheckReply((uint8_t *)"AT+CMGF=1\r", ok_reply,5000)){
    DebugOutString("AT+CMGF=1 fail\n");
    return 0; // fail
  }
  uint8_t sendcmd[30] = "AT+CMGR=";
  int temp_len = strlen((char*)sendcmd);
  if(message_index >= 100){
    sendcmd[temp_len++] = (uint8_t)(message_index/100+'0');
    message_index = message_index/100;
    sendcmd[temp_len++] = (uint8_t)(message_index/10+'0');
    sendcmd[temp_len++] = (uint8_t)(message_index%10+'0');
    sendcmd[temp_len] = '\r';
  }else if(message_index >= 10){
    sendcmd[temp_len++] = (uint8_t)(message_index/10+'0');
    sendcmd[temp_len++] = (uint8_t)(message_index%10+'0');
    sendcmd[temp_len] = '\r';
  }else{
    sendcmd[temp_len++] = (uint8_t)(message_index+48);
    sendcmd[temp_len] = '\r';
  }

  if(!getReply(sendcmd, 5000)){
    DebugOutString("AT+CMGR fail\n");
    return 0; // fail
  }

  char temp_buffer[255];
  char* message_received = strstr((char *)replybuffer, "+CMGR");
  if(message_received == 0){
    return 0; // fail
  }

  char* pt = strtok (message_received,",");
  if(pt != NULL){
    pt = strtok (NULL, ",");
  }

  else{
    DebugOutString("AT+CMGR unexpected response; fail\n");
    return 0;
  }

  if(pt == NULL){
    DebugOutString("AT+CMGR unexpected response; fail\n");
    return 0;
  }

  strcpy(temp_buffer, pt);

  //Using multiline read here; looks a bit tricky to get the exact line position of "OK"
  //since there could be other metadata between the actual message and OK
  //metadata typically small and much less than 255 bytes

  readline(1000,1); // read OK

  message_received = strstr((char *)replybuffer, "OK");

  if(message_received == 0){
    return 0; // fail
  }
  //Only print out and return success if whole transaction including OK receive was complete
  else{
    strncpy(sender, temp_buffer, senderlen);
    DebugOutString("Sender of requested message id: ");
    DebugOutString(sender);
    DebugOutString("\n\r");
    return 1; // success
  }
}


/**
 * @brief Send an SMS Message from a buffer provided
 *
 * @param phone     The SMS address buffer
 * @param message   The SMS message buffer
 * @return true: success, false: failure
 */
int SIM7600G_SendSMS(uint8_t *phone, uint8_t *message){
  if(!sendCheckReply((uint8_t *)"AT+CMGF=1\r", ok_reply,5000)){
    DebugOutString("AT+CMGF=1 fail\n");
    return 0; // fail
  }
  uint8_t sendcmd[30] = "AT+CMGS=\"";
  strncpy((char *)(sendcmd + 9), (char *)phone,
          30 - 9 - 3); // 9 bytes beginning, 2 bytes for close quote + LF + null
  int temp_length = strlen((char *)sendcmd);
  sendcmd[temp_length++] = '\"';
  sendcmd[temp_length] = '\r';

  if(!sendCheckReply(sendcmd, "> ",1000)){
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
  if(strstr((char *)replybuffer, "+CMGS") == 0){
    return 0; // fail
  }
  readline(1000,0); // read OK

  if(strcmp((char *)replybuffer, "OK") != 0){
    return 0; // fail
  }
  return 1; // success
}

