// Seeed_HC12main.c
// Message protocol definitions.
// Lab 1 start project for ECE382V, Technology for Embedded IoT
// Daniel I. Meza

#ifndef SEEED_HC12MAIN_H_
#define SEEED_HC12MAIN_H_


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


#define HEADER      '?'
#define MAX_MSG_LEN 12
#define RX_TIMEOUT  9600


// Message structure
typedef struct Message {
  uint8_t header;
  uint8_t address;
  uint8_t length;
  uint8_t data[MAX_MSG_LEN];
  uint8_t error;
} message_t;


// Global variables
char MY_ID =    '1';
char DEST_ID =  '1';
char OutMessage[] = "Hello World";

char G_SEND_BUFF[sizeof(message_t)];
int32_t G_UNSENT_BYTES = 0;
char *G_SEND_PTR = NULL;

char G_RECV_BUFF[sizeof(message_t)];
int32_t G_UNRECV_BYTES = 0;
char *G_RECV_PTR = NULL;

uint32_t RX_TIMEOUT_CNT = 0;


#endif /* SEEED_HC12_H_ */
