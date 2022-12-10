// LoRaFSM.ino
// Load onto two boards
// Jonathan Valvano
// Embedded IoT, University of Texas at Austin
// http://users.ece.utexas.edu/~valvano/
// Nov 11, 2020
//derived from Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX
/**************************************************************************
 OLED code from Adafruit
 This is an example for our Monochrome OLEDs based on SSD1306 drivers
 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98
 This example is for a 128x64 pixel display using I2C to communicate
 2 pins are required to interface (two I2C).
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!
 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
/* Hardware
SSD1306 interface
   3.3V power to SSD1306 VCC
   #2 - SDA I2C data pin, SSD1306
   #3 - SCL I2C clock pin, SSD1306
   GND  to SSD1306 GND
Switch inputs
   #5 - GPIO #5, switch input (external 10k pulldown positive logic)
LED outputs
   #12 - GPIO #12, red LED (2mA, 470 ohm, positive logic)
   #11 - GPIO #11, yellow LED (2mA, 470 ohm, positive logic)
   #10 - GPIO #10, green LED (2mA, 470 ohm, positive logic)
Circuits on the PCB
   #9 - GPIO #9, analog input A9, lipoly battery connected to a voltage divider
   #13 - GPIO #13, red LED on board
Radio pins
   SPI pins - SCK/MOSI/MISO 
   #8 - RFM95_CS
   #4 - RFM95_RST
   #7 - RFM95_INT
*/
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
// Lab 4 hardware
#define SW1 5
#define REDLED 12
#define YELLOWLED 11
#define GREENLED 10
#define LED 13
#define SERIALDEBUG 1
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int lastSW1,thisSW1;
int line;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
void debug(char *msg){ // 8 rows by 21 characters 
  display.setCursor(0,line*8);
  display.print(msg);
  display.display();
  line++;
  if(line > 7){
    display.clearDisplay();
    line=0;
  }
#ifdef SERIALDEBUG
  Serial.println(msg);
#endif
}
void debugNum(char *msg, int num){ // 8 rows by 21 characters 
  display.setCursor(0,line*8);
  display.print(msg);
  display.print(num,DEC);
  display.display();
  line++;
  if(line > 7){
    display.clearDisplay();
    line=0;
  }
#ifdef SERIALDEBUG
  Serial.print(msg);
  Serial.println(num,DEC);
#endif
}
void setup(){
  pinMode(LED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  // test LEDs
  digitalWrite(REDLED, HIGH);
  digitalWrite(YELLOWLED, HIGH);
  digitalWrite(GREENLED, HIGH);
  pinMode(SW1, INPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  lastSW1 = digitalRead(SW1);
#ifdef SERIALDEBUG
  Serial.begin(115200);
  delay(100);
#endif
// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ // Address 0x3C for 128x64
#ifdef SERIALDEBUG
    Serial.println(F("SSD1306 allocation failed"));
#endif
    digitalWrite(REDLED, HIGH);
    digitalWrite(YELLOWLED, LOW);
    digitalWrite(GREENLED, HIGH);
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(1000); // view Adafruit splash, wait for RFM95
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  line = 0;
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  debug("LoRa 32u4 FSM");
  debug("Valvano IoT");
  while (!rf95.init()) {
    debug("LoRa init fail");
    while(1);
  }
  debug("LoRa radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    debug("Frequency fail");
    while(1);
  }
  debug("F=433MHz");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  digitalWrite(REDLED, LOW);
  digitalWrite(YELLOWLED, LOW);
  digitalWrite(GREENLED, LOW);  
}

void loop(){ 
  if(rf95.available()){ // handle received messages
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if(rf95.recv(buf, &len)){
      if(buf[0] == 'R'){
        digitalWrite(REDLED, HIGH); 
        debugNum("Recv R, RSSI ",rf95.lastRssi());
        uint8_t data[] = "A"; // ack
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }
      else if(buf[0] == 'r'){
        digitalWrite(REDLED, LOW);   
        debugNum("Recv r, RSSI ",rf95.lastRssi());
        uint8_t data[] = "A"; // ack
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
      }else if(buf[0] == 'A'){
        debug("Recv Ack"); // ack from other
      }
    }
    else{
      debug("Receive failed");  
    }
  }
  thisSW1 = digitalRead(SW1);
  if((thisSW1==HIGH)&&(lastSW1==LOW)){
    debug("Send R");  
    digitalWrite(GREENLED, HIGH);
    uint8_t data[] = "R"; // turn red on
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  if((thisSW1==LOW)&&(lastSW1==HIGH)){
    debug("Send r");  
    digitalWrite(GREENLED, LOW);
    uint8_t data[] = "r"; // turn red off
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  lastSW1 = thisSW1;
  delay(10); // debounce switches
}
