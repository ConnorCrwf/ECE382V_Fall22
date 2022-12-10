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
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

#define HEADER '?'
#define MAX_MSG_LEN 32
#define MY_ADDRESS '1'
#define DEST_ADDRESS '1'


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int lastSW1,thisSW1;
int line;
uint8_t state;
uint8_t Message[MAX_MSG_LEN] = "Hello World";
struct message_t {
  uint8_t header;
  uint8_t address;
  uint8_t length;
  uint8_t data[MAX_MSG_LEN];
} MessagePacket;


// Function to print string to display.
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


// Function to print string and number to display.
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


// Handle sending message
void sendMessage(void) {
  // Turn off LEDs
  digitalWrite(REDLED, LOW);
  digitalWrite(YELLOWLED, LOW);
  digitalWrite(GREENLED, LOW);

  // Send data
  rf95.send((char*)&MessagePacket, sizeof(MessagePacket));
  rf95.waitPacketSent();

  // Debug statements
  Serial.print("Sent Message: ");
  debug((char*)&MessagePacket);

  // Receive acknowledgement (for distance testing)
  uint8_t buf[5];
  uint8_t len = sizeof(buf);
  // Wait for reply
  if (rf95.waitAvailableTimeout(100)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      debug("Received Ack");
    }
    else {
      debug("Failed Ack");
    }
  }
  else {
    debug("Missed Ack");
  }

  delay(100);   // small delay to see LED blink
}


// Handle receiving message
void receiveMessage(void) {
  uint8_t buf[MAX_MSG_LEN + 3];   // Message size + metadata
  uint8_t len = sizeof(buf);

  // Retrieve data into buffer
  if (rf95.recv(buf, &len)) {
    if (buf[0] == HEADER && buf[1] == MY_ADDRESS ) {   // Correct message is for me
      state = (state + 1) % 3;  // move state

      // Debug statements
      Serial.print("Received Message: ");
      debug((char*)&buf);

      // Send acknowledgement (for distance testing)
      rf95.send('A', sizeof('A'));
      rf95.waitPacketSent();
      debug("Sent Ack");
    }
  }
  else {
    Serial.print("Received Failed");
  }
}


// Handle state machine of the system
void stateMachine(void) {
  switch (state) {
    case 0:
      // Red LED on
      digitalWrite(REDLED, HIGH);
      digitalWrite(YELLOWLED, LOW);
      digitalWrite(GREENLED, LOW);
      break;
    case 1:
      // Yellow LED on
      digitalWrite(REDLED, LOW);
      digitalWrite(YELLOWLED, HIGH);
      digitalWrite(GREENLED, LOW);
      break;
    case 2:
      // Green LED on
      digitalWrite(REDLED,LOW);
      digitalWrite(YELLOWLED, LOW);
      digitalWrite(GREENLED, HIGH);
      break;
  }
}


void setup() { 
  // Setup LEDs, switch and radio pin output
  pinMode(LED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(RFM95_RST, OUTPUT);

  // Test LEDs, switch and radio pin output
  digitalWrite(REDLED, HIGH);
  digitalWrite(YELLOWLED, HIGH);
  digitalWrite(GREENLED, HIGH);
  digitalWrite(RFM95_RST, HIGH);
  lastSW1 = digitalRead(SW1);

  #ifdef SERIALDEBUG
    Serial.begin(115200);
    delay(100);
  #endif

  // Check if display is working
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
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

  // Manual reset
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

  // Initialization messages
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

  // Defaults after init are 433.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  digitalWrite(LED, LOW);   // Profiling
  digitalWrite(REDLED, LOW);
  digitalWrite(YELLOWLED, LOW);
  digitalWrite(GREENLED, LOW);

  // Prepare send message
  MessagePacket.header = HEADER;
  MessagePacket.address = DEST_ADDRESS;
  MessagePacket.length = sizeof(Message);
  memcpy(MessagePacket.data, Message, MessagePacket.length);
}


void loop() {
  // Handle sending message
  thisSW1 = digitalRead(SW1);
  if((thisSW1 == HIGH) && (lastSW1 == LOW)) {
    sendMessage();
  }

  // Handle receiving message
  if(rf95.available()){
    receiveMessage();
  }
  // Update state machine (when needed)
  stateMachine();

  lastSW1 = thisSW1;  // update switch input
  delay(10);  // debounce switches
}
