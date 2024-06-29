#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFi.h> // Include the WiFi library
#include <FastLED.h>

// Define the number of LEDs and the data pin
#define NUM_LEDS 30
#define LED_DATA_PIN 2

// Create an array to hold the color data for the LEDs
CRGB leds[NUM_LEDS];

#define CE 19
#define SCK 18
#define MISO 5
#define CSN 4
#define MOSI 16
#define IRQ 17

#define VERBOSE

// UnitID 1 - 7
// 5 byte address string
// Byte 0 - Unit ID (0x01 - 0x07) + RF Channel
// Byte 1 - 255 - UnitID
// Byte 2 - 255 - RF Channel
// Byte 3 - UnitID
// Byte 4 - RF Channel
//
// CONFIG = "3F"  0x00111111 RX mode, Power Up, EN_CRC
// EN_AA = "00" No CRC, No auto Ack
// EN_RXADDR = "01" Enable data pipe 1
// SETUP_AW = "03" Use 5 byte addresses
// SETUP_RETR = "00" no auto-retransmit delay
// RF_CH = "00"
// RF_SETUP = "26" 0x00100110  256kbps, 0dBm
// STATUS = "7E"
// RX_PW_P0 = "20" 32 byte payload
//
// example payload DMX ch 1-
// byte DE AD BE EF 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
//      80 00 FF 01 DE AD BE EF 05 06 07 08 09 0A 00 00 00 00 00 10 11 12 13 14 15 16 17 18 19 1A 1B 1C
//      80 payloadID FF 01 then 28 DMX channels
// thru
//      80 12 FF 01 XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX  01FF = 511
//

// Receiver code goes as follows
// 1. setup the radio gear
// 2. set receive address.
// 3. listen on a channel
// 4. if signal found, start taking payloads into DMX buffer
// 5. if no signal found, loop back to 2
//

uint8_t dmxBuf[512];  // initialise DMX buffer
uint8_t myAddress[5]; // initialise address
bool channelUsed[126];
unsigned long flashTimer, receiveTimer, lastPayloadTime, chaseTimer;

// int step = 1;
// bool txparamsSet;
bool gotLock = false;
RF24 radio(12, 14); // CE, CSN

bool transmitter = false; // = true;
bool configMode = false;

// int rfCH = 70;                // temporarily set rfCH
int unitID = 1;               // set initial unit ID. 1 = RED, 2 = GREEN
const int dmxPerPayload = 28; //
const int numPayloads = 19;   // 0x00 - 0x12 == 19 payloads

// int counter;

void clearDMX()
{
  for (int i = 0; i < 512; i++)
  {
    dmxBuf[i] = 0;
  }
}

void getAddress(int ID, int channel)
{
  myAddress[4] = ID + channel;
  myAddress[3] = 255 - ID;
  myAddress[2] = 255 - channel;
  myAddress[1] = ID;
  myAddress[0] = channel;
}

void setChannel(int rfCh)
{
  getAddress(unitID, rfCh);
  delay(1);
  radio.flush_rx();
  radio.openReadingPipe(0, myAddress);
  radio.startListening();
  radio.setChannel(rfCh);
}

void doScan()
{
  for (int rfCH = 0; rfCH < 126; rfCH++)
  {
    getAddress(unitID, rfCH);
    delay(1);
    radio.flush_rx();
    radio.openReadingPipe(0, myAddress);
    radio.startListening();
    radio.setChannel(rfCH);
#ifdef VERBOSE
    Serial.print("Trying channel ");
    Serial.println(radio.getChannel());
#endif
    unsigned long started_waiting_at = micros(); // timeout setup
    bool timeout = false;
    while (!radio.available())
    { // While nothing is received
      if (micros() - started_waiting_at > 10000)
      { // If waited longer than 10ms, indicate timeout and exit while loop
        timeout = true;
        break;
      }
    }
    if (!timeout)
    {
      uint8_t buf[32];
      radio.read(buf, sizeof(buf));
      if (buf[0] == 0x80)
      {
        gotLock = true;
      }
    }
    if (gotLock)
    {
#ifdef VERBOSE
      Serial.print("Found a transmitter on channel ");
      Serial.println(rfCH);
#endif
      break;
    }
  }
}

void printDmxData()
{
  // Print the DMX data
  Serial.print("DMX Data Received on Channel ");
  Serial.println(radio.getChannel());
  for (int i = 0; i < 512; i++)
  {
    Serial.print(dmxBuf[i]);
    Serial.print(" ");
    if ((i + 1) % 50 == 0)
    { // Print 16 values per line for better readability
      Serial.println();
    }
  }
  Serial.println();
}

void setupLed()
{
  // Initialize the FastLED library
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);

  // Set the brightness of the LEDs
  FastLED.setBrightness(255); // Adjust brightness (0-255)

  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show(); // Update the strip to show the new colors
}

void updateLed()
{
  int8_t bri = dmxBuf[0];

  int8_t red = dmxBuf[1];
  int8_t green = dmxBuf[2];
  int8_t blue = dmxBuf[3];

  // Convert RGB to CRGB
  CRGB color = CRGB(red, green, blue);

  // Set the brightness of the LEDs
  FastLED.setBrightness(bri); // Adjust brightness (0-255)

  fill_solid(leds, NUM_LEDS, color);
  FastLED.show(); // Update the strip to show the new colors
}

void setup()
{
  setupLed();

  clearDMX(); // set dmxBuf to all zeros

#ifdef VERBOSE
  Serial.begin(115200);
#endif
#ifdef VERBOSE
  if (WiFi.mode(WIFI_OFF))
  {
    Serial.println("wifi radio disabled");
  }
#endif

  if (!radio.begin())
  {
#ifdef VERBOSE
    Serial.println("ERROR: failed to start radio");
#endif
  }
  delay(100);
#ifdef VERBOSE
  Serial.println("Starting up!");
#endif
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setPayloadSize(32);
  radio.setChannel(0);

  doScan();
}

#define MAXPAYLOAD 32 // max payload size for nrf24l01
#define MAXGROUPS 17  // 17 groups of 30 channels = 510 channels
uint8_t buf[MAXPAYLOAD], DMXData[30 * MAXGROUPS];
uint8_t lastStamp, channel = 0, i;

void loop()
{
  if (!gotLock) doScan();

  while (radio.available())
  {
    lastPayloadTime = millis();
    radio.read(buf, sizeof(buf));

    if(buf[0] != 0x80 && buf[1] != 0xFF && buf[2] != 0x01)
    {
      continue;
    }

    int payloadID = buf[1];
    
    int channelPlace = payloadID * 28;

    for (int place = 4; place < 32; place++)
    { 
      dmxBuf[channelPlace + (place - 4)] = buf[place];
    }

    updateLed();
    delayMicroseconds(500);
  }

  if (millis() - lastPayloadTime > 10 * 1000) {
    gotLock = false;
    Serial.println ("Not received a payload for > 10s"); 
  }

  delay(500);
}