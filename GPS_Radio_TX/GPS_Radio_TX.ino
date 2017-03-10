// LoRa 9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

bool sendFlag = true;
bool buttonFlag = false;

const int timeDelay = 2000;
unsigned long rxTime = 0;


void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(200);

  Serial.println("Arduino LoRa TX Test!");
  Serial.println(RH_RF95_MAX_MESSAGE_LEN);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

}

void loop()
{
char c = GPS.read(); //Must call GPS.read() at some point for data transmission to occur

  if(GPS.newNMEAreceived() && sendFlag)
  {

    if (!GPS.parse(GPS.lastNMEA())) // this sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
sendFlag = false;

float latitude = GPS.latitude;
float longitude = GPS.longitude;
float altitude = GPS.altitude;
char lat = GPS.lat;
char lon = GPS.lon;

Serial.print("Incoming Data: ");
Serial.print(latitude);
Serial.print(lat);
Serial.print(",");
Serial.print(longitude);
Serial.print(lon);
Serial.print(",");
Serial.println(altitude);

uint8_t radioPacket[16];

union{
  float tempFloat;
  byte tempArray[3];
} u;

u.tempFloat = latitude;
for(int c=0; c<4; c++){
  radioPacket[c] = u.tempArray[c];
}
radioPacket[4] = lat;
u.tempFloat = longitude;
for(int c=0; c<4; c++){
  radioPacket[(c+5)] = u.tempArray[c];
}
radioPacket[9] = lon;
u.tempFloat = altitude;
for(int c=0; c<4; c++){
  radioPacket[(c+10)] = u.tempArray[c];
}
radioPacket[14] = GPS.fix;
radioPacket[15] = GPS.fixquality;
radioPacket[16] = 5;
 
 if(!buttonFlag) {
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  digitalWrite(13, HIGH);
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radioPacket, 16);
 }
 else {
  Serial.println("Warned not to send GPS data. Sending confirmation...");
  
  digitalWrite(13, HIGH);
  Serial.println("Sending..."); delay(10);
  uint8_t dummy[18] = "No GPS data sent!";
  dummy[17] = 0;
  rf95.send(dummy, 18);
 }

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  digitalWrite(13, LOW);
  // Now wait for a reply

  Serial.println("Waiting for reply...");

  rxTime = millis();
  }

else{ 
if (rf95.available())
  { 
    digitalWrite(13, HIGH);
    sendFlag = true;
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);   

      if (len > 20) {
        buttonFlag = true;
      }
      else {
        buttonFlag = false;
      }
    }
    else
    {
      Serial.println("Receive failed");
    }
    delay(100);
    digitalWrite(13, LOW);
  }

else if((millis() - rxTime) > timeDelay)
  sendFlag = true;
}

}
