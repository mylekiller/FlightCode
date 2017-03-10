// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13
#define BUTTON 11

bool sendFlag = false;

bool buttonFlag = false;

void setup() 
{
  Serial.begin(9600);

  Serial.println("Arduino LoRa RX Test!");
  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
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
  buttonFlag = digitalRead(BUTTON);
  if(sendFlag) {
    digitalWrite(LED, HIGH);

    if(!buttonFlag){
      Serial.println("Button off, so proceed!");
    uint8_t data[14] = "All is well.";
      data[12] = 0;
      data[14] = 0;
      rf95.send(data, 13);
    }
    else{
      Serial.println("Button on, so STOP!");
      uint8_t data[26] = "STOP in the name of love";
      data[24] = 0;
      data[26] = 1;
      rf95.send(data, 26);
    } 
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
      delay(20);
      sendFlag = false;
  }

  else{
  if (rf95.available())
  {
    sendFlag = true;
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);

  union{
    uint8_t tempBuff[3];
    float tempFloat;
  }u;
     float latitude;
     float longitude;
     float altitude;

for(int c=0; c<4; c++){
  u.tempBuff[c] = buf[c];
}
latitude = u.tempFloat;

for(int c=0; c<4; c++){
  u.tempBuff[c] = buf[c+5];
}
longitude = u.tempFloat;

for(int c=0; c<4; c++){
  u.tempBuff[c] = buf[c+10];
}
altitude = u.tempFloat;
      if(!buttonFlag) {
      RH_RF95::printBuffer("Received Raw Data: ", buf, len);
      Serial.println();
      Serial.print("Latitude: ");
      Serial.print(latitude);
      Serial.println((char)buf[4]);
      Serial.print("Longitude: ");
      Serial.print(longitude);
      Serial.println((char)buf[9]);
      Serial.print("Altitude: ");
      Serial.println(altitude);
      Serial.print("Fix: ");
      Serial.println(buf[14], DEC);
      Serial.print("Fix Quality: ");
      Serial.println(buf[15], DEC);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      }
      else{
        Serial.println((char*)buf);
      }
      
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  }
}

