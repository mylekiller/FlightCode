/*
 * Notre Dame Rocket Team Roll Control Payload Master Code V. 0.8.0
 * Aidan McDonald, 1/12/17
*/

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool sendFlag = false;

void setup() 
{  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
 
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {

    while (1);
  }
  
  rf95.setTxPower(23, false);
  
}

void loop()
{

  if(sendFlag) {

      Serial.println("Button off, so proceed!");
    uint8_t data[14] = "All is well!";
      data[12] = 0;
      data[14] = 0;
      rf95.send(data, 13);
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

