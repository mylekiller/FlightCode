/*
   Notre Dame Rocket Team Roll Control Payload Ground Station Code V. 0.6.1
   Aidan McDonald, 2/8/17
   Kyle Miller, 2/2/17

   Most recent changes:
   Changed the packet-processing code to match the flight code (v. 1.0.4)

   To-dones:
   Basic radio transmit-receive architecture in place
   Reconfigured packet processing code to properly match the current flight code (v. 1.0.4)
   Added error messages for every current failure mode
   All data now goes to an LCD! (Wiring correctness TBD)

   To-dos:
   Write more involved display code? Lots of data we aren't displaying.
   As to the above, maybe a button installed which shifts through different data modes?
   For example, the LCD could display whether the payload and ground station are on the same page in terms of the three command flags
   Also, no room currently for GPS altitude data or payload flight state- display mode(s) for those?
   Determine how many signal LEDS will be used and for what purpose.
   Figure out what other buttons and switches we have (if any) and what they need to do


*/

#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//Must match RX's freq!
#define RF95_FREQ 915.0

// Single instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Instance of the LCD- for wiring see below.
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
/*
  The circuit:
  LCD RS pin to digital pin 12
  LCD Enable pin to digital pin 11
  LCD D4 pin to digital pin 5
  LCD D5 pin to digital pin 4
  LCD D6 pin to digital pin 3
  LCD D7 pin to digital pin 2
  LCD R/W pin to ground
  LCD VSS pin to ground
  LCD VCC pin to 5V
  10K resistor:
  ends to +5V and ground
  wiper to LCD VO pin (pin 3)

*/

bool sendFlag = false;
bool radioWorkingFlag = true;

//Constants for packet interpretation
const int LEFT = 0;
const int CENTER = 1;
const int RIGHT = 2;

const int waiting = 0;
const int launched = waiting + 1;
const int burnout = launched + 1;
const int falling = burnout + 1;
const int landed = falling + 1;
int flightState;

const int masterEnablePin = 0;
const int finOverridePin = 1;
const int servoPowerPin = 2; //Digital inputs for important flags- CHANGE THESE TO FIT REAL WORLD!!



void setup()
{
  pinMode(masterEnablePin, INPUT);
  pinMode(finOverridePin, INPUT);
  pinMode(servoPowerPin, INPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual radio reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  if (!rf95.init()) //Initialize the radio, set frequency, and set power level to max
    radioWorkingFlag = false;
  if (!rf95.setFrequency(RF95_FREQ))
    radioWorkingFlag = false;
  rf95.setTxPower(23, false);

  if (!radioWorkingFlag) { //If radio setup fails, display a message and wait-
    lcd.print("ERR: Radio Init"); //Without radio, the ground station is useless.
    lcd.setCursor(0, 1);
    lcd.print("Failure. Reset?");
    while (1);
  }

}

void loop()
{

  if (sendFlag) { //Transmit select commands to the payload

    uint8_t data[8];
    //Send everything in triplicate, for verification/reliability/data clarity purposes.
    data[0] = digitalRead(masterEnablePin);
    data[1] = digitalRead(masterEnablePin); //Enables data processing
    data[2] = digitalRead(masterEnablePin);
    data[3] = digitalRead(finOverridePin);
    data[4] = digitalRead(finOverridePin); //Forcibly sets fins to vertical
    data[5] = digitalRead(finOverridePin);
    data[6] = digitalRead(servoPowerPin);
    data[7] = digitalRead(servoPowerPin); //Controls servo power
    data[8] = digitalRead(servoPowerPin);

    rf95.send(data, 8);
    rf95.waitPacketSent();

    sendFlag = false;
  }

  else {
    /*
       Notes on incoming packet format:
        Constant "header" with seven elements.
        0 is the current flight state, while 1, 2, and 3 are the payload's
        perceived states of the master override flag, fin override flag, and servo power
        flag, respectively. (Those three flags are the data the ground station is
        transmitting). Then, element 4 is "42," just to serve as a packet validity
        confirmation number. Element 5 is a flag which states whether the GPS is on;
        element 6 states the current fin position and element 7 states whether the SD
        is working.

       Now for the special cases:
        If masterEnableFlag (#1) is false, that is the end of the transmission.
        If gpsOnFlag (#5) is true, the rest of the packet is gps data.
          Latitude (4) then direction (1); longitude (4) then direction (1);
          Altitude (4) then Fix (1) then Fix Quality (1) for a total length of 23
        If flight state (#0) is burnout (2), then element 8 is the "end roll flag,"
        which indicates whether the two-full-revolution process is complete or not.
          If the flag is false, the next/last piece of data is the number of
          completed revolutions (4), leading to a total length of 12.
          If the flag is true, the next/last piece of data is rotational velocity
          on the z-axis (2), leading to a total length of 10.

        Finally, if no other conditions apply, the rest of the packet is altitude data
        (4), leading to a total length of 11.
    */

    if (rf95.available())
    {
      sendFlag = true;
      lcd.clear(); //Clear the LCD since new data is incoming
      lcd.setCursor(0, 0); //Return cursor to the origin

      // Should be a message for us now
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) //Fills 'buf' with data, returns false if an error occurs
      {

        union { //Memory union used for the majority of data processing
          uint8_t tempBuff[3];
          float tempFloat;
        } u;

        flightState = buf[0]; //Update the flight state tracker

        if (buf[4] != 42) {
          lcd.print("ERR: Bad Header"); //42 in slot 4 is the header's backup validity check
        }

        else if (buf[1] == 0) { //buf[1] is the sensor "sleep mode" flag
          lcd.print("Waiting for data");
          lcd.setCursor(0, 1);
          lcd.print("recording auth.");
        }

        else if (buf[5] == true) { //buf[5] is the GPS enabled flag
          float latitude;
          float longitude;
          float altitude;
          char latDirect;
          char longDirect;
          int gpsFix;
          int gpsQuality;

          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 8];
          }
          latitude = u.tempFloat;
          latDirect = buf[12];
          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 13];
          }
          longitude = u.tempFloat;
          longDirect = buf[17];
          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 18];
          }
          altitude = u.tempFloat;
          gpsFix = buf[22];
          gpsQuality = buf[23];

          if (gpsFix == 0 || gpsQuality == 0) {
            lcd.print("No GPS Fix...");
          }
          else {
            lcd.print(latitude, 7);
            lcd.setCursor(11, 0);
            lcd.print(latDirect);
            lcd.setCursor(0, 1);
            lcd.print(longitude, 7);
            lcd.setCursor(11, 1);
            lcd.print(longDirect);
          }
        }

        else if (flightState == burnout) { //This indicates we are in the roll-controll phase

          if (buf[8] == 0) { //This flag being false indicates the payload is still trying to complete its two revolutions, so completed revolutions are tracked
            float completedRevs;
            for (int c = 0; c < 4; c++) {
              u.tempBuff[c] = buf[c + 9];
            }
            completedRevs = u.tempFloat;
            lcd.print("Revs Complete:");
            lcd.setCursor(0, 1);
            lcd.print(completedRevs, 7);
          }
          else { //If the flag is true, rotational velocity is tracked
            int rotationVel;
            union { //Since we're receiving an integer, we need a different-sized memory union to work with
              int tempInt;
              byte tempArray[1];
            } uInt;
            uInt.tempArray[0] = buf[9];
            uInt.tempArray[1] = buf[10];
            rotationVel = uInt.tempInt;
            lcd.print("Rotation Speed:");
            lcd.setCursor(0, 1);
            lcd.print(rotationVel);
            lcd.setCursor(6, 1);
            lcd.print("rad/s");
          }
        }

        else { //If none of the other conditions are met, altitude data is transmitted
          float altitude;
          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 8];
          }
          altitude = u.tempFloat;
          lcd.print("Altitude: ");
          lcd.setCursor(0, 1);
          lcd.print(altitude, 7);
          lcd.setCursor(10, 1);
          lcd.print("m");
        }
      }

      
      else
        lcd.print("ERR: Bad Packet");

    }
  }
}

