/*
   Notre Dame Rocket Team Roll Control Payload Ground Station Code V. 1.2.0
   Aidan McDonald, 2/13/17
   Kyle Miller, 2/8/17

   Most recent changes:
   Ran comprehensive tests on code architecture and made necessary adjustments. Mostly
   communications-based, as well as a bit of packet structure tweaking and necessary
   pinout adjustments (avoid using 8, 4, or 3 for any external devices...)

   To-dones:
   Basic radio transmit-receive architecture in place
   Reconfigured packet processing code to properly match the current flight code (v. 1.2.0)
   Added error messages for every current failure mode
   All data now goes to an LCD! (Wiring correctness TBD)
   Tested for coherence, clarity, function

   To-dos:
   SET I/O PINS TO PROPER VALUES!!!!!!!
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3 //Assign nothing else to these digital pins!!!

//Must match RX's freq!
#define RF95_FREQ 915.0

// Single instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Instance of the LCD- for wiring see below.
LiquidCrystal lcd(13, 12, 11, 10, 9, 6);
/*
  The circuit:
  LCD RS pin to digital pin 13
  LCD Enable pin to digital pin 12
  LCD D4 pin to digital pin 11
  LCD D5 pin to digital pin 10
  LCD D6 pin to digital pin 9
  LCD D7 pin to digital pin 6
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
const int RIGHT = 0;
const int CENTER = 1;
const int LEFT = 2;

const int waiting = 0;
const int launched = waiting + 1;
const int burnout = launched + 1;
const int falling = burnout + 1;
const int landed = falling + 1;
int flightState;

//I/O pins
const int masterEnablePin = A5;
const int finOverridePin = A1;
//const int servoPowerPin = A2; //Digital inputs for important flags
const int buttonPin = A3;//Input to toggle display modes
const int packetLED = A2; //Output pins for display LEDs
const int mysteryLED = A0;
const int finLLED = 5;
const int finRLED = 20;
const int finCLED = 21;
//const int finOnLED = A2;

int packetTimeDelay = 5000; //Number of milliseconds the comms LED stays on for between valid packets
float lastRxTime = 0;

bool masterEnableFlag = false;
bool finOverrideFlag = false; //Flags for comparison w/ the payload's self-reporting
bool servoPowerFlag = false;

const int error = 0;
const int battery = 1; //Constants for the display switch-case statement
const int gps = 2;
const int burnoutA = 3;
const int burnoutB = 4;
const int baro = 5;


void setup()
{
  pinMode(masterEnablePin, INPUT);
  pinMode(finOverridePin, INPUT);
  //pinMode(servoPowerPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(packetLED, OUTPUT);
  //pinMode(finOverLED, OUTPUT);
  pinMode(finLLED, OUTPUT);
  pinMode(finCLED, OUTPUT);
  pinMode(finRLED, OUTPUT);
  //pinMode(finOnLED, OUTPUT);

  pinMode(13, OUTPUT);
  
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
  static int buttonState = 0; //Button-display-toggle-tracking variable; has 5 potential display states
  static bool buttonFlag = false; //Variable to prevent infinite cycline when the button is pushed

  if (millis() > lastRxTime + packetTimeDelay) //Update the communications LED
    digitalWrite(packetLED, LOW);
  else
    digitalWrite(packetLED, HIGH);

  if (digitalRead(buttonPin) == HIGH) {
    if (!buttonFlag) {
      buttonFlag = true; //Note that we've pushed the button, so that the counter only increments once
      buttonState++;
      if (buttonState == 5) {
        buttonState = 0;
      }
      lcd.setCursor(15, 1);
      lcd.print(buttonState);  //Update the button state on the LCD
      lcd.setCursor(0, 0); //Return cursor to the origin
    }
  }
  else {
    buttonFlag = false;
  }


  if (sendFlag) { //Transmit select commands to the payload

    uint8_t data[9];
delay(300); //For timing purposes
    digitalWrite(13, HIGH);
    //Send everything in triplicate, for verification/reliability/data clarity purposes.
    data[0] = digitalRead(masterEnablePin);
    data[1] = digitalRead(masterEnablePin); //Enables data processing
    data[2] = digitalRead(masterEnablePin);
    data[3] = digitalRead(finOverridePin);
    data[4] = digitalRead(finOverridePin); //Forcibly sets fins to vertical
    data[5] = digitalRead(finOverridePin);
    data[6] = digitalRead(masterEnablePin);
    data[7] = digitalRead(masterEnablePin); //Enables data processing
    data[8] = digitalRead(masterEnablePin);
    data[9] = 0; //Necessary end tag

    if (data[1])
      masterEnableFlag = true;
    if (data[4])
      finOverrideFlag = true; //Just like in the flight code, set flags true if the three overrides are ever pressed
   //if (data[7])
     // servoPowerFlag = true; //This allows us to compare what we've sent and what the payload does

    rf95.send(data, 9);
    rf95.waitPacketSent();

    digitalWrite(13, LOW);
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
          on the z-axis (4), leading to a total length of 12.

        Finally, if no other conditions apply, the rest of the packet is altitude data
        (4), leading to a total length of 11.
    */

    if (rf95.available())
    {
     digitalWrite(13, HIGH);
      sendFlag = true;
     
      lcd.begin(16, 2);
      
      lcd.clear(); //Clear the LCD since new data is incoming
      lcd.setCursor(15, 1);
      lcd.print(buttonState);  //Always print the display mode in the bottom-right corner of the screen
      lcd.setCursor(0, 0); //Return cursor to the origin

      // Should be a message for us now
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      int dataCase; //Variable for tracking what data comes in

      //Now instantiate a bunch of data variables; need to all be here because of the print section later on
      float batteryLevel;
      float latitude;
      float longitude;
      float altitude;
      char latDirect;
      char longDirect;
      int gpsFix;
      int gpsQuality;
      float completedRevs;
      float rotationVel;


      if (rf95.recv(buf, &len)) //Fills 'buf' with data, returns false if an error occurs
      {

        lastRxTime = millis();

        union { //Memory union used for the majority of data processing
          uint8_t tempBuff[3];
          float tempFloat;
        } u;

        flightState = buf[0]; //Update the flight state tracker

        if (buf[4] != 42) {
          lcd.print("ERR: Bad Header"); //42 in slot 4 is the header's backup validity check
          dataCase = error;
        }

        else if (buf[1] == 0 || flightState == waiting) { //buf[1] is the sensor "sleep mode" flag
          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 8];
          }
          batteryLevel = u.tempFloat;

          dataCase = battery;
        }

        else if (buf[5] == true) { //buf[5] is the GPS enabled flag

          dataCase = gps;

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

        }

        else if (flightState == burnout) { //This indicates we are in the roll-controll phase or that we want flight status

          if (buf[8] == 0) { //This flag being false indicates the payload is still trying to complete its two revolutions, so completed revolutions are tracked
            dataCase = burnoutA;

            for (int c = 0; c < 4; c++) {
              u.tempBuff[c] = buf[c + 9];
            }
            completedRevs = u.tempFloat;
          }
          else if (buf[8] == 1) { //If the flag is true, rotational velocity is tracked
            dataCase = burnoutB;
            
            for (int c = 0; c < 4; c++) {
              u.tempBuff[c] = buf[c + 9];
            }
            rotationVel = u.tempFloat;
          }
        }

        else { //If none of the other conditions are met, altitude data is transmitted
          dataCase = baro;
          for (int c = 0; c < 4; c++) {
            u.tempBuff[c] = buf[c + 8];
          }
          altitude = u.tempFloat;
        }

        //If the data is good, print stuff based on what button state we're in
        switch (buttonState) {
          case 0: //Case 0 displays the primary packet data
            switch (dataCase) {
              case error:
                //No need to display anything else in the error case
                break;
              case battery:
                lcd.print("Battery Level:");
                lcd.setCursor(0, 1);
                lcd.print(batteryLevel, 3);
                lcd.setCursor(8, 1);
                lcd.print("%");
                break;
              case gps:
                if (gpsFix == 0) {
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
                break;
              case burnoutA:
                lcd.print("Revs Complete:");
                lcd.setCursor(0, 1);
                lcd.print(completedRevs, 7);
                break;
              case burnoutB:
                lcd.print("Rotation Speed:");
                lcd.setCursor(0, 1);
                lcd.print(rotationVel, 5);
                lcd.setCursor(9, 1);
                lcd.print("rad/s");
                break;
              case baro:
                lcd.print("Altitude: ");
                lcd.setCursor(0, 1);
                lcd.print(altitude, 7);
                lcd.setCursor(10, 1);
                lcd.print("m");
                break;
                default:
                lcd.print("No Data");
                break;
            }
            break;

          case 1: //Case one checks whether the three critical flags are the same between the ground station and the payload
            if ((buf[1] == masterEnableFlag) && (buf[2] == finOverrideFlag) && (buf[3] == servoPowerFlag))
              lcd.print("Flags all good!");
            else {
              lcd.print("COMM ERR:");
              if (buf[1] != masterEnableFlag) {
                lcd.setCursor(12, 0);
                lcd.print("1:X");
              }
              if (buf[2] != finOverrideFlag) {
                lcd.setCursor(3, 1);
                lcd.print("2:X");
              }
              if (buf[3] != servoPowerFlag) {
                lcd.setCursor(12, 1);
                lcd.print("3:X");
              }
            }
            break;
          case 2: //Case 2 displays the flight state
            switch (flightState) {
              case 0:
                lcd.print("Waiting");
                break;
              case 1:
                lcd.print("Launched");
                break;
              case 2:
                lcd.print("Coasting");
                break;
              case 3:
                lcd.print("Falling");
                break;
              case 4:
                lcd.print("Landed");
                break;
            }
            break;
          case 3:
            if (buf[7]) { //Datum 7 is the SD-working flag
              lcd.print("SD Recording:");
              lcd.setCursor(0, 1);
              lcd.print("Working");
            }
            else {
              lcd.print("SD Recording:");
              lcd.setCursor(0, 1);
              lcd.print("Failure");
            }
            break;
          case 4: //Case 4 displays GPS altitude data if the GPS is enabled
            if (buf[5] == true) {
              if (gpsFix == 0 || gpsQuality == 0) {
                lcd.print("No GPS fix.");
              }
              else {
                lcd.print("GPS Altitude:");
                lcd.setCursor(0, 1);
                lcd.print(altitude, 7);
                lcd.setCursor(10, 1);
                lcd.print("m");
              }
            }
            else {
              lcd.print("No GPS data.");
            }
            break;
        }

        //Once data to the LCD have been displayed, update the LEDs with appropriate data
      /*  if (buf[2] == 1) //Confirmation of the Fin Override Flag
          digitalWrite(finOverLED, HIGH);
        else
          digitalWrite(finOverLED, LOW);*/

        /*if (buf[3] == 1) //Confirmation of the Servo Power Flag
          digitalWrite(finOnLED, HIGH);
        else
          digitalWrite(finOnLED, LOW);*/

        switch (buf[6]) { //Reports fin-position data
          case LEFT:
            digitalWrite(finLLED, HIGH);
            digitalWrite(finCLED, LOW);
            digitalWrite(finRLED, LOW);
            break;
          case CENTER:
            digitalWrite(finLLED, LOW);
            digitalWrite(finCLED, HIGH);
            digitalWrite(finRLED, LOW);
            break;
          case RIGHT:
            digitalWrite(finLLED, LOW);
            digitalWrite(finCLED, LOW);
            digitalWrite(finRLED, HIGH);
            break;
        }

      }
      else //If the data-retrieval statement returns FALSE, the packet is bad
        lcd.print("ERR: Bad Packet");

    digitalWrite(13, LOW);
    } //End of if(data received) section
  } //End of primary 'else' section (i.e. if we're not sending data)


}//End of void loop

