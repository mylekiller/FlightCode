/*
   Notre Dame Rocket Team Roll Control Payload Master Code V. 1.1.0
   Aidan McDonald, 2/8/17
   Kyle Miller, 2/2/17

   Most recent changes:
   Added radio transmission of battery charge levels
   Reconfigured the packet-processing code to use a variable address
   Revised how/when the code reads sensor data to ensure proper inputs
   

   To-dones:
    Basic switch-case structure
    Incorporation of Adafruit sensor code
    Added GPS functionality
    Integration of radio-transmission/reception code
    Datalogging capacity (untested)
    Running-average calcs for critical sensor values and Simpson's Rule integration of gyro data
    Multi-sensor verification of flight progress switch-case transitions
    Github Sync system working
    Packet transmission/reception over radio
    Roll control subroutine reconfigured for new servo mode

   To-dos:
    Revise and enhance the staging/thresholds, particularly burnout/apogee accel values
    Reconfigure the SD datalogging section to allow for easy spreadsheet conversion
    Test List:
      Servo Control
      Flight Staging
      Etc., etc.

*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h> //Radio library
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h> //Gyro library
#include <Adafruit_BMP085_U.h> //Baro library
#include <Adafruit_LSM303_U.h> //Accel library

//Assign unique IDs to each sensor (arbitrary?)
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

#define RFM95_CS 8 //Pinouts for the Feather LoRa module
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0 // Must match RX's freq; in this case must be either 915 or 868 MHz

// Single instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial); //Construct instance of the GPS object

//Constants for SD Card communication
const int cardSelectPin = 10; //Note: for some incredibly stupid reason, you must initialize on pin 8 first for proper functionality
const int stupidSDPin = 8;
File dataLog; //File to log flight data
int accelData[3];
int gyroData[3];
float baroData[3]; //Char buffers for storing sensor data
unsigned long timeData[2];

int accelZBuffer[5] = {0, 0, 0, 0, 0};
float baroAltBuffer[5] = {0, 0, 0, 0, 0};
int gyroZBuffer[7] = {0, 0, 0, 0, 0, 0, 0}; //The other values are arbitrary; this one is used in Simpsons Rule, so it MUST BE ODD!
unsigned long timeBuffer[7] = {0, 0, 0, 0, 0, 0, 0}; //Must equal the size of gyroZBuffer; used for the same calculations.

byte accelAverage;
byte baroAverage;
unsigned long timeStepAverage; //Averages of the above buffer values

float gpsLatitude = 0;
float gpsLongitude = 0;
float gpsAltitude = -1;
char gpsLatDirect = 'A';
char gpsLonDirect = 'B';
int gpsFix = -1;
int gpsFixQuality = -1; //Variables to store GPS data between the SD and radio functions

//Constants/values for sensor calibration
const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; //Default value of 1013.25 hPa, needs calibration!
int startAlt; //variable to note starting altitude
long startTime = 0; //variable to note flight time
long flightTime = 0;

//Constants for servo pin output
const int controlPin = 0; //"control" is the pin which is pulsed to trigger the servo to move
const int statePinA = 1; //The state of pins A and B determines which of four motions the servo takes when the control pin is pulsed.
const int statePinB = 2; // If B is low, move 1 position, if B is high, move two positions. High A is negative (counterclockwise?), low A is positive (clockwise?)
const int LEFT = 0;
const int CENTER = 1;
const int RIGHT = 2;
int finPosition = CENTER; //Since the servo moves based on increments and not absolute positions, these constants and variable are needed to track the fins' position.

bool servoPowerFlag = false;
const int servoPowerPin = 3; //Flag and pin to power on/off the servo with a transistor

const int batteryPin = 9; //Built-in power tracking pin
float batteryLevel = 100; //Battery power, in percentage

//Constants for flight staging
const int waiting = 0;
const int launched = waiting + 1;
const int burnout = launched + 1;
const int falling = burnout + 1;
const int landed = falling + 1;
int flightState; //Variable for switch case

const int LIFTOFF_ACCEL_THRESHOLD = 38;
const int BURNOUT_ACCEL_THRESHOLD = 12; //Constants for flight staging calculation
const int BURNOUT_BARO_THRESHOLD = 610; //Accel values are in m/s^2, baro values are in m, and time is in milliseconds
const int BURNOUT_TIME_THRESHOLD = 6000;
const int APOGEE_ACCEL_THRESHOLD = 23;
const int APOGEE_TIME_THRESHOLD = 20000;
const int LANDED_BARO_THRESHOLD = 6;
const int MIN_ROLL_THRESHOLD = 0;

const int GPS_BARO_THRESHOLD = 200; //600 feet/200m is when the recovery system deploys
const int MAX_PACKET_SIZE = 23; //The one and only constant you will ever need to update for packet-related reasons

bool startRollFlag = false;
bool endRollFlag = false; //Flags for tracking roll/counter-roll progress

float rotationCounter = 0; //Variable for tracking rotation, in revolutions

bool sendFlag = true; //Flag for toggling send/receive mode; makes 2-way communication much easier
bool radioWorkingFlag = true; //Flag to determine if radio initialized properly. If not, then it continues on without comms
bool dataFlag = false; //Since SD saving and radio transmission occur in two different functions, the SD routine uses this flag to tell the radio routine if data is ready for transmission
bool gpsOnFlag = false; //Flag to determine whether GPS is currently operating/enabled
bool sdWorkingFlag = true; //Flag to note if SD initializes properly, since that has been a problem in the past.

bool masterEnableFlag = false; //Flag that puts the Arduino in/out of "sleep mode."
bool finOverrideFlag = false; //Flag that acts as a "big red button" to stop the arduino's roll-control.

const int timeDelay = 333; //In milliseconds
unsigned long rxTime = 0; //Variables for the radio transmitter double-pulse backup code


void setup() {
  pinMode(RFM95_RST, OUTPUT); //Initialize radio; Reset pin must be high for normal function
  digitalWrite(RFM95_RST, HIGH);

  pinMode(controlPin, OUTPUT);
  pinMode(statePinA, OUTPUT);
  pinMode(statePinB, OUTPUT);
  pinMode(servoPowerPin, OUTPUT);
  pinMode(batteryPin, INPUT);

  accel.begin();
  bmp.begin();
  gyro.enableAutoRange(true);
  gyro.begin();//Initialize the sensors

  SD.begin(stupidSDPin);
  if (!SD.begin(cardSelectPin)) //Initialize the SD card; set a flag if the initialization fails
    sdWorkingFlag = false;

  digitalWrite(controlPin, LOW); //Set the servo control pin to low to make sure it doesn't pulse accidentally
  digitalWrite(servoPowerPin, servoPowerFlag); //Make sure the servo is powered off

  //Manual radio reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) { //Confirm radio was initialized properly and set frequency; note if a failure occurs
    radioWorkingFlag = false;
    masterEnableFlag = true; //If the radio fails to work, self-initialize data processing
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    radioWorkingFlag = false;
    masterEnableFlag = true; //If the radio fails to work, self-initialize data processing
  }
  rf95.setTxPower(23, false); //Set transmitter power to maximum (23 dBm); not sure what the bool value is for but all the examples used it


  sensors_event_t event;
  bmp.getEvent(&event);
  startAlt = bmp.pressureToAltitude(seaLevelPressure, event.pressure); //Set starting altitude for later reference


  flightState = waiting; //Initialize switch variable
}



void loop() {

  /* Get a new sensor event */
  sensors_event_t mainEvent;

  if (masterEnableFlag) //Only save/buffer data if we're actively sensing data
  {
    Record_Data(mainEvent);//Save sensor data (accel, baro, time, GPS) to the SD card
    BufferUpdate();
  }

  if (gpsOnFlag)
    char c = GPS.read(); //Must call GPS.read() at some point for data transmission to occur

  if (startTime != 0)
    flightTime = millis() - startTime; //Variable tracking post-launch time for backup flight staging

  if (radioWorkingFlag)
    Radio_Transmit(); //Transmit/receive select data


  switch (flightState) { //Switch statement for entire rocket flight
    /*
       Switch conditions have been greatly improved (running averages with multiple sensors)
       However, I do want to look at the Accel conditions again, maybe mess with the time as well?
       Getting this part right is really, really important
    */

    case waiting:
      if (accelAverage > LIFTOFF_ACCEL_THRESHOLD && masterEnableFlag)
      {
        flightState = launched;
        startTime = millis();
      }
      break;


    case launched:
      if (abs(accelAverage) < BURNOUT_ACCEL_THRESHOLD || baroAverage > BURNOUT_BARO_THRESHOLD || flightTime > BURNOUT_TIME_THRESHOLD)
      {
        flightState = burnout;
      }
      break;


    case burnout:

      Roll_Control(mainEvent);

      if (accelAverage > APOGEE_ACCEL_THRESHOLD || flightTime > APOGEE_TIME_THRESHOLD) //Add a baro test here?
      {
        flightState = falling;
      }
      break;


    case falling:
      if (baroAverage < LANDED_BARO_THRESHOLD)
      {
        flightState = landed;
      }

      if (baroAverage < GPS_BARO_THRESHOLD) //Enable GPS if we're low enough
      {
        GPS.begin(9600); //Initialize GPS; define output set and data rate
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        gpsOnFlag = true;
      }

      break;

    case landed:

      break;
  }
}

void Roll_Control(sensors_event_t event) {

  if (!startRollFlag) { //For roll initialization, cant fins in the direction of current roll
    startRollFlag = true;
    if (event.gyro.z > 0) {
      Set_Servo(finPosition, LEFT); //This assumes positive gyro values are clockwise. CONFIRM THIS!!!
    }
    else {
      Set_Servo(finPosition, RIGHT);
    }
  }
  else if (!endRollFlag) { //Wait until two revolutions have ben completed to begin counter-roll

    if (rotationCounter > 2) {
      Set_Servo(finPosition, abs(finPosition - 2)); //This will output LEFT if finPosition = RIGHT, and vice-versa
      endRollFlag = true;
    }
  }
  else { //At this point, continue to make adjustments to prevent roll
    if (abs(event.gyro.z) < MIN_ROLL_THRESHOLD)
    {
      Set_Servo(finPosition, CENTER);
    }
    else if (event.gyro.z > 0) //Again, this assumes positive gyro values are clockwise.
    {
      Set_Servo(finPosition, RIGHT);
    }
    else if (event.gyro.z < 0)
    {
      Set_Servo(finPosition, LEFT);
    }
  }

}


void Set_Servo(int current, int goal) { //Subroutine which takes current position and intended position and turns those into a servo output.
  //(I think this is more efficient/adaptive than writing the same thing out multiple times in Roll_Control()?

  if (current != goal) { //Only run the fin-setting pulse and delay if motion is necessary
    if (abs(current - goal) == 2)
      digitalWrite(statePinB, HIGH); //If moving from right to left, or vice-versa, pin B needs to be high.
    else
      digitalWrite(statePinB, LOW);

    if (current - goal > 0) //This assumes positive servo direction is clockwise. CONFIRM THIS IS THE CASE!!!
      digitalWrite(statePinA, LOW);
    else
      digitalWrite(statePinB, HIGH);

    digitalWrite(controlPin, HIGH); //Once the output pins have been set properly, pulse the control pin to update the servo.
    delay(2);
    digitalWrite(controlPin, LOW);

    finPosition = goal; //Update the fin state tracking variable
  }

}


void Record_Data(sensors_event_t event) { //Subroutine for saving sensor data to the SD card

  //Fill data buffers with sensor readings
  gyro.getEvent(&event);
  gyroData[0] = event.gyro.x;
  gyroData[1] = event.gyro.y;
  gyroData[2] = event.gyro.z;
  
  accel.getEvent(&event);
  accelData[0] = event.acceleration.x;
  accelData[1] = event.acceleration.y;
  accelData[2] = event.acceleration.z;

  bmp.getEvent(&event);
  float temperature;
  bmp.getTemperature(&temperature);
  baroData[0] = event.pressure;
  baroData[1] = temperature;
  baroData[2] = bmp.pressureToAltitude(seaLevelPressure, event.pressure);

  batteryLevel = analogRead(batteryPin);
  batteryLevel *= 0.1534598214;  //Read the battery level and convert to a percentage
  
  timeData[0] = millis();
  timeData[1] = millis() - startTime;

  if (!gpsOnFlag) {
    dataFlag = true; //If the GPS isn't on yet, set the dataFlag to true automatically
  }


  if (GPS.newNMEAreceived() && gpsOnFlag) { //Update GPS data if an update is available
    if (!GPS.parse(GPS.lastNMEA())) // this sets the newNMEAreceived() flag to false; prevents double(+)-update loop
      return;
    dataFlag = true; //Set flag to tell radio new data is ready
    gpsLatitude = GPS.latitude;
    gpsLongitude = GPS.longitude;
    gpsAltitude = GPS.altitude;
    gpsLatDirect = GPS.lat;
    gpsLonDirect = GPS.lon;
    gpsFix = GPS.fix;
    gpsFixQuality = GPS.fixquality;
  }

  if (sdWorkingFlag) { //Only actually work with the SD card if the SD card is working
    dataLog = SD.open("flight_data.txt", FILE_WRITE); //Open the file flight_data.txt in write mode

    if (dataLog) { //log data only if the file opened properly
      dataLog.println(); //Start a new line

      for (int c = 0; c < 1; c++) {
        dataLog.print(timeData[c]);
        dataLog.print(", "); //Separate data entries by a comma and a space
      }

      for (int c = 0; c < 2; c++) {
        dataLog.print(accelData[c]);
        dataLog.print(", ");
      }

      for (int c = 0; c < 2; c++) {
        dataLog.print(gyroData[c]);
        dataLog.print(", ");
      }

      for (int c = 0; c < 2; c++) {
        dataLog.print(baroData[c]);
        dataLog.print(", ");
      }

      dataLog.print(gpsLatitude);
      dataLog.print(gpsLatDirect);
      dataLog.print(", ");
      dataLog.print(gpsLongitude);
      dataLog.print(gpsLonDirect);
      dataLog.print(", ");
      dataLog.print(gpsAltitude);
      dataLog.print(", ");
      dataLog.print(gpsFix);
      dataLog.print(", ");
      dataLog.print(gpsFixQuality);

      dataLog.close(); //Close the file
    }

  }

}



void Radio_Transmit(void) {

  if (dataFlag && sendFlag) //Send data mode, only if GPS is ready
  {
    dataFlag = false; //Only turn off this flag if Arduino is actively sending data
    sendFlag = false; //Once we send data, wait for data to be received

    uint8_t radioPacket[MAX_PACKET_SIZE]; //Buffer of bytes for radio transmission
    int packetSize; //Depending on the flight state, the actual packet size may change
    int packAddr = 0; //Variable for counting address states

    radioPacket[packAddr] = flightState; //Start every packet with the current flight staging (lets the receiver know what data is going to come at the end of the packet)
    packAddr++;
    radioPacket[packAddr] = masterEnableFlag;
    packAddr++;
    radioPacket[packAddr] = finOverrideFlag; //Report back on the perceived state of affairs with the critical flags, in order for adjustments to be made if necessary.
    packAddr++;
    radioPacket[packAddr] = servoPowerFlag;
    packAddr++;
    radioPacket[packAddr] = 42; //Answer to life, the universe, and everything. Used to confirm packet validity.
    packAddr++;

    radioPacket[packAddr] = gpsOnFlag;
    packAddr++;
    radioPacket[packAddr] = finPosition; //The next items in each packet report the outputs to the servo and whether we are running GPS.
    packAddr++;
    radioPacket[packAddr] = sdWorkingFlag;

    union { //Float-to-byte-string converter, needed for floating-point sensor data
      float tempFloat;
      byte tempArray[3];
    } u;

    if (!masterEnableFlag) {
      //If sensors aren't enabled yet, send battery data
      u.tempFloat = batteryLevel;
      for (int c = 0; c < 4; c++) {
        packAddr++;
        radioPacket[packAddr] = u.tempArray[c];
      }
    }

    else if (gpsOnFlag)
    {
      packAddr++;
      u.tempFloat = gpsLatitude;
      for (int c = 0; c < 4; c++) {
        radioPacket[packAddr] = u.tempArray[c];
        packAddr++;
      }
      radioPacket[packAddr] = gpsLatDirect; //N or S
      packAddr++;
      u.tempFloat = gpsLongitude;
      for (int c = 0; c < 4; c++) {
        radioPacket[packAddr] = u.tempArray[c];
        packAddr++;
      }
      radioPacket[packAddr] = gpsLonDirect; //E or W
      packAddr++;
      u.tempFloat = gpsAltitude;
      for (int c = 0; c < 4; c++) {
        radioPacket[packAddr] = u.tempArray[c];
        packAddr++;
      }
      radioPacket[packAddr] = gpsFix;
      packAddr++;
      radioPacket[packAddr] = gpsFixQuality; //Useful for determining valid/invalid GPS data; if fix or quality are 0 the data isn't reliable
    }

    else if (flightState = burnout) {

      packAddr++;
      radioPacket[packAddr] = endRollFlag; //If we're in burnout, send the "end roll flag." It determines whether we send the running tally (float) or the current rotation speed (int)

      if (endRollFlag) {
        union { //Since we're sending an integer, we need a different-sized memory union to work with
          int tempInt;
          byte tempArray[1];
        } uInt;

        uInt.tempInt = gyroData[2];
        packAddr++;
        radioPacket[packAddr] = uInt.tempArray[0];
        packAddr++;
        radioPacket[packAddr] = uInt.tempArray[1];
      }

      else {
        u.tempFloat = rotationCounter;
        for (int c = 0; c < 4; c++) {
          packAddr++;
          radioPacket[packAddr] = u.tempArray[c];
        }
      }

    }

    else { //If we're not in the burnout phase and haven't turned on the GPS, then send current altitude
      u.tempFloat = baroData[2];
      for (int c = 0; c < 4; c++) {
        packAddr++;
        radioPacket[packAddr] = u.tempArray[c];
      }
    }

    packetSize = packAddr; //Update the packet size to match the most recently used address

    rf95.send((uint8_t *)radioPacket, packetSize); //Once packet has been constructed, transmit
    rf95.waitPacketSent(); //Short, necessary wait for packet transmission to complete

    rxTime = millis(); //Note the most recent packet-sent time
  }

  else { //Receive data mode

    if (rf95.available())
    {
      sendFlag = true; //If data is received, return to data-send mode
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf); //Variables to contain received message
      if (rf95.recv(buf, &len)) //If message is intact; this call also fills buf and len with data
      {

        //Three very important flags from the ground station, sent in triplicate.
        //masterEnableFlag enables the flight sensors to begin liftoff detection.
        //finOverrideFlag acts as a master interrupt to halt in-flight fin rotation.
        //servoPowerFlag controls the transistor powering the servo (so that it isn't enabled until the fin guide is in place)

        if (buf[0] || buf[1] || buf[2])
          masterEnableFlag = true;
        else
          masterEnableFlag = false;

        if (buf[3] || buf[4] || buf[5])
          finOverrideFlag = true;
        else
          finOverrideFlag = false;

        if (buf[6] || buf[7] || buf[8]) {
          servoPowerFlag = true;
          digitalWrite(servoPowerPin, servoPowerFlag);
        }

      }
    }

    else if ((millis() - rxTime) > timeDelay)
      sendFlag = true;

  }
}



void BufferUpdate() { //Function to keep a running buffer of select sensor values needed for data averaging (or numerical integration)

  accelAverage = 0;
  baroAverage = 0;

  for (int c = 0; c < 4; c++) {
    accelZBuffer[4 - c] = accelZBuffer[3 - c];
    baroAltBuffer[4 - c] = baroAltBuffer[3 - c];
  }
  accelZBuffer[0] = accelData[2];
  baroAltBuffer[0] = baroData[2]; //This always runs after Record_Data, so the data buffers should always be up-to-date

  for (int c = 0; c < 5; c++) {
    accelAverage = accelAverage + accelZBuffer[c];
    baroAverage = baroAverage + baroAltBuffer[c];
  }
  accelAverage /= 5;
  baroAverage /= 5;
  baroAverage = baroAverage - startAlt; //Have to compare the average to the ground, not sea level

  if (flightState == burnout && !endRollFlag) { //Keep track of rotations once we transition to the burnout stage
    /*
       This section functions stepwise. Once the two buffers are full, the algorithm performs numerical
       integration, increments rollCounter, and flushes the buffers before starting over.
    */
    if (timeBuffer[6] < 1) { //If the buffers are not yet full; avoiding == 0 because it's an unsigned long

      for (int c = 0; c < 6; c++) {
        timeBuffer[6 - c] = timeBuffer[5 - c];
        gyroZBuffer[6 - c] = gyroZBuffer[5 - c];
      }
      timeBuffer[0] = millis();
      gyroZBuffer[0] = gyroData[2]; //Again, this buffer should be up-to-date

    }

    else { //Once the buffers are full, perform the integration!

      float deltaT = (timeBuffer[6] - timeBuffer[0]) / 18; //Note: the average is technically /6, but since Simpson's Rule adds a /3 anyways, it's just more efficient to combine them
      float simpsonSum = gyroZBuffer[0] + gyroZBuffer[6];
      simpsonSum = simpsonSum + 4 * (gyroZBuffer[1] + gyroZBuffer[3] + gyroZBuffer[5]);
      simpsonSum = simpsonSum + 2 * (gyroZBuffer[2] + gyroZBuffer[4]);
      simpsonSum = simpsonSum * deltaT; //Hooray for numerical integration! Should look into whether there's a more efficient way to perform Simpson's rule here.

      rotationCounter = rotationCounter + (simpsonSum / 6.28318531); //Convert from radians to revolutions and increment the counter!

      for (int c = 0; c < 7; c++) {
        timeBuffer[c] = 0;
        gyroZBuffer[c] = 0; //Don't forget to clear the buffers for the next step!
      }

    }

  }

}


