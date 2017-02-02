/*
 * Notre Dame Rocket Team Roll Control Payload Master Code V. 0.91
 * Aidan McDonald, 2/1/17
 * Kyle Miller, 2/2/17
 * 
 * Most recent changes:
 * Moved the GPS enable code again, to <600 ft post-apogee
 * Incorporated receiving data from the ground station
    * Added two master flags to control sensor enabling and fin control disabling
 * Revised the data transmission section to send altitude data when gps/gyro data isn't being sent
 * Changed Pin Variables to match what the inputs to the servo will be 
 * 
 * To-dones:
  * Basic switch-case structure
  * Incorporation of Adafruit sensor code
  * Added GPS functionality
  * Integration of radio-transmission/reception code
  * Datalogging capacity (untested)
  * Running-average calcs for critical sensor values and Simpson's Rule integration of gyro data
  * Multi-sensor verification of flight progress switch-case transitions (untested)
  * Github Sync system working
  * Packet transmission/reception over radio (ADD THIS TO GROUND STATION CODE TOO!!)
 * 
 * To-dos:
  * Reconfigure the servo control for the new control scheme!!!!!!!
  * Revise and enhance the staging/thresholds, particularly burnout/apogee accel values
  * Reconfigure the SD datalogging section to allow for easy spreadsheet conversion
  * TEST EVERYTHING
  * NO REALLY, EVERYTHING
   *  Ada Sensors
   *  SD Saving
   *  Servo Control
   *  Flight Staging
   *  Etc., etc.
 * 
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
const int cardSelectPin = 53; //Standard for ATMega; other SPI comms pins are 50-52
File dataLog; //File to log flight data
int accelData[3];
int gyroData[3];
float baroData[3]; //Char buffers for storing sensor data
unsigned long timeData[2];

int accelZBuffer[5] = {0,0,0,0,0};
float baroAltBuffer[5] = {0,0,0,0,0};
int gyroZBuffer[7] = {0,0,0,0,0,0,0}; //The other values are arbitrary; this one is used in Simpsons Rule, so it MUST BE ODD!
unsigned long timeBuffer[7] = {0,0,0,0,0,0,0}; //Must equal the size of gyroZBuffer; used for the same calculations.

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
const int controlPin = 0; //"control" is the master switch- must be 1 to enable servo, pulse this pin to trigger a move 
const int inputA = 1; //The inputA and inputB pins control which way the fins will increment when the move is triggered
const int inputB = 2; //inputA = 0, inputB = 0 is +4 Degrees, inputA = 1, inputB = 0 is -4 Degrees, inputA = 0, inputB = 1 is +8 Degrees,
					  // inputA = 1, inputB = 1 is -8 Degrees 

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

bool startRollFlag = false;
bool endRollFlag = false; //Flags for tracking roll/counter-roll progress
bool finState = false; //Variable for tracking current fin position
bool servoOnFlag;
bool servoHomeFlag; //Two flags to keep track of the other two digital outputs to the servo. ALWAYS CHANGE THESE WHEN CHANGING SERVO OUTPUTS!

float rotationCounter = 0; //Variable for tracking rotation, in revolutions

bool sendFlag = true; //Flag for toggling send/receive mode; makes 2-way communication much easier
bool radioWorkingFlag = true; //Flag to determine if radio initialized properly. If not, then it continues on without comms
bool dataFlag = false; //Since SD saving and radio transmission occur in two different functions, the SD routine uses this flag to tell the radio routine if data is ready for transmission
bool gpsOnFlag = false; //Flag to determine whether GPS is currently operating/enabled

bool masterEnableFlag = false; //Flag that puts the Arduino in/out of "sleep mode."
bool finOverrideFlag = false; //Flag that acts as a "big red button" to stop the arduino's roll-control.


void setup() {
  pinMode(RFM95_RST, OUTPUT); //Initialize radio; Reset pin must be high for normal function
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(controlPin, OUTPUT);
  pinMode(inputA, OUTPUT);
  pinMode(inputB, OUTPUT);

  accel.begin();
  bmp.begin();
  gyro.enableAutoRange(true);
  gyro.begin(); 
  SD.begin(cardSelectPin);//Initialize the sensors and SD card
  
  digitalWrite(controlPin, HIGH); //Enable servo
  servoOnFlag = true;
  digitalWrite(inputA, HIGH); //Set servo to "home" position
  servoHomeFlag = true;

  //Manual radio reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if(!rf95.init()) { //Confirm radio was initialized properly and set frequency; note if a failure occurs
    radioWorkingFlag = false;
  }
  if(!rf95.setFrequency(RF95_FREQ)) {
    radioWorkingFlag = false;
  }
  rf95.setTxPower(23, false); //Set transmitter power to maximum (23 dBm); not sure what the bool value is for but all the examples used it


  sensors_event_t event;
  bmp.getEvent(&event);
  startAlt = bmp.pressureToAltitude(seaLevelPressure, event.pressure); //Set starting altitude for later reference
  

  flightState = waiting; //Initialize switch variable
}



void loop() {

  if(masterEnableFlag) //Only save/buffer data if we're actively sensing data
  {
/* Get a new sensor event */ 
  sensors_event_t mainEvent;
  gyro.getEvent(&mainEvent);
  accel.getEvent(&mainEvent); // Assume Z-axis is vertical for accel/gyro?
  bmp.getEvent(&mainEvent);
    
  Record_Data(mainEvent);//Save sensor data (accel, baro, time, GPS) to the SD card
  BufferUpdate(mainEvent);
  }
  
  if(gpsOnFlag)
  char c = GPS.read(); //Must call GPS.read() at some point for data transmission to occur

  if(startTime != 0)
  flightTime = millis() - startTime; //Timer backup variable
  
  if(radioWorkingFlag) {
  Radio_Transmit(); //Transmit/receive select data
  }
 
switch(flightState) { //Switch statement for entire rocket flight
  /* 
   * Switch conditions have been greatly improved (running averages with multiple sensors)
   * However, I do want to look at the Accel conditions again, maybe mess with the time as well?
   * Getting this part right is really, really important
   */

case waiting:
if(accelAverage > LIFTOFF_ACCEL_THRESHOLD && masterEnableFlag)
{
  flightState = launched;
  startTime = millis();
}
break;


case launched:
if(abs(accelAverage) < BURNOUT_ACCEL_THRESHOLD || baroAverage > BURNOUT_BARO_THRESHOLD || flightTime > BURNOUT_TIME_THRESHOLD)
{
  flightState = burnout;
}
break;


case burnout:
Roll_Control(mainEvent);
if(accelAverage > APOGEE_ACCEL_THRESHOLD || flightTime > APOGEE_TIME_THRESHOLD) //Add a baro test here?
{
  flightState = falling;
}
break;


case falling:
if(baroAverage < LANDED_BARO_THRESHOLD)
{
  flightState = landed;
}

if(baroAverage < GPS_BARO_THRESHOLD) //Enable GPS if we're low enough
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

  if(!startRollFlag) { //For roll initialization, cant fins in the direction of current roll
    digitalWrite(inputA, LOW);
    servoHomeFlag = false;
    startRollFlag = true;
    if(event.gyro.z > 0) {
      digitalWrite(inputB, HIGH);
      finState = true;
    }
    else {
      digitalWrite(inputB, LOW); 
      finState = false;
    }
  }
  else if(!endRollFlag) { //Wait until two revolutions have been completed to begin counter-roll
    
    if(rotationCounter > 2) {
      finState = !finState;
      digitalWrite(inputB, finState);
      endRollFlag = true;
    }
  }
  else { //At this point, continue to make adjustments to prevent roll
    if(abs(event.gyro.z) < MIN_ROLL_THRESHOLD)
    {
      digitalWrite(inputA, HIGH);
      servoHomeFlag = true;
    }
    else if(event.gyro.z > 0)
      {
      digitalWrite(inputA, LOW);
      servoHomeFlag = false;
      digitalWrite(inputB, LOW);
      finState = false;
      }
    else if(event.gyro.z < 0)
      {
      digitalWrite(inputA, LOW);
      servoHomeFlag = false;
      digitalWrite(inputB, HIGH);
      finState = true;
      }
  }
  
}



void Record_Data(sensors_event_t event){ //Subroutine for saving sensor data to the SD card

float temperature;
bmp.getTemperature(&temperature);

  //Fill data buffers with sensor readings
  accelData[0] = event.acceleration.x;
  accelData[1] = event.acceleration.y;
  accelData[2] = event.acceleration.z;
  gyroData[0] = event.gyro.x;
  gyroData[1] = event.gyro.y;
  gyroData[2] = event.gyro.z;
  baroData[0] = event.pressure;
  baroData[1] = temperature;
  baroData[2] = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
  timeData[0] = millis();
  timeData[1] = millis() - startTime;

if(!gpsOnFlag) {
  dataFlag = true; //If the GPS isn't on yet, set the dataFlag to true automatically
}
  

if(GPS.newNMEAreceived() && gpsOnFlag) { //Update GPS data if an update is available
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


dataLog = SD.open("flight_data.txt", FILE_WRITE); //Open the file flight_data.txt in write mode

if (dataLog) { //log data only if the file opened properly
  dataLog.println(); //Start a new line
  
for(int c = 0; c < 1; c++){
dataLog.print(timeData[c]);
dataLog.print(", "); //Separate data entries by a comma and a space
}

for(int c = 0; c < 2; c++){
dataLog.print(accelData[c]);
dataLog.print(", ");
}

for(int c = 0; c < 2; c++){
dataLog.print(gyroData[c]);
dataLog.print(", ");
}

for(int c = 0; c < 2; c++){
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



void Radio_Transmit(void) {

if(dataFlag && sendFlag) //Send data mode, only if GPS is ready
  {
      dataFlag = false; //Only turn off this flag if Arduino is actively sending data
      sendFlag = false; //Once we send data, wait for data to be received
      
      uint8_t radioPacket[21]; //Buffer of bytes for radio transmission
      int packetSize = 21; //Depending on the flight state, the actual packet size may change

      radioPacket[0] = flightState; //Start every packet with the current flight staging (lets the receiver know what data is going to come at the end of the packet)
      radioPacket[1] = masterEnableFlag;
      radioPacket[2] = finOverrideFlag; //Report back on the perceived state of affairs with the critical flags, in order for adjustments to be made if necessary.
      radioPacket[3] = 42; //Answer to life, the universe, and everything. Used to confirm packet validity.
      
      radioPacket[4] = gpsOnFlag;
      radioPacket[4] = servoOnFlag;
      radioPacket[5] = servoHomeFlag; //The next items in each packet report the outputs to the servo and whether we are running GPS.
      radioPacket[6] = finState;

union{ //Float-to-byte-string converter, needed for GPS data
  float tempFloat;
  byte tempArray[3];
} u;

if(!masterEnableFlag) {
  packetSize = 6; //If sensors aren't enabled yet, only send the above data
}

else if(gpsOnFlag)
{
u.tempFloat = gpsLatitude;
for(int c=0; c<4; c++){
  radioPacket[c+7] = u.tempArray[c];
}
radioPacket[4] = gpsLatDirect; //N or S
u.tempFloat = gpsLongitude;
for(int c=0; c<4; c++){
  radioPacket[(c+11)] = u.tempArray[c];
}
radioPacket[9] = gpsLonDirect; //E or W
u.tempFloat = gpsAltitude;
for(int c=0; c<4; c++){
  radioPacket[(c+16)] = u.tempArray[c];
}
radioPacket[20] = gpsFix;
radioPacket[21] = gpsFixQuality; //Useful for determining valid/invalid GPS data; if fix or quality are 0 the data isn't reliable
}

else if(flightState = burnout) {

  radioPacket[7] = endRollFlag; //If we're in burnout, send the "end roll flag." It determines whether we send the running tally (float) or the current rotation speed (int)

if(endRollFlag) {
union { //Since we're sending an integer, we need a different-sized memory union to work with
  int tempInt;
  byte tempArray[1];
} uInt;

uInt.tempInt = gyroData[2]; //Remember: THIS ASSUMES Z-AXIS IS VERTICAL!!!
radioPacket[8] = uInt.tempArray[0];
radioPacket[9] = uInt.tempArray[1]; 
packetSize = 9;
}

else {
  u.tempFloat = rotationCounter;
for(int c=0; c<4; c++){
  radioPacket[(c+8)] = u.tempArray[c];
}
packetSize = 11;
}

}

else { //If we're not in the burnout phase and haven't turned on the GPS, then send current altitude
u.tempFloat = baroData[2];
for(int c=0; c<4; c++){
  radioPacket[(c+7)] = u.tempArray[c];
}
packetSize = 10;
}


rf95.send((uint8_t *)radioPacket, packetSize); //Once packet has been constructed, transmit
rf95.waitPacketSent(); //Short, necessary wait for packet transmission to complete
}

 else { //Receive data mode

    if (rf95.available())
  {
    sendFlag = true; //If data is received, return to data-send mode
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf); //Variables to contain received message
    if (rf95.recv(buf, &len)) //If message is intact; this call also fills buf and len with data
   {

    //Two very important flags from the ground station, sent in triplicate.
    //masterEnableFlag enables the flight sensors to begin liftoff detection; finOverrideFlag acts as a master interrupt to halt in-flight fin rotation.
    
    if(buf[0] || buf[1] || buf[2])
        masterEnableFlag = true;
      else
        masterEnableFlag = false;

     if(buf[3] || buf[4] || buf[5])
        finOverrideFlag = true;
     else
        finOverrideFlag = false;
   }
  }  
  }
}



void BufferUpdate(sensors_event_t event) { //Function to keep a running buffer of select sensor values needed for data averaging (or numerical integration)

accelAverage = 0;
baroAverage = 0;

for(int c = 0; c<4; c++) {
  accelZBuffer[4-c] = accelZBuffer[3-c];
  baroAltBuffer[4-c] = baroAltBuffer[3-c];
}
accelZBuffer[0] = event.acceleration.z;
baroAltBuffer[0] = bmp.pressureToAltitude(seaLevelPressure, event.pressure);

for(int c = 0; c<5; c++) {
  accelAverage = accelAverage + accelZBuffer[c];
  baroAverage = baroAverage + baroAltBuffer[c];
}
accelAverage = accelAverage / 5;
baroAverage = baroAverage / 5;
baroAverage = baroAverage - startAlt; //Have to compare the average to the ground, not sea level

if(flightState == burnout && !endRollFlag){ //Keep track of rotations once we transition to the burnout stage
/*
 * This section functions stepwise. Once the two buffers are full, the algorithm performs numerical
 * integration, increments rollCounter, and flushes the buffers before starting over.
*/
if(timeBuffer[6] < 1) { //If the buffers are not yet full; avoiding == 0 because it's an unsigned long

for(int c = 0; c<6; c++) {
  timeBuffer[6-c] = timeBuffer[5-c];
  gyroZBuffer[6-c] = gyroZBuffer[5-c];
}
timeBuffer[0] = millis();
 gyroZBuffer[0] = event.gyro.z;
 
}

else { //Once the buffers are full, perform the integration!
  
float deltaT = (timeBuffer[6] - timeBuffer[0]) / 18; //Note: the average is technically /6, but since Simpson's Rule adds a /3 anyways, it's just more efficient to combine them
float simpsonSum = gyroZBuffer[0] + gyroZBuffer[6];
simpsonSum = simpsonSum + 4*(gyroZBuffer[1] + gyroZBuffer[3] + gyroZBuffer[5]);
simpsonSum = simpsonSum + 2*(gyroZBuffer[2] + gyroZBuffer[4]);
simpsonSum = simpsonSum * deltaT; //Hooray for numerical integration! Should look into whether there's a more efficient way to perform Simpson's rule here.

rotationCounter = rotationCounter + (simpsonSum / 6.28318531); //Convert from radians to revolutions and increment the counter!

for(int c = 0; c<7; c++) {
  timeBuffer[c] = 0;
  gyroZBuffer[c] = 0; //Don't forget to clear the buffers for the next step!
}

}

}

}


