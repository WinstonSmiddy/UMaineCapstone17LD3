//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//    ___   ____ ________    __    ____ _____
//   |__ \ / __ <  /__  /   / /   / __ \__  /
//   __/ // / / / /  / /   / /   / / / //_ <
//  / __// /_/ / /  / /   / /___/ /_/ /__/ /
// /____/\____/_/  /_/   /_____/_____/____/
//
// UNIVERSITY OF MAINE - SENIOR CAPSTONE, CLASS OF 2017
//
//   LAND DRONE 3:
//     W. GREGORY SMIDDY  |  SPENCER BERNIER  |  KAITLYN SEEHUSEN  |  SHANE CYR
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PREPROCESSOR LIBRARIES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <LSM303.h>
#include <RunningMedian.h>

// #include <SPI.h>
// #include <SD.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PREPROCESSOR DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG true
#define DEBUG_PRINT_TIME 1
#define DEBUG_DEC_PRECISION 4
#define DEBUG_GPS_RAW  false
#define DEBUG_MOTORS true
#define LOOP_DELAY 100

#define CONST_BAUD_GPS 9600 // Could use 4800 if needed, but 9600 is default
#define CONST_BAUD_SERIAL 115200
#define CONST_FILTER_SIZE 5 // 1 to 19
#define CONST_PI 3.14159265
#define CONST_G 9.805

// #define CONST_xRawMin -16384
// #define CONST_xRawMax 16384
// #define CONST_yRawMin -16384
// #define CONST_yRawMax 16384
// #define CONST_zRawMin -16384
// #define CONST_zRawMax 16384
#define CONST_RAD_TO_DEG 57.2958
#define CONST_FULL_TURN_RATE_DEGS 360.0
#define CONST_NUM_WPTS 3
#define CONST_TOL_HDG 3.0
#define CONST_TOL_LOC     0.00002

#define ABS 120 // PWM value for motors (constant for now)

#define in1 3
#define in2 4
#define in3 5
#define in4 6
#define ENA 2
#define ENB 7

// #define PIN_SD_RW 10
#define PIN_ENGINE_SWITCH 13

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CUSTOM CLASSES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Coord {
public:

  float lat;
  float lon;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GLOBAL OBJECTS & VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
SoftwareSerial mySerial(8, 7); // MAKE SURE THE SWITCH ON THE GPS SHIELD IS SET TO "SOFTSERIAL"
Adafruit_GPS   GPS(&mySerial);

LSM303 compass;
LSM303::vector<float> mag = { 0.0f, 0.0f, 0.0f };

//  accel_G = { 0.0f, 0.0f, 0.0f };

Coord loc;
Coord wpt[CONST_NUM_WPTS] { { 44.894862, -68.659313 },
                            { 44.894892, -68.659178 },
                            { 44.895003, -68.659327 } };
Coord target = wpt[0];

// RunningMedian arr_lat_raw = RunningMedian(CONST_FILTER_SIZE);
// RunningMedian arr_lon_raw = RunningMedian(CONST_FILTER_SIZE);
// RunningMedian arr_mx_raw  = RunningMedian(CONST_FILTER_SIZE);
// RunningMedian arr_my_raw  = RunningMedian(CONST_FILTER_SIZE);

uint32_t timer   = millis(); // Global millisecond timer
int motorState   = 0;        // Tracks the state of the motor (or rather, the motor's commanded state)
int motorCommand = 0;
int nextWpt      = 1;


boolean flag_useMotors      = true;
boolean flag_haveGPS        = false;
boolean flag_onFirstLoop    = true;
boolean flag_waypointChange = false;

float heading      = 0.0f;
float track        = 0.0f;
float headingError = 0.0f;
float latError     = 0.0f;
float lonError     = 0.0f;

float turnRate = CONST_FULL_TURN_RATE_DEGS;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP FUNCTION
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(PIN_ENGINE_SWITCH,INPUT);
  pinMode(in1,              OUTPUT);
  pinMode(in2,              OUTPUT);
  pinMode(in3,              OUTPUT);
  pinMode(in4,              OUTPUT);
  pinMode(ENA,              OUTPUT);
  pinMode(ENB,              OUTPUT);

  Serial.begin(CONST_BAUD_SERIAL); // Don't wanna drop chars

  Wire.begin();
  compass.init();
  compass.enableDefault();

  OCR0A   = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  GPS.begin(CONST_BAUD_GPS);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);             // Request updates on antenna status

  mySerial.println(PMTK_Q_RELEASE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPT TIMER
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();

#ifdef UDR0

  if (DEBUG_GPS_RAW)
    if (c) UDR0 = c;
#endif // ifdef UDR0
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  flag_useMotors = digitalRead(PIN_ENGINE_SWITCH); // Check physical switch and set boolean accordingly
  flag_haveGPS   = (GPS.fix && (GPS.satellites != 0));
  timer          = (timer > millis()) ? millis() : timer;

  getPhysicalStatus();


  if (flag_haveGPS && !(flag_onFirstLoop))
  {
    if ((abs(latError)  > CONST_TOL_LOC) || (abs(lonError) > CONST_TOL_LOC))
    {
      if (abs(headingError) > CONST_TOL_HDG)
      {
        if (headingError < 0.0)
        {
          _mRight();
        } else {
          _mLeft();
        }
        delay(abs(headingError) / turnRate * 1000);
      }
      _mForward();
    } else {
      flag_waypointChange = true;
    }
  }

  if (flag_waypointChange) changeWaypoint();

  #ifdef DEBUG
  doDebug();
  #endif // ifdef DEBUG

  delay(LOOP_DELAY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNAL FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void changeWaypoint()
{
  _mStop();
  nextWpt++;

  if (nextWpt > CONST_NUM_WPTS)
  {
    _mStop();
    nextWpt = CONST_NUM_WPTS;
  }

  target = wpt[nextWpt];

  latError = loc.lat - target.lat;
  lonError = loc.lon - target.lon;

  flag_waypointChange = false;
}

void getPhysicalStatus()
{
  static int arrCounter = 0;

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  compass.read();

  // accel_G.x = (map(compass.a.x,CONST_xRawMin,CONST_xRawMax,-1000,1000)) / 1000.0;
  // accel_G.y = (map(compass.a.y,CONST_yRawMin,CONST_yRawMax,-1000,1000)) / 1000.0;
  // accel_G.z = (map(compass.a.z,CONST_zRawMin,CONST_zRawMax,-1000,1000)) / 1000.0;

  // arr_lat_raw.add(GPS.latitudeDegrees);
  // arr_lon_raw.add(GPS.longitudeDegrees);
  // arr_mx_raw.add(compass.m.x);
  // arr_my_raw.add(compass.m.y);

  // arrCounter++;

  // if (arrCounter >= (CONST_FILTER_SIZE - 1))
  // {
  // loc.lat = arr_lat_raw.getMedian();
  // loc.lon = arr_lon_raw.getMedian();
  // mag.y   = arr_my_raw.getMedian();
  // mag.x   = arr_mx_raw.getMedian();

  loc.lat = GPS.latitudeDegrees;
  loc.lon = GPS.longitudeDegrees;
  mag.y   = compass.m.y;
  mag.x   = compass.m.x;

  arrCounter = 0;

  heading = atan2(-mag.y,mag.x) * CONST_RAD_TO_DEG;

  if (heading < 0)
  {
    heading += 360;
  }

  if ((loc.lat != 0.0) && (loc.lon != 0.0))
  {
    track = desiredHeading(loc,target);

    headingError = heading - track;

    latError = loc.lat - target.lat;
    lonError = loc.lon - target.lon;
  }

  // flag_onFirstLoop = false;
  // }
}

void _mForward()
{
  motorCommand = 1;

  if (flag_useMotors && flag_haveGPS && !(flag_onFirstLoop))
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    motorState = motorCommand;
  } else _mStop();
}

void _mBack()
{
  motorCommand = 2;

  if (flag_useMotors && flag_haveGPS && !(flag_onFirstLoop))
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    motorState = motorCommand;
  } else _mStop();
}

void _mLeft()
{
  motorCommand = 3;

  if (flag_useMotors && flag_haveGPS && !(flag_onFirstLoop))
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    motorState = motorCommand;
  } else _mStop();
}

void _mRight()
{
  motorCommand = 4;

  if (flag_useMotors && flag_haveGPS && !(flag_onFirstLoop))
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    motorState = motorCommand;
  } else _mStop();
}

void _mStop()
{
  motorCommand = 0;
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  motorState = motorCommand;
}

float desiredHeading(Coord start,Coord end)
{
  float track = 180.0 - (atan((end.lon - start.lon) / (end.lat - start.lat)) *
                         CONST_RAD_TO_DEG);

  if (track < 0) track += 360;
  return track;
}

#ifdef DEBUG
void doDebug() {
  if (millis() - timer > DEBUG_PRINT_TIME * 1000) {
    timer = millis(); // reset the timer

    Serial.print("\n");
    Serial.print(GPS.hour,   DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds,DEC);

    if (flag_haveGPS) {
      Serial.print(" - "); Serial.print((int)GPS.satellites);
      Serial.println(
        " satellites connected.");
      Serial.print("Going to waypoint "); Serial.print(nextWpt);
      Serial.print(" of "); Serial.println(CONST_NUM_WPTS);
      Serial.print("Location: ");
      Serial.print(loc.lat,6);
      Serial.print(", "); Serial.println(loc.lon,6);
      Serial.print("Loc Error: "); Serial.print(latError,6);
      Serial.print(", "); Serial.println(lonError,6);
    } else Serial.println("\n!!! NO GPS FIX !!!");
    Serial.print("Heading (deg.): "); Serial.println(heading);
    Serial.print("Track (deg.): "); Serial.println(track);
    # ifdef DEBUG_MOTORS
    Serial.print("Motor Command: ");

    switch (motorCommand) {
    case 0: Serial.println("Stopped");
      break;

    case 1: Serial.println("Forward");
      break;

    case 2: Serial.println("Backward");
      break;

    case 3: Serial.println("Left");
      break;

    case 4: Serial.println("Right");
      break;
    }
    Serial.print("Motor State: ");

    switch (motorState) {
    case 0: Serial.print("Stopped");
      break;

    case 1: Serial.print("Forward");
      break;

    case 2: Serial.print("Backward");
      break;

    case 3: Serial.print("Left");
      break;

    case 4: Serial.print("Right");
      break;
    }

    if (!flag_useMotors) Serial.println(" (switch is off)");
    else Serial.println(" (by logic)");
    # endif // ifdef DEBUG_MOTORS
  }
}

#endif // ifdef DEBUG
