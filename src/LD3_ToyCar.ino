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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PREPROCESSOR DEFINITIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG true
<<<<<<< HEAD
#define DEBUG_PRINT_TIME 3
=======
#define DEBUG_PRINT_TIME 1
#define DEBUG_DEC_PRECISION 4
#define DEBUG_GPS_RAW  false
>>>>>>> parent of 7e5841b... Test Candidate 1
#define DEBUG_MOTORS true

#define CONST_LOOP_DELAY 100
#define CONST_BAUD_GPS 9600
#define CONST_BAUD_SERIAL 115200
#define CONST_FILTER_SIZE 5 // 1 to 19
#define CONST_PI 3.14159265
#define CONST_G 9.805
#define CONST_RAD_TO_DEG 57.2958
<<<<<<< HEAD
#define CONST_FULL_TURN_RATE_DEGS 50.0
#define CONST_TOL_HDG 4.0
#define CONST_TOL_LOC 0.00001
#define CONST_NUM_WPTS 5
#define CONST_MAG_VAR 0//-17.0

#define ABS 155 // If you change this, change the CONST_FULL_TURN_RATE_DEGS as well
#define in1 2
#define in2 3
#define in3 4
#define in4 5
#define ENA 1
#define ENB 6
=======
#define CONST_FULL_TURN_RATE_DEGS 360.0
#define CONST_NUM_WPTS 3
#define CONST_TOL_HDG 3.0
#define CONST_TOL_LOC 0.00001

#define ABS 120 // PWM value for motors (constant for now)
#define in1 3
#define in2 4
#define in3 5
#define in4 6
#define ENA 2
#define ENB 7
>>>>>>> parent of 7e5841b... Test Candidate 1
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

Coord loc;
<<<<<<< HEAD
Coord wpt[CONST_NUM_WPTS] { { 44.90530776, -68.66659428 },
                            { 44.90514437, -68.66658719 },
                            { 44.90513722, -68.66639802 },
                            { 44.90531539, -68.66641304 },
                            { 44.90528110, -68.66696930 } };
// Coord wpt[CONST_NUM_WPTS] { { 44.894892, -68.659204 },
//                             { 44.895003, -68.659327 } };
=======
Coord wpt[CONST_NUM_WPTS] { { 44.894862, -68.659313 },
                            { 44.894892, -68.659178 },
                            { 44.895003, -68.659327 } };
Coord target = wpt[0];
>>>>>>> parent of 7e5841b... Test Candidate 1

uint32_t timer   = millis(); // Global millisecond timer
int motorState   = 0;        // Tracks the state of the motor (or rather, the motor's commanded state)
int motorCommand = 0;
int nextWpt      = 1;


boolean flag_useMotors = true;
boolean flag_haveGPS   = false;

float heading      = 0.0f;
float track        = 0.0f;
float headingError = 0.0f;
float locError     = 0.0f;
float turnRate     = CONST_FULL_TURN_RATE_DEGS;

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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);            // Request updates on antenna status

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
  flag_haveGPS = true;


<<<<<<< HEAD
  if (flag_haveGPS && !(flag_done) && flag_useMotors)
  {
    if (locError > CONST_TOL_LOC)
    {
      if (abs(headingError) > CONST_TOL_HDG)
      {
        if (headingError < 0.0) _mRight();
        else _mLeft();
        delay(abs(headingError) / turnRate * 1000);
      } else {
        _mForward();
      }
    } else {
      changeWaypoint();
    }
  } else {
    _mStop();
  }
=======
  flag_useMotors = digitalRead(PIN_ENGINE_SWITCH); // Check physical switch and set boolean accordingly
>>>>>>> parent of 7e5841b... Test Candidate 1

  _mRight();

  // flag_haveGPS   = (GPS.fix && (GPS.satellites != 0));
  // timer          = (timer > millis()) ? millis() : timer;
  //
  // getPhysicalStatus();
  //
  // if (flag_haveGPS)
  // {
  //   if (locError > CONST_TOL_LOC)
  //   {
  //     if (abs(headingError) > CONST_TOL_HDG)
  //     {
  //       if (headingError < 0.0)
  //       {
  //         _mRight();
  //       } else {
  //         _mLeft();
  //       }
  //
  //       delay(abs(headingError) / turnRate * 1000);
  //     }
  //
  //     _mForward();
  //   }
  //   else
  //   {
  //     changeWaypoint();
  //   }
  // }
  //
  //
  // #ifdef DEBUG
  // doDebug();
  // #endif // ifdef DEBUG
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNAL FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void changeWaypoint()
{
  _mStop();
  nextWpt++;

  if (nextWpt > CONST_NUM_WPTS) nextWpt = CONST_NUM_WPTS;
}

void getPhysicalStatus()
{
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  compass.read();

  loc.lat = GPS.latitudeDegrees;
  loc.lon = GPS.longitudeDegrees;

<<<<<<< HEAD
  locError = sqrt(pow((loc.lat - wpt[nextWpt-1].lat),2) +
                  pow((loc.lon - wpt[nextWpt-1].lon),2));


  compass.read();

  heading = (atan2(-compass.m.y,compass.m.x) * CONST_RAD_TO_DEG) + CONST_MAG_VAR;

  if (heading < 0) heading += 360;

  track = desiredHeading(loc,wpt[nextWpt-1]);
=======
  heading = atan2(-compass.m.y,compass.m.x) * CONST_RAD_TO_DEG;

  if (heading < 0) heading += 360;

  track = desiredHeading(loc,target);
>>>>>>> parent of 7e5841b... Test Candidate 1

  headingError = heading - track;

  locError =
    sqrt(pow((loc.lat - wpt[nextWpt].lat),
             2) + pow((loc.lon - wpt[nextWpt].lon),2));
}

void _mForward()
{
  motorCommand = 1;

  if (flag_useMotors && flag_haveGPS)
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

  if (flag_useMotors && flag_haveGPS)
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

  if (flag_useMotors && flag_haveGPS)
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

  if (flag_useMotors && flag_haveGPS)
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
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  motorState = 0;
}

float desiredHeading(Coord start,Coord end)
{
  float track = 180.0 - (atan((end.lon - start.lon) /
                              (end.lat - start.lat)) *
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
      Serial.print(" - "); Serial.print((int)(timer / 1000));
      Serial.println(" seconds elapsed.");
      Serial.print((int)GPS.satellites); Serial.println(" satellites connected.");
      Serial.print("Going to waypoint "); Serial.print(nextWpt);
<<<<<<< HEAD
      Serial.print(" of "); Serial.print(CONST_NUM_WPTS); Serial.println(".");
      # ifdef DEBUG_MOTORS
      Serial.print("Command : ");
=======
      Serial.print(" of "); Serial.println(CONST_NUM_WPTS);
      Serial.print("Location: ");
      Serial.print(loc.lat,6);
      Serial.print(", "); Serial.println(loc.lon,6);
      Serial.print("Loc Error: "); Serial.println(locError,6);
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
>>>>>>> parent of 7e5841b... Test Candidate 1

      switch (motorCommand) {
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

      if (!flag_useMotors) Serial.println(" - Motors disabled.");
      else Serial.println(" - Motors enabled.");
      # endif // ifdef DEBUG_MOTORS
      Serial.println();
      Serial.print("Location: ");
      Serial.print(loc.lat,6);
      Serial.print(", "); Serial.println(loc.lon,6);
      Serial.print("Loc Err : "); Serial.println(locError,6);
    } else Serial.println("\n!!! NO GPS FIX !!!");
    Serial.print("Heading : "); Serial.println(heading);
    Serial.print("Track   : "); Serial.println(track);
    Serial.print("Hdg Err : "); Serial.println(headingError);
    Serial.println();Serial.println();Serial.println();Serial.println();
  }
}

#endif // ifdef DEBUG
<<<<<<< HEAD

void doSetup()
{
  pinMode(PIN_ENGINE_SWITCH,INPUT);
  pinMode(in1,              OUTPUT);
  pinMode(in2,              OUTPUT);
  pinMode(in3,              OUTPUT);
  pinMode(in4,              OUTPUT);
  pinMode(ENA,              OUTPUT);
  pinMode(ENB,              OUTPUT);

  Serial.begin(CONST_BAUD_SERIAL);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  OCR0A   = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  GPS.begin(CONST_BAUD_GPS);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);             // Request updates on antenna status

  mySerial.println(PMTK_Q_RELEASE);
}
=======
>>>>>>> parent of 7e5841b... Test Candidate 1
