
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
#define CONST_xRawMin -16384
#define CONST_xRawMax 16384
#define CONST_yRawMin -16384
#define CONST_yRawMax 16384
#define CONST_zRawMin -16384
#define CONST_zRawMax 16384
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
LSM303::vector<float> mag = { 0.0f, 0.0f, 0.0f },
                      accel_G = { 0.0f, 0.0f, 0.0f };

Coord loc;
Coord wpt1;

RunningMedian arr_lat_raw = RunningMedian(CONST_FILTER_SIZE);
RunningMedian arr_lon_raw = RunningMedian(CONST_FILTER_SIZE);
RunningMedian arr_mx_raw  = RunningMedian(CONST_FILTER_SIZE);
RunningMedian arr_my_raw  = RunningMedian(CONST_FILTER_SIZE);

uint32_t timer = millis(); // Global millisecond timer
int motorState = 0;        // Tracks the state of the motor (or rather, the motor's commanded state)
int arrCounter = 0;        // Dumb counter to stop and filter the GPS signal

boolean useMotors = false;
boolean haveGPS   = false;

float hdg = 0.0f;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP FUNCTION
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(PIN_ENGINE_SWITCH, INPUT);
  pinMode(in1,               OUTPUT);
  pinMode(in2,               OUTPUT);
  pinMode(in3,               OUTPUT);
  pinMode(in4,               OUTPUT);
  pinMode(ENA,               OUTPUT);
  pinMode(ENB,               OUTPUT);

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
  useMotors = digitalRead(PIN_ENGINE_SWITCH); // Check physical switch and set boolean accordingly
  haveGPS   = (GPS.fix && (GPS.satellites != 0));
  timer     = (timer > millis()) ? millis() : timer;

  getPhysicalStatus();
  _mForward();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  DEBUG
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  #ifdef DEBUG

  if (millis() - timer > DEBUG_PRINT_TIME * 1000) {
    timer = millis(); // reset the timer

    Serial.print("\n");
    Serial.print(GPS.hour,    DEC); Serial.print(':');
    Serial.print(GPS.minute,  DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC);

    if (haveGPS) {
      Serial.print(" - "); Serial.print((int)GPS.satellites);
      Serial.println(
        " satellites connected.");
      Serial.print("Location: ");
      Serial.print(loc.lat, 5);
      Serial.print(", "); Serial.println(loc.lon,5);
    } else Serial.println("\n!!! NO GPS FIX !!!");
    Serial.print("hdg (deg.): "); Serial.println(hdg);
    Serial.print("Acceleration (Gs): ");
    Serial.print(accel_G.x, DEBUG_DEC_PRECISION); Serial.print(", ");
    Serial.print(accel_G.y, DEBUG_DEC_PRECISION); Serial.print(", ");
    Serial.println(accel_G.z, DEBUG_DEC_PRECISION);
    # ifdef DEBUG_MOTORS
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

    if (!useMotors) Serial.println(" (switch is off)");
    else Serial.println(" (by logic)");
    # endif // ifdef DEBUG_MOTORS
  }
  #endif    // ifdef
  delay(LOOP_DELAY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNAL FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getPhysicalStatus()
{
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  compass.read();

  accel_G.x = (map(compass.a.x,CONST_xRawMin,CONST_xRawMax,-1000,1000)) / 1000.0;
  accel_G.y = (map(compass.a.y,CONST_yRawMin,CONST_yRawMax,-1000,1000)) / 1000.0;
  accel_G.z = (map(compass.a.z,CONST_zRawMin,CONST_zRawMax,-1000,1000)) / 1000.0;

  arr_lat_raw.add(GPS.latitudeDegrees);
  arr_lon_raw.add(GPS.longitudeDegrees);
  arr_mx_raw.add(compass.m.x);
  arr_my_raw.add(compass.m.y);

  hdg = (atan2(mag.y,mag.x) * 180) / CONST_PI;
  hdg = (hdg < 0) ? hdg + 360 : hdg;

  arrCounter++;

  if (arrCounter >= (CONST_FILTER_SIZE - 1))
  {
    loc.lat = arr_lat_raw.getMedian();
    loc.lon = arr_lon_raw.getMedian();
    mag.y   = arr_my_raw.getMedian();
    mag.x   = arr_mx_raw.getMedian();

    arrCounter = 0;
  }
}

void _mForward()
{
  if (useMotors && haveGPS)
  {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorState = 1;
  } else _mStop();
}

void _mBack()
{
  if (useMotors && haveGPS)
  {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorState = 2;
  } else _mStop();
}

void _mLeft()
{
  if (useMotors && haveGPS)
  {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorState = 3;
  } else _mStop();
}

void _mRight()
{
  if (useMotors && haveGPS)
  {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorState = 4;
  } else _mStop();
}

void _mStop()
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  motorState = 0;
}

float desiredhdg(Coord start,Coord end)
{
  float dPhi =
    log(tan(end.lat / 2 + CONST_PI / 4) / tan(start.lat / 2 + CONST_PI / 4));
  float dLon = abs(start.lon - end.lon);

  return atan2(dPhi,dLon);
}
