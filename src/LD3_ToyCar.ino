#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <RunningMedian.h>

#define DEBUG_PRINT_TIME 3
#define DEBUG_GPS_RAW  false
#define DEBUG_GPS_TIDY true

#define CONST_BAUD_GPS 9600 // Could use 4800 if needed, but 9600 is default
#define CONST_BAUD_SERIAL 115200
#define CONST_FILTER_SIZE 15

// #define CONST_QUARTILE_SIZE (int)(CONST_FILTER_SIZE / 4)

#define in1 3
#define in2 4
#define in3 5
#define in4 6
#define ENA 2
#define ENB 7
#define ABS 120

// Make sure the switch is set to SoftSerial
SoftwareSerial mySerial(8, 7);
Adafruit_GPS   GPS(&mySerial);

boolean usingInterrupt = true;

int arrCounter = 0;

uint32_t timer = millis();

RunningMedian lat = RunningMedian(CONST_FILTER_SIZE);
RunningMedian lon = RunningMedian(CONST_FILTER_SIZE);
float latAvg;
float lonAvg;

// float arr_lat_raw[CONST_FILTER_SIZE];
// float arr_lon_raw[CONST_FILTER_SIZE];
// float lat_filtered = 0.0f;
// float lon_filtered = 0.0f;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void _mForward();
void _mBack();
void _mLeft();
void _mRight();
void _mStop();

void setup()
{
  Serial.begin(CONST_BAUD_SERIAL); // Don't wanna drop chars
  GPS.begin(CONST_BAUD_GPS);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);            // Request updates on antenna status

  useInterrupt(true);

  mySerial.println(PMTK_Q_RELEASE);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();

#ifdef UDR0

  if (DEBUG_GPS_RAW)
    if (c) UDR0 = c;
#endif // ifdef UDR0
}


void loop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (!usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();

    // if you want to debug, this is a good time to do it!
    if (DEBUG_GPS_RAW)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  timer = (timer > millis()) ? millis() : timer;

  lat.add(GPS.latitude);
  lon.add(GPS.longitude);

  latAvg = lat.getMedian();
  lonAvg = lon.getMedian();

  // arr_lat_raw[arrCounter] = GPS.latitude;
  // arr_lon_raw[arrCounter] = GPS.longitude;

  // arrCounter++;

  // if (arrCounter == (CONST_FILTER_SIZE - 1))
  // {
  //  lat_filtered = lon_filtered = 0;

  //  for (int i = CONST_QUARTILE_SIZE - 1;
  //       i < (CONST_FILTER_SIZE - CONST_QUARTILE_SIZE - 1); i++)
  //  {
  //    lat_filtered += arr_lat_raw[i];
  //    lon_filtered += arr_lon_raw[i];
  //  }
  //  lat_filtered /= (CONST_QUARTILE_SIZE * 2);
  //  lon_filtered /= (CONST_QUARTILE_SIZE * 2);

  //  arrCounter = 0;
  // }

  #ifdef DEBUG_GPS_TIDY

  if (millis() - timer > DEBUG_PRINT_TIME * 1000) {
    timer = millis(); // reset the timer

    Serial.print("\n");
    Serial.print(GPS.hour,   DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds,DEC);

    if (GPS.fix) {
      Serial.print(" - "); Serial.print((int)GPS.satellites);
      Serial.println(
        " satellites connected.");
      Serial.print("Location (raw.): "); Serial.print(GPS.latitude,4);
      Serial.print(", "); Serial.println(GPS.longitude,4);

      Serial.print("Location (flt.): "); Serial.print(latAvg,4);
      Serial.print(", "); Serial.println(lonAvg,4);
    } else Serial.println("\n!!! NO GPS FIX !!!");
  }
  #endif // ifdef DEBUG_GPS_TIDY
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A          = 0xAF;
    TIMSK0        |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0        &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void _mForward()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go forward!");
}

void _mBack()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("go back!");
}

void _mLeft()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("go left!");
}

void _mRight()
{
  analogWrite(ENA,ABS);
  analogWrite(ENB,ABS);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go right!");
}

void _mStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  Serial.println("Stop!");
}
