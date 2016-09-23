#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

#define DEBUG true
#define DEBUG_PRINT_TIME 1
#define DEBUG_GPS_RAW  false
#define DEBUG_MOTORS true

#define CONST_BAUD_GPS 9600  // Could use 4800 if needed, but 9600 is default
#define CONST_BAUD_SERIAL 115200
#define CONST_FILTER_SIZE 64 // Multiples of 4
#define CONST_QUARTILE_SIZE (int)(CONST_FILTER_SIZE / 4)

#define in1 3
#define in2 4
#define in3 5
#define in4 6
#define ENA 2
#define ENB 7
#define ABS 120
#define PIN_ENGINE_SWITCH 13

// Make sure the switch is set to SoftSerial
SoftwareSerial mySerial(8, 7);
Adafruit_GPS   GPS(&mySerial);

int motorState = 0;

boolean useMotors = false;
boolean haveGPS   = false;

int arrCounter = 0;

uint32_t timer = millis();

float arr_lat_raw[CONST_FILTER_SIZE];
float arr_lon_raw[CONST_FILTER_SIZE];
float lat_filtered = 0.0f;
float lon_filtered = 0.0f;


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

  OCR0A   = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  GPS.begin(CONST_BAUD_GPS);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);            // Request updates on antenna status

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
  useMotors = digitalRead(PIN_ENGINE_SWITCH);

  _mBack();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  haveGPS = (GPS.fix && (GPS.satellites != 0));
  timer   = (timer > millis()) ? millis() : timer;

  arr_lat_raw[arrCounter] = GPS.latitude;
  arr_lon_raw[arrCounter] = GPS.longitude;

  arrCounter++;

  if (arrCounter >= (CONST_FILTER_SIZE - 1))
  {
    lat_filtered = getIQM(arr_lat_raw,CONST_FILTER_SIZE);
    lon_filtered = getIQM(arr_lon_raw,CONST_FILTER_SIZE);
    arrCounter   = 0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  DEBUG
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  #ifdef DEBUG

  if (millis() - timer > DEBUG_PRINT_TIME * 1000) {
    timer = millis(); // reset the timer

    Serial.print("\n");
    Serial.print(GPS.hour,   DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds,DEC);

    if (haveGPS) {
      Serial.print(" - "); Serial.print((int)GPS.satellites);
      Serial.println(
        " satellites connected.");
      Serial.print("Location: "); Serial.print(lat_filtered,5);
      Serial.print(", "); Serial.println(lon_filtered,5);
    } else Serial.println("\n!!! NO GPS FIX !!!");
    # ifdef DEBUG_MOTORS
    Serial.print("Motor State: ");

    switch (motorState) {
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
    # endif // ifdef DEBUG_MOTORS
  }
  #endif    // ifdef
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXTERNAL FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void _mForward()
{
  if (useMotors)
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    motorState = 1;
  } else _mStop();
}

void _mBack()
{
  if (useMotors)
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    motorState = 2;
  } else _mStop();
}

void _mLeft()
{
  if (useMotors)
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    motorState = 3;
  } else _mStop();
}

void _mRight()
{
  if (useMotors)
  {
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    motorState = 4;
  } else _mStop();
}

void _mStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  motorState = 0;
}

void bubble_sort(float arr[],int n)
{
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n - i - 1; ++j)
      if (arr[j] > arr[j + 1])
      {
        float temp = arr[j];
        arr[j]     = arr[j + 1];
        arr[j + 1] = temp;
      }
}

float getIQM(float arr[],int n)
{
  bubble_sort(arr,CONST_FILTER_SIZE);

  float filtered = 0.0f; // = arr[(CONST_FILTER_SIZE - 1) / 2]; // 0.0f;

  int m = (int)(n / 4);

  for (int i = m - 1;
       i < (n - m - 1); i++)
  {
    filtered += arr[i];
  }
  filtered /= (m * 2);

  return filtered;
}
