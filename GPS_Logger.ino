#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

#include <TinyGPS.h>

#define USE_MAG_SENSOR
#define USE_ACCEL_SENSOR
// #define GPSECHO
#define SERIAL_OUTPUT

#define LOOP_DELAY 5000

#ifdef USE_MAG_SENSOR || USE_ACCEL_SENSOR
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#endif

#ifdef USE_ACCEL_SENSOR
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
#endif

#ifdef USE_MAG_SENSOR
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#endif

#define chipSelect 10
#define ledPin 13

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial ss(8, 7);
char filename[15];

File logfile;


static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void displaySensorDetails(void);
void error(uint8_t errno);
void serial_and_log_print(char);
void serial_and_log_println(char);
void serial_and_log_print(char*s = 0);
void serial_and_log_println(char *s = 0);
void serial_and_log_print(const __FlashStringHelper* s);
void serial_and_log_println(const __FlashStringHelper* s);
void serial_and_log_print(float &f, int& i);
void serial_and_log_println(float &f, int& i);

void setup()
{
  Serial.begin(115200);
  
  Serial.print(F("Using TinyGPS library v. ")); Serial.println(TinyGPS::library_version());
#ifdef SERIAL_OUTPUT
  Serial.println(F("by Mikal Hart"));
  Serial.println();
#endif

  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println(F("Card init. failed!"));
    error(2);
  }
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print(F("Couldn't create ")); 
    Serial.println(filename);
    error(3);
  }
  Serial.print(F("Writing to ")); 
  Serial.println(filename);

  logfile.print(F("Using TinyGPS library v. ")); logfile.println(TinyGPS::library_version());
//  logfile.println(F("by Mikal Hart"));
  logfile.println();

  #ifdef USE_MAG_SENSOR
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    serial_and_log_println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  #endif

  #ifdef USE_ACCEL_SENSOR
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    serial_and_log_println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  #endif

  displaySensorDetails();
  
  serial_and_log_println(F("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  mag   accel pressure temp go here              "));
  serial_and_log_println(F("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  x y z x y z                                   "));
  serial_and_log_println(F("-------------------------------------------------------------------------------------------------------------------------------------"));

  logfile.close();
  ss.begin(9600);
}
void openLogfile()
{
    logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print(F("Couldn't create ")); 
    Serial.println(filename);
    error(3);
  }
}

void loop()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
//  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  sensors_event_t mag_event; 
  mag.getEvent(&mag_event);
  sensors_event_t accel_event; 
  accel.getEvent(&accel_event);

  openLogfile();  

  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
//  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
//  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
//  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

   print_float(mag_event.magnetic.x,-1,7,3);
   print_float(mag_event.magnetic.y,-1,7,3);
   print_float(mag_event.magnetic.z,-1,7,3);
   print_float(accel_event.acceleration.x,-1,7,3);
   print_float(accel_event.acceleration.y,-1,7,3);
   print_float(accel_event.acceleration.z,-1,7,3);

#ifndef _GPS_NO_STATS
  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
#endif  
  serial_and_log_println();
  
//  logfile.flush();
  logfile.close();
  smartdelay(LOOP_DELAY);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      char c = ss.read();
      gps.encode(c);
      #ifdef GPSECHO
      Serial.print(c);
      #endif
    }
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      serial_and_log_print('*');
    serial_and_log_print(' ');
  }
  else
  {
    serial_and_log_print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      serial_and_log_print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  serial_and_log_print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if ( day == 0)
    serial_and_log_print(F("********** ******** "));
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    serial_and_log_print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    serial_and_log_print(i<slen ? str[i] : ' ');
  smartdelay(0);
}

// blink out an error code
void error(uint8_t errno) {
  /*
  if (SD.errorCode()) {
   putstring("SD error: ");
   Serial.print(card.errorCode(), HEX);
   Serial.print(',');
   Serial.println(card.errorData(), HEX);
   }
   */
   
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void displaySensorDetails(void)
{
#ifdef USE_ACCEL_SENSOR
  sensor_t accel_sensor;
  accel.getSensor(&accel_sensor);
#endif  

#ifdef USE_MAG_SENSOR
  sensor_t mag_sensor;
  mag.getSensor(&mag_sensor);
#endif

//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 3
  Serial.println(F("------------------------------------"));
#ifdef USE_ACCEL_SENSOR
  serial_and_log_print  (F("Accel Sensor:       ")); serial_and_log_println(accel_sensor.name);
  serial_and_log_print  (F("Accel Driver Ver:   ")); serial_and_log_println(accel_sensor.version);
  serial_and_log_print  (F("Accel Unique ID:    ")); serial_and_log_println(accel_sensor.sensor_id);
  serial_and_log_print  (F("Accel Max Value:    ")); serial_and_log_print(accel_sensor.max_value); serial_and_log_println(" m/s^2");
  serial_and_log_print  (F("Accel Min Value:    ")); serial_and_log_print(accel_sensor.min_value); serial_and_log_println(" m/s^2");
  serial_and_log_print  (F("Accel Resolution:   ")); serial_and_log_print(accel_sensor.resolution); serial_and_log_println(" m/s^2");  
#endif  
#ifdef USE_MAG_SENSOR
  serial_and_log_print  (F("Mag   Sensor:       ")); serial_and_log_println(mag_sensor.name);
  serial_and_log_print  (F("Mag   Driver Ver:   ")); serial_and_log_println(mag_sensor.version);
  serial_and_log_print  (F("Mag   Unique ID:    ")); serial_and_log_println(mag_sensor.sensor_id);
  serial_and_log_print  (F("Mag   Max Value:    ")); serial_and_log_print(mag_sensor.max_value); serial_and_log_println(" uT");
  serial_and_log_print  (F("Mag   Min Value:    ")); serial_and_log_print(mag_sensor.min_value); serial_and_log_println(" uT");
  serial_and_log_print  (F("Mag   Resolution:   ")); serial_and_log_print(mag_sensor.resolution); serial_and_log_println(" uT");  
#endif
  Serial.println(F("------------------------------------"));
}

void serial_and_log_print(char c)
{
#ifdef SERIAL_OUTPUT
  Serial.print(c);
#endif
  logfile.print(c);
}

void serial_and_log_print(char* s)
{
#ifdef SERIAL_OUTPUT
  Serial.print(s);
#endif
  logfile.print(s);
}  
void serial_and_log_println(char c)
{
#ifdef SERIAL_OUTPUT
  Serial.println(c);
#endif
  logfile.println(c);
}

void serial_and_log_println(char* s)
{
#ifdef SERIAL_OUTPUT
  Serial.println(s);
#endif
  logfile.println(s);
}

void serial_and_log_print(const __FlashStringHelper* s)
{
#ifdef SERIAL_OUTPUT
  Serial.print(s);
#endif
  logfile.print(s);
}  

void serial_and_log_println(const __FlashStringHelper* s)
{
#ifdef SERIAL_OUTPUT
  Serial.println(s);
#endif
  logfile.println(s);
}

void serial_and_log_print(float &f, int& i)
{
#ifdef SERIAL_OUTPUT
  Serial.print(f,i);
#endif
  logfile.print(f,i);
}

void serial_and_log_println(float &f, int& i)
{
#ifdef SERIAL_OUTPUT
  Serial.print(f,i);
#endif
  logfile.print(f,i);
}

