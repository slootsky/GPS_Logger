#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

#include <TinyGPS.h>

#define USE_MAG_SENSOR
#define USE_ACCEL_SENSOR
#define GPSECHO

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
  
  Serial.print(F("Testing TinyGPS library v. ")); Serial.println(TinyGPS::library_version());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

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

  logfile.print(F("Testing TinyGPS library v. ")); logfile.println(TinyGPS::library_version());
  logfile.println(F("by Mikal Hart"));
  logfile.println();

  serial_and_log_println(F("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  serial_and_log_println(F("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
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

#ifndef _GPS_NO_STATS
  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
#endif  
  serial_and_log_println();
  
//  logfile.flush();
  logfile.close();
  smartdelay(1000);
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
//  if (age == TinyGPS::GPS_INVALID_AGE)
  if (year == 2000)
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
  Serial.print  (F("Accel Sensor:       ")); Serial.println(accel_sensor.name);
  Serial.print  (F("Accel Driver Ver:   ")); Serial.println(accel_sensor.version);
  Serial.print  (F("Accel Unique ID:    ")); Serial.println(accel_sensor.sensor_id);
  Serial.print  (F("Accel Max Value:    ")); Serial.print(accel_sensor.max_value); Serial.println(" m/s^2");
  Serial.print  (F("Accel Min Value:    ")); Serial.print(accel_sensor.min_value); Serial.println(" m/s^2");
  Serial.print  (F("Accel Resolution:   ")); Serial.print(accel_sensor.resolution); Serial.println(" m/s^2");  
#endif  
#ifdef USE_MAG_SENSOR
  Serial.print  (F("Mag   Sensor:       ")); Serial.println(mag_sensor.name);
  Serial.print  (F("Mag   Driver Ver:   ")); Serial.println(mag_sensor.version);
  Serial.print  (F("Mag   Unique ID:    ")); Serial.println(mag_sensor.sensor_id);
  Serial.print  (F("Mag   Max Value:    ")); Serial.print(mag_sensor.max_value); Serial.println(" uT");
  Serial.print  (F("Mag   Min Value:    ")); Serial.print(mag_sensor.min_value); Serial.println(" uT");
  Serial.print  (F("Mag   Resolution:   ")); Serial.print(mag_sensor.resolution); Serial.println(" uT");  
#endif
  Serial.println(F("------------------------------------"));
}

void serial_and_log_print(char c)
{
  Serial.print(c);
  logfile.print(c);
}

void serial_and_log_print(char* s)
{
  Serial.print(s);
  logfile.print(s);
}  
void serial_and_log_println(char c)
{
  Serial.println(c);
  logfile.println(c);
}

void serial_and_log_println(char* s)
{
  Serial.println(s);
  logfile.println(s);
}

void serial_and_log_print(const __FlashStringHelper* s)
{
  Serial.print(s);
  logfile.print(s);
}  

void serial_and_log_println(const __FlashStringHelper* s)
{
  Serial.println(s);
  logfile.println(s);
}

void serial_and_log_print(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
}

void serial_and_log_println(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
}

