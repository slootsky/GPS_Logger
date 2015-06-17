#define _GPS_NO_STATS
// this is undo of everything in the IDE
#include <TinyGPS.h>

// GPS commands come from http://www.adafruit.com/datasheets/PMTK_A11.pdf

#define PAUSE_DURATION  180000  // 3 minutes
// #define PAUSE_DURATION  15000 // 15 seconds

enum MOVING_STATE { MOVING, PAUSED, STOPPED };
MOVING_STATE moving_state;


// Configuration for MEGA
#define GPS_ENABLE_PIN 7
#define RED_LED 4
#define YELLOW_LED 3
#define GREEN_LED 2
#define BLUE_LED 5
#define gpsSerial  Serial1

unsigned char red_level =75;
unsigned char blue_level =10;
unsigned char yellow_level =75;
unsigned char green_level =255;

unsigned long paused_millis = millis();

float seaLevelPressure = 101.325;
#define GYRO_THRESHOLD  1.5
//#define MAG_THRESHOLD  10
//#define ACCEL_THRESHOLD  3.0
//#define ACCEL_DELTA_THRESHOLD  0.25    // 25%
#define ACCEL_THRESHOLD  10.0
#define ACCEL_DELTA_THRESHOLD  0.95    // 25%
#define SPEED_DELTA_THRESHOLD  0.10    // 10%
#define THRESHOLD_METRES       500.0     // metres
#define SERIAL_OUTPUT
// #define EXPLAIN_OUTPUT
#define RUN_FLAG_OUTPUT
#define CSV_HEADER_OUTPUT
#define LED_INDICATORS

#include <SPI.h>

//#define DIRECT_SD
#ifndef DIRECT_SD
#define SD_LOGGER
#endif

#ifdef DIRECT_SD
#include <SD.h>

File logfile;
#endif

//#include "cryptoSerial.h"
#define logfile Serial2  
//CryptoHardwareSerial logfile(Serial2);


#define USE_10DOF_SENSOR
#define USE_BMP_SENSOR
//#define GPSECHO

#define LOOP_DELAY 5000



#ifdef USE_10DOF_SENSOR

#include <Wire.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(30303);

float g_pitch, g_roll, g_heading;


#endif

#ifdef USE_BMP_SENSOR
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
#endif

#define chipSelect 10
#define ledPin 13


struct all_the_data {
  float  latitude;
  float  longitude;
  unsigned long  fix_age;
  long   altitude;
  float  course;
  float  speed_kmph;
  unsigned long  hdop;
  unsigned short satellites;
  unsigned long last_position_fix;
  unsigned long last_time_fix;
#ifdef  USE_10DOF_SENSOR
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  sensors_event_t gyro_event;
#endif
#ifdef USE_BMP_SENSOR
  sensors_event_t bmp_event;
  float temperature;
#endif
  float aggregate_accel, aggregate_mag, aggregate_gyro;
};

all_the_data  g_sensor_data, g_last_sensor_data, g_last_logged_sensor_data;
char g_runFlags[4];

TinyGPS gps;

// float g_last_latitude, g_last_longitude;

static void smartdelay(unsigned long ms);
static void read_gps_buffer();
#include "print_procedures.h"

// need some millis helper functions

inline bool has_millis_passed(unsigned long since_millis, unsigned long milli_duration)
{
  return ( millis() - since_millis > milli_duration );
}

void stop_gps()
{
  // Hot Restart: Use all available data in the NV Store
  gps_write("$PMTK101");

  // Enter standby mode for power saving.
  gps_write("$PMTK161,0");
  // ground out GPS_ENABLE_PIN
  pinMode(GPS_ENABLE_PIN, OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, LOW);
#ifdef LED_INDICATORS
  analogWrite(BLUE_LED, 0);
#endif
}

void start_gps()
{
  // set this to INPUT to let it float (ie: not Ground)
  pinMode(GPS_ENABLE_PIN, INPUT);

  // Warm Restart: Don't use Ephemeris at re-start
  gps_write("PMTK102");

#ifdef LED_INDICATORS
  analogWrite(BLUE_LED, blue_level);
#endif
}

char *byte_to_hex(byte number)
{
  static char retval[3] = {0, 0, 0};
  if (number > 255) return NULL;
  // first break the digits out into numerical values
  retval[1] = number % 16;
  retval[0] = (number - retval[2]) / 16;
  // convert numbers to ascii hex values
  for ( int i = 0 ; i++ ; i < 2 ) // yes, a loop of two
  {
    retval[i] = ( retval[i] <= 9 ? '0' + retval[i] : 'A' + retval[i] - 9);
  }
  return retval;
}

void gps_write(char *cmd_string)
{
  char checksum_str[3];
  byte checksum = 0;
  char *cmd_char;
  
  // if there is already a $ at the beginning of the command, skip it
  cmd_char = cmd_string;
  while (*cmd_char == '$')
    cmd_char++;

  while ( *cmd_char )
  {
    checksum ^= *cmd_char ;  // (xor)
    cmd_char++;
  }
  cmd_char = cmd_string;
  while (*cmd_char == '$') cmd_char++;

  gpsSerial.print("$");
  gpsSerial.print(cmd_char);
  gpsSerial.print("*");
  gpsSerial.print(byte_to_hex(checksum));
  gpsSerial.print("\r\n");
}


#ifdef DIRECT_SD
void openLogfile()
{
  logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
#ifdef SERIAL_OUTPUT
    Serial.print("Couldn't create ");
    Serial.println(filename);
#endif
    error(3);
  }
}
#endif


static void read_gps_buffer()
{
  while (gpsSerial.available())
  {
    char c = gpsSerial.read();
    gps.encode(c);
#ifdef GPSECHO
    Serial.print(c);
#endif
  }
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    read_gps_buffer();
  } while (millis() - start < ms);
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

  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

#ifdef DISPLAY_SENSOR_DETAILS
//void displaySensorDetails(void)
//{
//#ifdef USE_10DOF_SENSOR
//  sensor_t accel_sensor;
//  accel.getSensor(&accel_sensor);
//
//  sensor_t mag_sensor;
//  mag.getSensor(&mag_sensor);
//#endif
//
////SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 3
//  #ifdef SERIAL_OUTPUT
//  Serial.println("------------------------------------");
//  #endif
//#ifdef USE_10DOF_SENSOR
//  serial_and_log_print  ("Accel Sensor:       "); serial_and_log_println(accel_sensor.name);
//  serial_and_log_print  ("Accel Driver Ver:   "); serial_and_log_println(accel_sensor.version);
//  serial_and_log_print  ("Accel Unique ID:    "); serial_and_log_println(accel_sensor.sensor_id);
//  serial_and_log_print  ("Accel Max Value:    "); serial_and_log_print(accel_sensor.max_value); serial_and_log_println(" m/s^2");
//  serial_and_log_print  ("Accel Min Value:    "); serial_and_log_print(accel_sensor.min_value); serial_and_log_println(" m/s^2");
//  serial_and_log_print  ("Accel Resolution:   "); serial_and_log_print(accel_sensor.resolution); serial_and_log_println(" m/s^2");
//
//  serial_and_log_print  ("Mag   Sensor:       "); serial_and_log_println(mag_sensor.name);
//  serial_and_log_print  ("Mag   Driver Ver:   "); serial_and_log_println(mag_sensor.version);
//  serial_and_log_print  ("Mag   Unique ID:    "); serial_and_log_println(mag_sensor.sensor_id);
//  serial_and_log_print  ("Mag   Max Value:    "); serial_and_log_print(mag_sensor.max_value); serial_and_log_println(" uT");
//  serial_and_log_print  ("Mag   Min Value:    "); serial_and_log_print(mag_sensor.min_value); serial_and_log_println(" uT");
//  serial_and_log_print  ("Mag   Resolution:   "); serial_and_log_print(mag_sensor.resolution); serial_and_log_println(" uT");
//#endif
//  #ifdef SERIAL_OUTPUT
//  Serial.println("------------------------------------");
//  #endif
//}
#endif





void read_gyro()
{
  gyro.getEvent(&g_sensor_data.gyro_event);
};

bool calculate_moving()
{
  if (g_sensor_data.speed_kmph > 10 )  // a minimal number  : unfortunately, it is common to see numbers greater than one : even at a standstill
  {
    Serial.println("moving because speed_kmph");
    g_runFlags[1]='s';
    return true;
  }

  if ( abs(g_sensor_data.gyro_event.gyro.z) > GYRO_THRESHOLD )
  {
    Serial.println("moving because gyro_z");
    g_runFlags[1]='g';
    return true;
  }
  if ( abs(g_sensor_data.gyro_event.gyro.x) > GYRO_THRESHOLD )
  {
    Serial.println("moving because gyro_x");
    g_runFlags[1]='g';
    return true;
  }
  if ( abs(g_sensor_data.gyro_event.gyro.y) > GYRO_THRESHOLD )
  {
    Serial.println("moving because gyro_y");
    g_runFlags[1]='g';
    return true;
  }
  if ( abs( g_sensor_data.aggregate_accel - g_last_sensor_data.aggregate_accel) > ACCEL_THRESHOLD )
  {
    Serial.print("moving because acceleration: aggregate_accel ( ");
    Serial.print(g_sensor_data.aggregate_accel);
    Serial.print(" ) - last.aggregate_accel( ");
    Serial.print(g_last_sensor_data.aggregate_accel);
    Serial.print(" ) = ");
    Serial.println( g_sensor_data.aggregate_accel - g_last_sensor_data.aggregate_accel);
    
    g_runFlags[1]='a';
    return true;
  }


//  if (g_sensor_data.speed > g_last_sensor_data.speed*SPEED_THRESHOLD )
//    return true;
//  if (g_sensor_data.speed < g_last_sensor_data.speed/SPEED_THRESHOLD )
//    return true;
  return false;
};

void take_readings()
{
  // save the last one
  g_last_sensor_data = g_sensor_data;
  
  //  struct all_the_data {
  //    float  latitude;
  //    float  longitude;
  //    unsigned long  fix_age;
  //    float  altitude;
  //    float  course;
  //    float  speed_kmph;
  //  #ifdef  USE_10DOF_SENSOR
  //    sensors_event_t accel_event;
  //    sensors_event_t mag_event;
  //    sensors_vec_t   orientation;
  //    sensors_event_t gyro_event;
  //  #endif
  //  #ifdef USE_BMP_SENSOR
  //    sensors_event_t bmp_event;
  //    float temperature;
  //  #endif
  gps.f_get_position(&g_sensor_data.latitude, &g_sensor_data.longitude, &g_sensor_data.fix_age);
  g_sensor_data.altitude = gps.f_altitude();
  g_sensor_data.course = gps.f_course();
  g_sensor_data.speed_kmph = gps.f_speed_kmph();
  g_sensor_data.satellites = gps.satellites();
  g_sensor_data.hdop = gps.hdop();

  mag.getEvent(&g_sensor_data.mag_event);
  accel.getEvent(&g_sensor_data.accel_event);
  if (!dof.fusionGetOrientation(&g_sensor_data.accel_event, &g_sensor_data.mag_event, &g_sensor_data.orientation))
  {
    g_sensor_data.orientation.heading = TinyGPS::GPS_INVALID_ANGLE;
    g_sensor_data.orientation.roll = TinyGPS::GPS_INVALID_ANGLE;
    g_sensor_data.orientation.pitch = TinyGPS::GPS_INVALID_ANGLE;
  }

  g_sensor_data.last_position_fix = gps.last_position_fix();
  g_sensor_data.last_time_fix = gps.last_time_fix();
  
  g_sensor_data.aggregate_accel = abs(g_sensor_data.accel_event.acceleration.x) + abs(g_sensor_data.accel_event.acceleration.y) + abs(g_sensor_data.accel_event.acceleration.z) ;
  g_sensor_data.aggregate_gyro = abs(g_sensor_data.gyro_event.gyro.x) + abs(g_sensor_data.gyro_event.gyro.y) + abs(g_sensor_data.gyro_event.gyro.z) ;
  g_sensor_data.aggregate_mag = abs(g_sensor_data.mag_event.magnetic.x) + abs(g_sensor_data.mag_event.magnetic.y) + abs(g_sensor_data.mag_event.magnetic.z) ;

  #ifdef USE_BMP_SENSOR
    sensors_event_t bmp_event;
//    float temperature;

    /* Calculate the altitude using the barometric pressure sensor */
    bmp.getEvent(&g_sensor_data.bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in Celcius */
      bmp.getTemperature(&g_sensor_data.temperature);
    }
  #endif

  
}

void write_csv_header()
{
#ifdef RUN_FLAG_OUTPUT
  print_str("RunFlags,");
#endif  
#ifdef EXPLAIN_OUTPUT
  print_str("course_hdr,");
#endif
  print_str("Course");
  print_str(",Speed");
  print_str(",HDOP,Satellites");
  print_str(",Latitude");
  print_str(",dt_age,Date,Time");
#ifdef EXPLAIN_OUTPUT
  print_str(",alt_hdr");
#endif
  print_str(",Altitude");
#ifdef EXPLAIN_OUTPUT
  print_str(",accel_hdr",10);
#endif  
  print_str(",accel_x,accel_y,accel_z");
  print_str(",Longitude");
#ifdef EXPLAIN_OUTPUT
  print_str(",mag_hdr",10);
#endif  
  print_str(",mag_x,mag_y,mag_z");
#ifdef EXPLAIN_OUTPUT
  print_str(",heading_pitch_roll_hdr",10);
#endif  
  print_str(",heading,pitch,roll");
#ifdef EXPLAIN_OUTPUT
  print_str(",gyro_hdr",10);
#endif  
  print_str(",gyro_x,gyro_y,gyro_z");
  print_str(",bmp_pressure");
  print_str(",pressure_by_altitude,Temperature");
#ifdef EXPLAIN_OUTPUT
  print_str(",fix_time_hdr");
#endif  
  print_str(",fix_age");
  


  print_str(",last_fix_position,last_fix_time");

#ifndef _GPS_NO_STATS
  print_str(",stats_chars,stats_sentences,stats_failed");
#endif
  print_str("\n");
  
}

void log_output_forced(bool take_reading = true)
{
  if (take_reading) take_readings();
  g_last_logged_sensor_data = g_sensor_data;
  
#ifdef RUN_FLAG_OUTPUT
  print_str(g_runFlags);
  print_str(",");
#endif  
#ifdef EXPLAIN_OUTPUT
  print_str("course,");
#endif  
  print_float(g_sensor_data.course, 1000.0 /*TinyGPS::GPS_INVALID_ANGLE*/, 11, 6);
  print_str(",");
  print_float(g_sensor_data.speed_kmph, -1 /*TinyGPS::GPS_INVALID_SPEED*/, 5, 1);
#ifdef EXPLAIN_OUTPUT
  print_str(" km/h");
#endif  
  print_str(",");
  print_ulong(g_sensor_data.hdop, TinyGPS::GPS_INVALID_HDOP, 5);
  print_str(",");
  print_int(g_sensor_data.satellites, TinyGPS::GPS_INVALID_SATELLITES, 2);
  print_str(",");

  print_float(g_sensor_data.latitude, 1000.0 /*TinyGPS::GPS_INVALID_ANGLE*/, 11, 6);
  print_str(",");
  print_date(gps);
  print_str(",");
#ifdef EXPLAIN_OUTPUT
  print_str("alt,");
#endif  
  print_ulong(g_sensor_data.altitude, 1000 /*TinyGPS::GPS_INVALID_ALTITUDE*/, 5);
  print_str(",");
#ifdef  USE_10DOF_SENSOR
#ifdef EXPLAIN_OUTPUT
  print_str("accel_xyz,");
#endif  
  print_float(g_sensor_data.accel_event.acceleration.x, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.accel_event.acceleration.y, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.accel_event.acceleration.z, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.longitude, 1000.0 /*TinyGPS::GPS_INVALID_ANGLE*/, 11, 6);
  print_str(",");
#ifdef EXPLAIN_OUTPUT
  print_str("mag_xyz,");
#endif  
  print_float(g_sensor_data.mag_event.magnetic.x, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.mag_event.magnetic.y, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.mag_event.magnetic.z, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
#ifdef EXPLAIN_OUTPUT
  print_str("heading/pitch/roll,");
#endif  
  print_float(g_sensor_data.orientation.heading, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.orientation.pitch, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.orientation.roll, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
#ifdef EXPLAIN_OUTPUT
  print_str("gyro_xyz,");
#endif  
  print_float(g_sensor_data.gyro_event.gyro.x, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.gyro_event.gyro.y, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
  print_str(",");
  print_float(g_sensor_data.gyro_event.gyro.z, TinyGPS::GPS_INVALID_ANGLE, 7, 2);
#endif
#ifdef USE_BMP_SENSOR
  print_str(",");
  print_float(g_sensor_data.bmp_event.pressure,999.0,6,2);
  print_str(",");
  print_float(bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                      g_sensor_data.bmp_event.pressure
                                      //,temperature
                                      ),999.0,6,2); 
#ifdef EXPLAIN_OUTPUT
  print_str(" m,");
#endif
  /* Display the temperature */
  print_float(g_sensor_data.temperature,999.0,6,2);
#ifdef EXPLAIN_OUTPUT
  print_str(" C");
#endif
#endif
  print_str(",");
#ifdef EXPLAIN_OUTPUT
  print_str("fix_time,");
#endif  
  print_ulong(g_sensor_data.fix_age, TinyGPS::GPS_INVALID_FIX_TIME, 5);
  print_str(",");
  print_ulong(g_sensor_data.last_position_fix, TinyGPS::GPS_INVALID_FIX_TIME, 5);
  print_str(",");
  print_ulong(g_sensor_data.last_time_fix, TinyGPS::GPS_INVALID_FIX_TIME, 5);

// TODO JZAS should add in the GPS Stats output here

  print_str("\n");
  
// This seems to be the wrong place for this.  Moved it to take_readings//  g_last_sensor_data = g_sensor_data;
  //  g_last_latitude = g_sensor_data.latitude;
  //  g_last_longitude = g_sensor_data.longitude;

  read_gps_buffer();
}

void log_output()
{
  bool do_log_output = false;

  take_readings();

  if ( g_sensor_data.aggregate_accel != 0.0
    && (abs(g_sensor_data.aggregate_accel-g_last_sensor_data.aggregate_accel)/g_sensor_data.aggregate_accel > ACCEL_DELTA_THRESHOLD )
    )
    {
#ifdef SERIAL_OUTPUT
      Serial.print("logging because accel delta threshold  abs(g_sensor_data.aggregate_accel(");

      Serial.print(g_sensor_data.aggregate_accel);
      Serial.print(" ) - last.aggregate_accel( ");
      Serial.print(g_last_sensor_data.aggregate_accel);
      Serial.print(" ) ) / g_sensor_data.aggregate_accel( ");
      Serial.print(g_sensor_data.aggregate_accel);
      Serial.print(" ) = ");
      Serial.println( abs(g_sensor_data.aggregate_accel-g_last_sensor_data.aggregate_accel)/g_sensor_data.aggregate_accel );
#endif      

      do_log_output=true;
	  g_runFlags[(g_runFlags[1]?2:1)] = 'A';

    }
/*
There is something very very awkward about the distance_between math.  I suspect it doesn't work worth a damn for very small values.  So I'm putting a 20,000 limit on the distance
*/
  else {
    float distance_between = TinyGPS::distance_between(g_sensor_data.latitude, g_sensor_data.longitude, g_last_logged_sensor_data.latitude, g_last_logged_sensor_data.longitude);
      if ( distance_between < 20000 && distance_between > THRESHOLD_METRES )
      {
#ifdef SERIAL_OUTPUT
        Serial.print("logging because distance travelled (");
        Serial.print(distance_between);
        Serial.println(") > THRESHOLD_METRES");
        Serial.print("Current : (");
        Serial.print(g_sensor_data.latitude);
        Serial.print(",");
        Serial.print(g_sensor_data.longitude);
        Serial.println(")");
        Serial.print("Logged  : (");
        Serial.print(g_last_logged_sensor_data.latitude);
        Serial.print(",");
        Serial.print(g_last_logged_sensor_data.longitude);
        Serial.println(")");
#endif        
        do_log_output = true;
    	g_runFlags[(g_runFlags[1]?2:1)] = 'D';
      }
  }

//  else if ( g_sensor_data.last_position_fix !=  TinyGPS::GPS_INVALID_FIX_TIME
//       && g_sensor_data.last_position_fix > g_last_sensor_data.last_position_fix
//     ) // we've gotten a valid reading since the last one
//     {
//       if ( g_sensor_data.speed_kmph > 5.0
//       && ( abs(g_sensor_data.speed_kmph-g_last_sensor_data.speed_kmph)/g_sensor_data.speed_kmph) > SPEED_DELTA_THRESHOLD
//       )
//       {
//         Serial.println("logging because speed_kmph delta threshold");
//         do_log_output = true;
//       }
//     }

  if (do_log_output)
    log_output_forced();
}


void setup() {
  g_last_sensor_data.latitude = g_last_sensor_data.longitude = 0.0;
  g_last_sensor_data.last_position_fix = 0;
  g_runFlags[0] = 0;
  g_runFlags[1] = 0;
  g_runFlags[2] = 0;
  g_runFlags[3] = 0;

  Serial.begin(57600);
  Serial.print("Using TinyGPS library v. "); Serial.println(TinyGPS::library_version());

  Serial.println("by Mikal Hart");
  Serial.println();

  pinMode(ledPin, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

#ifdef DIRECT_SD
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
#ifdef SERIAL_OUTPUT
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
#endif
    error(2);
  }
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i / 10;
    filename[7] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename))
      break;
  }

  openLogfile();

  Serial.print("Writing to ");
  Serial.println(filename);

#endif


  logfile.print("Using TinyGPS library v. "); logfile.println(TinyGPS::library_version());
#ifdef SD_LOGGER
  smartdelay(15);
  logfile.begin(9600);
#endif
  //  logfile.println("by Mikal Hart");
  logfile.println();

#ifdef USE_10DOF_SENSOR
  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    serial_and_log_println("no LSM303 detected");
    while (1);
  }

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    serial_and_log_println("no LSM303 detected");
    while (1);
  }

  /* Enable auto-ranging */
  gyro.enableAutoRange(true);

  /* Initialise the sensor */
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    serial_and_log_println("no L3GD20 detected (gyro)");
    while (1);
  }
#endif

#ifdef USE_BMP_SENSOR
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
#ifdef SERIAL_OUTPUT
    Serial.println("no BMP180 detected");
#endif
    while (1);
  }
#endif

#ifdef DISPLAY_SENSOR_DETAILS
  displaySensorDetails();
#endif

  serial_and_log_println("");
  //  serial_and_log_println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----    deg     deg     deg      x        y     z                 C  ");
  //  serial_and_log_println("------------------------------------------------------------------------------------------------------------------------------------------------------");

#ifdef DIRECT_SD
  logfile.close();
#endif
  gpsSerial.begin(9600);
  start_gps();

  read_gps_buffer();
#ifdef CSV_HEADER_OUTPUT  
  write_csv_header();
  read_gps_buffer();
#endif  
  log_output_forced();
}

void loop()
{
  bool delay_loop = false;
  g_runFlags[0] = 0;
  g_runFlags[1] = 0;
  g_runFlags[2] = 0;
  g_runFlags[3] = 0;

  read_gyro();
  take_readings();
  bool moving = calculate_moving();

  switch (moving_state) {
    case MOVING:
      if ( ! moving ) {
        paused_millis = millis();
        moving_state = PAUSED;
#ifdef LED_INDICATORS
        analogWrite(GREEN_LED, 0);
        analogWrite(YELLOW_LED, yellow_level);
#endif
        g_runFlags[0]='X';
      }
      else {
        delay_loop = true;
        g_runFlags[0]='M';
      }

      log_output();
      break;
    case PAUSED:
      if ( moving ) {
        moving_state = MOVING;
#ifdef LED_INDICATORS
        analogWrite(YELLOW_LED, 0);
        analogWrite(GREEN_LED, green_level);
#endif
        g_runFlags[0]='g';

        log_output();
      }
      else if ( has_millis_passed(paused_millis, PAUSE_DURATION) ) {
        moving_state = STOPPED;
#ifdef LED_INDICATORS
        analogWrite(YELLOW_LED, 0);
        analogWrite(RED_LED, red_level);
#endif
        g_runFlags[0]='S';
        log_output_forced();
        stop_gps();
      }
      else {
        g_runFlags[0]='P';
        log_output();
      }
      break;
    case STOPPED:
      if ( moving ) {
        moving_state = MOVING;
#ifdef LED_INDICATORS
        analogWrite(RED_LED, 0);
        analogWrite(GREEN_LED, green_level);
#endif
        start_gps();

        g_runFlags[0]='G'; 

        log_output_forced();
      }
  }
//  if (delay_loop)
//    smartdelay(LOOP_DELAY);
//  else
    read_gps_buffer();
}

