#define M_THRESH 1
#define STOP 60000

#include <SPI.h>

//#define DIRECT_SD
#ifndef DIRECT_SD
  #define SD_LOGGER
#endif  

#ifdef DIRECT_SD
  #include <SD.h>
  
  File logfile;
#endif  

#ifdef SD_LOGGER
  #define logfile Serial2
#endif

#include <TinyGPS.h>

// #define DISPLAY_SENSOR_DETAILS
// #define GPSECHO

#define LOOP_DELAY 5000

#include <Wire.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

//#define chipSelect 10
#define ledPin 13

TinyGPS gps;
volatile bool gpsReady = false;

//  HardwareSerial &gpsSerial = Serial;
  #define gpsSerial  Serial1

// char filename[15];

boolean moving = true;
unsigned long lastMove = 0;

float flat, flon;
float temperature;
float pitch, roll, heading;
float accel_x, accel_y, accel_z;

static void smartDelay(unsigned long ms=0);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

#ifdef DISPLAY_SENSOR_DETAILS
  void displaySensorDetails(void);
#endif

void error(uint8_t errno);

void serial_and_log_print(char);
void serial_and_log_println(char);
void serial_and_log_print(char*s = 0);
void serial_and_log_println(char *s = 0);
void serial_and_log_print(const __FlashStringHelper* s);
void serial_and_log_println(const __FlashStringHelper* s);
void serial_and_log_print(float &f, int& i);
void serial_and_log_println(float &f, int& i);

void start_gps();
void stop_gps();
void checkMovement();
void writeToLog(unsigned long age, sensors_vec_t   &orientation, sensors_event_t &accel_event);

void setup()
{
  Serial.begin(57600);
  Serial.print("Using TinyGPS library v. "); Serial.println(TinyGPS::library_version());

  Serial.println("by Mikal Hart");
  Serial.println();

  pinMode(ledPin, OUTPUT);

  gpsSerial.begin(9600);


  #ifdef DIRECT_SD
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);

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
      filename[6] = '0' + i/10;
      filename[7] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) {
        break;
      }
    }
  
    openLogfile();

    Serial.print("Writing to "); 
    Serial.println(filename);
  #endif

#ifdef SD_LOGGER
  logfile.begin(9600);
  Serial.println("Allowing the SD Logger to spin up... (2 second delay)");
  smartDelay(2000);
#endif   


   logfile.print("Using TinyGPS library v. "); logfile.println(TinyGPS::library_version());
//  logfile.println("by Mikal Hart");
  logfile.println();

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    serial_and_log_println("no LSM303 detected");
    while(1);
  }

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    serial_and_log_println("no LSM303 detected");
    while(1);
  }

  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    #ifdef SERIAL_OUTPUT
      Serial.println("no BMP180 detected");
    #endif
    while(1);
  }
  
  #ifdef DISPLAY_SENSOR_DETAILS
    displaySensorDetails();
  #endif  
  
//  serial_and_log_println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  heading  pitch    roll    accel                 pressure temp  ");
  serial_and_log_println("Sats,HDOP,Latitude,Longitude,Fix,Date,Time,Alt,Course,Speed,Card,heading,pitch,roll,accel_x,accel_y,accel_z,pressure,temp");
//  serial_and_log_println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----    deg     deg     deg      x        y     z                 C  ");
//  serial_and_log_println("------------------------------------------------------------------------------------------------------------------------------------------------------");

  #ifdef DIRECT_SD
    logfile.close();
  #endif
  smartDelay();
}

#ifdef DIRECT_SD
  void openLogfile()
  {
    logfile = SD.open(filename, FILE_WRITE);
    if( ! logfile ) {
      #ifdef SERIAL_OUTPUT
        Serial.print("Couldn't create "); 
        Serial.println(filename);
      #endif
      error(3);
    }
  }
#endif

void loop()
{
  unsigned long age, date, time, chars = 0;

  #ifndef _GPS_NO_STATS  
    unsigned short sentences = 0, failed = 0;
  #endif  
//  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  sensors_event_t bmp_event;

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;


  mag.getEvent(&mag_event);
  accel.getEvent(&accel_event);
  if (!dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    orientation.heading = -1.0;
    orientation.roll = -1.0;
    orientation.pitch = -1.0;
  }
  
  accel_x = accel_event.acceleration.x;
  accel_y = accel_event.acceleration.y;
  accel_z = accel_event.acceleration.z;

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in Celcius */
    bmp.getTemperature(&temperature);
//    /* Convert atmospheric pressure, SLP and temp to altitude    */
//  #ifdef SERIAL_OUTPUT

//    Serial.print("Alt: ");
//    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        bmp_event.pressure,
//                                        temperature)); 
//    Serial.print(" m; ");
//    /* Display the temperature */
//    Serial.print("Temp: ");
//    Serial.print(temperature);
//    Serial.print(" C");
//#endif
  }

  checkMovement();
  
  
  if ( moving  ) {
    writeToLog(age, orientation, accel_event);
  }
//  smartDelay(LOOP_DELAY);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      gpsReady = gps.encode(c);
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
  smartDelay(0);
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
  smartDelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if ( day == 0)
  {
    serial_and_log_print("**********,******** ");
  }
  else
  {
    char sz[32];

    sprintf(sz, "%02d/%02d/%04d,%02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    serial_and_log_print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartDelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
  {
    serial_and_log_print(i<slen ? str[i] : ' ');
  }
  smartDelay(0);
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
  sensor_t accel_sensor;
  accel.getSensor(&accel_sensor);

  sensor_t mag_sensor;
  mag.getSensor(&mag_sensor);

  
  #ifdef SERIAL_OUTPUT
  Serial.println("------------------------------------");
  #endif

  serial_and_log_print  ("Accel Sensor:       "); serial_and_log_println(accel_sensor.name);
  serial_and_log_print  ("Accel Driver Ver:   "); serial_and_log_println(accel_sensor.version);
  serial_and_log_print  ("Accel Unique ID:    "); serial_and_log_println(accel_sensor.sensor_id);
  serial_and_log_print  ("Accel Max Value:    "); serial_and_log_print(accel_sensor.max_value); serial_and_log_println(" m/s^2");
  serial_and_log_print  ("Accel Min Value:    "); serial_and_log_print(accel_sensor.min_value); serial_and_log_println(" m/s^2");
  serial_and_log_print  ("Accel Resolution:   "); serial_and_log_print(accel_sensor.resolution); serial_and_log_println(" m/s^2");  

  serial_and_log_print  ("Mag   Sensor:       "); serial_and_log_println(mag_sensor.name);
  serial_and_log_print  ("Mag   Driver Ver:   "); serial_and_log_println(mag_sensor.version);
  serial_and_log_print  ("Mag   Unique ID:    "); serial_and_log_println(mag_sensor.sensor_id);
  serial_and_log_print  ("Mag   Max Value:    "); serial_and_log_print(mag_sensor.max_value); serial_and_log_println(" uT");
  serial_and_log_print  ("Mag   Min Value:    "); serial_and_log_print(mag_sensor.min_value); serial_and_log_println(" uT");
  serial_and_log_print  ("Mag   Resolution:   "); serial_and_log_print(mag_sensor.resolution); serial_and_log_println(" uT");  

  #ifdef SERIAL_OUTPUT
  Serial.println("------------------------------------");
  #endif
}

void serial_and_log_print(char c)
{
  Serial.print(c);
  logfile.print(c);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
  
}

void serial_and_log_print(char* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}  
void serial_and_log_println(char c)
{
  Serial.println(c);
  logfile.println(c);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}

void serial_and_log_println(char* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}

void serial_and_log_print(const __FlashStringHelper* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}  

void serial_and_log_println(const __FlashStringHelper* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}

void serial_and_log_print(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}

void serial_and_log_println(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartDelay(15);
#endif   
}

void start_gps() {
// TODO Fill this in
}
 
void stop_gps() {
// TODO Fill this in
}

void checkMovement() {
  long acc;
  {
   int x = abs(accel_x*accel_x);
   int y = abs(accel_y*accel_y);
   int z = abs((accel_z-10.0)*(accel_z-10.0));

   acc = (long)x*x + (long)y*y + (long)z*z;
   Serial.print("x: "); Serial.print(accel_x);
   Serial.print(" y: "); Serial.print(accel_y);
   Serial.print(" z: "); Serial.print(accel_z);
   Serial.print(" acc :"); Serial.print(acc);
   
   Serial.println("");
  }
   if (acc > M_THRESH) {
       lastMove = millis();
       if (! moving) {
           start_gps();
//           start_sd();
           // Ignore data that has overflowed
//           gpsSerial.overflow();
       }
       moving = true;
       Serial.println("Moving");
   } else {
       if (moving) {
           unsigned long now = millis();
           // TODO: check for the overflow condition of millis 
           // (or reboot the arduino every 49 days <img src="http://www.open-electronics.org/wp-includes/images/smilies/icon_smile.gif" alt=":)" class="wp-smiley"> )
           if (now-lastMove > STOP) {
               moving = false;
               Serial.println("NOT Moving");
               stop_gps();
//               if(sd_available) {
//                   log_file.write("\r\n$PXXXX,Device is idle, stop logging\r\n");
//               } else {
//                   Serial.println("\r\n$PXXXX,Device is idle, stop logging\r\n");
//               }
           }
       }
   }
}

void writeToLog(unsigned long age, sensors_vec_t   &orientation, sensors_event_t &accel_event)
{
  #ifndef _GPS_NO_STATS  
    unsigned long chars = 0;
    unsigned short sentences, failed;
  #endif
  #ifdef DIRECT_SD
    openLogfile();  
  #endif  

  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_str(",",1);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  print_str(",",1);
  gps.f_get_position(&flat, &flon, &age);
  print_str(",",1);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_str(",",1);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_str(",",1);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_str(",",1);
  print_date(gps);
  print_str(",",1);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_str(",",1);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(",",1);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(",",1);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_str(",",1);
//  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
//  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
//  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

   print_float(orientation.heading,-1,9,3);
  print_str(",",1);
   print_float(orientation.pitch,-1,9,3);
  print_str(",",1);
   print_float(orientation.roll,-1,9,3);
  print_str(",",1);

   print_float(accel_event.acceleration.x,-1,9,3);
  print_str(",",1);
   print_float(accel_event.acceleration.y,-1,9,3);
  print_str(",",1);
   print_float(accel_event.acceleration.z,-1,9,3);
  print_str(",",1);
   

  #ifndef _GPS_NO_STATS
    gps.stats(&chars, &sentences, &failed);
    print_int(chars, 0xFFFFFFFF, 6);
  print_str(",",1);
    print_int(sentences, 0xFFFFFFFF, 10);
  print_str(",",1);
    print_int(failed, 0xFFFFFFFF, 9);
  #endif  

  serial_and_log_println("");

  #ifdef DIRECT_SD
  //  logfile.flush();
    logfile.close();
  #else
    logfile.flush();
  #endif
}
