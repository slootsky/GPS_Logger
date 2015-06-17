#define notF  // This define macro is used instead of F to easily turn off F

#define USE_MAG_SENSOR
#define USE_ACCEL_SENSOR
//#define USE_GPS_SENSOR
#define LOG_LOOP_MILLIS  1000

//SERIAL_DEBUGGING #define SERIAL_DEBUGGING 5

#include <SPI.h>

#ifdef USE_GPS_SENSOR
#include <Adafruit_GPS.h>
#endif

#include <SoftwareSerial.h>
#include <SD.h>
//#include <avr/sleep.h>

#ifdef USE_MAG_SENSOR || USE_ACCEL_SENSOR
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#endif

// const char *comma = notF(" , ");
// const char *dashes = F("------------------------------------");
// const char *empty_string = notF("");
const char no_LSM303_detected[] = "Ooops, no LSM303 detected ... Check your wiring!";
//const char *csvHeader = notF("mX , mY , mZ , aX , aY , aZ, hour, minute, second, millisecond, day, month, year, fix, quality, lat, long, speed, angle, altitude, num_satelites");
//const char *ready = notF("Ready!");

/* Assign a unique ID to this sensor at the same time */
#ifdef USE_ACCEL_SENSOR
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
#endif

#ifdef USE_MAG_SENSOR
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
#endif

uint32_t timer = millis();


// Ladyada's logger modified by Bill Greiman to use the SdFat library
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS Shield
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada
// Fllybob added 10 sec logging option
SoftwareSerial mySerial(8, 7);
#ifdef USE_GPS_SENSOR
Adafruit_GPS GPS(&mySerial);
#endif

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false  

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Set the pins used
#define chipSelect 10
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
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
//SERIAL_DEBUGGING #endif  
  delay(500);
}


void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
//#if SERIAL_DEBUGGING > 0
  Serial.println(F("\r\nUltimate GPSlogger Shield"));
//#endif  
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
//    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
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
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 0    
    Serial.print(F("Couldn't create ")); 
    Serial.println(filename);
//SERIAL_DEBUGGING #endif    
    error(3);
  }
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 2
  Serial.print(F("Writing to ")); 
  Serial.println(filename);
//SERIAL_DEBUGGING #endif  

#ifdef USE_GPS_SENSOR
  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
#endif

#ifdef USE_MAG_SENSOR
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 0    
    Serial.println(no_LSM303_detected);
//SERIAL_DEBUGGING #endif    

    while(1);
  }
#endif

#ifdef USE_ACCEL_SENSOR
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 0     
    Serial.println(no_LSM303_detected);
//SERIAL_DEBUGGING #endif    
    while(1);
  }
#endif

  displaySensorDetails();
  
#ifdef USE_MAG_SENSOR
  logfile.print(F("mX , mY , mZ , "));
#endif
#ifdef USE_ACCEL_SENSOR
  logfile.println(F("aX , aY , aZ, "));
#endif  
#ifdef USE_GPS_SENSOR
  logfile.println(F("hour, minute, second, millisecond, day, month, year, fix, quality, lat, long, speed, angle, altitude, num_satelites"));
#endif  
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 3
  Serial.println(F("Ready!"));
//SERIAL_DEBUGGING #endif  
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c;
#ifdef USE_GPS_SENSOR
  c = GPS.read();
#else
  c = 0;
#endif
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
#ifdef USE_GPS_SENSOR  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      ;
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 1      
      if (c) Serial.print(c);
//SERIAL_DEBUGGING #endif      
  }
#endif  

#ifdef USE_MAG_SENSOR  
  sensors_event_t mag_event; 
  mag.getEvent(&mag_event);
#endif

#ifdef USE_ACCEL_SENSOR
  sensors_event_t accel_event; 
  accel.getEvent(&accel_event);
#endif

  if (
#ifdef USE_GPS_SENSOR  
  GPS.newNMEAreceived()
#else
  true
#endif  
  ) {


    // if a sentence is received, we can check the checksum, parse it...

    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    
    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
#ifdef USE_GPS_SENSOR    
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
#endif

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > LOG_LOOP_MILLIS) { 
    timer = millis(); // reset the timer

//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 4
    /* Display the results (acceleration is measured in m/s^2) */
#ifdef USE_MAG_SENSOR    
    Serial.print(F("  mX: ")); Serial.print(mag_event.magnetic.x); //Serial.print("  ");
    Serial.print(F("  mY: ")); Serial.print(mag_event.magnetic.y); //Serial.print("  ");
    Serial.print(F("  mZ: ")); Serial.print(mag_event.magnetic.z); //Serial.print("  ");
    Serial.println(F(" uT"));
    logfile.print(mag_event.magnetic.x); logfile.print(" , ");
    logfile.print(mag_event.magnetic.y); logfile.print(" , ");  
    logfile.print(mag_event.magnetic.z); logfile.print(" , ");  
#endif
#ifdef USE_ACCEL_SENSOR
    Serial.print(F("  aX: ")); Serial.print(accel_event.acceleration.x); //Serial.print("  ");
    Serial.print(F("  aY: ")); Serial.print(accel_event.acceleration.y); //Serial.print("  ");
    Serial.print(F("  aZ: ")); Serial.print(accel_event.acceleration.z); //Serial.print("  ");
    Serial.println(F(" m/s^2 "));
    logfile.print(accel_event.acceleration.x); logfile.print(" , ");
    logfile.print(accel_event.acceleration.y); logfile.print(" , ");  
    logfile.print(accel_event.acceleration.z); logfile.print(" , ");  
#endif
//SERIAL_DEBUGGING #endif   // SERIAL_DEBUGGING    
    

//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 5    
#ifdef USE_GPS_SENSOR
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
	

    logfile.print(" , ");
    logfile.print(GPS.hour, DEC); 
    logfile.print(" , ");
    logfile.print(GPS.minute, DEC); 
    logfile.print(" , ");
    logfile.print(GPS.seconds, DEC); 
    logfile.print(" , ");
    logfile.println(GPS.milliseconds);
    logfile.print(" , ");
    logfile.print(GPS.day, DEC); 
    logfile.print(" , ");
    logfile.print(GPS.month, DEC); 
    logfile.print(" , ");
    logfile.println(GPS.year, DEC);
    logfile.print(" , "); 
    logfile.print((int)GPS.fix);
    logfile.print(" , "); 
    logfile.println((int)GPS.fixquality); 
    if (GPS.fix) 
	{
/*      Serial.print("Location: ");
      logfile.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      logfile.print(GPS.longitude, 4); Serial.println(GPS.lon);
      logfile.print("Location (in degrees, works with Google Maps): ");
*/      
      logfile.print(GPS.latitudeDegrees, 4);
      logfile.print(" , "); 
      logfile.print(GPS.longitudeDegrees, 4);
      logfile.print(" , "); 
      
      //Serial.print("Speed (knots): "); 
      logfile.println(GPS.speed);
      logfile.print(" , "); 
      logfile.print(GPS.angle);
      logfile.print(" , "); 
      logfile.print(GPS.altitude);
      logfile.print(" , "); 
      logfile.print((int)GPS.satellites);
    }
    else
      logfile.print(" no fix ");
#endif    
//SERIAL_DEBUGGING #endif
    
    logfile.println("");
    logfile.flush();
    
  }
    // Sentence parsed! 
//    Serial.println("OK");
//    if (LOG_FIXONLY && !GPS.fix) {
//      Serial.print("No Fix");
//      return;
//    }

    // Rad. lets log it!
//    Serial.println("Log");

/*
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
*/    
//SERIAL_DEBUGGING #if SERIAL_DEBUGGING > 1
/// EXTRA CARRIAGE RETURN    Serial.println("");
//SERIAL_DEBUGGING #endif    
  }
}




/* End code */

