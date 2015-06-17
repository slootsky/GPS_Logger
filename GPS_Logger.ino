#include <TinyGPS.h>

// GPS commands come from http://www.adafruit.com/datasheets/PMTK_A11.pdf

#define PAUSE_DURATION  180000  // 3 minutes

enum MOVING_STATE { MOVING, PAUSED, STOPPED };
MOVING_STATE moving_state;


// Configuration for MEGA
#define GPS_ENABLE_PIN 7
#define RED_LED 4
#define YELLOW_LED 3
#define GREEN_LED 2
#define BLUE_LED 5
#define gpsSerial  Serial1


float seaLevelPressure = 101.325;

#define SERIAL_OUTPUT
#define EXPLAIN_OUTPUT
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

#ifdef SD_LOGGER
  #define logfile Serial2
#endif


#define USE_10DOF_SENSOR
#define USE_BMP_SENSOR
// #define GPSECHO

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
  
#endif

#ifdef USE_BMP_SENSOR
  #include <Adafruit_BMP085_U.h>
  Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
#endif

#define chipSelect 10
#define ledPin 13

TinyGPS gps;



// need some millis helper functions

inline bool has_millis_passed(unsigned long since_millis, unsigned long milli_duration)
{
	return ( millis()-since_millis > milli_duration );
}

void stop_gps()
{
	// Hot Restart: Use all available data in the NV Store
	gps_write("$PMTK101");

	// Enter standby mode for power saving. 
	gps_write("$PMTK161,0");
	// ground out GPS_ENABLE_PIN
	pinMode(GPS_ENABLE_PIN, OUTPUT);
	digitalWrite(GPS_ENABLE_PIN,LOW);
}

void start_gps()
{
	// set this to INPUT to let it float (ie: not Ground)
	pinMode(GPS_ENABLE_PIN, INPUT);
	
	// Warm Restart: Don't use Ephemeris at re-start
	gps_write("PMTK102");
}

char *byte_to_hex(byte number)
{
  static char retval[3] = {0,0,0};
  if (number > 255) return NULL;
  // first break the digits out into numerical values
  retval[1] = number%16;
  retval[0] = (number-retval[2]) / 16;
  // convert numbers to ascii hex values
  for ( int i = 0 ; i++ ; i< 2 ) // yes, a loop of two
  {
    retval[i] = ( retval[i] <= 9 ? '0'+retval[i] : 'A'+retval[i]-9);
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
	if (*cmd_char == '$') cmd_char++;
	
	while ( *cmd_char )
	{
		checksum ^= *cmd_char ;  // (xor)
	}
	
	cmd_char = cmd_string;
	if (*cmd_char == '$') cmd_char++;
	
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
    if( ! logfile ) {
      #ifdef SERIAL_OUTPUT
        Serial.print("Couldn't create "); 
        Serial.println(filename);
      #endif
      error(3);
    }
  }
#endif

static void read_from_gps()
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
    read_from_gps();    
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
  read_from_gps();
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
  read_from_gps();
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
  read_from_gps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
  {
    serial_and_log_print(i<slen ? str[i] : ' ');
  }
  read_from_gps();
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

void serial_and_log_print(char c)
{
  Serial.print(c);
  logfile.print(c);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
  
}

void serial_and_log_print(char* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}  
void serial_and_log_println(char c)
{
  Serial.println(c);
  logfile.println(c);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}

void serial_and_log_println(char* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}

void serial_and_log_print(const __FlashStringHelper* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}  

void serial_and_log_println(const __FlashStringHelper* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}

void serial_and_log_print(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}

void serial_and_log_println(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartdelay(15);
#endif   
}



void setup() {
  Serial.begin(57600);
  Serial.print("Using TinyGPS library v. "); Serial.println(TinyGPS::library_version());

  Serial.println("by Mikal Hart");
  Serial.println();

  pinMode(ledPin, OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);

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
      filename[6] = '0' + i/10;
      filename[7] = '0' + i%10;
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
  #endif

  #ifdef USE_BMP_SENSOR
    if(!bmp.begin())
    {
      /* There was a problem detecting the BMP180 ... check your connections */
      #ifdef SERIAL_OUTPUT
        Serial.println("no BMP180 detected");
      #endif
      while(1);
    }
  #endif
  
  #ifdef DISPLAY_SENSOR_DETAILS
    displaySensorDetails();
  #endif  
  
//  serial_and_log_println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  heading  pitch    roll    accel                 pressure temp  ");
  serial_and_log_print("Sats,HDOP,Latitude,Longitude,FixAge,Date,Time,Alt,Course,Speed,Card");
#ifdef USE_10DOF_SENSOR
  serial_and_log_print(",heading,pitch,roll,accel_x,accel_y,accel_z");
#endif
#ifdef USE_BMP_SENSOR  
  serial_and_log_print(",pressure,temp");
#endif  
  #ifndef _GPS_NO_STATS
    serial_and_log_print(",stats_chars,stats_sentences,stats_failed");
#endif  
    serial_and_log_println("");
//  serial_and_log_println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----    deg     deg     deg      x        y     z                 C  ");
//  serial_and_log_println("------------------------------------------------------------------------------------------------------------------------------------------------------");

  #ifdef DIRECT_SD
    logfile.close();
  #endif
  gpsSerial.begin(9600);
}

void read_gyro()
{  
  
};

bool calculate_gyros_moving()
{
	return false;
};

void log_output_forced()
{
}

void log_output()
{
    //if we've moved far enough
      log_output_forced();
}

void loop()
{
  unsigned long paused_millis;

  
  	read_gyro();
	bool gyros_moving = calculate_gyros_moving();
	
	switch(moving_state){
	case MOVING:
		if ( ! gyros_moving ) {
			paused_millis = millis();
			moving_state = PAUSED;
                        #ifdef LED_INDICATORS
                          digitalWrite(GREEN_LED,0);
                          digitalWrite(YELLOW_LED,1);
                        #endif
		}
		log_output;
	case PAUSED:
		if ( gyros_moving ) {
			moving_state = MOVING;
                        #ifdef LED_INDICATORS
                          digitalWrite(YELLOW_LED,0);
                          digitalWrite(GREEN_LED,1);
                        #endif
		}
		else if ( has_millis_passed(paused_millis, PAUSE_DURATION) ) {
			moving_state = STOPPED;
                        #ifdef LED_INDICATORS
                          digitalWrite(YELLOW_LED,0);
                          digitalWrite(RED_LED,1);
                        #endif
			log_output_forced();
			stop_gps();
			}
		else {
			log_output();
		}
	case STOPPED:
		if ( gyros_moving ) {
			moving_state = MOVING;
                        #ifdef LED_INDICATORS
                          digitalWrite(RED_LED,0);
                          digitalWrite(GREEN_LED,1);
                        #endif
			start_gps();
			log_output_forced();
		}
	}
	read_from_gps();
}

