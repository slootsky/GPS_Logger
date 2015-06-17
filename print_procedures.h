#ifndef PRINT_PROCEDURES
#define PRINT_PROCEDURES

void serial_and_log_print(char c)
{
  Serial.print(c);
  logfile.print(c);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
  
}

void serial_and_log_print(char* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}  
void serial_and_log_println(char c)
{
  Serial.println(c);
  logfile.println(c);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}

void serial_and_log_println(char* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}

/*
void serial_and_log_print(const __FlashStringHelper* s)
{
  Serial.print(s);
  logfile.print(s);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}  

void serial_and_log_println(const __FlashStringHelper* s)
{
  Serial.println(s);
  logfile.println(s);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}
*/
void serial_and_log_print(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
}

void serial_and_log_println(float &f, int& i)
{
  Serial.print(f,i);
  logfile.print(f,i);
#ifdef SD_LOGGER
 smartdelay(15);
#else
  read_gps_buffer();
#endif   
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

  read_gps_buffer();
}

static void print_int(int val, int invalid, int len)
{
  char sz[len<8?8:len];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%d", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  serial_and_log_print(sz);

  read_gps_buffer();
}

static void print_ulong(unsigned long val, unsigned long invalid, int len)
{
  char sz[len<32?32:len];
  if (val == invalid)
  {
    for (int i = 0 ; i < len ; i++ )
      sz[i]='*';
  }    
  else
  {
    sprintf(sz, "%ld", val);
    for (int i=strlen(sz); i<len; ++i)
      sz[i] = ' ';
    if (len > 0) 
      sz[len-1] = ' ';
  }  
  sz[len] = 0;

  serial_and_log_print(sz);

  read_gps_buffer();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  if ( day == 0)
  {
    serial_and_log_print(",**********,********");
  }
  else
  {
    char sz[32];

    sprintf(sz, ",%02d/%02d/%04d,%02d:%02d:%02d",
        month, day, year, hour, minute, second);
    serial_and_log_print(sz);
  }

  read_gps_buffer();
}

static void print_str(const char *str, int len=-1)
{
  int slen = strlen(str);
  for (int i=0; i<(len<0?slen:len); ++i)
  {
    serial_and_log_print(i<slen ? str[i] : ' ');
  }

  read_gps_buffer();
}

#endif
