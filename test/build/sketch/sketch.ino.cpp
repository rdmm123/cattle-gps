#include <Arduino.h>
#line 1 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
#include <TinyGPSPlus.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

#line 8 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
void setup();
#line 14 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
void loop();
#line 65 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
static void smartDelay(unsigned long ms);
#line 75 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
static void printFloat(float val, bool valid, int len, int prec);
#line 95 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
static void printInt(unsigned long val, bool valid, int len);
#line 109 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
#line 137 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
static void printStr(const char *str, int len);
#line 8 "a:\\Users\\robed\\OneDrive - Universidad del Norte\\U\\#9\\Proyecto Final\\Code\\test\\sketch\\sketch.ino"
void setup()
{
  SerialUSB.begin(115200);
  Serial1.begin(GPSBaud);
}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  SerialUSB.println(gps.location.lat());
  SerialUSB.println(gps.location.lng());

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  SerialUSB.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    SerialUSB.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      SerialUSB.print('*');
    SerialUSB.print(' ');
  }
  else
  {
    SerialUSB.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      SerialUSB.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  SerialUSB.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    SerialUSB.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    SerialUSB.print(sz);
  }
  
  if (!t.isValid())
  {
    SerialUSB.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    SerialUSB.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    SerialUSB.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

