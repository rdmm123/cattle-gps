#include <TinyGPSPlus.h>

/* 
   This sample sketch should be the first you try out when you are testing a TinyGPSPlus
   (TinyGPSPlus) installation.  In normal use, you feed TinyGPSPlus objects characters from
   a SerialUSBUSB NMEA GPS device, but this example uses static strings for simplicity.
*/

static const int RXPin = 0, TXPin = 1; 
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup()
{
  SerialUSB.begin(115200);
  Serial1.begin(GPSBaud);
}

void loop()
{
  SerialUSB.print("Latitud: ");
  SerialUSB.println(gps.location.lat()); 
  SerialUSB.print("Longitud: ");
  SerialUSB.println(gps.location.lng());
  SerialUSB.println(gps.location.isValid());
  SerialUSB.print("Fecha y Hora: ");
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", gps.date.month(), gps.date.day(), gps.date.year());
  SerialUSB.print(sz);
  sprintf(sz, "%02d:%02d:%02d ", gps.time.hour(), gps.time.minute(), gps.time.second());
  SerialUSB.println(sz);
  
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
