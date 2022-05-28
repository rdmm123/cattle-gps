#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <ArduinoLowPower.h>
#include <SerialFlash.h>

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//static const PROGMEM u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
 static const PROGMEM u1_t NWKSKEY[16] = { 0xA1, 0x16, 0xA7, 0x51, 0x3D, 0xD4, 0x63, 0x56, 0xAD, 0x96, 0x61, 0xCD, 0xBF, 0x31, 0x52, 0x4A };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
 static const u1_t PROGMEM APPSKEY[16] = { 0x7F, 0xF4, 0x75, 0x8A, 0x2D, 0xF1, 0x38, 0x2A, 0x87, 0xDA, 0x6C, 0x43, 0xC3, 0x10, 0x33, 0x73 };
//static const u1_t PROGMEM APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260CA563 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t payload[12];
static osjob_t sendjob;

static uint16_t vaca = 123;
static uint16_t finca = 456;
static uint8_t depto = 33;
uint32_t payloadLat, payloadLng;

TinyGPSPlus gps;
static const int RXPin = 0, TXPin = 1; 
static const uint32_t GPSBaud = 9600;

#define PMTK_SET_STANDBY_MODE "$PMTK161,0*28"
#define PMTK_SET_FULLON_MODE "$PMTK353,1,1,0,0,0*2B"
#define PMTK_SET_ALWAYS_LOCATE_MODE "$PMTK225,8*23"

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 3600;
const int TX_INTERVAL_MS = TX_INTERVAL*1000;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 3,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 2,                       // reset pin
    .dio = {10, 11, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

void setPullUpPins() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  unsigned char pinNumber;
  for (pinNumber = 4; pinNumber <= 9; pinNumber++){
    pinMode(pinNumber, INPUT_PULLUP);
  }

  pinMode(12, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
}

void setup() {
  Serial1.begin(GPSBaud);
  Serial1.println(F(PMTK_SET_FULLON_MODE));
  // wait for SerialUSB to be initialized
  SerialUSB.begin(115200);
  delay(1000);     // per sample code on RF_95 test
  SerialUSB.println(F("Starting"));
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
  #else
   If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  // Disable link check validation
    LMIC_setLinkCheckMode(0);
  
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7,14);

  LMIC_selectSubBand(1);

  // Ahorro de energía
  SerialFlash.begin(4);
  SerialFlash.sleep();
  setPullUpPins();
  pinMode(25, OUTPUT);
  pinMode(25, LOW);
  pinMode(26, OUTPUT);
  
  // Start job
  do_send(&sendjob);
}

void displayInfo() {
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
  SerialUSB.print("Chars: ");
  SerialUSB.println(gps.charsProcessed());
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static bool fetchGPS(unsigned long maxTime) {
  unsigned long start = millis();
  while (true) {
    smartDelay(1000);
    displayInfo();
    if (gps.location.isValid()) {
      return true;
    }
    if (millis() - start > maxTime) {
      return false;
    }
  }
}

void get_coords () {
  unsigned long chars;
  unsigned short sentences, failed;
  unsigned long age;

  SerialUSB.println("pinga");
  Serial1.println(F(PMTK_SET_FULLON_MODE));
  SerialUSB.println(F(PMTK_SET_FULLON_MODE));
  delay(1000);
  bool newData = fetchGPS(10000);
  build_packet();
  Serial1.println(F(PMTK_SET_STANDBY_MODE));
  SerialUSB.println(F(PMTK_SET_STANDBY_MODE));
}

void build_packet() {
  payload[0] = depto;

  byte fincaHigh =  highByte(finca);
  payload[1] = fincaHigh;
  
  byte fincaLow =  lowByte(finca);
  payload[2] = fincaLow;

  byte vacaHigh =  highByte(vaca);
  payload[3] = vacaHigh;

  byte vacaLow =  lowByte(vaca);
  payload[4] = vacaLow;

  payloadLat = gps.location.lat() * 10000;
  payloadLng = gps.location.lng() * 10000;
  payload[5] = (payloadLat >> 16) & 0xFF;
  payload[6] = (payloadLat >> 8) & 0xFF;
  payload[7] = payloadLat & 0xFF;

  payload[8] = (payloadLng >> 16) & 0xFF;
  payload[9] = (payloadLng >> 8) & 0xFF;
  payload[10] = payloadLng & 0xFF;
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      SerialUSB.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      
      get_coords();
      
      LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
      SerialUSB.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        SerialUSB.print('0');
    SerialUSB.print(v, HEX);
}

void onEvent (ev_t ev) {
    SerialUSB.print(os_getTime());
    SerialUSB.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            SerialUSB.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            SerialUSB.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            SerialUSB.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            SerialUSB.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            SerialUSB.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              SerialUSB.print("netid: ");
              SerialUSB.println(netid, DEC);
              SerialUSB.print("devaddr: ");
              SerialUSB.println(devaddr, HEX);
              SerialUSB.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  SerialUSB.print("-");
                printHex2(artKey[i]);
              }
              SerialUSB.println("");
              SerialUSB.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              SerialUSB.print("-");
                      printHex2(nwkKey[i]);
              }
              SerialUSB.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            SerialUSB.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            SerialUSB.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            SerialUSB.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              SerialUSB.println(F("Received ack"));
            if (LMIC.dataLen) {
              SerialUSB.println(F("Received "));
              SerialUSB.println(LMIC.dataLen);
              SerialUSB.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            USBDevice.detach();
            LowPower.sleep(TX_INTERVAL_MS);
            USBDevice.attach();
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            break;
        case EV_LOST_TSYNC:
            SerialUSB.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            SerialUSB.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            SerialUSB.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            SerialUSB.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            SerialUSB.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            SerialUSB.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            SerialUSB.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            SerialUSB.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            SerialUSB.print(F("Unknown event: "));
            SerialUSB.println((unsigned) ev);
            break;
    }
}

void loop() {
    os_runloop_once();
}

void alarmMatch()
{

}
