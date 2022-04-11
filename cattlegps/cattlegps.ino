#include <SPI.h>
// Uses LMIC libary by Thomas Telkamp and Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
// Pin mappings based upon PCB Doug Larue
#include <lmic.h>
#include <hal/hal.h>

#include <TinyGPSPlus.h>

// define the activation method ABP or OTAA
#define ACT_METHOD_ABP

// show debug statements; comment next line to disable debug statements
#define DEBUG

/* **************************************************************
* keys for device
* *************************************************************/
static const uint8_t PROGMEM NWKSKEY[16] = { 0xA1, 0x16, 0xA7, 0x51, 0x3D, 0xD4, 0x63, 0x56, 0xAD, 0x96, 0x61, 0xCD, 0xBF, 0x31, 0x52, 0x4A };
static const uint8_t PROGMEM APPSKEY[16] = { 0x7F, 0xF4, 0x75, 0x8A, 0x2D, 0xF1, 0x38, 0x2A, 0x87, 0xDA, 0x6C, 0x43, 0xC3, 0x10, 0x33, 0x73 };
static const uint32_t DEVADDR = 0x260CA563;

// Declare the job control structures
static osjob_t sendjob;

// These callbacks are only used in over-the-air activation, so they are
// left empty when ABP (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
#ifdef ACT_METHOD_ABP
  void os_getArtEui (u1_t* buf) { }
  void os_getDevEui (u1_t* buf) { }
  void os_getDevKey (u1_t* buf) { }
#else
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
#endif

/* ************************************************************** 
 * Pin mapping
 * *************************************************************/
const lmic_pinmap lmic_pins = {
    .nss = 3,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 2,                       // reset pin
    .dio = {10, 11, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

/* **************************************************************
 * user settings
 * *************************************************************/
unsigned long starttime;
unsigned long cycle_length = 15 * 60 * 1000UL; // cycle * mins_or_secs * 1000;
static uint8_t dataTX[5];

/* **************************************************************
 * setup
 * *************************************************************/
void setup() {
  // Wait (max 10 seconds) for the SerialUSB Monitor
  while ((!SerialUSB) && (millis() < 10000)){ }

  //Set baud rate
  SerialUSB.begin(9600);

  init_node();
  init_gps();

  starttime = millis();
}


/* **************************************************************
 * loop
 * *************************************************************/
void loop() {
  
  feed_gps();

  // check if need to send
  if (((millis() - starttime) > cycle_length) && (gps.location.isValid())) {
    build_data();
    do_send();
    starttime = millis();
  }
  
}


/* **************************************************************
 * sensor code, typical would be init_sensor(), do_sense(), build_data()
 * *************************************************************/
/* **************************************************************
 * init the sensor
 * *************************************************************/
void init_gps() {
  Serial1.begin(GPSBaud);
}

void feed_gps() {
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
}

/* **************************************************************
 * detect motion by ultrasound
 * *************************************************************/


/* **************************************************************
 * build data to transmit in dataTX
 * *************************************************************/
void build_data() {
    #ifdef DEBUG 
      SerialUSB.print(F("Time:"));
      SerialUSB.print(millis());
      SerialUSB.print(F(" Latitud:"));
      SerialUSB.print(gps.location.lat());
      SerialUSB.print(F(" Longitudi:"));
      SerialUSB.print(gps.location.lng());
      SerialUSB.println();
    #endif
    
    // map it to dataTX; 1 leading byte (0x00) plus 2 data bytes
  /* *************************************************
   * Suggested payload function for this data
   *
   * if (bytes[0] >= 0x20) {
   *   str = '';
   *   for (var i = 0; i < bytes.length; i += 1) str += String.fromCharCode(bytes[i]);
   *   return { payload: str };
   * }
   *
   * ************************************************/
    float lat_norm = (gps.location.lat())/90; // Se normaliza en el rango -1, 1
    float lng_norm = (gps.location.lng())/180; // Se normaliza en el rango -1, 1

    uint16_t payloadLat = LMIC_f2sflt16(lat_norm);
    uint16_t payloadLng = LMIC_f2sflt16(lng_norm);

    byte latLow = lowByte(payloadLat);
    byte latHigh = highByte(payloadLat);

    dataTX[0] = latLow;
    dataTX[1] = latHigh;

    byte lngLow = lowByte(payloadLng);
    byte lngHigh = highByte(payloadLng);

    dataTX[2] = lngLow;
    dataTX[3] = lngHigh;
}

/* **************************************************************
 * radio code, typical would be init_node(), do_send(), etc
 * *************************************************************/
/* **************************************************************
 * init the Node
 * *************************************************************/
void init_node() {
  #ifdef VCC_ENABLE
     // For Pinoccio Scout boards
     pinMode(VCC_ENABLE, OUTPUT);
     digitalWrite(VCC_ENABLE, HIGH);
     delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  #ifdef ACT_METHOD_ABP
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
      // On AVR, these values are stored in flash and only copied to RAM
      // once. Copy them to a temporary buffer here, LMIC_setSession will
      // copy them into a buffer of its own again.
      uint8_t appskey[sizeof(APPSKEY)];
      uint8_t nwkskey[sizeof(NWKSKEY)];
      memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
      memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
      LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
      // If not running an AVR with PROGMEM, just use the arrays directly
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
      // Set up the channels used by the Things Network, which corresponds
      // to the defaults of most gateways. Without this, only three base
      // channels from the LoRaWAN specification are used, which certainly
      // works, so it is good for debugging, but can overload those
      // frequencies, so be sure to configure the full frequency range of
      // your network here (unless your network autoconfigures them).
      // Setting up channels should happen after LMIC_setSession, as that
      // configures the minimal channel set.
      // NA-US channels 0-71 are configured automatically
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
      // TTN defines an additional channel at 869.525Mhz using SF9 for class B
      // devices' ping slots. LMIC does not have an easy way to define set this
      // frequency and support for class B is spotty and untested, so this
      // frequency is not configured here.
    #elif defined(CFG_us915)
      // NA-US channels 0-71 are configured automatically
      // but only one group of 8 should (a subband) should be active
      // TTN recommends the second sub band, 1 in a zero based count.
      // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
      LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
  #endif

  #ifdef ACT_METHOD_OTAA
    // got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  #endif

}

/* **************************************************************
 * send the message
 * *************************************************************/
void do_send() {

  SerialUSB.print(millis());
  SerialUSB.print(F(" Sending.. "));  

  send_message(&sendjob);

  // wait for send to complete
  SerialUSB.print(millis());
  SerialUSB.print(F(" Waiting.. "));  
 
  while ( (LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND) ) { os_runloop_once();  }
  SerialUSB.print(millis());
  SerialUSB.println(F(" TX_COMPLETE"));
}
  
/* *****************************************************************************
* send_message
* ****************************************************************************/
void send_message(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    SerialUSB.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
    LMIC_setTxData2(1, dataTX, sizeof(dataTX)-1, 0);
    SerialUSB.println(F("Packet queued"));
  }
}

/*******************************************************************************/
void onEvent (ev_t ev) {
  switch (ev) {
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
      SerialUSB.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      SerialUSB.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      SerialUSB.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      SerialUSB.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      SerialUSB.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        SerialUSB.print(F("Data Received: "));
        SerialUSB.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
        SerialUSB.println();
      }
      // schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), send_message);
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
    default:
      SerialUSB.println(F("Unknown event"));
      break;
  }
    
}
