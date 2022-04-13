#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x70, 0xF3, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xDF, 0x0A, 0x41, 0xEC, 0xA6, 0xFA, 0x06, 0xE2, 0x11, 0xDD, 0xDC, 0xE1, 0x18, 0xCA, 0x8E, 0x8F };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[10];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 3,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 2,                       // reset pin
    .dio = {10, 11, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
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
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        SerialUSB.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        // Datos de prueba
        static float latitude = 11.005610;
        static float longitude = -74.791695;
      // static float longitude = 11.005610;
        static uint16_t vaca = 123;
        static uint16_t finca = 456;
        static uint8_t depto = 33;
        
        payload[0] = depto;

        byte fincaHigh =  highByte(finca);
        payload[1] = fincaHigh;
        
        byte fincaLow =  lowByte(finca);
        payload[2] = fincaLow;

        byte vacaHigh =  highByte(vaca);
        payload[3] = vacaHigh;

        byte vacaLow =  lowByte(vaca);
        payload[4] = vacaLow;

        // float normLatitude = latitude / 90; // Se normaliza para que tenga rango de -1 a 1
//        uint16_t payloadLatitude = LMIC_f2sflt16(normLatitude);
        int16_t payloadLatitude = (int) (latitude*100);
        byte latLow = lowByte(payloadLatitude);
        byte latHigh = highByte(payloadLatitude);
        payload[5] = latHigh;
        payload[6] = latLow;
        
        // float normLongitude = longitude / 180; // Se normaliza para que tenga rango de -1 a 1
//        uint16_t payloadLongitude = LMIC_f2sflt16(normLongitude);
        int16_t payloadLongitude = (int) longitude*100;
        SerialUSB.print("Latitud: ");
        SerialUSB.println(latitude);
        SerialUSB.println(payloadLatitude);
        SerialUSB.print("Longitud: ");
        SerialUSB.println(longitude);
        SerialUSB.println(payloadLongitude);
        byte lngLow = lowByte(payloadLongitude);
        byte lngHigh = highByte(payloadLongitude);
        payload[7] = lngHigh;
        payload[8] = lngLow;
        
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        SerialUSB.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
//    pinMode(13, OUTPUT);
    while (!SerialUSB); // wait for SerialUSB to be initialized
    SerialUSB.begin(115200);
    delay(100);     // per sample code on RF_95 test
    SerialUSB.println(F("Starting"));
//
//    #ifdef VCC_ENABLE
//    // For Pinoccio Scout boards
//    pinMode(VCC_ENABLE, OUTPUT);
//    digitalWrite(VCC_ENABLE, HIGH);
//    delay(1000);
//    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
//    LMIC_setDrTxpow(DR_SF9,7);
    LMIC_setDrTxpow(DR_SF7,14);

    LMIC_selectSubBand(1);
  
    // Start job
    do_send(&sendjob);
}

void loop() {
//    unsigned long now;
//    now = millis();
//    if ((now & 512) != 0) {
//      digitalWrite(13, HIGH);
//    }
//    else {
//      digitalWrite(13, LOW);
//    }

    os_runloop_once();

}
