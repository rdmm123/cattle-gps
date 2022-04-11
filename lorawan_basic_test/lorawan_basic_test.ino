#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

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

static uint8_t payload[9];
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
            SerialUSB.println(F("EV_JOINED"));
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
        static uint16_t vaca = 123;
        static uint16_t finca = 456;
        static uint8_t depto = 33;
        
        payload[0] = depto;
        
        byte fincaLow =  lowByte(finca);
        payload[1] = fincaLow;

        byte fincaHigh =  highByte(finca);
        payload[2] = fincaHigh;

        byte vacaLow =  lowByte(vaca);
        payload[3] = vacaLow;

        byte vacaHigh =  highByte(finca);
        payload[4] = vacaHigh;

        latitude = latitude / 90; // Se normaliza para que tenga rango de -1 a 1
        uint16_t payloadLatitude = LMIC_f2sflt16(latitude);
        byte latLow = lowByte(payloadLatitude);
        byte latHigh = highByte(payloadLatitude);
        payload[5] = latLow;
        payload[6] = latHigh;

        longitude = longitude / 180; // Se normaliza para que tenga rango de -1 a 1
        uint16_t payloadLongitude = LMIC_f2sflt16(longitude);
        byte lngLow = lowByte(payloadLongitude);
        byte lngHigh = highByte(payloadLongitude);
        payload[7] = lngLow;
        payload[8] = lngHigh;
        
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
//    LMIC_setDrTxpow(DR_SF9,7);
    LMIC_setDrTxpow(DR_SF7,14);

    LMIC_selectSubBand(1);

    // Start job
    do_send(&sendjob);
}

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }

    os_runloop_once();

}
