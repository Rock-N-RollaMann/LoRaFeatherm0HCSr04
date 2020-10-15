/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.
   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.
   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.
   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!
   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.
   Do not forget to define the radio type correctly in config.h.
 *******************************************************************************/
/* 
* 
* Arduino Version: 1.8.12
* Arduino/Genuino(Native USB Port)
* Lmic 3.0.99
* Arduino SAMD Boards (32 bit Cortex m0) 1.6.4
* 
*/
 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//#include <Wire.h>
#include <RTCZero.h>
#include <SerialFlash.h>
#define Serial SerialUSB

#define echoPin A2
#define trigPin A3
#define mosfetPin A5


RTCZero rtc;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70. isb
static const u1_t PROGMEM APPEUI[8] = { 0x43, 0x5C, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above. isb
static const u1_t PROGMEM DEVEUI[8] = { 0x8E, 0x0D, 0xE9, 0x00, 0x0B, 0xA3, 0x04, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is. msb
static const u1_t PROGMEM APPKEY[16] = { 0x07, 0x7C, 0x1C, 0xB9, 0x8B, 0xDA, 0xEF, 0x66, 0x9C, 0x33, 0xBC, 0xD3, 0xFE, 0x13, 0xC6, 0x88 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[5];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 900; //900 seconds = 15 mins

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 3,
  .dio = {2, 6, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Ensure all debugging messages are sent before sleep
            /////Mode(trigPin, INPUT_PULLUP);/////
            Serial.flush();
      
            // Sleep for a period of TX_INTERVAL using single shot alarm
            rtc.setAlarmEpoch(rtc.getEpoch() + TX_INTERVAL);
            rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
            rtc.attachInterrupt(alarmMatch);
            // USB port consumes extra current
            USBDevice.detach();
            // Enter sleep mode
            rtc.standbyMode();
            // Reinitialize USB for debugging
            USBDevice.init();
            USBDevice.attach();
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;  
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j)
{
  unsigned char counter;
  float batteryVoltage;
  int adcReading;
  int voltage;

  float duration, distance;
  
  ///digitalWrite(LED_BUILTIN, HIGH);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {

    //Engage Mosfet
    digitalWrite(mosfetPin, HIGH);
    delay(70);

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 micro seconds

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds

    duration = pulseIn(echoPin, HIGH);

    //Disengage Mosfet
    digitalWrite(mosfetPin, LOW);
    
    // Calculating the distance
    
    distance = duration*0.034/2;


    //Distance in CM
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: "); 
    Serial.print(distance);
    Serial.println(" cm");

    distance = distance /100;
    
    // ***** Battery monitor connection
    //
    // VBAT-----1M-----3M3-----GND
    //              | 
    //              ---0.1uF---GND
    //              |
    //              A5   

    adcReading = analogRead(A5);
    // Discard inaccurate 1st reading
    adcReading = 0;
    // Perform averaging
    for (counter = 10; counter > 0; counter--)
    {
      adcReading += analogRead(A5);
    }
    adcReading = adcReading / 10;
    // Convert to volts
    batteryVoltage = adcReading * (4.3 / 1023.0);

    Serial.print(F("Battery: "));
    Serial.print(batteryVoltage);
    Serial.println(F(" V"));

    voltage = batteryVoltage * 100;

  uint16_t payloadDistance = LMIC_f2sflt16(distance);

  byte distanceLow = lowByte(payloadDistance);
  byte distanceHigh = highByte(payloadDistance);
  //place bytes into payload
  payload[0] = distanceLow;
  payload[1] = distanceHigh;

  //battery decoding

  uint16_t payloadVoltage = LMIC_f2sflt16(voltage);

  byte voltageLow = lowByte(payloadVoltage);
  byte voltageHigh = highByte(payloadVoltage);
  //plave the bytes into payload
  payload[2] = voltageLow;
  payload[3] = voltageHigh;

     LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.

  ///digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  int count;
  unsigned char pinNumber;

  // ***** Put unused pins into known state *****
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  // D7-D13, A0(D14)-A5(D19), SDA(D20), SCL(D21), MISO(D22)
  for (pinNumber = 7; pinNumber <= 22; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  // RX_LED (D25) & TX_LED (D26) (both LED not mounted on Mini Ultra Pro)
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  // D30 (RX) & D31 (TX) of Serial
  pinMode(30, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  // D34-D38 (EBDG Interface)
  for (pinNumber = 34; pinNumber <= 38; pinNumber++)
  {
    pinMode(pinNumber, INPUT_PULLUP);
  }
  // ***** End of unused pins state initialization *****

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  
  
  Serial.begin(9600);
  Serial.println(F("Starting"));

  // Initialize serial flash
  SerialFlash.begin(4);
  // Put serial flash in sleep
  SerialFlash.sleep();

  // Initialize RTC
  rtc.begin();
  // Use RTC as a second timer instead of calendar
  rtc.setEpoch(0);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
   LMIC_setLinkCheckMode(0);
   LMIC_setDrTxpow(DR_SF7,14);
   LMIC_selectSubBand(1);
   

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void alarmMatch()
{

}
