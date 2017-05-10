/*******************************************************************************
  REAL ONE!!!!
  Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example will send Temperature and Humidity
   using frequency and encryption settings matching those of
   the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)

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

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7 /* Digitalport Pin 2 definieren */


#include "LowPower.h"

#include <CayenneLPP.h>

#define DS18B20
#define MAX_SIZE 200 // depends on spreading factor and frequency used
#define DEBUG
CayenneLPP Payload(MAX_SIZE);


#ifdef DS18B20
OneWire ds(ONE_WIRE_BUS); /* Ini oneWire instance*/

#endif


#include <Arduino.h>

int sleepcycles = 1;//600/8;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 6     // pin 13 LED is not used, because it is connected to the SPI port

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t DEVEUI[8]  = { 0x06, 0x05, 0x04, 0x03, 0x02, 0x00, 0x40, 0x88 };
static const u1_t APPEUI[8] = { 0xF4, 0x3E, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.

static const u1_t APPKEY[16] = { 0x98, 0xE9, 0xF4, 0x45, 0xDC, 0x05, 0x58, 0x93, 0x5F, 0x49, 0x4D, 0x90, 0x40, 0x00, 0x74, 0xDD };

void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
// Pin mapping
/*const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
  };
*/
/*
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = 0, //LMIC_UNUSED_PIN,
  .rst = 0,
  .dio = {4, 5, 7},
  };
  const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
  };*/

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  int i, j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      // Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      // Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin, HIGH);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times
        Serial.print(F("!!!Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
        i = (LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        sleepcycles = i;

        if (i > 10)
          i = 11;

        for (j = 0; j < i; j++)
        {
          digitalWrite(LedPin, HIGH);
          delay(200);
          digitalWrite(LedPin, LOW);
          delay(400);
        }

      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      digitalWrite(LedPin, HIGH);
      delay(500);
      digitalWrite(LedPin, LOW);
      delay(500);
      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

// initial job
static void initfunc (osjob_t* j) {
  // reset MAC state
  LMIC_reset();
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  /* if (LMIC.opmode & OP_TXRXPEND) {
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else */{
    // Prepare upstream data transmission at the next possible time.
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius, fahrenheit;

    float temp;
    int volt;

#ifdef DS18B20
    ds.reset_search();
    if ( !ds.search(addr)) {
      Serial.println("No more addresses.");
      Serial.println();
      ds.reset_search();
      delay(250);
      return;
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
    }
    switch (addr[0]) {
      case 0x10:
        type_s = 1;
        break;
      case 0x28:
        type_s = 0;
        break;
      case 0x22:
        type_s = 0;
        break;
      default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();

    }

    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    temp = (float)raw / 16.0;

    Payload.reset();

    // Get temperature event and print its value.
    Serial.print("Grad Celsius: ");
    Serial.println(temp);
    delay(500);
    volt = analogRead(A0);
    Serial.print("Spannung: ");
    Serial.println(volt);
    Payload.addTemperature(0, temp);
    Payload.addAnalogInput(1, (float)volt / 100);

    LMIC_setTxData2(2, Payload.getBuffer(), Payload.getSize(), 0);
    Serial.println(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
#else
    Payload.reset();
    // Get temperature event and print its value.
    temp = 77;
    Payload.addTemperature(0, temp);
    volt = 220;
    Payload.addAnalogInput(1, volt);

    LMIC_setTxData2(2, Payload.getBuffer(), Payload.getSize(), 0);
    Serial.println(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
#endif
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println(F("Enter setup"));

  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
#ifdef DS18B20
  Serial.println("Temperatur Messprogramm");
  Serial.println("Starte Temperatur abfragen ...");

#endif
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.


  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  /*LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF10, DR_SF7), BAND_CENTI);      // g-band
     LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_SF8,  DR_SF8),  BAND_CENTI);      // g2-band
     // TTN defines an additional channel at 869.525Mhz using SF9 for class B
     // devices' ping slots. LMIC does not have an easy way to define set this
     // frequency and support for class B is spotty and untested, so this
     // frequency is not configured here.
  */

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setClockError(MAX_CLOCK_ERROR * 1.5 / 100);


  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}

unsigned long time;
void loop()
{

  // start OTAA JOIN
  if (joined == false)
  {

    os_runloop_once();

  }
  else
  {
    do_send(&sendjob);    // Sent sensor values
    while (sleeping == false)
    {
      os_runloop_once();
    }
    sleeping = false;
    for (int i = 0; i < sleepcycles; i++)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
    }
  }

  digitalWrite(LedPin, ((millis() / 100) % 2) && (joined == false)); // only blinking when joining and not sleeping
}

