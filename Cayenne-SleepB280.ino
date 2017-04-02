/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with static payload,
   using frequency and encryption settings matching those of
   the (early prototype version of) The Things Network.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
    0.1% in g2).

   ToDo:
   - set NWKSKEY (value from staging.thethingsnetwork.com)
   - set APPKSKEY (value from staging.thethingsnetwork.com)
   - set DEVADDR (value from staging.thethingsnetwork.com)
   - optionally comment #define DEBUG
   - optionally comment #define SLEEP
   - set TX_INTERVAL in seconds
   - change mydata to another (small) static text

 *******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include "SparkFunBME280.h"
#include <lmic.h>
#include <hal/hal.h>
#include <CayenneLPP.h>

BME280 bme; // I2C

#define MAX_SIZE 200 // depends on spreading factor and frequency used
#define DEBUG
CayenneLPP Payload(MAX_SIZE);


static const PROGMEM u1_t NWKSKEY[16] = { 0x05, 0xF8, 0x27, 0x9F, 0xF0, 0x3E, 0x4E, 0xC7, 0xF1, 0xBB, 0xA0, 0x3F, 0x72, 0x78, 0x7C, 0x24 };
static const PROGMEM u1_t APPSKEY[16] = { 0xBA, 0x61, 0xBF, 0xF3, 0x95, 0xAB, 0xF8, 0xE1, 0x5B, 0x2A, 0xDB, 0x49, 0x5A, 0x85, 0xD1, 0x55 };
static const u4_t DEVADDR = 0x260117D2; //DHT Device

#define DEBUG
// use low power sleep; comment next line to not use low power sleep
#define SLEEP

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 600;

// static text, you can replace T
//static uint8_t mydata[8] = { 0x03, 0x67, 0x01, 0x10, 0x05, 0x67, 0x00, 0xFF };

#define SLEEP
#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Pin mapping Dragino Shield
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

void onEvent (ev_t ev)
{
#ifdef DEBUG
  Serial.println(F("Enter onEvent"));
#endif

  switch (ev)
  {
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
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
      next = true;
#endif

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
#ifdef DEBUG
  Serial.println(F("Leave onEvent"));
#endif

}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  /* if (LMIC.opmode & OP_TXRXPEND) {
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else */{
    // Prepare upstream data transmission at the next possible time.

    float temp;
    float pressure;
    float alt;
    float hum;

    bmeForceRead();
    Payload.reset();
    // Get temperature event and print its value.
    temp = bme.readTempC();
    pressure = bme.readFloatPressure() / 100.0F;
    alt = bme.readFloatAltitudeMeters();
    hum = bme.readFloatHumidity();

    Serial.println ("Temp:= " + String(temp) + " Pressure:=" + String(pressure) + " Humidity:=" + hum + "% Altitude:= " + String(alt));

    Payload.addTemperature(0, temp);
    Payload.addBarometricPressure (1, pressure);
    Payload.addRelativeHumidity (2, hum);
    Payload.addBarometricPressure(3, alt);

    LMIC_setTxData2(2, Payload.getBuffer(), Payload.getSize(), 0);
    Serial.println(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
  }

}
void bmeForceRead()
{

  // We set the sensor in "forced mode" to force a reading.
  // After the reading the sensor will go back to sleep mode.
  uint8_t value = bme.readRegister(BME280_CTRL_MEAS_REG);
  value = (value & 0xFC) + 0x01;
  bme.writeRegister(BME280_CTRL_MEAS_REG, value);

  // Measurement Time (as per BME280 datasheet section 9.1)
  // T_max(ms) = 1.25
  //  + (2.3 * T_oversampling)
  //  + (2.3 * P_oversampling + 0.575)
  //  + (2.4 * H_oversampling + 0.575)
  //  ~ 9.3ms, for current settings multiplied by 5 (sample rate
  delay(10 * 5);
}
void setup()
{
  Serial.begin(9600);
  Serial.println(F("Enter setup"));

  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  //bme.settings.commInterface = SPI_MODE;
  //bme.settings.chipSelectPin = 10;


  //***Operation settings*****************************//

  //runMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  bme.settings.runMode = 1; //Forced

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  bme.settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  bme.settings.filter = 4; //Lots of HW filter

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.tempOverSample = 5;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.pressOverSample = 5;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.humidOverSample = 5;

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  else
  {
    Serial.println("BME found");  
  }


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

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
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

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF10, 14);

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}

void loop() {

#ifndef SLEEP

  os_runloop_once();

#else
  extern volatile unsigned long timer0_overflow_count;

  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
#ifdef DEBUG
    Serial.print(F("Enter sleeping for "));
    Serial.print(sleepcycles);
    Serial.println(F(" cycles of 8 seconds"));
#endif
    Serial.flush(); // give the serial print chance to complete
    for (int i = 0; i < sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
#ifdef DEBUG
    Serial.println(F("Sleep complete"));
#endif
    next = false;
    // Start job
    do_send(&sendjob);
  }

#endif

}
