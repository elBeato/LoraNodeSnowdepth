/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Datum: 29.08.2020
 * Autor: Beat Furrer
 * Libraries: DallasTemperature, DHT_sensor_library, IBM_LMIC_framework, Low-Power, OneWire, RFM69_LowPowerLab, 
 * Pfad: C:\Users\beatu\OneDrive\Dokumente\Arduino\SchneeSensor_OK_V1.0
 * Arduino AVR Boards Version: 1.6.23 installiert (bei höheren Versionen ist der Speicherplatz nicht genügend gross)
 * 99% vom Speicherplatz verbraucht. (Einstellbar unter: Tools/Boardmanager/...)
 * - Umbau auf PIN 10: Schaltet Transistor ein um die Sensoren einzuschalten
 * 
 * Prog: LowPower-Modus für allfälligen deep sleep. 
 * Ideen für V1.1: PIN 10, welcher die Sensoren via Transistoren einschalten soll, Alle anderen IOs disablen oder auf pull-up schalten. 
 * falls Speicher knapp wird, Serial.Println()-Methoden löschen. 
 *
 * Do not forget to define the radio type correctly in config.h, folgende sind deaktiviert um Speicherplatz zu sparen
 * #define DISABLE_PING
 * #define DISABLE_BEACONS
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LowPower.h>
#include <DHT.h>

#define DHTPIN 8
#define DHTTYPE DHT11
#define ONE_WIRE_BUS 9

DHT dht1 = DHT(DHTPIN, DHTTYPE);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//#define DEBUG

// Variablendeklaration Temperaturmessung
float celcius = 0.0;
float exactCM = 0.0;
// Variablen definition, vorerst sind nur Zentimeter vorgesehen. 
float duration = 0.0;
float cm = 0.0;
float humi = 0.0;
float temp_humi = 0.0;
int c_up = 0;
int c_down = 0;

// Konstante PIN, welcher Signal empfängt und sendet.
const int pingPin = 7;


//******************* Konfugurationen *********************************************************************************************//
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x79, 0x66, 0x1D, 0x23, 0x16, 0x20, 0x37, 0x76, 0x20, 0xDB, 0x0D, 0x43, 0x07, 0x12, 0x0F, 0x49 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x9A, 0xDA, 0x28, 0x02, 0xAD, 0xEF, 0x6F, 0xC0, 0xDF, 0x60, 0x91, 0x7E, 0x7D, 0x18, 0xBC, 0xC3 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x2601179A ; // <-- Change this address for every node!
//*********************************************************************************************************************************//

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Beat";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 2;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

//****************** Methoden für Meteodaten ******************************************************************************************//
//Ultraschallmessung
void sendPing(){
      // The PING))) ist getriggered bei einem HIGH pulse von 5 Mikrosekunden.
    // Davor noch 2 us sicher auf LOW stellen, dadurch kann eine steigende 
    // Flanke gesichert werden. 
    // Signal für Ping))) = tout:  __-----______
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
  
    // Der gleiche Pin wird verwendet als Input Signal vom PING))): 
    // die Funktion pulseIN() ist eine Bibliothekt der Arduino Referenz
    // Sie startet die zeitliche Messung bei einem steigenden Flanke und
    // beendet diese bein einer fallenden. Return: Zeit in Mikrosekunden. 
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
}

void messungLuftfeuchte(){
    humi = dht1.readHumidity();    // Lesen der Luftfeuchtigkeit und speichern in die Variable h
    temp_humi = dht1.readTemperature(); // Lesen der Temperatur in °C und speichern in die Variable t
    #ifdef DEBUG
    Serial.println("----- Messung: DHT11 -------------------------------------");
    Serial.print("Temperature = ");
    Serial.print(temp_humi);
    Serial.println(" °C ");
    Serial.print("Humidity = ");
    Serial.print(humi);
    Serial.println(" % ");
    #endif
}

//Messungen Temperatur
void messungTemp(){
    sensors.requestTemperatures(); 
    celcius = sensors.getTempCByIndex(0);
    #ifdef DEBUG
    Serial.println("----- Messung: DS18B20 Sensor------------------------------");
    Serial.print(celcius);
    Serial.print(" °C  ");   
    #endif
}

//Messung Distanzsensor
float microsecondsToCentimeters(float microseconds, float temp) {
  float Vschall = 0;
  // Die Schallgeschwindigkeit in Luft betrÃ¤gt 343.2 m/s oder 29.15 microseconds per centimeter.
  // Die Zeit wird gemessen fÃ¼r hin und zurück, darum noch geteilt durch zwei. 
  Vschall = 10000 / ( 331.5 + ( 0.6 * temp ) );
  #ifdef DEBUG
  Serial.print("Schallgeschwindigkeit: ");
  Serial.print(Vschall);
  Serial.println("ms/cm");
  Serial.print(microseconds);
  Serial.println(" ms ");
  Serial.print((microseconds/Vschall/2));
  Serial.println(" cm exakt ");
  #endif
  return (microseconds / Vschall / 2);
}

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
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //**************************************************************************************************************************
            //Möglichkeit für tiefen stromverbrauch von RFM95W *************************************************************************
            //**************************************************************************************************************************
            //**************************************************************************************************************************
            // Schedule next transmission
            for (int i=0; i<int(TX_INTERVAL); i++) {
              // Use library from https://github.com/rocketscream/Low-Power
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            }
            //Sensoren Einschalten
            digitalWrite(10, HIGH);
            delay(2000);
            //Luftfeuchtigkeit und Temperatur messen DHT11 Sensor
            messungLuftfeuchte();
            
            //Temperaturmessung
            messungTemp();
        
            //Doppelte Temperaturmessung: 
            celcius = (celcius + temp_humi) / 2;
            c_up = (int)(celcius);
            c_down = (celcius - c_up) * 100;
            //#ifdef DEBUG
            Serial.print("Durchs. Temp: ");
            Serial.print(c_up);
            Serial.print(".");
            Serial.println(c_down);
            //#endif

            // Variablen definition, vorerst sind nur Zentimeter vorgesehen. 
            sendPing();

            // Funktionsaufruf, konvertieren in cm. 
            exactCM = microsecondsToCentimeters(duration, celcius);
            cm = (int)round(exactCM);

            #ifdef DEBUG
            Serial.print(cm);
            Serial.println(" cm gerundet");
            #endif
           
            // Start job
            do_send(&sendjob, cm, c_up, c_down, humi);
            
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //Sensoren ausschalten
            delay(2000);
            digitalWrite(10, LOW);
            break;
        case EV_LOST_TSYNC:
            //Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            //Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j, int cm, int c_up, int c_down, int humi){
    //Höhe und Temperatur skalieren damit INT versendet werden kann
    int ganzzahlHoehe = 150 - (int)(cm * 1);   //Standort = 1.5m Höhe zum Boden

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Konvertiert int to bytes
        mydata[0] = (ganzzahlHoehe & 0xFF);
        mydata[1] = (c_up & 0xFF);
        mydata[2] = (c_down & 0xFF);
        mydata[3] = ((humi * 1) & 0xFF);
        
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(9600);
    while (!Serial);  
    Serial.println("Begin");

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // Temperaturmessung 
    sensors.begin();

    //Schalter für Sensoren ein/aus
    pinMode(10, OUTPUT);  
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);

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

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob, cm, c_up, c_down, humi);
}

void loop() {
    os_runloop_once();
}
