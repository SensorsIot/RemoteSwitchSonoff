/* This sketch monitors a PIR sensor and transmitts an "ON" or "OFF" signal to a receiving device. Together with a
   Sonoff wireless switch, it can be used as a wireless motion detector. At startup, it connects to IOTappStory.com to
   check for updates.

   To add new constants in WiFiManager search for "NEW CONSTANTS" and insert them according the "boardName" example

  Copyright (c) [2016] [Andreas Spiess]

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/*
   LED signals of green LED:
   ---------------------------
   Setup till done: Blink
   ON: green LED completely on
   OFF: Green LED blinking with very short on-time
   Setup: very fast blinking green LED

*/

#define SKETCH "SonoffSender "
#define VERSION "V1.1"
#define FIRMWARE SKETCH VERSION

#define SERIALDEBUG       // Serial is used to present debugging messages 
#define REMOTEDEBUGGING       // telnet is used to present

#define LEDS_INVERSE   // LEDS on = GND

#include <credentials.h>
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>        //https://github.com/kentaylor/WiFiManager
#include <Ticker.h>
#include <EEPROM.h>

#ifdef REMOTEDEBUGGING
#include <WiFiUDP.h>
#endif

extern "C" {
#include "user_interface.h" // this is for the RTC memory read/write functions
}


//--------  Sketch Specific -------
// #include <ESP8266WebServer.h>
// #include <ESP8266HTTPClient.h>


// -------- PIN DEFINITIONS ------------------
#ifdef ARDUINO_ESP8266_ESP01           // Generic ESP's 
#define MODEBUTTON 0
#define LEDgreen 13
// #define LEDred 12
#else
#define MODEBUTTON D3
#define LEDgreen D7
// #define LEDred D6
#endif

// --- Sketch Specific -----
#ifdef ARDUINO_ESP8266_ESP01           // Generic ESP's 
#define PIRpin 14
#else
#define PIRpin D5
#endif


//---------- DEFINITIONS for SKETCH ----------
#define STRUCT_CHAR_ARRAY_SIZE 50  // length of config variables
#define MAX_WIFI_RETRIES 50
#define RTCMEMBEGIN 68
#define MAGICBYTE 85

// --- Sketch Specific -----
#define SERVICENAME "SONOFF"  // name of the MDNS service used in this group of ESPs
#define MAXDEVICES 5

//-------- SERVICES --------------
Ticker blink;

#ifdef REMOTEDEBUGGING
WiFiUDP UDP;
#endif

// --- Sketch Specific -----
HTTPClient http;

//WiFiClient client;


//--------- ENUMS AND STRUCTURES  -------------------


typedef struct {
  char ssid[STRUCT_CHAR_ARRAY_SIZE];
  char password[STRUCT_CHAR_ARRAY_SIZE];
  char boardName[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStory1[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStoryPHP1[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStory2[STRUCT_CHAR_ARRAY_SIZE];
  char IOTappStoryPHP2[STRUCT_CHAR_ARRAY_SIZE];
  char switchName1[STRUCT_CHAR_ARRAY_SIZE];
  char switchName2[STRUCT_CHAR_ARRAY_SIZE];
  char udpPort[5];
  // insert NEW CONSTANTS according boardname example HERE!
  char magicBytes[4];
} strConfig;

strConfig config = {
  mySSID,
  myPASSWORD,
  "SenderINIT",
  "iotappstory.org",
  "/ota/esp8266-v1.php",
  "iotappstory.org",
  "/ota/esp8266-v1.php",
  "",
  "",
  "8005",
  "CFG"  // Magic Bytes
};

typedef struct {
  byte markerFlag;
  int bootTimes;
} rtcMemDef __attribute__((aligned(4)));
rtcMemDef rtcMem;

// --- Sketch Specific -----
enum statusDef {
  POWER_OFF,
  POWER_ON,
  Renew
} loopStatus;

//---------- VARIABLES ----------

String switchName1, switchName2, boardName, IOTappStory1, IOTappStoryPHP1, IOTappStory2, IOTappStoryPHP2;
unsigned long debugEntry;
volatile unsigned long buttonEntry;
unsigned long buttonTime;
volatile bool buttonChanged = false;
volatile int greenTimesOff = 0;
volatile int redTimesOff = 0;
volatile int greenTimes = 0;
volatile int redTimes = 0;
char boardMode = 'N';  // Normal operation or Configuration mode?

#ifdef REMOTEDEBUGGING
// UDP variables
IPAddress broadcastIp(255, 255, 255, 255);
#endif


// --- Sketch Specific -----
// String xx; // add NEW CONSTANTS for WiFiManager according the variable "boardname"

bool PIRstatus;
IPAddress sonoffIP[10];
String deviceName[30];
unsigned long dnsRefreshEntry = 0;
unsigned long renewEntry;
int discoverCount = 0;  // # times DNS discover found no results



//---------- FUNCTIONS ----------
// to help the compiler, sometimes, functions have  to be declared here
void loopWiFiManager(void);
void readFullConfiguration(void);
bool readRTCmem(void);
void printRTCmem(void);
void initialize(void);
void showLoopState(statusDef);
void discovermDNSServices(void);
bool switchAllSonoffs(bool);
bool switchSonoff(bool, String);


//---------- OTHER .H FILES ----------
#include <ESP_Helpers.h>           // General helpers for all IOTappStory sketches
#include "IOTappStoryHelpers.h"    // Sketch specific helpers for all IOTappStory sketches




//-------------------------- SETUP -----------------------------------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Start "FIRMWARE);
  UDPDEBUG_START();
  UDPDEBUG_PRINTTXT("Start ");
  UDPDEBUG_PRINTTXT(FIRMWARE);
  UDPDEBUG_SEND();


  // ----------- PINS ----------------
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection
#ifdef LEDgreen
  pinMode(LEDgreen, OUTPUT);
  digitalWrite(LEDgreen, LEDOFF);
#endif
#ifdef LEDred
  pinMode(LEDred, OUTPUT);
  digitalWrite(LEDred, LEDOFF);
#endif

  // --- Sketch Specific -----
  pinMode(PIRpin, INPUT_PULLUP);


  // ------------- INTERRUPTS ----------------------------
  attachInterrupt(MODEBUTTON, ISRbuttonStateChanged, CHANGE);
  blink.detach();

  //------------- LED and DISPLAYS ------------------------
  LEDswitch(GreenBlink);


  // --------- BOOT STATISTICS ------------------------
  // read and increase boot statistics (optional)
  readRTCmem();
  rtcMem.bootTimes++;
  writeRTCmem();
  printRTCmem();



  //---------- SELECT BOARD MODE -----------------------------

  system_rtc_mem_read(RTCMEMBEGIN + 100, &boardMode, 1);   // Read the "boardMode" flag RTC memory to decide, if to go to config
  if (boardMode == 'C') configESP();

  DEBUG_PRINTLN("------------- Normal Mode -------------------");

  readFullConfiguration();

  // --------- START WIFI --------------------------

  connectNetwork('N');

  UDPDEBUG_START();
  UDPDEBUG_PRINTTXT("------------- Normal Mode -------------------");
  UDPDEBUG_SEND();

  IOTappStory();

  // ----------- SPECIFIC SETUP CODE ----------------------------
  discoverMDNS();
  switchAllSonoffs(false);
  loopStatus = POWER_OFF;
  // ----------- END SPECIFIC SETUP CODE ----------------------------

  LEDswitch(None);
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection

  DEBUG_PRINTLN("Setup done");
  UDPDEBUG_START();
  UDPDEBUG_PRINTTXT("Setup done");
  UDPDEBUG_SEND();
}


//--------------- LOOP ----------------------------------
void loop() {
  //-------- IOTappStory Block ---------------
  yield();
  handleModeButton();   // this routine handles the reaction of the Flash button. If short press: update of skethc, long press: Configuration

  // Normal blind (1 sec): Connecting to network
  // fast blink: Configuration mode. Please connect to ESP network
  // Slow Blink: IOTappStore Update in progress

  // ------- Debug Message --------
  if ((millis() - debugEntry) > 1000) {
    debugEntry = millis();
    sendDebugMessage();
  }


  //-------- Your Sketch ---------------

  discoverMDNS();

  PIRhandler();

}
// ------------------------- END LOOP ----------------------------------


void sendDebugMessage() {
  DEBUG_PRINT("Board: ");
  DEBUG_PRINT(config.boardName);
  DEBUG_PRINT(" Firmware: ");
  DEBUG_PRINT(FIRMWARE);
  DEBUG_PRINT(" Heap ");
  DEBUG_PRINT(ESP.getFreeHeap());
  DEBUG_PRINT(" LoopStatus: ");
  DEBUG_PRINT(loopStatus);
  if (PIRstatus) DEBUG_PRINTLN(" PIRstatus: ON ");
  else DEBUG_PRINTLN(" PIRstatus: OFF ");


  UDPDEBUG_START();
  UDPDEBUG_PRINTTXT("Board: ");
  UDPDEBUG_PRINTTXT(config.boardName);
  UDPDEBUG_PRINTTXT(" Firmware: ");
  UDPDEBUG_PRINTTXT(FIRMWARE);
  long h1 = ESP.getFreeHeap();
  UDPDEBUG_PRINT(" Heap ", h1);
  UDPDEBUG_PRINT(" LoopStatus: ", loopStatus);
  if (PIRstatus) UDPDEBUG_PRINTTXT(" PIRstatus: ON ");
  else UDPDEBUG_PRINTTXT(" PIRstatus: OFF ");
  UDPDEBUG_SEND();
}



void PIRhandler() {
  PIRstatus = digitalRead(PIRpin);

  switch (loopStatus) {
    case POWER_ON:        // Lamp is on
      LEDswitch(Green);

      // exit criteria
      if (PIRstatus == LOW) {
        Serial.println("0");
        loopStatus = POWER_OFF;
        LEDswitch(None);
        showLoopState(loopStatus);
      }
      else if (millis() - renewEntry > 10000) {  // every 10 sec
        Serial.println("1");
        loopStatus = Renew;
        showLoopState(loopStatus);
      }
      break;

    case Renew:       // Lamp is on, send additional message
      switchAllSonoffs(true);
      renewEntry = millis();

      // exit criteria
      loopStatus = POWER_ON;
      showLoopState(loopStatus);
      break;

    case POWER_OFF:       // Lamp is off

      // exit criteria
      if (PIRstatus == HIGH) {
        switchAllSonoffs(true);
        loopStatus = POWER_ON;
        showLoopState(loopStatus);
        debugEntry = millis();
      }
      break;

    default:
      break;
  }
}

void discoverMDNS() {
  if (MDNS.hostname(0) == "" || (millis() - dnsRefreshEntry > 60000)) { // every minute or if no device detected
    discoverCount = 0;
    do {
      discovermDNSServices();
      // for (int i = 0; i < 5; i++) DEBUG_PRINTLN(sonoffIP[i]);
      discoverCount++;
    } while (MDNS.hostname(0) == "");
    dnsRefreshEntry = millis();
    if (discoverCount > 10) {
      LEDswitch(RedFastBlink);
      delay(2000);
      ESP.restart();    // restart if no services available
    }
  }
}


void showLoopState(statusDef loopStatus) {
  DEBUG_PRINT(millis() / 1000);
  UDPDEBUG_START();
  UDPDEBUG_PRINT("Seconds ", (int)(millis() / 1000));
  DEBUG_PRINT(" Discovercount: ");
  DEBUG_PRINT(discoverCount);
  switch (loopStatus) {
    case POWER_OFF:
      DEBUG_PRINT(" Status: ");
      DEBUG_PRINTLN("OFF");
      UDPDEBUG_PRINTTXT(" Status: ");
      UDPDEBUG_PRINTTXT("OFF");
      break;

    case POWER_ON:
      DEBUG_PRINT(" Status: ");
      DEBUG_PRINTLN("ON");
      UDPDEBUG_PRINTTXT(" Status: ");
      UDPDEBUG_PRINTTXT("ON");
      break;

    case Renew:
      DEBUG_PRINT(" Status: ");
      DEBUG_PRINTLN("Renew");
      UDPDEBUG_PRINTTXT(" Status: ");
      UDPDEBUG_PRINTTXT("Renew");
      break;

    default:
      break;
  }
  UDPDEBUG_SEND();
}


bool switchAllSonoffs(bool command) {
  bool ret1, ret2;

  if (switchName1 != "") ret1 = switchSonoff(command, switchName1);
  if (!ret1) {
    DEBUG_PRINT(switchName1);
    DEBUG_PRINTLN(" not present");
    UDPDEBUG_START();
    UDPDEBUG_PRINTTXT(switchName1);
    UDPDEBUG_PRINTTXT(" not present");
    UDPDEBUG_SEND();
  }


  if (switchName2 != "") ret2 = switchSonoff(command, switchName2);
  if (!ret2) {
    DEBUG_PRINT(switchName2);
    DEBUG_PRINTLN(" not present");
    UDPDEBUG_START();
    UDPDEBUG_PRINTTXT(switchName2);
    UDPDEBUG_PRINTTXT(" not present");
    UDPDEBUG_SEND();
  }
  return ret1 & ret2;

}

bool switchSonoff(bool command, String device) {
  int i = 0;
  bool found = false;
  String payload;
  String switchString;

  if (WiFi.status() != WL_CONNECTED) espRestart('N', "Not connected");

  while ( deviceName[i].length() > 0 && i <= MAXDEVICES) {
    yield();
    payload = "";
    switchString = "";
    if (deviceName[i] == device) {
      switchString = "http://" + sonoffIP[i].toString();

      if (command == true) switchString = switchString + "/SWITCH=ON";
      else switchString = switchString + "/SWITCH=OFF";
      DEBUG_PRINTLN(switchString);
      UDPDEBUG_START();
      UDPDEBUG_PRINTTXT(debugBuffer);
      UDPDEBUG_SEND();

      // DEBUG_PRINT("Starting [HTTP] GET...\n");
      // start connection and send HTTP header
      http.begin(switchString);
      int httpCode = http.GET();

      DEBUG_PRINT("[HTTP] GET... code: ");
      DEBUG_PRINTLN(httpCode);

      UDPDEBUG_START();
      UDPDEBUG_PRINT("[HTTP] GET... code: ", httpCode);
      UDPDEBUG_SEND();
      //httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        if (httpCode == HTTP_CODE_OK) {
          payload = http.getString();
          found = true;
        } else {
          DEBUG_PRINT("[HTTP] GET... failed, error: ");
          DEBUG_PRINTLN(http.errorToString(httpCode));
          UDPDEBUG_START();
          UDPDEBUG_PRINTTXT("[HTTP] GET... code: ");
          UDPDEBUG_PRINTTXT(http.errorToString(httpCode));
          UDPDEBUG_SEND();
          LEDswitch(RedBlink);
        }
      } else {
        DEBUG_PRINTLN("HTTP code not ok");
        LEDswitch(RedBlink);
      }
      http.end();
    }
    i++;
  }
  return found;
}

void discovermDNSServices() {
  int j;
  for (j = 0; j < 5; j++) sonoffIP[j] = (0, 0, 0, 0);
  j = 0;
  DEBUG_PRINTLN("Sending mDNS query");
  yield();
  int n = MDNS.queryService(SERVICENAME, "tcp"); // Send out query for esp tcp services
  yield();
  DEBUG_PRINTLN("mDNS query done");
  if (n == 0) {
    espRestart('N', "No services found");
  }
  else {
    DEBUG_PRINT(n);
    DEBUG_PRINTLN(" service(s) found");
    for (int i = 0; i < n; ++i) {
      yield();
      // Print details for each service found
      DEBUG_PRINT(i + 1);
      DEBUG_PRINT(": ");
      DEBUG_PRINT(MDNS.hostname(i));
      DEBUG_PRINT(" (");
      DEBUG_PRINT(MDNS.IP(i));
      deviceName[j] = MDNS.hostname(i);
      sonoffIP[j++] = MDNS.IP(i);
      DEBUG_PRINT(":");
      DEBUG_PRINT(MDNS.port(i));
      DEBUG_PRINTLN(")");
      yield();
    }
  }
  DEBUG_PRINTLN();
}


void readFullConfiguration() {
  readConfig();  // configuration in EEPROM
  // insert NEW CONSTANTS according switchName1 example
  switchName1 = String(config.switchName1);
  switchName2 = String(config.switchName2);
}

bool readRTCmem() {
  bool ret = true;
  system_rtc_mem_read(RTCMEMBEGIN, &rtcMem, sizeof(rtcMem));
  if (rtcMem.markerFlag != MAGICBYTE) {
    rtcMem.markerFlag = MAGICBYTE;
    rtcMem.bootTimes = 0;
    system_rtc_mem_write(RTCMEMBEGIN, &rtcMem, sizeof(rtcMem));
    ret = false;
  }
  return ret;
}

void printRTCmem() {
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("rtcMem ");
  DEBUG_PRINT("markerFlag ");
  DEBUG_PRINTLN(rtcMem.markerFlag);
  DEBUG_PRINT("bootTimes ");
  DEBUG_PRINTLN(rtcMem.bootTimes);
}
