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


// --- Sketch Specific -----
#define SERVICENAME "SONOFF"  // name of the MDNS service used in this group of ESPs
#define MAXDEVICES 5

//-------- SERVICES --------------


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
  char automaticUpdate[2];   // right after boot

  // insert NEW CONSTANTS according boardname example HERE!
  char switchName1[STRUCT_CHAR_ARRAY_SIZE];
  char switchName2[STRUCT_CHAR_ARRAY_SIZE];

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
  "0",
  "",
  "",
  "CFG"  // Magic Bytes
};

// --- Sketch Specific -----
enum loopStatusDef {
  POWER_OFF,
  POWER_ON,
  RENEW
};

//---------- VARIABLES ----------

unsigned long debugEntry;
long counter = 0;
char boardMode = 'N';  // Normal operation or Configuration mode?

#ifdef REMOTEDEBUGGING
// UDP variables
char debugBuffer[255];
IPAddress broadcastIp(255, 255, 255, 255);
#endif


// --- Sketch Specific -----
// String xx; // add NEW CONSTANTS for WiFiManager according the variable "boardname"
String switchName1, switchName2;
loopStatusDef  loopStatus, lastLoopStatus;

bool PIRstatus;
IPAddress sonoffIP[10];
String deviceName[30];
unsigned long dnsRefreshEntry = 0;
unsigned long renewEntry;
int discoverCount = 0;  // # times DNS discover found no results
String sysMessage;



//---------- FUNCTIONS ----------
// to help the compiler, sometimes, functions have  to be declared here
void loopWiFiManager(void);
void readFullConfiguration(void);
bool readRTCmem(void);
void printRTCmem(void);
void initialize(void);
void showLoopState(loopStatusDef);
void discovermDNSServices(void);
bool switchAllSonoffs(bool);
bool switchSonoff(bool, String);


//---------- OTHER .H FILES ----------
#include <ESP_Helpers.h>           // General helpers for all IOTappStory sketches
#include "IOTappStoryHelpers.h"    // Sketch specific helpers for all IOTappStory sketches




// ================================== SETUP ================================================
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Start "FIRMWARE);


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

  readFullConfiguration();

  // --------- START WIFI --------------------------

  connectNetwork();

  sendSysLogMessage(2, 1, config.boardName, FIRMWARE, 10, counter++, "------------- Normal Mode -------------------");


  IOTappStory();
  // ----------- SPECIFIC SETUP CODE ----------------------------
  discoverMDNS();
  switchAllSonoffs(false);
  loopStatus = POWER_OFF;
  // ----------- END SPECIFIC SETUP CODE ----------------------------

  LEDswitch(None);
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection


  sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Setup done");
}


//========================== LOOP =============================
void loop() {
  //-------- IOTappStory Block ---------------
  yield();
  handleModeButton();   // this routine handles the reaction of the Flash button. If short press: update of skethc, long press: Configuration

  // Normal blind (1 sec): Connecting to network
  // fast blink: Configuration mode. Please connect to ESP network
  // Slow Blink: IOTappStore Update in progress

  // ------- Debug Message --------
  if ((millis() - debugEntry) > 5000) {
    debugEntry = millis();
    sendDebugMessage();
  }

  //-------- Your Sketch ---------------

  discoverMDNS();

  PIRhandler();

}
// ------------------------- END LOOP ----------------------------------

void sendDebugMessage() {
  // ------- Syslog Message --------

  /* severity: 2 critical, 6 info, 7 debug
     facility: 1 user level
      String hostName: Board Name
      app: FIRMWARE
      procID: unddefined
      msgID: counter
      message: Your message
  */

  sysMessage = "";

  long h1 = ESP.getFreeHeap();
  sysMessage += " Heap ";
  sysMessage += h1;
  sysMessage += " LoopStatus: ";
  sysMessage += loopStatus;
  if (PIRstatus) sysMessage += " PIRstatus: ON ";
  else sysMessage += " PIRstatus: OFF ";

  // sendSysLogMessage(6, 1, config.boardName, FIRMWARE, 10, counter++, sysMessage);
}



void PIRhandler() {
  PIRstatus = digitalRead(PIRpin);

  switch (loopStatus) {
    case POWER_ON:        // Lamp is on
      if (lastLoopStatus != POWER_ON) {
        switchAllSonoffs(true);
        LEDswitch(Green);
        lastLoopStatus = POWER_ON;
        renewEntry = millis();
      }
      // exit criteria
      if (PIRstatus == LOW) {
        loopStatus = POWER_OFF;
      }
      if ((millis() - renewEntry) > 5000)  loopStatus = RENEW; // every 10 sec) loopStatus = POWER_OFF;
      break;


    case RENEW:       // Lamp is on, send additional message
      switchAllSonoffs(true);

      // exit
      lastLoopStatus = RENEW;
      loopStatus = POWER_ON;
      renewEntry = millis();
      break;

    case POWER_OFF:
      if (lastLoopStatus != POWER_OFF) {
        LEDswitch(None);
        lastLoopStatus = POWER_OFF;
      }

      // exit criteria
      if (PIRstatus == HIGH) {
        loopStatus = POWER_ON;
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


int getDeviceNbr (String devName) {
  int i = 0, deviceNumber = 99;
  while ((deviceName[i] != devName) && (i <= MAXDEVICES)) i++;
  if (i < MAXDEVICES) deviceNumber = i;
  return deviceNumber;
}

bool switchAllSonoffs(bool command) {
  bool ret1 = true, ret2 = true;
  int deviceNr;

  if (switchName1 != "") ret1 = switchSonoff(command, getDeviceNbr(switchName1));
  if (switchName2 != "") ret2 = switchSonoff(command, getDeviceNbr(switchName2));

  return ret1 & ret2;
}


bool switchSonoff(bool command, int device) {
  bool found = false;
  String payload;
  String switchString;

  if (WiFi.status() != WL_CONNECTED) espRestart('N', "Not connected");
  payload = "";
  switchString = "";
  switchString = "http://" + sonoffIP[device].toString();

  if (command == true) switchString = switchString + "/SWITCH=ON";
  else switchString = switchString + "/SWITCH=OFF";

  sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, deviceName[device] + " " + switchString);

  // DEBUG_PRINT("Starting [HTTP] GET...\n");
  // start connection and send HTTP header
  http.begin(switchString);
  int httpCode = http.GET();
  sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "[HTTP] GET... code: " + String(httpCode));
  //httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
      found = true;
    } else {
      LEDswitch(RedBlink);
      sendSysLogMessage(2, 1, config.boardName, FIRMWARE, 10, counter++, "[Wrong response: " + http.errorToString(httpCode));
    }
  } else {
    sendSysLogMessage(2, 1, config.boardName, FIRMWARE, 10, counter++, "[No response: " + http.errorToString(httpCode));
    LEDswitch(RedBlink);
  }
  http.end();
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
    String message1 = String(n) + " service(s) found ";
    for (int i = 0; i < n; ++i) {
      yield();
      deviceName[j] = MDNS.hostname(i);
      sonoffIP[j++] = MDNS.IP(i);
      sysMessage = message1 + " Nr: " + String(i) + ": " + MDNS.hostname(i) + " (" + MDNS.IP(i).toString() + +": " + MDNS.port(i) + ")";
      sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, sysMessage);
    }
  }
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
