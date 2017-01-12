/* This sketch connects to the iopappstore and loads the assigned firmware down. The assignment is done on the server based on the MAC address of the board

    On the server, you need PHP script "IOTappStory.php" and the bin files are in the .\bin folder

    This work is based on the ESPhttpUpdate examples

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

#define SKETCH "SonoffReceiver "
#define VERSION "V1.1"
#define FIRMWARE SKETCH VERSION

#define SERIALDEBUG         // Serial is used to present debugging messages 
#define REMOTEDEBUGGING     // telnet is used to present

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

#include <ESP8266WebServer.h>
//#include <ESP8266HTTPClient.h>



// -------- PIN DEFINITIONS ------------------
#ifdef ARDUINO_ESP8266_ESP01           // Generic ESP's 
#define MODEBUTTON 0
#define LEDgreen 13
//#define LEDred 12
#else
#define MODEBUTTON D3
#define LEDgreen D7
//#define LEDred D6
#endif

// --- Sketch Specific -----
#ifdef ARDUINO_ESP8266_ESP01           // Generic ESP's 
#define RELAYPIN 12
#else
#define RELAYPIN D6
#endif


//---------- DEFINES for SKETCH ----------
#define STRUCT_CHAR_ARRAY_SIZE 50  // length of config variables


// --- Sketch Specific -----
#define SERVICENAME "SONOFF"  // name of the MDNS service used in this group of ESPs


//-------- SERVICES --------------



// --- Sketch Specific -----
ESP8266WebServer server(80);

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
  char delayTime[10];

  char magicBytes[4];
} strConfig;

strConfig config = {
  "",
  "",
  "SONOFF_RECEIVER_INIT",
  "192.168.0.200",
  "/IOTappStory/IOTappStoryv20.php",
  "iotappstory.org",
  "/ota/esp8266-v1.php",
  "0",
  "420",
  "CFG"  // Magic Bytes
};

// --- Sketch Specific -----
enum relayStatusDef {
  RELAY_OFF,
  RELAY_ON
} relayStatus;

enum wifiCommandDef {
  NONE,
  COMMAND_OFF,
  COMMAND_ON,
  COMMAND_STATUS
} wifiCommand = COMMAND_OFF;

//---------- VARIABLES ----------

String switchName1, switchName2;
unsigned long debugEntry;
long counter = 0;
char boardMode = 'N';  // Normal operation or Configuration mode?
String sysMessage;

#ifdef REMOTEDEBUGGING
// UDP variables
char debugBuffer[255];
IPAddress broadcastIp(255, 255, 255, 255);
#endif

// --- Sketch Specific -----
// String xx; // add NEW CONSTANTS for WiFiManager according the variable "boardname"

int timeToOff = 0;   // time for Relay to switch off without ON command
unsigned long timeToOffEntry, responseEntry;



//---------- FUNCTIONS ----------
// to help the compiler, sometimes, functions have  to be declared here
void loopWiFiManager(void);
void readFullConfiguration(void);
bool readRTCmem(void);
void printRTCmem(void);
void initialize(void);
void switchRelay(bool);


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
  pinMode(RELAYPIN, OUTPUT);


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

  if (atoi(config.automaticUpdate) == 1) IOTappStory();




  // ----------- SPECIFIC SETUP CODE ----------------------------

  // add a DNS service
  MDNS.addService(SERVICENAME, "tcp", 8080);

   webServerStart();


  // ----------- END SPECIFIC SETUP CODE ----------------------------

  LEDswitch(None);
  pinMode(MODEBUTTON, INPUT_PULLUP);  // MODEBUTTON as input for Config mode selection

  sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Setup done");
}



//======================= LOOP =========================
void loop() {

  //-------- IOTappStory Block ---------------
  yield();
  handleModeButton();   // this routine handles the reaction of the Flash button. If short press: update of skethc, long press: Configuration

  // Normal blink (1 sec): Connecting to network
  // fast blink: Configuration mode. Please connect to ESP network
  // Slow Blink: IOTappStore Update in progress

  if (millis() - debugEntry > 5000) { // Non-Blocking second counter
    debugEntry = millis();
    sendDebugMessage();
  }

  //-------- Your Sketch ---------------

  if (millis() - timeToOffEntry > 1000) { // Non-Blocking second counter
    timeToOffEntry = millis();
    if (timeToOff-- <= 0 ) timeToOff = 0;
  }
  server.handleClient();   // Wait for a client to connect and when they do process their requests
  handleStatus();  // define next status

}
//------------------------- END LOOP --------------------------------------------


void handleStatus() {

  if (wifiCommand == COMMAND_ON && atoi(config.delayTime) != 0 ) timeToOff = atoi(config.delayTime);

  switch (relayStatus) {

    case RELAY_OFF:
      if (relayStatus != RELAY_OFF) switchRelay(LOW);

      // exit
      if (wifiCommand == COMMAND_ON) switchRelay(HIGH);
      break;

    case RELAY_ON:
      if (relayStatus != RELAY_ON) switchRelay(HIGH);

      // exit
      if (wifiCommand == COMMAND_OFF || (atoi(config.delayTime) != 0 &&  timeToOff <= 0)) {
        timeToOff = 0;
        switchRelay(LOW);
      }
      break;
  }
  wifiCommand = NONE;
}

void webServerStart() {
  server.on ( "/", handleRoot );
  server.on ( "/SWITCH=ON", []() {
    wifiCommand = COMMAND_ON;
    handleStatus();  // define next status
    handleRoot();
  });
  server.on ( "/SWITCH=OFF", []() {
    wifiCommand = COMMAND_OFF;
    handleStatus();  // define next status
    handleRoot();
  });
  server.on ( "/STATUS", []() {
    wifiCommand = COMMAND_STATUS;
    handleStatus();  // define next status
    handleRoot();
    sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Status");
  });
  server.onNotFound ( handleNotFound );
  server.begin();
}


void handleRoot() {
  String message = "<html>\
  <head>\
    <title>ESP8266 Demo</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1> ";
  message += "BOARD: ";
  message += String(config.boardName);
  message += "<br />Delay Time: ";
  message += atoi(config.delayTime);
  message += "<br />STATUS: ";
  if (relayStatus == RELAY_ON) {
    message += "On ";
    message += timeToOff ;
    message += " sec";
  }
  else  message += "Off";

  message += "</h1><h2>";
  message += "Click <a href=\"/SWITCH=ON\">here</a> turn the SWITCH on pin 12 ON<br>";
  message += "Click <a href=\"/SWITCH=OFF\">here</a> turn the SWITCH on pin 12 OFF<br>";
  message += "Click <a href=\"/STATUS\">here</a> get status<br>";

  message += " </h2>\</body>\</html> <br />";
  server.send ( 200, "text / html", message );
}




void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text / plain", message );
}


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
  sysMessage += " relayStatus: ";
  sysMessage += relayStatus;
  sysMessage += " Delay: ";
  sysMessage += timeToOff;

  sendSysLogMessage(6, 1, config.boardName, FIRMWARE, 10, counter++, sysMessage);
}


void switchRelay(bool state) {
  if (state) {
    sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Switch On ");
    LEDswitch(Green);
    digitalWrite(RELAYPIN, HIGH);
    relayStatus = RELAY_ON;
  } else {
    sendSysLogMessage(7, 1, config.boardName, FIRMWARE, 10, counter++, "Switch Off ");
    LEDswitch(None);
    digitalWrite(RELAYPIN, LOW);
    relayStatus = RELAY_OFF;
  }
}


void readFullConfiguration() {
  readConfig();  // configuration in EEPROM
  // insert NEW CONSTANTS according switchName1 example
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



