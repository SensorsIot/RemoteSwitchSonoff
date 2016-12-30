extern "C" {
#include "user_interface.h" // this is for the RTC memory read/write functions
}


// -------- PIN DEFINITIONS ------------------


//---------- CODE DEFINITIONS ----------
#define MAXDEVICES 5
#define STRUCT_CHAR_ARRAY_SIZE 50  // length of config variables
#define SERVICENAME "SONOFF"  // name of the MDNS service used in this group of ESPs



//-------- SERVICES --------------



//--------- ENUMS AND STRUCTURES  -------------------


//---------- VARIABLES ----------



//---------- FUNCTIONS ----------


//---------- OTHER .H FILES ----------



//--------------- START ----------------------------------
void configESP() {
  Serial.begin(115200);
#ifdef LEDgreen
  digitalWrite(LEDgreen, !digitalRead(GPIO0));
#endif
  for (int i = 0; i < 5; i++) DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Start " FIRMWARE);
  DEBUG_PRINTLN("------------- Configuration Mode -------------------");

  for (int i = 0; i < 3; i++) DEBUG_PRINTLN("");

  pinMode(GPIO0, INPUT_PULLUP);  // GPIO0 as input for Config mode selection

#ifdef LEDgreen
  pinMode(LEDgreen, OUTPUT);
  LEDswitch(GreenFastBlink);
#endif
#ifdef LEDred
  pinMode(LEDred, OUTPUT);
#endif


  readFullConfiguration();  // configuration in EEPROM
  connectNetwork();

  UDPDEBUG_START();
  UDPDEBUG_PRINTTXT("------------- Configuration Mode -------------------");
  UDPDEBUG_SEND();

  initWiFiManager();


  //--------------- LOOP ----------------------------------

  while (1) {
    if (buttonChanged && buttonTime > 4000) espRestart('N', "Back to normal mode");  // long button press > 4sec
    yield();
    loopWiFiManager();
  }
}


void loopWiFiManager() {

  // additional fields
  WiFiManagerParameter p_delayTime("p_delayTime", "p_delayTime", config.delayTime, 5);

  // Standard
  WiFiManagerParameter p_boardName("boardName", "boardName", config.boardName, STRUCT_CHAR_ARRAY_SIZE);
  WiFiManagerParameter p_IOTappStory1("IOTappStory1", "IOTappStory1", config.IOTappStory1, STRUCT_CHAR_ARRAY_SIZE);
  WiFiManagerParameter p_IOTappStoryPHP1("IOTappStoryPHP1", "IOTappStoryPHP1", config.IOTappStoryPHP1, STRUCT_CHAR_ARRAY_SIZE);
  WiFiManagerParameter p_IOTappStory2("IOTappStory2", "IOTappStory2", config.IOTappStory2, STRUCT_CHAR_ARRAY_SIZE);
  WiFiManagerParameter p_IOTappStoryPHP2("IOTappStoryPHP2", "IOTappStoryPHP2", config.IOTappStoryPHP2, STRUCT_CHAR_ARRAY_SIZE);

  // Just a quick hint
  WiFiManagerParameter p_hint("<small>*Hint: if you want to reuse the currently active WiFi credentials, leave SSID and Password fields empty</small>");

  // Initialize WiFIManager
  WiFiManager wifiManager;
  wifiManager.addParameter(&p_hint);

  //add all parameters here
  wifiManager.addParameter(&p_delayTime);

  // Standard
  wifiManager.addParameter(&p_boardName);
  wifiManager.addParameter(&p_IOTappStory1);
  wifiManager.addParameter(&p_IOTappStoryPHP1);
  wifiManager.addParameter(&p_IOTappStory2);
  wifiManager.addParameter(&p_IOTappStoryPHP2);


  // Sets timeout in seconds until configuration portal gets turned off.
  // If not specified device will remain in configuration mode until
  // switched off via webserver or device is restarted.
  // wifiManager.setConfigPortalTimeout(600);

  // It starts an access point
  // and goes into a blocking loop awaiting configuration.
  // Once the user leaves the portal with the exit button
  // processing will continue
  if (!wifiManager.startConfigPortal(config.boardName)) {
    DEBUG_PRINTLN("Not connected to WiFi but continuing anyway.");
  } else {
    // If you get here you have connected to the WiFi
    DEBUG_PRINTLN("Connected... :-)");
  }
  // Getting posted form values and overriding local variables parameters
  // Config file is written

  //add all parameters here


  // Standard
  strcpy(config.boardName, p_boardName.getValue());
  strcpy(config.IOTappStory1, p_IOTappStory1.getValue());
  strcpy(config.IOTappStoryPHP1, p_IOTappStoryPHP1.getValue());
  strcpy(config.IOTappStory2, p_IOTappStory2.getValue());
  strcpy(config.IOTappStoryPHP2, p_IOTappStoryPHP2.getValue());
  //additional fields
  strcpy(config.delayTime, p_delayTime.getValue());

  writeConfig();
  readConfig();  // read back to fill all variables

  LEDswitch(None); // Turn LED off as we are not in configuration mode.

  espRestart('N', "Configuration finished"); //Normal Operation

}
