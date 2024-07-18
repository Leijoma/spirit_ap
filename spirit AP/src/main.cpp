#include <Arduino.h>
#include <ConfigAssist.h>        // Config assist class
#include <ConfigAssistHelper.h>  // Config assist helper class

#include "drive.h"
#include "nmea_receiver.h"
#include "autopilot.h"
#include "autopilotdata.h"
#include "logger.h"
#include "seatalk.h"

#define ESP32_CAN_TX_PIN GPIO_NUM_32 
#define ESP32_CAN_RX_PIN GPIO_NUM_34 

#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemTime},
  {127250L,&Heading},
  {127257L,&Attitude},
  {0,0}
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

#if defined(ESP32)
  WebServer server(80);
#else
  ESP8266WebServer server(80);
#endif

#define INI_FILE "/TestWifi.ini" // Define SPIFFS storage file

#ifndef LED_BUILTIN
  #define LED_BUILTIN 22
#endif

SeaTalk *seaTalk = nullptr;

RudderDriver rudder_driver; 
AutopilotData autopilotdata;
Autopilot autopilot(autopilotdata, &rudder_driver );

// Define intervals in milliseconds
const unsigned long interval50Hz = 20;  // 1000ms / 50Hz = 20ms
const unsigned long interval10Hz = 100; // 1000ms / 10Hz = 100ms

unsigned long previousMillis50Hz = 0;
unsigned long previousMillis10Hz = 0;
unsigned long previousMillis1Hz = 0;
unsigned long previousMillis01Hz = 0;

const char* VARIABLES_DEF_YAML PROGMEM = R"~(
Wifi settings:
  - st_ssid1:
      label: Name for WLAN
      default:
  - st_pass1:
      label: Password for WLAN
      default:
  - st_ip1:
      label: Static ip setup (ip mask gateway) (192.168.1.100 255.255.255.0 192.168.1.1)
      default:
  - st_ssid2:
      label: Name for WLAN
      default:
  - st_pass2:
      label: Password for WLAN
      default:
  - st_ip2:
      label: Static ip setup (ip mask gateway) (192.168.1.100 255.255.255.0 192.168.1.1)
      default:

  - host_name:
      label: >-
        Host name to use for MDNS and AP<br>{mac} will be replaced with device's mac id
      default: configAssist_{mac}
  - sim_ip:
      label: Simulator IP address
      default: 127.0.0.1
  - sim_port:
      label: Simulator UDP port
      default: 10111
Rudder PID settings:
  - rudder_p:
      label: Rudder PID Proportional .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 1
  - rudder_i:
      label: Rudder PID Intregral .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - rudder_d:
      label: Rudder PID Differential .
        Leave blank for auto.                                                                                                                                                                                                                     
      attribs: "min='2' max='23' step='1'"
      default: 0
  - rudder_i_limit:
      label: Rudder PID Integral limit .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 10
Heading PID settings:
  - heading_p:
      label: Heading PID Proportional .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 1
  - heading_i:
      label: Heading PID Intregral .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - heading_d:
      label: Heading PID Differential .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - heading_i_limit:
      label: Rudder PID Integral limit .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 2
Turnrate PID settings:
  - turn_p:
      label: Turnrate PID Proportional .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 1
  - turn_i:
      label: Turnrate PID Intregral .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - turn_d:
      label: Turnrate PID Differential .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - turn_i_limit:
      label: Rudder PID Integral limit .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 2
XTE PID settings:
  - xte_p:
      label: XTE PID Proportional .
        Leave blank for auto.
      attribs: "min='0' max='23' step='0.01'"
      default: 1
  - xte_i:
      label: XTE PID Intregral .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - xte_d:
      label: XTE PID Differential .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 0
  - xte_i_limit:
      label: XTE PID Integral limit .
        Leave blank for auto.
      attribs: "min='2' max='23' step='1'"
      default: 2
Targets:
  - target_heading:
      label: Target heading
      default: 0
  - target_turnrate:
      label: Targetturnrate
      default: 2
)~";

ConfigAssist conf(INI_FILE, VARIABLES_DEF_YAML);

void onDataChanged(String key){
  Serial.println(key);
  Logger::logf("config data changed");
  float foo=0;
  float foo2=0;
  float foo3=0;
  float foo4=0;
  
  if (key=="target_heading") {
    foo= atof(conf["target_heading"].c_str());
    //Serial.println(foo);
    autopilotdata.setTargetHeading(foo);

    rudder_driver.setTargetAngle(foo);
 
  }
   if (key=="target_turnrate") {
    foo= atof(conf["target_turnrate"].c_str());
    //Serial.println(foo);
    autopilotdata.setTargetTurnRate(foo);
  }
  if (key=="heading_p") {
    foo= atof(conf["heading_p"].c_str());
    //Serial.println(foo);
    autopilot.setHeadingPID_P(foo);
  }
  if (key=="heading_i") {
    foo= atof(conf["heading_i"].c_str());
    //Serial.println(foo);
    autopilot.setHeadingPID_I(foo);
  }
  if (key=="heading_d") {
    foo= atof(conf["heading_d"].c_str());
    //Serial.println(foo);
    autopilot.setHeadingPID_D(foo);
  }
  if (key=="heading_i_limit") {
    foo= atof(conf["heading_i_limit"].c_str());
    //Serial.println(foo);
    autopilot.setHeadingPID_maxI(foo);
  }
  if (key=="turn_p") {
    foo= atof(conf["turn_p"].c_str());
    //Serial.println(foo);
    autopilot.setTurnPID_P(foo);
  }
  if (key=="turn_i") {
    foo= atof(conf["turn_i"].c_str());
    //Serial.println(foo);
    autopilot.setTurnPID_I(foo);
  }
  if (key=="turn_d") {
    foo= atof(conf["turn_d"].c_str());
    //Serial.println(foo);
    autopilot.setTurnPID_D(foo);
  }
  if (key=="turn_i_limit") {
    foo= atof(conf["turn_i_limit"].c_str());
    //Serial.println(foo);
    autopilot.setTurnPID_maxI(foo);
  }
   if (key=="xte_p") {
    foo= atof(conf["xte_p"].c_str());
    //Serial.println(foo);
    autopilot.setXTEPID_P(foo);
  }
  if (key=="xte_i") {
    foo= atof(conf["xte_i"].c_str());
    //Serial.println(foo);
    autopilot.setXTEPID_I(foo);
  }
  if (key=="xte_d") {
    foo= atof(conf["xte_d"].c_str());
    //Serial.println(foo);
    autopilot.setXTEPID_D(foo);
  }
  if (key=="xte_i_limit") {
    foo= atof(conf["xtei_limit"].c_str());
    //Serial.println(foo);
    autopilot.setXTEPID_maxI(foo);
  }
  if (key=="rudder_p") {
    foo= atof(conf["rudder_p"].c_str());
    foo2= atof(conf["rudder_i"].c_str());
    foo3= atof(conf["rudder_d"].c_str());
    foo4= atof(conf["rudder_i_limit"].c_str());
    rudder_driver.setPIDparameters(foo,foo2,foo3,foo4 );
  }
  if (key=="rudder_i") {
    foo= atof(conf["rudder_p"].c_str());
    foo2= atof(conf["rudder_i"].c_str());
    foo3= atof(conf["rudder_d"].c_str());
    foo4= atof(conf["rudder_i_limit"].c_str());
    rudder_driver.setPIDparameters(foo,foo2,foo3,foo4 );
 }
  if (key=="rudder_d") {
    foo= atof(conf["rudder_p"].c_str());
    foo2= atof(conf["rudder_i"].c_str());
    foo3= atof(conf["rudder_d"].c_str());
    foo4= atof(conf["rudder_i_limit"].c_str());
    rudder_driver.setPIDparameters(foo,foo2,foo3,foo4 );
  }
  if (key=="rudder_i_limit") {
    foo= atof(conf["rudder_p"].c_str());
    foo2= atof(conf["rudder_i"].c_str());
    foo3= atof(conf["rudder_d"].c_str());
    foo4= atof(conf["rudder_i_limit"].c_str());
    rudder_driver.setPIDparameters(foo,foo2,foo3,foo4 );
  }
}

String hostName;
unsigned long pingMillis = millis();

// Print memory info
void debugMemory(const char* caller) {
  #if defined(ESP32)
    LOG_D("%s > Free: heap %u, block: %u, pSRAM %u\n", caller, ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), ESP.getFreePsram());
  #else
    LOG_D("%s > Free: heap %u\n", caller, ESP.getFreeHeap());
  #endif
}

// Web server root handler
void handleRoot() {
  String out("<h2>Hello from {name}</h2>");
  out += "<h4>Device time: " + conf.getLocalTime() +"</h4>";
  out += "<a href='/cfg'>Edit config</a>";

  #if defined(ESP32)
    out.replace("{name}", "ESP32");
  #else
    out.replace("{name}", "ESP8266!");
  #endif
  // Send browser time sync script
  #ifdef CA_USE_TESTWIFI
    out += "<script>" + conf.getTimeSyncScript() + "</script>";
  #endif
  server.send(200, "text/html", out);
}

// Page not found handler
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

// Function prototypes
void task50Hz();
void task10Hz();

// *********************   Begin Setup function  *********************
// *******************************************************************

void setup(void) {

// Initialize Logger
  Logger::begin(115200);
  Logger::log("Setup started");

  Serial.begin(115200);
  Serial.print("\n\n\n\n");
  Serial.flush();

  LOG_I("Starting..\n");
  debugMemory("setup");

   //conf.deleteConfig();  //Uncomment to remove ini file and re-built it fron yaml

  // Setup web server
  server.on("/", handleRoot);
  server.on("/d", []() {    // Append dump handler
    conf.dump(&server);
  });

  server.onNotFound(handleNotFound);

  // Define a ConfigAssist helper class to connect wifi
  ConfigAssistHelper confHelper(conf);

  // Connect to any available network
  bool bConn = confHelper.connectToNetwork(15000, LED_BUILTIN);

  // Append config assist handlers to web server, setup ap on no connection
  conf.setup(server, !bConn);
  conf.setRemotUpdateCallback(onDataChanged);

  if(!bConn) LOG_E("Connect failed.\n");

  if (MDNS.begin(conf[CA_HOSTNAME_KEY].c_str())) {
    LOG_I("MDNS responder started\n");
  }

  server.begin();
  LOG_I("HTTP server started\n");

  rudder_driver.init(conf["rudder_p"].toDouble(),conf["rudder_i"].toDouble(),conf["rudder_d"].toDouble(),26,36,25);
  rudder_driver.setPIDparameters(conf["rudder_p"].toDouble(),conf["rudder_i"].toDouble(),conf["rudder_d"].toDouble(),conf["rudder_i_limit"].toDouble());

  autopilotdata.setMode(Standby);
  autopilotdata.setTargetTurnRate(conf["target_turnrate"].toDouble()); 

  autopilot.setHeadingPID_P(conf["heading_p"].toDouble());
  autopilot.setHeadingPID_I(conf["heading_i"].toDouble());
  autopilot.setHeadingPID_D(conf["heading_d"].toDouble());
  autopilot.setHeadingPID_maxI(conf["heading_i_limit"].toDouble());

  
  autopilot.setTurnPID_P(conf["turn_p"].toDouble());
  autopilot.setTurnPID_I(conf["turn_i"].toDouble());
  autopilot.setTurnPID_D(conf["turn_d"].toDouble());
  autopilot.setTurnPID_maxI(conf["turn_i_limit"].toDouble());


  autopilot.setXTEPID_P(conf["xte_p"].toDouble());
  autopilot.setXTEPID_I(conf["xte_i"].toDouble());
  autopilot.setXTEPID_D(conf["xte_d"].toDouble());
  autopilot.setXTEPID_maxI(conf["xte_i_limit"].toDouble());

  autopilot.setUpdateRate(100);  

  seaTalk = new SeaTalk(autopilotdata);

  OutputStream=&Serial;

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->print("Running...");
}

//*******************************  END Setup  **********************************************


//*****************************************************************************
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val)=0, bool AddLf=false, int8_t Desim=-1 ) {
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if ( Desim<0 ) {
      if (ConvFunc) { OutputStream->print(ConvFunc(val)); } else { OutputStream->print(val); }
    } else {
      if (ConvFunc) { OutputStream->print(ConvFunc(val),Desim); } else { OutputStream->print(val,Desim); }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
}

//*****************************************************************************
void Attitude(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double Yaw;
    double Pitch;
    double Roll;
    
    if (ParseN2kAttitude(N2kMsg,SID,Yaw,Pitch,Roll) ) {
     // OutputStream->println("Attitude:");
     // PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      //PrintLabelValWithConversionCheckUnDef("  Yaw (deg): ",Yaw,&RadToDeg,true);
     // PrintLabelValWithConversionCheckUnDef("  Pitch (deg): ",Pitch,&RadToDeg,true);
     // PrintLabelValWithConversionCheckUnDef("  Roll (deg): ",Roll,&RadToDeg,true);
    } else {
      //OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Heading(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double Heading;
    double Deviation;
    double Variation;
    
    if (ParseN2kHeading(N2kMsg,SID,Heading,Deviation,Variation,HeadingReference) ) {
      //OutputStream->println("Heading:");
      //OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference,OutputStream);
      //Serial.println("Heading");
      //PrintLabelValWithConversionCheckUnDef("  Heading (deg): ",Heading,&RadToDeg,true);
     // PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ",Deviation,&RadToDeg,true);
     // PrintLabelValWithConversionCheckUnDef("  Variation (deg): ",Variation,&RadToDeg,true);
     autopilotdata.setActualHeading(Heading*(180.0/3.141592653589793238463));
    } else {
      //OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}



//*****************************************************************************
void SystemTime(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t SystemDate;
    double SystemTime;
    tN2kTimeSource TimeSource;

  
    
    if (ParseN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
      OutputStream->println("System time:");
      // PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      // PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      // PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
      OutputStream->print("  time source: "); PrintN2kEnumType(TimeSource,OutputStream);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}



//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
   // Serial.println("message received");
  // Find handler
  //OutputStream->print("In Main Handler: "); 
  //OutputStream->println(N2kMsg.MsgTime);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}


// *********************   Begin Main Loop  *************************
// *******************************************************************

void loop(void) {
  server.handleClient();
  autopilot.update();
  rudder_driver.update();
  
  NMEA2000.ParseMessages();

  seaTalk->processMessages();

  unsigned long currentMillis = millis();
  // Task for 50Hz (every 20ms)
  if (currentMillis - previousMillis50Hz >= interval50Hz) {
      previousMillis50Hz = currentMillis;
      
  }

  // Task for 10Hz (every 100ms)
  if (currentMillis - previousMillis10Hz >= 100) {
      previousMillis10Hz = currentMillis;
      
  }

  // Task for 1Hz (every 1000ms)
  if (currentMillis - previousMillis1Hz >= 1000) { 
    previousMillis1Hz = currentMillis;

    // TODO Move all Seatalk AP commands to a Raymarine emulator class.
    seaTalk->sendAP84(); 
    seaTalk->sendAP9C();  
    if (autopilotdata.getMode()==Auto_waiting_for_confirmation)
       seaTalk->sendAP83(2); 

    // Debug messages
    Serial.print("heading: ");
    Serial.println(autopilotdata.getActualHeading());  
    Serial.print("rudder: ");
    Serial.println(rudder_driver.getCurrentAngle());  
  }
  
  // Task for 0.1Hz (every 10000ms)
  if (currentMillis - previousMillis01Hz >= 10000) {
    previousMillis01Hz = currentMillis;
    // TODO Move all Seatalk AP commands to a Raymarine emulator class.
    seaTalk->sendAP90();
  }
}
