/*
 * module is a WeMos D1 R1
 * flash size set to 4M (1M SPIFFS) or 4MB (FS:1MB OTA:~1019KB) (latest esp board sw uses this)
 */

/*
 * todo:
 * - fix codes screen to capture type, data, and # bits
 * 
 * notes:
 * - disable windows antivirus running when this is compiling...its killing the machine
 *     open windows security, open virus and threat protection, click manage settings, 
 *     turn off real-time protection
 * - test arduino ota code
 *     had to do the following to get the esp to show up in teh ports section
 *     installed bonjour service from apple.  run bonjour browser to see if the esp is there
 *     ran some regedit command to fix some mdns issue.  (and reboot)
 *     seems very finicky
 *     also, as the ota starts, the web socket gets closed, so there is no way to send status to the browser...pity
 *     why is it closed?
 * 3 types: (listed simpliest to implement to most complex)
 * 1. ota update via arduino ide (see ArduinoOTA) -> works, but hard to get the network esp to show up in arduino ide
 * 2. ota update via web browser (see ESP8266HTTPUpdateServer) -> works
 * 3. authmatic ota update via http server (see ESPhttpUpdate) -> not needed (for production with many deployed devices)
 * - add remote control button panel: allow multiple remote pages, set button size, position, and label, and command script it runs
 *     added simple remote page...not configurable
 * - command script: set trigger (manual (button), IR cmd in, time, or mqtt subscribe), set action (series of IR cmd out (with repeat) and delays and mqtt publish)
 *     not really needed. do the workflow logic in node-red
 */

/*
 * library sources:
 * ESP8266WiFi, ESP8266WebServer, FS, DNSServer, Hash, EEPROM, ArduinoOTA - https://github.com/esp8266/Arduino
 * WebSocketsServer - https://github.com/Links2004/arduinoWebSockets (git)
 * WiFiManager - https://github.com/tzapu/WiFiManager (git)
 * ESPAsyncTCP - https://github.com/me-no-dev/ESPAsyncTCP (git)
 * ESPAsyncUDP - https://github.com/me-no-dev/ESPAsyncUDP (git)
 * OneWire - https://github.com/PaulStoffregen/OneWire (git)
 * DallasTemperature - https://github.com/milesburton/Arduino-Temperature-Control-Library (git)
 * PubSub - https://github.com/knolleary/pubsubclient (git)
 * TimeLib - https://github.com/PaulStoffregen/Time (git)
 * Timezone - https://github.com/JChristensen/Timezone (git)
 * ArduinoJson - https://github.com/bblanchon/ArduinoJson  (git)
 * IRremoteESP8266, IRsend, IRrecv, IRutils - https://github.com/crankyoldgit/IRremoteESP8266 (git)
 */

#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <TimeLib.h> 
#include <Timezone.h>

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

// --------------------------------------------

// web server library includes
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// --------------------------------------------

// file system (spiffs) library includes
#include <FS.h>

// --------------------------------------------

// wifi manager library includes
#include <DNSServer.h>
#include <WiFiManager.h>

// --------------------------------------------

// aync library includes
#include <ESPAsyncTCP.h>
#include <ESPAsyncUDP.h>

// --------------------------------------------

// infra red receiver/sender library includes
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>


IRsend irsend(D7);
IRrecv irrecv(D5);
decode_results results;


// --------------------------------------------

// ds18b20 temperture sensor library includes
#include <OneWire.h> 
#include <DallasTemperature.h>

// --------------------------------------------

// mqtt library includes
#include <PubSubClient.h>

// --------------------------------------------

// arduino ota library includes
#include <ArduinoOTA.h>

#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

// --------------------------------------------


ESP8266WebServer server(80);
File fsUploadFile;
bool isUploading;
bool isSetup = false;

WebSocketsServer webSocket = WebSocketsServer(81);
int webClient = -1;
int setupClient = -1;
int remoteClient = -1;
int codesClient = -1;

// temperature
#define ONE_WIRE_BUS D6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress thermometer;
unsigned long lastTempRequest = 0;
int delayInMillis = 0;
#define resolution 12
#define TEMP_ERROR -999
float lastTemp = TEMP_ERROR;

bool isTimeSet = false;
unsigned long lastMinutes;

// see https://escapequotes.net/esp8266-wemos-d1-mini-pins-and-diagram/ for pinout
#define PIR 16     // D0
int pirState = LOW;

#define NIGHTLIGHT 5     // D1
int nightlightState = LOW;

WiFiClient espClient;
PubSubClient client(espClient);

#define HOST_NAME "IR"
#define MQTT_IP_ADDR "192.168.1.210"
#define MQTT_IP_PORT 1883

bool isPromModified;
bool isMemoryReset = false;
//bool isMemoryReset = true;

typedef struct {
  char host_name[17];
  char mqtt_ip_addr[17];
  int mqtt_ip_port;
} configType;

configType config;

#define MAX_CODES 50

typedef struct {
  char name[13];
  decode_type_t type;
  uint64_t data;
  uint16_t nbits;
} codeType;

codeType code[MAX_CODES];
int numCodes;


void setup() {
  // start serial port
  Serial.begin(115200);
  Serial.print(F("\n\n"));

  Serial.println(F("esp8266 IR"));
  Serial.println(F("compiled:"));
  Serial.print( __DATE__);
  Serial.print(F(","));
  Serial.println( __TIME__);

  // must specify amount of eeprom to use (max is 4k?)
  EEPROM.begin(4*1024);
  
  loadConfig();
  loadCodeConfig();
  isMemoryReset = false;

  if (!setupTemperature()) {
//    return;
  }
    
  if (!setupWifi())
    return;

  setupTime();
  setupWebServer();  
  setupMqtt();
  setupOta();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  irsend.begin();
  irrecv.enableIRIn();  // Start the receiver

  isUploading = false;

  lastMinutes = 0;

  pinMode(PIR, INPUT);

  pinMode(NIGHTLIGHT, OUTPUT);
  digitalWrite(NIGHTLIGHT, nightlightState);

  isSetup = true;
}


#define MAGIC_NUM   0xAF

#define MAGIC_NUM_ADDRESS      0
#define CONFIG_ADDRESS         1
#define CODE_SIZE_ADDRESS      CONFIG_ADDRESS + sizeof(config)
#define CODE_ADDRESS           CODE_SIZE_ADDRESS + sizeof(numCodes)

void set(char *name, const char *value) {
  for (int i=strlen(value); i >= 0; --i)
    *(name++) = *(value++);
}

void loadConfig(void) {
  int magicNum = EEPROM.read(MAGIC_NUM_ADDRESS);
  if (magicNum != MAGIC_NUM) {
    Serial.println(F("invalid eeprom data"));
    isMemoryReset = true;
  }
  
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.println(F("using default config"));
    set(config.host_name, HOST_NAME);
    set(config.mqtt_ip_addr, MQTT_IP_ADDR);
    config.mqtt_ip_port = MQTT_IP_PORT;

    saveConfig();
  }
  else {
    int addr = CONFIG_ADDRESS;
    byte *ptr = (byte *)&config;
    for (int i=0; i < sizeof(config); ++i, ++ptr)
      *ptr = EEPROM.read(addr++);
  }

  Serial.printf("host_name %s\n", config.host_name);
  Serial.printf("mqqt_ip_addr %s\n", config.mqtt_ip_addr);
  Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
}


void loadCodeConfig(void) {
  if (isMemoryReset) {
    // nothing saved in eeprom, use defaults
    Serial.printf("using default codes\n");
    initCode();
  }
  else {
    Serial.printf("loading codes from eeprom\n");
    int addr = CODE_SIZE_ADDRESS;
    byte *ptr = (byte *)&numCodes;
    for (int i = 0; i < sizeof(numCodes); ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
    Serial.printf("%d codes\n", numCodes);

    addr = CODE_ADDRESS;
    ptr = (byte *)&code;
    for (int i = 0; i < sizeof(codeType) * numCodes; ++i, ++ptr, ++addr)
      *ptr = EEPROM.read(addr);
  }
  for (int i=0; i < numCodes; ++i) {
    Serial.printf("code %d: ", (i+1));
    printCode(code[i]);
  }
}


void initCode(void) {
  numCodes = 0;
  saveCodeConfig(); 
}


void saveConfig(void) {
  isPromModified = false;
  update(MAGIC_NUM_ADDRESS, MAGIC_NUM);

  byte *ptr = (byte *)&config;
  int addr = CONFIG_ADDRESS;
  for (int j=0; j < sizeof(config); ++j, ++ptr)
    update(addr++, *ptr);
  
  if (isPromModified)
    EEPROM.commit();
}


void saveCodeConfig(void) {
  isPromModified = false;
  Serial.printf("saving codes to eeprom\n");
  int addr = CODE_SIZE_ADDRESS;
  Serial.printf("num codes %d\n", numCodes);
  byte *ptr = (byte *)&numCodes;
  for (int i = 0; i < sizeof(numCodes); ++i, ++ptr, ++addr)
    update(addr, *ptr);

  addr = CODE_ADDRESS;
  ptr = (byte *)&code;
  int num = sizeof(codeType) * numCodes;
  Serial.printf("code bytes %d\n", num);
  for (int i = 0; i < num; ++i, ++ptr, ++addr)
    update(addr, *ptr);

  if (isPromModified) {
    Serial.printf("calling prom commit\n");
    EEPROM.commit();
  }
}


void update(int addr, byte data) {
  if (EEPROM.read(addr) != data) {
    EEPROM.write(addr, data);
    isPromModified = true;
  }
}


void setupOta(void) {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(config.host_name);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    const char *msg = "Unknown Error";
    if (error == OTA_AUTH_ERROR) {
      msg = "Auth Failed";
    } else if (error == OTA_BEGIN_ERROR) {
      msg = "Begin Failed";
    } else if (error == OTA_CONNECT_ERROR) {
      msg = "Connect Failed";
    } else if (error == OTA_RECEIVE_ERROR) {
      msg = "Receive Failed";
    } else if (error == OTA_END_ERROR) {
      msg = "End Failed";
    }
    Serial.println(msg);
  });
  
  ArduinoOTA.begin();
  Serial.println("Arduino OTA ready");

  char host[20];
  sprintf(host, "%s-webupdate", config.host_name);
  MDNS.begin(host);
  httpUpdater.setup(&server);
  MDNS.addService("http", "tcp", 80);
  Serial.println("Web OTA ready");
}


void setupMqtt() {
  client.setServer(config.mqtt_ip_addr, config.mqtt_ip_port);
  client.setCallback(callback);
}


void loop() {
  if (isUploading) {
    server.handleClient();
    return;
  }
  
  if (!isSetup)
    return;

  unsigned long time = millis();

  checkTime(time);
  checkTemperature(time);

  webSocket.loop();
  server.handleClient();
  MDNS.update();
 
  // mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkTemperature();

  checkIRreceiver();

  checkPir();

  ArduinoOTA.handle();
}


void checkIRreceiver(void) {
  if (irrecv.decode(&results)) {
    printIRreceiver();
    irrecv.resume();  // Receive the next value
  }  
}


void printCode(codeType c) {
  Serial.print(typeToString(c.type, false));
  Serial.print(" ");
  Serial.print(resultToHexidecimal2(c.data));
  Serial.print(" ");
  Serial.println(uint64ToString(c.nbits));
}


void printIRreceiver(void) {
  String typeS = typeToString(results.decode_type, results.repeat);
  String dataS = resultToHexidecimal(&results);
  String nbitsS = uint64ToString(results.bits);
  const char *type = typeS.c_str();
  const char *data = dataS.c_str();
  const char *nbits = nbitsS.c_str();

  // cpd...test this with samsumg36 power (soundbar)
  Serial.printf("received IR: %s %s %s\n", type, data, nbits);

  // if the codes page is connected, sent it the value
  // otherwise, look up the code for the command, and send to mqtt
  if (codesClient != -1) {
    char json[128];
    sprintf(json, "{\"command\":\"code\",\"value\":{\"type\":\"%s\",\"data\":\"%s\",\"nbits\":\"%s\"}}",
      type, data, nbits);
    webSocket.sendTXT(codesClient, json, strlen(json));
  }
  else {
    int index = getCodeFromResult();
    if (index != -1) {
      codeType c = code[index];
      // mqtt
      Serial.printf("publish to mqtt: %s\n", c.name);
      char topic[20];
      sprintf(topic, "%s/ir", config.host_name);
      client.publish(topic, c.name);

      // also send to main display
      if (webClient != -1) {
        sendWeb("code", c.name);
      }
    }
    else {
      Serial.printf("no match\n");
    }
  }
}

// dupe of resultToHexidecimal, but without the result struct
String resultToHexidecimal2(uint64_t data) {
  String output = F("0x");
  output += uint64ToString(data, 16);
  return output;
}


int getCodeFromResult(void) {
  for (int i=0; i < numCodes; ++i) {
    codeType c = code[i];
    Serial.printf("comparing %d: ", i);
    printCode(c); 
    if (c.type == results.decode_type && c.data == results.value) {
      Serial.printf("found match with %s\n", c.name);
      return i;
    }
  }
  return -1;
}


int getCodeIndex(const char *name) {
  for (int i=0; i < numCodes; ++i) {
    if (strcmp(code[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}


void checkPir(void) {
  int val = digitalRead(PIR);   // read input value
//  Serial.println(val);
  if (val == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) {
      movement(true);
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH) {
      movement(false);
      pirState = LOW;
    }
  }
}


void movement(bool isMoved) {
//  Serial.print("movement: ");
//  Serial.println(isMoved ? "started" : "stopped");

  if (webClient != -1) {
    sendWeb("movement", isMoved ? "Movement" : "");  
  }

  // mqtt
  char topic[20];
  sprintf(topic, "%s/movement", config.host_name);
  client.publish(topic, isMoved ? "started" : "stopped");
}


void checkTime(unsigned long time) {
  int minutes = minute();

  if (minutes == lastMinutes)
    return;

  // resync time at 3am every morning
  // this also catches daylight savings time changes which happen at 2am
  if (minutes == 0 && hour() == 3)
    isTimeSet = false;

  if (!isTimeSet)
    setupTime();
 
  lastMinutes = minutes;
  printTime();
}


void printNightlight() {
  if (webClient == -1)
    return;

  sendWeb("nightlight", nightlightState ? "1" : "0");
}


const char *weekdayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

void printTime() {
  if (webClient == -1)
    return;
    
  int dayOfWeek = weekday()-1;
  int hours = hour();
  int minutes = minute();
  const char *ampm = "a";
  int h = hours;
  if (hours == 0)
    h = 12;
  else if (h == 12)
    ampm = "p";
  else if (hours > 12) {
    h -= 12;
    ampm = "p";
  }
  
  char buf[10];
  sprintf(buf, "  %2d:%02d%s", h, minutes, ampm); 
//  Serial.println(buf);

  char msg[6+1+4];
  sprintf(msg, "%s %s", buf, weekdayNames[dayOfWeek]); 
  sendWeb("time", msg);
}


void configModeCallback(WiFiManager *myWiFiManager) {
  // this callback gets called when the enter AP mode, and the users
  // need to connect to us in order to configure the wifi
  Serial.print(F("Join:"));
  Serial.println(config.host_name);
  Serial.print(F("Goto:"));
  Serial.println(WiFi.softAPIP());
}

bool setupTemperature(void) {
  sensors.begin();
  if (sensors.getDeviceCount() != 1) {
    Serial.println("Unable to locate temperature sensor");
    return false;
  }
  if (!sensors.getAddress(thermometer, 0)) {
    Serial.println("Unable to find address for temperature sensor"); 
    return false;
  }
  sensors.setResolution(thermometer, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution)); 
  lastTempRequest = millis(); 

  return true;
}

void checkTemperature(unsigned long time) {
  if (time - lastTempRequest >= delayInMillis) // waited long enough??
  {
//  Serial.println(F("checking temperature"));
    checkTemperature();
        
    sensors.requestTemperatures(); 
    lastTempRequest = millis(); 
  }
}

void checkTemperature(void) {
  float tempF = sensors.getTempF(thermometer);
  tempF = (float)round(tempF*10)/10.0;
  if ( tempF == lastTemp )
    return;
    
  lastTemp = tempF;

//  Serial.print("Temperature is ");
//  Serial.println(tempF);

  printCurrentTemperature();
}

bool setupWifi(void) {
  WiFi.hostname(config.host_name);
  
  WiFiManager wifiManager;
//  wifiManager.setDebugOutput(false);
  
  //reset settings - for testing
  //wifiManager.resetSettings();

  String ssid = WiFi.SSID();
  if (ssid.length() > 0) {
    Serial.print(F("Connecting to "));
    Serial.println(ssid);
  }
  
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  if(!wifiManager.autoConnect(config.host_name)) {
    Serial.println(F("failed to connect and hit timeout"));
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  return true;
}

void setupTime(void) {
  Serial.println(F("Getting time"));

  AsyncUDP* udp = new AsyncUDP();

  // time.nist.gov NTP server
  // NTP requests are to port 123
  if (udp->connect(IPAddress(129,6,15,28), 123)) {
//    Serial.println("UDP connected");
    
    udp->onPacket([](void *arg, AsyncUDPPacket packet) {
//      Serial.println(F("received NTP packet"));
      byte *buf = packet.data();
      
      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
    
      // convert four bytes starting at location 40 to a long integer
      unsigned long secsSince1900 =  (unsigned long)buf[40] << 24;
      secsSince1900 |= (unsigned long)buf[41] << 16;
      secsSince1900 |= (unsigned long)buf[42] << 8;
      secsSince1900 |= (unsigned long)buf[43];
      time_t utc = secsSince1900 - 2208988800UL;
    
      TimeChangeRule *tcr;
      time_t local = myTZ.toLocal(utc, &tcr);
      Serial.printf("\ntime zone %s\n", tcr->abbrev);
    
      setTime(local);
    
      // just print out the time
      printTime();
    
      isTimeSet = true;

      free(arg);
    }, udp);
    
//    Serial.println(F("sending NTP packet"));

    const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
    byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold outgoing packet

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    
    // all NTP fields have been given values, now
    // send a packet requesting a timestamp:
    udp->write(packetBuffer, NTP_PACKET_SIZE);
  }
  else {
    free(udp);
    Serial.println(F("\nWiFi:time failed...will retry in a minute"));
  }
}


// 38kHz carrier frequency (soundbar, tv)
#define KHZ_38 38

// 56kHz carrier frequency for the xbox usb dongle?  (not verified)
#define KHZ_56 56 


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      if (num == webClient)
        webClient = -1;
      else if (num == setupClient)
        setupClient = -1;
      else if (num == remoteClient)
        remoteClient = -1;
      else if (num == codesClient)
        codesClient = -1;
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }      

      if (strcmp((char *)payload,"/") == 0) {
        webClient = num;
      
        // send the current state
        printCurrentTemperature();
        printTime();
        printNightlight();
      }
      else if (strcmp((char *)payload,"/setup") == 0) {
        setupClient = num;

        char json[256];
        strcpy(json, "{");
        sprintf(json+strlen(json), "\"date\":\"%s\"", __DATE__);
        sprintf(json+strlen(json), ",\"time\":\"%s\"", __TIME__);
        sprintf(json+strlen(json), ",\"host_name\":\"%s\"", config.host_name);
        sprintf(json+strlen(json), ",\"mqtt_ip_addr\":\"%s\"", config.mqtt_ip_addr);
        sprintf(json+strlen(json), ",\"mqtt_ip_port\":\"%d\"", config.mqtt_ip_port);
        strcpy(json+strlen(json), "}");
//        Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(setupClient, json, strlen(json));
      }
      else if (strcmp((char *)payload,"/remote") == 0) {
        remoteClient = num;
      }
      else if (strcmp((char *)payload,"/codes") == 0) {
        codesClient = num;
              
        // send codes
        char json[1024];
        strcpy(json, "{\"command\":\"codes\",\"value\":[");
        for (int i=0; i < numCodes; ++i) {
          codeType c = code[i];
          String typeS = typeToString(c.type, false);
          String dataS = resultToHexidecimal2(c.data);
          String nbitsS = uint64ToString(c.nbits);
          const char *type = typeS.c_str();
          const char *data = dataS.c_str();
          const char *nbits = nbitsS.c_str();
          sprintf(json+strlen(json), "%s[\"%s\",\"%s\",\"%s\",\"%s\"]",
            (i==0)?"":",",
            c.name, type, data, nbits);
        }
        strcpy(json+strlen(json), "]}");
        //Serial.printf("len %d\n", strlen(json));
        webSocket.sendTXT(codesClient, json, strlen(json));
      }
      else {
        Serial.printf("unknown call %s\n", payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);

      if (num == webClient) {
        // no commands on main page
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"nightlight",10) == 0) {
          target = "value";
          ptr = strstr(ptr, target) + strlen(target)+3;
          int value = strtol(ptr, &ptr, 10);
          nightlightState = (value == 0) ? LOW : HIGH;
          digitalWrite(NIGHTLIGHT, nightlightState);
          Serial.printf("nightlight %d\n", nightlightState);

          // mqtt
          char topic[30];
          sprintf(topic, "%s/nightlight", config.host_name);
          client.publish(topic, ((nightlightState == LOW) ? "off" : "on"));
        }
      }
      else if (num == setupClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"reboot",6) == 0) {
          ESP.restart();
        }
        else if (strncmp(ptr,"save",4) == 0) {
          Serial.printf("save setup\n");
          
          const char *target = "host_name";
          char *ptr = strstr((char *)payload, target) + strlen(target)+3;
          char *end = strchr(ptr, '\"');
          memcpy(config.host_name, ptr, (end-ptr));
          config.host_name[end-ptr] = '\0';

          target = "mqtt_ip_addr";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          end = strchr(ptr, '\"');
          memcpy(config.mqtt_ip_addr, ptr, (end-ptr));
          config.mqtt_ip_addr[end-ptr] = '\0';

          target = "mqtt_ip_port";
          ptr = strstr((char *)payload, target) + strlen(target)+3;
          config.mqtt_ip_port = strtol(ptr, &ptr, 10);

          Serial.printf("host_name %s\n", config.host_name);
          Serial.printf("mqtt_ip_addr %s\n", config.mqtt_ip_addr);
          Serial.printf("mqtt_ip_port %d\n", config.mqtt_ip_port);
          saveConfig();
        }
      }
      else if (num == remoteClient) {
// cpd...fix thes with stored codes (lookup by name)
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"Power",5) == 0) {
//Protocol  : NEC
//Code      : 0x20DF10EF (32 Bits)
uint16_t rawData[71] = {8910, 4450,  572, 562,  550, 562,  548, 1652,  574, 562,  550, 562,  550, 562,  550, 562,  550, 562,  550, 1656,  570, 1650,  574, 562,  550, 1654,  572, 1650,  576, 1650,  576, 1650,  574, 1654,  574, 560,  550, 562,  550, 562,  550, 1654,  572, 562,  550, 562,  548, 562,  550, 562,  550, 1650,  576, 1652,  574, 1648,  576, 562,  550, 1650,  576, 1652,  574, 1652,  576, 1650,  574, 39560,  8910, 2238,  574};  // NEC 20DF10EF
//uint32_t address = 0x4;
//uint32_t command = 0x8;
//uint64_t data = 0x20DF10EF;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_38);
//            irsend.sendNEC(data);
//            irsend.send(NEC, data, 32, 1);
        }
        else if (strncmp(ptr,"Vol-Up",6) == 0) {
//Protocol  : NEC
//Code      : 0x20DF40BF (32 Bits)
uint16_t rawData[67] = {8954, 4466,  550, 592,  524, 592,  524, 1686,  550, 592,  524, 592,  524, 592,  524, 592,  524, 592,  524, 1688,  548, 1684,  550, 592,  524, 1684,  550, 1686,  550, 1686,  550, 1688,  548, 1684,  552, 592,  524, 1686,  548, 592,  524, 594,  524, 592,  524, 592,  524, 592,  524, 592,  524, 1686,  550, 592,  524, 1686,  548, 1686,  550, 1686,  550, 1686,  550, 1684,  552, 1688,  548};  // NEC 20DF40BF
//uint32_t address = 0x4;
//uint32_t command = 0x2;
//uint64_t data = 0x20DF40BF;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_38);
//            irsend.sendNEC(data);
//            irsend.send(NEC, data, 32, 1);
        }
        else if (strncmp(ptr,"Vol-Down",7) == 0) {
//Protocol  : NEC
//Code      : 0x20DFC03F (32 Bits)
uint16_t rawData[67] = {8954, 4464,  552, 592,  524, 592,  550, 1660,  550, 590,  524, 592,  550, 566,  524, 592,  548, 568,  524, 1684,  552, 1684,  576, 566,  526, 1684,  552, 1684,  552, 1684,  576, 1658,  552, 1684,  578, 1656,  554, 1682,  578, 566,  524, 592,  524, 592,  526, 590,  524, 592,  524, 594,  524, 592,  550, 566,  550, 1658,  578, 1658,  552, 1684,  576, 1660,  576, 1660,  576, 1658,  580};  // NEC 20DFC03F
//uint32_t address = 0x4;
//uint32_t command = 0x3;
//uint64_t data = 0x20DFC03F;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_38);
//            irsend.sendNEC(data);
//            irsend.send(NEC, data, 32, 1);
        }
        else if (strncmp(ptr,"Mute",4) == 0) {
//Protocol  : NEC
//Code      : 0x20DF906F (32 Bits)
uint16_t rawData[67] = {8952, 4466,  550, 594,  524, 592,  524, 1682,  552, 592,  524, 592,  524, 592,  524, 592,  524, 592,  524, 1684,  552, 1684,  552, 592,  524, 1684,  550, 1686,  550, 1684,  552, 1684,  550, 1686,  550, 1682,  552, 592,  524, 592,  524, 1684,  550, 592,  524, 592,  524, 592,  524, 592,  524, 592,  524, 1686,  550, 1684,  552, 592,  524, 1684,  552, 1684,  550, 1684,  552, 1686,  548};  // NEC 20DF906F
//uint32_t address = 0x4;
//uint32_t command = 0x9;
//uint64_t data = 0x20DF906F;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_38);
//            irsend.sendNEC(data);
//            irsend.send(NEC, data, 32, 1);
        }
        else if (strncmp(ptr,"SB-Power",8) == 0) {
//Protocol  : SAMSUNG36
//Code      : 0xCF000EF1 (36 Bits)
uint16_t rawData[77] = {4490, 4512,  504, 522,  480, 524,  480, 524,  480, 524,  480, 1528,  480, 1526,  480, 526,  480, 524,  480, 1526,  482, 1526,  480, 1528,  480, 1526,  480, 524,  480, 524,  480, 524,  480, 524,  480, 4496,  504, 524,  480, 524,  480, 524,  480, 524,  480, 524,  480, 524,  478, 524,  480, 524,  480, 1526,  480, 1526,  480, 1528,  478, 524,  480, 1528,  480, 1526,  480, 1526,  480, 1528,  480, 524,  480, 524,  480, 524,  480, 1526,  480};  // SAMSUNG36 CF000EF1
//uint32_t address = 0xCF0;
//uint32_t command = 0xEF1;
//uint64_t data = 0xCF000EF1;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_38);
//            irsend.sendSamsung36(data);
//            irsend.send(SAMSUNG36, data, 32, 1);
        }
        else if (strncmp(ptr,"Up",2) == 0) {
//Protocol  : NIKAI
//Code      : 0xAA6559 (24 Bits)
uint16_t rawData[51] = {3872, 4098,  374, 1116,  374, 2090,  394, 1116,  374, 2092,  392, 1116,  374, 2066,  418, 1116,  374, 2042,  440, 2066,  420, 1116,  374, 1116,  374, 2092,  392, 2044,  440, 1116,  374, 2092,  392, 1116,  374, 2092,  392, 1116,  374, 2092,  392, 1116,  374, 1116,  374, 2066,  418, 2084,  400, 1116,  374};  // NIKAI AA6559
//uint64_t data = 0xAA6559;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Down",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAA7558 (24 Bits)
uint16_t rawData[51] = {3896, 4080,  392, 1116,  374, 2066,  418, 1116,  374, 2092,  392, 1116,  374, 2090,  392, 1116,  374, 2066,  418, 2066,  418, 1116,  376, 1116,  374, 1116,  372, 2066,  418, 1116,  374, 2066,  418, 1116,  374, 2092,  392, 1116,  374, 2092,  394, 1114,  374, 1116,  374, 2092,  392, 2066,  418, 2066,  418};  // NIKAI AA7558
//uint64_t data = 0xAA7558;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Left",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAA9556 (24 Bits)
uint16_t rawData[103] = {3898, 4054,  418, 1092,  398, 2064,  418, 1086,  406, 2064,  418, 1106,  384, 2066,  418, 1112,  378, 2064,  418, 1102,  388, 2066,  420, 2064,  418, 1094,  396, 2042,  442, 1114,  374, 2066,  420, 1070,  418, 2064,  420, 1116,  374, 2066,  418, 1112,  378, 2066,  418, 1114,  376, 1114,  376, 2064,  418, 8048,  3898, 4040,  432, 1114,  376, 2066,  418, 1092,  398, 2066,  418, 1098,  392, 2064,  420, 1070,  420, 2064,  420, 1094,  396, 2064,  420, 2064,  420, 1092,  400, 2042,  442, 1092,  398, 2066,  418, 1092,  398, 2064,  420, 1092,  398, 2042,  442, 1092,  398, 2066,  418, 1092,  398, 1092,  396, 2066,  420};  // NIKAI AA9556
//uint64_t data = 0xAA9556;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Right",5) == 0) {
//Protocol  : NIKAI
//Code      : 0xAA8557 (24 Bits)
uint16_t rawData[51] = {3896, 4052,  420, 1072,  420, 2064,  418, 1070,  420, 2042,  442, 1092,  398, 2066,  418, 1070,  420, 2042,  442, 1092,  398, 2066,  418, 2064,  420, 2044,  440, 2066,  418, 1082,  408, 2058,  426, 1092,  398, 2042,  442, 1070,  420, 2042,  442, 1092,  398, 2066,  418, 1092,  398, 1092,  398, 1092,  398};  // NIKAI AA8557
//uint64_t data = 0xAA8557;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Stop",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAE051F (24 Bits)
uint16_t rawData[155] = {3898, 4054,  420, 1114,  376, 2042,  442, 1104,  386, 2066,  418, 1114,  376, 1114,  376, 1114,  376, 2104,  380, 2064,  420, 2066,  420, 2062,  420, 2066,  418, 2066,  418, 1114,  374, 2066,  420, 1114,  376, 2066,  418, 2066,  418, 2066,  418, 1114,  376, 1114,  374, 1116,  376, 1114,  376, 1094,  396, 8050,  3896, 4054,  418, 1114,  376, 2042,  442, 1114,  376, 2064,  420, 1114,  376, 1112,  378, 1092,  398, 2066,  418, 2066,  418, 2044,  440, 2066,  418, 2066,  418, 2066,  418, 1116,  374, 2066,  418, 1092,  398, 2066,  418, 2066,  418, 2066,  418, 1114,  376, 1114,  376, 1114,  374, 1116,  376, 1092,  396, 8050,  3896, 4054,  418, 1114,  376, 2064,  418, 1072,  420, 2064,  418, 1094,  398, 1116,  374, 1116,  374, 2066,  420, 2064,  418, 2066,  418, 2066,  418, 2064,  420, 2064,  420, 1092,  396, 2066,  420, 1092,  396, 2066,  418, 2064,  418, 2064,  420, 1094,  396, 1072,  418, 1072,  418, 1072,  418, 1094,  396};  // NIKAI AE051F
//uint64_t data = 0xAE051F;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Play",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAEA515 (24 Bits)
uint16_t rawData[103] = {3898, 4054,  418, 1072,  418, 2042,  442, 1094,  396, 2042,  442, 1092,  398, 1070,  420, 1070,  420, 2066,  420, 1110,  380, 2066,  420, 1114,  376, 2066,  418, 2066,  416, 1072,  420, 2064,  420, 1070,  420, 2042,  442, 2042,  442, 2042,  442, 1070,  420, 2064,  420, 1072,  420, 2042,  440, 1094,  398, 8048,  3898, 4052,  420, 1092,  398, 2064,  420, 1092,  398, 2064,  418, 1114,  376, 1092,  398, 1092,  398, 2064,  420, 1106,  384, 2066,  418, 1094,  396, 2042,  442, 2066,  418, 1094,  396, 2042,  442, 1092,  398, 2066,  418, 2066,  418, 2066,  418, 1092,  398, 2042,  442, 1092,  398, 2066,  418, 1072,  418};  // NIKAI AEA515
//uint64_t data = 0xAEA515;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Select",6) == 0) {
//Protocol  : NIKAI
//Code      : 0xA0B5F4 (24 Bits)
uint16_t rawData[51] = {3872, 4054,  418, 1116,  374, 2066,  418, 1116,  374, 2068,  416, 2066,  418, 2090,  394, 2078,  406, 2066,  418, 1116,  374, 2066,  418, 1116,  374, 1116,  374, 2092,  392, 1116,  374, 2066,  418, 1116,  374, 1116,  374, 1116,  374, 1116,  374, 1116,  374, 2044,  440, 1116,  374, 2066,  418, 2088,  396};  // NIKAI A0B5F4
//uint64_t data = 0xA0B5F4;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Menu",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAF7508 (24 Bits)
uint16_t rawData[103] = {3896, 4054,  418, 1116,  374, 2066,  418, 1116,  374, 2076,  408, 1114,  376, 1114,  376, 1116,  374, 1116,  374, 2112,  374, 1114,  376, 1116,  374, 1116,  374, 2066,  418, 1116,  374, 2092,  394, 1116,  374, 2066,  418, 2066,  418, 2090,  394, 2090,  392, 1116,  374, 2066,  418, 2064,  420, 2066,  418, 8094,  3852, 4056,  418, 1114,  374, 2068,  416, 1116,  374, 2066,  420, 1114,  374, 1116,  374, 1116,  374, 1114,  376, 2066,  418, 1116,  374, 1116,  374, 1116,  374, 2090,  394, 1116,  374, 2064,  418, 1116,  374, 2064,  418, 2064,  420, 2064,  420, 2066,  418, 1116,  374, 2066,  418, 2066,  418, 2066,  418};  // NIKAI AF7508
//uint64_t data = 0xAF7508;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else if (strncmp(ptr,"Exit",4) == 0) {
//Protocol  : NIKAI
//Code      : 0xAD8527 (24 Bits)
uint16_t rawData[103] = {3920, 4054,  418, 1072,  420, 2042,  442, 1078,  412, 2042,  442, 1092,  398, 1070,  420, 2066,  418, 1070,  420, 1070,  418, 2042,  442, 2064,  420, 2064,  420, 2064,  420, 1092,  398, 2066,  418, 1092,  398, 2066,  418, 2044,  442, 1092,  398, 2044,  440, 2042,  442, 1090,  400, 1092,  398, 1092,  398, 8028,  3920, 4054,  418, 1094,  396, 2064,  420, 1048,  440, 2066,  418, 1072,  418, 1070,  420, 2064,  420, 1072,  416, 1050,  442, 2042,  440, 2066,  418, 2042,  442, 2064,  418, 1072,  418, 2066,  418, 1048,  442, 2042,  442, 2066,  418, 1072,  418, 2066,  418, 2044,  440, 1072,  418, 1072,  418, 1072,  418};  // NIKAI AD8527
//uint64_t data = 0xAD8527;
            irsend.sendRaw(rawData, sizeof(rawData) / sizeof(rawData[0]), KHZ_56);
//            irsend.sendNikai(data);
//            irsend.send(NIKAI, data, 24, 1);
        }
        else {
          Serial.printf("Unknown remote command\n");
        }
      }
      else if (num == codesClient) {
        const char *target = "command";
        char *ptr = strstr((char *)payload, target) + strlen(target)+3;
        if (strncmp(ptr,"saveCodes",9) == 0) {
          Serial.printf("save codes\n");

          const char *target = "num";
          char *ptr = strstr((char *)payload, target) + strlen(target)+2;
          numCodes = strtol(ptr, &ptr, 10);
          Serial.printf("got %d codes\n", numCodes);

          char type[20];
          char data[20];
          char nbits[20];
          target = "codes";
          ptr = strstr((char *)payload, target) + strlen(target)+5;
          for (int i=0; i < numCodes; ++i) {
            char *end = strchr(ptr, '\"');
            memcpy(code[i].name, ptr, (end-ptr));
            code[i].name[end-ptr] = '\0';
            ptr = end+3;
            end = strchr(ptr, '\"');
            memcpy(type, ptr, (end-ptr));
            type[end-ptr] = '\0';
            ptr = end+3;
            end = strchr(ptr, '\"');
            memcpy(data, ptr, (end-ptr));
            data[end-ptr] = '\0';
            ptr = end+3;
            end = strchr(ptr, '\"');
            memcpy(nbits, ptr, (end-ptr));
            nbits[end-ptr] = '\0';
            ptr = end+5;
            Serial.printf("'%s' '%s' '%s' '%s'\n", code[i].name, type, data, nbits);
            code[i].type = strToDecodeType(type);
            code[i].data = getUInt64fromHex(data+2); 
            code[i].nbits = strtoul(nbits, NULL, 10); 
            Serial.printf("got code: ");
            printCode(code[i]);
          }
          saveCodeConfig();
        }
        else if (strncmp(ptr,"sendCode",8) == 0) {
          Serial.printf("send code\n");
          codeType acode;
          char type[20];
          char data[20];
          char nbits[20];
          target = "code";
          ptr = strstr((char *)payload, target) + strlen(target)+4;
          char *end = strchr(ptr, '\"');
          memcpy(acode.name, ptr, (end-ptr));
          acode.name[end-ptr] = '\0';
          ptr = end+3;
          end = strchr(ptr, '\"');
          memcpy(type, ptr, (end-ptr));
          type[end-ptr] = '\0';
          ptr = end+3;
          end = strchr(ptr, '\"');
          memcpy(data, ptr, (end-ptr));
          data[end-ptr] = '\0';
          ptr = end+3;
          end = strchr(ptr, '\"');
          memcpy(nbits, ptr, (end-ptr));
          nbits[end-ptr] = '\0';
          Serial.printf("'%s' '%s' '%s' '%s'\n", acode.name, type, data, nbits);
          acode.type = strToDecodeType(type);
          acode.data = getUInt64fromHex(data+2); 
          acode.nbits = strtoul(nbits, NULL, 10); 

          // ignore receiving this command we're sending out
          irrecv.disableIRIn();  // Stop the receiver
          Serial.printf("ir: ");
          printCode(acode);
          irsend.send(acode.type, acode.data, acode.nbits, 1);   // send the code out
          irrecv.enableIRIn();  // Start the receiver
        }
      }
      break;
  }
}


uint64_t getUInt64fromHex(char const *str) {
    uint64_t accumulator = 0;
    for (size_t i = 0 ; isxdigit((unsigned char)str[i]) ; ++i) {
        char c = str[i];
        accumulator *= 16;
        if (isdigit(c)) /* '0' .. '9'*/
            accumulator += c - '0';
        else if (isupper(c)) /* 'A' .. 'F'*/
            accumulator += c - 'A' + 10;
        else /* 'a' .. 'f'*/
            accumulator += c - 'a' + 10;
    }
    return accumulator;
}


void sendWeb(const char *command, const char *value) {
  send(webClient, command, value);
}

void send(int client, const char *command, const char *value) {
  char json[128];
  sprintf(json, "{\"command\":\"%s\",\"value\":\"%s\"}", command, value);
  webSocket.sendTXT(client, json, strlen(json));
}


void printCurrentTemperature() {
  char buf[7];
  dtostrf(lastTemp, 4, 1, buf);

  if (webClient != -1) {
    sendWeb("currentTemp", buf);  
  }
  
  // mqtt
  char topic[20];
  sprintf(topic, "%s/temperature", config.host_name);
  client.publish(topic, buf);
}


//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload_edit(){
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //Serial.print("handleFileUpload Data: "); Serial.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  Serial.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  Serial.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

void countRootFiles(void) {
  int num = 0;
  size_t totalSize = 0;
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    ++num;
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    totalSize += fileSize;
    Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  Serial.printf("FS File: serving %d files, size: %s from /\n", num, formatBytes(totalSize).c_str());
}

void setupWebServer(void) {
  SPIFFS.begin();

  countRootFiles();

  // list directory
  server.on("/list", HTTP_GET, handleFileList);
  
  // load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  
  // create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  
  // delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  
  // first callback is called after the request has ended with all parsed arguments
  // second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload_edit);

  //called when the url is not defined here
  //use it to load content from SPIFFS
  server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();

  Serial.println("HTTP server started");
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
//    // Create a random client ID
//    String clientId = "ESP8266Client-";
//    clientId += String(random(0xffff), HEX);
//    // Attempt to connect
//    if (client.connect(clientId.c_str())) {
    if (client.connect(config.host_name)) {
      Serial.println("connected");
      // ... and resubscribe
      char topic[30];
      sprintf(topic, "%s/command", config.host_name);
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  // only topic we get is <host_name>/command or nightlight
  
  char value[12];
  memcpy(value, payload, length);
  value[length] = '\0';
  Serial.printf("Message arrived [%s] %s\n", topic, value);

  if (strcmp(topic, "command") == 0) {
    // get ir code, and send it out
    int index = getCodeIndex(value);
    if (index == -1) {
      Serial.printf("Unknown code\n");
      return;  
    }
    
    // ignore receiving this command we're sending out
    irrecv.disableIRIn();  // Stop the receiver
    codeType c = code[index];
    Serial.printf("ir: ");
    printCode(c);
    irsend.send(c.type, c.data, c.nbits, 1);   // send the code out
    irrecv.enableIRIn();  // Start the receiver
  
    // also send to main display
    if (webClient != -1) {
      sendWeb("code", value);
    }
  }
  else if (strcmp(topic, "nightlight") == 0) {
    if (strcmp(value, "off") == 0) {
      nightlightState = LOW;
    }
    else if (strcmp(value, "on") == 0) {
      nightlightState = HIGH;
    }
    else {
      Serial.printf("Unknown command\n");
      return;        
    }
    
    digitalWrite(NIGHTLIGHT, nightlightState);
    // also send to main display
    printNightlight();
  }
  else {
    Serial.printf("Unknown topic\n");
  }
}
