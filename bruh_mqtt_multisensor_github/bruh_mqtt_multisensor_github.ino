/*
  .______   .______    __    __   __    __          ___      __    __  .___________.  ______   .___  ___.      ___   .___________. __    ______   .__   __.
  |   _  \  |   _  \  |  |  |  | |  |  |  |        /   \    |  |  |  | |           | /  __  \  |   \/   |     /   \  |           ||  |  /  __  \  |  \ |  |
  |  |_)  | |  |_)  | |  |  |  | |  |__|  |       /  ^  \   |  |  |  | `---|  |----`|  |  |  | |  \  /  |    /  ^  \ `---|  |----`|  | |  |  |  | |   \|  |
  |   _  <  |      /  |  |  |  | |   __   |      /  /_\  \  |  |  |  |     |  |     |  |  |  | |  |\/|  |   /  /_\  \    |  |     |  | |  |  |  | |  . `  |
  |  |_)  | |  |\  \-.|  `--'  | |  |  |  |     /  _____  \ |  `--'  |     |  |     |  `--'  | |  |  |  |  /  _____  \   |  |     |  | |  `--'  | |  |\   |
  |______/  | _| `.__| \______/  |__|  |__|    /__/     \__\ \______/      |__|      \______/  |__|  |__| /__/     \__\  |__|     |__|  \______/  |__| \__|

  Thanks much to @corbanmailloux for providing a great framework for implementing flash/fade with HomeAssistant https://github.com/corbanmailloux/esp-mqtt-rgb-led

  To use this code you will need the following dependancies: 
  
  - Support for the ESP8266 boards. 
        - You can add it to the board manager by going to File -> Preference and pasting http://arduino.esp8266.com/stable/package_esp8266com_index.json into the Additional Board Managers URL field.
        - Next, download the ESP8266 dependancies by going to Tools -> Board -> Board Manager and searching for ESP8266 and installing it.
  
  - You will also need to download the follow libraries by going to Sketch -> Include Libraries -> Manage Libraries
  - DHT sensor library (Ver 1.3.0, 1.3.4 was not working)
  - Adafruit unified sensor (Ver 1.0.3)
  - PubSubClient (Ver 2.7.0)
  - ArduinoJSON (Ver 5.13.5)
  - Kalman Lib (https://github.com/bachagas/Kalman)
  - TaskScheduler (Ver 3.0.2)
  

  UPDATE 16 MAY 2017 by Knutella - Fixed MQTT disconnects when wifi drops by moving around Reconnect and adding a software reset of MCU
	           
  UPDATE 23 MAY 2017 - The MQTT_MAX_PACKET_SIZE parameter may not be setting appropriately due to a bug in the PubSub library. If the MQTT messages are not being transmitted as expected you may need to change the MQTT_MAX_PACKET_SIZE parameter in "PubSubClient.h" directly.
  
  UPDATE 27 NOV 2017 - Changed HeatIndex to built in function of DHT library. Added definition for fahrenheit or celsius

*/



#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>   // Include the Wi-Fi-Multi library. Example from here: https://tttapa.github.io/ESP8266/Chap07%20-%20Wi-Fi%20Connections.html
#include <DHT_U.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Kalman.h>
#include <ESP8266WebServer.h>   // Include the WebServer library
#include <TaskScheduler.h>



/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define wifi_ssid "YourSSID" //type your WIFI information inside the quotes
#define wifi_password "YourWIFIpassword"
#define mqtt_server "your.mqtt.server.ip"
#define mqtt_user "yourMQTTusername" 
#define mqtt_password "yourMQTTpassword"
#define mqtt_port 1883



/************* MQTT TOPICS (change these topics as you wish)  **************************/
String light_state_topic;

//#define light_set_topic "esp8266/01/set"
String ChipID;



/**************************** FOR TaskScheduler ****************************************/
void readDHT();
void checkNewValues();

Task t_readDHT(2500, TASK_FOREVER, &readDHT);
Task t_checkNewValues(2500, TASK_FOREVER, &checkNewValues);

Scheduler runner;

/**************************** FOR OTA **************************************************/
#define SENSORNAME "sensornode1"
#define OTApassword "YouPassword" // change this to whatever password you want to use when you upload OTA
int OTAport = 8266;



/**************************** PIN DEFINITIONS ********************************************/


#define DHTPIN    D7
#define DHTTYPE   DHT22
#define LDRPIN    A0



/**************************** SENSOR DEFINITIONS *******************************************/
float ldrValue;
int LDR = 0;
float calcLDR;
float diffLDR = 25;

float diffTEMP = 0.2;
float tempValue;

float diffHUM = 0.5;
float humValue;
float humValue_Kalman;

float newTempValue;
float newHumValue;
float newHumValue_Kalman;

char message_buff[100];

int calibrationTime = 5;

const int BUFFER_SIZE = 300;

//#define MQTT_MAX_PACKET_SIZE 512

bool SendMQTT = false;
bool DHT22_read_OK = false;


WiFiClient espClient;
ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti
PubSubClient client(espClient);
DHT_Unified dht(DHTPIN, DHTTYPE);
//https://www.arduino.cc/en/Reference/WiFiMACAddress
byte mac[6];                     // the MAC address of your Wifi shield
int SendCounter = 0;

// create the Kalman filter
Kalman myFilter(0.125, 32, 1023, 0); //suggested initial values for high noise filtering

// WEb server, example from https://tttapa.github.io/ESP8266/Chap10%20-%20Simple%20Web%20Server.html
ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80
void handleRoot();              // function prototypes for HTTP handlers
void handleNotFound();


/********************************** START SETUP*****************************************/
void setup() {

  //testing the blink led 
  // example here: https://lowvoltage.github.io/2017/07/09/Onboard-LEDs-NodeMCU-Got-Two
  pinMode(LED_BUILTIN, OUTPUT);
  String MDNS_String;
  Serial.begin(115200);
  Serial.println("Booting...");


  pinMode(DHTPIN, INPUT);
  pinMode(LDRPIN, INPUT);

  Serial.begin(115200);
  delay(10);

  ArduinoOTA.setPort(OTAport);

  ArduinoOTA.setHostname(SENSORNAME);

  ArduinoOTA.setPassword((const char *)OTApassword);

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Starting Node named " + String(SENSORNAME));


  //setup_wifi();
  wifiMulti.addAP("my_WIFI_SSID_01", "MyPass01");   // add Wi-Fi networks you want to connect to
  wifiMulti.addAP("my_WIFI_SSID_01", "MyPass02");

  client.setServer(mqtt_server, mqtt_port);



  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  //Serial.println("Ready");
  //Serial.print("IPess: ");
  //Serial.println(WiFi.localIP());
  Serial.println("Connecting ...");
  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    delay(1000);
    Serial.print('.');
  }
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());
  WiFi.macAddress(mac);
  ChipID = String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  ChipID.toUpperCase();

  light_state_topic = "esp8266/" + ChipID;
  Serial.println("MQTT RX Topic:" + light_state_topic);

  //ToDo: reconnect() needed?
  //reconnect();

  server.on("/", handleRoot);               // Call the 'handleRoot' function when a client requests URI "/"
  server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

  server.begin();                           // Actually start the server
  Serial.println("HTTP server started");
  
  MDNS_String = "esp8266_" + ChipID;
  char MDNS_client_ID [MDNS_String.length() + 1];
  MDNS_String.toCharArray(MDNS_client_ID, MDNS_String.length() + 1);
  if (!MDNS.begin(MDNS_client_ID)) {             // Start the mDNS responder for esp8266.local
     Serial.println("Error setting up MDNS responder!");
  }
  else{
  Serial.println("mDNS responder started");
  }
  
  //TaskScheduler stuff
  runner.init();
  Serial.println("Initialized scheduler");
  runner.addTask(t_readDHT);
  Serial.println("added t_readDHT");
  
  runner.addTask(t_checkNewValues);
  Serial.println("added t_checkNewValues");
  
  t_readDHT.enable();
  Serial.println("Enabled t_readDHT");
  t_checkNewValues.enable();
  Serial.println("Enabled t_checkNewValues");
}





/********************************** START SEND STATE*****************************************/
void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();




  root["humidity"] = (String)humValue;
  root["humidity_kalman"] = (String)humValue_Kalman;
  root["ldr"] = (String)LDR;
  root["temperature"] = (String)tempValue;



  char buffer[root.measureLength() + 1];
  char __light_state_topic[light_state_topic.length() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);

  // convert String to const char
  //https://pubsubclient.knolleary.net/api.html#loop

  light_state_topic.toCharArray(__light_state_topic, light_state_topic.length() + 1);


  if (!client.connected()) {
    //https://pubsubclient.knolleary.net/api.html#loop
    Serial.print("MQTT state:");
    Serial.println(client.state());
    Serial.print("WiFi status:");
    Serial.println(WiFi.status());
    //WiFi.printDiag(Serial);
    reconnect();
    Serial.print("WiFi status:");
    Serial.println(WiFi.status());
    // software_Reset();
  }
  else {
    Serial.println("Still connected! :)");
  }

  client.publish(__light_state_topic, buffer, true);
}





/********************************** START RECONNECT*****************************************/
void reconnect() {
  char MQTT_client_ID [light_state_topic.length() + 1];
  light_state_topic.toCharArray(MQTT_client_ID, light_state_topic.length() + 1);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_client_ID, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      //client.subscribe(light_set_topic);
      //setColor(0, 0, 0);
      //sendState();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}



/********************************** START CHECK SENSOR **********************************/
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


/********************************** START MAIN LOOP***************************************/
void loop() {
  
  

  ArduinoOTA.handle();






  // wifi
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("Wifi connected");
    }
  }

  // loop for the MQTT client
  client.loop();


  //readDHT();
  //checkNewValues();
  runner.execute();


  server.handleClient();                    // Listen for HTTP requests from clients

  //yield();
  //delay(2500);

}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  ESP.reset();
}


void handleRoot() {
  String HTML_String;

  HTML_String = "ESP: " + ChipID + "\n" + "Temperature: " + newTempValue + "\n"
                + "Humidity: " + newHumValue + "\n" + "Humidity Kalman: " + humValue_Kalman + "\n";
  server.send(200, "text/plain", HTML_String);   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

void checkNewValues(){
	Serial.println("Checking for new values");
  digitalWrite(LED_BUILTIN, HIGH); //turn the LED off
	SendMQTT = false;
	if (DHT22_read_OK) {

    // check temperature
    if (checkBoundSensor(newTempValue, tempValue, diffTEMP)) {
      tempValue = newTempValue;
      SendMQTT = true;
      digitalWrite(LED_BUILTIN, LOW);
    }

    // check humidity
    if (checkBoundSensor(newHumValue, humValue, diffHUM)) {
      humValue = newHumValue;
      SendMQTT = true;
      digitalWrite(LED_BUILTIN, LOW);
    }
    int newLDR = analogRead(LDRPIN);

    if (checkBoundSensor(newLDR, LDR, diffLDR)) {
      LDR = newLDR;
      SendMQTT = true;
    }
    // check Kalman filter humidity value
    newHumValue_Kalman = myFilter.getFilteredValue(newHumValue);
    if (checkBoundSensor(newHumValue_Kalman, humValue_Kalman, diffHUM)) {
      humValue_Kalman = newHumValue_Kalman;
      SendMQTT = true;
    }

    if (SendMQTT || (SendCounter >= 120)) {
      sendState();
      SendCounter = 0;
    }
    else {
      SendCounter++;
      Serial.println(SendCounter);
    }
    // increase the SendCounter even there is DHT22 read error
  }
  else {
    SendCounter++;
    Serial.println(SendCounter);
  } 
}

void readDHT() {
  Serial.println("Reading the temperature");

  // Get temperature event and print its value.
  sensors_event_t event;
  bool Error_Flag = true;
  // DHT22 temperature reading
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    Error_Flag = false;
  }
  else {
    newTempValue = event.temperature; //to use celsius remove the true
  }

  // DHT22 humidity reading
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    Error_Flag = false;
  }
  else {
    newHumValue = event.relative_humidity;
  }
  DHT22_read_OK = Error_Flag;
}
