//Libraries for all the various commands
#include <Adafruit_GFX.h>
#include <Adafruit_SGP30.h>
#include <ArduinoMqttClient.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <LOLIN_HP303B.h>
#include <SPI.h>
//#include <SSD1306Wire.h>
#include <WEMOS_SHT3X.h>
#include <Wire.h>
#include "arduino_secrets.h"

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

//Sensor Designation
//String whoAmI = String("office");
//String whoAmI = String("bedroom");
String whoAmI = String("kitchen");
//String whoAmI = String("livingroom");

WiFiClient wifiClient;

//Global Variables for mqttclient
MqttClient mqttClient(wifiClient);

const char broker[]   = SECRET_SERVER;
int        port       = 1883;
const char username[] = SECRET_USERNAME;
const char password[] = SECRET_PASSWORD;
const char tree[]     = "sensors/";
const char branch[]   = "/arduino";
const char topic0[]   = "/temp";
const char topic1[]   = "/humidity";
const char topic2[]   = "/pressure";
const char topic3[]   = "/tvoc";
const char topic4[]   = "/eco2";
const char topic5[]   = "/batteryvoltage";
const char topic6[]   = "/batterypercent";
const char topic7[]   = "/ipaddress";

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
const long interval = 1000;
unsigned long previousMillis = 0;

//SHT30 address assignment
SHT3X sht30(0x45);

//Global Variables for SHT30 sensor
int16_t temperatureAndHumiditySensorReturn;
float temperatureFahrenheit;
float percentHumidity;

// HP303B Opject
LOLIN_HP303B HP303BPressureSensor;

//Global Variables for HP303B sensor
int16_t pressureSensorReturn;
int32_t pressurePascal;
float pressureAtmosphere;
static int16_t oversampling = 7;
float paToATM = 101325;

//SGP30 Object
Adafruit_SGP30 sgp30;

//Global Variables for SGP30 sensor
int16_t eco2AndTVOCSensorReturn;
int16_t eco2Baseline = 400;
int16_t tvocBaseline = 0;
float eco2Measurement;
float compensatedECO2Measurement;
float tvocMeasurement;
float compensatedTVOCMeasurement;

//Global Variables for Battery Charge Level
int batteryVoltageRaw = analogRead(A0);
float batteryVoltageConverted = (float)batteryVoltageRaw * 0.00486;
float batteryChargeLevel;
float batteryVoltageMatrix[22][2] = {
  {4.2,  100},
  {4.15, 95},
  {4.11, 90},
  {4.08, 85},
  {4.02, 80},
  {3.98, 75},
  {3.95, 70},
  {3.91, 65},
  {3.87, 60},
  {3.85, 55},
  {3.84, 50},
  {3.82, 45},
  {3.80, 40},
  {3.79, 35},
  {3.77, 30},
  {3.75, 25},
  {3.73, 20},
  {3.71, 15},
  {3.69, 10},
  {3.61, 5},
  {3.27, 0},
  {0, 0}
};

//Global Variable for Miliseconds
uint32_t oneSecondInMicroseconds = 1000000;
uint32_t oneMinuteInMicroseconds = 60000000;
uint16_t oneSecondInMiliseconds = 1000;
uint16_t oneMinuteInMiliseconds = 60000;

//OLED screen pre-setup tasks
//#define wemosOLED096 GEOMETRY_128_64
//SSD1306Wire display(0x3c, D2, D1, wemosOLED096);
//#define wemosOLED066 GEOMETRY_64_48
//SSD1306Wire display(0x3c, D2, D1, wemosOLED066);
//#define wemosOLED049 GEOMETRY_64_32
//SSD1306Wire display(0x3c, D2, D1, wemosOLED049);

//#define NUMFLAKES 10
//#define XPOS 0
//#define YPOS 1
//#define DELTAY 2

void wifiSetup() { //Proceedure for initializing and connecting to a wireless network
  Serial.println("[*]Starting void wifiSetup...");
  
  Serial.println("[i]Setting ESP to be a WiFi-Client ONLY(Prevents network issues)...");
  WiFi.mode(WIFI_STA);
  Serial.println("[*]ESP set to WiFi-Client ONLY...");

  Serial.println("[i]Starting WiFi Connection...");
  WiFi.begin(ssid, pass);
  Serial.print("[i]Connecting");
  while (WiFi.status() != WL_CONNECTED) { //Print out connection status
    delay(500);
    Serial.print(".");
  }
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) { //Restart ESP if connection fails
    Serial.println("Connection Failed! Rebooting...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("");
  Serial.println("[i]WiFi Connected!");
  Serial.print("[i]IPv4 address: ");
  Serial.println(WiFi.localIP());

  Serial.println("[*]void wifiSetup has completed...");
}

void arduinoOTASetup() { //proceedure for initializing and setting up Arduino OTA update capabilities
  Serial.println("[*]Starting void arduinoOTASetup...");
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

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
    //display.clear();
    //display.setFont(ArialMT_Plain_10);
    //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    //display.drawString(display.getWidth()/2, display.getHeight()/2 - 10, "OTA Update");
    //display.display();
    Serial.println("[i]Start updating: " + type);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //display.drawProgressBar(4, 32, 120, 8, progress / (total / 100) );
    //display.display();
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[!]OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed!");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed!");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed!");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed!");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed!");
    }
  });
  
  ArduinoOTA.onEnd([]() {
    //display.clear();
    //display.setFont(ArialMT_Plain_10);
    //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    //display.drawString(display.getWidth()/2, display.getHeight()/2, "Restart");
    //display.display();
    Serial.println("\nOTA Update End");
  });
  
  ArduinoOTA.begin();
  Serial.println("[*]void arduinoOTASetup has completed...");
}

void resetDisplay() { //Clears and sets the default Display Settings
  Serial.println("[*]Starting void resetDisplay...");

  Serial.println("[i]Clearing Display Buffer...");
  //display.clear();

  Serial.println("[i]Setting Text Font & Size...");
  //display.setFont(ArialMT_Plain_10);

  Serial.println("[i]Setting Text Alignment...");
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  Serial.println("[*]void resetDisplay has completed...");
}

void mqttConnect() {
  Serial.print("[*]Starting void mqttConnect...");

  mqttClient.setId(String(whoAmI) + "Arduino");
  mqttClient.setUsernamePassword(username, password);

  Serial.print("My IP is: ");
  Serial.println(WiFi.localIP());

  Serial.println("[#]Connecting to MQTT broker " + String(broker) + " on port " + String(port) + " ...");

  if (!mqttClient.connect(broker, port)) {
    Serial.print("[!]MQTT connection failed! Error code = " + String(mqttClient.connectError()) + " ");

    switch (mqttClient.connectError()) {
      case -2:
        // MQTT_CONNECTION_REFUSED            -2
        Serial.println("MQTT_CONNECTION_REFUSED");
      break;
      case -1:
        // MQTT_CONNECTION_TIMEOUT            -1
        Serial.println("MQTT_CONNECTION_TIMEOUT");
      break;
      case 1:
        // MQTT_UNACCEPTABLE_PROTOCOL_VERSION  1
        Serial.println("MQTT_UNACCEPTABLE_PROTOCOL_VERSION");
      break;
      case 2:
        // MQTT_IDENTIFIER_REJECTED            2
        Serial.println("MQTT_IDENTIFIER_REJECTED");
      break;
      case 3:
        // MQTT_SERVER_UNAVAILABLE             3
        Serial.println("MQTT_SERVER_UNAVAILABLE");
      break;
      case 4:
        // MQTT_BAD_USER_NAME_OR_PASSWORD      4
        Serial.println("MQTT_BAD_USER_NAME_OR_PASSWORD");
      break;
      case 5:
        // MQTT_NOT_AUTHORIZED                 5
        Serial.println("MQTT_NOT_AUTHORIZED");
      break;
    }

    // add reboot
    while (1);

  } else {

    Serial.println("[i]Connected to the MQTT broker!");
    Serial.print("My IP:       ");
    Serial.println(WiFi.localIP());
    Serial.println("My clientID: " + String(whoAmI) + "Arduino");
    Serial.println("Broker IP:   " + String(broker));
    Serial.println("Broker Port: " + String(port));
  }

  Serial.println("[*]void mqttConnect has completed...");
}

void pressureSetup() {
  Serial.println("[*]Starting void pressureSetup...");
  
  Serial.println("[*]Starting HP303B Pressure Sensor...");
  HP303BPressureSensor.begin();
  Serial.println("[*]HP303B Pressure Sensor started...");

  Serial.println("[*]Starting single test of HP303B Pressure Sensor...");
  int32_t temperature;
  int32_t pressure;
  int16_t oversampling = 7;
  int16_t ret;

  //lets the HP303B perform a Single temperature measurement with the last (or standard) configuration
  //The result will be written to the paramerter temperature
  //ret = HP303BPressureSensor.measureTempOnce(temperature);
  //the commented line below does exactly the same as the one above, but you can also config the precision
  //oversampling can be a value from 0 to 7
  //the HP303B will perform 2^oversampling internal temperature measurements and combine them to one result with higher precision
  //measurements with higher precision take more time, consult datasheet for more information
  ret = HP303BPressureSensor.measureTempOnce(temperature, oversampling);

  if (ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("[!]Error! Pressure HP303B returned ret = ");
    Serial.println(ret);
  }
  else
  {
    Serial.print("[i]HP303B Temperature: ");
    Serial.print(temperature);
    Serial.println(" degrees Celsius");
  }

  //Pressure measurement behaves like temperature measurement
  //ret = HP303BPressureSensor.measurePressureOnce(pressure);
  ret = HP303BPressureSensor.measurePressureOnce(pressure, oversampling);
  if (ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("[!]Error! Pressure HP303B returned ret = ");
    Serial.println(ret);
  }
  else
  {
    Serial.print("[i]HP303B Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pascal");
  }
  delay(500);
  Serial.println("[i]Quick Test of the HP303B sensor complete...");

  Serial.println("[*]void pressureSetup has completed...");
}

void airQualitySetup() {
  Serial.println("[*]Starting void airQualitySetup...");
  
  Serial.println("[*]Starting SGP30 Sensor...");
  if (sgp30.begin())
  {
    Serial.println("[*]SGP30 Sensor started...");
    Serial.println("[i]Setting Calibration Values for SGP30 Sensor(Per LOLIN Documentation)...");
    sgp30.setIAQBaseline(eco2Baseline, tvocBaseline);
    Serial.println("[*]Calibration Values for SGP30 Sensor set...");
  }
  else {
    Serial.println("[!]ERROR: SGP Sensor could not start!");
  }
  Serial.println("[*]void airQualitySetup has completed...");
  
}

void mqttPublish() {
  Serial.println("[*]Starting void mqttPublish...");

  // Let everyone know what server I am sending data to
  Serial.println("[i] MQTT - Sending to broker at " + String(broker) + ":" + String(port));

  // Send Temperature Sensor Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic0);
  Serial.println("[#]Data: " + String(temperatureFahrenheit));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic0);
  mqttClient.print(temperatureFahrenheit);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic0);

  // Send Humidity Sensor Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic1);
  Serial.println("[#]Data: " + String(percentHumidity));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic1);
  mqttClient.print(percentHumidity);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic1);

  //Send Pressure Sensor Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic2);
  Serial.println("[#]Data: " + String(pressureAtmosphere));
  // Send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic2);
  mqttClient.print(pressureAtmosphere);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic2);

  // Send TVOC Sensor Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic3);
  Serial.println("[#]Data: " + String(compensatedTVOCMeasurement));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic3);
  mqttClient.print(compensatedTVOCMeasurement);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic3);

  // Send eCO2 Sensor Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic4);
  Serial.println("[#]Data: " + String(compensatedECO2Measurement));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic4);
  mqttClient.print(compensatedECO2Measurement);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic4);

  // Send Battery Voltage Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic5);
  Serial.println("[#]Data: " + String(batteryVoltageConverted));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic5);
  mqttClient.print(batteryVoltageConverted);
  mqttClient.endMessage();
  //
  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic5);

  // Send Battery Percent Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic6);
  Serial.println("[#]Data: " + String(batteryChargeLevel));
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic6);
  mqttClient.print(batteryChargeLevel);
  mqttClient.endMessage();

  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + topic6);

  // Send IP Address Data
  Serial.print("[i]MQTT - Sending message to topic: ");
  Serial.println(tree + String(whoAmI) + branch + topic7);
  Serial.print("[#]Data: ");
  Serial.println(WiFi.localIP());
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage(tree + String(whoAmI) + branch + topic7);
  mqttClient.print(WiFi.localIP());
  mqttClient.endMessage();

  Serial.print("[i]MQTT - Message sent to topic: ");
  Serial.println(tree + String(whoAmI) + topic6);

  Serial.println("[*]void mqttPublish has completed...");
}

void updateTemperature() {
  Serial.println("[*]Starting void updateTemperature...");

  int16_t updatedTemperatureAndHumiditySensorReturn;

  Serial.println("[?]Updating status of SHT3X Sensor...");

  Serial.println("[#]SHT3X Sensor previously returned: " + String(temperatureAndHumiditySensorReturn));
  
  updatedTemperatureAndHumiditySensorReturn = sht30.get();
  temperatureAndHumiditySensorReturn = updatedTemperatureAndHumiditySensorReturn;

  Serial.println("[#]SHT3X Sensor now returned: " + String(updatedTemperatureAndHumiditySensorReturn));

  Serial.println("[i]SHT3X Sensor Status updated...");
  
  if (temperatureAndHumiditySensorReturn == 0) {
    Serial.println("[i]SHT3X Sensor Started...");
    Serial.println("[*]Updating Temperature...");
    Serial.println("[#]Temperature was: " + String(temperatureFahrenheit) + "°F");
    temperatureFahrenheit = sht30.fTemp;
    Serial.println("[#]Temperature is now: " + String(temperatureFahrenheit) + "°F");
  }
  else if (temperatureAndHumiditySensorReturn == 2) {
    //display.println("SHT3x Busy!");
    Serial.println("[!]Error! Wire Busy! Wire.available() != 0...");
  }
  else {
    //display.println("SHT3x No Data!");
    Serial.println("[!]Error! No information recieved from the SHT3x Sensor!");
  }
  Serial.println("[*]void updateTemperature has completed...");
}

void updateHumidity() {
  Serial.println("[*]Starting void updateHumidity...");

  int16_t updatedTemperatureAndHumiditySensorReturn;

  Serial.println("[?]Updating status of SHT3X Sensor...");

  Serial.println("[#]SHT3X Sensor previously returned: " + String(temperatureAndHumiditySensorReturn));

  updatedTemperatureAndHumiditySensorReturn = sht30.get();
  temperatureAndHumiditySensorReturn = updatedTemperatureAndHumiditySensorReturn;

  Serial.println("[#]SHT3X Sensor now returned: " + String(updatedTemperatureAndHumiditySensorReturn));

  Serial.println("[i]SHT3X Sensor Status updated...");
  
  if (temperatureAndHumiditySensorReturn == 0) {
    Serial.println("[i]SHT3X Sensor Online...");
    Serial.println("[*]Updating Humidity...");
    Serial.println("[#]Humidity was: " + String(percentHumidity) + "%");
    percentHumidity = sht30.humidity;
    Serial.println("[#]Humidity is now: " + String(percentHumidity) + "%");
  }
  else if (temperatureAndHumiditySensorReturn == 2) {
    //display.println("SHT3x Busy!");
    Serial.println("[!]Error! Wire Busy! Wire.available() != 0...");
  }
  else
  {
    //display.println("SHT3x No Data!");
    Serial.println("[!]Error! No information recieved from the SHT3x Sensor!");
  }
  Serial.println("[*]void updateHumidity has completed...");
}

void updatePressure() {
  Serial.println("[*]Starting void updatePressure...");

  int32_t updatedPressurePascal;
  int16_t updatedPressureSensorReturn;
  
  Serial.println("[?]Updating status of HP303B Sensor...");

  Serial.println("[#]HP303B Sensor previously returned: " + String(pressureSensorReturn));

  updatedPressureSensorReturn = HP303BPressureSensor.measurePressureOnce(pressurePascal, oversampling);
  pressureSensorReturn = updatedPressureSensorReturn;

  Serial.println("[#]HP303B Sensor now returned: " + String(updatedPressureSensorReturn));

  Serial.println("[i]HP303B Sensor Status updated...");
  
  if (pressureSensorReturn == 0) { //HP303B__SUCCEEDED
    Serial.println("[*]Updating Pressure...");

    Serial.println("[#]Pressure was: " + String(pressureAtmosphere) + " Atmospheres");
    pressureAtmosphere = (pressurePascal / paToATM);
    Serial.println("[#]Pressure is now: " + String(pressureAtmosphere) + " Atmospheres");
  }
  else if (pressureSensorReturn == -1) { //HP303B__FAIL_UNKNOWN
    //display.println("HP303B -1!");
    Serial.println("[!]ERROR! HP303B Sensor returned with an unknown failure! pressureSensorReturn = -1, HP303B__FAIL_UNKNOWN...");
  }
  else if (pressureSensorReturn == -2) { //HP303B__FAIL_INIT_FAILED
    //display.println("HP303B -2!");
    Serial.println("[!]ERROR! HP303B Sensor returned with an initialization failure! pressureSensorReturn = -2, HP303B__FAIL_INIT_FAILED...");
  }
  else if (pressureSensorReturn == -3) { //HP303B__FAIL_TOOBUSY
    //display.println("HP303B -3!");
    Serial.println("[!]ERROR! HP303B Sensor returned with a busy response! pressureSensorReturn = -3, HP303B__FAIL_TOOBUSY...");
  }
  else if (pressureSensorReturn == -4) { //HP303B__FAIL_UNFINISHED
    //display.println("HP303B -4!");
    Serial.println("[!]ERROR! HP303B Sensor returned with an unfinisehd response! pressureSensorReturn = -4, HP303B__FAIL_UNFINISHED...");
  }
  else {
    //display.println("HP303B No Data!");
    Serial.println("[!]Error! HP303B Sensor returned an unknown response!");
  }
  Serial.println("[*]void updatePressure has completed...");
}

void updateECO2Measurement() {
  Serial.println("[*]Starting void updateECO2Measurement...");
  //CO2: 400 ppm  TVOC: 0 ppb
  Serial.println("[?]Updating status of SGP30 Sensor...");

  Serial.println("[#]SGP30 Sensor previously returned: " + String(eco2AndTVOCSensorReturn));

  eco2AndTVOCSensorReturn = sgp30.IAQmeasure();

  Serial.println("[#]SGP30 Sensor now returned: " + String(eco2AndTVOCSensorReturn));
  
  if (eco2AndTVOCSensorReturn == 1) { //Successfully talked to sensor
    Serial.println("[*]Updating eCO2...");
    Serial.println("[#]eCO2 was: " + String(compensatedECO2Measurement) + " ppm");
    eco2Measurement = sgp30.eCO2;
    compensatedECO2Measurement = eco2Measurement - eco2Baseline;
    Serial.println("[#]eCO2 is now: " + String(compensatedECO2Measurement) + " ppm");
  }
  else { //Failed to talk to sensor
    //display.println("SGP30 retunred 0!");
    Serial.println("[!]ERROR! No information recieved from the SGP30 Sensor!");
  }
}

void updateTVOCMeasurement() {
  Serial.println("[*]Starting void updateTVOCMeasurement...");
  //CO2: 400 ppm  TVOC: 0 ppb
  Serial.println("[?]Updating status of SGP30 Sensor...");

  Serial.println("[#]SGP30 Sensor previously returned: " + String(eco2AndTVOCSensorReturn));

  eco2AndTVOCSensorReturn = sgp30.IAQmeasure();

  Serial.println("[#]SGP30 Sensor now returned: " + String(eco2AndTVOCSensorReturn));
  
  if (eco2AndTVOCSensorReturn == 1) { //Successfully talked to sensor
    Serial.println("[*]Updating TVOC...");
    Serial.println("[#]TVOC was: " + String(compensatedTVOCMeasurement) + " ppb");
    tvocMeasurement = sgp30.TVOC;
    compensatedTVOCMeasurement = tvocMeasurement - tvocBaseline;
    Serial.println("[#]TVOC is now: " + String(compensatedTVOCMeasurement) + " ppb");
  }
  else { //Failed to talk to sensor
    //display.println("SGP30 returned 0!");
    Serial.println("[!]ERROR! Something went wrong with the SGP30 Sensor!");
  }
}

void updateBatteryStatus() {
  Serial.println("[*]Starting void updateBatteryStatus...");
  
  Serial.println("[?]Updating status of the Battery...");
  Serial.println("[#]Battery Voltage was: " + String(batteryVoltageConverted));
  Serial.println("[#]Battery Charge Level was: " + String(batteryChargeLevel));
  batteryVoltageRaw = analogRead(A0);
  batteryVoltageConverted = (float)batteryVoltageRaw * 0.00486;
  for(int i = 20; i>=0; i--) {
    if(batteryVoltageMatrix[i][0] >= batteryVoltageConverted) {
      batteryChargeLevel = batteryVoltageMatrix[i + 1][1];
      break;
    }
  }
  Serial.println("[#]Battery Voltage is now: " + String(batteryVoltageConverted));
  Serial.println("[#]Battery Charge Level is now: " + String(batteryChargeLevel));
}

// void displayLoop() {
//   Serial.println("[*]Starting void displayLoop...");

//   Serial.println("[*]Setting Display Properties...");
//   display.resetDisplay();
//   Serial.println("[i]Display Properties Set");

//   Serial.println("[*}Building Display Output...");
  
//   if (temperatureAndHumiditySensorReturn == 0) {
//     Serial.println("[*]Building the Temperature(*F) Display Output...");
//     display.print("Temp:");
//     Serial.println("Temperature:");
//     display.println(String(temperatureFahrenheit, 2) + "F");
//     Serial.println(String(temperatureFahrenheit, 2) + "F");
    
//     Serial.println("[*]Building the Humidity(%) Display Output...");
//     display.print("Humid:");
//     Serial.println("Relative Humidity:");
//     display.println(String(percentHumidity, 2) + "%");
//     Serial.println(String(percentHumidity, 2) + "%");
//     }
//     else {
//       display.print("Temp:");
//       display.println("[!]");
      
//       display.print("Humid:");
//       display.println("[!]");
      
//       Serial.println("[!]ERROR! No data recieved from SHT30!");
//     }

//   if (pressureSensorReturn == 0) {
//     Serial.println("[*]Building the Pressure(atm) Display Output...");
//     display.print("Press:");
//     Serial.println("Pressure:");
//     display.println(String(pressureAtmosphere, 2) + "atm");
//     Serial.println(String(pressureAtmosphere, 2) + "atm");
//     }
//     else {
//       display.print("Press:");
//       display.println("[!]");
      
//       Serial.println("[!]ERROR! No data recieved from HP303B!");
//     }

//    if (eco2AndTVOCSensorReturn == 0) {
//     Serial.println("[*]Building the eCO2 Display Output...");
//     display.print("eCO2:");
//     Serial.println("eCO2:");
//     display.println(String(eco2Measurement, 2) + "ppm");
//     Serial.println(String(eco2Measurement, 2) + "ppm");

//     Serial.println("[*]Building the TVOC Display Output...");
//     display.print("TVOC:");
//     Serial.println("TVOC:");
//     display.println(String(tvocMeasurement, 2) + "ppb");
//     Serial.println(String(tvocMeasurement, 2) + "ppb");
//     }
//     else {
//       display.print("eCO2:");
//       display.println("[!]");

//       display.print("TVOC:");
//       display.println("[!]");
      
//       Serial.println("[!]ERROR! No data recieved from SGP30!");
//    }

//   if (batteryVoltageConverted != 0) {
//     Serial.println("[*]Building the Battery Level/Voltage Display Output...");
//     display.print("Battery:");
//     Serial.println("Battery:");
//     display.println(String(batteryChargeLevel, 2) + "% " + String(batteryVoltageConverted, 2) + "V");
//     Serial.println(String(batteryChargeLevel, 2) + "% " + String(batteryVoltageConverted, 2) + "V");
//     }
//     else {
//       display.print("Battery:");
//       display.println("[!]");
//       Serial.println("[!]ERROR! Battery at 0V! It may be completely dead or disconnected!");
//     }
  
//   Serial.println("[i]Display Output Built. Displaying now...");
//   display.display();
//   Serial.println("[i]Displaying for 10 seconds...");
//   delay(10 * oneSecondInMiliseconds);
  
//   Serial.println("[i]Display Output Displayed. Clearing Display Output...");
//   display.resetDisplay();
//   Serial.println("[i]Display Output Cleared...");
  
//   Serial.println("[*]void displayLoop has completed...");
// }

void setup() {
  Serial.begin(9600);
  
  Serial.println("[!]Booting...");

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  //Serial.println("[*]Initializing Display I2C...");
  //display.init();
  //Serial.println("[i]Display I2C Initialization Complete...");
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //Serial.println("[*]Displaying Adafruit Splashscreen...");
  //display.display();
  //delay(2 * oneSecondInMiliseconds);

  // Clear the buffer.
  // Serial.println("[*]Clearing the Display Buffer...");
  // display.resetDisplay();
  // Serial.println("[i]Display Buffer cleared...");

  //Start WIFI connection
  Serial.println("[*]SETUP - Starting WIFI...");
  wifiSetup();
  Serial.println("[*]SETUP - WIFI Start Complete...");

  //Run mqttConnect
  Serial.println("[*]SETUP - Starting MQTT Connect...");
  mqttConnect();
  Serial.println("[*]SETUP - MQTT Connect Start Complete...");

  //Run Arduino OTA Setup
  Serial.println("[*]SETUP - Starting Arduino OTA...");
  arduinoOTASetup();
  Serial.println("[*]SETUP - Arduino OTA Start Complete...");

  //Run Pressure Sensor Startup
  Serial.println("[*]SETUP - Starting Pressure Sensor...");
  pressureSetup();
  Serial.println("[*]SETUP - Pressure Sensor Start Complete...");

  //Run Air Quality Sensor Startup
  Serial.println("[*]SETUP - Starting Air Quality Sensor...");
  airQualitySetup();
  Serial.println("[*]SETUP - Air Quality Sensor Start Complete...");

  //Handle Android OTA
  ArduinoOTA.handle();

  Serial.println("[i]All Start Jobs Complete[i]");
}

void loop() {

  // print IP for tracking
  Serial.print("My IP is: ");
  Serial.println(WiFi.localIP());

  // handle an OTA upgrades
  ArduinoOTA.handle();

  // poll MQTT to keep alive
  mqttClient.poll();
  // update current time for MQTT
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;
  
    updateTemperature();
    updateHumidity();
    updatePressure();
    updateECO2Measurement();
    updateTVOCMeasurement();
    updateBatteryStatus();
    //displayLoop();
    mqttPublish();
  }
}
