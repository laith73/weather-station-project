/*
  Weather Station Project Firmware
  Author: Laith Al Sairafi
  Date: Fall 2025
*/

#include <Wire.h>
#include "SparkFunBME280.h"
#include <SI1145_WE.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// Constants
#define SDA_PIN 4
#define SCL_PIN 5       
#define BME280_ADDR 0x76    // I2C address of BME280
#define SI1145_ADDR 0x60    // I2C address of Si1145
#define TESEO_ADDR 0x3A     // I2C address of Teseo-LIV3R
#define READ_SIZE 32        // number of bytes to request per cycle
#define READ_INTERVAL 5000  // milliseconds

// Sensor objects
BME280 bme280;
SI1145_WE si1145 = SI1145_WE();
TinyGPSPlus gps;

// Wi-Fi credentials - user needs to update this
const char* ssid  = "Coolest Project";
const char* password = "123456789";

// Web server
AsyncWebServer server(80);

// Sensor variables
float lat = 0.0;
float lng = 0.0;
float alt = 0.0;
int satNum = 0;
float hdop = 0.0;
float hum = 0.0;
float temp = 0.0;
float pres = 0.0;
byte failureCode = 0;
unsigned int amb_als = 0;
unsigned int amb_ir = 0;
unsigned int proximity = 0;
float uv = 0.0;

// Time variables 
unsigned long lastReadTime  = 0;

/*
  Funcation to update the GPS data (lat, lng, alt, satNum, hdop)
  using the Teseo-LIV3R module
  @returns true if data is updated, false otherwise. 
*/ 
bool updateGPS(unsigned long timeoutMs = 500) {
  unsigned long start = millis();
  bool updated = false;

  while (millis() - start < timeoutMs) {
    int returned = Wire.requestFrom((int)TESEO_ADDR, READ_SIZE);

    while (Wire.available()) {
      char c = Wire.read();
      gps.encode(c);

      if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        alt = gps.altitude.meters();
        satNum = gps.satellites.value();
        hdop = gps.hdop.hdop();
        updated = true;
      }
    }

    delay(20);
  }

  return updated;
}

/*
  Funcation to update the environmental data data (hum, pres, temp)
  using the BME280 module
*/ 
void updateBME280(){
  hum = bme280.readFloatHumidity(); // percent
  pres = bme280.readFloatPressure() / 100; // hPa
  temp = bme280.readTempC(); // C
}

/*
  Funcation to update the environmental data data (amb_als, amb_ir, proximity, uv)
  using the Si1145 module
*/ 
void updateSi1145(){
  
  amb_als = si1145.getAlsVisData();
  amb_ir = si1145.getAlsIrData();
  proximity = si1145.getPsData();
  uv = si1145.getUvIndex();

  failureCode = si1145.getFailureMode();  // reads the response register
  if((failureCode&128)){  // if bit 7 is set in response register, there is a failure
    handleFailure(failureCode);
  }
}

void handleFailure(byte code){
  String msg = "";
  switch(code){
    case SI1145_RESP_INVALID_SETTING:
      msg = "Invalid Setting";
      break;
    case SI1145_RESP_PS1_ADC_OVERFLOW:
      msg = "PS ADC Overflow";
      break;
    case SI1145_RESP_ALS_VIS_ADC_OVERFLOW:
      msg = "ALS VIS ADC Overflow";
      break;
    case SI1145_RESP_ALS_IR_ADC_OVERFLOW:
      msg = "ALS IR Overflow";
      break;
    case SI1145_RESP_AUX_ADC_OVERFLOW:
      msg = "AUX ADC Overflow";
      break;
    default:
      msg = "Unknown Failure";
      break;
  }
  
  Serial.println(msg); 
  si1145.clearFailure();
}

void setup() {
  
  // Serial Setup
  Serial.begin(115200);
  delay(1000);
  
  // I2C setup
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);

  // BME280 sensor setup
  bme280.setI2CAddress(BME280_ADDR);
  while(!bme280.beginI2C()){
    Serial.println("The BME280 sensor did not respond.");
    delay(1000);
  }

  // Si1145 sensor setup
  si1145.init();
  
  // si1145.enableHighSignalVisRange(); // Gain divided by 14.5
  // si1145.enableHighSignalIrRange(); // Gain divided by 14.5
  // si1145.enableHighSignalPsRange(); // Gain divided by 14.5

  si1145.disableHighSignalVisRange(); // Gain NOT divided by 14.5
  si1145.disableHighSignalIrRange(); // Gain NOT divided by 14.5
  si1145.disableHighSignalPsRange(); // Gain NOT divided by 14.5

  si1145.enableMeasurements(PSALSUV_TYPE, AUTO);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
      Serial.println("LittleFS Mount Failed!");
  }
  
  // Connect to Wi-Fi
  // WiFi.begin(ssid, password);  // Connect to Wi-Fi
  
  // Serial.print("Connecting");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // Serial.println();
  // Serial.println("Connected to Wi-Fi!");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());


  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // --- Web Server Endpoints ---

  // Handler for serving the html file 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/index.html")) {
      request->send(LittleFS, "/index.html", "text/html");
    } else {
      request->send(500, "text/plain", "index.html not found");
    }
  });

  // Handler for real-time JSON data
  server.on("/data", [](AsyncWebServerRequest *request) {
    StaticJsonDocument<256> doc;

    doc["temperature"] = temp;
    doc["humidity"] = hum;
    doc["pressure"] = pres;
    doc["UV-Index"] = uv;
    doc["Ambient Light"] = amb_als;
    doc["Infrared Light"] = amb_ir;
    doc["proximity"] = proximity;
    doc["latitude"] = lat;
    doc["longitude"] = lng;
    doc["altitude"] = alt;
    doc["satellites"] = satNum;
    doc["HDOP"] = hdop;

    // Serialize to string
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });

  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    
    bool gpsOK = updateGPS();
        
    if (!gpsOK) {
      Serial.println("GPS timeout — no new fix");
    } else {
      Serial.println("GPS updated");
    }

    updateBME280();
    Serial.println("BME280 updated");
    updateSi1145();
    Serial.println("Si1145 updated");

    Serial.print("Temperature: ");
    Serial.println(temp);  
    Serial.print("Humidity: ");
    Serial.println(hum);  
    Serial.print("Pressure: ");
    Serial.println(pres);
    Serial.print("Ambient Light: ");
    Serial.println(amb_als);
    Serial.print("Infrared Light: ");
    Serial.println(amb_ir);
    Serial.print("UV-Index: ");
    Serial.println(uv);
    Serial.print("Proximity: ");
    Serial.println(proximity);
    Serial.print("Lat: "); 
    Serial.println(lat, 6);
    Serial.print("Lon: "); 
    Serial.println(lng, 6);
    Serial.print("Alt: "); 
    Serial.println(alt, 2);
    Serial.print("Satellites: ");
    Serial.println(satNum);
    Serial.print("HDOP: "); 
    Serial.println(hdop);
    Serial.println("------------------------------------");
    
    lastReadTime = currentTime;
  }
}