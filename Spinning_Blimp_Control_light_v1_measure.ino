#include <crazyflieComplementary.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>

#define LightSensor1 A2
#define LightSensor2 A3

// Wi-Fi access details
const char * ssid = "AIRLab-BigLab";
const char * password = "Airlabrocks2022";

// c.c. Edward
SensFusion sensorSuite;

//
AsyncUDP udp;

// Define light sensors
float light1 = 0;
float light2 = 0;

// Define Sensor Variables
float roll, pitch, yaw;
float rollrate, pitchrate, yawrate;
float estimatedZ, velocityZ, groundZ;

void setup() {
  Serial.begin(9600); 
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

  // Access sensor suite c.c. Edward
  sensorSuite.initSensors();
  sensorSuite.updateKp(5, -1, 0.3); // 20, -1, 0
  groundZ = sensorSuite.returnZ();

  // magnetometer calibration
  float transformationMatrix[3][3] = {
    {1.0f, 9.693f, 0.6187f},
    {9.6624f, -0.6822f, 0.3864f},
    {-0.4155f, 0.6628f, -10.7386f}
  };
  float offsets[3] = {11.98f, 7.01f, 21.77f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  getSensorValues();
}

// Main loop 
void loop() {

  // Calls Edward's sensor suite to receive sensors values
  sensorSuite.sensfusionLoop(false, 5);
  getSensorValues();
  
  // Reads photoresistor analog values
  light1 = analogRead(LightSensor1);
  light2 = analogRead(LightSensor2);

  // Calls UDP Broadcast func
  send_udp_feedback();
}

// Edward sensor func
void getSensorValues(){ 
  //all in radians or meters or meters per second
  // roll = sensorSuite.getRoll();
  // pitch = -1*sensorSuite.getPitch();
  yaw = sensorSuite.getYaw();
  // rollrate = sensorSuite.getRollRate();
  // pitchrate = sensorSuite.getPitchRate();
  yawrate = sensorSuite.getYawRate();
  estimatedZ = sensorSuite.returnZ();
  velocityZ = sensorSuite.returnVZ(); 
}

// This function converts floats to strings and concatenates the strings for broadcastTo func
void send_udp_feedback(){
  String lightval = String("");
  String lightval2 = String("");
  String comma = String(", ");
  String comb = String("");
  String diff = String("");
  String height = String("");
  lightval = String(float(light1));
  lightval2 = String(float(light2));
  height = String(float(estimatedZ));
  diff = String(light1 - light2);
  String semicol = String(";");
  comb = lightval + comma + lightval2 + comma + diff + comma + height + semicol;

  // UDP Broadcast (string,port)
  udp.broadcastTo(comb.c_str(),8003);
// udp.broadcastTo("I Work",8001);
}
