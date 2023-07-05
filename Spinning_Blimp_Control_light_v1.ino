#include <crazyflieComplementary.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>

#define THRUST1 D0
#define THRUST2 D1
#define LightSensor1 A2
#define LightSensor2 A3

// min and max high signal of thruster PWMs
int minUs = 1100;
int maxUs = 1900;

// Wi-Fi access details
const char * ssid = "AIRLab-BigLab";
const char * password = "Airlabrocks2022";

// using servo lib to control brushless motors
Servo thrust1;
Servo thrust2; 

// c.c. Edward
SensFusion sensorSuite;

AsyncUDP udp;
// AsyncUDP udp2;
float joy_data[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
volatile bool joy_ready = false;
volatile unsigned long time_now, time_loop; 

// P.I.D. Values
float roll, pitch, yaw;
float rollrate, pitchrate, yawrate;
float estimatedZ, velocityZ, groundZ;
float abz = 0.0;
float kpz = 0.01*10.0; // N/meter
float kiz = 0.02;
float kdz = 0.2*0.7;
float kpx = 0.035;
float kdx = 0.2;
float kptz = 0.3;
float kdtz = -0.025;
float kptx = 0.01;
float kdtx = 0.01;
float lx = 0.25;
float m1 = 0.0;
float m2 = 0.0;
float heading = 0.0;
float massthrust = 0.15;
long dt, last_time, time_nw;
int m1us = 0;
int m2us = 0;

// lightSensor 
bool enableLight = false;
float kpLight = 8.0;
float light1 = 0;
float light2 = 0;

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

  // Allocate timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // Standard 50hz PWM for thrusters
  thrust1.setPeriodHertz(50);
  thrust2.setPeriodHertz(50);

  thrust1.attach(THRUST1, minUs, maxUs);
  thrust2.attach(THRUST2, minUs, maxUs);
  delay(100);

  // ESC arm
  escarm();
  
  // Access sensor suite c.c. Edward
  sensorSuite.initSensors();
  sensorSuite.updateKp(5, -1, 0.3); // 20, -1, 0
  groundZ = sensorSuite.returnZ();
  // sensorSuite.recordData();

  // magnetometer calibration
  float transformationMatrix[3][3] = {
    {1.0f, 9.693f, 0.6187f},
    {9.6624f, -0.6822f, 0.3864f},
    {-0.4155f, 0.6628f, -10.7386f}
  };
  float offsets[3] = {11.98f, 7.01f, 21.77f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  getSensorValues();

  
  time_now = millis();
  // time_loop = millis();
  if(udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());

    // setup callback functions of the udp
    udp.onPacket([](AsyncUDPPacket packet) {
      joy_ready = false;
      time_now = millis();
      unsigned char *buffer = packet.data();
      unpack_joystick(joy_data, buffer);
      joy_ready = true;
      //reply to the server
      // packet.printf("Got %u bytes of data", packet.length());
    });
  }

    //192.168.0.107
    // udp.connect(IPAddress(192,168,0,107),5006);

  // Yaw heading setup
  heading = sensorSuite.getYaw(); 
}

void loop() {
    //gyro, acc, mag, euler, z
  float cfx, cfy, cfz, ctx, cty, ctz;

  // read lightsensor value (normalized to 0-1), smaller value means brighter
  // light1 = analogRead(LightSensor1)*0.000244141;
  // light2 = analogRead(LightSensor2)*0.000244141;
  light1 = analogRead(LightSensor1);
  light2 = analogRead(LightSensor2);

  // Runs the sensor fusion loop
  sensorSuite.sensfusionLoop(false, 5);

  if (joy_data[7] != 1){

    // Serial.println("Initialization") //debug;
    thrust1.writeMicroseconds(minUs);
    thrust2.writeMicroseconds(minUs);

    // Debug joystick input
    // thrust1.writeMicroseconds((int) (1000 + joy_data[0]));
    // thrust2.writeMicroseconds((int) (1000 + joy_data[0]));

  } else if (joy_ready && millis() - time_now <= 1000){ //&& millis() - time_loop > 50) {kdz
    // Call sensor suite to update 10-DOF values
    getSensorValues();
    // Send back magnetometer values
    getControllerInputs(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, &abz);
    addFeedback(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, abz);
    controlOutputs(cfx, cfy, cfz, ctx, cty, ctz);

    if (enableLight == 1){
      // cfx = cfx + kpLight*(light1 - light2);
      // cfy = cfy + kpLight*(light1 - light2);
      // Serial.println(kpLight*(light1 - light2));
      m1us = (minUs + (maxUs - minUs)*m1*3.33*(0.75)) - kpLight*(light1 - light2);
      m2us = (minUs + (maxUs - minUs)*m2*3.33*(0.75)) + kpLight*(light1 - light2);
    } else {
      m1us = (minUs + (maxUs - minUs)*m1*3.33*(0.75));
      m2us = (minUs + (maxUs - minUs)*m2*3.33*(0.75));
    }

    //UDP Broadcast yaw value
    send_udp_feedback();

    // Write motor thrust values to the pins
    m1us = clamp(m1us, 1200, 1800);
    m2us = clamp(m2us, 1200, 1800);
    // m1us = clamp(m1us, 1125, 1550);
    // m2us = clamp(m2us, 1125, 1550);
    thrust1.write((int) m1us);
    thrust2.write((int) m2us);

    // Else statement for if the Wi-Fi signal is lost
  } else {
    thrust1.writeMicroseconds((int) minUs);
    thrust2.writeMicroseconds((int) minUs);
    
  }

}

//Enter arming sequence for ESC
void escarm(){
  // ESC arming sequence for BLHeli S
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);

  // Sweep up
  for(int i=1100; i<1500; i++) {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Sweep down
  for(int i=1500; i<1100; i--) {
    thrust1.writeMicroseconds(i);
    delay(5);
    thrust2.writeMicroseconds(i);
    delay(5);
  }
  // Back to minimum value
  thrust1.writeMicroseconds(1000);
  delay(10);
  thrust2.writeMicroseconds(1000);
  delay(10);

}


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

float valtz = 0;
void getControllerInputs(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float *abz){
  if (false) {
    *fx = 0;//joy_data[0];
    *fy = 0;//joy_data[1];
    *fz = 0;//joy_data[2];
    *tx = 0;//joy_data[3];
    *ty = 0;//joy_data[4];
    *tz = 0;//joy_data[5];
    *abz = 0;//joy_data[6];
    if (valtz > 1){
      valtz = -1;
    } else {
      valtz += .01;
    }
  } else{
  *fx = joy_data[0];
  *fy = joy_data[1];
  *fz = joy_data[2];
  *tx = joy_data[3];
  *ty = joy_data[4];
  *tz = joy_data[5];
  *abz = joy_data[6];
  enableLight = joy_data[3];
  }
}
void addFeedback(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float abz){
    // *fz = 0;
    float err = estimatedZ-groundZ;
    delta_time();
    float int_err =+ dt * err;
    // *fz = (*fz  - (estimatedZ-groundZ))*kpz - (int_err * kiz) - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    *fz = (*fz  - (estimatedZ-groundZ))*kpz - (velocityZ)*kdz + abz;//*fz = *fz + abz;//

  // Serial.println(int_err*kiz);
}

long delta_time(){
  time_nw = millis();
  dt = (time_nw - last_time);
  last_time = time_nw;
  return dt;
}

float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}

void controlOutputs(float ifx, float ify, float ifz, float itx, float ity, float itz) {
  // yaw = itx;

  // Heading added to the control loop
  heading = heading + radians(ity * 180);
  
  // Serial.println(yaw);

  // Convert joystick input to theta and magnitude for cyclic input
  float joytheta = atan2(ify,-ifx) + PI;  
  float joymag  = sqrt(pow(ifx,2) + pow(ify,2));

  // Cyclic pitch input
  // float lhs = joymag*(coscos  - sincos);
  // float rhs = joymag*(-sincos - coscos);
  float pitch = joymag * sin(joytheta) * cos(yaw);
  float roll =  joymag * cos(joytheta) * sin(yaw);
  // float pitch = joymag * sin(joytheta) * cos(yaw + heading);
  // float roll =  joymag * cos(joytheta) * sin(yaw + heading);

  // TODO: bring back x-y feedback
  // Mass Thrust it a proportional debug gain
  float mt = massthrust;

  float f1 = ifz - (mt * (pitch + roll)); // LHS motor
  float f2 = ifz + (mt * (pitch + roll)); // RHS motor


  // Clamp to ensure motor doesn't stop spinning at minimum
  // and motor doesn't draw too much current
  // m1 = clamp(f1, 0, 0.25);
  // m2 = clamp(f2, 0, 0.25);
  m1 = clamp(f1, 0.015, 0.18);
  m2 = clamp(f2, 0.015, 0.18);

}

// Having questions? Ask Jiawei!
void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 8;
  int num_bytes = 4;
  int i, j;

  for(i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for(j = 0; j < num_bytes; j++) {
      temp[j] = buffer[num_bytes*i + j];
    }
    dat[i] = *((float*) temp);

  }
;
}

void send_udp_feedback(){
String yaw_feedback = String("");
yaw_feedback = String(float(light1-light2));
udp.broadcastTo(yaw_feedback.c_str(),8001);
// udp.broadcastTo("wow",8001);

}
