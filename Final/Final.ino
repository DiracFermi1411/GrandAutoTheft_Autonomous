/*
 * Final Priject
 * Hitahm
 * Raj
 * Dheeraj
 */
#include <Arduino.h>
#include "vive510.h"
#include "body3.h"
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include "html510.h"

HTML510Server h(80); // port 80 is standard for websites

// vive
#define RGBLED 2  // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 14 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
// #define STUDENTIP 102 // choose a teammembers assigned IP number
#define teamNumber 4
#define FREQ 1 // in Hz

// photosensor
#define FRIQ23 23   // beacon frequncy (23Hz)
#define FRIQ550 550   // beacon frequncy (550Hz)
#define MARGIN23   5    // window for considering a flash
#define MARGIN550   50    // window for considering a flash
#define INPUT_PHOTO      34       // digital pin for sensor 
#define LED23     26       // digital pin for (23Hz)
#define LED550     27       // digital pin for (550Hz)

volatile unsigned long frequncy = 0;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long currentInterval = 0;
volatile bool detected23Hz = false;
volatile bool detected550Hz = false;

// ultrasonic pins
const int trigPin = 13;  // GPIO4 for the Trig pin
const int echoPin = 12;  // GPIO5 for the Echo pin
const int irSensorPin = 35;

float distance = 0;

// motors
// Front Ultrasonic sensor
const int frontTrigPin = 13;  // GPIO2 for the Trig pin
const int frontEchoPin = 12;  // GPIO3 for the Echo pin

// Side Ultrasonic sensor
const int sideTrigPin = 22;  // GPIO19 for the Trig pin
const int sideEchoPin = 23;  // GPIO18 for the Echo pin

// Left Motor Pins
const int rightMotorEn = 15;
const int rightMotorAIn1 = 4;
const int rightMotorAIn2 = 5;

// Right Motor Pins
const int leftMotorEn = 21;
const int leftMotorAIn1 = 18;
const int leftMotorAIn2 = 19;

// PWM initialization
const int rightPwmChannel = 0;
const int leftPwmChannel = 1;
const int pwmResolution = 8;
const int pwmFrequency = 1500;
const int pwmMaxValue = 200;

// PD control parameters
const float kp = 15;  // Proportional gain
const float kd = 2;  // Derivative gain

float lastError = 0;  // Previous error value for derivative calculation

const char* ssid     = "HDR";
const char* password = "";
const char* ssid1     = "TP-Link_E0C8";
const char* password1 = "52665134";

#define CHANNEL 0    // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0xC8,0xF0,0x9E,0xF6,0xE0,0xC8} // receiver MAC address (last digit should be even for STA) C8:F0:9E:F6:E0:C8 / 0x60,0x55,0xF9,0x57,0x47,0x4C
uint8_t peer_mac[] = {0xC8,0xF0,0x9E,0xF6,0xE0,0xC8};
esp_now_peer_info_t peer1 = 
{
  .peer_addr = MAC_RECV, 
  .channel = CHANNEL,
  .encrypt = false,
};


Vive510 vive1(SIGNALPIN1);

WiFiUDP UDPTestServer;
IPAddress myIP(192, 168, 1, 102);     // change to your IP
IPAddress ipTarget(192, 168, 1, 255); // 255 => broadcast
IPAddress gateway(192,168,1,1);               // init gateway IP
IPAddress subnet(255,255,255,0);              // init subnet mask

float pot = 0;
float pwm;

int x;
int y;

bool b = false;

void setup() {
  Serial.begin(115200);
  pinSetup();
  ledcsetup();
  wifiSetup();
  udpSetup();
  espNowSetup();
  viveSetup();
  InterruptSetup();
  h.begin();
  h.attachHandler("/",handleRoot);             //Linking to the Web Page
  h.attachHandler("/duty?val=",handleSlider_Pot);
  h.attachHandler("/hitF ",handleHitF);
  h.attachHandler("/hitB ",handleHitB);
  h.attachHandler("/hitR ",handleHitR);
  h.attachHandler("/hitL ",handleHitL);
  h.attachHandler("/hitS ",handleHitS);
  h.attachHandler("/hitW ",handleHitW);
  h.attachHandler("/hitT ",handleHitT);
  h.attachHandler("/hitTT ",handleHitTT);
  h.attachHandler("/hitP ",handleHitP);
  h.attachHandler("/hitC ",handleHitC);
}
                                 
void loop() {  
  h.serve();
  x, y = viveLoop();
  espNowLoc(x,y);
  ledFrequency();
  delay(10);
}

void pinSetup(){
  //set up pin mode
  // photosensor
  pinMode(INPUT_PHOTO, INPUT);
  pinMode(LED23, OUTPUT);
  pinMode(LED550, OUTPUT);
  pinMode(trigPin, OUTPUT);     // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);      // Sets the echoPin as an Input

  // IR sensor
  pinMode(irSensorPin, INPUT);


  // Front sensor setup
  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  // Side sensor setup
  pinMode(sideTrigPin, OUTPUT);
  pinMode(sideEchoPin, INPUT);
  // motors
  pinMode(rightMotorAIn1, OUTPUT);
  pinMode(rightMotorAIn2, OUTPUT);
  pinMode(rightMotorEn, OUTPUT);
  pinMode(leftMotorAIn1, OUTPUT);
  pinMode(leftMotorAIn2, OUTPUT);
  pinMode(leftMotorEn, OUTPUT);
}

void ledcsetup(){
  ledcSetup(rightPwmChannel, pwmFrequency, pwmResolution);
  ledcSetup(leftPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(rightMotorEn, rightPwmChannel);
  ledcAttachPin(leftMotorEn, leftPwmChannel);

  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);
}

void InterruptSetup(){
  // Attach interrupt for photo sensor
  attachInterrupt(digitalPinToInterrupt(INPUT_PHOTO), detectFrequency, RISING);
}

void espNowSetup(){
  WiFi.mode(WIFI_STA);  
  Serial.print("Sending MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed");
    ESP.restart();
  }
  
  esp_now_register_send_cb(OnDataSent); //optional if you want ack interrupt
    
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed");     // ERROR  should not happen
  }
}

// optional callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  else Serial.println("Fail ");
}

void espNowCom(){
  // esp_now_mod_peer(&peer1);
  esp_now_del_peer( peer_mac );
  esp_now_add_peer(&peer1);

  unsigned long currentTime = millis();
  currentInterval = currentTime - lastInterruptTime;

  if (currentInterval > 1000){
    static int count;
    uint8_t message[200]; // Max ESPnow packet is 250 byte data

    // put some message together to send
    sprintf((char *) message, "sender team 4: %d ", count++);
    
    if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
      Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
    else Serial.println("Send failed");
  }
  lastInterruptTime = currentTime;
  // delay(1000); // ESPNow max sending rate (with default speeds) is about 50Hz
}

void espNowLoc(int x, int y){
  esp_now_mod_peer(&peer1);
  // esp_now_del_peer( peer_mac );
  // esp_now_add_peer(&peer1);

  unsigned long currentTime = millis();
  currentInterval = currentTime - lastInterruptTime;

  if (currentInterval > 1000){
    uint8_t message[250]; // Max ESPnow packet is 250 byte data

    // put some message together to send
    sprintf((char *) message, "04:", x, y);
    
    if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
      Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
    else Serial.println("Send failed");
  }
  lastInterruptTime = currentTime;
  // delay(1000); // ESPNow max sending rate (with default speeds) is about 50Hz
}

void wifiSetup(){
  int i=0;
  // WiFi.mode(WIFI_STA);
  // WIFI setup

  // WiFi.softAPConfig(myIP, gateway, subnet);     // set IP address of ESP
  // Serial.print("Connecting to: "); 
  // Serial.println(ssid);  // debug statement
  // WiFi.softAP(ssid,password); // configures the ESP in AP mode with network name
  // delay(100);
  // Serial.println();
  // Serial.println("WiFi connected!");  
  // Serial.print("Use this URL to connect: http://");
  // Serial.print(myIP); 
  // Serial.println("/");
  
  delay(1000);
  WiFi.config(myIP, gateway, subnet);
  WiFi.begin(ssid1, password1);
  Serial.printf("team  #%d ",teamNumber); 
  Serial.println("");
  Serial.print("Connecting to ");  
  Serial.println(ssid1);
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);   
    Serial.print(".");
  }
  Serial.println("");
  if (i<19) {
    Serial.println("WiFi connected as "); Serial.println(WiFi.localIP());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
}

void viveSetup(){
  vive1.begin();
  Serial.println("Vive trackers started");
}

int viveLoop(){
  static long int ms = millis();
  static uint16_t x,y;
  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
    UdpSend(x,y);
  }
  
  if (vive1.status() == VIVE_RECEIVING) {
    x = vive1.xCoord();
    y = vive1.yCoord();
    neopixelWrite(RGBLED,0,x/200,y/200);  // blue to greenish
  }
  else {
    x=0;
    y=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  delay(10);
  return x,y;
}

void ledFrequency(){
  if (detected23Hz) {
    // Serial.println("Detected 23Hz");
    digitalWrite(LED23, HIGH);
    digitalWrite(LED550, LOW);
    detected23Hz = false;
  }
  else if (detected550Hz) {
    // Serial.println("Detected 550Hz");
    digitalWrite(LED550, HIGH);
    digitalWrite(LED23, LOW);
    detected550Hz = false;
  }
  else{
    digitalWrite(LED550, LOW);
    digitalWrite(LED23, LOW);
  }
  delay(50);
}

void detectFrequency() {
  unsigned long currentTime = micros();
  currentInterval = currentTime - lastInterruptTime;
  frequncy = 1000000/currentInterval;
  // Serial.println(frequncy);
  // Check if the detected frequency is close to 23Hz (within a margin)
  if (frequncy > (FRIQ23-MARGIN23) && frequncy < (FRIQ23+MARGIN23)) {
    detected23Hz = true;
    // Serial.println(detected23Hz);
  }
  // Check if the detected frequency is close to 550Hz (within a margin)
  if (frequncy > (FRIQ550-MARGIN550) && frequncy < (FRIQ550+MARGIN550)) {
    detected550Hz = true;
    // Serial.println(detected550Hz);
  }
  lastInterruptTime = currentTime;
}

void udpSetup(){
  UDPTestServer.begin(UDPPORT);
}

void UdpSend(int x, int y)
{
  char udpBuffer[13];
  sprintf(udpBuffer, "%1d:%4d,%4d",teamNumber,x,y);   
  // Ensure null termination
  // udpBuffer[999]=0;
  // udpBuffer[sizeof(udpBuffer) - 1] = '\0';  
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  unsigned long currentTime = millis();
  currentInterval = currentTime - lastInterruptTime;
  if (currentInterval > 1000){
    Serial.println(udpBuffer);
  }
  lastInterruptTime = currentTime;
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}


void leftTurn(){
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, HIGH);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, HIGH);
  ledcWrite(rightPwmChannel, pwmMaxValue);
  ledcWrite(leftPwmChannel, pwmMaxValue);
  delay(200);
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, HIGH);
  delay(1200);
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  return;
  
}

void backward(){
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, HIGH);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, HIGH);
  ledcWrite(rightPwmChannel, pwmMaxValue);
  ledcWrite(leftPwmChannel, pwmMaxValue);
  delay(500);
}


void wallFollowingControl() {

  int t = 0;

  while (t <= 5000){

  
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  
  
  const int thresholdDistance = 15;  // Threshold distance for wall following in centimeters
  const int frontThresholdDistance = 15;
  float currentDistanceSide = getDistance(sideTrigPin, sideEchoPin);
  Serial.println(currentDistanceSide);
  float currentDistanceFront = getDistance(frontTrigPin, frontEchoPin);
  // If an obstacle is detected in front, stop the robot
  if (currentDistanceFront < frontThresholdDistance) {
    ledcWrite(rightPwmChannel, 0);
    ledcWrite(leftPwmChannel, 0);
    Serial.println("Obstacle detected in front. Stopping.");
    leftTurn();
    backward();
    backward();
    //return;  // Exit the function to stop further processing
  }

  // Calculate the error (deviation from the threshold distance)
  float error = thresholdDistance - currentDistanceSide;

  // Calculate the derivative of the error
  float derivative = error - lastError;

  // Update the last error for the next iteration
  lastError = error;

  // Calculate the motor speed adjustment using PD control
  int speedAdjustment = kp * error + kd * derivative;

  // Adjust motor speeds
  int rightSpeed = pwmMaxValue + speedAdjustment;
  int leftSpeed = pwmMaxValue - speedAdjustment;

  // Apply constraints to ensure the motor speeds stay within the valid range
  rightSpeed = constrain(rightSpeed, 0, pwmMaxValue);
  leftSpeed = constrain(leftSpeed, 0, pwmMaxValue);

  // Update motor speeds
  ledcWrite(rightPwmChannel, rightSpeed);
  ledcWrite(leftPwmChannel, leftSpeed);
  delay(10);
  t = t + 10;

  }

  // Print debug information
  // Serial.print("Distance Side: ");
  // Serial.print(currentDistanceSide);
  // Serial.print(" cm, Distance Front: ");
  // Serial.print(currentDistanceFront);
  // Serial.print(" cm, Error: ");
  // Serial.print(error);
  // Serial.print(", Derivative: ");
  // Serial.print(derivative);
  // Serial.print(", Right Speed: ");
  // Serial.print(rightSpeed);
  // Serial.print(", Left Speed: ");
  // Serial.println(leftSpeed);

  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);
}



void policeControl() {

  int t = 0;
  int k = 0;

  while (t < 2000){
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  
  
  const int thresholdDistance = 15;  // Threshold distance for wall following in centimeters
  const int frontThresholdDistance = 15;
  float currentDistanceSide = getDistance(sideTrigPin, sideEchoPin);
  Serial.println(currentDistanceSide);
  float currentDistanceFront = getDistance(frontTrigPin, frontEchoPin);
  // If an obstacle is detected in front, stop the robot
  if (currentDistanceFront < frontThresholdDistance && k != 1) {
    k = 1;
    ledcWrite(rightPwmChannel, 0);
    ledcWrite(leftPwmChannel, 0);
    Serial.println("Obstacle detected in front. Stopping.");
    backward();
    leftTurn();
    backward();
    backward();
    forward(5000);
    backward();
    forward(5000);
    backward();
    //return;  // Exit the function to stop further processing
  }

  // Calculate the error (deviation from the threshold distance)
  float error = thresholdDistance - currentDistanceSide;

  // Calculate the derivative of the error
  float derivative = error - lastError;

  // Update the last error for the next iteration
  lastError = error;

  // Calculate the motor speed adjustment using PD control
  int speedAdjustment = kp * error + kd * derivative;

  // Adjust motor speeds
  int rightSpeed = pwmMaxValue + speedAdjustment;
  int leftSpeed = pwmMaxValue - speedAdjustment;

  // Apply constraints to ensure the motor speeds stay within the valid range
  rightSpeed = constrain(rightSpeed, 0, pwmMaxValue);
  leftSpeed = constrain(leftSpeed, 0, pwmMaxValue);

  // Update motor speeds
  ledcWrite(rightPwmChannel, rightSpeed);
  ledcWrite(leftPwmChannel, leftSpeed);
  delay(10);
  t = t + 10;
  }
  
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);
}

void Trophy550Hz(){


  int t = 0;
  int sensorValue = digitalRead(irSensorPin);
  Serial.println(sensorValue);

  while (sensorValue != 0 || t > 10000) {
    sensorValue = digitalRead(irSensorPin);

  if (detected550Hz) {
    // Serial.println("Detected 550Hz");
    digitalWrite(LED550, HIGH);
    digitalWrite(LED23, LOW);
    forward(750);
    detected550Hz = false;
  }
    
  else{
    digitalWrite(LED550, LOW);
    digitalWrite(LED23, LOW);
    turn();
  }
  delay(100);
  t = t + 100;
  

  }
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);

}


void Trophy23Hz(){


  int t = 0;
  int sensorValue = digitalRead(irSensorPin);
  Serial.println(sensorValue);

  while (sensorValue != 0 || t > 10000) {
    sensorValue = digitalRead(irSensorPin);

  if (detected23Hz) {
    // Serial.println("Detected 550Hz");
    digitalWrite(LED550, LOW);
    digitalWrite(LED23, HIGH);
    forward(750);
    detected23Hz = false;
  }
    
  else{
    digitalWrite(LED550, LOW);
    digitalWrite(LED23, LOW);
    turn();
  }
  delay(100);
  
  t = t + 100;
  }
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);

}


void forward(int time_in_ms){
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  ledcWrite(rightPwmChannel, 200);
  ledcWrite(leftPwmChannel, 200);
  delay(time_in_ms);
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);
  return;
}

void turn(){
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, HIGH);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  ledcWrite(rightPwmChannel, 170);
  ledcWrite(leftPwmChannel, 170);
  return;
  }

float calcDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  delay(100);

  return distance;
  
}

void handleRoot(){           
  h.sendhtml(body);           //Calling the body.h
}

void handleSlider_Pot(){  // speed control
  pot = h.getVal();
  pot = h.getVal();
  Serial.print("Pot= ");
  Serial.println(pot);
  pwm = map(pot, 0, 100, 0, 255);
  ledcWrite(rightPwmChannel, pwm);
  ledcWrite(leftPwmChannel, pwm);
  delay(10);
}

void handleHitF(){  // forward direction
  Serial.println("Going Forward");
  ledcWrite(rightPwmChannel, 250);
  ledcWrite(leftPwmChannel, 250);
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  delay(10);
}


void handleHitB(){  // backward direction
  Serial.println("Going Backward");
  ledcWrite(rightPwmChannel, 250);
  ledcWrite(leftPwmChannel, 250);
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, HIGH);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, HIGH);
  delay(10);
}

void handleHitR(){  // right direction
  Serial.println("Going Right");
  ledcWrite(rightPwmChannel, 250);
  ledcWrite(leftPwmChannel, 250);
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, HIGH);
  digitalWrite(leftMotorAIn2, LOW);
  delay(10);
}

void handleHitL(){  // left direction
  Serial.println("Going Left");
  ledcWrite(rightPwmChannel, 250);
  ledcWrite(leftPwmChannel, 250);
  digitalWrite(rightMotorAIn1, HIGH);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, LOW);
  delay(10);
}

void handleHitS(){  // stop
  Serial.println("Stop");
  ledcWrite(rightPwmChannel, 0);
  ledcWrite(leftPwmChannel, 0);
  digitalWrite(rightMotorAIn1, LOW);
  digitalWrite(rightMotorAIn2, LOW);
  digitalWrite(leftMotorAIn1, LOW);
  digitalWrite(leftMotorAIn2, LOW);
  espNowCom();
}

void handleHitW(){  // wall following
  Serial.println("Wall following");
  wallFollowingControl();
  delay(10);
}

void handleHitT(){  // Trophy following
  Serial.println("550Hz Trophy folloing");
  Trophy550Hz();
  delay(10);
}

void handleHitTT(){  // Trophy following
  Serial.println("23Hz Trophy folloing");
  Trophy23Hz();
  delay(10);
}

void handleHitP(){  // Police following
  Serial.println("Police folloing");
  policeControl();
}

void handleHitC(){  // Police following
  x, y = viveLoop();
  String s = String(x) + "," + String(y);
  h.sendplain(s);
  Serial.printf("received X,Y:=%d,%d\n",x,y);
}
