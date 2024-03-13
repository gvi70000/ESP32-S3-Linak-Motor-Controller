#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AM2302-Sensor.h>

//#define DEBUG

#ifdef DEBUG
  const uint16_t UART_SPEED       = 9600;
#endif
// Pins used
const uint8_t POSITION_PIN    = 1; // ADC value from violet wire
const uint8_t LED_PIN         = 4; // Used for errors
const uint8_t BTN_UP_PIN      = 10; // 1 on PCB
const uint8_t BTN_DOWN_PIN    = 11; // 2 on PCB
const uint8_t CTRL_UP_PIN     = 12; // 3 on PCB
const uint8_t CTRL_DOWN_PIN   = 13; // 4 on PCB
const uint8_t DHT22_PIN       = 14; // 5 on PCB 

// Constants
const char* SSID              = "WIF_SSID";
const char* PASSWORD          = "WIFI_PASSWORD";
const char* MQTT_SERVER       = "XXX.XXX.XXX.XXX";
const char* MQTT_USERNAME     = "MQTT_USER";
const char* MQTT_PASSWORD     = "MQTT_PASSWORD";
const char* CLIENT_ID         = "Linak1";
const char* TOPIC_TEMPERATURE = "wifi/Linak1/Temperature";
const char* TOPIC_HUMIDITY    = "wifi/Linak1/Humidity";
const char* TOPIC_POSITION    = "wifi/Linak1/Position"; // Report position
const char* TOPIC_S_POSITION  = "wifi/Linak1/setPosition"; // Set position
const char* TOPIC_BUTTON      = "wifi/Linak1/Button"; // Report Button
const char* TOPIC_S_BUTTON    = "wifi/Linak1/setButton"; //Set Button

// Handle in OpenHAB
//const char* TOPIC_ALLOW_OPEN  = "Linak1/OpeningAllowed";
// Distance 0 to 500mm
// Board 1
// R2 = 0.996k, R1 = 9.99k, @10V the output is 907mV or 3378 ADC
// @ 500mm we have 2880ADC, so the distance d = 0.1736*ADC units
const float  POSITION_COEFF     = 0.1736;
const float  POSITION_TOLERANCE = 0.5; // in mm
const float  POSITION_MAX       = 500.0; // in mm
// Handle in OpenHAB

const uint16_t WIFI_PERIOD      = 1000;
const uint16_t WIFI_BREAK       = 2000;
const uint16_t MQTT_PERIOD      = 5; // Seconds
const uint8_t DEBOUNCE_DELAY    = 50;
const uint8_t DELAY_MQTT        = 50;
const uint8_t NO_OF_READINGS    = 10;

const uint8_t BUTTON_NONE       = 0;
const uint8_t BUTTON_DOWN       = 1;
const uint8_t BUTTON_UP         = 2;
const uint8_t BUTTON_STOP       = 3;

// Global variables
AM2302::AM2302_Sensor am2302(DHT22_PIN);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
// Time incremented in interrupt
volatile uint16_t mqttTime = 0;
// Used to know the position to get to and current position
float targetPosition = 0.0;
// Current button
uint8_t crtMqttButton = BUTTON_NONE;
uint8_t crtHwButton = BUTTON_NONE;
// Used to get new position from MQTT server
uint8_t mqttPositionChanged = 0;
// Motor position 
float crtPosition = 0.0;
// Timer
hw_timer_t * timer = NULL;

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void publishToMQTT(const char* topic, const String& payload);
void mqttCallback(char* topic, byte* payload, unsigned int length);
uint8_t isInTolerace();
void processPosition() ;
void moveMotor();
void moveStop();
void buttonMoveMotor();
float getPosition();
void getButtons();
void publishSensors();
void wifiErrorHandler();
void mqttErrorHandler();

// Timer interrupt function
void IRAM_ATTR onTimer() {
  mqttTime++;
}

void setup() {
  #ifdef DEBUG
    Serial.begin(UART_SPEED);
  #endif
  analogSetAttenuation(ADC_2_5db);
  analogReadResolution(12);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_UP_PIN, INPUT);
  pinMode(BTN_DOWN_PIN, INPUT);
  pinMode(CTRL_UP_PIN, OUTPUT);
  pinMode(CTRL_DOWN_PIN, OUTPUT);
  digitalWrite(CTRL_UP_PIN, HIGH);
  digitalWrite(CTRL_DOWN_PIN, HIGH);
  // Get motor position
  crtPosition = getPosition();
  // Set up AM302 THT 22
  am2302.begin();
  // Start WiFi
  setupWiFi();
  // Start MQTT
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.subscribe(TOPIC_S_POSITION); // Subscribe to position topic
  mqttClient.subscribe(TOPIC_S_BUTTON); // Subscribe to Buttons topic
  // Publish all topics
  publishSensors();
  // Timer initialization
  // Set timer frequency to 1Mhz (default timer frequency is 80MHz)
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  #ifdef DEBUG
    Serial.println("Done.");
  #endif
}

void loop() {
  yield();
  mqttClient.loop();
  // Check if a button is pressed and move the motor
  buttonMoveMotor();
  // If we get ne position value from MQTT
  if (mqttPositionChanged) {
    mqttPositionChanged = 0;
    moveMotor();
  }

  // Publish sensor data to MQTT server
  if (mqttTime >= MQTT_PERIOD) {
    publishSensors();
    mqttTime = 0;
  }

  // Flash LED for WiFi or MQTT errors
  if (!WiFi.isConnected()) {
    wifiErrorHandler();
  }
  if (!mqttClient.connected()) {
    mqttErrorHandler();
  }
}

// Set up WiFi connection
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(CLIENT_ID);
  WiFi.useStaticBuffers(true);
  WiFi.disconnect();
  delay(WIFI_PERIOD);
  WiFi.begin(SSID, PASSWORD);
  // Wait until WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_PERIOD);
  }
}

// Reconnect to MQTT server if connection lost
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      // Subscribe to position topic once connected
      mqttClient.subscribe(TOPIC_S_POSITION);
      mqttClient.subscribe(TOPIC_S_BUTTON);
    } else {
      delay(WIFI_PERIOD);
    }
  }
}

// Publish data to MQTT server
void publishToMQTT(const char* topic, const String& payload) {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.publish(topic, payload.c_str());
  delay(DELAY_MQTT);
}

// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  if (strcmp(topic, TOPIC_S_POSITION) == 0) {
    // If the motor is not moving accept new position
    targetPosition = messageTemp.toFloat();
    mqttPositionChanged = 1;
  }
  if (strcmp(topic, TOPIC_S_BUTTON) == 0) {
    crtMqttButton = messageTemp.toInt();
  }
}

// Check if the current position is within tolerance
uint8_t isInTolerace() {
  if((targetPosition > (crtPosition - POSITION_TOLERANCE)) && (targetPosition < (crtPosition + POSITION_TOLERANCE))) {
    return 1;
  } else {
    return 0;
  }
}

void processPosition() {
  // Move motor based on position change
  if (targetPosition < crtPosition) {
    // Pull motor rod
    digitalWrite(CTRL_UP_PIN, HIGH);
    digitalWrite(CTRL_DOWN_PIN, LOW);
    crtMqttButton = BUTTON_DOWN;
  } else if (targetPosition > crtPosition) {
    // Push motor rod
    digitalWrite(CTRL_UP_PIN, LOW);
    digitalWrite(CTRL_DOWN_PIN, HIGH);
    crtMqttButton = BUTTON_UP;
  }
}

// Move the motor
void moveMotor() {
  // Get current position
  crtPosition = getPosition();
  // Do I need to move?
  if(isInTolerace()) {
    crtMqttButton = BUTTON_STOP;
    publishToMQTT(TOPIC_BUTTON, String(crtMqttButton));
    return;
  }
  processPosition();
  uint16_t prevMqttTime = mqttTime;
  while(!isInTolerace()) {
    // Get current position
    crtPosition = getPosition();
    // Check if stop button is pressed/ check MQTT
    getButtons();
    processPosition();
    yield();
    mqttClient.loop();
    // Exit if we detect stop
    if ((crtHwButton == BUTTON_STOP) || (crtMqttButton == BUTTON_STOP)) break;
    // Send position to MQTT every second
    if(prevMqttTime != mqttTime) {
      prevMqttTime = mqttTime;
      publishToMQTT(TOPIC_BUTTON, String(crtMqttButton));
      publishToMQTT(TOPIC_POSITION, String(crtPosition));
    }
  }
  moveStop();
}

// Stop the motor
void moveStop() {
  digitalWrite(CTRL_UP_PIN, HIGH);
  digitalWrite(CTRL_DOWN_PIN, HIGH);
  crtMqttButton = BUTTON_STOP;
  targetPosition = getPosition();
  publishToMQTT(TOPIC_POSITION, String(getPosition()));
  publishToMQTT(TOPIC_BUTTON, String(crtMqttButton));
}

// Moves the motor when a button is pressed
void buttonMoveMotor() {
  // Check if stop button is pressed
  getButtons();
  if ((crtHwButton == BUTTON_DOWN) || (crtMqttButton == BUTTON_DOWN)) {
    targetPosition = POSITION_TOLERANCE;
  } else if ((crtHwButton == BUTTON_UP) || (crtMqttButton == BUTTON_UP)) {
    targetPosition = POSITION_MAX;
  }
  moveMotor();
  // The stop button is handled in moveMotor()
}

// Read current position from analog pin
float getPosition() {
  float sum = 0.0;
  for(uint8_t i = 0; i < NO_OF_READINGS; i++){
    sum += (float)analogRead(POSITION_PIN);
    delay(1);
  }
  sum /= (float)NO_OF_READINGS;
  return sum * POSITION_COEFF;
}

// Read button states
void getButtons() {
  crtHwButton = (!digitalRead(BTN_UP_PIN) << 1) | !digitalRead(BTN_DOWN_PIN);
  delay(DEBOUNCE_DELAY);
}

// Read sensor data and publish to MQTT
void publishSensors() {
  publishToMQTT(TOPIC_TEMPERATURE, String(am2302.get_Temperature()));
  publishToMQTT(TOPIC_HUMIDITY, String(am2302.get_Humidity()));
  publishToMQTT(TOPIC_POSITION, String(getPosition()));
  publishToMQTT(TOPIC_BUTTON, String(crtMqttButton));
}

// Handle WiFi error
void wifiErrorHandler() {
  constexpr uint16_t wifi_delay = 500;
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(wifi_delay);
    digitalWrite(LED_PIN, LOW);
    delay(wifi_delay);
  }
  delay(WIFI_BREAK); // 2 seconds break
}

// Handle MQTT error
void mqttErrorHandler() {
  constexpr uint16_t mqtt_delay = 500;
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(mqtt_delay);
    digitalWrite(LED_PIN, LOW);
    delay(mqtt_delay);
  }
  delay(WIFI_BREAK); // 2 seconds break
}
