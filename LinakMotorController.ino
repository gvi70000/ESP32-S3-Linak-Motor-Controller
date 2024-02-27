#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AM2302-Sensor.h>

const uint16_t UART_SPEED       = 9600;
// Pins used
const uint8_t POSITION_PIN    = 1; // ADC value from violer wire
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
const char* TOPIC_TEMPERATURE = "Linak1/Temperature";
const char* TOPIC_HUMIDITY    = "Linak1/Humidity";
const char* TOPIC_POSITION    = "Linak1/Position";
const char* TOPIC_BUTTON      = "Linak1/Button";

// Distance 0 to 500mm
// Board 1
// R2 = 0.996k, R1 = 9.99k, @10V the output is 907mV or 3378 ADC
// @ 500mm we have 2880ADC, so the distance d = 0.1736*ADC units
const float  POSITION_COEFF     = 0.1736;
const float  POSITION_TOLERANCE = 0.5; // in mm
const float  POSITION_MAX       = 499.5; // in mm

const uint16_t WIFI_PERIOD      = 1000;
const uint16_t WIFI_BREAK       = 2000;
const uint16_t MQTT_PERIOD      = 30; // Seconds
const uint8_t  DEBOUNCE_DELAY   = 50;
const uint8_t  DELAY_MQTT       = 50;
const uint8_t NO_OF_READINGS    = 10;

const uint8_t BUTTON_NONE       = 0;
const uint8_t BUTTON_DOWN       = 1;
const uint8_t BUTTON_UP         = 2;
const uint8_t BUTTON_STOP       = 3;

// Global variables
AM2302::AM2302_Sensor am2302(DHT22_PIN);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
volatile uint16_t mqttTime = 0;
float crtPosition = 0.0, prevPosition = 0.0;
uint8_t crtButton = BUTTON_NONE;
// Timer
hw_timer_t * timer = NULL;

// Function prototypes
void onTimer();
void setupWiFi();
void reconnectMQTT();
void publishToMQTT(const char* topic, const String& payload);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void moveUp();
void moveDown();
void moveStop();
void moveMotor();
float getPosition();
void getButtons();
void publishSensors();
void wifiErrorHandler();
void mqttErrorHandler();

void setup() {
  Serial.begin(UART_SPEED);
  analogSetAttenuation(ADC_2_5db);
  analogReadResolution(12);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_UP_PIN, INPUT);
  pinMode(BTN_DOWN_PIN, INPUT);
  pinMode(CTRL_UP_PIN, OUTPUT);
  pinMode(CTRL_DOWN_PIN, OUTPUT);
  digitalWrite(CTRL_UP_PIN, HIGH);
  digitalWrite(CTRL_DOWN_PIN, HIGH);
  am2302.begin();
  
  setupWiFi();
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.subscribe(TOPIC_POSITION); // Subscribe to position topic
  // Timer initialization
  // Set timer frequency to 1Mhz (default timer frequency is 80MHz)
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  Serial.println("Done.");
}

void loop() {
  yield();
  mqttClient.loop();
  // Check button press and set control pins
  moveMotor();
  // MQTT Check if position changed, new value form MQTT
  if (crtPosition != prevPosition) {
    prevPosition = crtPosition;
    crtPosition = getPosition();
    float minPosition = prevPosition - POSITION_TOLERANCE;
    float maxPosition = prevPosition + POSITION_TOLERANCE;
    // Move motor based on position change
    if (crtPosition < prevPosition) {
      // Pull motor rod
      moveDown();
    } else if (crtPosition > prevPosition) {
      // Push motor rod
      moveUp();
    }
    publishToMQTT(TOPIC_BUTTON, String(crtButton));
    // Wait until position gets in tolerance, target +/- 0.5mm
    while (1) {
      crtPosition = getPosition();
      // If we are in tolerance
      if((minPosition < crtPosition) && (crtPosition < maxPosition)) {
        // Set previous position
        prevPosition = crtPosition;
        // Exit loop
        break; 
      }
    }
    // Stop motor movement
    moveStop();
    // Publish all topics
    publishSensors();
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

// Timer interrupt function
void IRAM_ATTR onTimer() {
  mqttTime++;
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
      mqttClient.subscribe(TOPIC_POSITION);
    } else {
      delay(1000);
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
  if (strcmp(topic, TOPIC_POSITION) == 0) {
    crtPosition = messageTemp.toFloat();
  }
}

void moveUp() {
  digitalWrite(CTRL_UP_PIN, LOW);
  digitalWrite(CTRL_DOWN_PIN, HIGH);
}

void moveDown() {
  digitalWrite(CTRL_UP_PIN, HIGH);
  digitalWrite(CTRL_DOWN_PIN, LOW);
}

void moveStop() {
  digitalWrite(CTRL_UP_PIN, HIGH);
  digitalWrite(CTRL_DOWN_PIN, HIGH);
  crtButton = BUTTON_NONE;
}

void moveMotor() {
  // Check to see if we have a button pressed
  getButtons();
  // If a button is pressed
  if(crtButton) {
    float motorPos = getPosition();
    switch(crtButton){
      case BUTTON_DOWN: // 1
        if(motorPos > POSITION_TOLERANCE) moveDown();
      break;
      case BUTTON_UP: // 2
        if(motorPos < POSITION_MAX) moveUp();
      break;
      case BUTTON_STOP: // 3
        moveStop();
      break;
    }
  }
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
  crtButton = (!digitalRead(BTN_UP_PIN) << 1) | !digitalRead(BTN_DOWN_PIN);
  delay(DEBOUNCE_DELAY);
}

// Read sensor data and publish to MQTT
void publishSensors() {
  publishToMQTT(TOPIC_TEMPERATURE, String(am2302.get_Temperature()));
  publishToMQTT(TOPIC_HUMIDITY, String(am2302.get_Humidity()));
  publishToMQTT(TOPIC_POSITION, String(crtPosition));
  publishToMQTT(TOPIC_BUTTON, String(crtButton));
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
