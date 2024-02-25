#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <AM2302-Sensor.h>

// Pins used
const uint8_t POSITION_PIN    = 1;
const uint8_t LED_PIN         = 4;
const uint8_t BTN_UP_PIN      = 10;
const uint8_t BTN_DOWN_PIN    = 11;
const uint8_t CTRL_UP_PIN     = 12;
const uint8_t CTRL_DOWN_PIN   = 13;
const uint8_t DHT22_PIN       = 14;

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
const char* TOPIC_BTN_UP      = "Linak1/Up";
const char* TOPIC_BTN_DOWN    = "Linak1/Down";
const float  POSITION_COEFF   = 7.976;
const uint16_t UART_SPEED     = 9600;
const uint16_t WIFI_PERIOD    = 1000;
const uint16_t MQTT_PERIOD    = 30000;
const uint8_t  DEBOUNCE_DELAY = 50;
const uint8_t  DELAY_MQTT     = 50;

// Global variables
AM2302::AM2302_Sensor am2302(DHT22_PIN);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
volatile uint16_t mqttTime = 0;
uint16_t crtPosition = 0, prevPosition = 0;

// Function prototypes
void onTimer();
void setupWiFi();
void reconnectMQTT();
void publishToMQTT(const char* topic, const String& payload);
void mqttCallback(char* topic, byte* payload, unsigned int length);
float getPosition();
void getButtons();
void getSensors();

void setup() {
  Serial.begin(UART_SPEED);
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
}

void loop() {
  mqttClient.loop();
  getButtons();
  
  // Check if position changed
  if (crtPosition != prevPosition) {
    prevPosition = crtPosition;
    crtPosition = getPosition();
    // Move motor based on position change
    if (crtPosition < prevPosition) {
      digitalWrite(CTRL_UP_PIN, LOW);
    } else if (crtPosition > prevPosition) {
      digitalWrite(CTRL_DOWN_PIN, LOW);
    }
    // Wait until position stabilizes
    while (crtPosition != prevPosition) {
      crtPosition = getPosition();
    }
    // Stop motor movement
    digitalWrite(CTRL_UP_PIN, HIGH);
    digitalWrite(CTRL_DOWN_PIN, HIGH);
  }

  // Publish sensor data to MQTT server
  if (mqttTime >= MQTT_PERIOD) {
    getSensors();
    mqttTime = 0;
  }
}

// Timer interrupt function
void onTimer() {
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
    crtPosition = messageTemp.toInt();
  }
}

// Read current position from analog pin
float getPosition() {
  return analogRead(POSITION_PIN) / POSITION_COEFF;
}

// Read button states
void getButtons() {
  if (digitalRead(BTN_UP_PIN)) {
    publishToMQTT(TOPIC_BTN_UP, "1");
  } else if (digitalRead(BTN_DOWN_PIN)) {
    publishToMQTT(TOPIC_BTN_DOWN, "1");
  }
  delay(DEBOUNCE_DELAY);
}

// Read sensor data and publish to MQTT
void getSensors() {
  publishToMQTT(TOPIC_TEMPERATURE, String(am2302.get_Temperature()));
  publishToMQTT(TOPIC_HUMIDITY, String(am2302.get_Humidity()));
  publishToMQTT(TOPIC_POSITION, String(crtPosition));
}
