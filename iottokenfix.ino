#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Hash.h>

// Wi-Fi credentials
const char* ssid = ".";
const char* password = "hulubalang";

// MQTT broker
const char* mqttServer = "192.168.101.81";
const int mqttPort = 1883;
const char* mqttUsername = "";
const char* mqttPassword = "";

// MQTT topics
const char* ultrasonicDataTopic = "ultrasonic";
const char* tokenVerificationTopic = "token_verification";

// Ultrasonic sensor pin configuration
const int trigPin = D6;
const int echoPin = D5;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long previousTimestamp = 0;
String tokenVerification = "";

// Salt for hashing
const String salt = "nicholas";

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP8266Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }

  // Publish token verification status
  mqttClient.publish(tokenVerificationTopic, tokenVerification.c_str());
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Read ultrasonic sensor data
  float distance = readUltrasonicSensor();

  // Generate token based on current timestamp and sensor data
  unsigned long currentTimestamp = millis();
  if (currentTimestamp - previousTimestamp >= 1000) {
    previousTimestamp = currentTimestamp;

    // Create a JSON object
    StaticJsonDocument<128> jsonDocument;
    jsonDocument["timestamp"] = currentTimestamp;
    jsonDocument["token"] = generateToken(currentTimestamp, distance);
    jsonDocument["distance"] = distance;

    // Serialize JSON to a string
    String jsonString;
    serializeJson(jsonDocument, jsonString);

    // Publish the JSON message to the ultrasonicDataTopic
    mqttClient.publish(ultrasonicDataTopic, jsonString.c_str());
    Serial.println("Data published");
  }

  // Update token verification status
  mqttClient.publish(tokenVerificationTopic, tokenVerification.c_str());

  delay(100); // Delay between readings
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (mqttClient.connect("ESP8266Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Callback function for MQTT messages
  Serial.print("Message received on topic: ");
  Serial.println(topic);
}

float readUltrasonicSensor() {
  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo response
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance based on the speed of sound
  float distance = duration * 0.034 / 2;

  // Return the calculated distance
  return distance;
}

String generateToken(unsigned long timestamp, float distance) {
  // Convert the timestamp to a string
  String timestampString = String(timestamp);

  // Generate a token with salt, timestamp, and sensor data
  String token = timestampString + salt + String(distance);

  // Create an MD5 hash object
  MD5Builder md5;
  md5.begin();
  md5.add(token);
  md5.calculate();

  // Get the hashed token as a hexadecimal string
  String hashedToken = md5.toString();

  return hashedToken;
}