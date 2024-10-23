#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// WiFi credentials
const char* ssid = "Redmi 12 5G";
const char* password = "param'shotspot";

// WebSocket client object
WebSocketsClient webSocket;

// WebSocket server address and port
const char* serverHost = "192.168.182.156";
const uint16_t serverPort = 8080;

const char* device = "esp32";
const char* secret = "esp32secret";
const char* id = "qq0k-rn0t-txew-qg0r";
const char* sensors = "PIR,Accelerometer";

const int pirSensorPins[] = { 4, 14 };
const int numSensors = sizeof(pirSensorPins) / sizeof(pirSensorPins[0]);

unsigned long lastSentTime = 0;
const unsigned long sendInterval = 60000;
bool activityDetected = false;


Adafruit_MPU6050 mpu;

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

void initializeSensors() {
  for (int i = 0; i < numSensors; i++) {
    pinMode(pirSensorPins[i], INPUT_PULLUP);
  }
}

void initializeMPU6050() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");

  // Configure the MPU6050 sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void initializeWebSocket() {
  webSocket.begin(serverHost, serverPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(500);
}

void readAndSendSensorData() {
  // JSON document to store the data
  JsonDocument doc;
  doc["user"] = device;
  doc["esp_UID"] = id;
  doc["secret"] = secret;
  doc["event"] = "sensor_data";

  // Read PIR sensor states
  JsonObject data = doc["data"].to<JsonObject>();
  for (int i = 0; i < numSensors; i++) {
    int sensorState = pirSensorTriggered(pirSensorPins[i]);
    data[String("PIR_A_") + String(i)] = sensorState;
  }

  // Read MPU6050 data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  JsonObject accData = data["accelerometer"].to<JsonObject>();

  accData["x"] = accel.acceleration.x;
  accData["y"] = accel.acceleration.y;
  accData["z"] = accel.acceleration.z;

  JsonObject gyroData = data["gyro"].to<JsonObject>();
  gyroData["x"] = gyro.gyro.x;
  gyroData["y"] = gyro.gyro.y;
  gyroData["z"] = gyro.gyro.z;

  data["temperature"] = temp.temperature;

  String output;

  doc.shrinkToFit();

  serializeJson(doc, output);

  webSocket.sendTXT(output);

  if (accel.acceleration.x == 0 && accel.acceleration.y == 0 && accel.acceleration.z == 0 && gyro.gyro.x == 0 && gyro.gyro.y == 0 && gyro.gyro.z == 0) {

    // Reinitialize MPU6050
    Serial.println("Zero readings detected, reinitializing MPU6050...");
    mpu.begin();
  }

  if (accel.acceleration.y > 0.15 || accel.acceleration.y < -0.15 || pirSensorTriggered(pirSensorPins[0]) || pirSensorTriggered(pirSensorPins[1])) {
    StaticJsonDocument<128> alertDoc;
    JsonArray arr = alertDoc.createNestedArray();
    if (accel.acceleration.y > 0.15 || accel.acceleration.y < -0.15) {
      arr.add("Door");
    }
    if (pirSensorTriggered(pirSensorPins[0])) {
      arr.add("PIR_A_0");
    }
    if (pirSensorTriggered(pirSensorPins[1])) {
      arr.add("PIR_A_1");
    }

    sendAlert(arr);
  }

  // Serialize JSON to string


  delay(100); 

bool pirSensorTriggered(int pin) {
  int currentState = digitalRead(pin);
  delay(100);
  int newState = digitalRead(pin);
  return currentState == LOW && newState == LOW;
}

void sendAlert(JsonArray arr) {
  JsonDocument doc;

  doc["user"] = device;
  doc["esp_UID"] = id;
  doc["secret"] = secret;
  doc["event"] = "alert";

  JsonObject data = doc["data"].to<JsonObject>();
  JsonArray data_alert = data["alert_sensor"].to<JsonArray>();
  for (JsonVariant value : arr) {
    data_alert.add(value);
  }
  String output;
  Serial.println("alert!!");

  serializeJson(doc, output);
  webSocket.sendTXT(output);
}

void handshakeServer() {
  String json;
  JsonDocument doc;
  doc["user"] = device;
  doc["esp_UID"] = id;
  doc["secret"] = secret;
  doc["event"] = "initialise";

  doc.shrinkToFit();

  serializeJson(doc, json);

  webSocket.sendTXT(json);
}

// Callback function to handle WebSocket events
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server");
      handshakeServer();
      break;
    case WStype_TEXT:
      Serial.printf("Received text: %s\n", payload);
      handleCommand(payload, length);
      break;
    case WStype_BIN:
      Serial.println("Received binary data");
      break;
    default:
      break;
  }
}

void handleCommand(uint8_t* payload, size_t length) {
  String command = String((char*)payload).substring(0, length);
  Serial.printf("Command received: %s\n", command.c_str());
}

bool checkForActivity() {
  // Read sensor values (e.g., accelerometer and PIR sensors)
  int pir1State = digitalRead(pirSensorPins[0]);
  int pir2State = digitalRead(pirSensorPins[1]);

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Define thresholds for detecting activity
  bool doorMoved = (accel.acceleration.y > 0.15 || accel.acceleration.y < -0.15);
  bool motionDetected = (pir1State == HIGH || pir2State == HIGH);

  // Return true if any activity is detected
  return (doorMoved || motionDetected);
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  connectToWiFi();
  initializeWebSocket();
  initializeMPU6050();
  initializeSensors();
}

void loop() {
  // Keep the WebSocket connection alive
  webSocket.loop();

  // Get the current time
  readAndSendSensorData();
}