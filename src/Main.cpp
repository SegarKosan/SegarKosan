/*
 * SegarKosan: Smart Kosan with Odor Detection (MQTT version)
 * ESP32-C3 Full Stable Build
 * Updated with Odor Analysis Logic (Weighted Score & Description)
 */

#include "DHT22.h"
#include "mq135.h" // Menggunakan file header yang baru diupdate
#include "esp32c3.h"
#include "SSH1106.h"
#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======================== SENSORS & DISPLAY ==========================
DHT22Sensor dht22;
SH1106Display oled;
MQ135Sensor mq135;

unsigned long lastSend = 0;

// ======================== WIFI & MQTT CONFIG ==========================
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* mqttServer = "192.168.18.76"; 
const uint16_t mqttPort = 1883;
const char* mqttTopic = "segar_kosan/sensors";

// ======================== SIMPLE PING FUNCTION ==========================
bool pingHost(const char* host, uint16_t port = 1883, uint16_t timeout = 1000) {
  WiFiClient testClient;
  unsigned long start = millis();

  Serial.printf("[PING] Testing %s:%d ... ", host, port);

  if (testClient.connect(host, port)) {
    testClient.stop();
    Serial.printf("OK (%lums)\n", millis() - start);
    return true;
  } else {
    Serial.println("FAILED");
    return false;
  }
}

// ======================== MQTT CONNECT ==============================
void mqttConnect() {
  Serial.println("[MQTT] Connecting...");

  while (!mqttClient.connected()) {
    if (mqttClient.connect("esp32c3-client")) {
      Serial.println("[MQTT] Connected!");
      break;
    } else {
      Serial.print("[MQTT] Failed rc=");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

// ======================== SETUP =====================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== SegarKosan on ESP32-C3 ==="));

  // OLED Init
  if (!oled.begin()) { 
    Serial.println(F("[ERROR] SH1106 allocation failed")); 
    while (1) delay(1000);
  }
  oled.clear();

  for (int i = 0; i < 6; i++) {
    oled.displayBootupMessage("Initializing", 10, i);
    delay(200);
  }

  // WiFi
  oled.displayBootupMessage("Networking", 30, 0);

  Net::Config cfg;
  cfg.ssid = ssid;
  cfg.pass = password;
  Net::begin(cfg);

  // Wait WiFi connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected!");

  // Sensors
  dht22.begin();

  oled.displayBootupMessage("Preheating", 40, 0);
  Serial.println(F("MQ135 preheat 30s..."));

  // Preheat dengan callback progress bar ke OLED
  mq135.preheat(30000, 50, [](int remaining) {
    static int frame = 0;
    static unsigned long lastFrame = 0;
    unsigned long now = millis();
    if (now - lastFrame > 250) {
      int progress = map(30 - remaining, 0, 30, 40, 90);
      oled.displayBootupMessage("Preheating", progress, frame++);
      lastFrame = now;
    }
  });

  mq135.begin(100, 100);
  Serial.print(F("MQ135 R0 = ")); Serial.println(mq135.getR0(), 3);

  oled.displayBootupMessage("Setup Complete!", 100);
  delay(800);

  // MQTT
  mqttClient.setServer(mqttServer, mqttPort);

  // First ping test
  pingHost(mqttServer, mqttPort);

  mqttConnect();
}

// ======================== MAIN LOOP =====================================
void loop() {
  unsigned long now = millis();

  Net::handle();
  mqttClient.loop();

  // Reconnect if needed
  if (!mqttClient.connected()) {
    Serial.println("[DEBUG] MQTT lost! Testing reachability...");
    pingHost(mqttServer, mqttPort);
    mqttConnect();
  }

  // Static variables untuk menyimpan pembacaan terakhir
  static float lastTemp = 25.0; // Default ke nilai ideal
  static float lastHum = 50.0;  // Default ke nilai ideal
  static float lastHI = 0;
  static float lastCO2 = 400.0; // Default ke nilai fresh
  static int lastScore = 0;     
  static String lastStatus = "OK";
  static String lastLevelDesc = ""; // Variabel baru untuk deskripsi level

  if (now - lastSend > 5000) {
    lastSend = now;

    // --- 1. Read DHT22 ---
    float humidity = dht22.readHumidity();
    float temperature = dht22.readTemperature();

    if (dht22.isValidReading(temperature, humidity)) {
      lastTemp = temperature;
      lastHum = humidity;
      lastHI = DHT22Sensor::computeHeatIndex(temperature, humidity);
    }

    // --- 2. Read MQ135 PPM ---
    mq135.update();
    float co2_raw = mq135.readCO2();
    if (isfinite(co2_raw) && co2_raw > 0 && co2_raw < 50000) {
      lastCO2 = co2_raw;
    }

    // --- 3. Read Raw Analog for Analysis ---
    int rawADC = analogRead(MQ135_ANALOG_PIN); 
    // Konversi ke skala 10-bit (0-1023) untuk konsistensi perhitungan
    int raw10bit = rawADC >> 2; 

    // --- 4. Calculate Odor Score & Type (Fixed) ---
    // Update panggilan fungsi agar sesuai dengan header MQ135Sensor yang baru
    // calculateOdorScore(float co2, float temp, float hum, int raw_mq)
    lastScore = mq135.calculateOdorScore(lastCO2, lastTemp, lastHum, raw10bit);
    
    // detectOdorType(int raw_mq, float hum)
    lastStatus = mq135.detectOdorType(raw10bit, lastHum);
    
    // Dapatkan deskripsi level
    lastLevelDesc = mq135.getScoreDescription(lastScore);

    // --- 5. JSON build ---
    StaticJsonDocument<512> doc;
    doc["type"] = "sensor";
    doc["device_id"] = "ESP32-C3";
    
    JsonObject payload = doc.createNestedObject("payload");
    payload["temperature"] = lastTemp;
    payload["humidity"] = lastHum;
    payload["heat_index"] = lastHI;
    payload["co2"] = lastCO2;
    payload["odor_score"] = lastScore;
    payload["odor_status"] = lastStatus;
    payload["odor_level"] = lastLevelDesc;

    char out[512];
    serializeJson(doc, out);

    mqttClient.publish(mqttTopic, out);
    
    // Debug Print
    Serial.print(F("[DATA] Score: ")); Serial.print(lastScore);
    Serial.print(F(" | Type: ")); Serial.print(lastStatus);
    Serial.print(F(" | Level: ")); Serial.println(lastLevelDesc);
  }

  // --- OLED Display Pages ---
  static unsigned long lastPageChange = 0;
  static int currentPage = 1;

  if (now - lastPageChange > 4000) {
    lastPageChange = now;
    
    oled.displayPage(currentPage, lastTemp, lastHum, lastHI, lastCO2);

    currentPage++;
    if (currentPage > 5) currentPage = 1;
  }
};