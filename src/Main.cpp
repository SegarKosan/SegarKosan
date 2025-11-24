/*
 * SegarKosan: Smart Kosan with Odor Detection
 * 
 * PIN CONFIGURATION (ESP32-C3):
 * =============================
 * DHT22:     GPIO3 (Analog ADC)
 * SH1106:    GPIO8 (SDA), GPIO9 (SCL) - I2C
 * MQ135:     GPIO2 (Analog ADC)
*/

#include "DHT22.h"
#include "MQ135.h"
#include "esp32c3.h"
#include "SSH1106.h"
#include <time.h>


DHT22Sensor dht22;
SH1106Display oled;
MQ135Sensor mq135;
WebSocketsClient webSocket;
unsigned long lastSend = 0;

// WiFi & Websocket Config
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* websocket_server = WEBSOCKET_SERVER; 
const uint16_t websocket_port = WEBSOCKET_PORT;

// Event Handler WebSocket 
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[INFO] Connected to WebSocket Server!");
      break;
    case WStype_DISCONNECTED:
      Serial.println("[INFO] Disconnected from server! Reconnecting...");
      break;
    case WStype_TEXT:
      Serial.printf("[INFO] Message from server: %s\n", payload);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  Serial.println(F("=== SegarKosan on ESP32-C3 ==="));
  
  // Initialize OLED
  if (!oled.begin()) { Serial.println(F("[ERROR] SH1106 allocation failed")); while (1) { delay(1000); } }
  oled.clear();
  // Animate Initializing
  for (int i = 0; i < 6; i++) {
    oled.displayBootupMessage("Initializing", 10, i);
    delay(250);
  }
  Serial.println(F("[INFO] SH1106 display initialized"));

  // Connect to WiFi using Net::begin
  oled.displayBootupMessage("Networking", 30, 0);
  Net::Config cfg;
  cfg.ssid = ssid;
  cfg.pass = password;
  Net::begin(cfg);

  // Initialize sensors
  Serial.println(F("Start DHT22..."));
  dht22.begin();

  // MQ135 preheat
  oled.displayBootupMessage("Preheating", 40, 0);
  Serial.println(F("Preheating MQ-135 in clean air (30s)..."));
  mq135.preheat(30000, 50, [](int remaining) {
    static int frame = 0;
    static unsigned long lastFrame = 0;
    unsigned long now = millis();
    // Update animation every 250ms
    if (now - lastFrame > 250) {
      int progress = map(30 - remaining, 0, 30, 40, 90);
      oled.displayBootupMessage("Preheating", progress, frame++);
      lastFrame = now;
    }
  });
  mq135.begin(100, 100); // 100 samples, 100ms interval
  Serial.print(F("MQ-135 R0 = ")); Serial.println(mq135.getR0(), 3);

  oled.displayBootupMessage("Setup Complete!", 100);
  delay(1000);

  // WebSocket
  webSocket.begin(websocket_server, websocket_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

void loop() {
  Net::handle(); // Handle Net tasks (WebServer, MQTT if enabled)
  webSocket.loop();

  static float lastTemp = 0;
  static float lastHum = 0;
  static float lastHI = 0;
  static float lastCO2 = 0;

  unsigned long now = millis();
  if (now - lastSend > 5000) {
    lastSend = now;

    float humidity = dht22.readHumidity();
    float temperature = dht22.readTemperature();

    if (!dht22.isValidReading(temperature, humidity)) {
      Serial.println(F("[ERROR] DHT22 read failed"));
      // Keep last valid values or show error on specific page if needed
    } else {
      lastTemp = temperature;
      lastHum = humidity;
      lastHI = DHT22Sensor::computeHeatIndex(temperature, humidity);
    }

    mq135.update();
    // Moving average CO2
    static const int N = 5;
    static float buf[N];
    static int idx = 0, filled = 0;
    float co2_raw = mq135.readCO2();
    float co2ppm = NAN;
    
    if (!isfinite(co2_raw) || co2_raw <= 0 || co2_raw > 50000) {
      Serial.println(F("[WARN] Invalid CO2 raw reading, skipping average update"));
      co2ppm = NAN;
      filled = 0; // Optionally reset buffer to avoid contamination
    } else {
      buf[idx] = co2_raw;
      idx = (idx + 1) % N;
      if (filled < N) filled++;
      float sum = 0;
      // Do not reset 'filled' here; preserve historical valid data
      co2ppm = sum / filled;
      if (!isfinite(co2ppm) || co2ppm <= 0 || co2ppm > 50000) co2ppm = NAN;
    }
    
    if (isfinite(co2ppm)) {
      lastCO2 = co2ppm;
    }

    // Serial debug
    Serial.print(F("Temp: ")); Serial.print(lastTemp,1); Serial.print("°C  ");
    Serial.print(F("Hum: ")); Serial.print(lastHum,1); Serial.print("%  ");
    Serial.print(F("Heat index: ")); Serial.print(lastHI,1); Serial.print("°C  ");
    Serial.print(F("CO2: ")); Serial.println(lastCO2,0);

    // WebSocket JSON
    StaticJsonDocument<256> doc;
    doc["type"] = "sensor";
    doc["device_id"] = "ESP32-C3";
    doc["payload"]["temperature"] = lastTemp;
    doc["payload"]["humidity"] = lastHum;
    doc["payload"]["heat_index"] = lastHI;
    doc["payload"]["co2"] = lastCO2;
    time_t epoch = time(nullptr);
    doc["timestamp"] = (epoch > 0) ? static_cast<long>(epoch) : static_cast<long>(millis() / 1000);

    String json; 
    if (serializeJson(doc, json) == 0) {
      Serial.println(F("[ERROR] JSON serialization failed"));
    } else if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[WARN] WiFi disconnected, skipping send"));
    } else {
      webSocket.sendTXT(json);
      Serial.print(F("[INFO] Sent data to server: "));
      Serial.println(json);
    }
  }

  // OLED Paging Logic (Every 4s)
  static unsigned long lastPageChange = 0;
  static int currentPage = 1;
  if (now - lastPageChange > 4000) {
    lastPageChange = now;
    oled.displayPage(currentPage, lastTemp, lastHum, lastHI, lastCO2);
    currentPage++;
    if (currentPage > 5) currentPage = 1;
  }
}
