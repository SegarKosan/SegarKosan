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

#undef MQ135_VOLTAGE_RESOLUTION
#define MQ135_VOLTAGE_RESOLUTION 3.3f
#undef MQ135_ADC_BIT_RESOLUTION
#define MQ135_ADC_BIT_RESOLUTION 12
#undef MQ135_ANALOG_PIN
#define MQ135_ANALOG_PIN 0   // GPIO2 (Analog ADC) 

DHT22Sensor dht22;
SH1106Display oled;
MQ135Sensor mq135;
WebSocketsClient webSocket;
unsigned long lastSend = 0;

// WiFi & Websocket Config
const char* ssid = "Ersyadha_32";
const char* password = "ersyadha123";
const char* websocket_server = "192.168.18.115"; 
const uint16_t websocket_port = 8080;

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
  
  // Connect to WiFi using Net::begin
  Net::Config cfg;
  cfg.ssid = ssid;
  cfg.pass = password;
  Net::begin(cfg);

#ifdef ESP32
  analogReadResolution(MQ135_ADC_BIT_RESOLUTION);
  analogSetPinAttenuation(MQ135_ANALOG_PIN, ADC_11db);
#endif

  // Initialize sensors
  dht22.begin();

  // MQ135 warm-up & calibration
  Serial.println(F("Calibrating MQ-135 in clean air..."));
  const unsigned long warmupMs = 5000; // 5s warm-up
  unsigned long wstart = millis();
  while (millis() - wstart < warmupMs) { mq135.update(); delay(50); }
  mq135.begin(100, 100); // 100 samples, 100ms interval
  Serial.print(F("MQ-135 R0 = ")); Serial.println(mq135.getR0(), 3);

  // Initialize OLED
  if (!oled.begin()) { Serial.println(F("❌ SH1106 allocation failed")); while (1); }
  oled.clear();
  oled.displayBootupMessage();
  delay(1500);
  Serial.println(F("[INFO] SH1106 display initialized"));

  // WebSocket
  webSocket.begin(websocket_server, websocket_port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

// ====== Loop ======
void loop() {
  Net::handle(); // Handle Net tasks (WebServer, MQTT if enabled)
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastSend > 5000) {
    lastSend = now;

    float humidity = dht22.readHumidity();
    float temperature = dht22.readTemperature();

    if (!dht22.isValidReading(temperature, humidity)) {
      Serial.println(F("[ERROR] DHT22 read failed"));
      oled.displayError("DHT22 Error");
      oled.show();
      return;
    }

    float heatIndex = DHT22Sensor::computeHeatIndex(temperature, humidity);

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
      for (int i = 0; i < filled; i++) sum += buf[i];
      co2ppm = sum / filled;
      if (!isfinite(co2ppm) || co2ppm <= 0 || co2ppm > 50000) co2ppm = NAN;
    }

    // Serial debug
    Serial.print(F("Temp: ")); Serial.print(temperature,1); Serial.print("°C  ");
    Serial.print(F("Hum: ")); Serial.print(humidity,1); Serial.print("%  ");
    Serial.print(F("Heat index: ")); Serial.print(heatIndex,1); Serial.print("°C  ");
    Serial.print(F("CO2: ")); if(isfinite(co2ppm)) Serial.println(co2ppm,0); else Serial.println(F("ERR"));

    // OLED
    oled.displayHeader();
    oled.displayDHT22(temperature, humidity, heatIndex);
    oled.displayMQ135(co2ppm);
    oled.show();

    // WebSocket JSON
    StaticJsonDocument<256> doc;
    doc["type"] = "sensor";
    doc["device_id"] = "ESP32-C3";
    doc["payload"]["temperature"] = temperature;
    doc["payload"]["humidity"] = humidity;
    doc["payload"]["heat_index"] = heatIndex;
    doc["payload"]["co2"] = isfinite(co2ppm) ? co2ppm : -1;
    time_t epoch = time(nullptr);
    doc["timestamp"] = (epoch > 0) ? static_cast<long>(epoch) : static_cast<long>(millis() / 1000);

    String json; 
    if (serializeJson(doc, json) == 0) {
      Serial.println(F("[ERROR] JSON serialization failed"));
      return;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("[WARN] WiFi disconnected, skipping send"));
      return;
    }

    webSocket.sendTXT(json);
    Serial.print(F("[INFO] Sent data to server: "));
    Serial.println(json);
  }
}
