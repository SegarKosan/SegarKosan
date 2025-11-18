#pragma once
// esp32.h — HTTP JSON endpoint /state untuk ESP32/ESP32-C3

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

namespace Net {

// Callback types
typedef float (*ReadR0Func)();
typedef bool (*RecalibrateFunc)(unsigned long samples, unsigned long intervalMs, unsigned long warmupMs);

struct Config {
  const char* ssid      = "YourWiFi";
  const char* pass      = "YourPass";
  const char* hostname  = "esp32c3";    // mDNS hostname (esp32c3.local)
  const char* ap_ssid   = "ESP32C3-AP";
  const char* ap_pass   = "pass12345";   // min 8 char
  bool        enable_ap_fallback = true; // SoftAP jika STA gagal
  bool        enable_cors = true;        // Access-Control-Allow-Origin: *
  uint32_t    sta_timeout_ms = 10000;     // tunggu koneksi STA

  // Optional: Static IP settings for STA mode (use DHCP if 0.0.0.0)
  IPAddress   sta_ip   = IPAddress(0,0,0,0);
  IPAddress   sta_gw   = IPAddress(0,0,0,0);
  IPAddress   sta_sn   = IPAddress(0,0,0,0);
  IPAddress   sta_dns1 = IPAddress(0,0,0,0);
  IPAddress   sta_dns2 = IPAddress(0,0,0,0);

  // Optional: SoftAP IP settings (defaults to 192.168.4.1/24)
  IPAddress   ap_ip    = IPAddress(192,168,4,1);
  IPAddress   ap_gw    = IPAddress(192,168,4,1);
  IPAddress   ap_sn    = IPAddress(255,255,255,0);

  // Optional callbacks for MQ135 integration
  ReadR0Func      readR0      = nullptr;
  RecalibrateFunc recalibrate = nullptr;

  // MQTT Configuration
  const char* mqtt_server   = nullptr;           // MQTT broker address (e.g., "broker.hivemq.com")
  uint16_t    mqtt_port     = 1883;              // MQTT broker port
  const char* mqtt_user     = nullptr;           // MQTT username (nullptr if no auth)
  const char* mqtt_pass     = nullptr;           // MQTT password
  const char* mqtt_client_id = "esp32c3-sensor"; // MQTT client ID
  const char* mqtt_topic    = "kosan/room204/sensors"; // MQTT topic for publishing
  const char* room_id       = "204";             // Room identifier for JSON payload
  uint32_t    mqtt_interval_ms = 5000;           // Publish interval in ms (default 5s)
};

// cache nilai terakhir
inline volatile float gT = NAN, gH = NAN, gHI = NAN, gCO2 = NAN;
inline volatile uint32_t gTS = 0;

// MQTT state
inline WiFiClient& wifiClient() { static WiFiClient c; return c; }
inline PubSubClient& mqttClient() { static PubSubClient c(wifiClient()); return c; }
inline bool gMqttEnabled = false;
inline uint32_t gLastMqttAttempt = 0;
inline uint32_t gLastMqttPublish = 0;
inline const char* gRoomId = "204";
inline const char* gMqttTopic = "kosan/room204/sensors";
inline uint32_t gMqttInterval = 5000;

// internal server
inline WebServer& server() {
  static WebServer s(80);
  return s;
}

// Forward declaration for helper returning active IP
inline IPAddress ip();

// Store last used network config for reconnects
inline Config& lastConfig() { static Config c; return c; }
inline bool& hasConfig() { static bool init = false; return init; }

// Stored callbacks
inline ReadR0Func& cbReadR0() { static ReadR0Func f = nullptr; return f; }
inline RecalibrateFunc& cbRecal() { static RecalibrateFunc f = nullptr; return f; }

inline void _setCors() {
  if (!server().hasHeader("Access-Control-Allow-Origin")) {
    server().sendHeader("Access-Control-Allow-Origin", "*");
  }
  server().sendHeader("Cache-Control", "no-cache");
}

inline void _handleRoot() {
  _setCors();
  const char html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Air Quality Monitor</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box}
    body{font:14px system-ui,-apple-system,Segoe UI,Roboto,Arial;background:#f5f5f5;padding:20px}
    .container{max-width:600px;margin:0 auto;background:#fff;border-radius:12px;padding:20px;box-shadow:0 2px 8px rgba(0,0,0,0.1)}
    h1{font-size:24px;margin-bottom:4px;color:#333}
    .subtitle{color:#666;font-size:13px;margin-bottom:20px}
    .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;margin-bottom:16px}
    .card{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);border-radius:10px;padding:16px;color:#fff}
    .card.temp{background:linear-gradient(135deg,#f093fb 0%,#f5576c 100%)}
    .card.hum{background:linear-gradient(135deg,#4facfe 0%,#00f2fe 100%)}
    .card.hi{background:linear-gradient(135deg,#fa709a 0%,#fee140 100%)}
    .card.co2{background:linear-gradient(135deg,#30cfd0 0%,#330867 100%)}
    .label{font-size:12px;opacity:0.9;margin-bottom:4px;text-transform:uppercase;letter-spacing:0.5px}
    .value{font-size:32px;font-weight:700;line-height:1}
    .unit{font-size:18px;opacity:0.9;margin-left:4px}
    .footer{text-align:center;color:#999;font-size:12px;padding-top:12px;border-top:1px solid #eee}
    .status{display:inline-block;width:8px;height:8px;border-radius:50%;background:#4caf50;margin-right:6px;animation:pulse 2s infinite}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
    .error{color:#f44336}
  </style>
</head>
<body>
  <div class="container">
    <h1>🌡️ Air Quality Monitor</h1>
    <div class="subtitle">ESP32-C3 Real-time Sensor Dashboard</div>
    
    <div class="grid">
      <div class="card temp">
        <div class="label">Temperature</div>
        <div class="value" id="t">--</div>
      </div>
      <div class="card hum">
        <div class="label">Humidity</div>
        <div class="value" id="h">--</div>
      </div>
      <div class="card hi">
        <div class="label">Heat Index</div>
        <div class="value" id="hi">--</div>
      </div>
      <div class="card co2">
        <div class="label">CO₂ Level</div>
        <div class="value" id="co2">--</div>
      </div>
    </div>
    
    <div class="footer">
      <span class="status"></span>
      <span id="status">Connecting...</span> • 
      Last update: <span id="ts">--</span>
    </div>
  </div>

  <script>
  async function update(){
    try{
      const r = await fetch('/state', {cache: 'no-store'});
      if(!r.ok) throw new Error(r.status);
      const d = await r.json();
      document.getElementById('t').innerHTML = d.t.toFixed(1) + '<span class="unit">°C</span>';
      document.getElementById('h').innerHTML = d.h.toFixed(1) + '<span class="unit">%</span>';
      document.getElementById('hi').innerHTML = d.hi.toFixed(1) + '<span class="unit">°C</span>';
      document.getElementById('co2').innerHTML = d.co2.toFixed(0) + '<span class="unit">ppm</span>';
      document.getElementById('ts').textContent = new Date().toLocaleTimeString();
      document.getElementById('status').textContent = 'Connected';
      document.getElementById('status').className = '';
    }catch(e){
      document.getElementById('status').textContent = 'Connection Error';
      document.getElementById('status').className = 'error';
      console.error(e);
    }
  }
  update();
  setInterval(update, 2000);
  </script>
</body>
</html>
)rawliteral";
  server().send(200, "text/html", html);
}

inline void _handleState() {
  _setCors();
  char buf[192];
  auto nz = [](float v){ return isfinite(v) ? v : 0.0f; };
  snprintf(buf, sizeof(buf),
    "{\"t\":%.1f,\"h\":%.1f,\"hi\":%.1f,\"co2\":%.0f,\"ts\":%lu}",
    nz(gT), nz(gH), nz(gHI), nz(gCO2), (unsigned long)gTS);
  server().send(200, "application/json", buf);
}

inline void _handleNet() {
  _setCors();
  bool sta = (WiFi.getMode() & WIFI_MODE_STA) && (WiFi.status() == WL_CONNECTED);
  bool ap  = (WiFi.getMode() & WIFI_MODE_AP);
  String mode = sta ? "STA" : (ap ? "AP" : "NONE");
  IPAddress ipaddr = ip();
  String ssid = sta ? WiFi.SSID() : (ap ? WiFi.softAPSSID() : String(""));
  long rssi = sta ? WiFi.RSSI() : 0;
  String mac = sta ? WiFi.macAddress() : WiFi.softAPmacAddress();
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"mode\":\"%s\",\"ssid\":\"%s\",\"rssi\":%ld,\"ip\":\"%s\",\"mac\":\"%s\"}",
           mode.c_str(), ssid.c_str(), rssi, ipaddr.toString().c_str(), mac.c_str());
  server().send(200, "application/json", buf);
}

inline void _handleMqR0() {
  _setCors();
  float r0 = NAN;
  if (cbReadR0()) r0 = cbReadR0()();
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"r0\":%.3f}", isfinite(r0) ? r0 : 0.0f);
  server().send(cbReadR0() ? 200 : 501, "application/json", buf);
}

inline void _handleMqRecalibrate() {
  _setCors();
  if (!cbRecal()) {
    server().send(501, "application/json", "{\"ok\":false,\"err\":\"recalibrate not available\"}");
    return;
  }
  unsigned long s = server().hasArg("s") ? strtoul(server().arg("s").c_str(), nullptr, 10) : 100;
  unsigned long i = server().hasArg("i") ? strtoul(server().arg("i").c_str(), nullptr, 10) : 100;
  unsigned long w = server().hasArg("w") ? strtoul(server().arg("w").c_str(), nullptr, 10) : 3000;
  bool ok = cbRecal()(s, i, w);
  float r0 = cbReadR0() ? cbReadR0()() : NAN;
  char buf[160];
  snprintf(buf, sizeof(buf), "{\"ok\":%s,\"r0\":%.3f,\"s\":%lu,\"i\":%lu,\"w\":%lu}",
           ok ? "true" : "false", isfinite(r0) ? r0 : 0.0f, s, i, w);
  server().send(ok ? 200 : 500, "application/json", buf);
}

inline void _handleMqtt() {
  _setCors();
  bool connected = mqttClient().connected();
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"connected\":%s,\"broker\":\"%s\",\"topic\":\"%s\",\"lastPublish\":%lu,\"interval\":%lu}",
           connected ? "true" : "false",
           mqttClient().connected() ? "connected" : "disconnected",
           gMqttTopic,
           (unsigned long)gLastMqttPublish,
           (unsigned long)gMqttInterval);
  server().send(200, "application/json", buf);
}

inline void begin(const Config& cfg = Config{}) {
  // Register optional MQ135 callbacks if provided
  cbReadR0() = cfg.readR0;
  cbRecal()  = cfg.recalibrate;

  // Mode STA
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  auto ipSet = [](const IPAddress& ip){ return ip != IPAddress(0,0,0,0); };

  // Apply static IP if provided (must be before WiFi.begin)
  if (ipSet(cfg.sta_ip) && ipSet(cfg.sta_gw) && ipSet(cfg.sta_sn)) {
    bool ok = WiFi.config(cfg.sta_ip, cfg.sta_gw, cfg.sta_sn,
                          ipSet(cfg.sta_dns1) ? cfg.sta_dns1 : IPAddress(0,0,0,0),
                          ipSet(cfg.sta_dns2) ? cfg.sta_dns2 : IPAddress(0,0,0,0));
    if (!ok) {
      Serial.println(F("[NET] WiFi.config (STA) failed"));
    }
  }
  // Set hostname if provided
  if (cfg.hostname && cfg.hostname[0]) {
    WiFi.setHostname(cfg.hostname);
  }
  WiFi.begin(cfg.ssid, cfg.pass);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < cfg.sta_timeout_ms) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    // connected as STA
    Serial.print(F("[NET] STA IP: ")); Serial.println(WiFi.localIP());
    Serial.print(F("[NET] Gateway: ")); Serial.println(WiFi.gatewayIP());
    Serial.print(F("[NET] DNS: ")); Serial.println(WiFi.dnsIP());
    
    // Start mDNS (wait for IP to be fully configured)
    delay(500);
    if (cfg.hostname && cfg.hostname[0]) {
      Serial.print(F("[NET] Starting mDNS with hostname: ")); Serial.println(cfg.hostname);
      if (MDNS.begin(cfg.hostname)) {
        MDNS.addService("http", "tcp", 80);
        Serial.println(F("[NET] ✓ mDNS started successfully"));
        Serial.print(F("[NET] Access at: http://")); Serial.print(cfg.hostname); Serial.println(F(".local/"));
        Serial.print(F("[NET] Or use IP: http://")); Serial.println(WiFi.localIP());
      } else {
        Serial.println(F("[NET] ✗ mDNS begin FAILED!"));
        Serial.println(F("[NET] Try:"));
        Serial.println(F("[NET]   1. Use IP address instead"));
        Serial.println(F("[NET]   2. Check if hostname is valid (no spaces/special chars)"));
        Serial.println(F("[NET]   3. Ensure Bonjour/Avahi is installed on your device"));
      }
    } else {
      Serial.println(F("[NET] WARNING: No hostname configured for mDNS"));
    }
  } else if (cfg.enable_ap_fallback) {
    // fallback SoftAP
    WiFi.mode(WIFI_AP);
    // Optionally set AP IP before starting AP
    if (cfg.ap_ip != IPAddress(0,0,0,0) && cfg.ap_sn != IPAddress(0,0,0,0)) {
      if (!WiFi.softAPConfig(cfg.ap_ip, cfg.ap_gw, cfg.ap_sn)) {
        Serial.println(F("[NET] softAPConfig failed"));
      }
    }
    WiFi.softAP(cfg.ap_ssid, cfg.ap_pass);
    // Set AP hostname if available
    if (cfg.hostname && cfg.hostname[0]) {
#if defined(ESP32)
      WiFi.softAPsetHostname(cfg.hostname);
#endif
      if (MDNS.begin(cfg.hostname)) {
        MDNS.addService("http", "tcp", 80);
        Serial.print(F("[NET] mDNS(AP): http://")); Serial.print(cfg.hostname); Serial.println(F(".local/"));
      }
    }
    Serial.print(F("[NET] SoftAP IP: ")); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[NET] STA failed and AP fallback disabled"));
  }

  // Initialize MQTT if configured
  if (cfg.mqtt_server && cfg.mqtt_server[0]) {
    gMqttEnabled = true;
    gRoomId = cfg.room_id;
    gMqttTopic = cfg.mqtt_topic;
    gMqttInterval = cfg.mqtt_interval_ms;
    mqttClient().setServer(cfg.mqtt_server, cfg.mqtt_port);
    Serial.print(F("[NET] MQTT broker: ")); Serial.print(cfg.mqtt_server);
    Serial.print(":"); Serial.println(cfg.mqtt_port);
  } else {
    gMqttEnabled = false;
    Serial.println(F("[NET] MQTT disabled (no broker configured)"));
  }

  // routes
  server().on("/", [](){ _handleRoot(); });
  server().on("/state", HTTP_GET, [](){ _handleState(); });
  server().on("/net", HTTP_GET, [](){ _handleNet(); });
  server().on("/mq/r0", HTTP_GET, [](){ _handleMqR0(); });
  server().on("/mq/recalibrate", HTTP_GET, [](){ _handleMqRecalibrate(); });
  server().on("/mqtt", HTTP_GET, [](){ _handleMqtt(); });

  // preflight CORS (opsional)
  server().onNotFound([](){
    if (server().method() == HTTP_OPTIONS) {
      _setCors();
      server().sendHeader("Access-Control-Allow-Methods", "GET,OPTIONS");
      server().sendHeader("Access-Control-Allow-Headers", "Content-Type");
      server().send(204);
    } else {
      server().send(404, "text/plain", "Not found");
    }
  });

  server().begin();

  // Save config for later reconnects
  lastConfig() = cfg;
  hasConfig() = true;
}

// MQTT reconnect helper
inline bool mqttReconnect(const char* clientId, const char* user, const char* pass) {
  if (mqttClient().connected()) return true;
  
  uint32_t now = millis();
  if (now - gLastMqttAttempt < 5000) return false; // retry every 5s
  gLastMqttAttempt = now;
  
  Serial.print(F("[MQTT] Connecting... "));
  bool ok = false;
  if (user && user[0] && pass && pass[0]) {
    ok = mqttClient().connect(clientId, user, pass);
  } else {
    ok = mqttClient().connect(clientId);
  }
  
  if (ok) {
    Serial.println(F("connected"));
  } else {
    Serial.print(F("failed, rc="));
    Serial.println(mqttClient().state());
  }
  return ok;
}

// Format sensor data as JSON string per your spec
inline String formatSensorJson(const char* roomId, float t, float h, float hi, float co2) {
  auto nz = [](float v){ return isfinite(v) ? v : 0.0f; };
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"roomId\":\"%s\",\"timestamp\":%lu,\"temperature\":%.1f,\"humidity\":%.1f,\"heatIndex\":%.1f,\"co2\":%.0f}",
    roomId,
    millis(),
    nz(t), nz(h), nz(hi), nz(co2));
  return String(buf);
}

// Publish sensor data to MQTT if enabled and interval elapsed
inline bool publishMqtt(const char* clientId, const char* user, const char* pass) {
  // Skip if MQTT not configured
  if (!gMqttEnabled) return false;
  
  if (!mqttClient().connected()) {
    if (!mqttReconnect(clientId, user, pass)) return false;
  }
  
  uint32_t now = millis();
  if (now - gLastMqttPublish < gMqttInterval) return false; // respect interval
  
  String json = formatSensorJson(gRoomId, gT, gH, gHI, gCO2);
  bool ok = mqttClient().publish(gMqttTopic, json.c_str());
  if (ok) {
    gLastMqttPublish = now;
    Serial.print(F("[MQTT] Published: "));
    Serial.println(json);
  } else {
    Serial.println(F("[MQTT] Publish failed"));
  }
  return ok;
}

inline void handle() {
  server().handleClient();
  if (gMqttEnabled) {
    mqttClient().loop(); // MQTT keep-alive (only if enabled)
  }
}

// panggil setelah baca sensor
inline void update(float t, float h, float hi, float co2) {
  gT  = t;
  gH  = h;
  gHI = hi;
  gCO2 = co2;
  gTS = millis() / 1000;
}

// util: ambil IP aktif (STA jika ada, else AP)
inline IPAddress ip() {
  if (WiFi.getMode() & WIFI_MODE_STA && WiFi.status() == WL_CONNECTED)
    return WiFi.localIP();
  if (WiFi.getMode() & WIFI_MODE_AP)
    return WiFi.softAPIP();
  return IPAddress(0,0,0,0);
}

// Attempt to reconnect WiFi using the last provided Config
inline bool reconnect() {
  Serial.println(F("[NET] Reconnecting WiFi..."));
  if (!hasConfig()) {
    Serial.println(F("[NET] No previous config; call Net::begin() first"));
    return false;
  }

  const Config& cfg = lastConfig();

  auto ipSet = [](const IPAddress& ip){ return ip != IPAddress(0,0,0,0); };

  // Ensure STA mode and clean disconnect
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  // Apply static IP if provided (must be before WiFi.begin)
  if (ipSet(cfg.sta_ip) && ipSet(cfg.sta_gw) && ipSet(cfg.sta_sn)) {
    if (!WiFi.config(cfg.sta_ip, cfg.sta_gw, cfg.sta_sn,
                     ipSet(cfg.sta_dns1) ? cfg.sta_dns1 : IPAddress(0,0,0,0),
                     ipSet(cfg.sta_dns2) ? cfg.sta_dns2 : IPAddress(0,0,0,0))) {
      Serial.println(F("[NET] WiFi.config (STA) failed"));
    }
  }
  if (cfg.hostname && cfg.hostname[0]) {
    WiFi.setHostname(cfg.hostname);
  }
  WiFi.begin(cfg.ssid, cfg.pass);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < cfg.sta_timeout_ms) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[NET] STA IP: ")); Serial.println(WiFi.localIP());
    // Restart mDNS on reconnect
    if (cfg.hostname && cfg.hostname[0]) {
#if defined(ESP32)
      MDNS.end();
#endif
      delay(100);
      if (MDNS.begin(cfg.hostname)) {
        MDNS.addService("http", "tcp", 80);
        Serial.print(F("[NET] mDNS: http://")); Serial.print(cfg.hostname); Serial.println(F(".local/"));
      } else {
        Serial.println(F("[NET] mDNS restart failed"));
      }
    }
    return true;
  }

  // Optional AP fallback
  if (cfg.enable_ap_fallback) {
    Serial.println(F("[NET] STA reconnect failed, enabling SoftAP"));
    WiFi.mode(WIFI_AP);
    if (cfg.ap_ip != IPAddress(0,0,0,0) && cfg.ap_sn != IPAddress(0,0,0,0)) {
      if (!WiFi.softAPConfig(cfg.ap_ip, cfg.ap_gw, cfg.ap_sn)) {
        Serial.println(F("[NET] softAPConfig failed"));
      }
    }
    WiFi.softAP(cfg.ap_ssid, cfg.ap_pass);
#if defined(ESP32)
    if (cfg.hostname && cfg.hostname[0]) {
      WiFi.softAPsetHostname(cfg.hostname);
      MDNS.end();
      delay(100);
      if (MDNS.begin(cfg.hostname)) {
        MDNS.addService("http", "tcp", 80);
        Serial.print(F("[NET] mDNS(AP): http://")); Serial.print(cfg.hostname); Serial.println(F(".local/"));
      }
    }
#endif
    Serial.print(F("[NET] SoftAP IP: ")); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[NET] STA reconnect failed and AP fallback disabled"));
  }
  return false;
}

} 
