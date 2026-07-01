/*
 ******************************************************************************
 * Soil Moisture Logger Firmware v1.0
 ******************************************************************************
 *
 *  TARGET BOARD : ESP32-S3-N16R8  (16 MB Flash, 8 MB OPI PSRAM)
 *
 *  PLATFORMIO / ARDUINO SETTINGS:
 *    Board            : "ESP32S3 Dev Module"
 *    Partition Scheme : "16M Flash (3MB APP OTA / 9.9MB FATFS)"
 *    PSRAM            : "OPI PSRAM"
 *    USB CDC On Boot  : "Enabled"
 *    Flash Size       : "16MB"
 *    Flash Mode       : "QIO 80MHz"
 *
 *  WIRING:
 *    Capacitive soil sensor SIG → MOISTURE_PIN (GPIO 10, ADC1)
 *    Capacitive soil sensor VCC → 3.3V
 *    Capacitive soil sensor GND → GND
 *
 *  HOW IT WORKS:
 *    The device logs soil moisture readings to one of four named chart slots.
 *    Before each recording session a 30-minute burn-off period runs to let
 *    the sensor stabilise after insertion into soil.  Session data is stored
 *    in PSRAM and kept until power-cycle.  Chart names persist in NVS.
 *
 *    Logger states:
 *      IDLE       → waiting for the user to start a recording
 *      BURN_OFF   → sensor inserted; 30-minute stabilisation countdown
 *      RECORDING  → reading soil moisture every READ_INTERVAL_SEC seconds (10 min)
 *
 *    Web UI sections (top → bottom):
 *      1. Instant Reading  — on-demand single reading with %, raw ADC
 *      2. Data Logger      — 4 named chart tabs, session history, start/stop/move
 *      3. Calibration      — dry / wet point setup
 *      4. Network          — WiFi info, change network, rename device
 *      5. OTA              — firmware update link
 *
 ******************************************************************************/

// ============================================================
//  FIRMWARE VERSION
// ============================================================
#define FIRMWARE_VERSION  "v1.0"

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define MOISTURE_PIN     10   // GPIO 10, ADC1 channel
#define MOISTURE_SAMPLES 16   // ADC readings averaged per sample

// ============================================================
//  AP PORTAL SETTINGS
// ============================================================
#define AP_SSID  "SoilLogger-Setup"
#define AP_PASS  ""             // open AP

// ============================================================
//  OTA CREDENTIALS
// ============================================================
#define OTA_USER  "admin"
#define OTA_PASS  "pw"

// ============================================================
//  TIMING
// ============================================================
#define BURN_OFF_SEC          600   // 10-minute stabilisation after sensor insertion
#define READ_INTERVAL_SEC     600   // 10-minute interval between logged readings
#define WIFI_RETRY_INTERVAL  30000UL

// ============================================================
//  CALIBRATION DEFAULTS
// ============================================================
#define DEFAULT_DEVICE_NAME  "SoilLogger"
#define DEFAULT_DRY_CAL      1489
#define DEFAULT_WET_CAL      935

// ============================================================
//  LOGGER STORAGE
// ============================================================
#define MAX_CHARTS        4    // chart slot count
#define MAX_SESSIONS      80   // total sessions across all charts (≈ 20 per chart)
#define MAX_READINGS      576  // readings per session (576 × 5 min = 48 hours)
#define MAX_SPARKLINE_PTS 100  // readings returned per session in chart/data API

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <time.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>

// ============================================================
//  LOGGER STATE MACHINE
// ============================================================
enum LogState { LOG_IDLE, LOG_BURN_OFF, LOG_RECORDING };
const char* logStateNames[] = { "IDLE", "BURN_OFF", "RECORDING" };

// ============================================================
//  DATA STRUCTURES  (sessions live in PSRAM)
// ============================================================
struct MoistureReading {
    uint32_t ts;   // Unix epoch
    int16_t  pct;  // moisture %
};

struct Session {
    bool     used;
    int8_t   chartIdx;
    bool     active;          // true while recording
    bool     inBurnOff;       // true during burn-off phase
    uint32_t burnOffStartMs;  // millis() when burn-off began
    uint32_t recordStartMs;   // millis() when actual recording began
    uint32_t startTs;         // Unix epoch at recording start
    uint32_t stopTs;          // Unix epoch at stop (0 = ongoing)
    int      count;
    MoistureReading readings[MAX_READINGS];
};

// ============================================================
//  OBJECTS
// ============================================================
WebServer   server(80);
Preferences prefs;

// ============================================================
//  RUNTIME STATE
// ============================================================
String deviceName;
int    dryCal;
int    wetCal;
bool   dryCalSet = false;
bool   wetCalSet = false;

char chartNames[MAX_CHARTS][32];

int    moistureRaw = 0;
int    moisturePct = 0;
bool   sensorReady = false;
String lastSensorTime = "Never";

unsigned long bootMillis    = 0;
unsigned long lastWiFiRetry = 0;

bool mdnsRunning = false;
bool inAPMode    = false;
bool otaRunning  = false;

String csrfToken;

// Logger
LogState      logState         = LOG_IDLE;
int           activeChartIdx   = -1;
int           activeSessionIdx = -1;
unsigned long burnOffStartMs   = 0;
unsigned long recordStartMs    = 0;
unsigned long lastReadingMs    = 0;

// Session pool — allocated from PSRAM in setup()
Session* sessions = nullptr;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
bool   connectWiFi();
void   startAPPortal();
void   startMDNS();
void   setupRoutes();
int    readMoistureRaw();
int    readMoistureMedian();
int    readMoistureStable();
int    moistureToPercent(int raw);
bool   moistureRawPlausible(int raw);
String sanitizeMDNS(const String &s);
String getTimeString();
String buildStatusJson();
bool   checkCsrf();
void   generateCsrfToken();
void   takeLogReading();

// ============================================================
//  NVS HELPERS
// ============================================================
void loadPrefs() {
    prefs.begin("system", true);
    deviceName = prefs.getString("name", DEFAULT_DEVICE_NAME);
    dryCal     = prefs.getInt("dry",    DEFAULT_DRY_CAL);
    wetCal     = prefs.getInt("wet",    DEFAULT_WET_CAL);
    dryCalSet  = prefs.getBool("drySet", false);
    wetCalSet  = prefs.getBool("wetSet", false);
    prefs.end();

    prefs.begin("charts", true);
    char key[4];
    for (int i = 0; i < MAX_CHARTS; i++) {
        snprintf(key, sizeof(key), "n%d", i);
        char def[16];
        snprintf(def, sizeof(def), "Chart %d", i + 1);
        String v = prefs.getString(key, def);
        strncpy(chartNames[i], v.c_str(), 31);
        chartNames[i][31] = '\0';
    }
    prefs.end();
}

void saveCalibration() {
    prefs.begin("system", false);
    prefs.putInt("dry",      dryCal);
    prefs.putInt("wet",      wetCal);
    prefs.putBool("drySet",  dryCalSet);
    prefs.putBool("wetSet",  wetCalSet);
    prefs.end();
}

void saveChartName(int id, const String &name) {
    char key[4];
    snprintf(key, sizeof(key), "n%d", id);
    prefs.begin("charts", false);
    prefs.putString(key, name);
    prefs.end();
    strncpy(chartNames[id], name.c_str(), 31);
    chartNames[id][31] = '\0';
}

void saveName(const String &name) {
    prefs.begin("system", false);
    prefs.putString("name", name);
    prefs.end();
}

void saveWiFiCreds(const String &ssid, const String &pass) {
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}

void loadWiFiCreds(String &ssid, String &pass) {
    prefs.begin("wifi", true);
    ssid = prefs.getString("ssid", "");
    pass = prefs.getString("pass", "");
    prefs.end();
}

void clearWiFiCreds() {
    prefs.begin("wifi", false);
    prefs.remove("ssid");
    prefs.remove("pass");
    prefs.end();
}

// ============================================================
//  CSRF TOKEN
// ============================================================
void generateCsrfToken() {
    csrfToken = "";
    for (int i = 0; i < 16; i++) csrfToken += String(esp_random() & 0xF, HEX);
}

bool checkCsrf() {
    String tok = server.hasArg("csrf") ? server.arg("csrf") : "";
    if (tok.isEmpty()) tok = server.header("X-CSRF-Token");
    if (tok != csrfToken) {
        server.send(403, "application/json", "{\"error\":\"Invalid or missing CSRF token\"}");
        return false;
    }
    return true;
}

// ============================================================
//  mDNS
// ============================================================
String sanitizeMDNS(const String &input) {
    String out = "";
    for (int i = 0; i < (int)input.length(); i++) {
        char c = tolower((unsigned char)input[i]);
        if (isalnum((unsigned char)c)) out += c;
        else if (c == '-' || c == ' ') out += '-';
    }
    while (out.startsWith("-")) out = out.substring(1);
    while (out.endsWith("-"))   out = out.substring(0, out.length() - 1);
    if (out.isEmpty())          out = DEFAULT_DEVICE_NAME;
    return out;
}

void startMDNS() {
    if (mdnsRunning) MDNS.end();
    String host = sanitizeMDNS(deviceName);
    if (MDNS.begin(host.c_str())) {
        MDNS.addService("http", "tcp", 80);
        mdnsRunning = true;
        Serial.printf("[mDNS] http://%s.local\n", host.c_str());
    } else {
        mdnsRunning = false;
        Serial.println("[mDNS] Failed");
    }
}

// ============================================================
//  NTP
// ============================================================
void setupTime() {
    configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
    Serial.println("[NTP] Sync requested");
}

String getTimeString() {
    time_t now = time(nullptr);
    if (now < 100000) return "Syncing...";
    struct tm t;
    localtime_r(&now, &t);
    char buf[32];
    strftime(buf, sizeof(buf), "%m/%d/%Y %I:%M:%S %p", &t);
    return String(buf);
}

// ============================================================
//  AP SETUP PORTAL
// ============================================================
void startAPPortal() {
    inAPMode = true;
    Serial.printf("[WiFi] Starting AP: %s\n", AP_SSID);
    WiFi.mode(WIFI_AP);
    if (strlen(AP_PASS) >= 8) WiFi.softAP(AP_SSID, AP_PASS);
    else                      WiFi.softAP(AP_SSID);
    Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Soil Logger WiFi Setup</title>
<style>
body{background:#111;color:#eee;font-family:Arial,sans-serif;display:flex;
  justify-content:center;align-items:center;min-height:100vh;margin:0;padding:1rem;box-sizing:border-box}
.box{background:#1e1e2e;padding:2rem;border-radius:1rem;width:100%;max-width:340px;
  text-align:center;box-shadow:0 4px 24px rgba(0,0,0,.5)}
h2{color:#4f8ef7;margin-bottom:1.5rem}
input{width:100%;padding:.6rem;margin:.4rem 0 1rem;border-radius:.5rem;border:1px solid #444;
  background:#2a2a3e;color:#fff;font-size:1rem;box-sizing:border-box}
button{width:100%;padding:.7rem;background:#4f8ef7;color:#fff;border:none;border-radius:.5rem;
  font-size:1rem;cursor:pointer;margin-bottom:.6rem}
button:hover{background:#3a7de0}
.scan-btn{background:#252840}.scan-btn:hover{background:#333658}
.note{font-size:.8rem;color:#888;margin-top:.8rem}
.net-item{display:flex;justify-content:space-between;align-items:center;padding:.4rem .6rem;
  border-radius:.4rem;cursor:pointer;margin-bottom:.3rem;background:#252840;text-align:left}
.net-item:hover{background:#333658}
.lock{font-size:.75rem;margin-left:.4rem;color:#f7b84f}
#netList{margin-bottom:.8rem;max-height:220px;overflow-y:auto}
#scanning{color:#8892a4;font-size:.85rem;margin:.5rem 0}
</style></head><body>
<div class="box">
  <h2>Soil Logger Setup</h2>
  <button class="scan-btn" onclick="doScan()">Scan for Networks</button>
  <div id="scanning" style="display:none">Scanning...</div>
  <div id="netList"></div>
  <input id="ssidInput" placeholder="Selected or type SSID" autocomplete="off">
  <input id="passInput" placeholder="Password" type="password" autocomplete="new-password">
  <button onclick="doConnect()">Connect and Save</button>
  <p class="note">Device will restart and connect to your network.</p>
</div>
<script>
function doScan(){
  var list=document.getElementById('netList'),sc=document.getElementById('scanning');
  list.innerHTML='';sc.style.display='block';
  fetch('/wifi/scan').then(function(r){return r.json();}).then(function(nets){
    sc.style.display='none';
    if(!nets.length){list.innerHTML='<p style="color:#8892a4;font-size:.85rem">No networks found</p>';return;}
    nets.forEach(function(n){
      var d=document.createElement('div');d.className='net-item';
      d.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:#8892a4">'+n.rssi+' dBm</span>';
      d.onclick=function(){document.getElementById('ssidInput').value=n.ssid;};
      list.appendChild(d);
    });
  }).catch(function(){sc.textContent='Scan failed';});
}
function doConnect(){
  var s=document.getElementById('ssidInput').value.trim();
  var p=document.getElementById('passInput').value;
  if(!s){alert('Enter or select an SSID first.');return;}
  var fd=new FormData();fd.append('ssid',s);fd.append('password',p);
  fetch('/save',{method:'POST',body:fd}).then(function(r){return r.text();})
    .then(function(t){document.body.innerHTML=t;});
}
doScan();
</script></body></html>
)HTML");
    });

    server.on("/save", HTTP_POST, []() {
        if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
            server.send(400, "text/plain", "SSID required");
            return;
        }
        String ssid = server.arg("ssid");
        String pass = server.hasArg("password") ? server.arg("password") : "";
        ssid.trim();
        saveWiFiCreds(ssid, pass);
        String host = sanitizeMDNS(deviceName);
        server.send(200, "text/html",
            String("<!DOCTYPE html><html><head>")
            + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            + "<style>body{background:#111;color:#eee;font-family:Arial;text-align:center;padding-top:3rem}</style>"
            + "</head><body><h2 style='color:#3ecf8e'>Credentials Saved!</h2>"
            + "<p>Restarting...</p><p style='color:#888;font-size:.9rem'>Reconnect to your WiFi then visit<br>"
            + "<strong>http://" + host + ".local</strong></p></body></html>");
        delay(1500);
        ESP.restart();
    });

    server.on("/wifi/scan", HTTP_GET, []() {
        int n = WiFi.scanNetworks();
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < n; i++) {
            JsonObject net = arr.add<JsonObject>();
            net["ssid"]   = WiFi.SSID(i);
            net["rssi"]   = WiFi.RSSI(i);
            net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        WiFi.scanDelete();
        String out; serializeJson(doc, out);
        server.send(200, "application/json", out);
    });

    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
}

// ============================================================
//  WiFi — STATION CONNECT
// ============================================================
bool connectWiFi() {
    String ssid, pass;
    loadWiFiCreds(ssid, pass);
    if (ssid.isEmpty()) { Serial.println("[WiFi] No saved credentials"); return false; }
    Serial.printf("[WiFi] Connecting to: %s\n", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) { delay(300); Serial.print("."); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        return true;
    }
    Serial.println("[WiFi] Connection failed");
    return false;
}

// ============================================================
//  SENSORS
// ============================================================
int readMoistureMedian() {
    // 64 samples, sort, average middle 50% to reject outliers
    int buf[64];
    for (int i = 0; i < 64; i++) { buf[i] = analogRead(MOISTURE_PIN); delay(3); }
    for (int i = 1; i < 64; i++) {
        int v = buf[i], j = i - 1;
        while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j--]; }
        buf[j + 1] = v;
    }
    long sum = 0;
    for (int i = 16; i < 48; i++) sum += buf[i];
    return (int)(sum / 32);
}

int readMoistureRaw() {
    int median = readMoistureMedian();

    // EMA across successive calls to smooth inter-reading noise
    static float ema = -1.0f;
    if (ema < 0.0f) ema = (float)median;
    else            ema = 0.25f * (float)median + 0.75f * ema;

    int result = (int)roundf(ema);
    Serial.printf("[Soil] Raw ADC: %d (median %d)\n", result, median);
    return result;
}

// Calibration needs a single trustworthy point, not a smoothed stream.
// WiFi radio activity injects noise bursts into ADC1 readings, so one
// 192ms sample burst can land entirely inside a burst and read way off.
// Take 5 independent medians spread over ~1s (spanning multiple WiFi
// radio cycles) and return their median to reject that.
int readMoistureStable() {
    int samples[5];
    for (int i = 0; i < 5; i++) {
        samples[i] = readMoistureMedian();
        if (i < 4) delay(200);
    }
    for (int i = 1; i < 5; i++) {
        int v = samples[i], j = i - 1;
        while (j >= 0 && samples[j] > v) { samples[j + 1] = samples[j--]; }
        samples[j + 1] = v;
    }
    return samples[2];
}

int moistureToPercent(int raw) {
    if (dryCal == wetCal) return 0;
    return constrain(map(raw, dryCal, wetCal, 0, 100), 0, 100);
}

bool moistureRawPlausible(int raw) { return raw > 50 && raw < 4090; }

// ============================================================
//  CALIBRATION
// ============================================================
int  calStatus()    { return (dryCalSet ? 1 : 0) + (wetCalSet ? 1 : 0); }
bool isCalibrated() { return dryCalSet && wetCalSet && (dryCal != wetCal); }

// ============================================================
//  LOGGER
// ============================================================
void takeLogReading() {
    moistureRaw = readMoistureRaw();
    moisturePct = moistureRawPlausible(moistureRaw) ? moistureToPercent(moistureRaw) : 0;
    lastSensorTime = getTimeString();
    sensorReady = true;

    if (activeSessionIdx < 0 || sessions[activeSessionIdx].count >= MAX_READINGS) return;
    int idx = sessions[activeSessionIdx].count;
    sessions[activeSessionIdx].readings[idx].ts  = (uint32_t)time(nullptr);
    sessions[activeSessionIdx].readings[idx].pct = (int16_t)moisturePct;
    sessions[activeSessionIdx].count++;
    Serial.printf("[Logger] Reading %d: %d%%\n", sessions[activeSessionIdx].count, moisturePct);
}

// Find an unused session slot; returns -1 if full
int findFreeSlot() {
    for (int i = 0; i < MAX_SESSIONS; i++) if (!sessions[i].used) return i;
    return -1;
}

// ============================================================
//  WiFi QUALITY
// ============================================================
String wifiQuality(int rssi) {
    if (!WiFi.isConnected()) return "Disconnected";
    if (rssi > -50) return "Excellent";
    if (rssi > -60) return "Good";
    if (rssi > -70) return "Fair";
    return "Weak";
}

// ============================================================
//  STATUS JSON
// ============================================================
String buildStatusJson() {
    int rssi = WiFi.isConnected() ? WiFi.RSSI() : 0;
    unsigned long uptimeSec = (millis() - bootMillis) / 1000UL;
    unsigned long nowMs = millis();

    JsonDocument doc;
    doc["firmware"]  = FIRMWARE_VERSION;
    doc["device"]    = deviceName;
    doc["mdns"]      = sanitizeMDNS(deviceName);
    doc["time"]      = getTimeString();
    doc["uptime"]    = uptimeSec;
    doc["csrf"]      = csrfToken;

    // Sensor
    bool rawOK = moistureRawPlausible(moistureRaw);
    doc["moisture"]   = moisturePct;
    doc["raw"]        = moistureRaw;
    doc["rawOk"]      = rawOK;
    doc["dryCal"]     = dryCal;
    doc["wetCal"]     = wetCal;
    doc["calibrated"] = isCalibrated();
    doc["calStatus"]  = calStatus();
    doc["dryCalSet"]  = dryCalSet;
    doc["wetCalSet"]  = wetCalSet;
    doc["lastRead"]   = lastSensorTime;

    // Logger state
    doc["logState"]    = logStateNames[logState];
    doc["activeChart"] = activeChartIdx;

    if (logState != LOG_IDLE && activeSessionIdx >= 0) {
        JsonObject as = doc["activeSession"].to<JsonObject>();
        as["globalIdx"]  = activeSessionIdx;
        as["chart"]      = activeChartIdx;
        as["inBurnOff"]  = sessions[activeSessionIdx].inBurnOff;
        as["count"]      = sessions[activeSessionIdx].count;

        if (logState == LOG_BURN_OFF) {
            long elapsed = (long)(nowMs - burnOffStartMs);
            long remain  = (long)(BURN_OFF_SEC * 1000L) - elapsed;
            as["burnOffRemain"]    = remain > 0 ? (int)(remain / 1000) : 0;
            as["recordingElapsed"] = 0;
        } else {
            as["burnOffRemain"]    = 0;
            long elapsed = (long)(nowMs - recordStartMs);
            as["recordingElapsed"] = (int)(elapsed / 1000);
        }

        // Last N readings for live sparkline
        JsonArray rdArr = as["recentReadings"].to<JsonArray>();
        int startIdx = max(0, sessions[activeSessionIdx].count - MAX_SPARKLINE_PTS);
        for (int i = startIdx; i < sessions[activeSessionIdx].count; i++) {
            JsonObject rd = rdArr.add<JsonObject>();
            rd["ts"]  = sessions[activeSessionIdx].readings[i].ts;
            rd["pct"] = sessions[activeSessionIdx].readings[i].pct;
        }
    }

    // Chart metadata (names + session counts)
    JsonArray charts = doc["charts"].to<JsonArray>();
    for (int c = 0; c < MAX_CHARTS; c++) {
        JsonObject ch = charts.add<JsonObject>();
        ch["id"]   = c;
        ch["name"] = chartNames[c];
        int cnt = 0;
        for (int i = 0; i < MAX_SESSIONS; i++) {
            if (sessions[i].used && sessions[i].chartIdx == c) cnt++;
        }
        ch["sessionCount"] = cnt;
    }

    // Network
    doc["ssid"]    = WiFi.isConnected() ? WiFi.SSID()              : "Not connected";
    doc["ip"]      = WiFi.isConnected() ? WiFi.localIP().toString() : "--";
    doc["mac"]     = WiFi.macAddress();
    doc["rssi"]    = rssi;
    doc["quality"] = wifiQuality(rssi);

    String out; serializeJson(doc, out);
    return out;
}

// ============================================================
//  ROUTE HANDLERS
// ============================================================
void handleStatus() { server.send(200, "application/json", buildStatusJson()); }
void handleHealth() { server.send(200, "text/plain", "OK"); }

void handleInstantReading() {
    if (!checkCsrf()) return;
    moistureRaw = readMoistureRaw();
    moisturePct = moistureRawPlausible(moistureRaw) ? moistureToPercent(moistureRaw) : 0;
    sensorReady    = true;
    lastSensorTime = getTimeString();
    JsonDocument doc;
    doc["moisture"] = moisturePct;
    doc["raw"]      = moistureRaw;
    doc["rawOk"]    = moistureRawPlausible(moistureRaw);
    doc["time"]     = lastSensorTime;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
}

void handleLogStart() {
    if (!checkCsrf()) return;
    if (logState != LOG_IDLE) {
        server.send(409, "application/json", "{\"error\":\"Already recording\"}");
        return;
    }
    if (!server.hasArg("chart")) {
        server.send(400, "application/json", "{\"error\":\"Missing chart parameter\"}");
        return;
    }
    int chartId = server.arg("chart").toInt();
    if (chartId < 0 || chartId >= MAX_CHARTS) {
        server.send(400, "application/json", "{\"error\":\"Invalid chart id\"}");
        return;
    }
    int slot = findFreeSlot();
    if (slot < 0) {
        server.send(507, "application/json", "{\"error\":\"Session storage full — delete old sessions\"}");
        return;
    }

    memset(&sessions[slot], 0, offsetof(Session, readings));
    sessions[slot].used           = true;
    sessions[slot].chartIdx       = (int8_t)chartId;
    sessions[slot].active         = true;
    sessions[slot].inBurnOff      = true;
    sessions[slot].burnOffStartMs = millis();
    sessions[slot].count          = 0;

    activeSessionIdx = slot;
    activeChartIdx   = chartId;
    burnOffStartMs   = millis();
    logState         = LOG_BURN_OFF;

    Serial.printf("[Logger] Burn-off started for chart %d (slot %d)\n", chartId, slot);
    server.send(200, "application/json",
        "{\"ok\":true,\"chart\":" + String(chartId) + ",\"state\":\"BURN_OFF\"}");
}

void handleLogStop() {
    if (!checkCsrf()) return;
    if (logState == LOG_IDLE) {
        server.send(400, "application/json", "{\"error\":\"Not recording\"}");
        return;
    }

    int prevSession = activeSessionIdx;
    int prevChart   = activeChartIdx;

    if (activeSessionIdx >= 0) {
        sessions[activeSessionIdx].active    = false;
        sessions[activeSessionIdx].inBurnOff = false;
        sessions[activeSessionIdx].stopTs    = (uint32_t)time(nullptr);
        if (sessions[activeSessionIdx].startTs == 0)
            sessions[activeSessionIdx].startTs = sessions[activeSessionIdx].stopTs;
    }

    logState         = LOG_IDLE;
    activeSessionIdx = -1;
    activeChartIdx   = -1;

    int cnt = prevSession >= 0 ? sessions[prevSession].count : 0;
    Serial.printf("[Logger] Stopped. Session %d: %d readings\n", prevSession, cnt);
    server.send(200, "application/json",
        "{\"ok\":true,\"chart\":" + String(prevChart)   +
        ",\"globalIdx\":"         + String(prevSession) +
        ",\"count\":"             + String(cnt) + "}");
}

void handleLogMove() {
    if (!checkCsrf()) return;
    if (!server.hasArg("globalIdx") || !server.hasArg("to")) {
        server.send(400, "application/json", "{\"error\":\"Missing globalIdx or to\"}");
        return;
    }
    int globalIdx = server.arg("globalIdx").toInt();
    int toChart   = server.arg("to").toInt();

    if (globalIdx < 0 || globalIdx >= MAX_SESSIONS || !sessions[globalIdx].used) {
        server.send(400, "application/json", "{\"error\":\"Invalid session\"}");
        return;
    }
    if (toChart < 0 || toChart >= MAX_CHARTS) {
        server.send(400, "application/json", "{\"error\":\"Invalid chart\"}");
        return;
    }
    if (sessions[globalIdx].active) {
        server.send(400, "application/json", "{\"error\":\"Cannot move active session\"}");
        return;
    }
    sessions[globalIdx].chartIdx = (int8_t)toChart;
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleLogDelete() {
    if (!checkCsrf()) return;
    if (!server.hasArg("globalIdx")) {
        server.send(400, "application/json", "{\"error\":\"Missing globalIdx\"}");
        return;
    }
    int globalIdx = server.arg("globalIdx").toInt();
    if (globalIdx < 0 || globalIdx >= MAX_SESSIONS || !sessions[globalIdx].used) {
        server.send(400, "application/json", "{\"error\":\"Invalid session\"}");
        return;
    }
    if (sessions[globalIdx].active) {
        server.send(400, "application/json", "{\"error\":\"Cannot delete active session\"}");
        return;
    }
    sessions[globalIdx].used = false;
    server.send(200, "application/json", "{\"ok\":true}");
}

// Returns session metadata + up to MAX_SPARKLINE_PTS readings per session for a chart
void handleChartData() {
    if (!server.hasArg("id")) {
        server.send(400, "application/json", "{\"error\":\"Missing id\"}");
        return;
    }
    int chartId = server.arg("id").toInt();
    if (chartId < 0 || chartId >= MAX_CHARTS) {
        server.send(400, "application/json", "{\"error\":\"Invalid chart id\"}");
        return;
    }

    JsonDocument doc;
    doc["chartId"] = chartId;
    doc["name"]    = chartNames[chartId];

    JsonArray sessArr = doc["sessions"].to<JsonArray>();
    int ordinal = 0;
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (!sessions[i].used || sessions[i].chartIdx != chartId) continue;
        JsonObject s = sessArr.add<JsonObject>();
        s["ordinal"]   = ordinal++;
        s["globalIdx"] = i;
        s["start"]     = sessions[i].startTs;
        s["stop"]      = sessions[i].stopTs;
        s["count"]     = sessions[i].count;
        s["active"]    = sessions[i].active;
        s["inBurnOff"] = sessions[i].inBurnOff;

        JsonArray rdArr = s["readings"].to<JsonArray>();
        // Evenly-sampled readings for sparkline (up to MAX_SPARKLINE_PTS)
        int cnt = sessions[i].count;
        if (cnt > 0) {
            int step = max(1, cnt / MAX_SPARKLINE_PTS);
            for (int j = 0; j < cnt; j += step) {
                JsonObject rd = rdArr.add<JsonObject>();
                rd["ts"]  = sessions[i].readings[j].ts;
                rd["pct"] = sessions[i].readings[j].pct;
            }
        }
    }

    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// Full CSV download for a session
void handleChartCsv() {
    if (!server.hasArg("chart") || !server.hasArg("session")) {
        server.send(400, "text/plain", "Missing chart or session");
        return;
    }
    int chartId = server.arg("chart").toInt();
    int ordinal = server.arg("session").toInt();
    if (chartId < 0 || chartId >= MAX_CHARTS || ordinal < 0) {
        server.send(400, "text/plain", "Invalid parameters");
        return;
    }

    // Find the Nth session for this chart
    int ord = 0;
    Session* s = nullptr;
    int gIdx = -1;
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (!sessions[i].used || sessions[i].chartIdx != chartId) continue;
        if (ord == ordinal) { s = &sessions[i]; gIdx = i; break; }
        ord++;
    }
    if (!s) { server.send(404, "text/plain", "Session not found"); return; }

    String csv = "timestamp,datetime,moisture_pct\r\n";
    for (int i = 0; i < s->count; i++) {
        csv += String(s->readings[i].ts);
        csv += ",";
        time_t t = (time_t)s->readings[i].ts;
        struct tm tm_info;
        localtime_r(&t, &tm_info);
        char buf[24];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
        csv += buf;
        csv += ",";
        csv += String(s->readings[i].pct);
        csv += "\r\n";
    }

    String filename = "chart" + String(chartId) + "_session" + String(ordinal) + ".csv";
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    server.send(200, "text/csv", csv);
}

void handleChartRename() {
    if (!checkCsrf()) return;
    if (!server.hasArg("id") || !server.hasArg("name") || server.arg("name").isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"Missing id or name\"}");
        return;
    }
    int id = server.arg("id").toInt();
    if (id < 0 || id >= MAX_CHARTS) {
        server.send(400, "application/json", "{\"error\":\"Invalid chart id\"}");
        return;
    }
    String name = server.arg("name"); name.trim();
    if (name.length() > 31) name = name.substring(0, 31);
    saveChartName(id, name);
    server.send(200, "application/json",
        "{\"ok\":true,\"id\":" + String(id) + ",\"name\":\"" + name + "\"}");
}

void handleCalDry() {
    if (!checkCsrf()) return;
    dryCal = readMoistureStable(); dryCalSet = true; saveCalibration();
    server.send(200, "application/json", "{\"ok\":true,\"dry\":" + String(dryCal) + "}");
}

void handleCalWet() {
    if (!checkCsrf()) return;
    wetCal = readMoistureStable(); wetCalSet = true; saveCalibration();
    server.send(200, "application/json", "{\"ok\":true,\"wet\":" + String(wetCal) + "}");
}

void handleCalReset() {
    if (!checkCsrf()) return;
    dryCal = DEFAULT_DRY_CAL; wetCal = DEFAULT_WET_CAL;
    dryCalSet = wetCalSet = false; saveCalibration();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleRename() {
    if (!checkCsrf()) return;
    if (!server.hasArg("name") || server.arg("name").isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"Missing name\"}");
        return;
    }
    String raw = server.arg("name"); raw.trim();
    deviceName = sanitizeMDNS(raw);
    saveName(deviceName);
    startMDNS();
    server.send(200, "application/json",
        "{\"ok\":true,\"name\":\"" + deviceName + "\"}");
}

void handleWiFiReset() {
    if (!checkCsrf()) return;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Rebooting into setup mode.\"}");
    clearWiFiCreds();
    delay(1000);
    ESP.restart();
}

void handleWiFiChange() {
    if (!checkCsrf()) return;
    if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
        server.send(400, "application/json", "{\"error\":\"Missing SSID\"}");
        return;
    }
    String ssid = server.arg("ssid");
    String pass = server.hasArg("pass") ? server.arg("pass") : "";
    saveWiFiCreds(ssid, pass);
    server.send(200, "application/json", "{\"ok\":true}");
    delay(800);
    ESP.restart();
}

void handleWiFiScanStart() {
    WiFi.scanNetworks(true);
    server.send(200, "application/json", "{\"ok\":true,\"status\":\"scanning\"}");
}

void handleWiFiScanResult() {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
        server.send(200, "application/json", "{\"status\":\"scanning\"}");
        return;
    }
    JsonDocument doc;
    if (n < 0) {
        doc["status"] = "error";
    } else {
        doc["status"] = "done";
        JsonArray arr = doc["networks"].to<JsonArray>();
        for (int i = 0; i < n; i++) {
            JsonObject net = arr.add<JsonObject>();
            net["ssid"]   = WiFi.SSID(i);
            net["rssi"]   = WiFi.RSSI(i);
            net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
}

// ============================================================
//  DASHBOARD HTML
// ============================================================
void handleRoot() {
server.send(200, "text/html", R"DASH(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Soil Logger</title>
<style>
:root{
  --bg:#0d0f1a;--card:#151724;--bdr:#252840;--acc:#4f8ef7;
  --grn:#3ecf8e;--red:#f76e6e;--warn:#f7b84f;--dim:#8892a4;--txt:#dde3f0;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',Arial,sans-serif;
     padding:1rem;max-width:620px;margin:auto}
h1{text-align:center;color:var(--acc);font-size:1.3rem;margin:.8rem 0 .3rem}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:14px;
      padding:1.2rem 1.4rem;margin-bottom:1rem}
.card-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:.9rem}
.card>h2,.card-hdr h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.1em;
  color:var(--dim);margin-bottom:.9rem}
.row{display:flex;justify-content:space-between;align-items:center;
     padding:.35rem 0;border-bottom:1px solid var(--bdr);font-size:.9rem}
.row:last-child{border-bottom:none}
.lbl{color:var(--dim);font-size:.88rem}.val{font-weight:600;font-size:.92rem}
.big{font-size:2.8rem;font-weight:700;color:var(--acc);text-align:center;padding:.5rem 0}
.badge{display:inline-block;padding:.2rem .75rem;border-radius:999px;
       font-size:.78rem;font-weight:600;background:#1e2535}
.badge.ok{color:var(--grn)}.badge.warn{color:var(--warn)}.badge.err{color:var(--red)}
.bar-wrap{background:#1e2535;border-radius:999px;height:10px;margin:.5rem 0}
.bar{height:10px;border-radius:999px;background:var(--acc);transition:width .5s;max-width:100%}
.btn-row{display:flex;gap:.6rem;flex-wrap:wrap;margin-top:.85rem}
button{padding:.5rem .95rem;border:none;border-radius:8px;font-size:.86rem;
       font-weight:600;cursor:pointer;transition:opacity .15s}
button:hover{opacity:.83}button:active{opacity:.65}
button:disabled{opacity:.4;cursor:default}
.b-blue{background:var(--acc);color:#fff}.b-grn{background:var(--grn);color:#111}
.b-red{background:var(--red);color:#fff}.b-dim{background:var(--bdr);color:var(--txt)}
.b-warn{background:var(--warn);color:#111}
.b-sm{padding:.3rem .7rem;font-size:.78rem}
.irow{display:flex;gap:.5rem;margin-top:.7rem}
input[type=text],input[type=password]{flex:1;padding:.5rem .8rem;background:#1e2535;
  border:1px solid var(--bdr);border-radius:8px;color:var(--txt);font-size:.9rem;outline:none}
input[type=text]:focus,input[type=password]:focus{border-color:var(--acc)}
.ota-link{display:block;text-align:center;color:var(--acc);text-decoration:none;
          font-size:.92rem;font-weight:600;padding:.4rem 0}
.last-read{font-size:.72rem;color:var(--dim);text-align:right;margin-top:.5rem}
#upd{text-align:center;font-size:.73rem;color:var(--dim);margin-top:.4rem}
.uptime{font-size:.75rem;color:var(--dim);text-align:center;margin-bottom:.2rem}
.net-list{margin-top:.7rem;max-height:180px;overflow-y:auto}
.net-item{display:flex;justify-content:space-between;align-items:center;
          padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;
          margin-bottom:.3rem;background:#1e2535}
.net-item:hover{background:var(--bdr)}
.lock{font-size:.72rem;margin-left:.3rem;color:var(--warn)}
#scanStatus{font-size:.82rem;color:var(--dim);margin:.4rem 0;text-align:center}
canvas{width:100%;height:80px;display:block;margin:.4rem 0}

/* ── Tabs ──────────────────────────────────── */
.tab-bar{display:flex;gap:.4rem;margin-bottom:1rem}
.tab-btn{flex:1;padding:.45rem .3rem;border:1px solid var(--bdr);border-radius:8px;
         background:var(--card);color:var(--dim);cursor:pointer;font-size:.78rem;
         font-weight:600;text-align:center;transition:all .15s;word-break:break-word;
         line-height:1.2}
.tab-btn.active{border-color:var(--acc);color:var(--acc);background:#1a2035}
.tab-btn.tab-recording{border-color:var(--grn);color:var(--grn)}
.tab-btn.tab-burnoff{border-color:var(--warn);color:var(--warn)}

/* ── Log state banner ────────────────────── */
.log-banner{border-radius:8px;padding:.55rem 1rem;font-size:.84rem;font-weight:600;
            text-align:center;margin-bottom:.8rem}
.log-idle{background:#1e1e2e;color:var(--dim);border:1px solid var(--bdr)}
.log-burnoff{background:#2a1e00;color:var(--warn);border:1px solid #5c3d00}
.log-recording{background:#0a2a0a;color:var(--grn);border:1px solid #1a5c1a}

/* ── Chart content header ────────────────── */
.chart-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:.7rem}
.chart-title{font-size:1.05rem;font-weight:700;color:var(--txt)}

/* ── Session items ───────────────────────── */
.session-list{margin-top:.8rem}
.session-item{background:#1e2535;border:1px solid var(--bdr);border-radius:10px;
              padding:.9rem 1rem;margin-bottom:.7rem}
.session-item.sess-active{border-color:var(--grn)}
.session-hdr{display:flex;align-items:center;flex-wrap:wrap;gap:.5rem;
             font-size:.84rem;margin-bottom:.3rem}
.live-badge{background:var(--grn);color:#111;padding:.1rem .5rem;
            border-radius:999px;font-size:.7rem;font-weight:700}
.burnoff-badge{background:var(--warn);color:#111;padding:.1rem .5rem;
               border-radius:999px;font-size:.7rem;font-weight:700}
.session-meta{font-size:.78rem;color:var(--dim)}
.session-actions{display:flex;gap:.4rem;flex-wrap:wrap;margin-top:.5rem;align-items:center}
.move-sel{background:#1e2535;border:1px solid var(--bdr);border-radius:6px;
          color:var(--txt);padding:.28rem .5rem;font-size:.78rem}
.no-sessions{color:var(--dim);font-size:.85rem;text-align:center;padding:1rem 0}
.warn-banner{background:#2a1e10;border:1px solid var(--warn);color:var(--warn);
  border-radius:8px;padding:.5rem .8rem;font-size:.82rem;margin-bottom:.6rem;display:none}
</style>
</head>
<body>

<h1 id="hTitle">Soil Moisture Logger</h1>
<div class="uptime" id="uptimeEl"></div>
<div class="uptime" id="versionEl"></div>

<!-- SENSOR DISCONNECT WARNING -->
<div class="warn-banner" id="sensorWarn">&#9888; Soil sensor reading out of range — check wiring</div>

<!-- ═══════════════════════════════════════════ -->
<!--  INSTANT READING                           -->
<!-- ═══════════════════════════════════════════ -->
<div class="card">
  <div class="card-hdr">
    <h2>Instant Reading</h2>
    <button class="b-blue b-sm" onclick="takeInstantReading()">&#x21BB; Take Reading</button>
  </div>
  <div class="big"><span id="moisture">--</span>%</div>
  <div class="bar-wrap"><div class="bar" id="bar" style="width:0%"></div></div>
  <div class="row"><span class="lbl">Raw ADC</span><span class="val" id="raw">--</span></div>
  <div class="row"><span class="lbl">Calibration</span><span id="instCalBadge" class="badge">--</span></div>
  <div class="last-read">Last read: <span id="lastRead">--</span></div>
</div>

<!-- ═══════════════════════════════════════════ -->
<!--  DATA LOGGER                               -->
<!-- ═══════════════════════════════════════════ -->
<div class="card">
  <h2>Data Logger</h2>
  <div id="logBanner" class="log-banner log-idle">Idle — select a chart and press Start Recording</div>
  <div class="tab-bar" id="tabBar"></div>
  <div id="chartContent">
    <div class="chart-hdr">
      <span class="chart-title" id="activeChartName">--</span>
      <button class="b-dim b-sm" onclick="renameActiveChart()">&#x270E; Rename</button>
    </div>
    <div id="logControls" class="btn-row"></div>
    <div class="session-list" id="sessionsList"></div>
  </div>
</div>

<!-- ═══════════════════════════════════════════ -->
<!--  CALIBRATION                               -->
<!-- ═══════════════════════════════════════════ -->
<div class="card">
  <h2>Calibration</h2>
  <div class="row"><span class="lbl">Raw ADC</span><span class="val" id="calRaw">--</span></div>
  <div class="row"><span class="lbl">Dry point</span><span class="val" id="dryCal">--</span></div>
  <div class="row"><span class="lbl">Wet point</span><span class="val" id="wetCal">--</span></div>
  <div class="row"><span class="lbl">Status</span><span id="calBadge" class="badge">--</span></div>
  <div class="btn-row">
    <button id="calDryBtn" class="b-red" onclick="calDry()">Set DRY point</button>
    <button id="calWetBtn" class="b-red" onclick="calWet()">Set WET point</button>
    <button class="b-red" style="margin-left:auto" onclick="calReset()">Reset</button>
  </div>
  <p style="color:var(--dim);font-size:.76rem;margin-top:.6rem">
    Remove sensor from soil and let it air-dry completely, then press <strong>Set DRY</strong>.
    Submerge fully in water, then press <strong>Set WET</strong>.
    Both points must be set for moisture % to display correctly.
  </p>
</div>

<!-- ═══════════════════════════════════════════ -->
<!--  NETWORK                                   -->
<!-- ═══════════════════════════════════════════ -->
<div class="card">
  <h2>Network</h2>
  <div class="row"><span class="lbl">SSID</span><span class="val" id="ssid">--</span></div>
  <div class="row"><span class="lbl">IP address</span><span class="val" id="ip">--</span></div>
  <div class="row"><span class="lbl">MAC address</span><span class="val" id="mac">--</span></div>
  <div class="row"><span class="lbl">RSSI</span><span class="val"><span id="rssi">--</span> dBm</span></div>
  <div class="row"><span class="lbl">Signal</span><span class="val" id="quality">--</span></div>
  <div class="row"><span class="lbl">mDNS hostname</span><span class="val" id="mdnsLabel">--</span></div>

  <h2 style="margin-top:1.1rem">Change WiFi Network</h2>
  <button class="b-dim" style="width:100%;margin-top:.5rem" onclick="doScan()">Scan for networks</button>
  <div id="scanStatus"></div>
  <div class="net-list" id="netList"></div>
  <div class="irow">
    <input type="text" id="wifiSSID" placeholder="Selected or type SSID" autocomplete="off">
    <input type="password" id="wifiPass" placeholder="Password" autocomplete="new-password"
           style="flex:1;padding:.5rem .8rem;background:#1e2535;border:1px solid var(--bdr);
                  border-radius:8px;color:var(--txt);font-size:.9rem;outline:none">
  </div>
  <div class="btn-row" style="margin-top:.7rem">
    <button class="b-blue" onclick="changeWiFi()">Connect to network</button>
  </div>

  <h2 style="margin-top:1.1rem">Change Device Name / mDNS</h2>
  <div class="irow">
    <input type="text" id="renameInput" placeholder="new-hostname">
    <button class="b-blue" onclick="renameDevice()">Save</button>
  </div>
  <p style="color:var(--dim);font-size:.76rem;margin-top:.5rem">
    Lowercase letters, numbers and hyphens only. mDNS updates immediately.
  </p>
  <div class="btn-row" style="margin-top:1rem">
    <button class="b-red" onclick="resetWiFi()">Reset WiFi credentials</button>
  </div>
</div>

<!-- ═══════════════════════════════════════════ -->
<!--  OTA                                       -->
<!-- ═══════════════════════════════════════════ -->
<div class="card">
  <h2>Firmware Update</h2>
  <a class="ota-link" href="/update">Open OTA update page &#x2197;</a>
</div>

<div id="upd">Connecting...</div>

<script>
var $=function(id){return document.getElementById(id);};
var csrf='';
var activeTab=0;
var logState='IDLE';
var activeChartIdx=-1;
var chartNames=['Chart 1','Chart 2','Chart 3','Chart 4'];
var currentChartData=null;
var lastActiveSession=null;
var pollInterval=5000;
var pollTimer=null;

// ── Utilities ───────────────────────────────────────────
function escHtml(s){
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

function toast(msg,ms){
  ms=ms||2500;
  var d=document.createElement('div');
  d.textContent=msg;
  d.style.cssText='position:fixed;bottom:1.4rem;left:50%;transform:translateX(-50%);'
    +'background:#252840;color:#dde3f0;padding:.55rem 1.1rem;border-radius:8px;'
    +'font-size:.86rem;z-index:9999;box-shadow:0 2px 12px rgba(0,0,0,.4);'
    +'opacity:1;transition:opacity .4s;pointer-events:none';
  document.body.appendChild(d);
  setTimeout(function(){d.style.opacity='0';setTimeout(function(){d.parentNode&&d.parentNode.removeChild(d);},500);},ms);
}

function api(url){
  var sep=url.indexOf('?')<0?'?':'&';
  return fetch(url+sep+'csrf='+encodeURIComponent(csrf));
}

function uptimeStr(s){
  var d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sec=s%60;
  if(d>0) return d+'d '+h+'h '+m+'m';
  if(h>0) return h+'h '+m+'m '+sec+'s';
  if(m>0) return m+'m '+sec+'s';
  return sec+'s';
}

function fmtDuration(sec){
  var h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;
  if(h>0) return h+'h '+m+'m';
  if(m>0) return m+'m '+s+'s';
  return s+'s';
}

function fmtDateTime(unix){
  if(!unix) return '--';
  var dt=new Date(unix*1000);
  return dt.toLocaleDateString()+' '+dt.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'});
}

function fmtHHMM(unix){
  var dt=new Date(unix*1000);
  var h=dt.getHours(),m=dt.getMinutes();
  return (h<10?'0':'')+h+':'+(m<10?'0':'')+m;
}

// ── Sparkline (same implementation as PlantMonitor) ─────
function drawSparkline(canvasId,points,color,unit,minVal,maxVal){
  var cv=document.getElementById(canvasId);
  if(!cv||!points||points.length<2) return;
  var dpr=window.devicePixelRatio||1;
  var LEFT=34,AXIS=18,CH=80;
  cv.width=cv.offsetWidth*dpr; cv.height=CH*dpr;
  var ctx=cv.getContext('2d'); ctx.scale(dpr,dpr);
  var W=cv.offsetWidth;
  var plotW=W-LEFT, plotH=CH-AXIS;

  var vals=points.map(function(p){return p.v;});
  var mn=minVal!=null?minVal:Math.min.apply(null,vals);
  var mx=maxVal!=null?maxVal:Math.max.apply(null,vals);
  if(mn===mx){mn-=1;mx+=1;}

  ctx.clearRect(0,0,W,CH);

  ctx.strokeStyle='rgba(255,255,255,0.06)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,0);ctx.lineTo(LEFT,plotH);ctx.stroke();

  ctx.fillStyle='rgba(136,146,164,0.9)';
  ctx.font='10px sans-serif';ctx.textAlign='right';
  var mid=(mx+mn)/2;
  function fmtY(v){return(mx-mn<20?v.toFixed(1):Math.round(v))+(unit||'');}
  ctx.fillText(fmtY(mx),LEFT-3,10);
  ctx.fillText(fmtY(mid),LEFT-3,plotH/2+4);
  ctx.fillText(fmtY(mn),LEFT-3,plotH-2);

  ctx.strokeStyle='rgba(255,255,255,0.04)';ctx.lineWidth=0.5;
  [0,0.5,1].forEach(function(frac){
    var y=plotH-(frac*(plotH-16))-4;
    ctx.beginPath();ctx.moveTo(LEFT,y);ctx.lineTo(W,y);ctx.stroke();
  });

  ctx.beginPath();
  points.forEach(function(p,i){
    var x=LEFT+i/(points.length-1)*plotW;
    var y=plotH-(p.v-mn)/(mx-mn)*(plotH-16)-4;
    i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
  });
  ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
  ctx.lineTo(LEFT+plotW,plotH);ctx.lineTo(LEFT,plotH);ctx.closePath();
  ctx.fillStyle=color.replace(')',',0.12)').replace('rgb','rgba');ctx.fill();

  ctx.strokeStyle='rgba(255,255,255,0.07)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,plotH);ctx.lineTo(W,plotH);ctx.stroke();

  if(!points[0]||!points[0].ts) return;
  ctx.fillStyle='rgba(136,146,164,0.9)';ctx.font='10px sans-serif';
  var numLabels=Math.min(5,points.length);
  for(var li=0;li<numLabels;li++){
    var idx=Math.round(li/(numLabels-1)*(points.length-1));
    var x=LEFT+idx/(points.length-1)*plotW;
    ctx.textAlign=li===0?'left':li===numLabels-1?'right':'center';
    ctx.fillText(fmtHHMM(points[idx].ts),x,CH-3);
  }
}

// ── Tabs ────────────────────────────────────────────────
function renderTabs(){
  var html='';
  for(var i=0;i<4;i++){
    var cls='tab-btn';
    if(i===activeTab) cls+=' active';
    if(logState!=='IDLE'&&activeChartIdx===i){
      cls+=(logState==='BURN_OFF')?' tab-burnoff':' tab-recording';
    }
    html+='<button class="'+cls+'" onclick="selectTab('+i+')">'
      +escHtml(chartNames[i])+'</button>';
  }
  $('tabBar').innerHTML=html;
}

function selectTab(id){
  activeTab=id;
  renderTabs();
  $('activeChartName').textContent=chartNames[id];
  loadChartData(id);
  renderLogControls();
}

// ── Log state banner ────────────────────────────────────
function renderLogBanner(d){
  var el=$('logBanner');
  el.className='log-banner';
  if(d.logState==='IDLE'){
    el.textContent='Idle — select a chart and press Start Recording';
    el.classList.add('log-idle');
  } else if(d.logState==='BURN_OFF'){
    var rem=(d.activeSession&&d.activeSession.burnOffRemain)||0;
    el.textContent='Burn-off in progress — sensor stabilising: '
      +fmtDuration(rem)+' remaining ('+escHtml(chartNames[d.activeChart])+')';
    el.classList.add('log-burnoff');
  } else {
    var as=d.activeSession||{};
    var elapsed=as.recordingElapsed||0;
    var cnt=as.count||0;
    el.textContent='Recording: '+escHtml(chartNames[d.activeChart])
      +' — '+fmtDuration(elapsed)+' elapsed, '+cnt+' readings';
    el.classList.add('log-recording');
  }
}

// ── Log controls ────────────────────────────────────────
function renderLogControls(){
  var html='';
  var onThis=(logState!=='IDLE'&&activeChartIdx===activeTab);
  var onOther=(logState!=='IDLE'&&activeChartIdx!==activeTab);

  if(logState==='IDLE'){
    html='<button class="b-blue" onclick="startRecording()">&#x25B6; Start Recording</button>';
  } else if(onThis){
    html='<button class="b-red" onclick="stopRecording()">&#x23F9; Stop Recording</button>';
  } else if(onOther){
    html='<button class="b-blue" disabled>&#x25B6; Start Recording</button>'
      +'<span class="lbl" style="font-size:.8rem;align-self:center">Active on: '
      +escHtml(chartNames[activeChartIdx])+'</span>';
  }
  $('logControls').innerHTML=html;
}

// ── Load chart session data ─────────────────────────────
function loadChartData(id){
  fetch('/chart/data?id='+id)
    .then(function(r){return r.json();})
    .then(function(data){
      currentChartData=data;
      renderSessions(data);
    })
    .catch(function(){$('sessionsList').innerHTML='<p class="no-sessions">Failed to load sessions</p>';});
}

function renderSessions(data){
  if(!data||!data.sessions||!data.sessions.length){
    $('sessionsList').innerHTML='<p class="no-sessions">No sessions yet. Press Start Recording to begin.</p>';
    return;
  }

  // Build other chart options for move dropdown
  var moveOpts='';
  for(var i=0;i<4;i++){
    if(i!==data.chartId){
      moveOpts+='<option value="'+i+'">'+escHtml(chartNames[i])+'</option>';
    }
  }

  // Sessions newest-first
  var sessions=data.sessions.slice().reverse();
  var html='';
  sessions.forEach(function(s){
    var isActive=s.active;
    html+='<div class="session-item'+(isActive?' sess-active':'')+'">';

    // Header row
    html+='<div class="session-hdr">';
    if(isActive&&s.inBurnOff){
      html+='<span class="burnoff-badge">BURN-OFF</span>';
    } else if(isActive){
      html+='<span class="live-badge">LIVE</span>';
    }
    if(s.start){
      html+='<span>'+fmtDateTime(s.start)+'</span>';
      if(s.stop) html+=' &rarr; <span>'+fmtDateTime(s.stop)+'</span>';
    } else {
      html+='<span>Recording...</span>';
    }
    html+='</div>';

    // Meta
    html+='<div class="session-meta">'+s.count+' readings';
    if(s.stop&&s.start) html+=' &middot; '+(Math.round((s.stop-s.start)/60))+' min';
    html+='</div>';

    // Sparkline canvas
    if(s.readings&&s.readings.length>1){
      html+='<canvas id="sc_'+s.globalIdx+'"></canvas>';
    }

    // Action buttons (completed sessions only)
    if(!isActive&&!s.inBurnOff){
      html+='<div class="session-actions">';
      html+='<a class="b-dim b-sm" style="padding:.3rem .7rem;font-size:.78rem;text-decoration:none;'
        +'border-radius:8px;font-weight:600;background:var(--bdr);color:var(--txt)"'
        +' href="/chart/csv?chart='+data.chartId+'&session='+s.ordinal+'" download>&#x21E9; CSV</a>';
      html+='<select class="move-sel" id="mv_'+s.globalIdx+'">'+moveOpts+'</select>';
      html+='<button class="b-dim b-sm" onclick="moveSession('+s.globalIdx+',\'mv_'+s.globalIdx+'\')">Move</button>';
      html+='<button class="b-red b-sm" onclick="deleteSession('+s.globalIdx+')">Delete</button>';
      html+='</div>';
    }
    html+='</div>';
  });

  $('sessionsList').innerHTML=html;

  // Draw sparklines after DOM update
  setTimeout(function(){
    sessions.forEach(function(s){
      if(s.readings&&s.readings.length>1){
        var pts=s.readings.map(function(r){return{v:r.pct,ts:r.ts};});
        var color=s.active?'rgb(62,207,142)':'rgb(79,142,247)';
        drawSparkline('sc_'+s.globalIdx,pts,color,'%',0,100);
      }
    });
  },80);
}

// Update live sparkline from status active session data
function updateLiveSparkline(as){
  if(!as||!as.recentReadings||as.recentReadings.length<2) return;
  var canvasId='sc_'+as.globalIdx;
  var cv=document.getElementById(canvasId);
  if(!cv) return;
  var pts=as.recentReadings.map(function(r){return{v:r.pct,ts:r.ts};});
  drawSparkline(canvasId,pts,'rgb(62,207,142)','%',0,100);
}

// ── Logger actions ──────────────────────────────────────
function startRecording(){
  api('/log/start?chart='+activeTab)
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){toast('Burn-off started — 30 min stabilisation');update();}
      else toast('Error: '+(d.error||'Unknown'));
    }).catch(function(){toast('Start failed');});
}

function stopRecording(){
  if(!confirm('Stop recording?')) return;
  api('/log/stop')
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){
        toast('Stopped. '+d.count+' readings saved.');
        update();
        loadChartData(activeTab);
      } else toast('Error: '+(d.error||'Unknown'));
    }).catch(function(){toast('Stop failed');});
}

function moveSession(globalIdx,selId){
  var toChart=parseInt(document.getElementById(selId).value);
  api('/log/move?globalIdx='+globalIdx+'&to='+toChart)
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){toast('Moved to '+chartNames[toChart]);loadChartData(activeTab);}
      else toast('Move failed: '+(d.error||'Unknown'));
    }).catch(function(){toast('Move failed');});
}

function deleteSession(globalIdx){
  if(!confirm('Delete this session? This cannot be undone.')) return;
  api('/log/delete?globalIdx='+globalIdx)
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){toast('Session deleted');loadChartData(activeTab);}
      else toast('Delete failed: '+(d.error||'Unknown'));
    }).catch(function(){toast('Delete failed');});
}

function renameActiveChart(){
  var name=prompt('Enter new name for this chart:',chartNames[activeTab]);
  if(!name||!name.trim()) return;
  api('/chart/rename?id='+activeTab+'&name='+encodeURIComponent(name.trim()))
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){
        chartNames[activeTab]=d.name;
        $('activeChartName').textContent=d.name;
        renderTabs();
        toast('Chart renamed to: '+d.name);
        update();
      } else toast('Rename failed');
    }).catch(function(){toast('Rename failed');});
}

// ── Instant reading ─────────────────────────────────────
function takeInstantReading(){
  $('lastRead').textContent='Reading...';
  api('/sensor/instant')
    .then(function(r){return r.json();})
    .then(function(d){
      $('moisture').textContent=d.moisture;
      $('bar').style.width=d.moisture+'%';
      $('raw').textContent=d.raw+(d.rawOk===false?' ⚠':'');
      $('lastRead').textContent=d.time;
      $('sensorWarn').style.display=d.rawOk===false?'block':'none';
    }).catch(function(){toast('Reading failed');});
}

// ── Calibration ──────────────────────────────────────────
function calDry(){
  api('/cal/dry').then(function(r){return r.json();})
    .then(function(d){toast('Dry point saved: ADC '+d.dry);update();});
}
function calWet(){
  api('/cal/wet').then(function(r){return r.json();})
    .then(function(d){toast('Wet point saved: ADC '+d.wet);update();});
}
function calReset(){
  if(!confirm('Reset calibration to factory defaults?')) return;
  api('/cal/reset').then(function(){toast('Calibration reset');update();});
}

// ── WiFi ────────────────────────────────────────────────
var scanTimer=null;
function signalBars(r){return r>-50?'||||':r>-60?'||| ':r>-70?'||  ':'|   ';}

function doScan(){
  $('scanStatus').textContent='Starting scan...';$('netList').innerHTML='';
  fetch('/wifi/scan/start').then(function(){
    $('scanStatus').textContent='Scanning... (5-10 s)';
    clearInterval(scanTimer);
    scanTimer=setInterval(pollScan,2000);
  }).catch(function(){$('scanStatus').textContent='Scan request failed';});
}

function pollScan(){
  fetch('/wifi/scan/result').then(function(r){return r.json();}).then(function(d){
    if(d.status==='scanning') return;
    clearInterval(scanTimer);
    if(d.status==='error'||!d.networks){$('scanStatus').textContent='Scan failed';return;}
    $('scanStatus').textContent=d.networks.length+' network'+(d.networks.length!==1?'s':'')+' found';
    d.networks.forEach(function(n){
      var div=document.createElement('div');div.className='net-item';
      div.innerHTML='<span>'+escHtml(n.ssid)+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:var(--dim)">'+signalBars(n.rssi)+' '+n.rssi+' dBm</span>';
      div.onclick=function(){$('wifiSSID').value=n.ssid;$('wifiPass').focus();};
      $('netList').appendChild(div);
    });
  }).catch(function(){clearInterval(scanTimer);$('scanStatus').textContent='Poll failed';});
}

function changeWiFi(){
  var ssid=$('wifiSSID').value.trim(),pass=$('wifiPass').value;
  if(!ssid){toast('Enter or select a network first');return;}
  if(!confirm('Connect to "'+ssid+'"? Device will reboot.')) return;
  var fd=new FormData();fd.append('ssid',ssid);fd.append('pass',pass);
  fetch('/wifi/change',{method:'POST',body:fd}).then(function(){toast('Saved. Rebooting...',4000);});
}

function renameDevice(){
  var name=$('renameInput').value.trim();
  if(!name){toast('Enter a name first');return;}
  api('/device/rename?name='+encodeURIComponent(name))
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){toast('Renamed to: '+d.name);$('renameInput').value='';update();}
      else toast('Error: '+(d.error||'Unknown'));
    });
}

function resetWiFi(){
  if(!confirm('Clear WiFi credentials and reboot into setup mode?')) return;
  api('/wifi/reset').then(function(){toast('Rebooting into setup mode...',4000);});
}

// ── Poll rate ────────────────────────────────────────────
function setPollInterval(ms){
  if(ms===pollInterval) return;
  pollInterval=ms;
  clearInterval(pollTimer);
  pollTimer=setInterval(update,pollInterval);
}

// ── Apply status ─────────────────────────────────────────
function applyStatus(d){
  csrf=d.csrf||csrf;
  $('hTitle').textContent=(d.device||'Soil Logger')+' Logger';
  document.title=(d.device||'Soil Logger')+' Logger';
  $('uptimeEl').textContent='Uptime: '+uptimeStr(d.uptime||0);
  $('versionEl').textContent='Firmware: '+d.firmware;

  // Sensor warning
  $('sensorWarn').style.display=d.rawOk===false?'block':'none';

  // Instant reading section
  if(d.moisture!=null){
    $('moisture').textContent=d.moisture;
    $('bar').style.width=d.moisture+'%';
    $('raw').textContent=d.raw+(d.rawOk===false?' ⚠':'');
    $('lastRead').textContent=d.lastRead||'--';
  }

  // Calibration badge
  var instBadge=$('instCalBadge');
  var calB=$('calBadge');
  if(d.calStatus===2){
    instBadge.textContent=calB.textContent='Calibrated';
    instBadge.className=calB.className='badge ok';
  } else if(d.calStatus===1){
    var lbl=d.dryCalSet?'Wet point needed':'Dry point needed';
    instBadge.textContent=calB.textContent=lbl;
    instBadge.className=calB.className='badge warn';
  } else {
    instBadge.textContent=calB.textContent='Not calibrated';
    instBadge.className=calB.className='badge warn';
  }
  $('calRaw').textContent=d.raw;
  $('dryCal').textContent=d.dryCal;
  $('wetCal').textContent=d.wetCal;
  $('calDryBtn').className=d.dryCalSet?'b-grn':'b-red';
  $('calWetBtn').className=d.wetCalSet?'b-grn':'b-red';

  // Chart names from status
  if(d.charts){
    d.charts.forEach(function(c){chartNames[c.id]=c.name;});
  }

  // Logger state
  var prevLogState=logState;
  var prevChart=activeChartIdx;
  logState=d.logState||'IDLE';
  activeChartIdx=d.activeChart!=null?d.activeChart:-1;

  renderTabs();
  renderLogBanner(d);

  // If log state changed, refresh controls and reload chart data
  if(prevLogState!==logState||prevChart!==activeChartIdx){
    renderLogControls();
    loadChartData(activeTab);
  }

  // Update live sparkline if recording on active tab
  if(logState==='RECORDING'&&activeChartIdx===activeTab&&d.activeSession){
    updateLiveSparkline(d.activeSession);
    // Also update count/elapsed in session list header if present
    var liveEl=document.querySelector('.sess-active .session-meta');
    if(liveEl){
      var as=d.activeSession;
      liveEl.textContent=(as.count||0)+' readings · '+fmtDuration(as.recordingElapsed||0)+' elapsed';
    }
  }

  // Network
  $('ssid').textContent=d.ssid;
  $('ip').textContent=d.ip;
  $('mac').textContent=d.mac;
  $('rssi').textContent=d.rssi;
  $('quality').textContent=d.quality;
  $('mdnsLabel').textContent=(d.mdns||'--')+'.local';
  $('activeChartName').textContent=chartNames[activeTab];

  // Poll rate: fast during burn-off/recording, normal otherwise
  setPollInterval(logState==='IDLE'?15000:5000);
  $('upd').textContent='Updated: '+new Date().toLocaleTimeString();
}

// ── Main poll ────────────────────────────────────────────
function update(){
  fetch('/status')
    .then(function(r){return r.json();})
    .then(function(d){applyStatus(d);})
    .catch(function(){$('upd').textContent='Connection lost — retrying...';});
}

// ── Init ─────────────────────────────────────────────────
pollTimer=setInterval(update,pollInterval);
update();
setTimeout(function(){loadChartData(activeTab);},800);

// ── Live ADC feed for calibration section ────────────────
setInterval(function(){
  api('/sensor/instant')
    .then(function(r){return r.json();})
    .then(function(d){$('calRaw').textContent=d.raw;})
    .catch(function(){});
},2000);
</script>
</body>
</html>
)DASH");
}

// ============================================================
//  ROUTE REGISTRATION
// ============================================================
void setupRoutes() {
    server.on("/",                  HTTP_GET,  handleRoot);
    server.on("/status",            HTTP_GET,  handleStatus);
    server.on("/health",            HTTP_GET,  handleHealth);
    server.on("/sensor/instant",    HTTP_GET,  handleInstantReading);
    server.on("/log/start",         HTTP_GET,  handleLogStart);
    server.on("/log/stop",          HTTP_GET,  handleLogStop);
    server.on("/log/move",          HTTP_GET,  handleLogMove);
    server.on("/log/delete",        HTTP_GET,  handleLogDelete);
    server.on("/chart/data",        HTTP_GET,  handleChartData);
    server.on("/chart/csv",         HTTP_GET,  handleChartCsv);
    server.on("/chart/rename",      HTTP_GET,  handleChartRename);
    server.on("/cal/dry",           HTTP_GET,  handleCalDry);
    server.on("/cal/wet",           HTTP_GET,  handleCalWet);
    server.on("/cal/reset",         HTTP_GET,  handleCalReset);
    server.on("/device/rename",     HTTP_GET,  handleRename);
    server.on("/wifi/scan/start",   HTTP_GET,  handleWiFiScanStart);
    server.on("/wifi/scan/result",  HTTP_GET,  handleWiFiScanResult);
    server.on("/wifi/change",       HTTP_POST, handleWiFiChange);
    server.on("/wifi/reset",        HTTP_GET,  handleWiFiReset);
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// ============================================================
//  ArduinoOTA
// ============================================================
void setupArduinoOTA() {
    ArduinoOTA.setHostname(sanitizeMDNS(deviceName).c_str());
    ArduinoOTA.onStart([]() {
        esp_task_wdt_delete(NULL);
        Serial.println("[ArduinoOTA] Start");
    });
    ArduinoOTA.onEnd([]()  { Serial.println("\n[ArduinoOTA] Done"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        if (t > 0) Serial.printf("[ArduinoOTA] %u%%\r", p * 100 / t);
    });
    ArduinoOTA.onError([](ota_error_t err) {
        const char* msgs[] = {"","Auth failed","Begin failed","Connect failed","Receive failed","End failed"};
        Serial.printf("[ArduinoOTA] Error[%u]: %s\n", err, err <= 5 ? msgs[err] : "Unknown");
    });
    ArduinoOTA.begin();
    otaRunning = true;
    Serial.println("[ArduinoOTA] Ready on port 3232");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] ESP32-S3-N16R8 Soil Moisture Logger " FIRMWARE_VERSION);
    bootMillis = millis();

    // ── Hardware watchdog (30 s) ───────────────────────────
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms    = 30000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt_cfg);
    esp_task_wdt_add(NULL);
    Serial.println("[WDT] Enabled — 30 s timeout");

    // ── ADC ───────────────────────────────────────────────
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // ── PSRAM session pool ─────────────────────────────────
    size_t poolSize = sizeof(Session) * MAX_SESSIONS;
    if (psramFound()) {
        sessions = (Session*)ps_malloc(poolSize);
        Serial.printf("[PSRAM] Allocated %zu KB for session pool\n", poolSize / 1024);
    } else {
        sessions = (Session*)malloc(poolSize);
        Serial.println("[WARN] PSRAM not found — using regular heap (limited capacity)");
    }
    if (!sessions) {
        Serial.println("[FATAL] Session pool allocation failed — halting");
        while (true) delay(1000);
    }
    memset(sessions, 0, poolSize);

    // ── NVS ───────────────────────────────────────────────
    loadPrefs();

    // ── CSRF ──────────────────────────────────────────────
    generateCsrfToken();

    // ── WiFi ──────────────────────────────────────────────
    String ssid, pass;
    loadWiFiCreds(ssid, pass);
    bool connected = !ssid.isEmpty() && connectWiFi();

    if (!connected) {
        Serial.println(ssid.isEmpty()
            ? "[WiFi] No credentials — starting AP portal"
            : "[WiFi] Could not connect — starting AP portal");
        startAPPortal();
        return;
    }

    // ── NTP ───────────────────────────────────────────────
    setupTime();

    // ── Modem sleep (keep WiFi responsive, save power) ────
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    // ── mDNS ──────────────────────────────────────────────
    startMDNS();

    // ── ArduinoOTA ────────────────────────────────────────
    setupArduinoOTA();

    // ── Web server ────────────────────────────────────────
    setupRoutes();
    ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
    server.begin();
    Serial.println("[HTTP] Server started");

    // ── Take an initial reading ───────────────────────────
    moistureRaw = readMoistureRaw();
    moisturePct = moistureRawPlausible(moistureRaw) ? moistureToPercent(moistureRaw) : 0;
    sensorReady    = true;
    lastSensorTime = getTimeString();
    Serial.printf("[Soil] Initial reading: %d%% (raw %d)\n", moisturePct, moistureRaw);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    server.handleClient();
    ElegantOTA.loop();
    if (otaRunning) ArduinoOTA.handle();
    esp_task_wdt_reset();

    // ── WiFi reconnect watchdog ────────────────────────────
    if (!inAPMode
        && WiFi.getMode() == WIFI_STA
        && WiFi.status() != WL_CONNECTED
        && millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL)
    {
        lastWiFiRetry = millis();
        Serial.println("[WiFi] Reconnecting...");
        String ssid, pass;
        loadWiFiCreds(ssid, pass);
        WiFi.begin(ssid.c_str(), pass.c_str());
    }

    // ── Logger state machine ───────────────────────────────
    unsigned long nowMs = millis();

    if (logState == LOG_BURN_OFF) {
        unsigned long elapsed = nowMs - burnOffStartMs;
        if (elapsed >= (unsigned long)BURN_OFF_SEC * 1000UL) {
            logState      = LOG_RECORDING;
            recordStartMs = nowMs;
            sessions[activeSessionIdx].recordStartMs = nowMs;
            sessions[activeSessionIdx].startTs       = (uint32_t)time(nullptr);
            sessions[activeSessionIdx].inBurnOff     = false;
            // Take the first reading immediately
            takeLogReading();
            lastReadingMs = nowMs;
            Serial.println("[Logger] Burn-off complete — recording started");
        }
    } else if (logState == LOG_RECORDING) {
        if (nowMs - lastReadingMs >= (unsigned long)READ_INTERVAL_SEC * 1000UL) {
            takeLogReading();
            lastReadingMs = nowMs;
        }
    }

    delay(50);
}
