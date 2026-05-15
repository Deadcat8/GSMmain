/**
 * gsm_functions.h
 * GSM/GPRS transport for TeleAgriCulture Board
 *
 * Mirrors the WiFi transport pattern (setupXIfNeeded / handleXLoop)
 * and posts sensor data to the same API endpoint as wifi_sendData().
 *
 * Required libraries (add to platformio.ini lib_deps):
 *   vshymanskyy/TinyGSM @ ^0.11.7
 *   arduino-libraries/ArduinoHttpClient @ ^0.5.0
 *   bblanchon/ArduinoJson @ ^6          (already present)
 *
 * NOTE: SIM800 does not support TLS natively. This module uses HTTP.
 * For production HTTPS, wrap TinyGsmClient with SSLClient:
 *   https://github.com/OPEnSLab-OSU/SSLClient
 * and generate trust anchors for kits.teleagriculture.org.
 *
 * Pins (CON Serial Connector):
 *   GPIO 37 (U0TXD)  ->  SIM800X RX1  (ESP32 TX -> modem)
 *   GPIO 36 (U0RXD)  ->  SIM800X TX1  (ESP32 RX <- modem)
 *   GPIO 46 (pin 16) ->  SIM800X PWR  (power key, active LOW ~1 s)
 *   GPIO  3 (pin 15) ->  SIM800X DTR  (LOW = awake)
 */

#pragma once

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

// ── Pin definitions (CON Serial Connector) ──────────────────
#define GSM_RX_PIN   37   // U0TXD — ESP32 TX -> SIM800X RX1
#define GSM_TX_PIN   36   // U0RXD — ESP32 RX <- SIM800X TX1
#define GSM_PWR_PIN  46   // GPIO46 (pin 16) — power key
#define GSM_DTR_PIN   3   // GPIO3  (pin 15) — sleep control

#define GSM_BAUD     9600

// ── API endpoint (matches wifi_sendData) ─────────────────────
#define GSM_API_HOST "kits.teleagriculture.org"
#define GSM_API_PORT 80    // HTTP — see note above re: HTTPS
// Path is built at runtime using boardID from board_credentials.h

// ── Internal state ───────────────────────────────────────────
static HardwareSerial  _gsmSerial(1);         // UART1, explicit pins
static TinyGsm         _modem(_gsmSerial);
static TinyGsmClient   _gsmClient(_modem);

static bool _gsmInitialised = false;
static bool _gprsConnected  = false;

bool sendDataGSM = false;   // set true by updateTimeHandle() timer

// ── Helpers ──────────────────────────────────────────────────

static const char* _gsmRegStr(RegStatus s) {
  switch (s) {
    case REG_UNREGISTERED: return "NOT_REGISTERED";
    case REG_SEARCHING:    return "SEARCHING";
    case REG_DENIED:       return "DENIED";
    case REG_OK_HOME:      return "HOME";
    case REG_OK_ROAMING:   return "ROAMING";
    case REG_UNKNOWN:      return "UNKNOWN";
    default:               return "?";
  }
}

static void _gsmPowerOn() {
  if (GSM_DTR_PIN >= 0) {
    pinMode(GSM_DTR_PIN, OUTPUT);
    digitalWrite(GSM_DTR_PIN, LOW);   // keep modem awake
  }

  if (GSM_PWR_PIN < 0) return;
  pinMode(GSM_PWR_PIN, OUTPUT);
  digitalWrite(GSM_PWR_PIN, HIGH);
  delay(300);

  // Check if already on before pulsing — pulsing an on modem turns it OFF
  _gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_TX_PIN, GSM_RX_PIN);
  delay(500);
  if (_modem.testAT(800)) {
    Serial.println("[GSM] Already powered on");
    return;
  }

  Serial.println("[GSM] Pulsing PWR key...");
  digitalWrite(GSM_PWR_PIN, LOW);
  delay(1200);   // SIM800 datasheet: hold > 1 s
  digitalWrite(GSM_PWR_PIN, HIGH);
  delay(4000);   // allow boot
}

static bool _gsmInit() {
  Serial.println("[GSM] Initialising modem...");
  _gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_TX_PIN, GSM_RX_PIN);
  delay(2000);

  // AT handshake with retry
  Serial.print("[GSM] AT handshake");
  bool atOk = false;
  for (int i = 0; i < 12; i++) {
    if (_modem.testAT(1000)) { atOk = true; break; }
    Serial.print(".");
    delay(500);
  }
  if (!atOk) {
    Serial.println("\n[GSM] No AT response — check wiring and 5V supply");
    return false;
  }
  Serial.println(" OK");

  _modem.restart();
  delay(5000);   // allow radio to initialise fully

  Serial.print("[GSM] Model : "); Serial.println(_modem.getModemInfo());
  Serial.print("[GSM] CCID  : "); Serial.println(_modem.getSimCCID());
  Serial.print("[GSM] IMEI  : "); Serial.println(_modem.getIMEI());

  // Poll signal — 0/31 immediately after boot is normal
  Serial.print("[GSM] Signal");
  int csq = 0;
  for (int i = 0; i < 10; i++) {
    csq = _modem.getSignalQuality();
    if (csq != 0 && csq != 99) break;
    Serial.print(".");
    delay(2000);
  }
  Serial.print(" : ");
  Serial.println(csq == 99 ? "unknown" :
                 csq ==  0 ? "0/31 (no signal)" :
                             String(csq) + "/31");

  // Network registration — up to 90 s for cold SIM
  Serial.print("[GSM] Registering");
  bool registered = false;
  for (int i = 0; i < 30; i++) {
    SIM800RegStatus rs = (SIM800RegStatus)_modem.getRegistrationStatus();
    if (rs == REG_OK_HOME || rs == REG_OK_ROAMING) { registered = true; break; }
    Serial.print(" ["); Serial.print(_gsmRegStr(rs)); Serial.print("]");
    delay(3000);
  }
  Serial.println();

  if (!registered) {
    Serial.println("[GSM] Registration failed");
    return false;
  }

  Serial.print("[GSM] Registered on: ");
  Serial.println(_modem.getOperator());
  return true;
}

static bool _connectGPRS() {
  Serial.print("[GPRS] Connecting APN: ");
  Serial.println(apnName);   // apnName loaded from config via load_Config()

  if (!_modem.gprsConnect(apnName.c_str(), apnUser.c_str(), apnPass.c_str())) {
    Serial.println("[GPRS] Connection failed");
    return false;
  }

  Serial.print("[GPRS] Connected — IP: ");
  Serial.println(_modem.localIP());
  return true;
}

// ── Public API ───────────────────────────────────────────────

void setupGSMIfNeeded() {
  if (upload != "GSM") return;

  _gsmPowerOn();

  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("[GSM] Init attempt "); Serial.print(attempt); Serial.println("/3");
    if (_gsmInit()) { _gsmInitialised = true; break; }
    if (attempt < 3) delay(8000);
  }

  if (!_gsmInitialised) {
    Serial.println("[GSM] Failed to initialise — GSM transport disabled");
    return;
  }

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (_connectGPRS()) { _gprsConnected = true; break; }
    Serial.print("[GPRS] Retry "); Serial.println(attempt);
    delay(5000);
  }

  if (!_gprsConnected)
    Serial.println("[GPRS] Could not connect — will retry on first send");
}

void gsm_sendData() {
  if (!_gsmInitialised) return;

  // Reconnect GPRS if dropped
  if (!_modem.isGprsConnected()) {
    Serial.println("[GPRS] Reconnecting...");
    _gprsConnected = _connectGPRS();
    if (!_gprsConnected) return;
  }

  // ── Build JSON payload (mirrors wifi_sendData) ────────────
  size_t nItems = 0;
  for (size_t i = 0; i < sensorVector.size(); ++i) {
    const Sensor &s = sensorVector[i];
    int nTake = max(0, min(s.returnCount, (int)arrlen(s.measurements)));
    for (int j = 0; j < nTake; ++j) {
      const Measurement &m = s.measurements[j];
      if (m.data_name.length() && !isnan(m.value)) ++nItems;
    }
  }

  const size_t capacity = JSON_OBJECT_SIZE(nItems + 1) + nItems * 32 + 64;
  DynamicJsonDocument doc(capacity);

  for (size_t i = 0; i < sensorVector.size(); ++i) {
    const Sensor &s = sensorVector[i];
    int nTake = max(0, min(s.returnCount, (int)arrlen(s.measurements)));
    for (int j = 0; j < nTake; ++j) {
      const Measurement &m = s.measurements[j];
      if (m.data_name.length() && !isnan(m.value))
        doc[m.data_name] = round2f(static_cast<float>(m.value));
    }
  }

  char timeStr[64];
  getLocalTimeString(timeStr, sizeof(timeStr));
  if (timeStr[0] != '\0') doc["time"] = timeStr;

  String payload;
  payload.reserve(capacity);
  serializeJson(doc, payload);

  if (payload.length() <= 2) {
    Serial.println("[GSM] Empty payload — nothing to send");
    return;
  }

  // ── POST to API ───────────────────────────────────────────
  String path = "/api/kits/" + String(boardID) + "/measurements";
  String auth = "Bearer " + API_KEY;

  HttpClient http(_gsmClient, GSM_API_HOST, GSM_API_PORT);
  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Authorization", auth);
  http.sendHeader("Content-Length", payload.length());
  http.beginBody();
  http.print(payload);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  http.stop();

  if (statusCode > 0) {
    Serial.print("[GSM] POST "); Serial.print(statusCode); Serial.println(" OK");
  } else {
    Serial.print("[GSM] POST failed, code: "); Serial.println(statusCode);
  }
}

void handleGSMLoop() {
  if (!sendDataGSM) return;

  sensorRead();
  gsm_sendData();
  sendDataGSM = false;

  setUploadTime();

  if ((!useBattery || !gotoSleep) && useDisplay)
    renderPage(currentPage);

  // Deep sleep (mirrors handleWiFiLoop / handleLoraLoop pattern)
  if ((useBattery && gotoSleep) || (!useDisplay)) {
    // Power down modem before sleep to save current
    _modem.poweroff();
    _gsmSerial.end();

    wifiManager.disconnect();
    WiFi.mode(WIFI_OFF);

    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
    if (upload_interval == 60) {
      esp_sleep_enable_timer_wakeup((seconds_to_next_hour() - 15) * uS_TO_S_FACTOR);
      Serial.println("[GSM] Sleeping until next hour");
    } else {
      int time_interval = upload_interval * uS_TO_MIN_FACTOR;
      esp_sleep_enable_timer_wakeup(time_interval);
      Serial.print("[GSM] Sleeping for ");
      Serial.print(upload_interval);
      Serial.println(" minutes");
    }

    blinker.detach();
    Wire.end();
    tft->fillScreen(ST7735_BLACK);
    analogWrite(TFT_BL, 0);
    tft->enableSleep(true);
    spi_bus_free(SPI2_HOST);

    gpio_reset_pin((gpio_num_t)SW_3V3);
    gpio_reset_pin((gpio_num_t)SW_5V);
    gpio_reset_pin((gpio_num_t)TFT_BL);

    pinMode(SW_3V3, OUTPUT); digitalWrite(SW_3V3, LOW);
    pinMode(SW_5V,  OUTPUT); digitalWrite(SW_5V,  LOW);
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, LOW);

    gpio_hold_en((gpio_num_t)SW_3V3);
    gpio_hold_en((gpio_num_t)SW_5V);
    gpio_hold_en((gpio_num_t)TFT_BL);
    gpio_deep_sleep_hold_en();

    Serial.end();
    Serial.flush();
    delay(100);
    esp_deep_sleep_start();
  }
}
