// Distance traffic-light indicator + boom gate - ESP32 + 6x HC-SR04 (2 bays
// of 3) + 1x RGB LED + 1x BMP180 pressure plate + 1x SG90 boom gate servo
//
// Sensor layout per bay - 3 HC-SR04 all aimed into the SAME bay, from
// different positions, so a vehicle sitting in that bay blocks all three at
// once (this is what makes "all 3 blocked" a reasonable stand-in for "bay
// occupied", per the pitch's 3-sensor fusion idea):
//   TOP   - mounted at the back wall of the bay, facing forward into it.
//   LEFT  - mounted on the bay's left-hand boundary, facing across it.
//   RIGHT - mounted on the bay's right-hand boundary, facing across it.
// Bay 1 = sensorTop/sensorLeft/sensorRight. Bay 2 = bay2Top/bay2Left/bay2Right,
// same arrangement, duplicated for a second physical bay.
//
// All 6 sensors share ONE echo pin (ECHO_SHARED_PIN) to free up GPIOs for
// other hardware. This is safe only because every sensor is triggered and
// read one at a time, in sequence - never two at once - so their echo
// pulses never overlap on the shared line.
//
// Per-sensor rule (same for every sensor, both bays): <= 40cm = "blocked", > 40cm = "clear".
//
// LED rule (same for both bays, each with its own LED and its own overstay
// timer): green by default. When ALL THREE of that bay's sensors are blocked
// at once, it goes red - and if that holds continuously for OVERSTAY_MS
// (10s), it escalates to yellow. The moment any of the 3 clears, it drops
// straight back to green immediately.
// Each bay has its own buzzer, sounding only during that bay's own overstay
// (yellow) - Bay 1's buzzer never sounds for Bay 2's overstay and vice versa.
//
// Gate rule: two plates now - "plate" (entry) and "plateExit" (exit), each on
// its own I2C bus (BMP180 has a fixed address, so two can't share one bus).
// Pressing EITHER plate opens the gate. Once that same plate is released
// (vehicle has moved past it), the gate holds open for GATE_HOLD_MS (6s)
// then closes. This is independent of the bay logic above - EXCEPT for
// capacity lockout, below.
//
// Capacity lockout: when BOTH bays are full (occupied or overstay - i.e.
// neither is green), the entry plate no longer opens the gate at all - a
// press is logged as refused instead. The exit plate is never affected by
// this, since a full lot should still let cars leave.
//
// gateLed: solid RED whenever both bays are full (this takes priority - it's
// a standing status, not a blip). Otherwise, it flashes red for the whole
// time the gate is open - from the moment a plate press triggers it open,
// through the post-clear hold, until the gate actually closes.

#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <time.h>

#include "secrets.h"
#include "cloud_config.h"

// cloud_config.h must be included before this Blynk header.
#include <BlynkSimpleEsp32_SSL.h>

// GPIO pins - must match diagram.json's wiring
const uint8_t ECHO_SHARED_PIN = 36; // VP - shared by all 6 sensors, see note above

// Bay 1
const uint8_t TOP_TRIG_PIN   = 33;
const uint8_t LEFT_TRIG_PIN  = 25;
const uint8_t RIGHT_TRIG_PIN = 27;

// Bay 2 (same top/left/right arrangement, duplicated)
const uint8_t BAY2_TOP_TRIG_PIN   = 15;
const uint8_t BAY2_LEFT_TRIG_PIN  = 2;
const uint8_t BAY2_RIGHT_TRIG_PIN = 4;

const uint8_t LED_R = 12;
const uint8_t LED_G = 13;
const uint8_t LED_B = 14;

// Bay 2's LED only has Red+Green wired (2 GPIOs were all that were left in
// the pin budget) - enough for green/red, but no yellow/overstay for Bay 2
// unless a pin is freed up later.
const uint8_t LED2_R = 17;
const uint8_t LED2_G = 23;

const uint8_t PLATE_SDA_PIN = 21;
const uint8_t PLATE_SCL_PIN = 22;
const uint8_t BMP180_ADDR   = 0x77;

// Exit plate - separate I2C bus (Wire1), since BMP180's address is fixed
// and two can't share one bus.
const uint8_t PLATE2_SDA_PIN = 26;
const uint8_t PLATE2_SCL_PIN = 32;
TwoWire ExitWire = TwoWire(1);

const uint8_t GATE_SERVO_PIN = 18;
const uint8_t GATE_LED_PIN   = 16; // RX2 - flashes while either plate is pressed
const uint32_t GATE_LED_FLASH_MS = 200;
bool gateLedState = false;
uint32_t gateLedLastToggle = 0;

const uint8_t  BUZZER_PIN      = 19;
const uint8_t  BUZZER2_PIN     = 5;
const uint16_t BUZZER_FREQ_HZ  = 2000; // audible alarm pitch
bool buzzerOn  = false; // tracks current state so we only log on a change, not every cycle
bool buzzer2On = false;

const float NEAR_CM = 40.0; // at/below this distance a sensor counts as "blocked"

const uint32_t READ_INTERVAL_MS = 150;
uint32_t lastReadAt = 0;

// --- overstay tuning (each bay tracks its own independently) ---
const uint32_t OVERSTAY_MS = 10000; // how long all-blocked must hold before it's an "overstay"
uint32_t blockedSince = 0;          // Bay 1: millis() when all-blocked started; 0 = not currently blocked
uint32_t bay2BlockedSince = 0;      // Bay 2: same, independent timer
bool bay1OverstayActive = false;     // exported to cloud telemetry
bool bay2OverstayActive = false;     // exported to cloud telemetry

// --- pressure plate tuning (entry) ---
const int16_t  PRESSURE_DELTA_THRESHOLD = 120; // raw ADC counts above baseline = "pressed"
const uint32_t PLATE_DEBOUNCE_MS        = 300; // must be stable this long before it counts
int16_t plateBaseline = 0;
bool plateCandidateActive = false;
uint32_t plateCandidateSince = 0;
bool plateActive = false;
int16_t plateLastDelta = 0; // for the Serial debug print - how close to PRESSURE_DELTA_THRESHOLD right now

// --- pressure plate tuning (exit) - same thresholds, independent state ---
int16_t plateExitBaseline = 0;
bool plateExitCandidateActive = false;
uint32_t plateExitCandidateSince = 0;
bool plateExitActive = false;
int16_t plateExitLastDelta = 0;

// --- gate FSM ---
const int      GATE_CLOSED_ANGLE = 0;
const int      GATE_OPEN_ANGLE   = 90;
const uint32_t GATE_HOLD_MS      = 6000; // how long the gate stays open once the plate clears
Servo gateServo;
bool gateOpen = false;
bool gateForExit = false;  // which plate opened it, so we know which one to wait on
uint32_t gateHoldUntil = 0; // 0 = not yet counting down

// --- capacity lockout ---
bool bothBaysFull = false;       // set fresh each loop, before serviceGate() runs
bool entryRefusedLatched = false; // avoids spamming the refusal log every cycle while held

// Fires one HC-SR04 (given its own TRIG pin) and returns distance in cm, or
// -1 if nothing echoed back within range. Always reads on ECHO_SHARED_PIN -
// safe because callers only ever trigger one sensor at a time. Same math as
// before: sound takes ~58us per cm of round trip.
float readDistanceCm(uint8_t trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t durationUs = pulseIn(ECHO_SHARED_PIN, HIGH, 25000UL); // ~430cm max range
  if (durationUs == 0) return -1;
  return durationUs / 58.0;
}

// A sensor only counts as "blocked" if it actually got a valid echo AND that
// echo is within NEAR_CM. A timeout (-1, nothing in range) is never blocked.
bool isBlocked(float distanceCm) {
  return distanceCm >= 0 && distanceCm <= NEAR_CM;
}

// Drives the 3 LED legs directly - true = that color's GPIO pin goes HIGH.
void setColor(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

// Bay 2's LED - only Red/Green available (see LED2_R/LED2_G above).
void setColor2(bool r, bool g) {
  digitalWrite(LED2_R, r);
  digitalWrite(LED2_G, g);
}

// A plain digitalWrite(HIGH) holds the buzzer's diaphragm steady - silent,
// since it's simulated as a bare piezo speaker rather than a self-oscillating
// "active" buzzer chip. tone()/noTone() actually drives it at an audible
// frequency, which is what makes it produce sound in the simulator.
void setBuzzer(bool on) {
  if (on == buzzerOn) return; // only act (and log) on an actual change
  buzzerOn = on;
  if (on) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    Serial.println("[BUZZER1] ON");
  } else {
    noTone(BUZZER_PIN);
    Serial.println("[BUZZER1] OFF");
  }
}

// Bay 2's buzzer - same tone()/noTone() approach as Bay 1's, own pin.
void setBuzzer2(bool on) {
  if (on == buzzer2On) return;
  buzzer2On = on;
  if (on) {
    tone(BUZZER2_PIN, BUZZER_FREQ_HZ);
    Serial.println("[BUZZER2] ON");
  } else {
    noTone(BUZZER2_PIN);
    Serial.println("[BUZZER2] OFF");
  }
}

// --- BMP180 minimal raw-pressure driver ---
// We only need a relative change to detect "something is pressing the
// plate", not a calibrated absolute pressure, so this skips the full
// temperature-compensation sequence and just reads the raw pressure
// register directly.
int16_t readRawPressure(TwoWire &bus) {
  bus.beginTransmission(BMP180_ADDR);
  bus.write(0xF4);
  bus.write(0x34); // start pressure measurement, OSS = 0
  bus.endTransmission();
  delay(5);

  bus.beginTransmission(BMP180_ADDR);
  bus.write(0xF6);
  bus.endTransmission(false);
  bus.requestFrom(BMP180_ADDR, (uint8_t)2);
  if (bus.available() < 2) return 0;
  uint8_t msb = bus.read();
  uint8_t lsb = bus.read();
  return (int16_t)((msb << 8) | lsb);
}

// Updates plateActive with the same debounce pattern used elsewhere: a
// candidate state must hold steady for PLATE_DEBOUNCE_MS before it commits.
void servicePlate() {
  int16_t raw = readRawPressure(Wire);
  int16_t delta = raw - plateBaseline;
  plateLastDelta = delta;
  bool rawActive = delta > PRESSURE_DELTA_THRESHOLD;

  if (rawActive != plateCandidateActive) {
    plateCandidateActive = rawActive;
    plateCandidateSince = millis();
  }
  if ((millis() - plateCandidateSince) >= PLATE_DEBOUNCE_MS) {
    plateActive = plateCandidateActive;
  }
}

// Exit plate - identical logic to servicePlate(), own bus/state.
void servicePlateExit() {
  int16_t raw = readRawPressure(ExitWire);
  int16_t delta = raw - plateExitBaseline;
  plateExitLastDelta = delta;
  bool rawActive = delta > PRESSURE_DELTA_THRESHOLD;

  if (rawActive != plateExitCandidateActive) {
    plateExitCandidateActive = rawActive;
    plateExitCandidateSince = millis();
  }
  if ((millis() - plateExitCandidateSince) >= PLATE_DEBOUNCE_MS) {
    plateExitActive = plateExitCandidateActive;
  }
}

// Opens the gate when EITHER plate is pressed; once that same plate clears,
// holds open for GATE_HOLD_MS before closing again. If both were pressed at
// once, entry takes priority (an edge case, not something either plate's
// normal use should trigger). Entry is refused outright while bothBaysFull -
// exit is never blocked by capacity.
void serviceGate() {
  uint32_t now = millis();

  if (!gateOpen) {
    if (plateActive) {
      if (!bothBaysFull) {
        gateServo.write(GATE_OPEN_ANGLE);
        gateOpen = true;
        gateForExit = false;
        gateHoldUntil = 0;
        Serial.println("[GATE] Opening (entry)");
      } else if (!entryRefusedLatched) {
        entryRefusedLatched = true;
        Serial.println("[GATE] Entry refused - capacity FULL");
      }
    } else {
      entryRefusedLatched = false; // plate released, re-arm for the next attempt
      if (plateExitActive) {
        gateServo.write(GATE_OPEN_ANGLE);
        gateOpen = true;
        gateForExit = true;
        gateHoldUntil = 0;
        Serial.println("[GATE] Opening (exit)");
      }
    }
    return;
  }

  // Gate is open: start the hold timer once the relevant plate is released,
  // then close once that hold time has passed.
  bool relevantActive = gateForExit ? plateExitActive : plateActive;
  if (!relevantActive && gateHoldUntil == 0) {
    gateHoldUntil = now + GATE_HOLD_MS;
  }
  if (gateHoldUntil != 0 && now >= gateHoldUntil) {
    gateServo.write(GATE_CLOSED_ANGLE);
    gateOpen = false;
    gateHoldUntil = 0;
    Serial.println("[GATE] Closing");
  }
}

// Solid red whenever both bays are full (a standing status, takes priority).
// Otherwise, flashes (toggles every GATE_LED_FLASH_MS) for the entire time
// the gate is open - triggered by a plate press, continues through the
// post-clear hold, stops the moment the gate actually closes.
void serviceGateLed() {
  if (bothBaysFull) {
    if (!gateLedState) {
      gateLedState = true;
      digitalWrite(GATE_LED_PIN, HIGH);
    }
    return;
  }

  if (!gateOpen) {
    if (gateLedState) {
      gateLedState = false;
      digitalWrite(GATE_LED_PIN, LOW);
    }
    return;
  }

  uint32_t now = millis();
  if (now - gateLedLastToggle >= GATE_LED_FLASH_MS) {
    gateLedLastToggle = now;
    gateLedState = !gateLedState;
    digitalWrite(GATE_LED_PIN, gateLedState);
  }
}


// ============================================================================
// SNAY'S CLOUD INTEGRATION MODULE
// ============================================================================
//
// Proven communication path from the cloud-module prototype:
//   - MQTT over verified TLS -> Blynk V0-V7
//   - HTTPS over verified TLS -> Blynk V8-V14
//   - Event-driven updates for important state changes
//   - 30 second full-state heartbeat for re-synchronisation/history
//   - Local parking control remains authoritative if cloud connectivity fails
//
// IMPORTANT ROLE BOUNDARY:
// Dat/Aasman's local controller decides occupancy, gate behaviour, overstay,
// capacity, and faults.  This cloud layer only reports those decisions.
// ============================================================================

struct ParkingTelemetry {
  bool bay1Occupied = false;          // V0
  bool bay2Occupied = false;          // V1
  int availableSpaces = 2;            // V2
  int gateState = 0;                  // V3: 0 closed, 1 entry, 2 exit, 3 wait-close, 4 locked-full
  uint32_t bay1DurationSec = 0;        // V4
  uint32_t bay2DurationSec = 0;        // V5
  bool entryPlate = false;             // V6
  bool exitPlate = false;              // V7
  bool fullCapacity = false;           // V8
  bool bay1Overstay = false;           // V9
  bool bay2Overstay = false;           // V10
  bool sensorFault = false;            // V11 (reserved for Aasman's fault detector)
  int wifiRSSI = -120;                 // V12
  String systemStatus = "OFFLINE";     // V13
  String alertMessage = "NONE";        // V14
};

ParkingTelemetry telemetry;

struct MqttEventState {
  bool bay1Occupied = false;
  bool bay2Occupied = false;
  int availableSpaces = 2;
  int gateState = 0;
  bool entryPlate = false;
  bool exitPlate = false;
};

struct HttpsEventState {
  bool fullCapacity = false;
  bool bay1Overstay = false;
  bool bay2Overstay = false;
  bool sensorFault = false;
  String systemStatus = "OFFLINE";
  String alertMessage = "NONE";
};

MqttEventState lastMqttState;
HttpsEventState lastHttpsState;
bool mqttStateValid = false;
bool httpsStateValid = false;

WiFiClientSecure mqttTlsClient;
WiFiClientSecure httpsTlsClient;
PubSubClient mqttClient(mqttTlsClient);

bool wifiWasConnected = false;
bool timeSyncStarted = false;
bool timeIsValid = false;
uint32_t lastWiFiRetry = 0;
uint32_t lastMQTTRetry = 0;
uint32_t lastCloudPublish = 0;

const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
const uint32_t MQTT_RETRY_INTERVAL_MS = 10000;
const uint32_t CLOUD_HEARTBEAT_INTERVAL_MS = 30000;

String urlEncode(const String &value) {
  String encoded = "";
  const char hex[] = "0123456789ABCDEF";

  for (size_t i = 0; i < value.length(); ++i) {
    unsigned char c = static_cast<unsigned char>(value.charAt(i));
    if ((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += static_cast<char>(c);
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }
  return encoded;
}

void startWiFi() {
  Serial.println("[CLOUD] Starting Wi-Fi...");
  WiFi.mode(WIFI_STA);

  // Wokwi-GUEST uses channel 6.  For real hardware, change secrets.h.
  if (String(WIFI_SSID) == "Wokwi-GUEST") {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    telemetry.wifiRSSI = WiFi.RSSI();
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      telemetry.systemStatus = "DEGRADED"; // MQTT may still be connecting
      Serial.printf("[CLOUD] Wi-Fi connected | IP %s | RSSI %d dBm\n",
                    WiFi.localIP().toString().c_str(), telemetry.wifiRSSI);
    }
    return;
  }

  telemetry.wifiRSSI = -120;
  telemetry.systemStatus = "OFFLINE";

  if (wifiWasConnected) {
    wifiWasConnected = false;
    if (mqttClient.connected()) mqttClient.disconnect();
    timeSyncStarted = false;
    timeIsValid = false;
    mqttStateValid = false;
    httpsStateValid = false;
    Serial.println("[CLOUD] Wi-Fi lost. Local parking control continues.");
  }

  uint32_t now = millis();
  if (now - lastWiFiRetry >= WIFI_RETRY_INTERVAL_MS) {
    lastWiFiRetry = now;
    Serial.println("[CLOUD] Retrying Wi-Fi...");
    WiFi.disconnect();
    if (String(WIFI_SSID) == "Wokwi-GUEST") {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
    } else {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

void maintainTimeSync() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!timeSyncStarted) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    timeSyncStarted = true;
    Serial.println("[CLOUD] NTP time sync started...");
  }

  if (timeIsValid) return;
  time_t now = time(nullptr);
  if (now > 1700000000) {
    timeIsValid = true;
    Serial.println("[CLOUD] System time synchronised for TLS validation.");
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("[CLOUD] MQTT RX [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; ++i) Serial.print((char)payload[i]);
  Serial.println();
}

void publishDeviceInfo() {
  if (!mqttClient.connected()) return;

  JsonDocument doc;
  doc["tmpl"] = BLYNK_TEMPLATE_ID;
  doc["ver"] = FIRMWARE_VERSION;
  doc["build"] = String(__DATE__) + " " + String(__TIME__);
  doc["type"] = BLYNK_TEMPLATE_ID;
  doc["rxbuff"] = 1024;

  char payload[256];
  if (serializeJson(doc, payload, sizeof(payload)) == 0) return;
  mqttClient.publish("info/mcu", payload);
}

bool publishMqttTelemetry() {
  if (!mqttClient.connected()) return false;

  JsonDocument doc;
  doc["Bay1Occupied"] = telemetry.bay1Occupied ? 1 : 0;
  doc["Bay2Occupied"] = telemetry.bay2Occupied ? 1 : 0;
  doc["AvailableSpaces"] = telemetry.availableSpaces;
  doc["GateState"] = telemetry.gateState;
  doc["Bay1DurationSec"] = telemetry.bay1DurationSec;
  doc["Bay2DurationSec"] = telemetry.bay2DurationSec;
  doc["EntryPlate"] = telemetry.entryPlate ? 1 : 0;
  doc["ExitPlate"] = telemetry.exitPlate ? 1 : 0;

  char payload[512];
  if (serializeJson(doc, payload, sizeof(payload)) == 0) return false;

  bool ok = mqttClient.publish("batch_ds", payload);
  if (!ok) Serial.println("[CLOUD] MQTT V0-V7 publish failed.");
  return ok;
}

bool publishHttpsTelemetry() {
  if (WiFi.status() != WL_CONNECTED || !timeIsValid) return false;

  String url = "https://" + String(BLYNK_MQTT_HOST) +
               "/external/api/batch/update?token=" + urlEncode(String(BLYNK_AUTH_TOKEN));

  url += "&v8="  + String(telemetry.fullCapacity ? 1 : 0);
  url += "&v9="  + String(telemetry.bay1Overstay ? 1 : 0);
  url += "&v10=" + String(telemetry.bay2Overstay ? 1 : 0);
  url += "&v11=" + String(telemetry.sensorFault ? 1 : 0);
  url += "&v12=" + String(telemetry.wifiRSSI);
  url += "&v13=" + urlEncode(telemetry.systemStatus);
  url += "&v14=" + urlEncode(telemetry.alertMessage);

  HTTPClient http;
  // Keep cloud failures from stalling the local controller for a long time.
  http.setTimeout(1500);

  if (!http.begin(httpsTlsClient, url)) {
    Serial.println("[CLOUD] HTTPS setup failed.");
    return false;
  }

  int code = http.GET();
  bool ok = code >= 200 && code < 300;
  if (!ok) {
    Serial.printf("[CLOUD] HTTPS V8-V14 failed, code %d\n", code);
    String body = http.getString();
    if (body.length()) Serial.println(body);
  }
  http.end();
  return ok;
}

bool mqttEventChanged() {
  if (!mqttStateValid) return true;
  return telemetry.bay1Occupied != lastMqttState.bay1Occupied ||
         telemetry.bay2Occupied != lastMqttState.bay2Occupied ||
         telemetry.availableSpaces != lastMqttState.availableSpaces ||
         telemetry.gateState != lastMqttState.gateState ||
         telemetry.entryPlate != lastMqttState.entryPlate ||
         telemetry.exitPlate != lastMqttState.exitPlate;
}

bool httpsEventChanged() {
  if (!httpsStateValid) return true;
  return telemetry.fullCapacity != lastHttpsState.fullCapacity ||
         telemetry.bay1Overstay != lastHttpsState.bay1Overstay ||
         telemetry.bay2Overstay != lastHttpsState.bay2Overstay ||
         telemetry.sensorFault != lastHttpsState.sensorFault ||
         telemetry.systemStatus != lastHttpsState.systemStatus ||
         telemetry.alertMessage != lastHttpsState.alertMessage;
}

void rememberMqttState() {
  lastMqttState.bay1Occupied = telemetry.bay1Occupied;
  lastMqttState.bay2Occupied = telemetry.bay2Occupied;
  lastMqttState.availableSpaces = telemetry.availableSpaces;
  lastMqttState.gateState = telemetry.gateState;
  lastMqttState.entryPlate = telemetry.entryPlate;
  lastMqttState.exitPlate = telemetry.exitPlate;
  mqttStateValid = true;
}

void rememberHttpsState() {
  lastHttpsState.fullCapacity = telemetry.fullCapacity;
  lastHttpsState.bay1Overstay = telemetry.bay1Overstay;
  lastHttpsState.bay2Overstay = telemetry.bay2Overstay;
  lastHttpsState.sensorFault = telemetry.sensorFault;
  lastHttpsState.systemStatus = telemetry.systemStatus;
  lastHttpsState.alertMessage = telemetry.alertMessage;
  httpsStateValid = true;
}

void publishCloudSnapshot() {
  bool mqttOK = publishMqttTelemetry();
  if (mqttOK) rememberMqttState();

  if (mqttClient.connected()) mqttClient.loop();

  bool httpsOK = publishHttpsTelemetry();
  if (httpsOK) rememberHttpsState();

  Serial.printf("[CLOUD] Heartbeat -> MQTT:%s HTTPS:%s\n",
                mqttOK ? "OK" : "FAIL", httpsOK ? "OK" : "FAIL");
}

void setupCloudClients() {
  mqttTlsClient.setCACert(BLYNK_DEFAULT_ROOT_CA);
  httpsTlsClient.setCACert(BLYNK_DEFAULT_ROOT_CA);
  mqttTlsClient.setHandshakeTimeout(10);
  httpsTlsClient.setHandshakeTimeout(10);

  mqttClient.setServer(BLYNK_MQTT_HOST, BLYNK_MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(45);
  mqttClient.setBufferSize(1024);
  mqttClient.setSocketTimeout(3);

  startWiFi();
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED || !timeIsValid || mqttClient.connected()) return;

  uint32_t now = millis();
  if (lastMQTTRetry != 0 && now - lastMQTTRetry < MQTT_RETRY_INTERVAL_MS) return;
  lastMQTTRetry = now;

  Serial.printf("[CLOUD] Connecting MQTT -> %s:%u\n", BLYNK_MQTT_HOST, BLYNK_MQTT_PORT);

  bool connected = mqttClient.connect(
      "smart-parking-integrated",
      "device",
      BLYNK_AUTH_TOKEN);

  if (connected) {
    telemetry.systemStatus = "ONLINE";
    Serial.println("[CLOUD] MQTT connected.");
    mqttClient.subscribe("downlink/#");
    publishDeviceInfo();
    mqttStateValid = false;
    httpsStateValid = false;
    publishCloudSnapshot();
    lastCloudPublish = millis();
  } else {
    telemetry.systemStatus = "DEGRADED";
    Serial.printf("[CLOUD] MQTT connect failed, state=%d. Local control continues.\n",
                  mqttClient.state());
  }
}

void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED || !timeIsValid) return;

  if (!mqttClient.connected()) {
    telemetry.systemStatus = "DEGRADED";
    connectMQTT();
    return;
  }

  telemetry.systemStatus = "ONLINE";
  mqttClient.loop();
}

void publishChangedTelemetry() {
  if (WiFi.status() != WL_CONNECTED || !timeIsValid) return;

  if (mqttClient.connected() && mqttEventChanged()) {
    if (publishMqttTelemetry()) {
      rememberMqttState();
      Serial.println("[CLOUD] Immediate MQTT state update sent.");
    }
  }

  if (httpsEventChanged()) {
    if (publishHttpsTelemetry()) {
      rememberHttpsState();
      Serial.println("[CLOUD] Immediate HTTPS state update sent.");
    }
  }
}

void maintainCloudHeartbeat() {
  if (WiFi.status() != WL_CONNECTED || !timeIsValid) return;
  uint32_t now = millis();
  if (now - lastCloudPublish >= CLOUD_HEARTBEAT_INTERVAL_MS) {
    lastCloudPublish = now;
    publishCloudSnapshot();
  }
}

// Maps Dat/Aasman's authoritative controller state into Snay's cloud contract.
void syncTelemetryFromController(bool bay1Occupied,
                                 bool bay2Occupied,
                                 bool bay1Overstay,
                                 bool bay2Overstay) {
  telemetry.bay1Occupied = bay1Occupied;
  telemetry.bay2Occupied = bay2Occupied;
  telemetry.availableSpaces = 2 - (bay1Occupied ? 1 : 0) - (bay2Occupied ? 1 : 0);
  telemetry.fullCapacity = bothBaysFull;
  telemetry.entryPlate = plateActive;
  telemetry.exitPlate = plateExitActive;
  telemetry.bay1Overstay = bay1Overstay;
  telemetry.bay2Overstay = bay2Overstay;

  telemetry.bay1DurationSec =
      (bay1Occupied && blockedSince != 0) ? (millis() - blockedSince) / 1000UL : 0;
  telemetry.bay2DurationSec =
      (bay2Occupied && bay2BlockedSince != 0) ? (millis() - bay2BlockedSince) / 1000UL : 0;

  // Gate state mapping used by Blynk V3.
  if (gateOpen) {
    if (gateHoldUntil != 0) telemetry.gateState = 3;          // WAITING_TO_CLOSE
    else telemetry.gateState = gateForExit ? 2 : 1;          // OPEN_EXIT / OPEN_ENTRY
  } else if (bothBaysFull) {
    telemetry.gateState = 4;                                  // LOCKED_FULL
  } else {
    telemetry.gateState = 0;                                  // CLOSED
  }

  // Dat's current code has no authoritative fault detector yet.
  // Keep false until Aasman's fault-detection output is wired here.
  telemetry.sensorFault = false;

  if (telemetry.sensorFault) {
    telemetry.alertMessage = "SENSOR FAULT";
  } else if (entryRefusedLatched) {
    telemetry.alertMessage = "ENTRY REFUSED - FULL";
  } else if (bay1Overstay && bay2Overstay) {
    telemetry.alertMessage = "BOTH BAYS OVERSTAY";
  } else if (bay1Overstay) {
    telemetry.alertMessage = "BAY 1 OVERSTAY";
  } else if (bay2Overstay) {
    telemetry.alertMessage = "BAY 2 OVERSTAY";
  } else if (bothBaysFull) {
    telemetry.alertMessage = "CAR PARK FULL";
  } else {
    telemetry.alertMessage = "NONE";
  }
}

void serviceCloud() {
  maintainWiFi();
  maintainTimeSync();
  maintainMQTT();
  publishChangedTelemetry();
  maintainCloudHeartbeat();
}


void setup() {
  Serial.begin(115200);

  pinMode(ECHO_SHARED_PIN, INPUT);

  pinMode(TOP_TRIG_PIN, OUTPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);

  pinMode(BAY2_TOP_TRIG_PIN, OUTPUT);
  pinMode(BAY2_LEFT_TRIG_PIN, OUTPUT);
  pinMode(BAY2_RIGHT_TRIG_PIN, OUTPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED2_R, OUTPUT);
  pinMode(LED2_G, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUZZER2_PIN, OUTPUT);
  pinMode(GATE_LED_PIN, OUTPUT);

  Wire.begin(PLATE_SDA_PIN, PLATE_SCL_PIN);
  plateBaseline = readRawPressure(Wire); // ambient reading, no vehicle on the plate yet

  ExitWire.begin(PLATE2_SDA_PIN, PLATE2_SCL_PIN);
  plateExitBaseline = readRawPressure(ExitWire);

  gateServo.setPeriodHertz(50);
  gateServo.attach(GATE_SERVO_PIN, 500, 2500);
  gateServo.write(GATE_CLOSED_ANGLE);

  // Start Snay's cloud subsystem after all local hardware is initialised.
  setupCloudClients();
  Serial.println("[SYSTEM] Local parking controller started; cloud is best-effort.");
}


void loop() {
  uint32_t now = millis();

  // Local parking logic always gets first priority when its 150 ms cycle is due.
  if (now - lastReadAt >= READ_INTERVAL_MS) {
    lastReadAt = now;

    float topDistance   = readDistanceCm(TOP_TRIG_PIN);
    float leftDistance  = readDistanceCm(LEFT_TRIG_PIN);
    float rightDistance = readDistanceCm(RIGHT_TRIG_PIN);

    bool topBlocked   = isBlocked(topDistance);
    bool leftBlocked  = isBlocked(leftDistance);
    bool rightBlocked = isBlocked(rightDistance);
    bool allBlocked   = topBlocked && leftBlocked && rightBlocked;

    const char *ledLabel;
    if (allBlocked) {
      if (blockedSince == 0) blockedSince = now;

      bay1OverstayActive = (now - blockedSince) >= OVERSTAY_MS;
      if (bay1OverstayActive) {
        setColor(true, true, false);
        setBuzzer(true);
        ledLabel = "YELLOW (overstay)";
      } else {
        setColor(true, false, false);
        setBuzzer(false);
        ledLabel = "RED";
      }
    } else {
      blockedSince = 0;
      bay1OverstayActive = false;
      setColor(false, true, false);
      setBuzzer(false);
      ledLabel = "GREEN";
    }

    Serial.printf("Bay1 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED %s\n",
                  topDistance,   topBlocked   ? "blocked" : "clear",
                  leftDistance,  leftBlocked  ? "blocked" : "clear",
                  rightDistance, rightBlocked ? "blocked" : "clear",
                  ledLabel);

    float bay2TopDistance   = readDistanceCm(BAY2_TOP_TRIG_PIN);
    float bay2LeftDistance  = readDistanceCm(BAY2_LEFT_TRIG_PIN);
    float bay2RightDistance = readDistanceCm(BAY2_RIGHT_TRIG_PIN);

    bool bay2TopBlocked   = isBlocked(bay2TopDistance);
    bool bay2LeftBlocked  = isBlocked(bay2LeftDistance);
    bool bay2RightBlocked = isBlocked(bay2RightDistance);
    bool bay2AllBlocked   = bay2TopBlocked && bay2LeftBlocked && bay2RightBlocked;

    const char *led2Label;
    if (bay2AllBlocked) {
      if (bay2BlockedSince == 0) bay2BlockedSince = now;

      bay2OverstayActive = (now - bay2BlockedSince) >= OVERSTAY_MS;
      if (bay2OverstayActive) {
        setColor2(true, true);
        setBuzzer2(true);
        led2Label = "YELLOW (overstay)";
      } else {
        setColor2(true, false);
        setBuzzer2(false);
        led2Label = "RED";
      }
    } else {
      bay2BlockedSince = 0;
      bay2OverstayActive = false;
      setColor2(false, true);
      setBuzzer2(false);
      led2Label = "GREEN";
    }

    Serial.printf("Bay2 Top: %.1f cm (%s) | Left: %.1f cm (%s) | Right: %.1f cm (%s) -> LED2 %s\n",
                  bay2TopDistance,   bay2TopBlocked   ? "blocked" : "clear",
                  bay2LeftDistance,  bay2LeftBlocked  ? "blocked" : "clear",
                  bay2RightDistance, bay2RightBlocked ? "blocked" : "clear",
                  led2Label);

    // Controller-owned capacity state.
    bothBaysFull = allBlocked && bay2AllBlocked;

    servicePlate();
    servicePlateExit();
    serviceGate();
    serviceGateLed();

    Serial.printf("Entry plate delta: %d / %d%s | Exit plate delta: %d / %d%s%s\n",
                  plateLastDelta, PRESSURE_DELTA_THRESHOLD, plateActive ? " (PRESSED)" : "",
                  plateExitLastDelta, PRESSURE_DELTA_THRESHOLD, plateExitActive ? " (PRESSED)" : "",
                  bothBaysFull ? " | CAPACITY FULL" : "");

    // Export the controller's authoritative state to Snay's cloud layer.
    syncTelemetryFromController(allBlocked,
                                bay2AllBlocked,
                                bay1OverstayActive,
                                bay2OverstayActive);
  }

  // Best-effort network/cloud servicing occurs after local control work.
  serviceCloud();
}
