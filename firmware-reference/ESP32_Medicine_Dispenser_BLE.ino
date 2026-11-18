// ============================================================================
// ESP32 scale + medicine dispenser
//  - HX711 load cell with buzzer / LED alarm
//  - 4 servos, one per medicine. Each servo has a medicine name, a number of days
//    and up to 8 dispensing times per day, all set from the web dashboard.
//  - The count starts tomorrow: a course of 7 days saved today runs tomorrow
//    through the day after 7 days.
//  - Everything is stored in flash (Preferences) and survives a restart.
//  - The clock is set by the dashboard over BLE (or by a DS3231, see USE_RTC).
//
// Commands (one per line, over BLE GATT or the USB Serial Monitor set to "Newline"):
//   t                tare            w  current weight           h  help
//   T <epoch>        set the clock (local wall-clock time as seconds since 1970)
//   P <n> <on> <startDay> <days> <count> <m1> ... <mk>|<medicine name>
//                    set the plan of servo n (0-3):
//                      on        1 = enabled, 0 = disabled
//                      startDay  first dispensing day, in days since 1970-01-01 (local date)
//                      days      length of the course in days (1-365)
//                      count     number of dispensing times per day (0-8)
//                      m1..mk    the times, as minutes after midnight (08:30 = 510)
//   P?               send all plans, their status and the clock
//   G <n>            turn servo n now (test)
//
// Lines sent by the ESP32 besides "Load: ...":
//   PLAN <n> <on> <startDay> <days> <count> <m1> ... <mk>|<name>   the stored plan
//   STAT <n> <state> <dayNo> <daysLeft> <nextEpoch>                status computed here
//        state: 0 off, 1 waiting to start, 2 running, 3 finished, 4 clock not set
//        nextEpoch: next dispensing time as local wall-clock seconds since 1970 (0 = none)
//   CLOCK <set 0/1> <yyyy-mm-dd hh:mm:ss>      SERVO <n> RUN|DONE      ERR <text>
//
// Wiring: servo signal wires to GPIO 13, 14, 27, 33. Power the servos from a
// separate 5V supply and join its GND to the ESP32 GND.
// ============================================================================
#include <ESP32Servo.h>
#include <Preferences.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include "HX711.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include "PlanTypes.h"

// Set to 1 if a DS3231 clock module is connected (SDA = GPIO 21, SCL = GPIO 22).
// With it the plans keep running after a power cut, even without the dashboard.
// Needs the "RTClib" library (Adafruit).
#define USE_RTC 0
#if USE_RTC
#include <Wire.h>
#include <RTClib.h>
RTC_DS3231 rtc;
#endif

// ---------- BLE GATT ----------
// These UUIDs must match BLE_CONFIG in the web application.
static const char* BLE_DEVICE_NAME = "DC-PROTO-02";
static const char* SERVICE_UUID        = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static const char* DISPENSE_CHAR_UUID  = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
static const char* TELEMETRY_CHAR_UUID = "13187b10-80a2-4a00-a2ef-98844e135540";

BLEServer* bleServer = nullptr;
BLECharacteristic* telemetryCharacteristic = nullptr;
volatile bool bleClientConnected = false;
bool bleDispensePending[4] = {false, false, false, false};

void handleBleDispenseCommand(const std::string& payload);

void sendBoth(const String &s) {
  Serial.println(s);
  if (bleClientConnected && telemetryCharacteristic != nullptr) {
    telemetryCharacteristic->setValue((uint8_t*)s.c_str(), s.length());
    telemetryCharacteristic->notify();
  }
}

class DispenserServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleClientConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* server) override {
    bleClientConnected = false;
    Serial.println("BLE client disconnected; advertising restarted");
    BLEDevice::startAdvertising();
  }
};

class DispenseWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string value = characteristic->getValue();
    handleBleDispenseCommand(value);
  }
};

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new DispenserServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);
  BLECharacteristic* dispenseCharacteristic = service->createCharacteristic(
    DISPENSE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  dispenseCharacteristic->setCallbacks(new DispenseWriteCallbacks());

  telemetryCharacteristic = service->createCharacteristic(
    TELEMETRY_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  telemetryCharacteristic->addDescriptor(new BLE2902());
  telemetryCharacteristic->setValue("READY");

  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.println("BLE GATT advertising started: DC-PROTO-02");
}

// ---------- Load cell (HX711) ----------
const int HX_DT = 18;
const int HX_SCK = 19;
// IMPORTANT: 697.0 is the existing starting value, not a verified calibration.
// Calibrate this against known weights and confirm that readings are in grams.
float calibrationFactor = 697.0;
const unsigned long READ_MS = 100;          // sample when HX711 reports ready
const unsigned long LOAD_REPORT_MS = 500;   // Bluetooth/Serial reporting interval
const float WEIGHT_FILTER_ALPHA = 0.25f;    // exponential smoothing (0..1)

HX711 scale;
unsigned long lastRead = 0;
unsigned long lastLoadReport = 0;
float weight = 0.0f;                        // filtered weight in grams
bool weightFilterInitialized = false;
bool weightSampleAvailable = false;          // one fresh HX711 sample for alarm confirmation

// ---------- Buzzer ----------
const int BUZZER_PIN = 26;
// Alarm is triggered above 2.0 g and clears only after falling below 1.5 g.
const float WEIGHT_ON = 2.0f;
const float WEIGHT_OFF = 1.5f;
const uint8_t ALARM_ON_CONFIRM_READINGS = 2;  // reject brief spikes
const uint8_t ALARM_OFF_CONFIRM_READINGS = 3; // reject brief dips
const unsigned long BEEP_PERIOD_MS = 1000;
const unsigned long BEEP_ON_MS = 200;

bool alarmActive = false;
uint8_t alarmOnReadings = 0;
uint8_t alarmOffReadings = 0;
unsigned long beepStart = 0;

// ---------- LED ----------
const int LED_PIN = 25;
const unsigned long LED_BLINK_MS = 100;      // LED toggles every 100 ms during a beep

// ---------- Servos ----------
const int NUM_SERVOS = 4;
const int servoPins[NUM_SERVOS] = {13, 14, 27, 33};
const int MIN_US = 600;                     // conservative SG90 starting range
const int MAX_US = 2300;                     // verify endpoints on each mechanism
const int HOME_ANGLE = 0;                    // resting position
const int TURN_ANGLE = 90;                   // position when dispensing
const bool RETURN_TO_HOME = true;            // false = stay at 90 degrees
const unsigned long STEP_MS = 15;            // ms per degree (90 degrees takes about 1.4 s)
const unsigned long HOLD_MS = 1000;          // pause at 90 degrees before returning

enum ServoPhase { PH_OUT, PH_HOLD, PH_BACK };

Servo servos[NUM_SERVOS];
bool queued[NUM_SERVOS] = {false, false, false, false};
int activeServo = -1;                        // -1 = no servo is moving
int servoAngle = HOME_ANGLE;
ServoPhase servoPhase = PH_OUT;
unsigned long lastStepMs = 0;
unsigned long holdStart = 0;

// ---------- Plans (stored in flash; definitions are in PlanTypes.h) ----------
Plan plans[NUM_SERVOS];
long lastFiredMinute[NUM_SERVOS] = {-1, -1, -1, -1};
unsigned long lastSchedCheck = 0;
Preferences prefs;

// ---------- Clock ----------
// The time is kept as local wall-clock time stored as if it were UTC, so it is
// read back with gmtime_r() and no time zone handling is needed on the ESP32.
bool timeSynced = false;
bool statusDirty = false;                    // send the status lines at the next chance
unsigned long lastStatusMsg = 0;

// ---------- Serial input ----------
String serialBuf = "";

// ============================================================================
// Clock helpers
// ============================================================================
void setClock(time_t t) {
  struct timeval tv;
  tv.tv_sec = t;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  timeSynced = true;
  statusDirty = true;
}

bool getLocalTm(struct tm &out) {
  if (!timeSynced) return false;
  time_t t = time(nullptr);
  gmtime_r(&t, &out);
  return true;
}

String clockText() {
  struct tm t;
  if (!getLocalTm(t)) return "-";
  char b[24];
  strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &t);
  return String(b);
}

// ============================================================================
// Plan storage
// ============================================================================
void defaultPlan(Plan &p) {
  memset(&p, 0, sizeof(p));
  p.days = 1;
}

bool planValid(const Plan &p) {
  if (p.enabled > 1 || p.count > MAX_TIMES) return false;
  if (p.days < 1 || p.days > MAX_DAYS) return false;
  if (memchr(p.name, 0, sizeof(p.name)) == nullptr) return false;
  for (int k = 0; k < p.count; k++) {
    if (p.times[k] > 1439) return false;
    if (k > 0 && p.times[k] <= p.times[k - 1]) return false;
  }
  return true;
}

void loadPlans() {
  prefs.begin("servo-plans", false);
  bool ok = (prefs.getBytesLength("plans") == sizeof(plans));
  if (ok) prefs.getBytes("plans", plans, sizeof(plans));
  prefs.end();

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!ok || !planValid(plans[i])) defaultPlan(plans[i]);
  }
}

void savePlans() {
  prefs.begin("servo-plans", false);
  prefs.putBytes("plans", plans, sizeof(plans));
  prefs.end();
}

void sendPlan(int i) {
  const Plan &p = plans[i];
  String s = "PLAN " + String(i) + " " + String((int)p.enabled) + " " + String((unsigned long)p.startDay) + " " +
             String((int)p.days) + " " + String((int)p.count);
  for (int k = 0; k < p.count; k++) s += " " + String((int)p.times[k]);
  s += "|";
  s += p.name;
  sendBoth(s);
}

// ============================================================================
// Plan status: which day of the course it is, days to go, next dispensing time
// ============================================================================
enum PlanState { ST_OFF = 0, ST_WAITING = 1, ST_RUNNING = 2, ST_DONE = 3, ST_NOCLOCK = 4 };

PlanStat computeStat(int i) {
  const Plan &p = plans[i];
  PlanStat s = {ST_OFF, 0, 0, 0};
  if (!p.enabled || p.count == 0) return s;

  s.daysLeft = p.days;
  if (!timeSynced) {
    s.state = ST_NOCLOCK;
    return s;
  }

  long nowLocal = (long)time(nullptr);
  long today = nowLocal / 86400;
  long minuteNow = (nowLocal % 86400) / 60;
  long firstDay = (long)p.startDay;
  long lastDay = firstDay + p.days - 1;

  if (today < firstDay) {
    s.state = ST_WAITING;
    s.nextEpoch = (unsigned long)firstDay * 86400UL + (unsigned long)p.times[0] * 60UL;
    return s;
  }
  if (today > lastDay) {
    s.state = ST_DONE;
    s.dayNo = p.days;
    s.daysLeft = 0;
    return s;
  }

  s.dayNo = today - firstDay + 1;
  int next = -1;
  for (int k = 0; k < p.count; k++) {
    if (p.times[k] > minuteNow) { next = k; break; }
  }
  if (next >= 0) {
    s.state = ST_RUNNING;
    s.daysLeft = lastDay - today + 1;
    s.nextEpoch = (unsigned long)today * 86400UL + (unsigned long)p.times[next] * 60UL;
  } else if (today < lastDay) {
    s.state = ST_RUNNING;
    s.daysLeft = lastDay - today;
    s.nextEpoch = (unsigned long)(today + 1) * 86400UL + (unsigned long)p.times[0] * 60UL;
  } else {
    s.state = ST_DONE;                     // the last dose of the course has been given
    s.daysLeft = 0;
  }
  return s;
}

void sendStat(int i) {
  PlanStat s = computeStat(i);
  sendBoth("STAT " + String(i) + " " + String(s.state) + " " + String(s.dayNo) + " " +
           String(s.daysLeft) + " " + String(s.nextEpoch));
}

void broadcastStatus(unsigned long now) {
  if (!statusDirty && now - lastStatusMsg < 5000) return;
  lastStatusMsg = now;
  statusDirty = false;
  sendBoth("CLOCK " + String(timeSynced ? 1 : 0) + " " + clockText());
  for (int i = 0; i < NUM_SERVOS; i++) sendStat(i);
}

// ============================================================================
// Servo runner (non-blocking, one servo at a time)
// ============================================================================
void startServoRun(int i, unsigned long now) {
  activeServo = i;
  servos[i].setPeriodHertz(50);
  servos[i].attach(servoPins[i], MIN_US, MAX_US);
  servoAngle = HOME_ANGLE;
  servos[i].write(servoAngle);
  servoPhase = PH_OUT;
  lastStepMs = now;
  sendBoth("SERVO " + String(i) + " RUN");
}

void finishServoRun() {
  int finishedServo = activeServo;
  servos[finishedServo].detach();            // no holding current, no jitter between runs
  sendBoth("SERVO " + String(finishedServo) + " DONE");
  if (bleDispensePending[finishedServo]) {
    // The web app receives DISPENSED only after the servo has completed its full cycle.
    sendBoth("ACK:DISPENSED:" + String(finishedServo));
    bleDispensePending[finishedServo] = false;
  }
  activeServo = -1;
  statusDirty = true;                        // the next dispensing time has moved on
}

void updateServos(unsigned long now) {
  if (activeServo < 0) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (queued[i]) {
        queued[i] = false;
        startServoRun(i, now);
        break;
      }
    }
    return;
  }

  switch (servoPhase) {
    case PH_OUT:
      if (now - lastStepMs >= STEP_MS) {
        lastStepMs = now;
        if (servoAngle < TURN_ANGLE) {
          servoAngle++;
          servos[activeServo].write(servoAngle);
        } else {
          servoPhase = PH_HOLD;
          holdStart = now;
        }
      }
      break;

    case PH_HOLD:
      if (now - holdStart >= HOLD_MS) {
        if (RETURN_TO_HOME) {
          servoPhase = PH_BACK;
          lastStepMs = now;
        } else {
          finishServoRun();
        }
      }
      break;

    case PH_BACK:
      if (now - lastStepMs >= STEP_MS) {
        lastStepMs = now;
        if (servoAngle > HOME_ANGLE) {
          servoAngle--;
          servos[activeServo].write(servoAngle);
        } else {
          finishServoRun();
        }
      }
      break;
  }
}

// Check once a second whether a dispensing time has come. A dose that was due
// while the ESP32 was off or had no clock is skipped, not given late.
void checkSchedules(unsigned long now) {
  if (now - lastSchedCheck < 1000) return;
  lastSchedCheck = now;
  if (!timeSynced) return;

  long nowLocal = (long)time(nullptr);
  long today = nowLocal / 86400;
  long minuteNow = (nowLocal % 86400) / 60;
  long minuteStamp = nowLocal / 60;

  for (int i = 0; i < NUM_SERVOS; i++) {
    const Plan &p = plans[i];
    if (!p.enabled || p.count == 0) continue;
    if (today < (long)p.startDay || today > (long)p.startDay + p.days - 1) continue;

    bool due = false;
    for (int k = 0; k < p.count; k++) {
      if (p.times[k] == minuteNow) due = true;
    }
    if (!due) continue;
    if (lastFiredMinute[i] == minuteStamp) continue;   // already ran in this minute
    lastFiredMinute[i] = minuteStamp;
    queued[i] = true;
  }
}

// ============================================================================
// Load cell and alarm
// ============================================================================
void readLoadCell(unsigned long now) {
  if (now - lastRead < READ_MS) return;
  if (!scale.is_ready()) return;
  lastRead = now;

  // Read one ready sample rather than blocking for a multi-sample average.
  // This keeps the non-blocking servo state machine responsive during dispensing.
  float sample = scale.get_units(1);
  if (!isfinite(sample)) return;

  weightSampleAvailable = true;
  if (!weightFilterInitialized) {
    weight = sample;
    weightFilterInitialized = true;
  } else {
    weight += WEIGHT_FILTER_ALPHA * (sample - weight);
  }

  if (now - lastLoadReport >= LOAD_REPORT_MS) {
    lastLoadReport = now;
    sendBoth("WEIGHT:" + String(weight, 2) + "g");
  }
}

void updateBuzzer(unsigned long now) {
  // Hysteresis plus consecutive *fresh HX711 samples* prevents chatter.
  // Do not count repeated loop iterations using the same filtered reading.
  if (weightSampleAvailable) {
    weightSampleAvailable = false;

    if (!alarmActive) {
      alarmOffReadings = 0;
      if (weightFilterInitialized && weight > WEIGHT_ON) {
        if (alarmOnReadings < ALARM_ON_CONFIRM_READINGS) alarmOnReadings++;
        if (alarmOnReadings >= ALARM_ON_CONFIRM_READINGS) {
          alarmActive = true;
          alarmOnReadings = 0;
          beepStart = now;
          sendBoth("ALARM:CAREGIVER_ALERT:UNCOLLECTED_DOSE");
        }
      } else {
        alarmOnReadings = 0;
      }
    } else {
      alarmOnReadings = 0;
      if (weightFilterInitialized && weight < WEIGHT_OFF) {
        if (alarmOffReadings < ALARM_OFF_CONFIRM_READINGS) alarmOffReadings++;
        if (alarmOffReadings >= ALARM_OFF_CONFIRM_READINGS) {
          alarmActive = false;
          alarmOffReadings = 0;
        }
      } else {
        alarmOffReadings = 0;
      }
    }
  }

  bool beeping = false;
  if (alarmActive) {
    unsigned long phase = (now - beepStart) % BEEP_PERIOD_MS;
    beeping = (phase < BEEP_ON_MS);

    // LED blinks only while the buzzer is sounding
    if (beeping) {
      bool ledOn = (phase / LED_BLINK_MS) % 2 == 0;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  digitalWrite(BUZZER_PIN, beeping ? HIGH : LOW);
}

// ============================================================================
// BLE dispense command: frontend sends the compartment index as UTF-8 text.
// Current hardware supports indexes 0..3 (displayed compartments 1..4).
// Indexes 4..12 are deliberately rejected until their servos are installed.
// ============================================================================
void handleBleDispenseCommand(const std::string& payload) {
  if (payload.empty()) {
    sendBoth("ERROR:INVALID_COMPARTMENT");
    return;
  }

  for (char c : payload) {
    if (c < '0' || c > '9') {
      sendBoth("ERROR:INVALID_COMPARTMENT");
      return;
    }
  }

  long firmwareIndex = strtol(payload.c_str(), nullptr, 10);
  if (firmwareIndex < 0 || firmwareIndex > 12 || firmwareIndex >= NUM_SERVOS) {
    sendBoth("ERROR:INVALID_COMPARTMENT");
    return;
  }

  bool anyQueued = false;
  for (int i = 0; i < NUM_SERVOS; i++) anyQueued = anyQueued || queued[i];
  if (activeServo >= 0 || anyQueued) {
    sendBoth("ERROR:DISPENSE_BUSY");
    return;
  }

  // Reject a dispense if the scale has not produced a recent valid sample.
  unsigned long now = millis();
  if (!weightFilterInitialized || now - lastRead > 2000 || !scale.is_ready()) {
    sendBoth("ERROR:SCALE_NOT_READY");
    return;
  }

  queued[firmwareIndex] = true;
  bleDispensePending[firmwareIndex] = true;
  sendBoth("ACK:RECEIVED:" + String((int)firmwareIndex));
}

// ============================================================================
// Commands
// ============================================================================
void handleCommand(char c) {
  switch (c) {
    case 't':
      // Never change the scale baseline while a dose is being dispensed.
      // Tare only when the collection platform is in its intended empty/tare state.
      {
        bool anyQueued = false;
        for (int k = 0; k < NUM_SERVOS; k++) anyQueued = anyQueued || queued[k];
        if (activeServo >= 0 || anyQueued) {
          sendBoth("ERR cannot tare while dispensing or queued");
        } else if (scale.wait_ready_timeout(1000)) {
          scale.tare();
          weight = 0.0f;
          weightFilterInitialized = true;
          alarmActive = false;
          alarmOnReadings = 0;
          alarmOffReadings = 0;
          digitalWrite(BUZZER_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          sendBoth("Tared");
        } else {
          sendBoth("ERR HX711 not responding");
        }
      }
      break;
    case 'w':
      sendBoth("Weight: " + String(weight, 2) + " g");
      break;
    case 'h':
      sendBoth("Commands: t=tare (only when not dispensing), w=weight, h=help, T <epoch>=set clock, "
               "P <n> <on> <startDay> <days> <count> <m1>..|<name>=set plan, P?=list, G <n>=turn servo now");
      break;
    default:
      break;  // ignore other characters
  }
}

// P <n> <on> <startDay> <days> <count> <m1> ... <mk>|<name>
void handlePlanLine(const String &line) {
  int bar = line.indexOf('|');
  String head = (bar >= 0) ? line.substring(0, bar) : line;
  String name = (bar >= 0) ? line.substring(bar + 1) : String("");

  char buf[160];
  strncpy(buf, head.c_str() + 1, sizeof(buf) - 1);   // skip the leading "P"
  buf[sizeof(buf) - 1] = 0;

  long v[5 + MAX_TIMES + 1];
  int n = 0;
  bool bad = false;
  char *save = nullptr;
  for (char *tok = strtok_r(buf, " ", &save); tok != nullptr; tok = strtok_r(nullptr, " ", &save)) {
    char *end = nullptr;
    long x = strtol(tok, &end, 10);
    if (*end != 0 || n >= 5 + MAX_TIMES + 1) { bad = true; break; }
    v[n++] = x;
  }
  if (bad || n < 5) { sendBoth("ERR bad plan"); return; }

  long i = v[0], en = v[1], startDay = v[2], days = v[3], count = v[4];
  if (i < 0 || i >= NUM_SERVOS || en < 0 || en > 1 || startDay < 0 || startDay > 200000L ||
      days < 1 || days > MAX_DAYS || count < 0 || count > MAX_TIMES || n != 5 + count) {
    sendBoth("ERR bad plan");
    return;
  }

  uint16_t times[MAX_TIMES];
  for (int k = 0; k < count; k++) {
    if (v[5 + k] < 0 || v[5 + k] > 1439) { sendBoth("ERR bad time"); return; }
    times[k] = (uint16_t)v[5 + k];
  }
  // sort, then drop duplicates
  for (int a = 1; a < count; a++) {
    uint16_t key = times[a];
    int b = a - 1;
    while (b >= 0 && times[b] > key) { times[b + 1] = times[b]; b--; }
    times[b + 1] = key;
  }
  int unique = 0;
  for (int k = 0; k < count; k++) {
    if (unique == 0 || times[k] != times[unique - 1]) times[unique++] = times[k];
  }

  // the name: printable ASCII only, no "|", at most NAME_LEN characters, trimmed
  String clean = "";
  for (unsigned int k = 0; k < name.length() && clean.length() < NAME_LEN; k++) {
    char c = name.charAt(k);
    if (c >= 32 && c <= 126 && c != '|') clean += c;
  }
  clean.trim();

  Plan p;
  defaultPlan(p);
  p.enabled = (uint8_t)en;
  p.count = (uint8_t)unique;
  p.days = (uint16_t)days;
  p.startDay = (uint32_t)startDay;
  for (int k = 0; k < unique; k++) p.times[k] = times[k];
  strncpy(p.name, clean.c_str(), NAME_LEN);
  p.name[NAME_LEN] = 0;

  plans[i] = p;
  savePlans();
  lastFiredMinute[i] = -1;
  sendPlan((int)i);
  sendStat((int)i);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.length() == 1) {
    handleCommand(line.charAt(0));
    return;
  }

  int i;

  if (line.startsWith("T ")) {
    long epoch = line.substring(2).toInt();
    if (epoch < 1700000000L) {
      sendBoth("ERR bad time");
      return;
    }
    setClock((time_t)epoch);
#if USE_RTC
    rtc.adjust(DateTime((uint32_t)epoch));
#endif

  } else if (line == "P?") {
    for (int k = 0; k < NUM_SERVOS; k++) {
      sendPlan(k);
      sendStat(k);
    }
    sendBoth("CLOCK " + String(timeSynced ? 1 : 0) + " " + clockText());

  } else if (line.startsWith("P ")) {
    handlePlanLine(line);

  } else if (line.startsWith("G ") && sscanf(line.c_str(), "G %d", &i) == 1) {
    if (i < 0 || i >= NUM_SERVOS) {
      sendBoth("ERR bad servo");
      return;
    }
    queued[i] = true;

  } else {
    sendBoth("ERR unknown command");
  }
}

// Collect characters until a line ends, then run the command
void pollInput(Stream &port, String &buf) {
  while (port.available()) {
    char c = (char)port.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        handleLine(buf);
        buf = "";
      }
    } else if (buf.length() < 200) {
      buf += c;
    }
  }
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  setupBLE();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int t = 0; t < 4; t++) ESP32PWM::allocateTimer(t);
  loadPlans();
  for (int k = 0; k < NUM_SERVOS; k++) {
    Serial.print("Servo ");
    Serial.print(k + 1);
    Serial.print(plans[k].enabled ? ": plan loaded (on) " : ": plan loaded (off) ");
    Serial.println(plans[k].name);
  }

#if USE_RTC
  Wire.begin(21, 22);
  if (!rtc.begin()) {
    Serial.println("RTC not found. Check SDA/SCL wiring.");
  } else if (rtc.lostPower()) {
    Serial.println("RTC lost power. Waiting for the dashboard to set the time.");
  } else {
    setClock((time_t)rtc.now().unixtime());
    Serial.println("Clock set from the RTC.");
  }
#endif

  scale.begin(HX_DT, HX_SCK);
  scale.set_scale(calibrationFactor);
  Serial.println("Taring... keep the load cell empty");
  if (scale.wait_ready_timeout(3000)) {
    scale.tare();
  } else {
    Serial.println("HX711 not responding. Check DT/SCK/VCC/GND wiring.");
  }
  Serial.println("Ready. Send h for the command list.");
}

void loop() {
  unsigned long now = millis();

  readLoadCell(now);
  updateBuzzer(now);
  checkSchedules(now);
  updateServos(now);
  broadcastStatus(now);

  pollInput(Serial, serialBuf);
}
