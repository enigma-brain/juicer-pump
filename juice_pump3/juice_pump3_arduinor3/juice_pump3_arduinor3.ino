/*
 * Pump Controller — Arduino Uno R3 Port
 * 
 * Ported from ESP32-S2 version. Changes:
 *  - Display removed entirely
 *  - NeoPixel removed
 *  - ESP32 LEDC PWM replaced with simple digitalWrite HIGH/LOW on STEP_PIN
 *    (simulates pump: pin goes HIGH for duration of dispense, LOW when stopped)
 *  - Preferences replaced with EEPROM
 *  - Baud rate reduced to 115200 (Uno max reliable)
 *  - ArduinoJson buffer sizes reduced for 2KB RAM
 *  - String usage minimized where possible
 *
 * Pin mapping (adjust to your wiring):
 *   STEP_PIN  = 3   (pump simulation output — HIGH when running)
 *   DIR_PIN   = 4   (direction output)
 *   EN_PIN    = 5   (enable output)
 *   SWITCH_D0 = 6   (reset button, INPUT_PULLUP, active LOW)
 *   SWITCH_D1 = 7   (purge button, active HIGH)
 *   SWITCH_D2 = 8   (manual water button, active HIGH)
 *   TTL_WATER = 9   (TTL input, active HIGH)
 *   JUICE_LVL = 10  (juice level sensor, LOW = water detected)
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>  // 7.2.0 by Benoit Blanchon

// ── Pin Definitions ──────────────────────────────────────────────────────────
#define STEP_PIN        11
#define DIR_PIN         4
#define EN_PIN          5
#define SWITCH_D0_PIN   6   // Reset button (active LOW with internal pullup)
#define SWITCH_D1_PIN   7   // Purge button (active HIGH — wire with pull-down or use external)
#define SWITCH_D2_PIN   8   // Manual water button (active HIGH)
#define TTL_WATER_PIN   9   // TTL input (active HIGH)
#define JUICE_LEVEL_LOW_PIN 10

// ── EEPROM Addresses ─────────────────────────────────────────────────────────
// Each float is 4 bytes, ints are 2 bytes (stored as int16_t for safety)
#define EEPROM_FLOW_RATE_ADDR    0   // float, 4 bytes
#define EEPROM_PURGE_VOL_ADDR    4   // float, 4 bytes
#define EEPROM_TARGET_RPS_ADDR   8   // float, 4 bytes
#define EEPROM_DIRECTION_ADDR   12   // int16_t, 2 bytes
#define EEPROM_ROP_ADDR         14   // int16_t, 2 bytes
#define EEPROM_MAGIC_ADDR       16   // uint16_t, 2 bytes (to detect first-run)
#define EEPROM_MAGIC_VALUE  0xBEEF

// ── Motor Constants ──────────────────────────────────────────────────────────
const int PULSES_PER_STEP = 32;
const int STEPS_PER_REV = 200;
const int MAX_RPS = 8;

// ── State Variables ──────────────────────────────────────────────────────────
float flow_rate;
float purge_vol;
float target_rps;
int   reward_number = 0;
float reward_mls = 0.0;

bool purging = false;
bool manual_watering = false;
bool ttl_watering = false;
bool serial_watering = false;
bool calibration_in_progress = false;
int  calibration_n = 0;
int  calibration_on = 0;
int  calibration_off = 0;
int  calibration_count = 0;
unsigned long calibration_start_time = 0;

float serial_vol = 0.0;
bool  reset_pressed = false;
unsigned long water_start_time = 0;
unsigned long ttl_start_time = 0;
bool  pump_running = false;
uint32_t pump_stop_time = 0;
bool  serialConnected = false;
bool  notify_reward_complete_pending = false;

// Juice level
String juice_level = "<50mLs";
unsigned long last_juice_check_time = 0;

// ── Direction & Overlap Policy ───────────────────────────────────────────────
enum PumpDirection { DIR_LEFT = 0, DIR_RIGHT = 1 };
const PumpDirection DEFAULT_DIRECTION = DIR_LEFT;
PumpDirection current_direction = DEFAULT_DIRECTION;
PumpDirection calibration_direction = DEFAULT_DIRECTION;

enum RewardOverlapPolicy { ROP_REPLACE = 0, ROP_APPEND = 1, ROP_REJECT = 2 };
RewardOverlapPolicy reward_overlap_policy = ROP_REPLACE;

// ── EEPROM Helpers ───────────────────────────────────────────────────────────
void eeprom_write_float(int addr, float val) {
  EEPROM.put(addr, val);
}

float eeprom_read_float(int addr, float defaultVal) {
  float val;
  EEPROM.get(addr, val);
  // Check for NaN / uninitialized
  if (isnan(val) || isinf(val)) return defaultVal;
  return val;
}

void eeprom_write_int16(int addr, int16_t val) {
  EEPROM.put(addr, val);
}

int16_t eeprom_read_int16(int addr, int16_t defaultVal) {
  int16_t val;
  EEPROM.get(addr, val);
  return val;
}

bool eeprom_is_initialized() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return magic == EEPROM_MAGIC_VALUE;
}

void eeprom_set_initialized() {
  uint16_t magic = EEPROM_MAGIC_VALUE;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
}

// ── String Helpers ───────────────────────────────────────────────────────────
String direction_to_string(PumpDirection dir) {
  return dir == DIR_RIGHT ? F("right") : F("left");
}

String rop_to_string(RewardOverlapPolicy p) {
  if (p == ROP_APPEND) return F("append");
  if (p == ROP_REJECT) return F("reject");
  return F("replace");
}

String pump_state_to_string() {
  if (purging) return F("purge");
  if (serial_watering) return F("serial_reward");
  if (ttl_watering) return F("ttl");
  if (manual_watering) return F("manual");
  if (calibration_in_progress) return F("calibration");
  return F("idle");
}

bool parse_rop(const String& s, RewardOverlapPolicy& out) {
  String v = s;
  v.toLowerCase();
  if (v == "replace") { out = ROP_REPLACE; return true; }
  if (v == "append")  { out = ROP_APPEND;  return true; }
  if (v == "reject")  { out = ROP_REJECT;  return true; }
  return false;
}

// ── Pump Control (Simulated) ─────────────────────────────────────────────────
// Instead of variable-frequency PWM, we just drive STEP_PIN HIGH while "running"
// and LOW when stopped. This is enough to simulate the pump on an Uno.

void apply_direction(PumpDirection dir) {
  digitalWrite(DIR_PIN, dir == DIR_LEFT ? HIGH : LOW);
}

void start_pump_with_direction(PumpDirection direction) {
  apply_direction(direction);
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(STEP_PIN, HIGH);  // Simulated pump ON
  pump_running = true;
}

void start_pump() {
  start_pump_with_direction(current_direction);
}

void stop_pump() {
  digitalWrite(STEP_PIN, LOW);   // Simulated pump OFF
  pump_running = false;
  apply_direction(current_direction);
}

// ── Counter / Reward Helpers ─────────────────────────────────────────────────
void reset_counters() {
  reward_number = 0;
  reward_mls = 0;
}

void reconcile_running_serial_reward() {
  reward_mls -= serial_vol;
  reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
}

void start_serial_reward(float reward_value) {
  serial_vol = reward_value;
  serial_watering = true;
  water_start_time = millis();
  start_pump();
  pump_stop_time = millis() + (unsigned long)(reward_value / flow_rate * 1000.0);
  reward_mls += reward_value;
  reward_number++;
}

void replace_serial_reward(float reward_value) {
  if (serial_watering) {
    reconcile_running_serial_reward();
  }
  start_serial_reward(reward_value);
}

void append_serial_reward(float reward_value) {
  serial_vol += reward_value;
  uint32_t extra_ms = (uint32_t)(reward_value / flow_rate * 1000.0);
  pump_stop_time += extra_ms;
  reward_mls += reward_value;
  reward_number++;
}

void handle_calibration(int n, int on, int off) {
  calibration_in_progress = true;
  calibration_n = n;
  calibration_on = on;
  calibration_off = off;
  calibration_count = 0;
  calibration_direction = current_direction;
  calibration_start_time = millis();
  start_pump_with_direction(calibration_direction);
  reward_mls += calibration_on / 1000.0 * flow_rate;
  reward_number++;
  water_start_time = millis();
}

// ── Juice Level Check ────────────────────────────────────────────────────────
void check_juice_level() {
  if (millis() - last_juice_check_time >= 5000) {
    bool low_sensor = (digitalRead(JUICE_LEVEL_LOW_PIN) == LOW);
    juice_level = low_sensor ? ">50mLs" : "<50mLs";
    last_juice_check_time = millis();
  }
}

// ── Button Checks ────────────────────────────────────────────────────────────
void check_buttons() {
  // Reset button (active LOW)
  if (digitalRead(SWITCH_D0_PIN) == LOW && !reset_pressed) {
    reset_pressed = true;
    reset_counters();
  } else if (digitalRead(SWITCH_D0_PIN) == HIGH) {
    reset_pressed = false;
  }

  // Purge button (S1, active HIGH)
  if (digitalRead(SWITCH_D1_PIN) == HIGH && !purging && !manual_watering && !ttl_watering) {
    purging = true;
    pump_stop_time = millis() + (unsigned long)(purge_vol / flow_rate * 1000.0);
    start_pump();
  }

  // Manual water button (S2, active HIGH)
  if (digitalRead(SWITCH_D2_PIN) == HIGH && !purging && !manual_watering && !ttl_watering) {
    manual_watering = true;
    water_start_time = millis();
    start_pump();
  }
}

// ── TTL Watering ─────────────────────────────────────────────────────────────
void check_ttl_watering() {
  bool ttl_high = (digitalRead(TTL_WATER_PIN) == HIGH);

  if (ttl_high && !ttl_watering) {
    if (!purging && !manual_watering && !serial_watering && !calibration_in_progress) {
      ttl_watering = true;
      ttl_start_time = millis();
      reward_number++;
      start_pump();
    }
  } else if (!ttl_high && ttl_watering) {
    reward_mls += ((millis() - ttl_start_time) / 1000.0) * flow_rate;
    ttl_watering = false;
    stop_pump();
  }
}

// ── Pump Stop Check ──────────────────────────────────────────────────────────
void check_for_pump_stop() {
  if (purging && (millis() >= pump_stop_time)) {
    purging = false;
    stop_pump();
  }

  if (serial_watering && (millis() >= pump_stop_time)) {
    serial_watering = false;
    stop_pump();
    if (notify_reward_complete_pending) {
      Serial.println(F("{\"notify\":\"reward_complete\"}"));
      notify_reward_complete_pending = false;
    }
  }

  if (manual_watering && !digitalRead(SWITCH_D2_PIN)) {
    reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
    manual_watering = false;
    stop_pump();
  }

  if (calibration_in_progress) {
    if (calibration_count < calibration_n) {
      if (pump_running && millis() >= calibration_start_time + (unsigned long)calibration_on) {
        stop_pump();
        calibration_start_time = millis();
      } else if (!pump_running && millis() >= calibration_start_time + (unsigned long)calibration_off) {
        calibration_count++;
        if (calibration_count < calibration_n) {
          start_pump_with_direction(calibration_direction);
          water_start_time = millis();
          reward_mls += calibration_on / 1000.0 * flow_rate;
          reward_number++;
          calibration_start_time = millis();
        } else {
          calibration_in_progress = false;
        }
      }
    }
  }
}

// ── Serial Command Handler ───────────────────────────────────────────────────
void check_serial_commands() {
  if (Serial.available() <= 0) return;

  // Sync to start of JSON
  while (Serial.available() > 0 && Serial.peek() != '{') {
    Serial.read();
  }
  if (Serial.available() == 0) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  StaticJsonDocument<512> doc;
  StaticJsonDocument<400> responseDoc; // Response JSON object to collect status
  DeserializationError error = deserializeJson(doc, command);

  if (error) {
    Serial.println(F("{\"status\":\"Invalid JSON format\"}"));
    return;
  }

  // Validate at most one "do" operation
  if (doc.containsKey("do")) {
    if (doc["do"].is<JsonObject>()) {
      JsonObject doParams = doc["do"].as<JsonObject>();
      int doCount = 0;
      for (JsonPair kv : doParams) { doCount++; (void)kv; }
      if (doCount > 1) {
        Serial.println(F("{\"status\":\"failure\",\"error\":\"Only one do operation allowed\"}"));
        return;
      }
    } else if (!doc["do"].is<const char*>()) {
      Serial.println(F("{\"status\":\"failure\",\"error\":\"Invalid do format\"}"));
      return;
    }
  }

  // Check for get.notify validation
  bool get_notify_requested = false;
  if (doc.containsKey("get")) {
    JsonArray getArray = doc["get"].as<JsonArray>();
    for (JsonVariant value : getArray) {
      if (value.as<String>() == "notify") {
        get_notify_requested = true;
        break;
      }
    }
  }
  if (get_notify_requested) {
    bool reward_in_do = false;
    if (doc.containsKey("do") && doc["do"].is<JsonObject>()) {
      reward_in_do = doc["do"].as<JsonObject>().containsKey("reward");
    }
    if (!reward_in_do) {
      Serial.println(F("{\"status\":\"failure\",\"error\":\"get.notify only valid with do.reward\"}"));
      return;
    }
  }

  // ── Handle SET ───────────────────────────────────────────────────────────
  if (doc.containsKey("set")) {
    bool success = true;
    JsonObject setParams = doc["set"].as<JsonObject>();

    if (setParams.containsKey("flow_rate") && setParams.containsKey("adjust_flow_rate")) {
      success = false;
      responseDoc["error"] = "Use flow_rate or adjust_flow_rate, not both";
    }

    if (success && setParams.containsKey("flow_rate")) {
      float v = setParams["flow_rate"].as<float>();
      if (v > 0) {
        flow_rate = v;
        eeprom_write_float(EEPROM_FLOW_RATE_ADDR, flow_rate);
      } else {
        success = false;
        responseDoc["error"] = "Invalid flow_rate";
      }
    }

    if (success && setParams.containsKey("adjust_flow_rate")) {
      JsonObject adj = setParams["adjust_flow_rate"].as<JsonObject>();
      float expected_mls = adj["expected_mls"].as<float>();
      float actual_mls = adj["actual_mls"].as<float>();
      if (expected_mls > 0 && actual_mls > 0) {
        float old_fr = flow_rate;
        float scale = actual_mls / expected_mls;
        flow_rate = old_fr * scale;
        eeprom_write_float(EEPROM_FLOW_RATE_ADDR, flow_rate);
        responseDoc["flow_rate_old"] = old_fr;
        responseDoc["flow_rate_new"] = flow_rate;
        responseDoc["scale_factor"] = scale;
      } else {
        success = false;
        responseDoc["error"] = "Invalid adjust_flow_rate values";
      }
    }

    if (success && setParams.containsKey("purge_vol")) {
      float v = setParams["purge_vol"].as<float>();
      if (v > 0) {
        purge_vol = v;
        eeprom_write_float(EEPROM_PURGE_VOL_ADDR, purge_vol);
      } else {
        success = false;
        responseDoc["error"] = "Invalid purge_vol";
      }
    }

    if (success && setParams.containsKey("target_rps")) {
      float v = setParams["target_rps"].as<float>();
      if (v > 0 && v <= MAX_RPS) {
        target_rps = v;
        eeprom_write_float(EEPROM_TARGET_RPS_ADDR, target_rps);
        // On Uno we don't change PWM frequency — just stored for flow_rate calculations
      } else {
        success = false;
        responseDoc["error"] = "target_rps out of range";
      }
    }

    if (success && setParams.containsKey("direction")) {
      const char* dv = setParams["direction"];
      if (dv) {
        String ds = String(dv);
        ds.toLowerCase();
        if (ds == "right") {
          current_direction = DIR_RIGHT;
        } else if (ds == "left") {
          current_direction = DIR_LEFT;
        } else {
          success = false;
          responseDoc["error"] = "Invalid direction";
        }
        if (success) {
          calibration_direction = current_direction;
          eeprom_write_int16(EEPROM_DIRECTION_ADDR, (int16_t)current_direction);
        }
      } else {
        success = false;
        responseDoc["error"] = "Invalid direction";
      }
    }

    if (success && setParams.containsKey("reward_overlap_policy")) {
      const char* pv = setParams["reward_overlap_policy"];
      if (pv) {
        RewardOverlapPolicy np;
        if (parse_rop(String(pv), np)) {
          reward_overlap_policy = np;
          eeprom_write_int16(EEPROM_ROP_ADDR, (int16_t)reward_overlap_policy);
        } else {
          success = false;
          responseDoc["error"] = "Invalid reward_overlap_policy";
        }
      } else {
        success = false;
        responseDoc["error"] = "Invalid reward_overlap_policy";
      }
    }

    responseDoc["status"] = success ? "success" : "failure";
  }

  // ── Handle DO ────────────────────────────────────────────────────────────
  if (doc.containsKey("do")) {
    bool success = true;

    if (doc["do"].is<const char*>()) {
      String action = doc["do"].as<String>();

      if (action == "abort") {
        stop_pump();
        if (serial_watering) {
          reward_mls -= serial_vol;
          reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
        }
        if (ttl_watering) {
          reward_mls += ((millis() - ttl_start_time) / 1000.0) * flow_rate;
        }
        if (calibration_in_progress) {
          reward_mls -= calibration_on / 1000.0 * flow_rate;
          reward_mls += ((millis() - water_start_time) / 1000.0) * flow_rate;
        }
        serial_watering = false;
        ttl_watering = false;
        calibration_in_progress = false;
        notify_reward_complete_pending = false;

      } else if (action == "reset") {
        reset_counters();

      } else {
        success = false;
        responseDoc["error"] = "Unknown action";
      }

    } else if (doc["do"].is<JsonObject>()) {
      JsonObject doParams = doc["do"].as<JsonObject>();
      bool validCommand = false;

      if (doParams.containsKey("reward")) {
        validCommand = true;
        float rv = doParams["reward"].as<float>();
        if (rv > 0) {
          if (purging) {
            success = false; responseDoc["error"] = "Busy: purge";
          } else if (manual_watering) {
            success = false; responseDoc["error"] = "Busy: manual";
          } else if (ttl_watering) {
            success = false; responseDoc["error"] = "Busy: ttl";
          } else if (calibration_in_progress) {
            success = false; responseDoc["error"] = "Busy: calibration";
          } else if (serial_watering) {
            if (reward_overlap_policy == ROP_REJECT) {
              success = false; responseDoc["error"] = "Ignored: policy=reject";
            } else if (reward_overlap_policy == ROP_APPEND) {
              if ((int32_t)(pump_stop_time - millis()) <= 0) {
                serial_watering = false;
                stop_pump();
                start_serial_reward(rv);
              } else {
                append_serial_reward(rv);
              }
            } else {
              replace_serial_reward(rv);
            }
          } else {
            start_serial_reward(rv);
          }
          if (success && get_notify_requested) {
            notify_reward_complete_pending = true;
          }
        } else {
          success = false; responseDoc["error"] = "Invalid reward value";
        }
      }

      if (doParams.containsKey("purge")) {
        validCommand = true;
        float pa = doParams["purge"].as<float>();
        if (pa > 0) {
          purging = true;
          pump_stop_time = millis() + (unsigned long)(pa / flow_rate * 1000.0);
          start_pump();
        } else {
          success = false; responseDoc["error"] = "Invalid purge amount";
        }
      }

      if (doParams.containsKey("calibration")) {
        validCommand = true;
        JsonObject cp = doParams["calibration"].as<JsonObject>();
        int n   = cp["n"].as<int>();
        int on  = cp["on"].as<int>();
        int off = cp["off"].as<int>();
        if (n > 0 && on > 0 && off > 0) {
          handle_calibration(n, on, off);
        } else {
          success = false; responseDoc["error"] = "Invalid calibration params";
        }
      }

      if (!validCommand) {
        success = false; responseDoc["error"] = "Invalid do command";
      }
    }

    responseDoc["status"] = success ? "success" : "failure";
  }

  // ── Handle GET ───────────────────────────────────────────────────────────
  if (doc.containsKey("get")) {
    JsonArray getArray = doc["get"].as<JsonArray>();
    for (JsonVariant value : getArray) {
      String param = value.as<String>();
      if      (param == "flow_rate")      responseDoc["flow_rate"] = flow_rate;
      else if (param == "purge_vol")      responseDoc["purge_vol"] = purge_vol;
      else if (param == "target_rps")     responseDoc["target_rps"] = target_rps;
      else if (param == "reward_mls")     responseDoc["reward_mls"] = reward_mls;
      else if (param == "reward_number")  responseDoc["reward_number"] = reward_number;
      else if (param == "direction")      responseDoc["direction"] = direction_to_string(current_direction);
      else if (param == "reward_overlap_policy") responseDoc["reward_overlap_policy"] = rop_to_string(reward_overlap_policy);
      else if (param == "pump_state")     responseDoc["pump_state"] = pump_state_to_string();
      else if (param == "juice_level")    responseDoc["juice_level"] = juice_level;
      else if (param == "notify")         { /* async */ }
      else                                responseDoc[param] = "Unknown parameter";
    }
  }

  // Send response
  String response;
  serializeJson(responseDoc, response);
  Serial.println(response);
  Serial.flush();
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // Limit readStringUntil() blocking time so loop() stays responsive
  Serial.setTimeout(50);

  // Load from EEPROM (or set defaults on first run)
  if (eeprom_is_initialized()) {
    flow_rate  = eeprom_read_float(EEPROM_FLOW_RATE_ADDR, 0.5);
    purge_vol  = eeprom_read_float(EEPROM_PURGE_VOL_ADDR, 10.0);
    target_rps = eeprom_read_float(EEPROM_TARGET_RPS_ADDR, 3.0);
    int16_t sd = eeprom_read_int16(EEPROM_DIRECTION_ADDR, (int16_t)DEFAULT_DIRECTION);
    current_direction = (sd == (int16_t)DIR_RIGHT) ? DIR_RIGHT : DIR_LEFT;
    reward_overlap_policy = (RewardOverlapPolicy)eeprom_read_int16(EEPROM_ROP_ADDR, (int16_t)ROP_REPLACE);
  } else {
    flow_rate  = 0.5;
    purge_vol  = 10.0;
    target_rps = 3.0;
    current_direction = DEFAULT_DIRECTION;
    reward_overlap_policy = ROP_REPLACE;
    // Write defaults
    eeprom_write_float(EEPROM_FLOW_RATE_ADDR, flow_rate);
    eeprom_write_float(EEPROM_PURGE_VOL_ADDR, purge_vol);
    eeprom_write_float(EEPROM_TARGET_RPS_ADDR, target_rps);
    eeprom_write_int16(EEPROM_DIRECTION_ADDR, (int16_t)current_direction);
    eeprom_write_int16(EEPROM_ROP_ADDR, (int16_t)reward_overlap_policy);
    eeprom_set_initialized();
  }
  calibration_direction = current_direction;

  // Pin setup
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SWITCH_D0_PIN, INPUT_PULLUP);
  pinMode(SWITCH_D1_PIN, INPUT);  // Uno doesn't have INPUT_PULLDOWN; use external resistor
  pinMode(SWITCH_D2_PIN, INPUT);  // Same — needs external pull-down if button is active HIGH
  pinMode(TTL_WATER_PIN, INPUT);  // Same
  pinMode(JUICE_LEVEL_LOW_PIN, INPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);
  apply_direction(current_direction);

  last_juice_check_time = millis();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  check_juice_level();
  check_buttons();
  check_ttl_watering();
  check_serial_commands();
  check_for_pump_stop();
}
