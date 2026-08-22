/*
  ModubotFirmwarePID — controle PI multicore das rodas

  Entradas:
    W <wL> <wR>              setpoints em rad/s (malha fechada)
    V <nL> <nR>              comandos normalizados (malha aberta)
    S                        freio e limpa falha de feedback
    M <0|1>                  modo aberto/fechado
    K <kp> <ki> <kd>         ganhos do controlador
    F <kff> <dac_min>        parâmetros do feedforward
    C <ticksL> <ticksR>      bordas por volta
    P <0|1> [hz]             telemetria
    G                        configuração

  Saídas:
    O <dL> <dR> <dt_ms>      odometria em ticks
    T <ms> <spL> <wL> <uL> <dacL> <spR> <wR> <uR> <dacR>
    J <ms> <dt_us> <min_us> <max_us> <mean_us> <misses> <cycles> <stack>
    # ...                    mensagens de estado
*/

#include <Arduino.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#if CONFIG_FREERTOS_UNICORE
#error "ModubotFirmwarePID requer uma ESP32 dual-core"
#endif

#define DAC_L   26
#define DAC_R   25
#define DIR_L   17
#define DIR_R   19
#define BRAKE   16
#define SPEED_L 35
#define SPEED_R 34

const uint32_t SER_BAUD              = 115200;
const uint32_t CTRL_PERIOD_MS        = 20;
const uint32_t CTRL_PERIOD_US        = CTRL_PERIOD_MS * 1000;
const uint32_t JITTER_TOLERANCE_US   = 1000;
const uint32_t ODOM_PERIOD_MS        = 50;
const uint32_t JITTER_REPORT_MS      = 1000;
const uint32_t WATCHDOG_MS           = 600;
const uint16_t T_DIR_SETTLE_MS       = 100;
const uint16_t T_DIR_MAX_MS          = 500;
const uint32_t GLITCH_US             = 100;
const uint32_t SPEED_ZERO_TIMEOUT_MS = 150;
const float    SPEED_ALPHA           = 0.35f;
const int      DAC_SLEW_PER_CYCLE    = 12;
const float    INTEG_MAX             = 255.0f;
const float    DEAD_NORM             = 0.02f;
const float    W_ABS_MAX             = 50.0f;
const int      FEEDBACK_FAULT_DAC     = 40;
const float    FEEDBACK_FAULT_W_MIN   = 0.2f;
const uint32_t FEEDBACK_FAULT_MS      = 1200;

const BaseType_t COMM_CORE       = 0;
const BaseType_t CONTROL_CORE    = 1;
const UBaseType_t COMM_PRIORITY  = 2;
const UBaseType_t CTRL_PRIORITY  = 4;
const uint32_t COMM_STACK_BYTES  = 6144;
const uint32_t CTRL_STACK_BYTES  = 4096;

struct Config {
  float kp = 6.0f;
  float ki = 30.0f;
  float kd = 0.0f;
  float kff = 5.9f;
  float dac_min = 7.0f;
  bool closed_loop = true;
};

struct Wheel {
  uint8_t pin_dac, pin_dir;
  float ticks_per_rev = 91.0f;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t ticks = 0;
  volatile uint32_t last_edge_us = 0;
  uint32_t prev_ticks = 0;
  uint32_t prev_edge_us = 0;
  uint32_t last_tick_ms = 0;
  float speed_filt = 0.0f;
  float w_meas = 0.0f;
  float setpoint = 0.0f;
  float norm_cmd = 0.0f;
  float integ = 0.0f;
  float prev_meas = 0.0f;
  float u = 0.0f;
  bool dir_front = true;
  bool reversing = false;
  bool pending_dir = true;
  uint32_t rev_start_ms = 0;
  uint32_t no_feedback_since_ms = 0;
  int dac_applied = 0;
  int32_t ticks_signed = 0;
};

enum MotionKind : uint8_t {
  MOTION_STOP,
  MOTION_CLOSED_LOOP,
  MOTION_OPEN_LOOP
};

struct MotionPacket {
  MotionKind kind;
  float left;
  float right;
  uint32_t received_ms;
};

enum ConfigKind : uint8_t {
  CONFIG_MODE,
  CONFIG_GAINS,
  CONFIG_FEEDFORWARD,
  CONFIG_TICKS
};

struct ConfigPacket {
  ConfigKind kind;
  float a;
  float b;
  float c;
};

struct StatePacket {
  uint32_t timestamp_ms;
  int32_t ticks_left;
  int32_t ticks_right;
  float setpoint_left;
  float measured_left;
  float output_left;
  int dac_left;
  float setpoint_right;
  float measured_right;
  float output_right;
  int dac_right;
  float kp;
  float ki;
  float kd;
  float kff;
  float dac_min;
  float ticks_per_rev_left;
  float ticks_per_rev_right;
  bool closed_loop;
  bool brake_on;
  uint32_t control_dt_us;
  uint32_t control_min_us;
  uint32_t control_max_us;
  float control_mean_us;
  uint32_t deadline_misses;
  uint32_t control_cycles;
  uint32_t stack_high_watermark;
  uint32_t watchdog_trips;
  uint32_t rejected_commands;
  uint32_t feedback_faults;
  uint8_t feedback_fault_mask;
  bool feedback_fault_latched;
};

Config cfg;
Wheel WL, WR;

QueueHandle_t motion_queue = nullptr;
QueueHandle_t config_queue = nullptr;
QueueHandle_t state_queue = nullptr;
TaskHandle_t control_task_handle = nullptr;
TaskHandle_t communication_task_handle = nullptr;

bool brake_on = true;
bool watchdog_tripped = true;
uint32_t last_motion_ms = 0;
uint32_t watchdog_trips = 0;
uint32_t rejected_commands = 0;
uint32_t feedback_faults = 0;
uint8_t feedback_fault_mask = 0;
bool feedback_fault_latched = false;

char rxline[96];
uint8_t rxlen = 0;
bool discard_rx_line = false;

void IRAM_ATTR isrSpeedL() {
  uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&WL.mux);
  if (now - WL.last_edge_us >= GLITCH_US) {
    WL.ticks = WL.ticks + 1U;
    WL.last_edge_us = now;
  }
  portEXIT_CRITICAL_ISR(&WL.mux);
}

void IRAM_ATTR isrSpeedR() {
  uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&WR.mux);
  if (now - WR.last_edge_us >= GLITCH_US) {
    WR.ticks = WR.ticks + 1U;
    WR.last_edge_us = now;
  }
  portEXIT_CRITICAL_ISR(&WR.mux);
}

static inline void brakesSet(bool on) {
  brake_on = on;
  digitalWrite(BRAKE, on ? HIGH : LOW);
  if (on) {
    dacWrite(DAC_L, 0);
    dacWrite(DAC_R, 0);
    WL.dac_applied = 0;
    WR.dac_applied = 0;
  }
}

static void pidReset(Wheel &w) {
  w.integ = 0.0f;
  w.prev_meas = w.w_meas;
  w.u = 0.0f;
}

static void clearFeedbackFault() {
  feedback_fault_latched = false;
  WL.no_feedback_since_ms = 0;
  WR.no_feedback_since_ms = 0;
}

static void stopAndBrake() {
  WL.setpoint = WR.setpoint = 0.0f;
  WL.norm_cmd = WR.norm_cmd = 0.0f;
  WL.reversing = WR.reversing = false;
  WL.pending_dir = WL.dir_front;
  WR.pending_dir = WR.dir_front;
  WL.no_feedback_since_ms = 0;
  WR.no_feedback_since_ms = 0;
  pidReset(WL);
  pidReset(WR);
  brakesSet(true);
  watchdog_tripped = true;
}

static void releaseIfBraked() {
  if (brake_on) {
    brakesSet(false);
    pidReset(WL);
    pidReset(WR);
  }
  watchdog_tripped = false;
}

static void setWheelTarget(Wheel &w, float target) {
  if (fabsf(target) < 1e-4f || w.setpoint * target < 0.0f) {
    w.integ = 0.0f;
    w.prev_meas = w.w_meas;
  }
  w.setpoint = target;
}

static void sampleEncoder(Wheel &w, uint32_t now_ms) {
  uint32_t ticks, edge_us;
  portENTER_CRITICAL(&w.mux);
  ticks = w.ticks;
  edge_us = w.last_edge_us;
  portEXIT_CRITICAL(&w.mux);

  uint32_t delta_ticks = ticks - w.prev_ticks;
  if (delta_ticks > 0) {
    if (w.prev_edge_us != 0) {
      float edge_period = (float)(edge_us - w.prev_edge_us) * 1e-6f;
      if (edge_period > 1e-5f) {
        float raw = (float)delta_ticks / edge_period;
        w.speed_filt += SPEED_ALPHA * (raw - w.speed_filt);
      }
    }
    w.prev_ticks = ticks;
    w.prev_edge_us = edge_us;
    w.last_tick_ms = now_ms;
  } else if (now_ms - w.last_tick_ms > SPEED_ZERO_TIMEOUT_MS) {
    w.speed_filt = 0.0f;
  }

  float magnitude = w.speed_filt / w.ticks_per_rev * (2.0f * PI);
  w.w_meas = w.dir_front ? magnitude : -magnitude;
  w.ticks_signed += w.dir_front
      ? (int32_t)delta_ticks
      : -(int32_t)delta_ticks;
}

static float pidStep(Wheel &w, float dt) {
  if (fabsf(w.setpoint) < 1e-4f) {
    w.integ = 0.0f;
    w.prev_meas = w.w_meas;
    w.u = 0.0f;
    return 0.0f;
  }

  float error = w.setpoint - w.w_meas;
  float feedforward = 0.0f;
  if (fabsf(w.setpoint) > 1e-4f) {
    feedforward = cfg.dac_min + cfg.kff * fabsf(w.setpoint);
    if (w.setpoint < 0.0f) feedforward = -feedforward;
  }

  float derivative = dt > 1e-4f
      ? (w.w_meas - w.prev_meas) / dt
      : 0.0f;
  w.prev_meas = w.w_meas;

  float output_min = w.setpoint > 0.0f ? 0.0f : -255.0f;
  float output_max = w.setpoint > 0.0f ? 255.0f : 0.0f;

  float predicted = feedforward + cfg.kp * error
      + w.integ - cfg.kd * derivative;
  bool saturated_high = predicted >= output_max && error > 0.0f;
  bool saturated_low = predicted <= output_min && error < 0.0f;
  if (!w.reversing && !saturated_high && !saturated_low) {
    w.integ += cfg.ki * error * dt;
    w.integ = constrain(w.integ, -INTEG_MAX, INTEG_MAX);
  }

  float output = feedforward + cfg.kp * error
      + w.integ - cfg.kd * derivative;
  output = constrain(output, output_min, output_max);
  w.u = output;
  return output;
}

static void writeDacRamped(Wheel &w, int dac) {
  if (dac > w.dac_applied + DAC_SLEW_PER_CYCLE) {
    dac = w.dac_applied + DAC_SLEW_PER_CYCLE;
  }
  w.dac_applied = dac;
  dacWrite(w.pin_dac, (uint8_t)dac);
}

static void applyOutput(Wheel &w, float output, uint32_t now_ms) {
  if (fabsf(output) < 0.5f) {
    w.reversing = false;
    w.pending_dir = w.dir_front;
    writeDacRamped(w, 0);
    return;
  }

  bool front = output > 0.0f;
  int dac = (int)constrain(fabsf(output), 0.0f, 255.0f);

  if (w.reversing) {
    uint32_t elapsed = now_ms - w.rev_start_ms;
    w.dac_applied = 0;
    dacWrite(w.pin_dac, 0);
    if (elapsed < T_DIR_SETTLE_MS ||
        (fabsf(w.w_meas) > 0.2f && elapsed < T_DIR_MAX_MS)) {
      return;
    }
    w.dir_front = w.pending_dir;
    digitalWrite(w.pin_dir, w.dir_front ? HIGH : LOW);
    w.reversing = false;
  }

  if (front != w.dir_front) {
    if (w.dac_applied > 0 || fabsf(w.w_meas) > 0.2f) {
      w.dac_applied = 0;
      dacWrite(w.pin_dac, 0);
      w.pending_dir = front;
      w.reversing = true;
      w.rev_start_ms = now_ms;
      return;
    }
    w.dir_front = front;
    digitalWrite(w.pin_dir, w.dir_front ? HIGH : LOW);
  }
  writeDacRamped(w, dac);
}

static bool feedbackMissing(Wheel &w, uint32_t now_ms) {
  bool monitor = cfg.closed_loop
      && !brake_on
      && !w.reversing
      && fabsf(w.setpoint) >= FEEDBACK_FAULT_W_MIN
      && w.dac_applied >= FEEDBACK_FAULT_DAC;
  if (!monitor || fabsf(w.w_meas) >= 0.05f) {
    w.no_feedback_since_ms = 0;
    return false;
  }
  if (w.no_feedback_since_ms == 0) {
    w.no_feedback_since_ms = now_ms;
    return false;
  }
  return now_ms - w.no_feedback_since_ms >= FEEDBACK_FAULT_MS;
}

static void checkFeedbackFault(uint32_t now_ms) {
  if (feedback_fault_latched) return;
  uint8_t mask = 0;
  if (feedbackMissing(WL, now_ms)) mask |= 0x01;
  if (feedbackMissing(WR, now_ms)) mask |= 0x02;
  if (!mask) return;

  feedback_fault_mask = mask;
  feedback_fault_latched = true;
  feedback_faults++;
  stopAndBrake();
}

static void applyConfig(const ConfigPacket &packet) {
  switch (packet.kind) {
    case CONFIG_MODE:
      cfg.closed_loop = packet.a != 0.0f;
      clearFeedbackFault();
      stopAndBrake();
      break;
    case CONFIG_GAINS:
      if (packet.a >= 0.0f && packet.b >= 0.0f && packet.c >= 0.0f) {
        cfg.kp = packet.a;
        cfg.ki = packet.b;
        cfg.kd = packet.c;
        pidReset(WL);
        pidReset(WR);
      }
      break;
    case CONFIG_FEEDFORWARD:
      if (packet.a >= 0.0f && packet.b >= 0.0f && packet.b <= 255.0f) {
        cfg.kff = packet.a;
        cfg.dac_min = packet.b;
      }
      break;
    case CONFIG_TICKS:
      if (packet.a > 0.0f && packet.b > 0.0f) {
        WL.ticks_per_rev = packet.a;
        WR.ticks_per_rev = packet.b;
      }
      break;
  }
}

static void applyMotion(const MotionPacket &packet) {
  if (packet.kind == MOTION_STOP) {
    clearFeedbackFault();
    stopAndBrake();
    return;
  }

  if (feedback_fault_latched) return;

  if (packet.kind == MOTION_CLOSED_LOOP) {
    if (!cfg.closed_loop) {
      rejected_commands++;
      stopAndBrake();
      return;
    }
    float left = constrain(packet.left, -W_ABS_MAX, W_ABS_MAX);
    float right = constrain(packet.right, -W_ABS_MAX, W_ABS_MAX);
    last_motion_ms = packet.received_ms;
    if (fabsf(left) < 1e-4f && fabsf(right) < 1e-4f) {
      stopAndBrake();
      return;
    }
    setWheelTarget(WL, left);
    setWheelTarget(WR, right);
    releaseIfBraked();
    return;
  }

  if (cfg.closed_loop) {
    rejected_commands++;
    stopAndBrake();
    return;
  }
  float left = constrain(packet.left, -1.0f, 1.0f);
  float right = constrain(packet.right, -1.0f, 1.0f);
  WL.norm_cmd = fabsf(left) < DEAD_NORM ? 0.0f : left;
  WR.norm_cmd = fabsf(right) < DEAD_NORM ? 0.0f : right;
  last_motion_ms = packet.received_ms;
  if (WL.norm_cmd == 0.0f && WR.norm_cmd == 0.0f) {
    stopAndBrake();
  } else {
    releaseIfBraked();
  }
}

static void publishState(
    uint32_t now_ms,
    uint32_t dt_us,
    uint32_t min_us,
    uint32_t max_us,
    uint64_t sum_us,
    uint32_t cycles,
    uint32_t misses) {
  StatePacket state = {};
  state.timestamp_ms = now_ms;
  state.ticks_left = WL.ticks_signed;
  state.ticks_right = WR.ticks_signed;
  state.setpoint_left = WL.setpoint;
  state.measured_left = WL.w_meas;
  state.output_left = WL.u;
  state.dac_left = WL.dir_front ? WL.dac_applied : -WL.dac_applied;
  state.setpoint_right = WR.setpoint;
  state.measured_right = WR.w_meas;
  state.output_right = WR.u;
  state.dac_right = WR.dir_front ? WR.dac_applied : -WR.dac_applied;
  state.kp = cfg.kp;
  state.ki = cfg.ki;
  state.kd = cfg.kd;
  state.kff = cfg.kff;
  state.dac_min = cfg.dac_min;
  state.ticks_per_rev_left = WL.ticks_per_rev;
  state.ticks_per_rev_right = WR.ticks_per_rev;
  state.closed_loop = cfg.closed_loop;
  state.brake_on = brake_on;
  state.control_dt_us = dt_us;
  state.control_min_us = min_us;
  state.control_max_us = max_us;
  state.control_mean_us = cycles > 0 ? (float)sum_us / cycles : 0.0f;
  state.deadline_misses = misses;
  state.control_cycles = cycles;
  state.stack_high_watermark = uxTaskGetStackHighWaterMark(nullptr);
  state.watchdog_trips = watchdog_trips;
  state.rejected_commands = rejected_commands;
  state.feedback_faults = feedback_faults;
  state.feedback_fault_mask = feedback_fault_mask;
  state.feedback_fault_latched = feedback_fault_latched;
  xQueueOverwrite(state_queue, &state);
}

static void controlTask(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  uint64_t last_cycle_us = esp_timer_get_time();
  uint64_t sum_us = 0;
  uint32_t min_us = UINT32_MAX;
  uint32_t max_us = 0;
  uint32_t cycles = 0;
  uint32_t misses = 0;

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    uint64_t now_us = esp_timer_get_time();
    uint32_t dt_us = (uint32_t)(now_us - last_cycle_us);
    last_cycle_us = now_us;
    uint32_t now_ms = millis();

    cycles++;
    sum_us += dt_us;
    if (dt_us < min_us) min_us = dt_us;
    if (dt_us > max_us) max_us = dt_us;
    if (dt_us > CTRL_PERIOD_US + JITTER_TOLERANCE_US) misses++;

    ConfigPacket config_packet;
    while (xQueueReceive(config_queue, &config_packet, 0) == pdPASS) {
      applyConfig(config_packet);
    }

    MotionPacket motion_packet;
    if (xQueueReceive(motion_queue, &motion_packet, 0) == pdPASS) {
      applyMotion(motion_packet);
    }

    if (!watchdog_tripped && now_ms - last_motion_ms > WATCHDOG_MS) {
      watchdog_trips++;
      stopAndBrake();
    }

    sampleEncoder(WL, now_ms);
    sampleEncoder(WR, now_ms);

    if (!brake_on) {
      float output_left;
      float output_right;
      if (cfg.closed_loop) {
        float dt = dt_us > 100000 ? 0.02f : dt_us * 1e-6f;
        output_left = pidStep(WL, dt);
        output_right = pidStep(WR, dt);
      } else {
        output_left = WL.norm_cmd * 255.0f;
        output_right = WR.norm_cmd * 255.0f;
        WL.u = output_left;
        WR.u = output_right;
      }
      applyOutput(WL, output_left, now_ms);
      applyOutput(WR, output_right, now_ms);
      checkFeedbackFault(now_ms);
    }

    publishState(now_ms, dt_us, min_us, max_us, sum_us, cycles, misses);
  }
}

static bool parse2f(char *text, float &a, float &b) {
  char *end;
  a = strtof(text, &end);
  if (end == text) return false;
  text = end;
  b = strtof(text, &end);
  return end != text;
}

static bool enqueueConfig(ConfigKind kind, float a, float b, float c) {
  ConfigPacket packet = {kind, a, b, c};
  if (xQueueSend(config_queue, &packet, 0) == pdPASS) return true;
  Serial.println(F("# fila de configuração cheia"));
  return false;
}

static void enqueueMotion(MotionKind kind, float left, float right) {
  MotionPacket packet = {kind, left, right, millis()};
  xQueueOverwrite(motion_queue, &packet);
}

static void printConfig(const StatePacket &state) {
  Serial.printf("# modo=%s kp=%.3f ki=%.3f kd=%.3f\n",
                state.closed_loop ? "FECHADA" : "ABERTA",
                state.kp, state.ki, state.kd);
  Serial.printf("# kff=%.3f dac_min=%.1f ticks_rev L=%.1f R=%.1f\n",
                state.kff, state.dac_min,
                state.ticks_per_rev_left, state.ticks_per_rev_right);
  Serial.printf("# freio=%s controle=Core%d/%luHz comunicação=Core%d\n",
                state.brake_on ? "ON" : "OFF",
                (int)CONTROL_CORE, 1000UL / CTRL_PERIOD_MS, (int)COMM_CORE);
  Serial.printf("# falha_feedback=%s ultima_mascara=%u ocorrencias=%lu\n",
                state.feedback_fault_latched ? "TRAVADA" : "OK",
                (unsigned int)state.feedback_fault_mask,
                (unsigned long)state.feedback_faults);
}

static void handleLine(
    char *line,
    bool &telemetry_on,
    uint32_t &telemetry_period_ms,
    bool &print_config_pending) {
  while (*line == ' ') line++;
  if (!*line) return;
  char command = toupper((unsigned char)*line);
  char *text = line + 1;
  float a, b, c;
  char *end;

  switch (command) {
    case 'W':
      if (parse2f(text, a, b)) enqueueMotion(MOTION_CLOSED_LOOP, a, b);
      break;
    case 'V':
      if (parse2f(text, a, b)) enqueueMotion(MOTION_OPEN_LOOP, a, b);
      break;
    case 'S':
      enqueueMotion(MOTION_STOP, 0.0f, 0.0f);
      break;
    case 'M':
      a = strtof(text, &end);
      if (end != text) {
        enqueueMotion(MOTION_STOP, 0.0f, 0.0f);
        enqueueConfig(CONFIG_MODE, a, 0.0f, 0.0f);
      }
      break;
    case 'K':
      a = strtof(text, &end); if (end == text) break; text = end;
      b = strtof(text, &end); if (end == text) break; text = end;
      c = strtof(text, &end); if (end == text) break;
      enqueueConfig(CONFIG_GAINS, a, b, c);
      break;
    case 'F':
      if (parse2f(text, a, b)) {
        enqueueConfig(CONFIG_FEEDFORWARD, a, b, 0.0f);
      }
      break;
    case 'C':
      if (parse2f(text, a, b)) enqueueConfig(CONFIG_TICKS, a, b, 0.0f);
      break;
    case 'P':
      a = strtof(text, &end);
      if (end != text) {
        telemetry_on = a != 0.0f;
        text = end;
        b = strtof(text, &end);
        if (end != text && b >= 1.0f && b <= 100.0f) {
          telemetry_period_ms = (uint32_t)(1000.0f / b);
        }
        Serial.printf("# telemetria %s\n", telemetry_on ? "ON" : "OFF");
      }
      break;
    case 'G':
      print_config_pending = true;
      break;
    default:
      break;
  }
}

static void readSerial(
    bool &telemetry_on,
    uint32_t &telemetry_period_ms,
    bool &print_config_pending) {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!discard_rx_line) {
        rxline[rxlen] = '\0';
        if (rxlen) {
          handleLine(rxline, telemetry_on, telemetry_period_ms,
                     print_config_pending);
        }
      }
      rxlen = 0;
      discard_rx_line = false;
    } else if (!discard_rx_line && rxlen < sizeof(rxline) - 1) {
      rxline[rxlen++] = ch;
    } else {
      discard_rx_line = true;
    }
  }
}

static void communicationTask(void *) {
  bool telemetry_on = false;
  bool print_config_pending = true;
  uint32_t telemetry_period_ms = 40;
  uint32_t last_odom_ms = millis();
  uint32_t last_telemetry_ms = millis();
  uint32_t last_jitter_ms = millis();
  int32_t last_odom_left = 0;
  int32_t last_odom_right = 0;
  uint32_t last_watchdog_trips = 0;
  uint32_t last_rejected_commands = 0;
  uint32_t last_feedback_faults = 0;
  StatePacket state = {};
  bool have_state = false;

  for (;;) {
    readSerial(telemetry_on, telemetry_period_ms, print_config_pending);

    StatePacket received;
    if (xQueueReceive(state_queue, &received, 0) == pdPASS) {
      state = received;
      have_state = true;
    }

    uint32_t now = millis();
    if (have_state && print_config_pending) {
      printConfig(state);
      print_config_pending = false;
    }

    if (have_state && state.watchdog_trips != last_watchdog_trips) {
      last_watchdog_trips = state.watchdog_trips;
      Serial.println(F("# WATCHDOG: sem comando, freio acionado"));
    }
    if (have_state && state.rejected_commands != last_rejected_commands) {
      last_rejected_commands = state.rejected_commands;
      Serial.println(F("# comando incompatível com o modo de controle"));
    }
    if (have_state && state.feedback_faults != last_feedback_faults) {
      last_feedback_faults = state.feedback_faults;
      const char *side = state.feedback_fault_mask == 0x03
          ? "L+R"
          : (state.feedback_fault_mask == 0x01 ? "L" : "R");
      Serial.printf(
          "# FALHA_FEEDBACK_%s: DAC sem pulsos; freio acionado; envie S\n",
          side);
    }

    if (have_state && now - last_odom_ms >= ODOM_PERIOD_MS) {
      uint32_t dt_ms = now - last_odom_ms;
      last_odom_ms = now;
      int32_t delta_left = state.ticks_left - last_odom_left;
      int32_t delta_right = state.ticks_right - last_odom_right;
      last_odom_left = state.ticks_left;
      last_odom_right = state.ticks_right;
      Serial.printf("O %ld %ld %lu\n",
                    (long)delta_left, (long)delta_right,
                    (unsigned long)dt_ms);
    }

    if (have_state && telemetry_on &&
        now - last_telemetry_ms >= telemetry_period_ms) {
      last_telemetry_ms = now;
      Serial.printf("T %lu %.3f %.3f %.1f %d %.3f %.3f %.1f %d\n",
                    (unsigned long)state.timestamp_ms,
                    state.setpoint_left, state.measured_left,
                    state.output_left, state.dac_left,
                    state.setpoint_right, state.measured_right,
                    state.output_right, state.dac_right);
    }

    if (have_state && now - last_jitter_ms >= JITTER_REPORT_MS) {
      last_jitter_ms = now;
      Serial.printf("J %lu %lu %lu %lu %.1f %lu %lu %lu\n",
                    (unsigned long)state.timestamp_ms,
                    (unsigned long)state.control_dt_us,
                    (unsigned long)state.control_min_us,
                    (unsigned long)state.control_max_us,
                    state.control_mean_us,
                    (unsigned long)state.deadline_misses,
                    (unsigned long)state.control_cycles,
                    (unsigned long)state.stack_high_watermark);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup() {
  Serial.begin(SER_BAUD);
  delay(300);

  WL.pin_dac = DAC_L;
  WL.pin_dir = DIR_L;
  WR.pin_dac = DAC_R;
  WR.pin_dir = DIR_R;

  pinMode(BRAKE, OUTPUT);
  pinMode(DIR_L, OUTPUT_OPEN_DRAIN);
  pinMode(DIR_R, OUTPUT_OPEN_DRAIN);
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH);
  dacWrite(DAC_L, 0);
  dacWrite(DAC_R, 0);
  brakesSet(true);

  pinMode(SPEED_L, INPUT);
  pinMode(SPEED_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_L), isrSpeedL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SPEED_R), isrSpeedR, CHANGE);

  motion_queue = xQueueCreate(1, sizeof(MotionPacket));
  config_queue = xQueueCreate(8, sizeof(ConfigPacket));
  state_queue = xQueueCreate(1, sizeof(StatePacket));
  if (!motion_queue || !config_queue || !state_queue) {
    Serial.println(F("# ERRO: não foi possível criar as filas"));
    while (true) delay(1000);
  }

  uint32_t now = millis();
  last_motion_ms = now;
  WL.last_tick_ms = now;
  WR.last_tick_ms = now;

  Serial.println(F("# [ModubotFirmwarePID READY]"));
  Serial.println(F("# controle=Core1/50Hz/prio4 comunicação=Core0/prio2"));
  Serial.println(F("# W/V | S | M 0/1 | K kp ki kd | F kff dac_min | C ticks | P | G"));

  BaseType_t communication_created = xTaskCreatePinnedToCore(
      communicationTask, "Communication", COMM_STACK_BYTES,
      nullptr, COMM_PRIORITY, &communication_task_handle, COMM_CORE);
  BaseType_t control_created = xTaskCreatePinnedToCore(
      controlTask, "WheelControl", CTRL_STACK_BYTES,
      nullptr, CTRL_PRIORITY, &control_task_handle, CONTROL_CORE);

  if (communication_created != pdPASS || control_created != pdPASS) {
    brakesSet(true);
    Serial.println(F("# ERRO: não foi possível criar as tarefas"));
    while (true) delay(1000);
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
