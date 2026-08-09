/*
  ModubotFirmwarePID — controle PI de velocidade das rodas

  Entradas:
    W <wL> <wR>              setpoints em rad/s (malha fechada)
    V <nL> <nR>              comandos normalizados (malha aberta)
    S                        freio
    M <0|1>                  modo aberto/fechado
    K <kp> <ki> <kd>         ganhos do controlador
    F <kff> <dac_min>        parâmetros do feedforward
    C <ticksL> <ticksR>      bordas por volta
    P <0|1> [hz]             telemetria
    G                        configuração

  Saídas:
    O <dL> <dR> <dt_ms>      odometria em ticks
    T <ms> <spL> <wL> <uL> <dacL> <spR> <wR> <uR> <dacR>
    # ...                    mensagens de estado
*/

#include <Arduino.h>
#include <ctype.h>

// Pinos
#define DAC_L   25
#define DAC_R   26
#define DIR_L   19
#define DIR_R   17
#define BRAKE   16
#define SPEED_L 35
#define SPEED_R 34

// Temporização e limites
const uint32_t SER_BAUD              = 115200;
const uint32_t CTRL_PERIOD_MS        = 20;
const uint32_t ODOM_PERIOD_MS        = 50;
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

// Controle PI + feedforward
struct Config {
  float kp  = 6.0f;
  float ki  = 30.0f;
  float kd  = 0.0f;
  float kff = 5.9f;
  float dac_min = 7.0f;
  bool  closed_loop = true;
  bool  telem_on = false;
  uint32_t telem_period_ms = 40;
};
Config cfg;

struct Wheel {
  uint8_t pin_dac, pin_dir;
  float   ticks_per_rev = 91.0f;
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
  int dac_applied = 0;
  int32_t ticks_signed = 0;
  int32_t odo_last_sent = 0;
};

Wheel WL, WR;

bool     brake_on = true;
bool     wdt_tripped = false;
uint32_t last_cmd_ms = 0;
uint32_t last_ctrl_ms = 0;
uint32_t last_odom_ms = 0;
uint32_t last_telem_ms = 0;
uint32_t last_vwarn_ms = 0;

char     rxline[80];
uint8_t  rxlen = 0;

// Pulsos de velocidade
void IRAM_ATTR isrSpeedL() {
  uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&WL.mux);
  if (now - WL.last_edge_us >= GLITCH_US) { WL.ticks++; WL.last_edge_us = now; }
  portEXIT_CRITICAL_ISR(&WL.mux);
}
void IRAM_ATTR isrSpeedR() {
  uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&WR.mux);
  if (now - WR.last_edge_us >= GLITCH_US) { WR.ticks++; WR.last_edge_us = now; }
  portEXIT_CRITICAL_ISR(&WR.mux);
}

static inline void brakesSet(bool on) {
  brake_on = on;
  digitalWrite(BRAKE, on ? HIGH : LOW);
  if (on) {
    dacWrite(DAC_L, 0);  WL.dac_applied = 0;
    dacWrite(DAC_R, 0);  WR.dac_applied = 0;
  }
}

static void pidReset(Wheel &w) {
  w.integ = 0.0f;
  w.prev_meas = w.w_meas;
  w.u = 0.0f;
}

static void stopAndBrake() {
  WL.setpoint = WR.setpoint = 0.0f;
  WL.norm_cmd = WR.norm_cmd = 0.0f;
  pidReset(WL);
  pidReset(WR);
  brakesSet(true);
  wdt_tripped = true;
}

static void setWheelTarget(Wheel &w, float target) {
  if (fabsf(target) < 1e-4f || w.setpoint * target < 0.0f) {
    w.integ = 0.0f;
    w.prev_meas = w.w_meas;
  }
  w.setpoint = target;
}

// Velocidade angular pelo período entre bordas
static void sampleEncoder(Wheel &w, uint32_t now_ms) {
  uint32_t t, e;
  portENTER_CRITICAL(&w.mux);
  t = w.ticks;
  e = w.last_edge_us;
  portEXIT_CRITICAL(&w.mux);

  uint32_t dticks = t - w.prev_ticks;
  if (dticks > 0) {
    if (w.prev_edge_us != 0) {
      float dte = (float)(e - w.prev_edge_us) * 1e-6f;
      if (dte > 1e-5f) {
        float raw = (float)dticks / dte;
        w.speed_filt += SPEED_ALPHA * (raw - w.speed_filt);
      }
    }
    w.prev_ticks = t;
    w.prev_edge_us = e;
    w.last_tick_ms = now_ms;
  } else if (now_ms - w.last_tick_ms > SPEED_ZERO_TIMEOUT_MS) {
    w.speed_filt = 0.0f;
  }

  float wr = w.speed_filt / w.ticks_per_rev * (2.0f * PI);
  w.w_meas = w.dir_front ? wr : -wr;

  w.ticks_signed += (w.dir_front ? (int32_t)dticks : -(int32_t)dticks);
}

// Controlador PI; kd permanece disponível para ensaios
static float pidStep(Wheel &w, float dt) {
  float sp   = w.setpoint;
  float meas = w.w_meas;
  float err  = sp - meas;

  float u_ff = 0.0f;
  if (fabsf(sp) > 1e-4f) {
    u_ff = cfg.dac_min + cfg.kff * fabsf(sp);
    if (sp < 0.0f) u_ff = -u_ff;
  }

  float dmeas = (dt > 1e-4f) ? (meas - w.prev_meas) / dt : 0.0f;
  w.prev_meas = meas;

  float u_pred = u_ff + cfg.kp * err + w.integ - cfg.kd * dmeas;
  bool sat_hi = (u_pred >=  255.0f && err > 0.0f);
  bool sat_lo = (u_pred <= -255.0f && err < 0.0f);
  if (!w.reversing && !sat_hi && !sat_lo) {
    w.integ += cfg.ki * err * dt;
    w.integ = constrain(w.integ, -INTEG_MAX, INTEG_MAX);
  }

  float u = u_ff + cfg.kp * err + w.integ - cfg.kd * dmeas;
  u = constrain(u, -255.0f, 255.0f);

  if (fabsf(sp) < 1e-4f && fabsf(meas) < 0.05f) {
    w.integ = 0.0f;
    u = 0.0f;
  }
  w.u = u;
  return u;
}

// Atuação com rampa e inversão não bloqueante
static void writeDacRamped(Wheel &w, int dac) {
  if (dac > w.dac_applied + DAC_SLEW_PER_CYCLE)
    dac = w.dac_applied + DAC_SLEW_PER_CYCLE;
  w.dac_applied = dac;
  dacWrite(w.pin_dac, (uint8_t)dac);
}

static void applyOutput(Wheel &w, float u, uint32_t now_ms) {
  bool wantFront = (u >= 0.0f);
  int  dac = (int)constrain(fabsf(u), 0.0f, 255.0f);

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

  if (wantFront != w.dir_front) {
    if (w.dac_applied > 0 || fabsf(w.w_meas) > 0.2f) {
      w.dac_applied = 0;
      dacWrite(w.pin_dac, 0);
      w.pending_dir = wantFront;
      w.reversing = true;
      w.rev_start_ms = now_ms;
      return;
    }
    w.dir_front = wantFront;
    digitalWrite(w.pin_dir, w.dir_front ? HIGH : LOW);
  }
  writeDacRamped(w, dac);
}

static bool parse2f(char *p, float &a, float &b) {
  char *q;
  a = strtof(p, &q); if (q == p) return false; p = q;
  b = strtof(p, &q); if (q == p) return false;
  return true;
}

static void printConfig() {
  Serial.printf("# modo=%s  kp=%.3f ki=%.3f kd=%.3f\n",
                cfg.closed_loop ? "FECHADA" : "ABERTA", cfg.kp, cfg.ki, cfg.kd);
  Serial.printf("# kff=%.3f dac_min=%.1f  ticks_rev L=%.1f R=%.1f\n",
                cfg.kff, cfg.dac_min, WL.ticks_per_rev, WR.ticks_per_rev);
  Serial.printf("# telemetria=%s (%lu ms)  freio=%s\n",
                cfg.telem_on ? "ON" : "OFF", (unsigned long)cfg.telem_period_ms,
                brake_on ? "ON" : "OFF");
}

static void releaseIfBraked() {
  if (brake_on) { brakesSet(false); pidReset(WL); pidReset(WR); }
  wdt_tripped = false;
}

static void handleLine(char *s, uint32_t now_ms) {
  while (*s == ' ') s++;
  if (!*s) return;
  char cmd = toupper((unsigned char)*s);
  char *p = s + 1;
  float a, b, c;
  char *q;
  bool motion_command = false;

  switch (cmd) {
    case 'W':
      if (parse2f(p, a, b)) {
        if (!cfg.closed_loop) {
          Serial.println(F("# W ignorado: malha aberta (use M 1)"));
          break;
        }
        a = constrain(a, -W_ABS_MAX, W_ABS_MAX);
        b = constrain(b, -W_ABS_MAX, W_ABS_MAX);
        if (fabsf(a) < 1e-4f && fabsf(b) < 1e-4f) {
          stopAndBrake();
        } else {
          setWheelTarget(WL, a);
          setWheelTarget(WR, b);
          releaseIfBraked();
        }
        motion_command = true;
      }
      break;

    case 'V':
      if (parse2f(p, a, b)) {
        if (cfg.closed_loop) {
          if (now_ms - last_vwarn_ms > 2000) {
            last_vwarn_ms = now_ms;
            Serial.println(F("# V ignorado em malha fechada: use 'W <radL> <radR>' (rad/s) ou 'M 0'"));
          }
          break;
        }
        a = constrain(a, -1.0f, 1.0f);
        b = constrain(b, -1.0f, 1.0f);
        WL.norm_cmd = (fabsf(a) < DEAD_NORM) ? 0.0f : a;
        WR.norm_cmd = (fabsf(b) < DEAD_NORM) ? 0.0f : b;
        if (WL.norm_cmd == 0.0f && WR.norm_cmd == 0.0f) {
          stopAndBrake();
        } else {
          releaseIfBraked();
        }
        motion_command = true;
      }
      break;

    case 'S':
      stopAndBrake();
      break;

    case 'M':
      a = strtof(p, &q);
      if (q != p) {
        cfg.closed_loop = (a != 0.0f);
        stopAndBrake();
        Serial.printf("# malha %s\n", cfg.closed_loop ? "FECHADA" : "ABERTA");
      }
      break;

    case 'K':
      a = strtof(p, &q); if (q == p) break; p = q;
      b = strtof(p, &q); if (q == p) break; p = q;
      c = strtof(p, &q); if (q == p) break;
      if (a >= 0.0f && b >= 0.0f && c >= 0.0f) {
        cfg.kp = a; cfg.ki = b; cfg.kd = c;
        pidReset(WL); pidReset(WR);
        Serial.printf("# K kp=%.3f ki=%.3f kd=%.3f (DAC por rad/s)\n", cfg.kp, cfg.ki, cfg.kd);
      }
      break;

    case 'F':
      if (parse2f(p, a, b) && a >= 0.0f && b >= 0.0f && b <= 255.0f) {
        cfg.kff = a; cfg.dac_min = b;
        Serial.printf("# F kff=%.3f dac_min=%.1f\n", cfg.kff, cfg.dac_min);
      }
      break;

    case 'C':
      if (parse2f(p, a, b) && a > 0.0f && b > 0.0f) {
        WL.ticks_per_rev = a;
        WR.ticks_per_rev = b;
        Serial.printf("# C ticks_rev L=%.1f R=%.1f\n", a, b);
      }
      break;

    case 'P':
      a = strtof(p, &q);
      if (q != p) {
        cfg.telem_on = (a != 0.0f);
        p = q;
        b = strtof(p, &q);
        if (q != p && b >= 1.0f && b <= 100.0f)
          cfg.telem_period_ms = (uint32_t)(1000.0f / b);
        Serial.printf("# telemetria %s\n", cfg.telem_on ? "ON" : "OFF");
      }
      break;

    case 'G':
      printConfig();
      break;

    default:
      break;
  }
  if (motion_command) last_cmd_ms = now_ms;
}

static void readSerial(uint32_t now_ms) {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxline[rxlen] = '\0';
      if (rxlen) handleLine(rxline, now_ms);
      rxlen = 0;
    } else if (rxlen < sizeof(rxline) - 1) {
      rxline[rxlen++] = ch;
    } else {
      rxlen = 0;
    }
  }
}

void setup() {
  Serial.begin(SER_BAUD);
  delay(300);

  WL.pin_dac = DAC_L;  WL.pin_dir = DIR_L;
  WR.pin_dac = DAC_R;  WR.pin_dir = DIR_R;

  pinMode(BRAKE, OUTPUT);
  pinMode(DIR_L, OUTPUT_OPEN_DRAIN);
  pinMode(DIR_R, OUTPUT_OPEN_DRAIN);

  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH);
  dacWrite(DAC_L, 0);
  dacWrite(DAC_R, 0);
  brakesSet(true);

  // GPIO 34/35 exigem nível externo definido.
  pinMode(SPEED_L, INPUT);
  pinMode(SPEED_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_L), isrSpeedL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SPEED_R), isrSpeedR, CHANGE);

  uint32_t now = millis();
  last_cmd_ms = last_ctrl_ms = last_odom_ms = last_telem_ms = now;
  WL.last_tick_ms = WR.last_tick_ms = now;

  Serial.println(F("# [ModubotFirmwarePID READY]"));
  Serial.println(F("# W <radL> <radR> rad/s | V <nL> <nR> DAC (malha aberta) | S freio"));
  Serial.println(F("# M 0/1 malha | K kp ki kd | F kff dac_min | C tickL tickR | P 0/1 [hz] | G"));
  printConfig();
}

void loop() {
  uint32_t now = millis();

  readSerial(now);

  if (!wdt_tripped && (now - last_cmd_ms > WATCHDOG_MS)) {
    stopAndBrake();
    Serial.println(F("# WATCHDOG: sem comando, freio acionado"));
  }

  if (now - last_ctrl_ms >= CTRL_PERIOD_MS) {
    float dt = (float)(now - last_ctrl_ms) * 1e-3f;
    last_ctrl_ms = now;

    sampleEncoder(WL, now);
    sampleEncoder(WR, now);

    if (!brake_on) {
      float uL, uR;
      if (cfg.closed_loop) {
        uL = pidStep(WL, dt);
        uR = pidStep(WR, dt);
      } else {
        uL = WL.norm_cmd * 255.0f;  WL.u = uL;
        uR = WR.norm_cmd * 255.0f;  WR.u = uR;
      }
      applyOutput(WL, uL, now);
      applyOutput(WR, uR, now);
    }
  }

  if (cfg.telem_on && (now - last_telem_ms >= cfg.telem_period_ms)) {
    last_telem_ms = now;
    int dacLs = WL.dir_front ? WL.dac_applied : -WL.dac_applied;
    int dacRs = WR.dir_front ? WR.dac_applied : -WR.dac_applied;
    Serial.printf("T %lu %.3f %.3f %.1f %d %.3f %.3f %.1f %d\n",
                  (unsigned long)now,
                  WL.setpoint, WL.w_meas, WL.u, dacLs,
                  WR.setpoint, WR.w_meas, WR.u, dacRs);
  }

  if (now - last_odom_ms >= ODOM_PERIOD_MS) {
    uint32_t dt_ms = now - last_odom_ms;
    last_odom_ms = now;

    int32_t dL = WL.ticks_signed - WL.odo_last_sent;
    int32_t dR = WR.ticks_signed - WR.odo_last_sent;
    WL.odo_last_sent = WL.ticks_signed;
    WR.odo_last_sent = WR.ticks_signed;

    Serial.print('O');  Serial.print(' ');
    Serial.print(dL);   Serial.print(' ');
    Serial.print(dR);   Serial.print(' ');
    Serial.println(dt_ms);
  }
}
