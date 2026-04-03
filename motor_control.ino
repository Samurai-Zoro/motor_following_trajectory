/*
 * Motor Control with ADC Current & Voltage Sensing
 * For STM32 Nucleo-F401RE
 * 
 * NO I2C - All analog sensing (immune to motor EMI!)
 * 
 * CURRENT SENSING (ACS712-5A):
 *   VCC  -> 5V (Nucleo)
 *   GND  -> GND (Nucleo)
 *   OUT  -> A2 (Nucleo)
 *   IP+  <- Motor Driver OUT1
 *   IP-  -> Motor +
 * 
 * VOLTAGE SENSING (Voltage Divider + RC Filter):
 *   Motor Driver OUT1 --[10kΩ]--+--[3.3kΩ]-- Motor Driver OUT2
 *                               |
 *                            [1kΩ]
 *                               |
 *                               +---------> A4 (Nucleo)
 *                               |
 *                            [1µF]
 *                               |
 *                              GND
 * 
 * LIMIT SWITCH (NC - Normally Closed):
 *   D13 <- Switch COM
 *   GND <- Switch NC
 *   (Uses internal pullup - reads HIGH when pressed/open, LOW when closed)
 */

#define STREAM_BINARY 1   // 1=binary frames, 0=CSV lines

#include <Arduino.h>
#include "HardwareTimer.h"
#include "target_positions.h"
#include "quantized_rotation_deg.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ============================================
// LIMIT SWITCH (NC Configuration)
// ============================================
#define LIMIT_SWITCH_PIN  13
#define LIMIT_DEBOUNCE_MS 50

// Slow reset parameters (no PID, just fixed slow PWM)
#define SLOW_RESET_PWM_A        120      // PWM for linear motor A (may need more torque)
#define SLOW_RESET_PWM_B        20      // PWM for rotation motor B
#define SLOW_RESET_TOLERANCE_A  0.15f   // cm - stop when within this of target
#define SLOW_RESET_TOLERANCE_B  1.5f    // deg - stop when within this of target

volatile bool limit_switch_triggered = false;
volatile uint32_t last_limit_press_ms = 0;
volatile bool slow_reset_active = false;

// ============================================
// CURRENT SENSING (ACS712)
// ============================================
#define CURRENT_SENSE_PIN  A2

// ACS712-5A: 185mV/A, 2.5V at 0A
// ACS712-20A: 100mV/A, 2.5V at 0A  
// ACS712-30A: 66mV/A, 2.5V at 0A
#define ACS712_MV_PER_AMP    185.0f   // For ACS712-5A
#define ACS712_ZERO_POINT_MV 2500.0f  // 2.5V at 0 amps

// ============================================
// VOLTAGE SENSING (Voltage Divider)
// ============================================
#define VOLTAGE_SENSE_PIN  A4

// Voltage divider: OUT1 --[R1]--+--[R2]-- OUT2
//                               |
//                            [R3]--+--[C1]-- GND
//                                  |
//                                  A4
//
// R1 = 10kΩ, R2 = 3.3kΩ (voltage divider)
// R3 = 100Ω, C1 = 10µF (RC filter for 20kHz PWM)
// RC cutoff = 1/(2π×100×10µF) ≈ 159Hz - smooths 20kHz PWM
//
// Divider ratio = R2 / (R1 + R2) = 3.3 / 13.3 = 0.248
// Multiplier = (R1 + R2) / R2 = 13.3 / 3.3 = 4.03
#define VOLTAGE_DIVIDER_R1     5600.0f  // 5.6kΩ
#define VOLTAGE_DIVIDER_R2     2000.0f   // 2kΩ
#define VOLTAGE_MULTIPLIER     ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)

// ============================================
// ADC Configuration
// ============================================
#define ADC_MAX_VALUE    4095.0f   // 12-bit ADC
#define ADC_REF_MV       3300.0f   // 3.3V reference

// Convert ADC reading to millivolts
#define ADC_TO_MV(adc)   ((adc) * ADC_REF_MV / ADC_MAX_VALUE)

// Global readings (updated in ISR)
volatile int16_t measured_current_mA = 0;
volatile int16_t measured_voltage_mV = 0;

// ============================================
// Original Configuration
// ============================================
#ifndef NUM_TARGETS
constexpr int num_targets = sizeof(target_cm_array) / sizeof(target_cm_array[0]);
#endif
static_assert((sizeof(quantized_rotation_deg) / sizeof(quantized_rotation_deg[0])) == num_targets,
              "target_cm_array and quantized_rotation_deg must be same length");

#define TICKS_PER_CM_A  (644880.0 / 201.5)
#define TICKS_PER_DEG_B (87750.0f / 360.0f)

HardwareTimer *sharedTimer = new HardwareTimer(TIM3);

// ---- Pins ----
#define ENCA_A 2
#define ENCB_A 3
#define PWM_A  6
#define IN1_A  12
#define IN2_A  9

#define ENCA_B 11
#define ENCB_B 8
#define PWM_B  10
#define IN1_B  5
#define IN2_B  4

#define LED_PIN   LED_BUILTIN

#define PIEZO_PIN A3
#define PIEZO_USE_BASELINE_HP 1
#define PIEZO_FIXED_OFFSET    2048
#define PIEZO_HP_ALPHA        0.031f

#define ENC_C_PIN_A A0
#define ENC_C_PIN_B A1
#define ENC_C_USE_PULLUPS 1
#define ENC_C_DIR_INVERT  0

// ---- State ----
volatile long  pos_ticks_A = 0, pos_ticks_B = 0;
volatile float current_cm_A  = 0.0f;
volatile float current_deg_B = 0.0f;

volatile long     pos_ticks_C = 0;
volatile uint8_t  enc_prev_C  = 0;

bool    stopped_A = false, stopped_B = false;
volatile float actual_frequency_hz = 0;

int current_target_index = 0;

const uint16_t KICKA_MS = 40, KICKB_MS = 40;
const float    KICKA_PWM = 18.0f, KICKB_PWM = 18.0f;

constexpr float LOOP_HZ = 1000.0f;
constexpr float DT_SEC  = 1.0f / LOOP_HZ;

constexpr uint32_t PRE_HOLD_MS  = 5000;
constexpr uint32_t POST_HOLD_MS = 5000;

const float KP_A_CM = 600.0f, KI_A_CM = 2.0f, KD_A_CM = 10.0f;
float KP_B_DEG = 400.0f, KI_B_DEG = 0.0f, KD_B_DEG = 1.0f;
const float I_LIM_A_CM  = 20000.0f / TICKS_PER_CM_A;
const float I_LIM_B_DEG = 20000.0f / TICKS_PER_DEG_B;

// ===== Payload (30 bytes) =====
struct __attribute__((packed)) SampleLite {
  uint32_t t_ms;
  float    actA_cm;
  float    tgtA_cm;
  float    actB_deg;
  float    tgtB_deg;
  int16_t  piezo_raw;
  int16_t  pwmA_255;
  int16_t  pwmB_255;     // Added: PWM for motor B
  int16_t  current_mA;   // From ACS712
  int16_t  voltage_mV;   // From voltage divider
};
static_assert(sizeof(SampleLite) == 30, "SampleLite must be 30 bytes");

// ==== Ring buffer ====
constexpr uint16_t RB_SIZE = 2048;
static    SampleLite rb[RB_SIZE];
volatile uint16_t  rb_head = 0, rb_tail = 0;

volatile uint16_t max_rb_usage = 0;
volatile uint32_t overflow_count = 0;

volatile bool      header_sent = false;
volatile bool      streaming   = false;
static volatile uint32_t t0_us = 0;

// ---- Quadrature decoding ----
#define INVERT_DIR_A 0
#define INVERT_DIR_B 0
static const int8_t QUAD[16] = {
  0,+1,-1,0, -1,0,0,+1, +1,0,0,-1, 0,-1,+1,0
};
volatile uint8_t enc_prev_A = 0;
volatile uint8_t enc_prev_B = 0;

inline uint8_t readAB_A() { return (digitalRead(ENCA_A) << 1) | digitalRead(ENCB_A); }
inline uint8_t readAB_B() { return (digitalRead(ENCA_B) << 1) | digitalRead(ENCB_B); }
inline uint8_t readAB_C() { return (digitalRead(ENC_C_PIN_A) << 1) | digitalRead(ENC_C_PIN_B); }

enum Phase : uint8_t { PRE_HOLD=0, RUN=1, POST_HOLD=2, DONE=3 };
volatile Phase phase = PRE_HOLD;
volatile uint32_t phase_ticks_remaining = PRE_HOLD_MS;

// ---- Prototypes ----
void PID_ISR();
void setMotorA(int dir, int pwmVal);
void setMotorB(int dir, int pwmVal);
void readEncoderA();
void readEncoderB();
void readEncoderC();
void limitSwitchISR();
void slowResetRoutine();

#if STREAM_BINARY
struct __attribute__((packed)) BinFrame {
  uint16_t magic;
  uint16_t len;
  uint32_t seq;
  SampleLite payload;
  uint16_t crc;
};
static_assert(sizeof(BinFrame) == (2+2+4+sizeof(SampleLite)+2), "BinFrame size unexpected");

static inline uint16_t crc16_ccitt(const uint8_t* data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}
#endif

// PWM - 20kHz (ultrasonic, no audible whine)
HardwareTimer *pwmTimA = nullptr;
HardwareTimer *pwmTimB = nullptr;
static const uint32_t PWM_FREQ_HZ = 20000;

// ============================================
// Current & Voltage Reading Functions
// ============================================

// Read current from ACS712 (called in ISR - very fast!)
int16_t readCurrent_mA() {
  int32_t adc = analogRead(CURRENT_SENSE_PIN);
  float mv = ADC_TO_MV(adc);
  float ma = (mv - ACS712_ZERO_POINT_MV) / ACS712_MV_PER_AMP * 1000.0f;
  return (int16_t)ma*(-1);
}

// Read voltage from voltage divider (called in ISR - very fast!)
int16_t readVoltage_mV() {
  int32_t adc = analogRead(VOLTAGE_SENSE_PIN);
  float mv_adc = ADC_TO_MV(adc);
  float mv_motor = mv_adc * VOLTAGE_MULTIPLIER;
  return (int16_t)mv_motor;
}

// Calibrate ACS712 zero point (call with no current flowing)
void calibrateCurrentZero() {
  Serial.println("[CAL] Calibrating current zero point...");
  Serial.println("[CAL] Make sure motor is OFF!");
  delay(500);
  
  int32_t sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(CURRENT_SENSE_PIN);
    delay(5);
  }
  float avg_mv = ADC_TO_MV(sum / 100.0f);
  
  Serial.print("[CAL] Measured zero point: ");
  Serial.print(avg_mv, 1);
  Serial.print(" mV (expected: ");
  Serial.print(ACS712_ZERO_POINT_MV, 1);
  Serial.println(" mV)");
  
  if (abs(avg_mv - ACS712_ZERO_POINT_MV) > 200) {
    Serial.println("[CAL] WARNING: Large offset! Check wiring or adjust ACS712_ZERO_POINT_MV");
  } else {
    Serial.println("[CAL] Zero point OK");
  }
}

// ============================================
// Limit Switch Functions
// ============================================

// ISR for limit switch - NC config means HIGH = pressed (circuit open)
void limitSwitchISR() {
  uint32_t now = millis();
  if ((now - last_limit_press_ms) > LIMIT_DEBOUNCE_MS) {
    if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
      limit_switch_triggered = true;
      last_limit_press_ms = now;
    }
  }
}

// Slow reset routine - runs in loop(), NO PID, just slow fixed PWM
void slowResetRoutine() {
  // Pause the PID timer during slow reset
  sharedTimer->pause();
  
  Serial.println("[RESET] Starting slow reset to initial position...");
  Serial.print("[RESET] Target A: ");
  Serial.print(target_cm_array[0]);
  Serial.print(" cm, Target B: ");
  Serial.print(quantized_rotation_deg[0]);
  Serial.println(" deg");
  Serial.print("[RESET] Using PWM - Motor A: ");
  Serial.print(SLOW_RESET_PWM_A);
  Serial.print(", Motor B: ");
  Serial.println(SLOW_RESET_PWM_B);
  
  float target_cm_A  = target_cm_array[0];
  float target_deg_B = quantized_rotation_deg[0];
  
  bool a_done = false;
  bool b_done = false;
  
  // Phase 1: Move slowly to initial position
  while (!a_done || !b_done) {
    // Read current positions (encoder interrupts still active)
    noInterrupts();
    float actual_cm_A  = (float)pos_ticks_A / TICKS_PER_CM_A;
    float actual_deg_B = (float)pos_ticks_B / TICKS_PER_DEG_B;
    interrupts();
    
    float err_A = target_cm_A - actual_cm_A;
    float err_B = target_deg_B - actual_deg_B;
    
    // Motor A - simple direction control with fixed low PWM
    if (fabsf(err_A) < SLOW_RESET_TOLERANCE_A) {
      setMotorA(0, 0);  // Stop
      if (!a_done) {
        Serial.println("[RESET] Motor A arrived");
        a_done = true;
      }
    } else {
      int dir_A = (err_A > 0) ? +1 : -1;
      setMotorA(dir_A, SLOW_RESET_PWM_A);
    }
    
    // Motor B - simple direction control with fixed low PWM
    if (fabsf(err_B) < SLOW_RESET_TOLERANCE_B) {
      setMotorB(0, 0);  // Stop
      if (!b_done) {
        Serial.println("[RESET] Motor B arrived");
        b_done = true;
      }
    } else {
      int dir_B = (err_B > 0) ? +1 : -1;
      setMotorB(dir_B, SLOW_RESET_PWM_B);
    }
    
    delay(1);  // Small delay
  }
  
  Serial.println("[RESET] Arrived at initial position. Holding forever...");
  Serial.println("[RESET] (Reset board or press switch again to restart)");
  
  // Phase 2: Hold position forever using simple control (no PID)
  // This loop never exits - motors hold at initial position
  while (true) {
    noInterrupts();
    float actual_cm_A  = (float)pos_ticks_A / TICKS_PER_CM_A;
    float actual_deg_B = (float)pos_ticks_B / TICKS_PER_DEG_B;
    interrupts();
    
    float err_A = target_cm_A - actual_cm_A;
    float err_B = target_deg_B - actual_deg_B;
    
    // Hold Motor A at position with low PWM
    if (fabsf(err_A) < SLOW_RESET_TOLERANCE_A) {
      setMotorA(0, 0);
    } else {
      int dir_A = (err_A > 0) ? +1 : -1;
      setMotorA(dir_A, SLOW_RESET_PWM_A);
    }
    
    // Hold Motor B at position with low PWM
    if (fabsf(err_B) < SLOW_RESET_TOLERANCE_B) {
      setMotorB(0, 0);
    } else {
      int dir_B = (err_B > 0) ? +1 : -1;
      setMotorB(dir_B, SLOW_RESET_PWM_B);
    }
    
    delay(1);
  }
}

// ============================================
// Setup
// ============================================
void setup() {
  Serial.begin(2000000);
  delay(2000);

#if defined(ARDUINO_ARCH_STM32)
  analogReadResolution(12);
#endif

  pinMode(ENCA_A, INPUT);
  pinMode(ENCB_A, INPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);

  pinMode(ENCA_B, INPUT);
  pinMode(ENCB_B, INPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

#if ENC_C_USE_PULLUPS
  pinMode(ENC_C_PIN_A, INPUT_PULLUP);
  pinMode(ENC_C_PIN_B, INPUT_PULLUP);
#else
  pinMode(ENC_C_PIN_A, INPUT);
  pinMode(ENC_C_PIN_B, INPUT);
#endif

  pinMode(PIEZO_PIN, INPUT);
  pinMode(CURRENT_SENSE_PIN, INPUT);
  pinMode(VOLTAGE_SENSE_PIN, INPUT);

  // Setup limit switch - NC config with internal pullup
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING);

  enc_prev_A = readAB_A();
  enc_prev_B = readAB_B();
  enc_prev_C = readAB_C();

  attachInterrupt(digitalPinToInterrupt(ENCA_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), readEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_B), readEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C_PIN_A), readEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C_PIN_B), readEncoderC, CHANGE);

  Serial.println("=====================================");
  Serial.println("  ADC Current & Voltage Sensing");
  Serial.println("  (No I2C - EMI Immune!)");
  Serial.println("=====================================");
  Serial.println();
  Serial.println("Connections:");
  Serial.println("  A2 = ACS712 OUT (current)");
  Serial.println("  A4 = Voltage divider (voltage)");
  Serial.println("  D13 = Limit switch (NC config)");
  Serial.println();
  Serial.print("Current sensor: ACS712-5A (");
  Serial.print(ACS712_MV_PER_AMP, 0);
  Serial.println(" mV/A)");
  Serial.print("Voltage divider: x");
  Serial.print(VOLTAGE_MULTIPLIER, 2);
  Serial.print(" (max ");
  Serial.print(3.3f * VOLTAGE_MULTIPLIER, 1);
  Serial.println("V)");
  Serial.print("PWM frequency: ");
  Serial.print(PWM_FREQ_HZ);
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Limit switch: D13 (NC - press to slow-reset to initial position)");
  Serial.println();
  
  // Calibrate current sensor
  calibrateCurrentZero();
  Serial.println();

  // PWM setup
  pwmTimA = new HardwareTimer(TIM2);
  pwmTimA->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PWM_A);
  pwmTimA->setOverflow(PWM_FREQ_HZ, HERTZ_FORMAT);
  pwmTimA->setCaptureCompare(3, 0, PERCENT_COMPARE_FORMAT);
  pwmTimA->resume();

  pwmTimB = new HardwareTimer(TIM4);
  pwmTimB->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_B);
  pwmTimB->setOverflow(PWM_FREQ_HZ, HERTZ_FORMAT);
  pwmTimB->setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
  pwmTimB->resume();

  sharedTimer->setOverflow(1000, MICROSEC_FORMAT);
  sharedTimer->attachInterrupt(PID_ISR);
  sharedTimer->resume();
  
  Serial.println("[INFO] System ready!");
  Serial.println();
}

// ============================================
// Main Loop
// ============================================
void loop() {
  // Check if limit switch was triggered - run slow reset routine
  if (limit_switch_triggered) {
    slow_reset_active = true;
    slowResetRoutine();
  }

  // Debug print
  static uint32_t lastPrint_ms = 0;
  if (millis() - lastPrint_ms >= 100) {
    lastPrint_ms = millis();
    noInterrupts();
    long a = pos_ticks_A;
    long b = pos_ticks_B;
    long c = pos_ticks_C;
    int16_t curr = measured_current_mA;
    int16_t volt = measured_voltage_mV;
    interrupts();
    
    Serial.print("A="); Serial.print(a);
    Serial.print(" B="); Serial.print(b);
    Serial.print(" C="); Serial.print(c);
    Serial.print(" | I="); Serial.print(curr); Serial.print("mA");
    Serial.print(" V="); Serial.print(volt); Serial.println("mV");
  }

  // Streaming logic
  if (!streaming && Serial) {
    rb_head = rb_tail = 0;
    t0_us = micros();
    streaming = true;
    header_sent = false;
    max_rb_usage = 0;
    overflow_count = 0;
  }

  if (streaming && !header_sent && Serial) {
#if STREAM_BINARY
    static const char hdr[] = "# bin_stream v12 (magic=0xA55A, 30B, ADC current+voltage+pwmB)\n";
#else
    static const char hdr[] = "t_ms,actA_cm,tgtA_cm,actB_deg,tgtB_deg,piezo_adc,pwmA_255,pwmB_255,current_mA,voltage_mV\n";
#endif
    Serial.write((const uint8_t*)hdr, sizeof(hdr) - 1);
    header_sent = true;
  }

  while (streaming && (rb_tail != rb_head)) {
#if STREAM_BINARY
    if (Serial.availableForWrite() < (int)sizeof(BinFrame)) break;
    SampleLite s = rb[rb_tail]; rb_tail = (rb_tail + 1) % RB_SIZE;
    static uint32_t seq = 0;
    BinFrame f;
    f.magic = 0xA55A;
    f.len   = sizeof(SampleLite);
    f.seq   = seq++;
    f.payload = s;
    const uint8_t* p = (const uint8_t*)&f.len;
    f.crc = crc16_ccitt(p, sizeof(f.len) + sizeof(f.seq) + sizeof(f.payload));
    Serial.write((const uint8_t*)&f, sizeof(f));
#else
    SampleLite s = rb[rb_tail]; rb_tail = (rb_tail + 1) % RB_SIZE;
    char line[180];
    int n = snprintf(line, sizeof(line),
                     "%lu,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d\n",
                     (unsigned long)s.t_ms,
                     s.actA_cm, s.tgtA_cm,
                     s.actB_deg, s.tgtB_deg,
                     (int)s.piezo_raw,
                     (int)s.pwmA_255,
                     (int)s.pwmB_255,
                     (int)s.current_mA,
                     (int)s.voltage_mV);
    if (n > 0 && Serial.availableForWrite() >= n) {
      Serial.write((const uint8_t*)line, n);
    } else break;
#endif
  }
}

// ===== PID ISR (1kHz) =====
void PID_ISR() {
  if (stopped_A && stopped_B) return;

  static uint32_t prevMicros = 0;
  const uint32_t now = micros();
  const uint32_t interval = now - prevMicros;
  prevMicros = now;

  if (interval > 0) actual_frequency_hz = 1000000.0f / interval;

  int idx = current_target_index;
  if (phase == PRE_HOLD) idx = 0;
  else if (phase == POST_HOLD || phase == DONE) idx = num_targets - 1;

  float target_cm_A  = target_cm_array[idx];
  float target_deg_B = quantized_rotation_deg[idx];

  float actual_cm_A  = (float)pos_ticks_A / TICKS_PER_CM_A;
  float actual_deg_B = (float)pos_ticks_B / TICKS_PER_DEG_B;
  current_cm_A  = actual_cm_A;
  current_deg_B = actual_deg_B;

  // PID A
  static float i_A_cm = 0.0f, prev_errA_cm = 0.0f;
  float errA_cm = target_cm_A - actual_cm_A;
  i_A_cm += errA_cm * DT_SEC;
  float derrA_cmps = (errA_cm - prev_errA_cm) / DT_SEC; prev_errA_cm = errA_cm;
  float uA = KP_A_CM * errA_cm + KI_A_CM * i_A_cm + KD_A_CM * derrA_cmps;
  float pwmA_cmd = fabsf(uA);
  if (pwmA_cmd > 255.0f) pwmA_cmd = 255.0f;
  if (current_target_index < (int)KICKA_MS && phase == RUN)
    pwmA_cmd = max(pwmA_cmd, KICKA_PWM);
  int16_t pwmA_255 = (int16_t)lrintf(pwmA_cmd);

  // PID B
  static float i_B_deg = 0.0f, prev_errB_deg = 0.0f;
  float errB_deg = target_deg_B - actual_deg_B;
  i_B_deg += errB_deg * DT_SEC;
  float derrB_degps = (errB_deg - prev_errB_deg) / DT_SEC; prev_errB_deg = errB_deg;
  float uB = KP_B_DEG * errB_deg + KI_B_DEG * i_B_deg + KD_B_DEG * derrB_degps;
  float pwmB_cmd = fabsf(uB); if (pwmB_cmd > 255.0f) pwmB_cmd = 255.0f;
  if (current_target_index < (int)KICKB_MS && phase == RUN) pwmB_cmd = max(pwmB_cmd, KICKB_PWM);
  int16_t pwmB_255 = (int16_t)lrintf(pwmB_cmd);

  setMotorA(uA >= 0 ? +1 : -1, (int)pwmA_cmd);
  setMotorB(uB >= 0 ? +1 : -1, (int)pwmB_cmd);

  // Read piezo
  int32_t adc = (int32_t)analogRead(PIEZO_PIN);
#if PIEZO_USE_BASELINE_HP
  static float baseline = 2048.0f;
  baseline += PIEZO_HP_ALPHA * ((float)adc - baseline);
  int16_t piezo_val = (int16_t)lrintf((float)adc - baseline);
#else
  int16_t piezo_val = (int16_t)(adc - (int32_t)PIEZO_FIXED_OFFSET);
#endif

  // Read current and voltage (ADC - fast, no I2C!)
  measured_current_mA = readCurrent_mA();
  measured_voltage_mV = readVoltage_mV();

  uint16_t usage = (rb_head >= rb_tail) ? (rb_head - rb_tail) : (RB_SIZE - rb_tail + rb_head);
  if (usage > max_rb_usage) max_rb_usage = usage;

  if (streaming) {
    SampleLite s;
    s.t_ms       = (now - t0_us) / 1000U;
    s.actA_cm    = actual_cm_A;
    s.tgtA_cm    = target_cm_A;
    s.actB_deg   = actual_deg_B;
    s.tgtB_deg   = target_deg_B;
    s.piezo_raw  = piezo_val;
    s.pwmA_255   = pwmA_255;
    s.pwmB_255   = pwmB_255;
    s.current_mA = measured_current_mA;
    s.voltage_mV = measured_voltage_mV;

    uint16_t next_head = (rb_head + 1) % RB_SIZE;
    if (next_head != rb_tail) {
      rb[rb_head] = s;
      rb_head = next_head;
    } else {
      overflow_count++;
    }
  }

  switch (phase) {
    case PRE_HOLD:  if (phase_ticks_remaining > 0) phase_ticks_remaining--; else phase = RUN; break;
    case RUN:
      if (current_target_index < num_targets - 1) {
        current_target_index++;
        if (current_target_index >= num_targets - 1) {
          phase = POST_HOLD; phase_ticks_remaining = POST_HOLD_MS;
        }
      }
      break;
    case POST_HOLD: if (phase_ticks_remaining > 0) phase_ticks_remaining--; else phase = DONE; break;
    case DONE: break;
  }
}

// ---- Motor control ----
static inline uint32_t pctFrom255(int pwmVal) {
  if (pwmVal <= 0) return 0;
  if (pwmVal >= 255) return 100;
  return (uint32_t)((pwmVal * 100 + 127) / 255);
}

void setMotorA(int dir, int pwmVal) {
  digitalWrite(IN1_A, dir < 0);
  digitalWrite(IN2_A, dir > 0);
  if (pwmTimA) pwmTimA->setCaptureCompare(3, pctFrom255(pwmVal), PERCENT_COMPARE_FORMAT);
}

void setMotorB(int dir, int pwmVal) {
  digitalWrite(IN1_B, dir < 0);
  digitalWrite(IN2_B, dir > 0);
  if (pwmTimB) pwmTimB->setCaptureCompare(1, pctFrom255(pwmVal), PERCENT_COMPARE_FORMAT);
}

// ---- Encoder ISRs ----
void readEncoderA() {
  uint8_t curr = readAB_A();
  int8_t step = QUAD[(enc_prev_A << 2) | curr];
#if INVERT_DIR_A
  step = -step;
#endif
  pos_ticks_A += step;
  enc_prev_A = curr;
}

void readEncoderB() {
  uint8_t curr = readAB_B();
  int8_t step = QUAD[(enc_prev_B << 2) | curr];
#if INVERT_DIR_B
  step = -step;
#endif
  pos_ticks_B += step;
  enc_prev_B = curr;
}

void readEncoderC() {
  uint8_t a = digitalRead(ENC_C_PIN_A), b = digitalRead(ENC_C_PIN_B);
  uint8_t curr = (a << 1) | b;
  uint8_t idx  = (enc_prev_C << 2) | curr;
  int8_t delta = QUAD[idx];
  if (delta == 0 && curr != enc_prev_C) { enc_prev_C = curr; return; }
#if ENC_C_DIR_INVERT
  delta = -delta;
#endif
  pos_ticks_C += delta;
  enc_prev_C = curr;
}
