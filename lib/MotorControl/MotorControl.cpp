#include "MotorControl.h"
#include "RobotConfig.h"
#include <Arduino.h>
#include <esp_arduino_version.h>

/*
 * Mỗi bên xe nhận một PWM có dấu: dấu quyết định chiều hai chân IN, trị tuyệt
 * đối quyết định duty chân EN. Khi quay tại chỗ, một bên tiến và bên kia lùi.
 * Hàm stop đưa cả hai chân chiều về LOW và duty về 0 để xe dừng chủ động.
 */

namespace {
  bool g_pump = false;
  bool g_leftPwmReady = false;
  bool g_rightPwmReady = false;

  int16_t clampPwm(int16_t v) {
    if (v > 255) return 255;
    if (v < -255) return -255;
    return v;
  }

  int16_t innerPwm(uint8_t pwm) {
    return (int16_t)((int32_t)pwm * ARC_INNER_PCT / 100);
  }

  void writeLeftPwm(int pwm) {
    pwm = constrain(pwm, 0, MOTOR_PWM_LIMIT);
    if (!g_leftPwmReady) {
      digitalWrite(PIN_ENA, LOW);
      return;
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_ENA, (uint32_t)pwm);
#else
    ledcWrite(MOTOR_PWM_CHANNEL_LEFT, (uint32_t)pwm);
#endif
  }

  void writeRightPwm(int pwm) {
    pwm = constrain(pwm, 0, MOTOR_PWM_LIMIT);
    if (!g_rightPwmReady) {
      digitalWrite(PIN_ENB, LOW);
      return;
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_ENB, (uint32_t)pwm);
#else
    ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, (uint32_t)pwm);
#endif
  }

  int16_t applyTrim(int16_t pwm, uint8_t trimPercent) {
    const int32_t trimmed = (int32_t)pwm * trimPercent / 100;
    if (trimmed > MOTOR_PWM_LIMIT) return MOTOR_PWM_LIMIT;
    if (trimmed < -(int32_t)MOTOR_PWM_LIMIT)
      return -(int16_t)MOTOR_PWM_LIMIT;
    return (int16_t)trimmed;
  }

  void setLeft(int16_t pwm) {
    pwm = clampPwm(pwm);
    pwm = applyTrim(pwm, MOTOR_TRIM_LEFT_PCT);

    if (pwm < 0) {
      digitalWrite(PIN_IN1, HIGH);
      digitalWrite(PIN_IN2, LOW);
      writeLeftPwm(-pwm);
    } else if (pwm > 0) {
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, HIGH);
      writeLeftPwm(pwm);
    } else {
      writeLeftPwm(0);
      digitalWrite(PIN_IN1, LOW);
      digitalWrite(PIN_IN2, LOW);
    }
  }

  void setRight(int16_t pwm) {
    pwm = clampPwm(pwm);
    pwm = applyTrim(pwm, MOTOR_TRIM_RIGHT_PCT);

    if (pwm < 0) {
      digitalWrite(PIN_IN3, HIGH);
      digitalWrite(PIN_IN4, LOW);
      writeRightPwm(-pwm);
    } else if (pwm > 0) {
      digitalWrite(PIN_IN3, LOW);
      digitalWrite(PIN_IN4, HIGH);
      writeRightPwm(pwm);
    } else {
      writeRightPwm(0);
      digitalWrite(PIN_IN3, LOW);
      digitalWrite(PIN_IN4, LOW);
    }
  }
}

namespace MotorControl {
  void begin() {
    pinMode(PIN_PUMP, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    g_leftPwmReady = ledcAttach(PIN_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    g_rightPwmReady = ledcAttach(PIN_ENB, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
    g_leftPwmReady = ledcSetup(MOTOR_PWM_CHANNEL_LEFT, MOTOR_PWM_FREQ,
                               MOTOR_PWM_BITS) > 0;
    g_rightPwmReady = ledcSetup(MOTOR_PWM_CHANNEL_RIGHT, MOTOR_PWM_FREQ,
                                MOTOR_PWM_BITS) > 0;
    ledcAttachPin(PIN_ENA, MOTOR_PWM_CHANNEL_LEFT);
    ledcAttachPin(PIN_ENB, MOTOR_PWM_CHANNEL_RIGHT);
#endif

    stop();
    setPump(false);
    Serial.printf("[MOTOR] PWM=%luHz/%ubit limit=%u | "
                  "L ENA=%u IN1=%u IN2=%u status=%s ch=%u | "
                  "R ENB=%u IN3=%u IN4=%u status=%s ch=%u\n",
                  (unsigned long)MOTOR_PWM_FREQ, MOTOR_PWM_BITS,
                  MOTOR_PWM_LIMIT,
                  PIN_ENA, PIN_IN1, PIN_IN2,
                  g_leftPwmReady ? "OK" : "FAIL", MOTOR_PWM_CHANNEL_LEFT,
                  PIN_ENB, PIN_IN3, PIN_IN4,
                  g_rightPwmReady ? "OK" : "FAIL", MOTOR_PWM_CHANNEL_RIGHT);
  }

  void stop() {
    setLeft(0);
    setRight(0);
  }

  
  void forward(int pwm) {
    pwm = constrain(pwm, -255, 255);
    setLeft(pwm);
    setRight(pwm);
  }

  void forwardCorrected(int pwm, int correction) {
    pwm = constrain(pwm, 0, 255);
    correction = constrain(correction,
                           -(int)MPU_HEADING_HOLD_MAX_PWM,
                           (int)MPU_HEADING_HOLD_MAX_PWM);
    setLeft(pwm + correction);
    setRight(pwm - correction);
  }

  void backward(int pwm) {
    pwm = constrain(pwm, -255, 255);
    setLeft(-pwm);
    setRight(-pwm);
  }

  void left(int pwm) {
    pwm = constrain(pwm, -255, 255);
    setLeft(-pwm);
    setRight(pwm);
  }

  void right(int pwm) {
    pwm = constrain(pwm, -255, 255);
    setLeft(pwm);
    setRight(-pwm);
  }

  void setPump(bool on) {
    g_pump = on;
    digitalWrite(PIN_PUMP, (on == PUMP_ACTIVE_HIGH) ? HIGH : LOW);
  }

  bool pumpOn() {
    return g_pump;
  }
}
