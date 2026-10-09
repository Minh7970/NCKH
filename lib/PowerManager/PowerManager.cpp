#include <Arduino.h>
#include <esp_sleep.h>

#include "PowerManager.h"
#include "RobotConfig.h"
#include "SensorManager.h"
#include "MotorControl.h"
#include "ServoScanner.h"
#include "WebControl.h"

/*
 * PowerManager là cổng chặn chung của điều hướng. Khi sleeping() trả về đúng, mọi
 * tác vụ phải giữ motor/bơm/radar tắt. Light sleep dùng timer để ESP32 thức
 * ngắn, đọc bản chụp MQ-2 rồi quyết định ngủ tiếp hay trở lại tuần tra.
 */

namespace {
  volatile bool sleepState = false;
  volatile bool gasWakeState = false;
  volatile bool coverageSleepState = false;
}

namespace PowerManager {
  void begin() {
    sleepState = MQ2_SENSOR_ENABLED && MQ2_SLEEP_MODE_ENABLED;
    gasWakeState = false;
    coverageSleepState = false;
    if (sleepState) {
      Serial.println(F("[POWER] MQ-2 standby enabled; waiting for gas alarm"));
    }
  }

  bool sleeping() {
    return sleepState;
  }

  void enterCoverageSleep() {
    if (coverageSleepState) return;

    MotorControl::stop();
    MotorControl::setPump(false);
    ServoScanner::stop();
    gasWakeState = false;
    coverageSleepState = true;
    sleepState = true;
    Serial.println(F("[POWER] Coverage complete; entering light sleep"));
  }

  bool coverageSleepActive() {
    return coverageSleepState;
  }

  bool gasWakeActive() {
    return gasWakeState;
  }

  void taskLoop(void*) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
      if (!sleepState) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(250));
        continue;
      }

      // Ưu tiên an toàn: không cho motor, bơm hoặc radar hoạt động khi đang chờ.
      MotorControl::stop();
      MotorControl::setPump(false);
      ServoScanner::stop();

      // Thức ngắn để tác vụ an toàn/trạng thái cập nhật. Ngủ sau phủ bản đồ độc
      // lập với MQ-2 nên vẫn hoạt động nếu không lắp cảm biến gas.
      const uint32_t sleepIntervalMs = coverageSleepState
        ? COVERAGE_SLEEP_INTERVAL_MS : MQ2_SLEEP_INTERVAL_MS;
      esp_sleep_enable_timer_wakeup(
        (uint64_t)sleepIntervalMs * 1000ULL);
      esp_light_sleep_start();
      vTaskDelay(pdMS_TO_TICKS(80));

      const SensorSnapshot snapshot = SensorManager::get();
      if (MQ2_SENSOR_ENABLED && snapshot.gasReady && snapshot.gasDetected) {
        sleepState = false;
        gasWakeState = true;
        coverageSleepState = false;
        // Cảnh báo gas luôn khởi động tuần tra tự động, kể cả trước đó đang Manual.
        if (!WebControl::automaticMode()) WebControl::setMode("AUTO");
        Serial.printf("[POWER] MQ-2 alarm confirmed signal=%u; robot awake\n",
                      snapshot.mq2Signal);
        // sleeping() đang chặn Navigation; nhịp 20 ms tiếp theo sẽ tự bắt đầu
        // tuần tra và tìm lửa sau khi cờ này được mở.
      }
    }
  }
}
