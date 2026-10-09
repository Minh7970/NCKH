#pragma once
#include <Arduino.h>

/*
 * Máy trạng thái servo radar 0 -> 180 -> 0. Module chỉ báo khi servo đã ổn
 * định và đến hạn đo; việc phát xung/đọc siêu âm do SensorManager thực hiện.
 */

namespace ServoScanner {
  void begin();
  void startScan();
  void update();
  bool active();
  bool measurementDue();
  uint8_t angle();
  void measurementTaken();
  bool cycleFinished();
  // Dừng máy trạng thái quét nhưng không ra lệnh đổi góc cơ khí.
  void holdPosition();
  // Ngắt xung PWM và nhả servo radar sau hai lượt quét ban đầu.
  void powerOff();
  void stop();
}
