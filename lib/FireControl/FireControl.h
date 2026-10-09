#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

/*
 * Giao diện thuật toán xử lý cháy.
 * Cảm biến quang học xác định hướng để xoay thân xe; MQ-2 chỉ đóng vai trò
 * xác nhận khói/khí gas trước khi cho phép relay máy bơm hoạt động. Việc tách
 * hai điều kiện này giúp xe vẫn căn hướng sớm nhưng không phun nhầm do ánh sáng.
 */

namespace FireControl {
  enum Direction : uint8_t {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    FORWARD = 3,
    AMBIGUOUS = 4,
    BACKWARD = 5
  };

  void begin();
  void update(bool opticalFire, bool gasConfirmed, bool nearFire,
              uint16_t flameLeft, uint16_t flameRight,
              uint8_t scanAngleDeg = FLAME_SERVO_CENTER_DEG);

  Direction direction();
  uint8_t flameScanAngle();
  uint8_t flameTrackingAngle();
  bool flameAtScanEdge();
  bool flameAlignmentComplete();
  bool active();

  bool isPumpRunning();
  uint8_t getPumpCycleCount();
  uint32_t getFireElapsedMs();
  void emergencyStop();
}
