#pragma once

#include <Arduino.h>

/*
 * Quét qua lại bằng hai servo gắn cảm biến lửa trái/phải. Servo phải lắp đối
 * xứng nên góc vật lý = 180 - góc logic. Module cũng hỗ trợ giữ góc phát hiện,
 * căn đồng bộ khi thân xe quay và điều khiển trực tiếp từ dashboard Manual.
 */
namespace FlameServoScanner {
  // Gắn servo vào GPIO và đưa về vị trí bắt đầu.
  void begin();

  // Gọi liên tục; update() không dùng delay chặn.
  void update();

  // Góc lệnh hiện tại của servo trái (0..180).
  uint8_t leftAngle();

  // Góc vật lý hiện tại của servo phải.
  uint8_t rightAngle();

  // Trạng thái attach để chương trình chính kiểm tra lỗi phần cứng.
  bool ready();

  // Giữ hai cảm biến tại góc hiện tại trong khi SensorManager xác minh dao động
  // IR là nhấp nháy của lửa, không phải thay đổi do servo quét.
  void setHold(bool hold);
  bool held();
  uint8_t heldAngle();

  // Giá trị hướng khớp FireControl::Direction mà không tạo vòng phụ thuộc header:
  // 0 không có, 1 trái, 2 phải, 3 trước, 4 chưa rõ, 5 sau.
  void aimDirection(uint8_t direction);
  // Giữ cảm biến phát hiện tại góc đã bắt; đưa cảm biến còn lại về 0 độ ở mũi xe.
  void aimDetectedSensor(uint8_t direction, uint8_t scanAngle);
  // Giữ cả hai cảm biến tại góc quét thực tế đã phát hiện lửa.
  void aimScanAngle(uint8_t scanAngle);
  // Điều khiển trực tiếp góc vật lý từ dashboard trong chế độ MANUAL.
  void aimManual(uint8_t leftAngle, uint8_t rightAngle);
  bool manualAiming();
  void clearAim();
  bool aiming();
}
