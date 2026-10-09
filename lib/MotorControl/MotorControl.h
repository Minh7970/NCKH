#pragma once

#include <Arduino.h>

/*
 * Lớp điều khiển mức thấp cho L298N. Giá trị PWM dương/âm được đổi thành mức
 * IN1..IN4 và duty ENA/ENB tương ứng; mọi lệnh đều đi qua giới hạn công suất
 * và hệ số cân hai bên trước khi xuất ra phần cứng.
 */

namespace MotorControl {

  void begin();

  // Các lệnh cơ bản của hệ truyền động vi sai.
  void forward(int pwm);
  // Hiệu chỉnh dương tăng bên trái và giảm bên phải, làm xe chỉnh nhẹ sang phải khi tiến.
  void forwardCorrected(int pwm, int correction);
  void backward(int pwm);
  void left(int pwm);
  void right(int pwm);
  void stop();

  // Relay bơm đặt tại đây vì Navigation/FireControl cùng sử dụng lớp chấp hành này.
  void setPump(bool on);
  bool pumpOn();

}
