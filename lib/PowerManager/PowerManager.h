#pragma once

/*
 * Quản lý trạng thái tiết kiệm năng lượng sau khi phủ bản đồ. Light sleep chỉ
 * được vào khi motor, bơm và radar đã dừng; hệ thống thức định kỳ để cập nhật
 * an toàn và thoát ngủ khi MQ-2 xác nhận khói/khí gas.
 */

namespace PowerManager {
  // Khởi tạo trạng thái nguồn trước khi tạo các tác vụ RTOS.
  void begin();

  // Trả về đúng khi cơ cấu chấp hành và bộ điều hướng phải đứng yên trong chế độ ngủ nhẹ.
  bool sleeping();

  // Vào chờ tiết kiệm điện sau khi đã phủ mọi ô FREE có thể tới.
  void enterCoverageSleep();

  // Trả về đúng khi chế độ ngủ được yêu cầu do xe đã phủ xong bản đồ.
  bool coverageSleepActive();

  // Trả về đúng sau khi cảnh báo MQ-2 đánh thức xe để tuần tra và tìm lửa.
  bool gasWakeActive();

  // Thân tác vụ RTOS: vào light sleep định kỳ và mở khóa Navigation sau khi
  // phần cứng MQ-2 xác nhận cảnh báo gas.
  void taskLoop(void* parameter);
}
