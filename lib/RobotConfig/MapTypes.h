#pragma once
#include <Arduino.h>

/*
 * Các kiểu dữ liệu cơ bản được dùng chung bởi bộ nhớ bản đồ và thuật toán dẫn đường.
 * Mỗi ô chỉ có một trong ba trạng thái: chưa biết, có thể đi hoặc có vật cản.
 * Heading lưu hướng nhìn theo bốn hướng chính để việc dịch chuyển giữa các ô luôn
 * dùng tọa độ nguyên, tránh tích lũy sai số số thực trong quá trình lập kế hoạch.
 */
enum CellType : uint8_t {
  CELL_UNKNOWN = 0,   // Ô chưa được cảm biến quan sát đủ để kết luận.
  CELL_FREE = 1,      // Ô trống, xe có thể đưa vào đường tuần tra.
  CELL_OBSTACLE = 2   // Ô có vật cản, thuật toán không được lập đường đi xuyên qua.
};

enum Heading : uint8_t {
  NORTH = 0, // Hướng Bắc trên bản đồ.
  EAST = 1,  // Hướng Đông trên bản đồ.
  SOUTH = 2, // Hướng Nam trên bản đồ.
  WEST = 3   // Hướng Tây trên bản đồ.
};

// Vị trí ô hiện tại của xe và hướng mũi xe trong hệ tọa độ bản đồ.
struct Pose {
  int16_t x;
  int16_t y;
  Heading heading;
};
