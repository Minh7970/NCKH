#pragma once

/*
 * Máy chủ HTTP cung cấp bản đồ, trạng thái cảm biến và các lệnh dashboard.
 * CORS được bật để file HTML trong thư mục NCKH có thể gọi trực tiếp ESP32.
 */

namespace WifiMapServer {
  void begin();
  void update();
}
