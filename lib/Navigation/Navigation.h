#pragma once
#include <Arduino.h>
#include "MapTypes.h"

/*
 * Giao diện điều hướng tự động. Module duy trì pose theo ô lưới, điều khiển
 * quét bản đồ ban đầu, chọn đường phủ các ô FREE, tránh vật cản SA2, chuyển
 * sang xử lý cháy và yêu cầu ngủ khi không còn ô nào cần tuần tra.
 */

namespace Navigation {
  enum Mode { EXPLORE, PATROL, FIRE };
  void begin();
  void firstRunRoomScan();  // Quét radar để tạo bản đồ phòng lần đầu.
  void update();
  Pose pose();
  Mode mode();
  bool visited(int16_t x, int16_t y);
  void forceStop();
}
