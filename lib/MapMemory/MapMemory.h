#pragma once
#include <Arduino.h>
#include "MapTypes.h"

/*
 * MapMemory quản lý bản đồ chiếm chỗ dạng ô lưới và lưu/nạp bằng EEPROM giả
 * lập. Mỗi tia siêu âm đánh dấu các ô đi qua là trống và điểm phản xạ cuối là
 * vật cản; bộ đếm xác nhận giúp tránh một mẫu nhiễu làm thay đổi bản đồ ngay.
 */

namespace MapMemory {
  void begin();
  void clear();
  bool load();
  bool save();
  bool isLoaded();

  CellType get(int16_t x, int16_t y);
  bool isBlocked(int16_t x, int16_t y);
  void set(int16_t x, int16_t y, CellType c);
  void markRay(int16_t x, int16_t y, Heading h, uint16_t distanceCm);
  void markRayAngle(int16_t x, int16_t y, float angleDeg, uint16_t distanceCm,
                    bool hasReflection = true,
                    bool confirmedReflection = false);
  void markObstacleAhead(const Pose& p, uint16_t distanceCm);

  void updatePoseForward(Pose& p);
  void rotateLeft(Pose& p);
  void rotateRight(Pose& p);
  void rotateBack(Pose& p);

  void print(const Pose& p);
  uint16_t checksum();
  bool dirty();
  void clearDirty();
}
