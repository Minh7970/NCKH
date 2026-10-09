#pragma once
#include <Arduino.h>
#include "MapTypes.h"

/*
 * ScanMapper nhận các mẫu radar theo góc, đổi sang hệ tọa độ thế giới và chỉ
 * công bố chúng lên MapMemory khi hoàn tất vòng 0 -> 180 -> 0. Cách chốt theo
 * vòng tránh để dashboard và bộ lập đường nhìn thấy một bản đồ nửa cập nhật.
 */

namespace ScanMapper {
  void beginSweep();
  void addReading(const Pose& pose, float worldAngleDeg, uint16_t distanceCm,
                 uint8_t validSamples, uint8_t totalSamples,
                 bool integrateIntoMap = true);
  void latestReading(float& worldAngleDeg, uint16_t& distanceCm,
                     bool& valid, uint32_t& measuredAtMs);
  void sweepStatus(uint32_t& completedSweeps, uint32_t& completedAtMs,
                   uint16_t& reflectionCount, bool& publishing);
  void finishSweep(const Pose& pose, uint8_t cycle);
}
