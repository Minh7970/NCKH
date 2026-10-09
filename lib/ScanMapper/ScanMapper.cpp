#include "ScanMapper.h"
#include "MapMemory.h"
#include "RobotConfig.h"

/*
 * Mỗi góc radar có nhiều mẫu. Module lưu số đo mới nhất cho giao diện nhưng
 * tách việc hiển thị khỏi việc ghi bản đồ. Chỉ số đo hợp lệ khi pose đứng yên
 * mới tạo tia FREE và điểm OBSTACLE; no-echo chỉ mở ô sát xe theo cấu hình.
 */
#include <freertos/FreeRTOS.h>

// Namespace ẩn danh: các biến thống kê và dữ liệu số đo chỉ dùng nội bộ trong file này
namespace {
  constexpr uint16_t MAX_RAYS_PER_SWEEP =
    ((SERVO_SCAN_MAX_DEG - SERVO_SCAN_MIN_DEG) / SERVO_SCAN_STEP_DEG) * 2 + 1;

  struct PendingRadarRay {
    int16_t x;
    int16_t y;
    float worldAngleDeg;
    uint16_t distanceCm;
    bool hasReflection;
    bool confirmedReflection;
  };

  // Giữ quan sát radar nội bộ cho đến khi hoàn tất vòng quét 0 -> 180 -> 0.
  PendingRadarRay pendingRays[MAX_RAYS_PER_SWEEP];
  uint16_t pendingRayCount = 0;
  uint16_t confirmedReflections = 0;
  uint16_t noEchoClearRays = 0;
  uint16_t lastPublishedReflectionCount = 0;
  uint32_t completedSweepCount = 0;
  uint32_t lastSweepCompletedAtMs = 0;
  bool publishingSweep = false;
  // Số góc servo đã đo trong vòng quét hiện tại
  uint16_t measuredAngles = 0;
  // Số góc có số đo hợp lệ (ít nhất một mẫu có tiếng vọng)
  uint16_t validAngles = 0;
  // Số tia đã được ghi vào bản đồ
  uint16_t mappedRays = 0;
  // Tổng số mẫu siêu âm đã đo (kể cả mẫu không hợp lệ)
  uint16_t totalEchoSamples = 0;
  // Tổng số mẫu siêu âm hợp lệ (có tiếng vọng)
  uint16_t validEchoSamples = 0;
  // Số đo gần nhất: góc trong hệ tọa độ thế giới (độ)
  float lastWorldAngleDeg = 0.0f;
  // Số đo gần nhất: khoảng cách (cm)
  uint16_t lastDistanceCm = 0;
  // Số đo gần nhất có hợp lệ hay không
  bool lastReadingValid = false;
  // Thời điểm (ms) của số đo gần nhất
  uint32_t lastMeasuredAtMs = 0;
  // Khóa (spinlock) của FreeRTOS để bảo vệ nhóm biến "số đo gần nhất",
  // vì có thể được ghi ở tác vụ này và đọc ở tác vụ khác (ví dụ tác vụ web)
  portMUX_TYPE readingMux = portMUX_INITIALIZER_UNLOCKED;
}
namespace ScanMapper {
  // Bắt đầu một vòng quét mới: đặt lại toàn bộ bộ đếm thống kê về 0
  void beginSweep() {
    measuredAngles = 0;
    validAngles = 0;
    mappedRays = 0;
    totalEchoSamples = 0;
    validEchoSamples = 0;
    pendingRayCount = 0;
    confirmedReflections = 0;
    noEchoClearRays = 0;
  }

  // Thêm một số đo (tại một góc servo) vào bản đồ
  // pose: vị trí robot; worldAngleDeg: góc tia đo trong hệ tọa độ thế giới
  // distanceCm: khoảng cách trung bình; validSamples/totalSamples: số mẫu hợp lệ/tổng số mẫu
  void addReading(const Pose& pose, float worldAngleDeg, uint16_t distanceCm,
                  uint8_t validSamples, uint8_t totalSamples,
                  bool integrateIntoMap) {
    // Tăng số góc đã đo và cộng dồn số mẫu vào thống kê
    ++measuredAngles;
            totalEchoSamples += totalSamples;
            validEchoSamples += validSamples;

    // Vào vùng găng (critical section) để cập nhật số đo gần nhất một cách nguyên vẹn,
    // tránh để tác vụ khác đọc phải dữ liệu đang ghi dở
    taskENTER_CRITICAL(&readingMux);
    lastWorldAngleDeg = worldAngleDeg;
    lastDistanceCm = distanceCm;
    // Số đo chỉ hợp lệ khi có ít nhất một mẫu hợp lệ và khoảng cách > 0
    lastReadingValid = validSamples > 0 && distanceCm > 0;
    lastMeasuredAtMs = millis();
    taskEXIT_CRITICAL(&readingMux);

    // Số đo không hợp lệ thì không ghi vào bản đồ (không coi là đường trống)
    if (validSamples == 0 || distanceCm == 0) {
      // Ở vùng rộng, timeout không được làm mọi ô luôn UNKNOWN. Chỉ công bố ô
      // sát xe là FREE và ghi rõ đây là tia không phản xạ để không tạo vật cản giả.
      if (!integrateIntoMap || !RADAR_NO_ECHO_CLEARS_ADJACENT_CELL) return;
      if (pendingRayCount >= MAX_RAYS_PER_SWEEP) {
        Serial.println(F("[RADAR] SWEEP BUFFER FULL; NO-ECHO RAY DROPPED"));
        return;
      }

      PendingRadarRay& ray = pendingRays[pendingRayCount++];
      ray.x = pose.x;
      ray.y = pose.y;
      ray.worldAngleDeg = worldAngleDeg;
      ray.distanceCm = RADAR_NO_ECHO_CLEAR_CM;
      ray.hasReflection = false;
      ray.confirmedReflection = false;
      ++noEchoClearRays;
      ++mappedRays;
      return;
    }

    // Số đo hợp lệ luôn được giữ để hiển thị radar. Chỉ tích hợp vào map
    // toàn cục khi pose ổn định để tránh ghi tia sai lúc xe đang chạy/quay.
    ++validAngles;
    if (!integrateIntoMap) return;
    if (pendingRayCount >= MAX_RAYS_PER_SWEEP) {
      Serial.println(F("[RADAR] SWEEP BUFFER FULL; RAY DROPPED"));
      return;
    }

    PendingRadarRay& ray = pendingRays[pendingRayCount++];
    ray.x = pose.x;
    ray.y = pose.y;
    ray.worldAngleDeg = worldAngleDeg;
    ray.distanceCm = distanceCm;
    ray.hasReflection = true;
    ray.confirmedReflection =
      validSamples >= RADAR_OBSTACLE_MIN_VALID_SAMPLES &&
      distanceCm < MAX_RANGE_CM;
    if (ray.confirmedReflection) ++confirmedReflections;
    ++mappedRays;

  }

  // Lấy số đo gần nhất một cách an toàn giữa các tác vụ (trả về qua tham chiếu)
  void latestReading(float& worldAngleDeg, uint16_t& distanceCm,
                     bool& valid, uint32_t& measuredAtMs) {
    // Đọc trong vùng găng để bốn giá trị luôn thuộc về cùng một số đo
    taskENTER_CRITICAL(&readingMux);
    worldAngleDeg = lastWorldAngleDeg;
    distanceCm = lastDistanceCm;
    valid = lastReadingValid;
    measuredAtMs = lastMeasuredAtMs;
    taskEXIT_CRITICAL(&readingMux);
  }

  void sweepStatus(uint32_t& completedSweeps, uint32_t& completedAtMs,
                   uint16_t& reflectionCount, bool& publishing) {
    taskENTER_CRITICAL(&readingMux);
    completedSweeps = completedSweepCount;
    completedAtMs = lastSweepCompletedAtMs;
    reflectionCount = lastPublishedReflectionCount;
    publishing = publishingSweep;
    taskEXIT_CRITICAL(&readingMux);
  }

  // Kết thúc một vòng quét: in thống kê và in bản đồ ra Serial để gỡ lỗi
  void finishSweep(const Pose& pose, uint8_t cycle) {
    taskENTER_CRITICAL(&readingMux);
    publishingSweep = true;
    taskEXIT_CRITICAL(&readingMux);

    for (uint16_t i = 0; i < pendingRayCount; ++i) {
      const PendingRadarRay& ray = pendingRays[i];
      MapMemory::markRayAngle(ray.x, ray.y, ray.worldAngleDeg, ray.distanceCm,
                              ray.hasReflection, ray.confirmedReflection);
    }

    taskENTER_CRITICAL(&readingMux);
    ++completedSweepCount;
    lastSweepCompletedAtMs = millis();
    lastPublishedReflectionCount = confirmedReflections;
    publishingSweep = false;
    taskEXIT_CRITICAL(&readingMux);

    // In: chu kỳ, số góc đã đo, số góc hợp lệ, số tia đã ghi, số tiếng vọng hợp lệ/tổng
    Serial.printf("\n[OCCUPANCY] cycle=%u angles=%u validAngles=%u rays=%u obstacles=%u noEchoFree=%u echoes=%u/%u\n",
                  (unsigned)cycle,
                  (unsigned)measuredAngles,
                  (unsigned)validAngles,
                  (unsigned)mappedRays,
                  (unsigned)confirmedReflections,
                  (unsigned)noEchoClearRays,
                  (unsigned)validEchoSamples,
                  (unsigned)totalEchoSamples);
    // In bản đồ hiện tại kèm vị trí robot
    MapMemory::print(pose);
  }
}
