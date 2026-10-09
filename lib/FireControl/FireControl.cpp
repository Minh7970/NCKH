#include "FireControl.h"
#include "RobotConfig.h"
#include "MotorControl.h"
#include "SensorManager.h"

/*
 * Thuật toán chữa cháy gồm bốn bước chính:
 * 1. Phân loại nguồn lửa ở trái, phải, trước hoặc sau từ hai kênh cảm biến.
 * 2. Khóa hướng ban đầu và xoay thân xe để ống nước cố định hướng vào nguồn.
 * 3. Chờ MQ-2 xác nhận; chỉ lúc đó relay máy bơm mới được phép bật.
 * 4. Giới hạn thời gian và số chu kỳ bơm, sau đó trả quyền điều khiển cho
 *    Navigation. Khóa phiên ngăn một tín hiệu lửa liên tục khởi động lại bơm.
 */

// Namespace ẩn danh: các biến và hàm bên trong chỉ dùng nội bộ trong file này
namespace {
  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Trạng thái an toàn bơm
  // ═════════════════════════════════════════════════════════════════════════════
  
  // Cờ cho biết robot đang trong quá trình xử lý đám cháy hay không
  bool activeFire = false;
  // Phiên đã hoàn tất hoặc bị hủy vẫn bị khóa cho đến khi mất tín hiệu lửa.
  // Nhờ vậy tín hiệu còn giữ mức cao không thể tạo lại một phiên bơm 10 giây
  // sau mỗi lần Navigation gọi hàm cập nhật.
  bool fireSessionLocked = false;
  
  // Thời điểm (ms) bắt đầu xử lý đám cháy, dùng để tính thời gian phun tối đa
  uint32_t fireStart = 0;
  
  // Hướng lửa hiện tại được xác định từ cảm biến
  FireControl::Direction currentDirection = FireControl::NONE;
  uint8_t currentScanAngle = FLAME_SERVO_CENTER_DEG;
  
  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Trạng thái bơm an toàn
  // ═════════════════════════════════════════════════════════════════════════════
  
  // Cờ cho biết bơm đang chạy hay không
  bool pumpActive = false;
  
  // Thời điểm bơm bắt đầu
  uint32_t pumpStart = 0;
  
  // Số lần chu kỳ bơm trong phiên hiện tại
  uint8_t pumpCycles = 0;
  
  // Thời điểm chu kỳ bơm gần nhất kết thúc (để tính cooldown)
  uint32_t lastPumpEnd = 0;

  // Ống nước cố định ở hướng 0 độ; phải xoay thân cho cảm biến lửa cũng về
  // hướng 0 độ trước khi cho phép bật máy bơm.
  bool bodyTurnInProgress = false;
  bool bodyTurnAligned = false;
  uint32_t bodyTurnStart = 0;
  uint32_t bodyTurnDuration = 0;
  uint32_t bodyTurnMaxDuration = 0;
  float bodyTurnStartYaw = 0.0f;
  float bodyTurnTargetYaw = 0.0f;
  float bodyTurnStopDegrees = 0.0f;
  bool bodyTurnImuValid = false;
  FireControl::Direction bodyTurnDirection = FireControl::NONE;
  bool gasConfirmedOnce = false;
  uint32_t lastGasWaitLogMs = 0;

  bool imuFresh(const SensorSnapshot& s) {
    return MPU6050_ENABLED && s.imuValid && s.imuMs > 0 &&
      millis() - s.imuMs <= MPU6050_FRESH_MS;
  }

  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Phân loại hướng của ngọn lửa dựa trên 2 cảm biến
  // ═════════════════════════════════════════════════════════════════════════════
  
  FireControl::Direction classify(uint16_t leftSensor, uint16_t rightSensor,
                                  uint8_t scanAngleDeg) {
    // ═════════════════════════════════════════════════════════════════════
    // Kiểm tra giới hạn góc trước khi dùng dữ liệu của servo.
    // ═════════════════════════════════════════════════════════════════════
    
    // Kiểm tra LEFT có hợp lệ không
    if (leftSensor < FLAME_SIGNAL_MIN || leftSensor > FLAME_SIGNAL_MAX) {
      Serial.print(F("[FIRE] LEFT out of range: "));
      Serial.println(leftSensor);
      return FireControl::AMBIGUOUS;  // Giá trị không hợp lệ nên coi là chưa rõ hướng.
    }
    
    // Kiểm tra RIGHT có hợp lệ không
    if (rightSensor < FLAME_SIGNAL_MIN || rightSensor > FLAME_SIGNAL_MAX) {
      Serial.print(F("[FIRE] RIGHT out of range: "));
      Serial.println(rightSensor);
      return FireControl::AMBIGUOUS;  // Giá trị không hợp lệ nên coi là chưa rõ hướng.
    }
    
    // Lửa ở PHÍA TRÁI: cảm biến trái mạnh hơn cảm biến phải
    if (leftSensor > rightSensor + FIRE_DIRECTION_MARGIN) {
      return FireControl::LEFT;
    }
    
    // Lửa ở PHÍA PHẢI: cảm biến phải mạnh hơn cảm biến trái
    if (rightSensor > leftSensor + FIRE_DIRECTION_MARGIN) {
      return FireControl::RIGHT;
    }
    
    // Hai cảm biến cân bằng: góc 0..5 là hướng thẳng của ống cố định;
    // góc 175..180 là phía sau và cần xoay thân 180 độ.
    if (leftSensor > 0 && rightSensor > 0) {
      const bool rear = scanAngleDeg >=
        FLAME_SERVO_MAX_DEG - FLAME_ALIGNMENT_TOLERANCE_DEG;
      if (rear) return FireControl::BACKWARD;

      const bool front = scanAngleDeg <= FLAME_ALIGNMENT_TOLERANCE_DEG;
      if (front) return FireControl::FORWARD;

      // Ở góc trung gian, hai tín hiệu gần bằng nhau không đủ để quyết định
      // trái/phải an toàn. Giữ xe đứng yên thay vì chọn tùy ý và làm sai hướng.
      return FireControl::AMBIGUOUS;
    }
    
    // Cả 2 cảm biến = 0: không phát hiện lửa
    return FireControl::NONE;
  }
}

namespace FireControl {
  
  // ═════════════════════════════════════════════════════════════════════════════
  // Khởi tạo module: đảm bảo bơm ở trạng thái tắt khi bắt đầu
  // ═════════════════════════════════════════════════════════════════════════════
  
  void begin() {
    MotorControl::setPump(false);
    pumpActive = false;
    pumpCycles = 0;
    fireSessionLocked = false;
    bodyTurnInProgress = false;
    bodyTurnAligned = false;
    bodyTurnImuValid = false;
    bodyTurnDirection = NONE;
    gasConfirmedOnce = false;
    lastGasWaitLogMs = 0;
    lastPumpEnd = millis();
    Serial.println(F("[FIRE] Initialized - pump OFF"));
  }

  // ═════════════════════════════════════════════════════════════════════════════
  // Trả về hướng lửa hiện tại (LEFT, RIGHT, FORWARD, AMBIGUOUS hoặc NONE)
  // ═════════════════════════════════════════════════════════════════════════════
  
  Direction direction() {
    return currentDirection;
  }

  uint8_t flameScanAngle() {
    return currentScanAngle;
  }

  uint8_t flameTrackingAngle() {
    if (!activeFire) return currentScanAngle;
    if (bodyTurnAligned) return PUMP_FIXED_HEADING_DEG;
    if (!bodyTurnInProgress || bodyTurnDuration == 0) return currentScanAngle;

    uint32_t progressValue = millis() - bodyTurnStart;
    uint32_t progressTarget = bodyTurnDuration;
    const SensorSnapshot imu = SensorManager::get();
    if (bodyTurnImuValid && imuFresh(imu) && bodyTurnStopDegrees > 0.0f) {
      progressValue = (uint32_t)fabsf(imu.yawDeg - bodyTurnStartYaw);
      progressTarget = (uint32_t)ceilf(bodyTurnStopDegrees);
    }
    if (progressValue > progressTarget) progressValue = progressTarget;

    // Hai giá đỡ servo đối xứng cùng dùng quy ước logic: 0 độ là hướng mũi xe
    // và ống nước. Khi thân xe quay, servo phải dần về 0 độ nhưng cảm biến vẫn
    // giữ hướng nhìn ngoài thực tế. Nếu đưa servo phải về 180 độ, nó sẽ nhìn
    // ra sau và gây sai lệch khi căn nguồn lửa bên phải.
    const uint8_t target = PUMP_FIXED_HEADING_DEG;

    const int16_t delta = (int16_t)target - (int16_t)currentScanAngle;
    int16_t angle = (int16_t)currentScanAngle +
      (int32_t)delta * (int32_t)progressValue /
        (int32_t)(progressTarget > 0 ? progressTarget : 1);
    if (angle < FLAME_SERVO_MIN_DEG) angle = FLAME_SERVO_MIN_DEG;
    if (angle > FLAME_SERVO_MAX_DEG) angle = FLAME_SERVO_MAX_DEG;
    return (uint8_t)angle;
  }

  bool flameAtScanEdge() {
    return activeFire &&
      (currentScanAngle <= FLAME_ALIGNMENT_TOLERANCE_DEG ||
       currentScanAngle >= FLAME_SERVO_MAX_DEG -
         FLAME_ALIGNMENT_TOLERANCE_DEG);
  }

  bool flameAlignmentComplete() {
    return activeFire && bodyTurnAligned;
  }

  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Hàm cập nhật chính - An toàn bơm tích hợp
  // ═════════════════════════════════════════════════════════════════════════════
  
  void update(bool opticalFire, bool gasConfirmed, bool nearFire,
              uint16_t flameLeft, uint16_t flameRight,
              uint8_t scanAngleDeg) {
    
    // ═════════════════════════════════════════════════════════════════════
    // Trường hợp không còn lửa (hoặc chưa xác nhận)
    // ═════════════════════════════════════════════════════════════════════
    
    if (!opticalFire) {
      if (activeFire) {
        // Dừng bơm ngay lập tức
        if (pumpActive) {
          MotorControl::setPump(false);
          pumpActive = false;
          lastPumpEnd = millis();
          Serial.println(F("[FIRE] Emergency stop - flame lost!"));
        }
        
        // Dừng xe
        MotorControl::stop();
        Serial.println(F("[FIRE] Fire lost, stopping"));
      }
      
      // Đặt lại trạng thái
      activeFire = false;
      fireSessionLocked = false;
      currentDirection = NONE;
      currentScanAngle = FLAME_SERVO_CENTER_DEG;
      pumpCycles = 0;
      bodyTurnInProgress = false;
      bodyTurnAligned = false;
      bodyTurnImuValid = false;
      bodyTurnDirection = NONE;
      gasConfirmedOnce = false;
      return;
    }

    if (fireSessionLocked) {
      MotorControl::setPump(false);
      MotorControl::stop();
      return;
    }

    // ═════════════════════════════════════════════════════════════════════
    // Có lửa xác nhận: xác định hướng từ 2 cảm biến
    // ═════════════════════════════════════════════════════════════════════
    
    // Khi đã bắt đầu xoay thân, giữ nguyên hướng đã khóa. Servo di chuyển đồng
    // bộ sẽ làm ADC thay đổi tạm thời; phân loại lại giữa chừng có thể khiến xe
    // đảo chiều trước khi hoàn tất góc quay.
    if (bodyTurnAligned) {
      currentDirection = FORWARD;
    } else if (bodyTurnInProgress && bodyTurnDirection != NONE) {
      currentDirection = bodyTurnDirection;
    } else {
      currentDirection = classify(flameLeft, flameRight, scanAngleDeg);
    }
    currentScanAngle = scanAngleDeg;

    // Đánh dấu phiên chữa cháy trước các nhánh thoát sớm của quá trình xoay.
    if (!activeFire) {
      activeFire = true;
      fireStart = millis();
      pumpCycles = 0;
      gasConfirmedOnce = false;
      MotorControl::stop();
      Serial.printf("[FIRE] Detected dir=%d angle=%u signal(L/R)=%u/%u\n",
                    (int)currentDirection, scanAngleDeg,
                    flameLeft, flameRight);
    }
    const bool edgeFront = scanAngleDeg <= FLAME_ALIGNMENT_TOLERANCE_DEG;
    const bool edgeRear = scanAngleDeg >=
      FLAME_SERVO_MAX_DEG - FLAME_ALIGNMENT_TOLERANCE_DEG;
    const bool sourceAtFront = currentDirection == FORWARD || edgeFront;
    const bool sourceAtRear = currentDirection == BACKWARD || edgeRear;
    bool needsBodyTurn = false;
    if (sourceAtFront) {
      bodyTurnInProgress = false;
      bodyTurnAligned = true;
      bodyTurnImuValid = false;
      bodyTurnDirection = NONE;
    } else if (sourceAtRear) {
      needsBodyTurn = true;
    } else if (currentDirection == LEFT || currentDirection == RIGHT) {
      // FlameServoScanner đã đảo cơ khí servo phải, vì vậy góc quét logic dùng
      // trực tiếp làm góc xoay thân cho cả trái và phải; không đảo lần thứ hai.
      const uint8_t sourceAngle = scanAngleDeg;
      needsBodyTurn = sourceAngle > FLAME_ALIGNMENT_TOLERANCE_DEG;
    }

    const bool sourceAligned = sourceAtFront ||
      ((currentDirection == LEFT || currentDirection == RIGHT) &&
       scanAngleDeg <= FLAME_ALIGNMENT_TOLERANCE_DEG);

    if (!needsBodyTurn && sourceAligned) {
      bodyTurnInProgress = false;
      bodyTurnAligned = true;
      bodyTurnImuValid = false;
      bodyTurnDirection = NONE;
    } else if (!needsBodyTurn) {
      // Hai tín hiệu bằng nhau ở góc trung gian vẫn chưa rõ hướng; không chạy
      // và không phun cho đến khi lần quét khác xác định được phía cụ thể.
      bodyTurnInProgress = false;
      bodyTurnAligned = false;
      bodyTurnImuValid = false;
      bodyTurnDirection = NONE;
      MotorControl::stop();
      return;
    } else if (!bodyTurnAligned) {
      if (!bodyTurnInProgress || bodyTurnDirection != currentDirection) {
        bodyTurnInProgress = true;
        bodyTurnDirection = currentDirection;
        bodyTurnStart = millis();
        uint16_t turnDegrees = 0;
        if (sourceAtRear) {
          turnDegrees = 180;
        } else if (currentDirection == LEFT || currentDirection == RIGHT) {
          turnDegrees = scanAngleDeg;
        }
        if (turnDegrees < FLAME_ALIGNMENT_TOLERANCE_DEG)
          turnDegrees = 90;
        bodyTurnDuration = (uint32_t)TURN_90_MS * turnDegrees / 90UL;
        bodyTurnMaxDuration = (uint32_t)MPU_TURN_MAX_90_MS *
          turnDegrees / 90UL;
        if (bodyTurnMaxDuration < MPU_TURN_MIN_MS)
          bodyTurnMaxDuration = MPU_TURN_MIN_MS;
        bodyTurnStopDegrees = turnDegrees > FLAME_ALIGNMENT_TOLERANCE_DEG
          ? turnDegrees - FLAME_ALIGNMENT_TOLERANCE_DEG : turnDegrees;
        const SensorSnapshot imu = SensorManager::get();
        bodyTurnStartYaw = imu.yawDeg;
        bodyTurnImuValid = imuFresh(imu);
        const bool turnLeft = bodyTurnDirection == LEFT;
        bodyTurnTargetYaw = bodyTurnStartYaw +
          (turnLeft ? (float)turnDegrees : -(float)turnDegrees);
        Serial.printf(
          "[FIRE] Aim locked dir=%d angle=%u yaw=%.1f->%.1f MQ2=confirmed IMU=%d\n",
          (int)bodyTurnDirection, turnDegrees, bodyTurnStartYaw,
          bodyTurnTargetYaw, bodyTurnImuValid ? 1 : 0);
      }

      const SensorSnapshot imu = SensorManager::get();
      const uint32_t turnElapsed = millis() - bodyTurnStart;
      const bool imuAvailable = bodyTurnImuValid && imuFresh(imu);
      const float yawError = bodyTurnTargetYaw - imu.yawDeg;
      const bool targetCrossed = bodyTurnDirection == LEFT
        ? yawError <= 0.0f : yawError >= 0.0f;
      const bool imuReached = imuAvailable &&
        turnElapsed >= MPU_TURN_MIN_MS &&
        (fabsf(yawError) <= MPU_FIRE_AIM_TOLERANCE_DEG || targetCrossed);
      const bool timedOut = turnElapsed >=
        (imuAvailable ? bodyTurnMaxDuration : bodyTurnDuration);
      if (!imuReached && !timedOut) {
        MotorControl::setPump(false);
        if (currentDirection == LEFT) {
          MotorControl::left(PWM_FIRE);
        } else {
          // Hướng phải và phía sau đều dùng quay tại chỗ theo chiều kim đồng hồ;
          // trường hợp phía sau dùng góc 180 độ đã tính ở trên.
          MotorControl::right(PWM_FIRE);
        }
        return;
      }

      bodyTurnInProgress = false;
      bodyTurnAligned = true;
      bodyTurnImuValid = false;
      currentDirection = FORWARD;
      MotorControl::stop();
      Serial.printf("[FIRE] Body aligned yaw=%.1f source=%s\n",
                    imu.yawDeg,
                    imuReached ? "IMU" : "TIME");
    }

    // Cảm biến quang học quyết định hướng, nhưng MQ-2 phải cấp quyền cho relay
    // một lần trong mỗi phiên. Vì MQ-2 thường dao động quanh ngưỡng, sau khi đủ
    // chuỗi mẫu xác nhận thì giữ khóa cho đến khi mất lửa hoặc kết thúc phiên.
    if (gasConfirmed && !gasConfirmedOnce) {
      gasConfirmedOnce = true;
      fireStart = millis();
      Serial.println(F("[FIRE] MQ2 confirmed and latched -> relay permitted"));
    }

    if (!gasConfirmedOnce) {
      if (pumpActive) {
        MotorControl::setPump(false);
        pumpActive = false;
        lastPumpEnd = millis();
      }
      MotorControl::stop();
      if (millis() - lastGasWaitLogMs >= 1000) {
        lastGasWaitLogMs = millis();
        Serial.println(F("[FIRE] Optical source aligned; MQ2 not confirmed -> relay OFF"));
      }
      return;
    }

    // ═════════════════════════════════════════════════════════════════════
    // ═════════════════════════════════════════════════════════════════════
    // CẬP NHẬT ĐỢT 1: Giới hạn thời gian xử lý tối đa 10 giây
    // An toàn: tắt bơm nếu vượt quá thời gian
    // ═════════════════════════════════════════════════════════════════════
    
    uint32_t elapsedMs = millis() - fireStart;
    if (elapsedMs > PUMP_MAX_SPRAY_TIME_MS) {
      // Timeout - dừng hoàn toàn
      if (pumpActive) {
        MotorControl::setPump(false);
        pumpActive = false;
      }
      MotorControl::stop();
      activeFire = false;
      fireSessionLocked = true;
      currentDirection = NONE;
      currentScanAngle = FLAME_SERVO_CENTER_DEG;
      pumpCycles = 0;
      bodyTurnInProgress = false;
      bodyTurnAligned = false;
      bodyTurnImuValid = false;
      bodyTurnDirection = NONE;
      gasConfirmedOnce = false;
      Serial.println(F("[FIRE] Timeout 10s - STOP"));
      return;
    }

    // ═════════════════════════════════════════════════════════════════════
    // CẬP NHẬT ĐỢT 1: Kiểm tra giới hạn chu kỳ bơm
    // An toàn: giới hạn số lần bơm trong một phiên
    // ═════════════════════════════════════════════════════════════════════
    
    if (pumpCycles >= PUMP_MAX_CYCLES_PER_SESSION && !pumpActive) {
      // Đã bơm quá nhiều lần - dừng
      if (pumpActive) {
        MotorControl::setPump(false);
        pumpActive = false;
      }
      fireSessionLocked = true;
      Serial.println(F("[FIRE] Max pump cycles reached - STOP"));
      return;
    }

    // ═════════════════════════════════════════════════════════════════════
    // Kiểm tra khoảng cách: nếu đủ gần lửa thì dừng tại chỗ và phun
    // ═════════════════════════════════════════════════════════════════════
    
    if (nearFire) {
      MotorControl::stop();
      
      // ═══════════════════════════════════════════════════════════════════
      // CẬP NHẬT ĐỢT 1: Điều khiển bơm an toàn với cooldown
      // ═══════════════════════════════════════════════════════════════════
      
      uint32_t timeSincePumpEnd = millis() - lastPumpEnd;
      
      // Kiểm tra cooldown: tránh bơm liên tục
      if (!pumpActive && timeSincePumpEnd < PUMP_COOLDOWN_MS) {
        // Đang trong cooldown - không bơm
        // (Đợi hết cooldown)
      } else if (!pumpActive) {
        // Hết cooldown - bắt đầu bơm mới
        MotorControl::setPump(true);
        pumpActive = true;
        pumpStart = millis();
        pumpCycles++;
        Serial.print(F("[FIRE] Pump ON - Cycle "));
        Serial.print(pumpCycles);
        Serial.print(F("/"));
        Serial.println(PUMP_MAX_CYCLES_PER_SESSION);
      }
      
      // ═══════════════════════════════════════════════════════════════════
      // CẬP NHẬT ĐỢT 1: Kiểm tra timeout chu kỳ bơm đơn lẻ
      // ═══════════════════════════════════════════════════════════════════
      
      if (pumpActive) {
        uint32_t pumpElapsed = millis() - pumpStart;
        
        // Giới hạn mỗi chu kỳ phun không quá 3 giây
        if (pumpElapsed > 3000) {
          MotorControl::setPump(false);
          pumpActive = false;
          lastPumpEnd = millis();
          Serial.println(F("[FIRE] Pump cycle timeout 3s"));
        }
      }
      
      return;
    }

    // ═════════════════════════════════════════════════════════════════════
    // Chưa đủ gần lửa: tắt bơm và tiếp tục di chuyển để tiếp cận
    // ═════════════════════════════════════════════════════════════════════
    
    if (pumpActive) {
      MotorControl::setPump(false);
      pumpActive = false;
      lastPumpEnd = millis();
    }

    // ═════════════════════════════════════════════════════════════════════
    // Bám theo ngọn lửa dựa trên hướng được xác định
    // ═════════════════════════════════════════════════════════════════════
    
    if (currentDirection == LEFT) {
      MotorControl::left(PWM_FIRE);
    } else if (currentDirection == RIGHT) {
      MotorControl::right(PWM_FIRE);
    } else if (currentDirection == BACKWARD) {
      MotorControl::backward(PWM_FIRE);
    } else {
      // FORWARD hoặc AMBIGUOUS: tiến thẳng
      MotorControl::forward(PWM_FIRE);
    }
  }

  // ═════════════════════════════════════════════════════════════════════════════
  // Trả về đúng nếu robot đang trong quá trình xử lý đám cháy.
  // ═════════════════════════════════════════════════════════════════════════════
  
  bool active() {
    return activeFire;
  }
  
  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Hàm kiểm tra trạng thái bơm
  // ═════════════════════════════════════════════════════════════════════════════
  
  bool isPumpRunning() {
    return pumpActive;
  }
  
  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Lấy số lần chu kỳ bơm hiện tại
  // ═════════════════════════════════════════════════════════════════════════════
  
  uint8_t getPumpCycleCount() {
    return pumpCycles;
  }
  
  // ═════════════════════════════════════════════════════════════════════════════
  // CẬP NHẬT ĐỢT 1: Lấy thời gian xử lý lửa hiện tại (ms)
  // ═════════════════════════════════════════════════════════════════════════════
  
  uint32_t getFireElapsedMs() {
    if (!activeFire) return 0;
    return millis() - fireStart;
  }
  
  // ═════════════════════════════════════════════════════════════════════════════
  // Dừng khẩn cấp: tắt bơm và dừng xe ngay lập tức.
  // ═════════════════════════════════════════════════════════════════════════════
  
  void emergencyStop() {
    if (pumpActive) {
      MotorControl::setPump(false);
      pumpActive = false;
    }
    MotorControl::stop();
    activeFire = false;
    fireSessionLocked = true;
    currentDirection = NONE;
    pumpCycles = 0;
    bodyTurnInProgress = false;
    bodyTurnAligned = false;
    bodyTurnImuValid = false;
    bodyTurnDirection = NONE;
    gasConfirmedOnce = false;
    Serial.println(F("[FIRE] EMERGENCY STOP"));
  }
}
