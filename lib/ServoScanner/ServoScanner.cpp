#include <ESP32Servo.h>
#include "ServoScanner.h"
#include "RobotConfig.h"

/*
 * Quét radar không dùng delay chặn. Mỗi lần update chỉ chuyển một trạng thái;
 * phép đo chỉ được phép sau khoảng ổn định cơ khí. Nhờ vậy các tác vụ an toàn,
 * lửa và Wi-Fi vẫn chạy trong lúc servo đang di chuyển.
 */

// Namespace ẩn danh: trạng thái của bộ quét servo chỉ dùng nội bộ trong file này
namespace {
  // Đối tượng servo gắn cảm biến siêu âm để quét
  Servo scannerServo;
  // Đang trong một chu kỳ quét hay không
  bool scanActive = false;
  // Có một phép đo đang chờ được thực hiện (servo đã quay tới góc mới)
  bool measurementPending = false;
  // Góc hiện tại mà servo đang được đặt tới
  uint8_t scanAngle = SERVO_SCAN_MIN_DEG;
  // Góc ứng với phép đo đang chờ (dùng để báo cho bên đo biết đo ở góc nào)
  uint8_t measurementAngle = SERVO_SCAN_MIN_DEG;
  // Đã hoàn thành trọn một chu kỳ quét (đi tới rồi quay về) hay chưa
  bool cycleComplete = false;
  // Máy trạng thái của quá trình quét:
  // quét tiến từ góc nhỏ tới lớn -> dừng ở góc lớn nhất -> quét lùi về góc nhỏ -> kết thúc
  enum class ScanState : uint8_t {
    MOVING_FORWARD,
    PAUSE_AT_MAX,
    MOVING_BACKWARD,
    FINISHED
  };
  ScanState state = ScanState::FINISHED;
  // Chiều quét: +1 là tăng góc, -1 là giảm góc
  int8_t scanDirection = 1;
  // Thời điểm (ms) bắt đầu trạng thái hiện tại
  uint32_t stateStartMs = 0;
  // Thời điểm (ms) mà phép đo đang chờ được phép thực hiện (sau khi servo ổn định)
  uint32_t measurementDueMs = 0;

  // Ghi góc cho servo, có giới hạn trong khoảng [MIN, MAX] để bảo vệ cơ cấu
  void writeAngle(uint8_t angle) {
    scanAngle = constrain(angle, SERVO_SCAN_MIN_DEG, SERVO_SCAN_MAX_DEG);
    scannerServo.write(scanAngle);
  }
}

namespace ServoScanner {
  // Khởi tạo: gắn servo vào chân điều khiển và đưa về góc chính giữa
  void begin() {
    scannerServo.attach(PIN_SERVO);
    writeAngle(SERVO_SCAN_CENTER_DEG);
  }

  // Bắt đầu một chu kỳ quét mới từ góc nhỏ nhất
  void startScan() {
    // Phần lớn chu kỳ bắt đầu khi servo đã ở MIN; chỉ dùng thời gian chờ dài
    // khi servo thật sự phải đi từ vị trí khác, thường là CENTER.
    const bool longInitialMove = scanAngle != SERVO_SCAN_MIN_DEG;
    // Đặt lại toàn bộ trạng thái về đầu chu kỳ
    scanActive = true;
    measurementPending = true;
    cycleComplete = false;
    state = ScanState::MOVING_FORWARD;
    scanDirection = 1;
    stateStartMs = millis();
    // Phép đo đầu tiên được phép sau khi servo có thời gian ổn định (SETTLE)
    measurementDueMs = stateStartMs +
      (longInitialMove ? SERVO_SCAN_START_SETTLE_MS : SERVO_SCAN_SETTLE_MS);
    measurementAngle = SERVO_SCAN_MIN_DEG;
    scanAngle = SERVO_SCAN_MIN_DEG;
    // Quay servo về góc nhỏ nhất
    writeAngle(SERVO_SCAN_MIN_DEG);
  }

  // Cập nhật máy trạng thái, gọi lặp lại trong vòng lặp chính (không chặn)
  void update() {
    // Không quét thì không làm gì
    if (!scanActive) return;

    const uint32_t now = millis();
    // Đang dừng ở góc lớn nhất và đã hết thời gian dừng: chuyển sang quét lùi
    if (state == ScanState::PAUSE_AT_MAX &&
        now - stateStartMs >= SERVO_SCAN_END_PAUSE_MS) {
      state = ScanState::MOVING_BACKWARD;
      scanDirection = -1;
      stateStartMs = now;
      // Lùi một bước so với góc lớn nhất (góc lớn nhất đã đo rồi nên không đo lại)
      writeAngle((uint8_t)(SERVO_SCAN_MAX_DEG - SERVO_SCAN_STEP_DEG));
      measurementAngle = scanAngle;
      // Chờ servo ổn định rồi mới cho phép đo
      measurementDueMs = now + SERVO_SCAN_SETTLE_MS;
      measurementPending = true;
    }
  }

  // Trả về đúng nếu bộ quét còn hoạt động hoặc còn phép đo chưa xử lý.
  bool active() {
    return scanActive || measurementPending;
  }

  // Trả về đúng nếu có phép đo đang chờ và servo đã ổn định để bắt đầu đo.
  bool measurementDue() {
    return measurementPending && millis() >= measurementDueMs;
  }

  // Trả về góc mà phép đo hiện tại ứng với
  // (nếu đang chờ đo thì là góc của phép đo, ngược lại là góc servo hiện tại)
  uint8_t angle() {
    return measurementPending ? measurementAngle : scanAngle;
  }

  // Được gọi sau khi bên ngoài đã đo xong ở góc hiện tại: chuyển sang góc kế tiếp
  void measurementTaken() {
    // Chưa đến hạn đo thì bỏ qua, tránh xử lý nhầm
    if (!measurementDue()) return;

    // Đánh dấu phép đo hiện tại đã xong
    measurementPending = false;
    measurementDueMs = 0;
    // Đã đo xong ở góc lớn nhất khi đang quét tiến: chuyển sang trạng thái dừng
    if (state == ScanState::MOVING_FORWARD && scanAngle >= SERVO_SCAN_MAX_DEG) {
      state = ScanState::PAUSE_AT_MAX;
      stateStartMs = millis();
      return;
    }

    // Đã đo xong ở góc nhỏ nhất khi đang quét lùi: kết thúc chu kỳ quét
    if (state == ScanState::MOVING_BACKWARD && scanAngle <= SERVO_SCAN_MIN_DEG) {
      scanActive = false;
      cycleComplete = true;
      state = ScanState::FINISHED;
      return;
    }

    // Tính góc kế tiếp theo chiều quét hiện tại
    int16_t nextAngle = (int16_t)scanAngle +
                        scanDirection * SERVO_SCAN_STEP_DEG;
    // Kẹp góc vào [MIN, MAX] để không vượt phạm vi khi bước không chia hết khoảng quét
    if (nextAngle >= SERVO_SCAN_MAX_DEG) {
      nextAngle = SERVO_SCAN_MAX_DEG;
    } else if (nextAngle <= SERVO_SCAN_MIN_DEG) {
      nextAngle = SERVO_SCAN_MIN_DEG;
    }
    // Quay servo tới góc mới và hẹn thời điểm đo sau khi servo ổn định
    writeAngle((uint8_t)nextAngle);
    measurementAngle = scanAngle;
    measurementDueMs = millis() + SERVO_SCAN_SETTLE_MS;
    measurementPending = true;
  }

  // Trả về đúng khi cả chu kỳ quét đã xong và không còn phép đo chờ xử lý.
  bool cycleFinished() {
    return cycleComplete && !measurementPending;
  }

  // Dừng máy trạng thái nhưng giữ góc cơ khí hiện tại. Khi kết thúc vòng
  // 0-180-0, servo vốn đã nằm tại 0 độ.
  void holdPosition() {
    scanActive = false;
    measurementPending = false;
    cycleComplete = false;
    state = ScanState::FINISHED;
    scanDirection = 1;
    stateStartMs = 0;
    measurementDueMs = 0;
  }

  void powerOff() {
    holdPosition();
    if (scannerServo.attached()) scannerServo.detach();
    Serial.println(F("[RADAR] SERVO PWM OFF"));
  }

  // Dừng quét: đặt lại mọi trạng thái và đưa servo về góc giữa
  void stop() {
    holdPosition();
    writeAngle(SERVO_SCAN_CENTER_DEG);
  }
}
