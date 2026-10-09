
#pragma once

#include <Arduino.h>

/*
 * SensorManager gom dữ liệu từ cảm biến lửa, MQ-2, hai cảm biến siêu âm và
 * MPU6050 thành một SensorSnapshot thống nhất. Các tác vụ khác chỉ đọc bản
 * chụp này, nhờ đó không gặp dữ liệu bị thay đổi dở dang giữa nhiều tác vụ.
 */

// Mã lỗi phải được khai báo trước cấu trúc SensorSnapshot.
enum SensorError : uint8_t {
    SENSOR_OK = 0,
    FLAME_LEFT_OUT_OF_RANGE,
    FLAME_RIGHT_OUT_OF_RANGE,
    FLAME_SIGNAL_INVALID
};

/*
 * Phần cứng hiện tại dùng hai cảm biến lửa dạng tương tự:
 * - Cảm biến trái nối với PIN_FLAME_LEFT.
 * - Cảm biến phải nối với PIN_FLAME_RIGHT.
 *
 * Hai trường flameCenter và flameCenterSignal vẫn được giữ để tương thích với
 * các phần mã cũ. Trong cấu hình hai cảm biến, hai trường ở giữa luôn bằng 0.
 */
struct SensorSnapshot {
    uint16_t flameLeft;
    uint16_t flameCenter;
    uint16_t flameRight;

    uint16_t flameLeftSignal;
    uint16_t flameCenterSignal;
    uint16_t flameRightSignal;

    uint16_t mq2;
    uint16_t mq2Signal;

    uint16_t distanceCm;
    bool ultrasonicValid;
    uint32_t ultrasonicMs;

    uint16_t sa2DistanceCm;
    bool sa2Valid;
    uint32_t sa2Ms;

    bool mwirDetected;
    bool gasDetected;
    bool gasReady;
    // Kết quả lửa quang học trước khi MQ-2 xác nhận. Navigation dùng cờ này để
    // dừng và căn thân xe; fireDetected bên dưới là báo động cuối đã có gas,
    // dùng để quyết định relay/máy bơm.
    bool opticalFireDetected;
    bool fireDetected;
    bool fireNear;
    bool ready;

    uint32_t heartbeatMs;

    // Thông tin tình trạng hoạt động và lỗi của cảm biến.
    SensorError lastError;
    uint32_t errorCount;
    uint8_t flameLeftHealth;
    uint8_t flameRightHealth;
    uint16_t flameLeftFlicker;
    uint16_t flameRightFlicker;
    bool flameVerifying;

    // Góc tương đối từ MPU6050. yawDeg được tích phân từ gyro trục Z, lấy hướng
    // lúc khởi động làm mốc; đây không phải hướng la bàn tuyệt đối.
    float accelXG;
    float accelYG;
    float accelZG;
    float yawDeg;
    float gyroZDps;
    bool imuValid;
    uint32_t imuMs;
};

// Kiểm tra giá trị có nằm trong giới hạn hợp lệ.
bool isFlameAmbientValid(uint16_t ambient);

// Kiểm tra tình trạng hoạt động của từng kênh cảm biến lửa.
uint8_t getFlameLeftHealth();
uint8_t getFlameRightHealth();

// Truy xuất thông tin lỗi cảm biến gần nhất.
SensorError getLastError();
void clearError();

namespace SensorManager {
    void begin();
    void update();
    SensorSnapshot get();

    uint16_t ultrasonicCm();
    uint16_t readUltrasonicNow();
    uint16_t readSA2Now();

    void setUltrasonicScanMode(bool enabled);

    bool fireConfirmed();
    int strongestFlame();
}
