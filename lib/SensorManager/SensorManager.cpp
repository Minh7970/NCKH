#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include "SensorManager.h"
#include "RobotConfig.h"
#include "FlameServoScanner.h"

/*
 * Luồng xử lý cảm biến:
 * - Đọc ADC và mức số, kiểm tra phạm vi hợp lệ.
 * - Học mức nền chậm để ánh sáng môi trường ổn định không bị coi là lửa.
 * - Khi tín hiệu hồng ngoại tăng mạnh, giữ servo và kiểm tra đặc trưng nhấp
 *   nháy theo thời gian; lửa thật phải vừa mạnh vừa có dao động.
 * - MQ-2 được làm nóng, so với nền không khí sạch và xác nhận qua nhiều mẫu.
 * - Siêu âm và MPU6050 được cập nhật định kỳ, sau đó xuất bản nguyên tử thành
 *   một SensorSnapshot cho Navigation, FireControl và giao diện web.
 */

// Namespace ẩn danh: trạng thái cảm biến và các hàm hỗ trợ chỉ dùng nội bộ trong file này
namespace {
  // Bản chụp dữ liệu cảm biến mới nhất, được chia sẻ cho các module khác qua get()
  SensorSnapshot s = {};
  // Bộ đếm số mẫu liên tiếp phát hiện lửa (dùng để xác nhận, lọc nhiễu)
  uint8_t fireCount = 0;
  // Thời điểm (ms) đo siêu âm chính lần gần nhất
  uint32_t lastUltrasonic = 0;
  // Thời điểm (ms) của mẫu siêu âm chính gần nhất
  uint32_t ultrasonicSampleMs = 0;
  // Thời điểm bắt đầu hiệu chuẩn cảm biến lửa/gas (dùng cho thời gian khởi động)
  uint32_t fireCalibrationStart = 0;
  // Thời điểm in debug gần nhất
  uint32_t lastDebugPrint = 0;
  // Khoảng cách hợp lệ gần nhất của siêu âm chính (cm)
  uint16_t lastDistanceCm = MAX_RANGE_CM;
  // Lần đo siêu âm chính gần nhất có hợp lệ không
  bool ultrasonicValid = false;
  // Thời điểm đo siêu âm SA2 (cảm biến tránh vật cản phía trước) lần gần nhất
  uint32_t lastSA2 = 0;
  // Thời điểm của mẫu SA2 gần nhất
  uint32_t sa2SampleMs = 0;
  // Khoảng cách hợp lệ gần nhất của SA2 (cm)
  uint16_t lastSA2DistanceCm = MAX_RANGE_CM;
  // Lần đo SA2 gần nhất có hợp lệ không
  bool sa2Valid = false;
  // Khi bật, siêu âm chính do module quét servo điều khiển nên update() không tự đo
  bool ultrasonicScanMode = false;
  SemaphoreHandle_t ultrasonicMutex = nullptr;
  // Mức nền (ambient) của cảm biến khí gas MQ-2
  uint16_t mq2Ambient = 0;
  // Bộ đếm số mẫu liên tiếp phát hiện gas
  uint8_t gasCount = 0;
  // Đã khởi tạo mức nền MQ-2 chưa
  bool mq2AmbientInitialized = false;
  // Mức nền của 2 cảm biến lửa (trái, phải)
  constexpr uint8_t FLAME_SENSOR_COUNT = 2;
  uint16_t flameAmbient[FLAME_SENSOR_COUNT] = {};
  // Giá trị nhỏ nhất và lớn nhất trong cửa sổ đo dao động (nhấp nháy) của lửa
  uint16_t flickerMin[FLAME_SENSOR_COUNT] = {};
  uint16_t flickerMax[FLAME_SENSOR_COUNT] = {};
  // Biên độ dao động (max - min) của từng cảm biến lửa trong cửa sổ gần nhất
  uint16_t flickerDelta[FLAME_SENSOR_COUNT] = {};
  uint16_t flickerPrevious[FLAME_SENSOR_COUNT] = {};
  uint32_t flickerTotalVariation[FLAME_SENSOR_COUNT] = {};
  uint8_t flickerReversals[FLAME_SENSOR_COUNT] = {};
  int8_t flickerSlopeSign[FLAME_SENSOR_COUNT] = {};
  uint8_t flickerSaturatedSamples[FLAME_SENSOR_COUNT] = {};
  bool flameVerificationActive = false;
  bool analogueFireLatched = false;
  bool digitalFireLatched = false;
  uint8_t digitalFireCount = 0;
  uint8_t digitalFlameLostCount = 0;
  uint32_t flameVerificationStartMs = 0;
  uint32_t flameVerificationRejectUntilMs = 0;
  uint8_t flameVerificationSamples = 0;
  uint8_t flameLostCount = 0;
  // Đã khởi tạo mức nền cảm biến lửa chưa
  bool flameAmbientInitialized = false;
  bool mpuReady = false;
  float gyroBiasZDps = 0.0f;
  float accelXG = 0.0f;
  float accelYG = 0.0f;
  float accelZG = 0.0f;
  float yawDeg = 0.0f;
  float gyroZDps = 0.0f;
  uint32_t lastImuMs = 0;
  uint8_t mpuReadFailures = 0;
  // Khóa (spinlock) FreeRTOS bảo vệ bản chụp cảm biến khi nhiều tác vụ truy cập
  portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
  static uint32_t flameValidCount[FLAME_SENSOR_COUNT] = {};
  static uint32_t flameTotalCount[FLAME_SENSOR_COUNT] = {};

  static uint8_t flameHealth[FLAME_SENSOR_COUNT] = {100, 100};
  static SensorError lastSensorError = SENSOR_OK;
  static uint32_t sensorErrorCount = 0;

  bool writeMpu(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readMpu(uint8_t reg, uint8_t* data, uint8_t length) {
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    const uint8_t received = Wire.requestFrom(
      (int)MPU6050_I2C_ADDRESS, (int)length, (int)true);
    if (received != length) return false;
    for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
    return true;
  }

  bool readMpuMotion(float& ax, float& ay, float& az, float& gz) {
    uint8_t data[14];
    if (!readMpu(0x3B, data, sizeof(data))) return false;

    const int16_t rawAx = (int16_t)((data[0] << 8) | data[1]);
    const int16_t rawAy = (int16_t)((data[2] << 8) | data[3]);
    const int16_t rawAz = (int16_t)((data[4] << 8) | data[5]);
    const int16_t rawGz = (int16_t)((data[12] << 8) | data[13]);
    ax = (float)rawAx / 16384.0f;
    ay = (float)rawAy / 16384.0f;
    az = (float)rawAz / 16384.0f;
    gz = (float)rawGz / MPU6050_GYRO_SCALE_DPS - gyroBiasZDps;
    if (fabsf(gz) < MPU6050_GYRO_DEADBAND_DPS) gz = 0.0f;
    return true;
  }

  void initMpu() {
    mpuReady = false;
    if (!MPU6050_ENABLED) {
      Serial.println(F("[IMU] MPU6050 disabled"));
      return;
    }

    uint8_t whoAmI = 0;
    if (!readMpu(0x75, &whoAmI, 1) ||
        (whoAmI != 0x68 && whoAmI != 0x69 && whoAmI != 0x70)) {
      Serial.printf("[IMU] MPU6050 not found at 0x%02X\n",
                    MPU6050_I2C_ADDRESS);
      return;
    }

    if (!writeMpu(0x6B, 0x00) ||
        !writeMpu(0x1A, 0x03) ||
        !writeMpu(0x1B, 0x00) ||
        !writeMpu(0x1C, 0x00)) {
      Serial.println(F("[IMU] MPU6050 configuration failed"));
      return;
    }
    delay(50);

    float sumZDps = 0.0f;
    uint16_t validSamples = 0;
    for (uint16_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; ++i) {
      uint8_t data[2];
      if (readMpu(0x47, data, sizeof(data))) {
        const int16_t rawGz = (int16_t)((data[0] << 8) | data[1]);
        sumZDps += (float)rawGz / MPU6050_GYRO_SCALE_DPS;
        ++validSamples;
      }
      delay(5);
    }

    if (validSamples < MPU6050_CALIBRATION_SAMPLES / 2) {
      Serial.printf("[IMU] calibration failed samples=%u/%u\n",
                    validSamples, MPU6050_CALIBRATION_SAMPLES);
      return;
    }

    gyroBiasZDps = sumZDps / validSamples;
    yawDeg = 0.0f;
    gyroZDps = 0.0f;
    mpuReadFailures = 0;
    lastImuMs = millis();
    mpuReady = true;
    Serial.printf("[IMU] MPU6050 READY SDA=%u SCL=%u biasZ=%.3f dps\n",
                  PIN_MPU_SDA, PIN_MPU_SCL, gyroBiasZDps);
  }

  void updateMpu(uint32_t now) {
    if (!mpuReady || now - lastImuMs < MPU6050_READ_PERIOD_MS) return;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gz = 0.0f;
    if (!readMpuMotion(ax, ay, az, gz)) {
      if (mpuReadFailures < 255) ++mpuReadFailures;
      if (mpuReadFailures >= MPU6050_MAX_READ_FAILURES) {
        mpuReady = false;
        Serial.println(F("[IMU] MPU6050 read failed; timed-turn fallback active"));
      }
      return;
    }

    const float dt = (now - lastImuMs) / 1000.0f;
    accelXG = ax;
    accelYG = ay;
    accelZG = az;
    gyroZDps = gz;
    yawDeg += gz * dt * MPU6050_YAW_SIGN;
    lastImuMs = now;
    mpuReadFailures = 0;
  }

  bool isFlameValueValid(uint16_t rawValue) {
    // Bộ chuyển đổi tương tự - số của ESP32 có độ phân giải 12 bit.
    return rawValue <= 4095;
  }

  bool isFlameSignalValid(uint16_t signal) {
    return signal <= 4095;
  }

  // Tính cường độ tín hiệu lửa = độ lệch của giá trị đo so với mức nền
  // (tùy loại cảm biến mà lửa làm giá trị giảm hoặc tăng)
  uint16_t flameSignal(uint16_t ambient, uint16_t raw) {
    // Cảm biến tích cực mức thấp: có lửa thì giá trị đo giảm xuống dưới mức nền
    if (FLAME_SIGNAL_ACTIVE_LOW)
      return ambient > raw ? ambient - raw : 0;
    return raw > ambient ? raw - ambient : 0;
  }

  uint16_t requiredFlameFlicker(uint16_t signal) {
    const uint16_t relative =
      (uint16_t)((uint32_t)signal * FLAME_FLICKER_MIN_PERCENT / 100UL);
    return relative > FLAME_FLICKER_DELTA ? relative : FLAME_FLICKER_DELTA;
  }

  bool flameAdcSaturated(uint16_t raw) {
    return raw <= FLAME_ADC_SATURATION_MARGIN ||
      raw >= (uint16_t)(4095U - FLAME_ADC_SATURATION_MARGIN);
  }

  // Cập nhật mức nền của cảm biến lửa bằng bộ lọc trung bình trượt (15/16 cũ + 1/16 mới)
  void updateAmbient(const uint16_t raw[FLAME_SENSOR_COUNT]) {
    // Thích nghi chậm giúp ánh sáng phòng thay đổi dần trở thành mức nền nhưng
    // không hấp thụ nhanh một ngọn lửa đã được xác nhận vào mức nền.
    for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i)
      flameAmbient[i] = (uint16_t)((flameAmbient[i] * 15UL + raw[i]) / 16UL);
  }

  // Tính cường độ tín hiệu gas = độ lệch của giá trị MQ-2 so với mức nền
  uint16_t gasSignal(uint16_t ambient, uint16_t raw) {
    if (MQ2_SIGNAL_ACTIVE_HIGH)
      return raw > ambient ? raw - ambient : 0;
    return ambient > raw ? ambient - raw : 0;
  }

  // Cập nhật mức nền MQ-2 bằng trung bình trượt (31/32 cũ + 1/32 mới)
  void updateGasAmbient(uint16_t raw) {
    // MQ-2 thay đổi chậm theo nhiệt độ và không khí phòng nên nền cũng đổi chậm.
    mq2Ambient = (uint16_t)((mq2Ambient * 31UL + raw) / 32UL);
  }

  void updateGasAmbientAfterWarmup(uint16_t raw) {
    // Sau khi làm nóng, mức nền phải chậm hơn nhiều so với một sự kiện gas đang
    // tăng. Bộ lọc 31/32 cũ chạy mỗi 30 ms có thể hấp thụ luôn mức tăng MQ-2
    // trước khi tín hiệu kịp đạt ngưỡng cảnh báo.
    mq2Ambient = (uint16_t)((mq2Ambient * 255UL + raw) / 256UL);
  }

  // Đo khoảng cách bằng cảm biến siêu âm HC-SR04/SRF04, trả về cm (0 nếu không hợp lệ)
  uint16_t readUltrasonic(uint8_t trigPin, uint8_t echoPin) {
    if (!ultrasonicMutex ||
        xSemaphoreTake(ultrasonicMutex,
                       pdMS_TO_TICKS(ULTRASONIC_TIMEOUT_US / 1000UL + 20UL)) != pdTRUE) {
      return 0;
    }

    // Phát xung kích 10 µs ở chân TRIG
    digitalWrite(trigPin, LOW);
    delayMicroseconds(3);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Đo độ rộng xung phản hồi ở chân ECHO, có giới hạn thời gian chờ
    unsigned long us = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
    // Không có tiếng vọng (hết thời gian chờ) thì trả về 0
    if (us == 0) {
      xSemaphoreGive(ultrasonicMutex);
      return 0;
    }
    // Đổi thời gian sang cm (khoảng 58 µs cho mỗi cm khứ hồi)
    uint16_t cm = (uint16_t)(us / 58UL);
    // Bỏ các giá trị quá gần hoặc vượt tầm đo tối đa
    if (cm == 0 || cm > MAX_RANGE_CM) {
      xSemaphoreGive(ultrasonicMutex);
      return 0;
    }
    xSemaphoreGive(ultrasonicMutex);
    return cm;
  }
}

// Các hàm tiện ích công khai được khai báo trong SensorManager.h.
bool isFlameAmbientValid(uint16_t ambient) {
  return ambient <= 4095;
}

uint8_t getFlameLeftHealth() {
  return flameHealth[0];
}

uint8_t getFlameRightHealth() {
  return flameHealth[1];
}

SensorError getLastError() {
  return lastSensorError;
}

void clearError() {
  lastSensorError = SENSOR_OK;
  sensorErrorCount = 0;
}

namespace SensorManager {
  // Khởi tạo các chân siêu âm, cảm biến lửa/gas và ADC
  void begin() {
    if (!ultrasonicMutex) ultrasonicMutex = xSemaphoreCreateMutex();

    if (MPU6050_ENABLED) {
      Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
      Wire.setClock(MPU6050_I2C_FREQUENCY);
    }
    initMpu();

    // Cấu hình chân: TRIG là ngõ ra, ECHO và các cảm biến là ngõ vào
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_SA2_TRIG, OUTPUT);
    pinMode(PIN_SA2_ECHO, INPUT);
    pinMode(PIN_FLAME_LEFT, INPUT);
    pinMode(PIN_FLAME_RIGHT, INPUT);
    if (MQ2_SENSOR_ENABLED) pinMode(PIN_MQ2, INPUT);
    if (MWIR_SENSOR_ENABLED) pinMode(PIN_MWIR_CONFIRM, INPUT);
    // ADC 12 bit (giá trị 0..4095) và suy hao 11 dB để đo được dải điện áp rộng (ESP32)
    analogReadResolution(12);
    if (!FLAME_USE_DIGITAL_OUTPUT) {
      analogSetPinAttenuation(PIN_FLAME_LEFT, ADC_11db);
      analogSetPinAttenuation(PIN_FLAME_RIGHT, ADC_11db);
    }
    if (MQ2_SENSOR_ENABLED) analogSetPinAttenuation(PIN_MQ2, ADC_11db);
    // Đưa chân TRIG về mức thấp ban đầu
    digitalWrite(PIN_TRIG, LOW);
    digitalWrite(PIN_SA2_TRIG, LOW);
    // Ghi thời điểm bắt đầu để tính thời gian khởi động/làm nóng cảm biến
    fireCalibrationStart = millis();
    // Giá trị khoảng cách ban đầu là tầm đo tối đa (coi như chưa có vật cản)
    lastDistanceCm = MAX_RANGE_CM;
    lastSA2DistanceCm = MAX_RANGE_CM;
    // Điền các trường ban đầu của bản chụp cảm biến
    s.distanceCm = lastDistanceCm;
    s.sa2DistanceCm = lastSA2DistanceCm;
    s.accelXG = accelXG;
    s.accelYG = accelYG;
    s.accelZG = accelZG;
    s.yawDeg = yawDeg;
    s.gyroZDps = gyroZDps;
    s.imuValid = mpuReady;
    s.imuMs = lastImuMs;
    s.heartbeatMs = fireCalibrationStart;
  }

  // Hàm cập nhật chính, gọi lặp lại: đọc mọi cảm biến, lọc nhiễu, xác nhận lửa/gas
  // và ghi kết quả vào bản chụp cảm biến
  void update() {
    const bool flameDigitalLevel[FLAME_SENSOR_COUNT] = {
      digitalRead(PIN_FLAME_LEFT) == HIGH,
      digitalRead(PIN_FLAME_RIGHT) == HIGH
    };
    const bool digitalFlameActive[FLAME_SENSOR_COUNT] = {
      FLAME_DIGITAL_ACTIVE_LOW ? !flameDigitalLevel[0] : flameDigitalLevel[0],
      FLAME_DIGITAL_ACTIVE_LOW ? !flameDigitalLevel[1] : flameDigitalLevel[1]
    };
    // Ở chế độ số, quy đổi DO thành 0/4095 để tương thích API trạng thái hiện
    // có. Ở chế độ tương tự, đọc điện áp AO theo cách bình thường.
    uint16_t raw[FLAME_SENSOR_COUNT] = {
      FLAME_USE_DIGITAL_OUTPUT
        ? (uint16_t)(flameDigitalLevel[0] ? 4095 : 0)
        : (uint16_t)analogRead(PIN_FLAME_LEFT),
      FLAME_USE_DIGITAL_OUTPUT
        ? (uint16_t)(flameDigitalLevel[1] ? 4095 : 0)
        : (uint16_t)analogRead(PIN_FLAME_RIGHT)
    };
    const uint16_t mq2 = MQ2_SENSOR_ENABLED
      ? (uint16_t)analogRead(PIN_MQ2) : 0;
    const uint32_t now = millis();
    updateMpu(now);

    bool flameRawValid[FLAME_SENSOR_COUNT] = {};

    for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
        flameRawValid[i] = isFlameValueValid(raw[i]);

        ++flameTotalCount[i];

        if (!flameRawValid[i]) {
            lastSensorError = (i == 0)
                ? FLAME_LEFT_OUT_OF_RANGE
                : FLAME_RIGHT_OUT_OF_RANGE;

            ++sensorErrorCount;
            continue;
        }

        ++flameValidCount[i];

        // Tính health theo cửa sổ 100 mẫu
        if (flameTotalCount[i] >= 100) {
            flameHealth[i] = static_cast<uint8_t>(
                (flameValidCount[i] * 100UL) / flameTotalCount[i]
            );

            flameTotalCount[i] = 0;
            flameValidCount[i] = 0;
        }
    }
    uint16_t mq2SignalValue = 0;
    bool gasReady = false;
    bool gasDetected = false;
    if (MQ2_SENSOR_ENABLED) {
      // Lần đầu: lấy giá trị hiện tại làm mức nền của MQ-2.
      if (!mq2AmbientInitialized) {
        mq2Ambient = mq2;
        mq2AmbientInitialized = true;
      }
      gasReady = now - fireCalibrationStart >= MQ2_WARMUP_MS;
      mq2SignalValue = gasSignal(mq2Ambient, mq2);
      const bool rawGas = gasReady && mq2SignalValue >= MQ2_DETECT_DELTA;
      if (!gasReady) {
        gasCount = 0;
        updateGasAmbient(mq2);
      } else if (rawGas) {
        if (gasCount < MQ2_CONFIRM_SAMPLES) ++gasCount;
      } else {
        gasCount = 0;
        // Không học trạng thái lửa quang học đang hoạt động vào mức nền không
        // khí sạch khi MQ-2 đang được dùng để xác nhận cháy.
        if (!analogueFireLatched && !digitalFireLatched)
          updateGasAmbientAfterWarmup(mq2);
      }
      gasDetected = gasCount >= MQ2_CONFIRM_SAMPLES;
    } else {
      gasCount = 0;
    }

    bool mwirDetected = false;
    if (MWIR_SENSOR_ENABLED) {
      const bool mwirRaw = digitalRead(PIN_MWIR_CONFIRM) == HIGH;
      mwirDetected = MWIR_CONFIRM_ACTIVE_HIGH ? mwirRaw : !mwirRaw;
    }

    // Lần đầu: lấy giá trị hiện tại làm mức nền của cảm biến lửa
    if (!flameAmbientInitialized) {
      for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) flameAmbient[i] = raw[i];
      flameAmbientInitialized = true;
    }
    // Hệ thống phát hiện lửa chỉ "vũ trang" sau khoảng trễ FIRE_ARM_DELAY_MS
    const bool ready = now - fireCalibrationStart >= FIRE_ARM_DELAY_MS;
    uint16_t signal[FLAME_SENSOR_COUNT];
    bool flameChannel[FLAME_SENSOR_COUNT];
    uint16_t strongestSignal = 0;
  for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
    signal[i] = 0;
    flameChannel[i] = false;

    // Không sử dụng mẫu ADC không hợp lệ để xác nhận lửa
    if (!flameRawValid[i]) {
        continue;
    }

    if (FLAME_USE_DIGITAL_OUTPUT) {
      signal[i] = digitalFlameActive[i] ? 4095 : 0;
      if (signal[i] > strongestSignal) strongestSignal = signal[i];
      continue;
    }

    signal[i] = flameSignal(flameAmbient[i], raw[i]);

    if (!isFlameSignalValid(signal[i])) {
        lastSensorError = FLAME_SIGNAL_INVALID;
        ++sensorErrorCount;
        continue;
    }

    if (signal[i] > strongestSignal) {
        strongestSignal = signal[i];
    }

    }

    const bool strongCandidate = !FLAME_USE_DIGITAL_OUTPUT && ready &&
      (signal[0] >= FLAME_DETECT_DELTA ||
       signal[1] >= FLAME_DETECT_DELTA);

    // Giai đoạn 1: thay đổi mạnh theo hướng chỉ bắt đầu bước xác minh. Giữ servo
    // đứng yên để chuyển động/góc quét không làm sai dữ liệu nhấp nháy tiếp theo.
    if (!flameVerificationActive && !analogueFireLatched && strongCandidate &&
        (int32_t)(now - flameVerificationRejectUntilMs) >= 0) {
      flameVerificationActive = true;
      flameVerificationStartMs = now;
      flameVerificationSamples = 0;
      fireCount = 0;
      for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
        flickerMin[i] = raw[i];
        flickerMax[i] = raw[i];
        flickerDelta[i] = 0;
        flickerPrevious[i] = raw[i];
        flickerTotalVariation[i] = 0;
        flickerReversals[i] = 0;
        flickerSlopeSign[i] = 0;
        flickerSaturatedSamples[i] = 0;
      }
      FlameServoScanner::setHold(true);
    }

    // Giai đoạn 2: sau khi cơ cấu ổn định, chỉ đo dao động theo thời gian tại
    // hướng cố định. Ánh nắng ổn định có thể mạnh nhưng ít dao động; lửa thật
    // thường đồng thời có cường độ lớn và đặc trưng nhấp nháy.
    if (flameVerificationActive) {
      const uint32_t verifyElapsed = now - flameVerificationStartMs;
      if (verifyElapsed < FLAME_VERIFY_SETTLE_MS) {
        flameVerificationSamples = 0;
        for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
          flickerMin[i] = raw[i];
          flickerMax[i] = raw[i];
          flickerDelta[i] = 0;
          flickerPrevious[i] = raw[i];
          flickerTotalVariation[i] = 0;
          flickerReversals[i] = 0;
          flickerSlopeSign[i] = 0;
          flickerSaturatedSamples[i] = 0;
        }
      } else {
        if (flameVerificationSamples == 0) {
          for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
            flickerMin[i] = flickerMax[i] = raw[i];
            flickerPrevious[i] = raw[i];
            flickerTotalVariation[i] = 0;
            flickerReversals[i] = 0;
            flickerSlopeSign[i] = 0;
          }
        } else {
          for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
            if (raw[i] < flickerMin[i]) flickerMin[i] = raw[i];
            if (raw[i] > flickerMax[i]) flickerMax[i] = raw[i];

            const int32_t step = (int32_t)raw[i] - flickerPrevious[i];
            const uint16_t magnitude =
              (uint16_t)(step < 0 ? -step : step);
            if (magnitude >= FLAME_FLICKER_STEP_DELTA) {
              const int8_t newSign = step > 0 ? 1 : -1;
              if (flickerSlopeSign[i] != 0 &&
                  newSign != flickerSlopeSign[i] &&
                  flickerReversals[i] < 255) {
                ++flickerReversals[i];
              }
              flickerSlopeSign[i] = newSign;
              flickerTotalVariation[i] += magnitude;
            }
            flickerPrevious[i] = raw[i];
          }
        }
        for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
          if (flameAdcSaturated(raw[i]) &&
              flickerSaturatedSamples[i] < 255) {
            ++flickerSaturatedSamples[i];
          }
        }
        if (flameVerificationSamples < 255) ++flameVerificationSamples;
        for (uint8_t i = 0; i < FLAME_SENSOR_COUNT; ++i) {
          flickerDelta[i] = flickerMax[i] - flickerMin[i];
          const uint16_t requiredFlicker = requiredFlameFlicker(signal[i]);
          const bool enoughVariation = flickerTotalVariation[i] >=
            (uint32_t)requiredFlicker *
              FLAME_FLICKER_MIN_VARIATION_MULTIPLIER;
          const bool saturated =
            (uint16_t)flickerSaturatedSamples[i] * 100U >
            (uint16_t)flameVerificationSamples *
              FLAME_MAX_SATURATED_PERCENT;
          flameChannel[i] =
            flameVerificationSamples >= FLAME_VERIFY_MIN_SAMPLES &&
            signal[i] >= FLAME_DETECT_DELTA &&
            flickerDelta[i] >= requiredFlicker &&
            flickerReversals[i] >= FLAME_FLICKER_MIN_REVERSALS &&
            enoughVariation && !saturated;
        }
      }
    }

    const bool rawFire = ready && flameVerificationActive &&
      (flameChannel[0] || flameChannel[1]);

    if (rawFire) {
      // Tăng bộ đếm xác nhận (có giới hạn trên)
      if (fireCount < FIRE_CONFIRM_SAMPLES) fireCount++;
    } else if (!analogueFireLatched) {
      fireCount = 0;
    }

    if (!analogueFireLatched && fireCount >= FIRE_CONFIRM_SAMPLES) {
      analogueFireLatched = true;
      flameLostCount = 0;
      Serial.printf(
        "[FIRE] verified L(delta=%u rev=%u sat=%u/%u) R(delta=%u rev=%u sat=%u/%u)\n",
        flickerDelta[0], flickerReversals[0], flickerSaturatedSamples[0],
        flameVerificationSamples, flickerDelta[1], flickerReversals[1],
        flickerSaturatedSamples[1], flameVerificationSamples);
    }

    if (analogueFireLatched) {
      if (rawFire) {
        flameLostCount = 0;
      } else {
        // Servo và thân xe chuyển động đồng bộ nên góc nhìn quang học cố ý thay
        // đổi. Không xóa xác nhận lửa trong khoảng mất tín hiệu tạm thời này;
        // nếu không Navigation sẽ chạy tiếp trước khi xe căn xong về 0 độ.
        const uint8_t lostLimit = FlameServoScanner::aiming()
          ? FLAME_ALIGNMENT_LOST_SAMPLES : FLAME_LOST_SAMPLES;
        if (++flameLostCount >= lostLimit) {
          analogueFireLatched = false;
          flameVerificationActive = false;
          flameLostCount = 0;
          fireCount = 0;
          flameVerificationRejectUntilMs = now + FLAME_VERIFY_REJECT_COOLDOWN_MS;
          FlameServoScanner::setHold(false);
        }
      }
    } else if (flameVerificationActive &&
               now - flameVerificationStartMs >= FLAME_VERIFY_TIMEOUT_MS) {
      // Hồng ngoại mạnh nhưng ổn định: coi là ánh sáng nền rồi tiếp tục quét.
      flameVerificationActive = false;
      fireCount = 0;
      flameVerificationRejectUntilMs = now + FLAME_VERIFY_REJECT_COOLDOWN_MS;
      FlameServoScanner::setHold(false);
      Serial.printf(
        "[FIRE] IR rejected L(delta=%u rev=%u sat=%u/%u) R(delta=%u rev=%u sat=%u/%u)\n",
        flickerDelta[0], flickerReversals[0], flickerSaturatedSamples[0],
        flameVerificationSamples, flickerDelta[1], flickerReversals[1],
        flickerSaturatedSamples[1], flameVerificationSamples);
    }

    if (FLAME_USE_DIGITAL_OUTPUT) {
      const bool rawDigitalFire = ready &&
        (digitalFlameActive[0] || digitalFlameActive[1]);
      if (rawDigitalFire) {
        FlameServoScanner::setHold(true);
        digitalFlameLostCount = 0;
        if (digitalFireCount < FIRE_CONFIRM_SAMPLES) ++digitalFireCount;
        if (!digitalFireLatched && digitalFireCount >= FIRE_CONFIRM_SAMPLES) {
          digitalFireLatched = true;
          Serial.printf("[FIRE] digital DO confirmed L=%d R=%d\n",
                        digitalFlameActive[0], digitalFlameActive[1]);
        }
      } else {
        digitalFireCount = 0;
        if (digitalFireLatched) {
          if (digitalFlameLostCount < 255) ++digitalFlameLostCount;
          if (digitalFlameLostCount >= FLAME_LOST_SAMPLES) {
            digitalFireLatched = false;
            digitalFlameLostCount = 0;
            FlameServoScanner::setHold(false);
            Serial.println(F("[FIRE] digital DO cleared"));
          }
        } else {
          digitalFlameLostCount = 0;
          FlameServoScanner::setHold(false);
        }
      }
    } else {
      digitalFireCount = 0;
      digitalFlameLostCount = 0;
      digitalFireLatched = false;
    }

    // Chọn đầu ra số LM393 hoặc bộ nhận dạng nhấp nháy tương tự tại góc cố
    // định. Các bước xác nhận tùy chọn chỉ hoạt động khi phần cứng tương ứng
    // được bật rõ ràng trong RobotConfig.
    const bool selectedFlameDetected = FLAME_USE_DIGITAL_OUTPUT
      ? digitalFireLatched : analogueFireLatched;
    const bool fireDetected = selectedFlameDetected &&
      (!REQUIRE_MWIR_CONFIRMATION || mwirDetected) &&
      (!REQUIRE_GAS_CONFIRMATION || gasDetected);
    // DO số không chứa thông tin biên độ nên lửa số đã xác nhận cũng được coi
    // là ở gần; chế độ tương tự vẫn sử dụng ngưỡng cường độ riêng.
    const bool fireNear = fireDetected &&
      (FLAME_USE_DIGITAL_OUTPUT || strongestSignal >= FLAME_NEAR_DELTA);

    // Nguồn sáng tĩnh được phép thích nghi dần thành một phần của mức nền.
    // Khi không có lửa thô, cho mức nền thích nghi theo (nguồn sáng tĩnh dần thành nền)
    if (!FLAME_USE_DIGITAL_OUTPUT &&
        !flameVerificationActive && !analogueFireLatched) updateAmbient(raw);

    // Đo siêu âm chính theo chu kỳ (bỏ qua khi đang ở chế độ quét servo vì module quét tự đo)
    if (!ultrasonicScanMode && now - lastUltrasonic >= ULTRASONIC_PERIOD_MS) {
      lastUltrasonic = now;
      uint16_t d = readUltrasonic(PIN_TRIG, PIN_ECHO);
      ultrasonicSampleMs = now;
      ultrasonicValid = d > 0;
      // Chỉ cập nhật khoảng cách khi số đo hợp lệ
      if (ultrasonicValid) lastDistanceCm = d;
    }

    // Đo siêu âm SA2 (tránh vật cản phía trước) theo chu kỳ, luôn hoạt động
    if (now - lastSA2 >= ULTRASONIC_PERIOD_MS) {
      lastSA2 = now;
      uint16_t d = readUltrasonic(PIN_SA2_TRIG, PIN_SA2_ECHO);
      sa2SampleMs = now;
      sa2Valid = d > 0;
      if (sa2Valid) lastSA2DistanceCm = d;
    }

    // Gom toàn bộ kết quả vào một bản chụp mới (thứ tự phải khớp định nghĩa SensorSnapshot)
    SensorSnapshot next = {
      raw[0], 0, raw[1],
      signal[0], 0, signal[1],
      mq2, mq2SignalValue, lastDistanceCm, ultrasonicValid, ultrasonicSampleMs,
      lastSA2DistanceCm, sa2Valid, sa2SampleMs,
      mwirDetected, gasDetected, gasReady, selectedFlameDetected,
      fireDetected, fireNear, ready,
      now,
      lastSensorError, sensorErrorCount, flameHealth[0], flameHealth[1],
      flickerDelta[0], flickerDelta[1], flameVerificationActive,
      accelXG, accelYG, accelZG, yawDeg, gyroZDps, mpuReady, lastImuMs
    };
    // Ghi bản chụp trong vùng găng để tác vụ khác đọc không bị dữ liệu dở dang
    taskENTER_CRITICAL(&sensorMux);
    s = next;
    taskEXIT_CRITICAL(&sensorMux);

    // In thông tin gỡ lỗi cảm biến lửa/gas theo chu kỳ khi bật SENSOR_DEBUG
    if (SENSOR_DEBUG && now - lastDebugPrint >= SENSOR_DEBUG_PERIOD_MS) {
      lastDebugPrint = now;
      Serial.printf("[FIRE] raw(L/R)=%u/%u signal(L/R)=%u/%u flicker(L/R)=%u/%u mwir=%d gas=%u/%u ready=%d fire=%d near=%d\n",
                    raw[0], raw[1], signal[0], signal[1],
                    flickerDelta[0], flickerDelta[1],
                    mwirDetected, mq2, mq2SignalValue, gasReady, fireDetected, fireNear);
    }

    // if (SA2_DEBUG && now - lastDebugPrint >= SA2_DEBUG_PERIOD_MS) {
    //   lastDebugPrint = now;
    //   Serial.printf("[SA2] khoang_cach=%u cm hop_le=%d nguong=%u cm\n",
    //                 (unsigned)(sa2Valid ? lastSA2DistanceCm : 0),
    //                 sa2Valid ? 1 : 0,
    //                 (unsigned)PHYSICAL_STOP_CM);
    // }
  }

  // Lấy bản sao dữ liệu cảm biến mới nhất theo cách an toàn giữa nhiều tác vụ.
  SensorSnapshot get() {
    taskENTER_CRITICAL(&sensorMux);
    SensorSnapshot copy = s;
    taskEXIT_CRITICAL(&sensorMux);
    return copy;
  }

  // Trả về khoảng cách siêu âm chính gần nhất (cm) từ bản chụp
  uint16_t ultrasonicCm() {
    return s.distanceCm;
  }

  // Đo siêu âm chính ngay lập tức (dùng khi quét servo, không phụ thuộc chu kỳ update)
  uint16_t readUltrasonicNow() {
    return readUltrasonic(PIN_TRIG, PIN_ECHO);
  }

  // Đo siêu âm SA2 ngay lập tức
  uint16_t readSA2Now() {
    return readUltrasonic(PIN_SA2_TRIG, PIN_SA2_ECHO);
  }

  // Bật/tắt chế độ quét: khi bật, update() ngừng tự đo siêu âm chính để tránh xung đột với servo
  void setUltrasonicScanMode(bool enabled) {
    ultrasonicScanMode = enabled;
  }

  // Trả về đúng nếu đã xác nhận có lửa.
  bool fireConfirmed() { return s.fireDetected; }

  // Trả về giá trị cảm biến lửa mạnh nhất trong 2 kênh (trái, phải)
  int strongestFlame() {
    uint16_t a = s.flameLeft;
    uint16_t b = s.flameRight;
    return max(a, b);
  }
}
