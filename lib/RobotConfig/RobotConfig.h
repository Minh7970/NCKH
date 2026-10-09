#pragma once
#include <Arduino.h>



/*
 * Tập trung toàn bộ chân GPIO, ngưỡng cảm biến, tốc độ và thời gian hiệu chỉnh.
 * Các module chỉ đọc hằng số ở đây để tránh mỗi nơi sử dụng một giá trị khác
 * nhau. Khi hiệu chỉnh phần cứng nên đổi từng nhóm nhỏ rồi kiểm tra thực tế.
 */

// Điền mạng Wi-Fi cục bộ; để SSID trống sẽ dùng chế độ phát điểm truy cập dự phòng.
constexpr char WIFI_SSID[] = "Nha Tro235 L2";
constexpr char WIFI_PASSWORD[] = "TH909002@*@";
constexpr char WIFI_AP_SSID[] = "ESP32S3-FireRobot";
constexpr char WIFI_AP_PASSWORD[] = "12345678";
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

/*
 * Robot chữa cháy - ESP32 DevKitC V4, cấu hình PlatformIO dạng module.
 *
 * LƯU Ý:
 * HC-SR04 không đo tin cậy ở khoảng cách 1 cm; giới hạn thực tế khoảng 2-3 cm
 * và sai số thân xe/quay còn lớn hơn. Vì vậy TARGET_WALL_GAP_CM chỉ là mục
 * tiêu trên bản đồ. PHYSICAL_STOP_CM nên từ 5 cm trở lên để mô hình an toàn.
 */

// Kênh L298N bên trái.
constexpr uint8_t PIN_ENA = 25;
constexpr uint8_t PIN_IN1 = 26;
constexpr uint8_t PIN_IN2 = 27;

// Kênh L298N bên phải.
constexpr uint8_t PIN_ENB = 14;
constexpr uint8_t PIN_IN3 = 16;
constexpr uint8_t PIN_IN4 = 17;
 
// AO tương tự của cảm biến lửa: AO trái -> GPIO32, AO phải -> GPIO33.
constexpr uint8_t PIN_FLAME_LEFT   = 32;
constexpr uint8_t PIN_FLAME_RIGHT  = 33;
// Giữ sẵn chế độ số nhưng phần cứng hiện tại dùng AO và đặc trưng nhấp nháy.
constexpr bool FLAME_USE_DIGITAL_OUTPUT = false;
constexpr bool FLAME_DIGITAL_ACTIVE_LOW = true;

// MQ-2 dùng GPIO35 chỉ có khả năng nhập; tuyệt đối không dùng chân này điều khiển servo.
constexpr uint8_t PIN_MQ2 = 35;
// Ống bơm cố định thẳng mũi xe, trùng góc logic 0 độ của servo cảm biến lửa;
// phiên bản phần cứng hiện tại không có servo riêng cho vòi nước.
constexpr uint8_t PUMP_FIXED_HEADING_DEG = 0;
constexpr uint8_t FLAME_ALIGNMENT_TOLERANCE_DEG = 5;
// AO của MQ-2 nối GPIO35 và làm bước xác nhận không định hướng trước khi bật bơm.
constexpr bool MQ2_SENSOR_ENABLED = true;
constexpr bool MQ2_SIGNAL_ACTIVE_HIGH = true;
constexpr uint16_t MQ2_DETECT_DELTA = 350;
constexpr uint8_t MQ2_CONFIRM_SAMPLES = 10;
constexpr uint32_t MQ2_WARMUP_MS = 60000;
// Không chặn điều hướng khi khởi động; MQ-2 tăng độ tin cậy nhưng không phải
// điều kiện bắt buộc để xe bắt đầu hoạt động ở chế độ hiện tại.
constexpr bool MQ2_SLEEP_MODE_ENABLED = false;
constexpr uint32_t MQ2_SLEEP_INTERVAL_MS = 500;
// Sau khi phủ hết ô FREE có thể tới, thức ngắn mỗi giây để kiểm tra an toàn và
// trạng thái, sau đó quay lại light sleep.
constexpr uint32_t COVERAGE_SLEEP_INTERVAL_MS = 1000;

/*
 * Nếu lắp cảm biến MWIR thật loại 3-5 um có bộ lọc dải thông tích hợp thì tín
 * hiệu xác nhận số của nó sẽ nối vào chân này; đây không phải ngõ vào cảm biến
 * lửa thông thường. GPIO36 chỉ nhận tín hiệu vào và cần điện trở kéo xuống 10 kΩ.
 * Phần cứng hiện tại không lắp MWIR nên cấu hình bên dưới đang vô hiệu hóa nó.
 */
constexpr uint8_t PIN_MWIR_CONFIRM = 36;
constexpr bool MWIR_SENSOR_ENABLED = false;
constexpr bool MWIR_CONFIRM_ACTIVE_HIGH = true;
// Phần cứng hiện tại không dùng MWIR; hai kênh lửa tương tự xác định hướng.
constexpr bool REQUIRE_MWIR_CONFIRMATION = false;
// Cảnh báo cuối chỉ hợp lệ khi MQ-2 cũng xác nhận khí gas/khói.
constexpr bool REQUIRE_GAS_CONFIRMATION = true;
static_assert(!REQUIRE_MWIR_CONFIRMATION || MWIR_SENSOR_ENABLED,
              "MWIR confirmation requires an installed MWIR sensor");
static_assert(!REQUIRE_GAS_CONFIRMATION || MQ2_SENSOR_ENABLED,
              "Gas confirmation requires an installed MQ-2 sensor");

// MPU6050 đo hướng tương đối. GPIO21/22 đã dùng cho siêu âm quét nên I2C được
// ánh xạ sang chân khác: VCC->3V3, GND->GND, SDA->GPIO23, SCL->GPIO5, AD0->GND.
constexpr bool MPU6050_ENABLED = true;
constexpr uint8_t PIN_MPU_SDA = 23;
constexpr uint8_t PIN_MPU_SCL = 5;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68;
constexpr uint32_t MPU6050_I2C_FREQUENCY = 400000;
constexpr uint16_t MPU6050_CALIBRATION_SAMPLES = 120;
constexpr uint32_t MPU6050_READ_PERIOD_MS = 20;
constexpr float MPU6050_GYRO_SCALE_DPS = 131.0f; // Thang đo +/-250 độ/giây.
// Chỉ dùng -1 khi module lắp ngược khiến hiệu chỉnh hướng làm sai số tăng lên.
constexpr int8_t MPU6050_YAW_SIGN = 1;
constexpr float MPU6050_GYRO_DEADBAND_DPS = 0.45f;
constexpr uint8_t MPU6050_MAX_READ_FAILURES = 5;
constexpr uint32_t MPU6050_FRESH_MS = 150;

// Điều khiển yaw tương đối: MPU6050 không có từ kế nên hướng lúc khởi động được
// coi là Bắc; gyro chỉ duy trì các góc quay tương đối 90/180 độ sau đó.
constexpr float MPU_TURN_90_STOP_DEG = 88.0f;
constexpr float MPU_TURN_180_STOP_DEG = 178.0f;
constexpr uint32_t MPU_TURN_MIN_MS = 100;
constexpr uint32_t MPU_TURN_MAX_90_MS = 2500;
constexpr float MPU_HEADING_HOLD_DEADBAND_DEG = 1.0f;
constexpr float MPU_HEADING_HOLD_KP = 2.0f;
constexpr int16_t MPU_HEADING_HOLD_MAX_PWM = 35;
constexpr float MPU_FIRE_RETURN_TOLERANCE_DEG = 3.0f;
constexpr uint32_t MPU_FIRE_RETURN_MAX_MS = 5000;
constexpr float MPU_FIRE_AIM_TOLERANCE_DEG = 3.0f;

// Cảm biến siêu âm chính HC-SR04 gắn trên servo radar.
constexpr uint8_t PIN_TRIG = 21;
constexpr uint8_t PIN_ECHO = 22;
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 20000;

// SA2: SRF05 cố định ở mũi xe để tránh vật cản xuất hiện khi đang chạy.
constexpr uint8_t PIN_SA2_TRIG = 4;
constexpr uint8_t PIN_SA2_ECHO = 13;

// Relay máy bơm và servo radar.
constexpr uint8_t PIN_PUMP  = 18;
constexpr uint8_t PIN_SERVO = 19;

// Flame scanning servos (mỗi servo mang 1 cảm biến lửa trái/phải, quét đối xứng
// để mở rộng vùng phát hiện; đã bỏ cảm biến lửa ở giữa)
constexpr uint8_t PIN_FLAME_SERVO_LEFT  = 2;
constexpr uint8_t PIN_FLAME_SERVO_RIGHT = 15;

// Cấu hình quét của hai servo cảm biến lửa.
constexpr uint8_t FLAME_SERVO_MIN_DEG    = 0;
constexpr uint8_t FLAME_SERVO_MAX_DEG    = 180;
constexpr uint8_t FLAME_SERVO_CENTER_DEG = 90;
constexpr uint8_t FLAME_SERVO_STEP_DEG   = 5;
constexpr uint32_t FLAME_SERVO_INTERVAL_MS = 100;
// Góc ngắm theo hướng; cần hiệu chỉnh FRONT/REAR theo giá đỡ thực tế.
constexpr uint8_t FLAME_AIM_LEFT_DEG = 0;
constexpr uint8_t FLAME_AIM_RIGHT_DEG = 180;
constexpr uint8_t FLAME_AIM_FRONT_DEG = 90;
constexpr uint8_t FLAME_AIM_REAR_DEG = 0;

// Servo vị trí SG90 mang cảm biến radar SRF04.
constexpr uint8_t SERVO_SCAN_MIN_DEG = 0;
constexpr uint8_t SERVO_SCAN_MAX_DEG = 180;
constexpr uint8_t SERVO_SCAN_CENTER_DEG = 90;
constexpr uint8_t SERVO_SCAN_STEP_DEG = 5;
// Bước SG90 5 độ chỉ cần thời gian ổn định ngắn; lần đầu đi từ 90 về 0 độ sau
// khởi động/chế độ Manual cần khoảng chờ riêng dài hơn.
constexpr uint32_t SERVO_SCAN_SETTLE_MS = 35;
constexpr uint32_t SERVO_SCAN_START_SETTLE_MS = 200;
constexpr uint32_t SERVO_SCAN_END_PAUSE_MS = 80;
static_assert((SERVO_SCAN_MAX_DEG - SERVO_SCAN_MIN_DEG) % SERVO_SCAN_STEP_DEG == 0,
			  "Servo scan range must be divisible by the angle step");

constexpr bool PUMP_ACTIVE_HIGH   = true;

// Giới hạn hợp lệ của ADC và tín hiệu cảm biến lửa.
constexpr uint16_t FLAME_ADC_MIN = 0;
constexpr uint16_t FLAME_ADC_MAX = 4095;
constexpr uint16_t FLAME_SIGNAL_MIN = 0;
// Phân loại hướng chấp nhận toàn bộ dải tín hiệu tương tự 12 bit.
constexpr uint16_t FLAME_SIGNAL_MAX = 4095;
constexpr uint16_t FLAME_AMBIENT_MIN = 100;
constexpr uint16_t FLAME_AMBIENT_MAX = 3500;

// Giới hạn an toàn của máy bơm.
constexpr uint32_t PUMP_MAX_SPRAY_TIME_MS = 10000;  // 10s tối đa
constexpr uint32_t PUMP_COOLDOWN_MS = 500;          // Nghỉ 500 ms giữa hai chu kỳ.
constexpr uint8_t PUMP_MAX_CYCLES_PER_SESSION = 3;  // Tối đa 3 chu kỳ
// Phun ngay khi bộ lọc xác nhận lửa. Đặt giá trị sai nếu muốn xe tiếp cận đến khi
// đạt FLAME_NEAR_DELTA rồi mới phun.
constexpr bool PUMP_ON_CONFIRMED_FIRE = true;

// Mức ưu tiên của các tác vụ FreeRTOS.
#define TASK_PRIORITY_SENSOR 4
#define TASK_PRIORITY_FIRE_SAFETY 5
// PWM L298N: 800 Hz giúp mạch cầu H lưỡng cực đời cũ có mô-men khởi động tốt ở
// duty vừa phải và vẫn nằm trong khả năng LEDC. Lệnh duty dùng dải 8 bit 0..255.
constexpr uint32_t MOTOR_PWM_FREQ = 800;
constexpr uint8_t MOTOR_PWM_BITS = 8;
static_assert(MOTOR_PWM_FREQ >= 100 && MOTOR_PWM_FREQ <= 20000,
              "Motor PWM frequency is outside the supported tuning range");
static_assert(MOTOR_PWM_BITS == 8,
              "Motor speed constants assume 8-bit PWM (0..255)");
// Dùng kênh LEDC số cao để tránh kênh thấp thường được ESP32Servo cấp phát.
constexpr uint8_t MOTOR_PWM_CHANNEL_LEFT = 6;
constexpr uint8_t MOTOR_PWM_CHANNEL_RIGHT = 7;
// Giới hạn cứng sau cân chỉnh từng bên: 80% = 204/255.
constexpr uint8_t MOTOR_MAX_PWM_PERCENT = 80;
constexpr uint8_t MOTOR_PWM_LIMIT =
  (uint16_t)255 * MOTOR_MAX_PWM_PERCENT / 100;
static_assert(MOTOR_MAX_PWM_PERCENT <= 100,
              "Motor PWM limit must be at most 100 percent");

// Tốc độ motor (PWM 0..255), gom một chỗ để dễ hiệu chỉnh.
constexpr uint8_t MOTOR_SPEED_MANUAL = 160;
constexpr uint8_t MOTOR_SPEED_SEARCH = 200;
constexpr uint8_t MOTOR_SPEED_AUTO   = 200;
// Quay tại chỗ chậm hơn giúp giảm vượt góc khi MPU6050 đóng vòng điều khiển.
constexpr uint8_t MOTOR_SPEED_TURN   = 200;
constexpr uint8_t MOTOR_SPEED_STEER  = 200;
constexpr uint8_t MOTOR_SPEED_FIRE   = 200;
static_assert(MOTOR_SPEED_MANUAL <= MOTOR_PWM_LIMIT &&
              MOTOR_SPEED_SEARCH <= MOTOR_PWM_LIMIT &&
              MOTOR_SPEED_AUTO <= MOTOR_PWM_LIMIT &&
              MOTOR_SPEED_TURN <= MOTOR_PWM_LIMIT &&
              MOTOR_SPEED_STEER <= MOTOR_PWM_LIMIT &&
              MOTOR_SPEED_FIRE <= MOTOR_PWM_LIMIT,
              "A configured motor speed exceeds the 80 percent safety cap");

// Hai bên bắt đầu không khuếch đại. Hệ số 180% cũ khiến mọi lệnh đều chạm giới
// hạn 80%, không còn khoảng điều chỉnh để cân hai bên khi chạy thẳng.
constexpr uint8_t MOTOR_TRIM_LEFT_PCT  = 100;
constexpr uint8_t MOTOR_TRIM_RIGHT_PCT = 100;
constexpr uint8_t MOTOR_MIN_START_PWM  = 0;
constexpr uint8_t ARC_INNER_PCT        = 35;

// Ánh xạ tên tốc độ mà các module đang dùng sang nhóm cấu hình phía trên.
constexpr uint8_t PWM_EXPLORE = MOTOR_SPEED_SEARCH;
constexpr uint8_t PWM_PATROL  = MOTOR_SPEED_AUTO;
constexpr uint8_t PWM_TURN    = MOTOR_SPEED_TURN;
constexpr uint8_t PWM_FIRE    = MOTOR_SPEED_FIRE;

/*
 * Xác nhận lửa trên ADC 12 bit của ESP32 (0..4095).
 * Phần lớn module lửa tương tự cho giá trị THẤP hơn khi hồng ngoại mạnh lên.
 * Thuật toán dùng độ giảm so với nền đã học kết hợp dao động giống ngọn lửa,
 * nhờ đó loại ánh nắng hoặc đèn chiếu ổn định đáng tin cậy hơn.
 */
constexpr bool FLAME_SIGNAL_ACTIVE_LOW = true;
constexpr uint16_t FLAME_DETECT_DELTA = 450;
constexpr uint16_t FLAME_NEAR_DELTA = 1100;
// Cả biên độ tuyệt đối và thành phần dao động tương đối phải giống nhấp nháy của lửa.
constexpr uint16_t FLAME_FLICKER_DELTA = 80;
constexpr uint8_t FLAME_FLICKER_MIN_PERCENT = 5;
constexpr uint16_t FLAME_FLICKER_STEP_DELTA = 20;
constexpr uint8_t FLAME_FLICKER_MIN_REVERSALS = 3;
constexpr uint8_t FLAME_FLICKER_MIN_VARIATION_MULTIPLIER = 2;
// Ánh nắng trực tiếp có thể làm phototransistor cận hồng ngoại bão hòa sát biên ADC.
constexpr uint16_t FLAME_ADC_SATURATION_MARGIN = 40;
constexpr uint8_t FLAME_MAX_SATURATED_PERCENT = 50;
// Khi thấy thay đổi IR mạnh, dừng hai servo tại góc đó rồi xác minh dao động
// theo thời gian để chuyển động servo không làm nhiễu mẫu.
constexpr uint32_t FLAME_VERIFY_SETTLE_MS = 100;
constexpr uint32_t FLAME_VERIFY_TIMEOUT_MS = 1800;
constexpr uint32_t FLAME_VERIFY_REJECT_COOLDOWN_MS = 600;
constexpr uint8_t FLAME_VERIFY_MIN_SAMPLES = 12;
constexpr uint8_t FLAME_LOST_SAMPLES = 8;
// Khi thân xe và servo căn đồng bộ, góc nhìn cảm biến chủ động thay đổi nên tín
// hiệu tương tự có thể mất ngắn dù vẫn là cùng nguồn lửa. Giữ trạng thái xác
// nhận đủ lâu cho vòng quay 180 độ; sau khi căn xong mới dùng bộ lọc mất lửa ngắn.
constexpr uint8_t FLAME_ALIGNMENT_LOST_SAMPLES = 80;
constexpr uint16_t FIRE_DIRECTION_MARGIN = 80;
constexpr bool SENSOR_DEBUG = false;
constexpr uint32_t SENSOR_DEBUG_PERIOD_MS = 1000;
constexpr bool SA2_DEBUG = true;
constexpr uint32_t SA2_DEBUG_PERIOD_MS = 250;

// Kích thước và độ phân giải bản đồ ô lưới.
constexpr uint8_t MAP_W = 40;
constexpr uint8_t MAP_H = 40;
constexpr uint8_t MAP_CELL_CM = 5;
constexpr uint8_t START_X = MAP_W / 2;
constexpr uint8_t START_Y = MAP_H / 2;
// Cần hai quan sát siêu âm trùng nhau trước khi thay đổi trạng thái một ô.
constexpr uint8_t DYNAMIC_OBSTACLE_CONFIRMATIONS = 2;
constexpr uint8_t DYNAMIC_FREE_CONFIRMATIONS = 2;
// Nở tiếng vọng đã xác nhận thêm 10 cm để mặt vật lớn trở thành vùng cấm có
// diện tích, thay vì chỉ là một điểm dày một ô trên bản đồ.
constexpr uint8_t MAP_OBSTACLE_INFLATION_CELLS = 2;
constexpr uint32_t COVERAGE_RESCAN_MS = 60000;

// Ngưỡng dùng cho lập bản đồ và tránh vật cản thời gian thực.
constexpr uint8_t TARGET_WALL_GAP_CM = 20;
constexpr uint8_t PHYSICAL_STOP_CM   = 15;
constexpr uint8_t OBSTACLE_MAP_CM    = 10;
// SA2 kích hoạt tránh vật cản gần; phép so sánh có lấy bằng nên đúng 15 cm cũng dừng.
constexpr uint8_t FORWARD_CLEARANCE_CM = PHYSICAL_STOP_CM;
constexpr uint8_t SA2_OBSTACLE_DISTANCE_CM = PHYSICAL_STOP_CM;
// Giá trị sai: không có tiếng vọng nghĩa là chưa phát hiện, không khóa xe. Số đo SA2 hợp lệ
// nhỏ hơn hoặc bằng ngưỡng vẫn dừng ngay.
constexpr bool SA2_STOP_ON_INVALID_READING = false;
constexpr uint8_t RADAR_FORWARD_CONE_DEG = 20;
constexpr uint8_t RADAR_EMERGENCY_STOP_CM = PHYSICAL_STOP_CM;
constexpr uint32_t RADAR_FORWARD_FRESH_MS = 1500;
// Không ghi số quét lúc xe đang chạy vào bản đồ lâu dài; khi thiếu encoder và
// pose tin cậy, tia đo trong lúc thân xe chuyển động có thể bị đặt sai ô.
constexpr bool RADAR_MAP_ONLY_WHEN_STOPPED = true;
// Hết thời gian/không echo thường nghĩa là không thấy mặt phản xạ trong tầm.
// Không coi toàn tia là trống; chỉ mở ô 5 cm ngay cạnh để xe có thể bắt đầu ở
// khu vực rộng. Đặt giá trị sai nếu muốn thiếu tiếng vọng phải làm xe dừng.
constexpr bool RADAR_NO_ECHO_CLEARS_ADJACENT_CELL = true;
constexpr uint16_t RADAR_NO_ECHO_CLEAR_CM = OBSTACLE_MAP_CM + MAP_CELL_CM;
// Lùi trước một đoạn theo các ô đã xác nhận, đánh giá đường khác, rồi lùi thêm
// từng ô đến giới hạn khi chưa tìm được hướng thoát an toàn.
constexpr uint8_t AVOID_BACKTRACK_MIN_CELLS = 3;
constexpr uint8_t AVOID_BACKTRACK_MAX_CELLS = 5;
constexpr uint8_t AVOID_ROUTE_LOOKAHEAD_CELLS = 10;
static_assert(AVOID_BACKTRACK_MIN_CELLS <= AVOID_BACKTRACK_MAX_CELLS,
              "Minimum avoidance backtrack must not exceed maximum");
constexpr uint32_t SA2_AVOID_BACK_MS = 580;
constexpr uint32_t SA2_AVOID_TURN_90_MS = 575;
constexpr uint32_t SA2_AVOID_FORWARD_MS = 1300;
constexpr uint8_t SA2_OBSTACLE_CONFIRM_SAMPLES = 3;
// Số đo hợp lệ 1 cm được coi là kẹt khẩn cấp; 0 vẫn là timeout/không echo,
// tuyệt đối không diễn giải là khoảng cách vật lý bằng 0.
constexpr uint8_t SA2_STUCK_DISTANCE_CM = 1;
constexpr uint8_t SA2_CRITICAL_STOP_CM = SA2_STUCK_DISTANCE_CM;
constexpr uint32_t ULTRASONIC_SETTLE_AFTER_TURN_MS = 100;

// Khoảng thời gian ước lượng đi hết một ô 5 cm ở AUTO_CRUISE; motor không dừng
// tại ranh giới ô. Phải đo thời gian xe thật đi 5 cm ở MOTOR_SPEED_AUTO để hiệu
// chỉnh, vì giá trị sai sẽ làm pose chạy nhanh/chậm hơn vị trí thực tế.
constexpr uint32_t AUTO_POSE_CELL_INTERVAL_MS = 400;
// Thời gian dự phòng cho PWM quay hiện tại; MPU6050 vẫn là điều kiện dừng chính
// mỗi khi dữ liệu IMU hợp lệ và còn mới.
constexpr uint32_t TURN_90_MS = 575;
constexpr uint32_t TURN_SETTLE_MS = 130;
constexpr uint32_t SCAN_BODY_TURN_MS = 2 * TURN_90_MS;
constexpr uint8_t SCAN_CYCLES_PER_POSE = 2;
static_assert(SCAN_CYCLES_PER_POSE == 2,
              "Simulation requires exactly front and rear radar sweeps");
constexpr uint8_t MAX_TURNS_WITHOUT_FORWARD = 2;

// Giới hạn khảo sát tự động trong lần chạy đầu tiên.
constexpr uint16_t MAX_EXPLORE_STEPS = 180;
constexpr uint16_t MAX_NO_NEW_CELL = 45;

// Chu kỳ cảm biến, lưu bản đồ và giám sát tác vụ RTOS.
constexpr uint32_t ULTRASONIC_PERIOD_MS = 40;
constexpr uint32_t MAP_SAVE_PERIOD_MS = 5000;
constexpr uint32_t SENSOR_HEARTBEAT_TIMEOUT_MS = 700;
// Giữ xe đứng yên khi cảm biến lửa đang học mức hồng ngoại môi trường.
constexpr uint32_t FIRE_ARM_DELAY_MS = 2500;
constexpr uint8_t FIRE_CONFIRM_SAMPLES = 8;

// Radar quét trong giới hạn bằng servo vị trí trên đỉnh. Hai mẫu tại mỗi góc
// cộng xác nhận chéo giữa hai lượt vẫn lọc nhiễu nhưng giảm thời gian mỗi bước.
constexpr uint8_t SCAN_SAMPLES_PER_ANGLE = 2;
// Echo cuối được cả hai mẫu tại cùng góc nhìn thấy sẽ trở thành vật cản xác nhận
// khi toàn vòng 0 -> 180 -> 0 được công bố lên bản đồ.
constexpr uint8_t RADAR_OBSTACLE_MIN_VALID_SAMPLES = 2;
static_assert(RADAR_OBSTACLE_MIN_VALID_SAMPLES <= SCAN_SAMPLES_PER_ANGLE,
              "Radar obstacle confirmation cannot exceed samples per angle");
constexpr uint16_t MAX_RANGE_CM = 250;

// Cấu hình EEPROM giả lập trên flash.
constexpr uint16_t EEPROM_MAGIC = 0xA65A;
// EEPROM của ESP32 là giả lập trên flash nên phải khởi tạo đúng dung lượng.
constexpr uint16_t EEPROM_SIZE = MAP_W * MAP_H + 9;
// Tăng phiên bản khi định dạng/ý nghĩa bản đồ đổi để bản đồ cũ không giữ nhầm
// hành lang khởi động ở trạng thái bị chặn vĩnh viễn.
constexpr uint8_t EEPROM_VERSION = 10;
