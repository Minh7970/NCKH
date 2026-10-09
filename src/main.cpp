#include <Arduino.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

#include "RobotConfig.h"
#include "MotorControl.h"
#include "SensorManager.h"
#include "MapMemory.h"
#include "Navigation.h"
#include "FireControl.h"
#include "WifiMapServer.h"
#include "WebControl.h"
#include "FlameServoScanner.h"
#include "PowerManager.h"

/*
 * Điểm khởi động của toàn bộ hệ thống robot chữa cháy.
 * Chương trình chia công việc thành các tác vụ FreeRTOS độc lập: đọc cảm biến,
 * điều hướng, giám sát an toàn, lưu bản đồ, phục vụ giao diện web và quản lý
 * tiết kiệm năng lượng. Cách chia này giúp cảm biến và cơ chế dừng khẩn cấp
 * vẫn được cập nhật đều ngay cả khi thuật toán bản đồ hoặc Wi-Fi đang bận.
 */

TaskHandle_t sensorTaskHandle=nullptr;
TaskHandle_t navTaskHandle=nullptr;
TaskHandle_t safetyTaskHandle=nullptr;
TaskHandle_t mapTaskHandle=nullptr;
TaskHandle_t wifiTaskHandle=nullptr;

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWER_ON";
    case ESP_RST_EXT:       return "EXTERNAL_PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INTERRUPT_WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
    case ESP_RST_WDT:       return "OTHER_WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_WAKE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

void TaskSensors(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    SensorManager::update();
    // Quét servo lửa cùng nhịp với đọc cảm biến; hàm tự giới hạn theo
    // FLAME_SERVO_INTERVAL_MS nên gọi thường xuyên hơn không gây hại.
    // Giữ hai servo cảm biến lửa đứng yên khi hệ thống đang ở chế độ tiết kiệm điện.
    if (!PowerManager::sleeping()) {
      FlameServoScanner::update();
      if (FireControl::active()) {
        if (FireControl::flameAlignmentComplete()) {
          // Nếu nguồn lửa đã nằm trong cửa sổ 0..5 độ phía trước thì giữ nguyên
          // góc phát hiện. Sau khi thân xe đã xoay xong, đưa cả hai cảm biến về
          // hướng 0 độ trùng với ống nước cố định ở mũi xe.
          const uint8_t detectedAngle = FireControl::flameScanAngle();
          FlameServoScanner::aimScanAngle(
            detectedAngle <= FLAME_ALIGNMENT_TOLERANCE_DEG
              ? detectedAngle : PUMP_FIXED_HEADING_DEG);
        } else {
          // Giữ cảm biến đã phát hiện lửa tại góc bắt được; cảm biến còn lại
          // quay về hướng 0 độ ở mũi xe để không làm thay đổi hướng đã khóa.
          FlameServoScanner::aimDetectedSensor(
            (uint8_t)FireControl::direction(), FireControl::flameScanAngle());
        }
      } else if (!(WebControl::mode() == WebControl::MANUAL_MODE &&
                   FlameServoScanner::manualAiming())) {
        FlameServoScanner::clearAim();
      }
    }
    vTaskDelayUntil(&last,pdMS_TO_TICKS(30));
  }
}

void TaskNavigation(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    Navigation::update();
    vTaskDelayUntil(&last,pdMS_TO_TICKS(20));
  }
}

void TaskPower(void* parameter) {
  PowerManager::taskLoop(parameter);
}

void TaskSafety(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    SensorSnapshot s=SensorManager::get();
    uint32_t now=millis();

    // Tác vụ an toàn có ưu tiên cao nhất: nếu tác vụ cảm biến treo hoặc trễ,
    // dừng toàn bộ cơ cấu chấp hành để xe không chạy với dữ liệu đã cũ.
    if(now-s.heartbeatMs > SENSOR_HEARTBEAT_TIMEOUT_MS) {
      MotorControl::stop();
      MotorControl::setPump(false);
    }

    vTaskDelayUntil(&last,pdMS_TO_TICKS(20));
  }
}

void TaskMap(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    static uint32_t lastSave=0;
    // Trong lần khảo sát đầu tiên không lưu một bản đồ còn dang dở. Nếu mất
    // nguồn, xe sẽ quét lại thay vì hiểu nhầm bản đồ thiếu là bản đồ tuần tra.
    if(Navigation::mode() == Navigation::PATROL &&
       MapMemory::dirty() && millis()-lastSave>=MAP_SAVE_PERIOD_MS) {
      MapMemory::save();
      lastSave=millis();
      Serial.println(F("[EEPROM] map saved"));
      MapMemory::print(Navigation::pose());
    }
    vTaskDelayUntil(&last,pdMS_TO_TICKS(500));
  }
}

void TaskWifiServer(void*) {
  for (;;) {
    WifiMapServer::update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("\n[BOOT] reset_reason=%s (%d)\n",
                resetReasonName(resetReason), (int)resetReason);
  MotorControl::begin();
  SensorManager::begin();
  FireControl::begin();
  WebControl::begin();
  FlameServoScanner::begin();

  MapMemory::begin();
  WifiMapServer::begin();
  PowerManager::begin();


  Serial.println(F("\n=== FIRE ROBOT / MODULAR MAPPING ==="));
  Serial.println(F("ESP32 DevKitC V4 + FreeRTOS"));
  Serial.print(F("Map: ")); Serial.print(MAP_W); Serial.print('x');
  Serial.print(MAP_H); Serial.print(F(", cell=")); Serial.print(MAP_CELL_CM);
  Serial.println(F(" cm"));
  // Serial.print(F("Khoang cach tuong muc tieu khi lap ban do="));
  // Serial.print(TARGET_WALL_GAP_CM);
  // Serial.println(F(" cm (HC-SR04 khong bao dam chinh xac vat ly o moi dieu kien)"));
  // Serial.print(F("Khoang cach dung vat ly=")); Serial.print(PHYSICAL_STOP_CM);
  // Serial.println(F(" cm"));
  // Serial.println(F("Servo quet SRF04: gioi han hanh trinh de bao ve day dan"));
  // Serial.print(F("Chan SA2 SRF05: TRIG=")); Serial.print(PIN_SA2_TRIG);
  // Serial.print(F(", ECHO=")); Serial.println(PIN_SA2_ECHO);

  Navigation::begin();

  if(MapMemory::isLoaded()) {
    Serial.println(F("[BOOT] EEPROM MAP FOUND -> PATROL"));
  } else {
    Serial.println(F("[BOOT] NO MAP -> AUTONOMOUS FIRST-RUN MAPPING"));
    Serial.println(F("[BOOT] Robot will drive, learn the room, then start patrol."));
  }

  xTaskCreate(TaskSafety,"Safety",4096,nullptr,3,&safetyTaskHandle);
  xTaskCreate(TaskSensors,"Sensor",4096,nullptr,2,&sensorTaskHandle);
  xTaskCreate(TaskNavigation,"Nav",4096,nullptr,1,&navTaskHandle);
  xTaskCreate(TaskMap,"Map",4096,nullptr,0,&mapTaskHandle);
  xTaskCreate(TaskWifiServer,"WifiMap",6144,nullptr,1,&wifiTaskHandle);
  xTaskCreate(TaskPower,"Power",3072,nullptr,1,nullptr);
}

void loop() {
  // Arduino trên ESP32 đã chạy FreeRTOS; toàn bộ công việc nằm trong các tác vụ.
  vTaskDelay(portMAX_DELAY);
}
