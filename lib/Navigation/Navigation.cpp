#include "Navigation.h"
#include "RobotConfig.h"
#include "SensorManager.h"
#include "ServoScanner.h"
#include "FlameServoScanner.h"
#include "ScanMapper.h"
#include "WebControl.h"
#include "MotorControl.h"
#include "FireControl.h"
#include "MapMemory.h"
#include "PowerManager.h"

#include <Arduino.h>
#include <string.h>

/*
 * Thuật toán điều hướng hoạt động theo máy trạng thái không chặn:
 * 1. Xe đứng yên quét radar phía trước, quay 180 độ rồi quét phía sau.
 * 2. Bản đồ được chốt; radar tắt và SA2 tiếp tục giám sát vật cản động.
 * 3. Bộ phủ ưu tiên hành lang liền kề có nhiều ô FREE chưa đi nhất. Khi bị
 *    tách khỏi vùng mới, BFS được phép đi qua ô đã thăm để tới ô chưa thăm xa.
 * 4. Mỗi ranh giới ô cập nhật pose và lịch sử. Khi gặp vật cản, xe lùi đúng
 *    các cạnh đã đi, ghi vật cản vào bản đồ rồi lập kế hoạch lại.
 * 5. Xử lý lửa có ưu tiên cao hơn tuần tra; hoàn tất sẽ trở lại bản đồ cũ.
 * 6. Khi mọi ô FREE có thể tới đã được đánh dấu, xe chuyển sang sleep mode.
 */

namespace {

constexpr uint8_t NO_HEADING = 0xFF;

Pose p = {START_X, START_Y, NORTH};
Navigation::Mode current = Navigation::EXPLORE;

// --------------------------------------------------------------------------
// Các trạng thái chuyển động; khi IMU không hợp lệ sẽ dùng thời gian dự phòng.
// --------------------------------------------------------------------------
enum Motion {
  IDLE,
  AUTO_CRUISE,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_BACK,
  BACK_CELL,      // Lùi một ô khi phát hiện xe bị kẹt.
  AVOID_FORWARD   // Tiến một ô theo hướng thoát khỏi vật cản.
};

enum AvoidPhase {
  AVOID_NONE,
  AVOID_ALIGNING_BACKTRACK,
  AVOID_BACKING,
  AVOID_TURNING,
  AVOID_ESCAPING
};

Motion motion = IDLE;
AvoidPhase avoidPhase = AVOID_NONE;
uint32_t motionStart = 0;
uint32_t lastMotionFinish = 0;
float turnStartYaw = 0.0f;
bool turnImuValid = false;
float forwardTargetYaw = 0.0f;
bool forwardImuValid = false;

// Lịch sử đường đi theo từng ô chỉ lưu trong RAM. Sau khi reset không biết vị
// trí vật lý thật nên phát lại lịch sử đã lưu sẽ nguy hiểm. Trong cùng một lần
// chạy, lịch sử giúp xe lùi qua đúng các ô đã đi thay vì ước lượng mù.
constexpr uint16_t PATH_HISTORY_CAPACITY = 256;
Pose pathHistory[PATH_HISTORY_CAPACITY];
uint16_t pathHistoryCount = 0;
uint8_t avoidBacktrackRemaining = 0;
uint8_t avoidBacktrackReserve = 0;
uint8_t avoidBacktrackedCells = 0;
bool backingAlongHistory = false;
bool blindBacktrackMode = false;
// Số ô logic còn lại trong hành lang thẳng hiện tại. Motor chạy liên tục nhưng
// pose và lịch sử vẫn cập nhật tại từng ranh giới ô.
uint16_t cruiseCellsRemaining = 0;

// --------------------------------------------------------------------------
// Trạng thái thuật toán phủ khu vực: ưu tiên đường thẳng và dùng BFS khi cần.
// --------------------------------------------------------------------------
bool coverageVisited[MAP_H][MAP_W];
bool searchSeen[MAP_H][MAP_W];
uint8_t searchFirstHeading[MAP_H][MAP_W];
uint16_t searchDistance[MAP_H][MAP_W];
uint16_t searchQueue[MAP_W * MAP_H];

Heading sweepHeading = EAST;
Heading laneHeading = NORTH;
uint8_t pendingHeading = NO_HEADING;
// Cờ này chỉ bật khi buộc phải băng qua ô trống đã thăm để tới ô chưa thăm còn lại.
bool plannedVisitedTransit = false;
int16_t coverageTargetX = -1;
int16_t coverageTargetY = -1;

bool coverageComplete = false;

// --------------------------------------------------------------------------
// Trạng thái radar: chỉ lập bản đồ khi thân xe đứng yên; SA2 xử lý vật cản mới
// xuất hiện trong lúc xe tiến.
// --------------------------------------------------------------------------
bool backgroundScanActive = false;
uint8_t scanCycle = 0;
Heading scanStartHeading = NORTH;
uint32_t lastScanTime = 0;
bool initialMapReady = false;
bool radarPatrolDisabled = false;
bool initialScanTurnActive = false;
uint32_t initialScanTurnStartMs = 0;
float initialScanTurnStartYaw = 0.0f;
bool initialScanTurnImuValid = false;
uint16_t forwardRadarDistanceCm = 0;
uint32_t forwardRadarMs = 0;
bool forwardRadarValid = false;
uint32_t lastSafetyLogMs = 0;

// --------------------------------------------------------------------------
// Bản đồ cục bộ 3x3 dùng cho tránh vật cản SA2.
// --------------------------------------------------------------------------
enum LocalCellState {
  LC_UNKNOWN = 0,
  LC_FREE = 1,
  LC_BLOCKED = 2
};

LocalCellState localMap[3][3];  // [hàng][cột], robot nằm tại ô giữa [1][1].
uint32_t localMapLastUpdate = 0;
constexpr uint32_t LOCAL_MAP_FORGET_MS = 6000;  // Quên dữ liệu cục bộ sau 6 giây.

// --------------------------------------------------------------------------
// Phát hiện kẹt khi khoảng cách SA2 gần như không thay đổi.
// --------------------------------------------------------------------------
uint16_t sa2History[5];  // Năm số đo SA2 gần nhất.
uint8_t sa2HistoryIndex = 0;
uint32_t stuckStartTime = 0;
constexpr uint16_t STUCK_TOLERANCE_CM = 5;  // Sai khác tối đa 5 cm vẫn coi là đứng yên.
constexpr uint8_t STUCK_READINGS = 5;       // Số mẫu dùng để đánh giá.
constexpr uint32_t STUCK_TIMEOUT_MS = 3000; // Kẹt 3 giây thì bắt đầu lùi.

bool controlWasManual = false;
uint8_t consecutiveTurnsWithoutForward = 0;
uint8_t sa2CloseSampleCount = 0;
uint32_t lastSa2DecisionSampleMs = 0;
struct ObstaclePoint { int16_t x; int16_t y; };
ObstaclePoint sa2Obstacles[4];
uint8_t sa2ObstacleCount = 0;
bool rectangularBoundaryClosed = false;
int16_t patrolMinX = 0;
int16_t patrolMaxX = MAP_W - 1;
int16_t patrolMinY = 0;
int16_t patrolMaxY = MAP_H - 1;
bool navigationWasSleeping = false;
float fireEntryYaw = 0.0f;
bool fireEntryImuValid = false;
bool fireReturnActive = false;
uint32_t fireReturnStartMs = 0;

// Khai báo trước các hàm mà bộ chọn hướng tránh cục bộ sử dụng.
bool inside(int16_t x, int16_t y);
void advance(int16_t& x, int16_t& y, Heading h);
bool coverageCellIsFree(int16_t x, int16_t y);

bool imuReadingFresh(const SensorSnapshot& s) {
  return MPU6050_ENABLED && s.imuValid && s.imuMs > 0 &&
    millis() - s.imuMs <= MPU6050_FRESH_MS;
}

float directedTurnProgressDeg(Motion turn, float startYaw,
                              const SensorSnapshot& s) {
  // Với MPU6050_YAW_SIGN=1, quay trái làm yaw dương và quay phải làm yaw âm.
  // Chỉ tính tiến độ đúng chiều để quay sai chiều không thể vô tình đạt đích.
  if (turn == TURN_LEFT) return s.yawDeg - startYaw;
  return startYaw - s.yawDeg;
}

void captureTurnYaw() {
  const SensorSnapshot s = SensorManager::get();
  turnStartYaw = s.yawDeg;
  turnImuValid = imuReadingFresh(s);
}

void captureForwardYaw() {
  const SensorSnapshot s = SensorManager::get();
  forwardTargetYaw = s.yawDeg;
  forwardImuValid = imuReadingFresh(s);
}

void maintainForwardHeading(const SensorSnapshot& s) {
  const uint8_t basePwm = current == Navigation::PATROL
    ? PWM_PATROL : PWM_EXPLORE;
  if (!forwardImuValid || !imuReadingFresh(s)) {
    MotorControl::forward(basePwm);
    return;
  }

  float errorDeg = s.yawDeg - forwardTargetYaw;
  if (fabsf(errorDeg) <= MPU_HEADING_HOLD_DEADBAND_DEG) errorDeg = 0.0f;
  const int correction = constrain(
    (int)lroundf(errorDeg * MPU_HEADING_HOLD_KP),
    -(int)MPU_HEADING_HOLD_MAX_PWM,
    (int)MPU_HEADING_HOLD_MAX_PWM);
  MotorControl::forwardCorrected(basePwm, correction);
}

// --------------------------------------------------------------------------
// Các hàm hỗ trợ bản đồ cục bộ.
// --------------------------------------------------------------------------
void resetLocalMap() {
  memset(localMap, LC_UNKNOWN, sizeof(localMap));
  localMapLastUpdate = millis();
  Serial.println(F("[LOCALMAP] RESET"));
}

void initLocalMap() {
  resetLocalMap();
}

void markLocalObstacle(Heading heading) {
  // Đổi hướng la bàn sang tọa độ cục bộ; robot ở [1][1], hướng quyết định ô kề.
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX >= 0 && localX < 3 && localY >= 0 && localY < 3) {
    localMap[localY][localX] = LC_BLOCKED;
    localMapLastUpdate = millis();
    
    Serial.print(F("[LOCALMAP] BLOCKED "));
    Serial.print((int)heading);
    Serial.print(F(" at ["));
    Serial.print(localX);
    Serial.print(F(","));
    Serial.print(localY);
    Serial.println(F("]"));
  }
}

void markLocalFree(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX >= 0 && localX < 3 && localY >= 0 && localY < 3) {
    localMap[localY][localX] = LC_FREE;
    localMapLastUpdate = millis();
  }
}

bool isLocalFree(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX < 0 || localX >= 3 || localY < 0 || localY >= 3) {
    return false;  // Ngoài phạm vi thì không phải ô trống hợp lệ.
  }
  
  return localMap[localY][localX] == LC_FREE;
}

bool isLocalBlocked(Heading heading) {
  int8_t dx = 0, dy = 0;
  if (heading == NORTH) dy = -1;
  else if (heading == SOUTH) dy = 1;
  else if (heading == EAST) dx = 1;
  else if (heading == WEST) dx = -1;
  
  int8_t localX = 1 + dx;
  int8_t localY = 1 + dy;
  
  if (localX < 0 || localX >= 3 || localY < 0 || localY >= 3) {
    return true;  // Ngoài phạm vi được coi là bị chặn.
  }
  
  return localMap[localY][localX] == LC_BLOCKED;
}

int16_t scoreAvoidDirection(Heading heading) {
  if (isLocalBlocked(heading)) return -1;

  int16_t x = p.x;
  int16_t y = p.y;
  int16_t score = 0;
  uint8_t openCells = 0;

  for (uint8_t step = 0; step < AVOID_ROUTE_LOOKAHEAD_CELLS; ++step) {
    advance(x, y, heading);
    if (!coverageCellIsFree(x, y)) break;

    ++openCells;
    score += 3;
    if (!coverageVisited[y][x]) score += 3;
  }

  if (openCells == 0) return -1;

  // Nếu độ thoáng tương đương, ưu tiên đường bên mới thay vì tiếp tục lùi trên
  // tuyến đã phủ để tăng khả năng khám phá vùng chưa đi.
  if (heading == (Heading)((p.heading + 2) % 4)) {
    score = score > 4 ? score - 4 : 0;
  }
  return score;
}

uint8_t chooseAvoidDirection() {
  const Heading candidates[3] = {
    (Heading)((p.heading + 3) % 4),
    (Heading)((p.heading + 1) % 4),
    (Heading)((p.heading + 2) % 4)
  };

  int16_t bestScore = -1;
  uint8_t bestHeading = NO_HEADING;
  for (uint8_t i = 0; i < 3; ++i) {
    const int16_t score = scoreAvoidDirection(candidates[i]);
    Serial.print(F("[NAV] ESCAPE EVAL heading="));
    Serial.print((int)candidates[i]);
    Serial.print(F(" score="));
    Serial.println(score);

    if (score > bestScore) {
      bestScore = score;
      bestHeading = (uint8_t)candidates[i];
    }
  }
  return bestHeading;
}

// --------------------------------------------------------------------------
// Phát hiện xe bị kẹt.
// --------------------------------------------------------------------------
void updateSA2History(uint16_t distance) {
  sa2History[sa2HistoryIndex] = distance;
  sa2HistoryIndex = (sa2HistoryIndex + 1) % STUCK_READINGS;
}

bool isStuck() {
  if (sa2History[0] == 0) return false;  // Chưa đủ dữ liệu để kết luận.
  
  uint16_t minDist = sa2History[0];
  uint16_t maxDist = sa2History[0];
  
  for (uint8_t i = 1; i < STUCK_READINGS; ++i) {
    if (sa2History[i] < minDist) minDist = sa2History[i];
    if (sa2History[i] > maxDist) maxDist = sa2History[i];
  }
  
  return (maxDist - minDist) < STUCK_TOLERANCE_CM;
}

// --------------------------------------------------------------------------
// Các hàm tiện ích điều hướng và quản lý lịch sử đường đi.
// --------------------------------------------------------------------------
bool inside(int16_t x, int16_t y) {
  return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
}

Heading backOf(Heading h) {
  return (Heading)((h + 2) % 4);
}

void advance(int16_t& x, int16_t& y, Heading h) {
  if (h == NORTH) --y;
  else if (h == SOUTH) ++y;
  else if (h == EAST) ++x;
  else --x;
}

bool coverageCellIsFree(int16_t x, int16_t y) {
  if (!inside(x, y) || MapMemory::get(x, y) != CELL_FREE) return false;
  if (!rectangularBoundaryClosed) return true;
  return x >= patrolMinX && x <= patrolMaxX &&
         y >= patrolMinY && y <= patrolMaxY;
}

void registerSa2Obstacle(uint16_t distanceCm) {
  int16_t obstacleX = p.x;
  int16_t obstacleY = p.y;
  uint16_t cellsAheadValue =
    (distanceCm + MAP_CELL_CM - 1) / MAP_CELL_CM;
  if (cellsAheadValue == 0) cellsAheadValue = 1;
  const uint8_t cellsAhead = (uint8_t)cellsAheadValue;
  for (uint8_t i = 0; i < cellsAhead; ++i)
    advance(obstacleX, obstacleY, p.heading);
  if (!inside(obstacleX, obstacleY)) return;

  for (uint8_t i = 0; i < sa2ObstacleCount; ++i) {
    if (sa2Obstacles[i].x == obstacleX && sa2Obstacles[i].y == obstacleY)
      return;
  }

  if (sa2ObstacleCount < 4) {
    sa2Obstacles[sa2ObstacleCount++] = {obstacleX, obstacleY};
    Serial.printf("[SA2] UNIQUE OBSTACLE %u/4 at x=%d y=%d\n",
                  (unsigned)sa2ObstacleCount, obstacleX, obstacleY);
  }

  if (sa2ObstacleCount == 4 && !rectangularBoundaryClosed) {
    patrolMinX = patrolMaxX = p.x;
    patrolMinY = patrolMaxY = p.y;
    for (uint8_t i = 0; i < 4; ++i) {
      patrolMinX = min(patrolMinX, sa2Obstacles[i].x);
      patrolMaxX = max(patrolMaxX, sa2Obstacles[i].x);
      patrolMinY = min(patrolMinY, sa2Obstacles[i].y);
      patrolMaxY = max(patrolMaxY, sa2Obstacles[i].y);
    }
    rectangularBoundaryClosed = true;
    coverageTargetX = -1;
    coverageTargetY = -1;
    Serial.printf("[SA2] 4 OBJECTS -> CLOSED RECTANGLE x=%d..%d y=%d..%d\n",
                  patrolMinX, patrolMaxX, patrolMinY, patrolMaxY);
  }
}

bool sameCell(const Pose& a, const Pose& b) {
  return a.x == b.x && a.y == b.y;
}

void resetTravelPath() {
  pathHistoryCount = 1;
  pathHistory[0] = p;
}

void recordTravelPose() {
  if (pathHistoryCount > 0 && sameCell(pathHistory[pathHistoryCount - 1], p))
    return;

  if (pathHistoryCount >= PATH_HISTORY_CAPACITY) {
    memmove(&pathHistory[0], &pathHistory[1],
            sizeof(pathHistory[0]) * (PATH_HISTORY_CAPACITY - 1));
    pathHistoryCount = PATH_HISTORY_CAPACITY - 1;
  }
  pathHistory[pathHistoryCount++] = p;
}

uint8_t availableBacktrackCells() {
  if (pathHistoryCount < 2) {
    Serial.print(F("[NAV] BACKTRACK FAIL: historyCount="));
    Serial.println(pathHistoryCount);
    return 0;
  }
  if (!sameCell(pathHistory[pathHistoryCount - 1], p)) {
    Serial.print(F("[NAV] BACKTRACK FAIL: pose("));
    Serial.print(p.x); Serial.print(F(",")); Serial.print(p.y);
    Serial.print(F(") != history("));
    Serial.print(pathHistory[pathHistoryCount - 1].x);
    Serial.print(F(","));
    Serial.print(pathHistory[pathHistoryCount - 1].y);
    Serial.println(F(")"));
    return 0;
  }

  // Kiểm tra mọi cạnh đã ghi để có thể lùi nhiều ô ngay cả khi đường cũ có góc rẽ.
  uint8_t available = 0;
  for (int32_t i = (int32_t)pathHistoryCount - 1;
       i > 0 && available < AVOID_BACKTRACK_MAX_CELLS; --i) {
    int16_t expectedX = pathHistory[i - 1].x;
    int16_t expectedY = pathHistory[i - 1].y;
    advance(expectedX, expectedY, pathHistory[i].heading);
    if (expectedX != pathHistory[i].x || expectedY != pathHistory[i].y) {
      Serial.print(F("[NAV] BACKTRACK EDGE BREAK at step "));
      Serial.println(available);
      break;
    }
    ++available;
  }
  return available;
}

bool restorePreviousTravelCell() {
  if (pathHistoryCount < 2) return false;

  int16_t expectedX = p.x;
  int16_t expectedY = p.y;
  advance(expectedX, expectedY, backOf(p.heading));
  const Pose& previous = pathHistory[pathHistoryCount - 2];
  if (previous.x != expectedX || previous.y != expectedY) return false;

  --pathHistoryCount;
  p.x = previous.x;
  p.y = previous.y;
  // Giữ nguyên heading của thân xe vì xe vừa chạy lùi, không quay đầu.
  return true;
}

bool cellIsUncovered(Heading h) {
  int16_t x = p.x;
  int16_t y = p.y;
  advance(x, y, h);

  return coverageCellIsFree(x, y) && !coverageVisited[y][x];
}

uint16_t freeCorridorLength(Heading h, uint16_t& unvisitedCells) {
  int16_t x = p.x;
  int16_t y = p.y;
  uint16_t length = 0;
  unvisitedCells = 0;

  for (uint16_t step = 0; step < MAP_W + MAP_H; ++step) {
    advance(x, y, h);
    if (!coverageCellIsFree(x, y)) break;

    ++length;
    if (!coverageVisited[y][x]) ++unvisitedCells;
  }

  return length;
}

uint16_t unvisitedCorridorLength(Heading h) {
  int16_t x = p.x;
  int16_t y = p.y;
  uint16_t length = 0;

  for (uint16_t step = 0; step < MAP_W + MAP_H; ++step) {
    advance(x, y, h);
    if (!coverageCellIsFree(x, y)) break;
    // Dừng trước ô đã phủ; chỉ bộ lập kế hoạch BFS mới được đi qua ô này khi
    // thật sự cần để tới vùng mới.
    if (coverageVisited[y][x]) break;
    ++length;
  }
  return length;
}

void markCurrentVisited() {
  if (!inside(p.x, p.y)) return;
  coverageVisited[p.y][p.x] = true;
  MapMemory::set(p.x, p.y, CELL_FREE);
}

void resetCoverage() {
  memset(coverageVisited, 0, sizeof(coverageVisited));
  memset(searchSeen, 0, sizeof(searchSeen));
  memset(searchFirstHeading, NO_HEADING, sizeof(searchFirstHeading));
  memset(searchDistance, 0, sizeof(searchDistance));

  pendingHeading = NO_HEADING;
  cruiseCellsRemaining = 0;
  plannedVisitedTransit = false;
  coverageTargetX = -1;
  coverageTargetY = -1;
  coverageComplete = false;
  // Bắt đầu lượt phủ theo hướng thân xe đang nhìn để tránh quay 90 độ không cần
  // thiết khi phía trước đã là đường trống.
  sweepHeading = p.heading;
  laneHeading = (Heading)((p.heading + 1) % 4);

  markCurrentVisited();
}

// --------------------------------------------------------------------------
// BFS dự phòng để tìm ô chưa thăm có thể tới.
// --------------------------------------------------------------------------
uint8_t findFarthestUncoveredHeading() {
  memset(searchSeen, 0, sizeof(searchSeen));
  memset(searchFirstHeading, NO_HEADING, sizeof(searchFirstHeading));
  memset(searchDistance, 0, sizeof(searchDistance));

  if (!inside(p.x, p.y)) return NO_HEADING;

  const uint16_t start = (uint16_t)(p.y * MAP_W + p.x);
  uint16_t head = 0;
  uint16_t tail = 0;

  searchQueue[tail++] = start;
  searchSeen[p.y][p.x] = true;

  uint8_t bestFirstHeading = NO_HEADING;
  uint16_t bestDistance = 0;
  int16_t bestX = -1;
  int16_t bestY = -1;
  const bool followLockedTarget =
    coverageCellIsFree(coverageTargetX, coverageTargetY) &&
    !coverageVisited[coverageTargetY][coverageTargetX];

  const Heading order[4] = {
    sweepHeading,
    laneHeading,
    backOf(sweepHeading),
    backOf(laneHeading)
  };

  while (head < tail) {
    const uint16_t packed = searchQueue[head++];
    const int16_t x = packed % MAP_W;
    const int16_t y = packed / MAP_W;

    for (uint8_t i = 0; i < 4; ++i) {
      const Heading h = order[i];
      int16_t nx = x;
      int16_t ny = y;
      advance(nx, ny, h);

      if (!inside(nx, ny)) continue;
      if (searchSeen[ny][nx]) continue;
      // Chỉ lập đường trên ô FREE đã được radar xác nhận. Ô UNKNOWN phải được
      // radar công bố trước; bộ lập kế hoạch không tự suy đoán để đi vào.
      if (!coverageCellIsFree(nx, ny)) continue;

      searchSeen[ny][nx] = true;

      const uint8_t firstHeading =
        (packed == start)
          ? (uint8_t)h
          : searchFirstHeading[y][x];

      searchFirstHeading[ny][nx] = firstHeading;
      searchDistance[ny][nx] = searchDistance[y][x] + 1;

      if (followLockedTarget && nx == coverageTargetX &&
          ny == coverageTargetY) {
        Serial.printf("[PLAN] CONTINUE TARGET x=%d y=%d distance=%u heading=%u\n",
                      coverageTargetX, coverageTargetY,
                      (unsigned)searchDistance[ny][nx],
                      (unsigned)firstHeading);
        return firstHeading;
      }

      if (!coverageVisited[ny][nx]) {
        // Tiếp tục BFS thay vì trả ô gần nhất. Mục tiêu có khoảng cách lớn nhất
        // tạo tuyến dài nhất trong thành phần FREE hiện có.
        if (bestFirstHeading == NO_HEADING ||
            searchDistance[ny][nx] > bestDistance) {
          bestFirstHeading = firstHeading;
          bestDistance = searchDistance[ny][nx];
          bestX = nx;
          bestY = ny;
        }
      }

      if (tail < MAP_W * MAP_H) {
        searchQueue[tail++] = (uint16_t)(ny * MAP_W + nx);
      }
    }
  }

  if (bestFirstHeading != NO_HEADING) {
    coverageTargetX = bestX;
    coverageTargetY = bestY;
    Serial.printf("[PLAN] LOCK FAR FREE x=%d y=%d distance=%u heading=%u\n",
                  bestX, bestY, (unsigned)bestDistance,
                  (unsigned)bestFirstHeading);
  } else {
    coverageTargetX = -1;
    coverageTargetY = -1;
  }
  return bestFirstHeading;
}

// --------------------------------------------------------------------------
// Quyết định hướng phủ bản đồ.
// --------------------------------------------------------------------------
uint8_t chooseCoverageHeading() {
  // Giai đoạn 1: chọn dải liền kề dài nhất chỉ gồm ô FREE chưa thăm đã được
  // radar xác nhận, tối đa hóa ô mới và tránh đi lại khi còn đường mới.
  const Heading routeOrder[4] = {
    sweepHeading, laneHeading, backOf(sweepHeading), backOf(laneHeading)
  };
  uint8_t bestHeading = NO_HEADING;
  uint16_t bestLength = 0;

  for (uint8_t i = 0; i < 4; ++i) {
    const uint16_t length = unvisitedCorridorLength(routeOrder[i]);
    Serial.printf("[PLAN] heading=%u new_run=%u\n",
                  (unsigned)routeOrder[i], (unsigned)length);
    if (length > bestLength) {
      bestHeading = (uint8_t)routeOrder[i];
      bestLength = length;
    }
  }
  if (bestHeading != NO_HEADING) {
    plannedVisitedTransit = false;
    coverageTargetX = -1;
    coverageTargetY = -1;
    Serial.printf("[PLAN] LONGEST NEW RUN heading=%u cells=%u\n",
                  (unsigned)bestHeading, (unsigned)bestLength);
    return bestHeading;
  }

  // Giai đoạn 2: không còn ô mới liền kề. BFS được đi qua ô FREE đã thăm nhưng
  // chỉ để tới ô chưa thăm xa nhất vẫn có thể tiếp cận.
  const uint8_t transitHeading = findFarthestUncoveredHeading();
  plannedVisitedTransit = transitHeading != NO_HEADING;
  return transitHeading;
}

// --------------------------------------------------------------------------
// Phát lệnh chuyển động cho motor và ghép nhiều ô thành một hành trình thẳng.
// --------------------------------------------------------------------------
void startForward() {
  if (cruiseCellsRemaining == 0) {
    cruiseCellsRemaining = plannedVisitedTransit
      ? 1 : unvisitedCorridorLength(p.heading);
    // Hành lang mới được ghép thành một lần chạy dài. Nếu buộc đi qua ô cũ,
    // chỉ tiến một ô rồi để BFS lập kế hoạch lại.
    if (cruiseCellsRemaining == 0) {
      Serial.println(F("[NAV] NO CONFIRMED FREE CORRIDOR; WAIT FOR RADAR SWEEP"));
      motion = IDLE;
      pendingHeading = NO_HEADING;
      return;
    }
  }
  motion = AUTO_CRUISE;
  motionStart = millis();
  captureForwardYaw();
  MotorControl::forward(
    current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE
  );

  Serial.print(F("[NAV] FREE CORRIDOR cells="));
  Serial.print(cruiseCellsRemaining);
  if (plannedVisitedTransit) Serial.print(F(" (required visited transit)"));
  Serial.print(F(" PWM="));
  Serial.println(current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE);
}

void startBack() {
  motion = BACK_CELL;
  motionStart = millis();
  MotorControl::backward(
    current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE
  );
  Serial.println(F("[NAV] BACK UP (STUCK)"));
}

void startLeft() {
  motion = TURN_LEFT;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::left(PWM_TURN);
  Serial.printf("[NAV] TURN LEFT 90 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startRight() {
  motion = TURN_RIGHT;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::right(PWM_TURN);
  Serial.printf("[NAV] TURN RIGHT 90 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startTurnBack() {
  motion = TURN_BACK;
  motionStart = millis();
  captureTurnYaw();
  MotorControl::right(PWM_TURN);
  Serial.printf("[NAV] TURN BACK 180 IMU=%d\n", turnImuValid ? 1 : 0);
}

void startAvoidForward() {
  avoidPhase = AVOID_ESCAPING;
  motion = AVOID_FORWARD;
  motionStart = millis();
  captureForwardYaw();
  MotorControl::forward(PWM_EXPLORE);
  Serial.println(F("[NAV] AVOID FORWARD (escape)"));
}

void startAvoidBack() {
  avoidPhase = AVOID_BACKING;
  backingAlongHistory = !blindBacktrackMode && avoidBacktrackRemaining > 0;
  motion = BACK_CELL;
  motionStart = millis();
  MotorControl::backward(current == Navigation::PATROL ? PWM_PATROL : PWM_EXPLORE);
  Serial.println(blindBacktrackMode
    ? F("[NAV] AVOID: BLIND BACKUP (map-verified)")
    : (backingAlongHistory
      ? F("[NAV] AVOID: RETRACE PREVIOUS CELL")
      : F("[NAV] AVOID: CONTROLLED BACKUP")));
}

void turnToward(Heading target) {
  const uint8_t delta = (uint8_t)((target + 4 - p.heading) % 4);

  if (delta == 1) startRight();
  else if (delta == 3) startLeft();
  else if (delta == 2) startTurnBack();
}

void continueAvoidBacktrack() {
  if (avoidBacktrackRemaining == 0 || pathHistoryCount == 0) return;

  // Heading lưu là hướng đã dùng để đi vào ô. Căn thân theo hướng đó rồi chạy
  // lùi qua đúng cạnh vật lý; lặp lại sau mỗi góc đã ghi.
  const Heading retraceHeading = pathHistory[pathHistoryCount - 1].heading;
  if (p.heading != retraceHeading) {
    avoidPhase = AVOID_ALIGNING_BACKTRACK;
    turnToward(retraceHeading);
  } else {
    startAvoidBack();
  }
}

bool startBestAvoidEscape() {
  const uint8_t avoidHeading = chooseAvoidDirection();
  if (avoidHeading == NO_HEADING) return false;

  Serial.print(F("[NAV] ESCAPE SELECT heading="));
  Serial.println(avoidHeading);
  avoidPhase = AVOID_TURNING;
  if ((Heading)avoidHeading == p.heading) startAvoidForward();
  else turnToward((Heading)avoidHeading);
  return true;
}

// --------------------------------------------------------------------------
// Hoàn tất một trạng thái chuyển động và cập nhật pose/bản đồ.
// --------------------------------------------------------------------------
void finishMotion() {
  const Motion completedMotion = motion;
  MotorControl::stop();

  if (completedMotion == BACK_CELL) {
    // Ưu tiên lịch sử thật; chỉ dùng ước lượng khi không có ô trước khớp, ví dụ
    // ngay sau khởi động.
    const bool restored = backingAlongHistory && restorePreviousTravelCell();
    if (!restored) {
      Heading back = backOf(p.heading);
      int16_t nextX = p.x;
      int16_t nextY = p.y;
      advance(nextX, nextY, back);
      if (inside(nextX, nextY) && !MapMemory::isBlocked(nextX, nextY)) {
        p.x = nextX;
        p.y = nextY;
      }
    }
    backingAlongHistory = false;
  } else if (completedMotion == TURN_LEFT) {
    MapMemory::rotateLeft(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == TURN_RIGHT) {
    MapMemory::rotateRight(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == TURN_BACK) {
    MapMemory::rotateBack(p);
    if (consecutiveTurnsWithoutForward < 255) ++consecutiveTurnsWithoutForward;
  } else if (completedMotion == AVOID_FORWARD) {
    // Pha tránh né: tiến một ô theo hướng thoát đã chọn.
    MapMemory::updatePoseForward(p);
    markCurrentVisited();
    recordTravelPose();
    consecutiveTurnsWithoutForward = 0;
  }

  motion = IDLE;
  lastMotionFinish = millis();

  if (avoidPhase == AVOID_ALIGNING_BACKTRACK &&
      (completedMotion == TURN_LEFT || completedMotion == TURN_RIGHT ||
       completedMotion == TURN_BACK)) {
    startAvoidBack();
  } else if (avoidPhase == AVOID_BACKING && completedMotion == BACK_CELL) {
    ++avoidBacktrackedCells;
    if (avoidBacktrackRemaining > 0) --avoidBacktrackRemaining;
    if (avoidBacktrackRemaining > 0) {
      if (blindBacktrackMode) {
        startAvoidBack();   // Không có lịch sử: tiếp tục lùi thẳng, không căn hướng.
      } else {
        continueAvoidBacktrack();  // Có lịch sử: căn hướng rồi lùi đúng cạnh cũ.
      }
      return;
    }

    // Hoàn tất pha lùi; xóa cờ lùi không có lịch sử.
    blindBacktrackMode = false;

    // Bản đồ 3x3 thuộc pose cũ. Sau khi lùi xa, đánh giá pose mới bằng bản đồ
    // lâu dài thay vì dùng các ô cục bộ đã cũ.
    resetLocalMap();
    Serial.print(F("[NAV] BACKTRACKED cells="));
    Serial.println(avoidBacktrackedCells);

    if (startBestAvoidEscape()) return;

    // Sau đoạn lùi đầu chưa có hướng thoát, lùi thêm từng ô đã xác nhận rồi
    // chấm điểm lại, không vượt quá giới hạn lùi tối đa.
    if (avoidBacktrackReserve > 0) {
      --avoidBacktrackReserve;
      avoidBacktrackRemaining = 1;
      Serial.println(F("[NAV] NO ESCAPE; EXTEND BACKTRACK BY 1 CELL"));
      continueAvoidBacktrack();
      return;
    }

    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    Serial.println(F("[NAV] NO SAFE ESCAPE AFTER MAX BACKTRACK"));
  } else if (avoidPhase == AVOID_TURNING &&
             (completedMotion == TURN_LEFT || completedMotion == TURN_RIGHT ||
              completedMotion == TURN_BACK)) {
    startAvoidForward();
  } else if (avoidPhase == AVOID_ESCAPING && completedMotion == AVOID_FORWARD) {
    avoidPhase = AVOID_NONE;
    avoidBacktrackReserve = 0;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    plannedVisitedTransit = false;
    // Giữ lớp coverageVisited và tiếp tục từ bản đồ đã được SA2 cập nhật.
    Serial.println(F("[NAV] AVOID COMPLETE -> REPLAN SAVED MAP"));
  }

  Serial.print(F("[NAV] POSE x="));
  Serial.print(p.x);
  Serial.print(F(" y="));
  Serial.print(p.y);
  Serial.print(F(" heading="));
  Serial.println((int)p.heading);
}

// --------------------------------------------------------------------------
// Quét radar khi thân xe đứng yên.
// --------------------------------------------------------------------------
void startBackgroundScan() {
  backgroundScanActive = true;
  scanStartHeading = p.heading;
  lastScanTime = millis();

  SensorManager::setUltrasonicScanMode(true);
  ScanMapper::beginSweep();
  ServoScanner::startScan();

  Serial.println(F("[RADAR] STATIONARY SCAN START"));
}

// --------------------------------------------------------------------------
// Chọn hướng bắt đầu tốt nhất sau khi quét map 360° ban đầu:
// Đếm số cell FREE liên tiếp theo từng hướng từ vị trí robot,
// ưu tiên hướng có nhiều cell tự do nhất (corridor dài nhất).
// --------------------------------------------------------------------------
Heading chooseBestInitialHeading() {
  Heading bestHeading = NORTH;
  uint16_t bestCount  = 0;

  for (uint8_t h = 0; h < 4; ++h) {
    const Heading dir = (Heading)h;
    int16_t x = p.x;
    int16_t y = p.y;
    uint16_t count = 0;

    // Đếm số cell FREE liên tiếp (dừng khi gặp OBSTACLE hoặc UNKNOWN)
    for (uint16_t step = 0; step < (uint16_t)(MAP_W + MAP_H); ++step) {
      advance(x, y, dir);
      if (!inside(x, y) || MapMemory::get(x, y) != CELL_FREE) break;
      ++count;
    }

    Serial.print(F("[RADAR] INITIAL DIR "));
    Serial.print((int)dir);
    Serial.print(F(" free_cells="));
    Serial.println(count);

    if (count > bestCount) {
      bestCount   = count;
      bestHeading = dir;
    }
  }

  Serial.print(F("[RADAR] BEST INITIAL HEADING="));
  Serial.print((int)bestHeading);
  Serial.print(F(" cells="));
  Serial.println(bestCount);

  return bestHeading;
}

void runBackgroundScan() {
  ServoScanner::update();

  if (!ServoScanner::measurementDue()) return;

  uint32_t sum = 0;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < SCAN_SAMPLES_PER_ANGLE; ++i) {
    const uint16_t distance = SensorManager::readUltrasonicNow();

    if (distance > 0) {
      sum += distance;
      ++validSamples;
    }

    delay(5);
  }

  const uint16_t distanceCm =
    validSamples > 0 ? (uint16_t)(sum / validSamples) : 0;

  const float worldAngleDeg =
    (float)p.heading * 90.0f +
    (float)ServoScanner::angle() -
    (float)SERVO_SCAN_CENTER_DEG;

  const int16_t relativeAngle =
    (int16_t)ServoScanner::angle() - (int16_t)SERVO_SCAN_CENTER_DEG;
  if (relativeAngle >= -(int16_t)RADAR_FORWARD_CONE_DEG &&
      relativeAngle <= (int16_t)RADAR_FORWARD_CONE_DEG) {
    forwardRadarDistanceCm = distanceCm;
    forwardRadarValid = validSamples > 0 && distanceCm > 0;
    forwardRadarMs = millis();
  }

  // Khi thiếu encoder/pose tin cậy, chỉ quét lúc thân xe dừng mới đủ chính xác để ghi map.
  const bool headingStable =
    motion != TURN_LEFT && motion != TURN_RIGHT && motion != TURN_BACK &&
    !initialScanTurnActive;
  const bool integrateIntoMap = RADAR_MAP_ONLY_WHEN_STOPPED
    ? (motion == IDLE && !initialScanTurnActive)
    : headingStable;

  ScanMapper::addReading(
    p,
    worldAngleDeg,
    distanceCm,
    validSamples,
    SCAN_SAMPLES_PER_ANGLE,
    integrateIntoMap
  );

  ServoScanner::measurementTaken();

  if (ServoScanner::cycleFinished()) {
    ScanMapper::finishSweep(p, (uint8_t)(scanCycle + 1));
    scanCycle++;
    lastScanTime = millis();
    ServoScanner::holdPosition();
    // Ngăn đọc tự động kênh radar giữa hai lượt trước/sau; SA2 dùng chân riêng
    // nên vẫn đo liên tục.
    SensorManager::setUltrasonicScanMode(true);
    backgroundScanActive = false;
    // Không tái sử dụng số đo lúc đứng yên như số đo vật cản trực tiếp sau khi xe chạy.
    forwardRadarValid = false;
    // Vòng quét vừa công bố ô FREE/OBSTACLE mới; chạy lại bộ chọn đường để các
    // ô trống vừa phát hiện chắc chắn được ghé thăm.
    coverageComplete = false;
    if (motion == IDLE) pendingHeading = NO_HEADING;

    if (!initialMapReady && scanCycle < SCAN_CYCLES_PER_POSE) {
      backgroundScanActive = false;
      initialScanTurnActive = true;
      initialScanTurnStartMs = millis();
      const SensorSnapshot imu = SensorManager::get();
      initialScanTurnStartYaw = imu.yawDeg;
      initialScanTurnImuValid = imuReadingFresh(imu);
      MotorControl::right(PWM_TURN);
      Serial.printf("[RADAR] INITIAL BODY TURN 180 START IMU=%d\n",
                    initialScanTurnImuValid ? 1 : 0);
      return;
    }

    if (!initialMapReady) {
      initialMapReady = true;
      // Chọn hướng có nhiều cell FREE nhất trong bản đồ vừa quét (cả lần 1 lẫn lần 2).
      // Điều này đảm bảo xe đi về phía có không gian tự do nhiều nhất,
      // bất kể lần quét nào (trước hay sau khi xoay 180°) cho nhiều cell hơn.
      const Heading bestDir = chooseBestInitialHeading();
      sweepHeading = bestDir;
      laneHeading  = (Heading)((bestDir + 1) % 4);
      pendingHeading = NO_HEADING;
      MapMemory::save();
      current = Navigation::PATROL;
      ServoScanner::powerOff();
      radarPatrolDisabled = true;
      // Giữ kênh siêu âm radar im sau khi lập map; SensorManager vẫn đọc SA2
      // độc lập trong suốt tuần tra và sleep.
      SensorManager::setUltrasonicScanMode(true);
      Serial.println(F("[RADAR] FRONT+REAR MAP SAVED; RADAR DISABLED FOR PATROL"));
    }
    
    Serial.print(F("[RADAR] CYCLE "));
    Serial.print(scanCycle);
    Serial.println(F(" COMPLETE"));
    
  }
}

// --------------------------------------------------------------------------
// Tránh vật cản SA2 kết hợp bản đồ cục bộ 3x3.
// --------------------------------------------------------------------------
bool sa2ReadingFresh(const SensorSnapshot& s) {
  return s.sa2Valid &&
    millis() - s.sa2Ms <= 2 * ULTRASONIC_PERIOD_MS + 100;
}

bool sa2ObstacleConfirmed(const SensorSnapshot& s) {
  if (!sa2ReadingFresh(s) || s.sa2DistanceCm == 0) {
    sa2CloseSampleCount = 0;
    return false;
  }

  // Echo thật ở 1 cm nghĩa là mũi xe gần như chạm vật; xử lý ngay, không chờ
  // đủ chuỗi mẫu xác nhận thông thường.
  if (s.sa2DistanceCm <= SA2_STUCK_DISTANCE_CM) {
    sa2CloseSampleCount = 0;
    Serial.println(F("[SA2] STUCK <= 1cm -> IMMEDIATE BACKTRACK"));
    return true;
  }

  if (s.sa2Ms != lastSa2DecisionSampleMs) {
    lastSa2DecisionSampleMs = s.sa2Ms;
    if (s.sa2DistanceCm <= SA2_OBSTACLE_DISTANCE_CM) {
      if (sa2CloseSampleCount < SA2_OBSTACLE_CONFIRM_SAMPLES)
        ++sa2CloseSampleCount;
    } else {
      sa2CloseSampleCount = 0;
    }
  }

  return sa2CloseSampleCount >= SA2_OBSTACLE_CONFIRM_SAMPLES;
}

bool forwardRadarBlocked() {
  return forwardRadarValid &&
    millis() - forwardRadarMs <= RADAR_FORWARD_FRESH_MS &&
    forwardRadarDistanceCm > 1 &&
    forwardRadarDistanceCm <= RADAR_EMERGENCY_STOP_CM;
}

void handleForwardObstacle(uint16_t distanceCm,
                           const __FlashStringHelper* source,
                           bool persistToMap) {
  if (avoidPhase == AVOID_ESCAPING) avoidPhase = AVOID_NONE;
  if (avoidPhase != AVOID_NONE) return;

  MotorControl::stop();
  motion = IDLE;
  cruiseCellsRemaining = 0;

  // Đánh dấu vật cản trong bản đồ cục bộ.
  markLocalObstacle(p.heading);
  
  // Trong tuần tra radar đã tắt nên vật cản SA2 xác nhận được ghi trực tiếp vào
  // bản đồ chiếm chỗ lâu dài.
  if (persistToMap) {
    MapMemory::markObstacleAhead(p, distanceCm);
    int16_t x = p.x;
    int16_t y = p.y;
    advance(x, y, p.heading);
    if (inside(x, y)) MapMemory::set(x, y, CELL_OBSTACLE);
  }

  Serial.print(F("["));
  Serial.print(source);
  Serial.print(F("] OBSTACLE AT "));
  Serial.print(distanceCm);
  Serial.println(F("cm"));

  pendingHeading = NO_HEADING;

  // Lùi trước theo số ô đã xác nhận tối thiểu (hoặc hết lịch sử nếu ngắn hơn),
  // giữ phần còn lại làm dự phòng nếu vị trí đầu chưa có đường thoát an toàn.
  const uint8_t availableBacktrack = availableBacktrackCells();
  avoidBacktrackRemaining = availableBacktrack > AVOID_BACKTRACK_MIN_CELLS
    ? AVOID_BACKTRACK_MIN_CELLS : availableBacktrack;
  avoidBacktrackReserve = availableBacktrack - avoidBacktrackRemaining;
  avoidBacktrackedCells = 0;

  Serial.print(F("[NAV] BACKTRACK PLAN initial="));
  Serial.print(avoidBacktrackRemaining);
  Serial.print(F(" reserve="));
  Serial.println(avoidBacktrackReserve);

  if (avoidBacktrackRemaining == 0) {
    // Không có ô phía sau đã xác nhận, thường xảy ra lúc mới khởi động. Không
    // lùi mù; thay vào đó quay về hướng cục bộ chưa bị chặn.
    if (!startBestAvoidEscape()) {
      avoidPhase = AVOID_NONE;
      Serial.println(F("[NAV] NO VERIFIED BACKTRACK OR SAFE ESCAPE"));
    }
  } else {
    continueAvoidBacktrack();
  }
}

// --------------------------------------------------------------------------
// Khôi phục khi xe bị kẹt.
// --------------------------------------------------------------------------
void handleStuck() {
  MotorControl::stop();
  motion = IDLE;
  cruiseCellsRemaining = 0;
  
  Serial.println(F("[STUCK] BACKING UP AND REPLANNING"));
  
  // Lùi một ô.
  startBack();
  
  // Đặt lại bộ phát hiện kẹt.
  memset(sa2History, 0, sizeof(sa2History));
  sa2HistoryIndex = 0;
  stuckStartTime = 0;
  
  // Lập kế hoạch lại sau khi lùi.
  pendingHeading = NO_HEADING;
}

// --------------------------------------------------------------------------
// Luồng chính của thuật toán phủ bản đồ.
// --------------------------------------------------------------------------
void runCoverage(const SensorSnapshot& s) {
  // Quên bản đồ cục bộ đã quá cũ.
  if (millis() - localMapLastUpdate > LOCAL_MAP_FORGET_MS) {
    resetLocalMap();
  }

  if (avoidPhase == AVOID_NONE && forwardRadarBlocked()) {
    handleForwardObstacle(forwardRadarDistanceCm, F("RADAR"), true);
    return;
  }

  // Chỉ echo gần, hợp lệ và còn mới mới là vật cản. Theo chính sách no-echo,
  // nếu SA2 không trả về thì lớp bảo vệ từ radar/bản đồ vẫn còn hiệu lực.
  const bool sa2Fresh = sa2ReadingFresh(s);
  if (!sa2Fresh) {
    if (millis() - lastSafetyLogMs >= 1000) {
      lastSafetyLogMs = millis();
      Serial.println(SA2_STOP_ON_INVALID_READING
        ? F("[SAFETY] SA2 invalid/stale; motors stopped")
        : F("[SA2] no valid echo; continuing with radar/map protection"));
    }
    if (SA2_STOP_ON_INVALID_READING) {
      MotorControl::stop();
      return;
    }
  }

  if (avoidPhase == AVOID_NONE && sa2ObstacleConfirmed(s)) {
    registerSa2Obstacle(s.sa2DistanceCm);
    handleForwardObstacle(s.sa2DistanceCm, F("SA2"), true);
    MapMemory::save();
    return;
  }
  
  // Xóa trạng thái kẹt khi không còn vật cản gần.
  if (sa2Fresh && s.sa2DistanceCm > SA2_OBSTACLE_DISTANCE_CM) {
    stuckStartTime = 0;
    markLocalFree(p.heading);
  }

  // Quyết định hướng tiếp theo.
  if (pendingHeading == NO_HEADING) {
    pendingHeading = chooseCoverageHeading();

    if (pendingHeading == NO_HEADING) {
      coverageComplete = true;
      MapMemory::save();
      MotorControl::stop();
      Serial.println(F("[NAV] COVERAGE COMPLETE"));
      PowerManager::enterCoverageSleep();
      return;
    }
  }

  const Heading target = (Heading)pendingHeading;

  if (p.heading != target) {
    if (consecutiveTurnsWithoutForward >= MAX_TURNS_WITHOUT_FORWARD) {
      MotorControl::stop();
      pendingHeading = NO_HEADING;
      plannedVisitedTransit = false;
      consecutiveTurnsWithoutForward = 0;
      Serial.println(F("[NAV] TURN LOOP GUARD -> REPLAN SAVED MAP"));
      return;
    }
    turnToward(target);
    return;
  }

  // Kiểm tra ô kế có bị chặn trong bản đồ toàn cục hay không.
  int16_t nextX = p.x;
  int16_t nextY = p.y;
  advance(nextX, nextY, target);

  if (!coverageCellIsFree(nextX, nextY)) {
    Serial.println(F("[NAV] NEXT CELL NOT CONFIRMED FREE -> WAIT/REPLAN"));
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    plannedVisitedTransit = false;
    return;
  }

  // Tiến theo hành lang đã chọn.
  startForward();
}

} // namespace

namespace Navigation {

void begin() {
  p = {START_X, START_Y, NORTH};
  current = MapMemory::isLoaded() ? PATROL : EXPLORE;

  motion = IDLE;
  motionStart = millis();
  lastMotionFinish = millis();
  turnImuValid = false;
  forwardImuValid = false;
  initialScanTurnImuValid = false;

  backgroundScanActive = false;
  scanCycle = 0;
  controlWasManual = false;
  // Tạo bản đồ đầu tiên bằng hai lượt quét đứng yên (trước rồi sau) trước khi
  // chuyển động ước lượng làm thay đổi pose giả định.
  initialMapReady = false;
  radarPatrolDisabled = false;
  initialScanTurnActive = false;
  consecutiveTurnsWithoutForward = 0;
  sa2CloseSampleCount = 0;
  lastSa2DecisionSampleMs = 0;
  memset(sa2Obstacles, 0, sizeof(sa2Obstacles));
  sa2ObstacleCount = 0;
  rectangularBoundaryClosed = false;
  patrolMinX = 0;
  patrolMaxX = MAP_W - 1;
  patrolMinY = 0;
  patrolMaxY = MAP_H - 1;
  navigationWasSleeping = false;
  fireEntryImuValid = false;
  fireReturnActive = false;
  fireReturnStartMs = 0;
  avoidBacktrackRemaining = 0;
  avoidBacktrackReserve = 0;
  avoidBacktrackedCells = 0;
  backingAlongHistory = false;
  forwardRadarValid = false;
  forwardRadarMs = 0;

  resetCoverage();
  resetTravelPath();
  initLocalMap();
  ServoScanner::begin();

  Serial.println(F("[NAV] CONTINUOUS NAVIGATION"));
  Serial.println(F("[NAV] STATIONARY RADAR MAP + SA2 3x3 AVOIDANCE"));
  Serial.println(F("[NAV] WAITING FOR FRONT+REAR INITIAL MAP"));
}

void firstRunRoomScan() {
  // Điều hướng đứng yên cho đến khi quét tĩnh phía trước và phía sau hoàn tất.
  startBackgroundScan();
}

void update() {
  const SensorSnapshot s = SensorManager::get();

  // Khi chờ MQ-2, thân xe phải đứng hoàn toàn. Sau khi gas được xác nhận,
  // PowerManager mở khóa và tuần tra/xử lý cháy tiếp tục ở nhịp kế tiếp.
  if (PowerManager::sleeping()) {
    navigationWasSleeping = true;
    MotorControl::stop();
    MotorControl::setPump(false);
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    return;
  }

  if (navigationWasSleeping) {
    navigationWasSleeping = false;
    resetCoverage();
    resetTravelPath();
    current = Navigation::PATROL;
    Serial.println(F("[POWER] MQ2 smoke wake -> resume patrol on saved map"));
  }

  if (!s.ready) {
    MotorControl::stop();
    MotorControl::setPump(false);
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    return;
  }

  // Cháy có ưu tiên cao nhất. Ứng viên lửa quang học dừng điều hướng và bắt đầu
  // căn yaw; MQ-2 được FireControl kiểm tra riêng để ánh nắng không thể tự bật relay.
  if (s.opticalFireDetected) {
    if (current != FIRE) {
      fireEntryImuValid = imuReadingFresh(s);
      // Nếu lửa ngắt giữa lúc quay, lưu yaw bắt đầu vì heading bản đồ chưa được
      // chốt. Nếu ngắt lúc chạy thẳng, lưu hướng đang giữ vì cùng lý do.
      if (fireEntryImuValid && initialScanTurnActive &&
          initialScanTurnImuValid) {
        fireEntryYaw = initialScanTurnStartYaw;
      } else if (fireEntryImuValid &&
          (motion == TURN_LEFT || motion == TURN_RIGHT || motion == TURN_BACK) &&
          turnImuValid) {
        fireEntryYaw = turnStartYaw;
      } else if (fireEntryImuValid &&
                 (motion == AUTO_CRUISE || motion == AVOID_FORWARD) &&
                 forwardImuValid) {
        fireEntryYaw = forwardTargetYaw;
      } else {
        fireEntryYaw = s.yawDeg;
      }
      Serial.printf("[FIRE] navigation heading saved yaw=%.1f IMU=%d\n",
                    fireEntryYaw, fireEntryImuValid ? 1 : 0);
    }
    fireReturnActive = false;
    current = FIRE;
    motion = IDLE;
    avoidPhase = AVOID_NONE;
    pendingHeading = NO_HEADING;
    cruiseCellsRemaining = 0;
    plannedVisitedTransit = false;
    backgroundScanActive = false;
    initialScanTurnActive = false;
    if (!initialMapReady) scanCycle = 0;

    SensorManager::setUltrasonicScanMode(radarPatrolDisabled);
    ServoScanner::stop();

    FireControl::update(
    true,
    s.gasReady && s.gasDetected,
    s.fireNear || PUMP_ON_CONFIRMED_FIRE,
    s.flameLeftSignal,
    s.flameRightSignal,
    FlameServoScanner::held()
      ? FlameServoScanner::heldAngle() : FlameServoScanner::leftAngle()
    );
    return;
  }

  // Chế độ thủ công: giao quyền lái cho WebControl.
  if (!WebControl::automaticMode()) {
    if (!controlWasManual) {
      controlWasManual = true;
      motion = IDLE;
      avoidPhase = AVOID_NONE;
      pendingHeading = NO_HEADING;
      cruiseCellsRemaining = 0;
      plannedVisitedTransit = false;
      backgroundScanActive = false;
      initialScanTurnActive = false;
      if (!initialMapReady) scanCycle = 0;

      SensorManager::setUltrasonicScanMode(radarPatrolDisabled);
      ServoScanner::stop();
      MotorControl::stop();
    }
    return;
  }

  // Khôi phục hướng và tiếp tục tuần tra sau phiên chữa cháy.
  if (current == FIRE) {
    FireControl::update(false, false, false, 0, 0);

    if (fireEntryImuValid && imuReadingFresh(s)) {
      const float returnError = fireEntryYaw - s.yawDeg;
      if (!fireReturnActive &&
          fabsf(returnError) > MPU_FIRE_RETURN_TOLERANCE_DEG) {
        fireReturnActive = true;
        fireReturnStartMs = millis();
        Serial.printf("[FIRE] returning to saved heading error=%.1f deg\n",
                      returnError);
      }

      if (fireReturnActive &&
          fabsf(returnError) > MPU_FIRE_RETURN_TOLERANCE_DEG &&
          millis() - fireReturnStartMs < MPU_FIRE_RETURN_MAX_MS) {
        if (returnError > 0.0f) MotorControl::left(PWM_FIRE);
        else MotorControl::right(PWM_FIRE);
        return;
      }
    }

    MotorControl::stop();
    if (fireReturnActive) {
      Serial.printf("[FIRE] navigation heading restored yaw=%.1f\n", s.yawDeg);
    }
    fireReturnActive = false;
    current = MapMemory::isLoaded() ? PATROL : EXPLORE;
    controlWasManual = false;
    lastMotionFinish = millis();
    pendingHeading = NO_HEADING;
    plannedVisitedTransit = false;
    Serial.println(F("[FIRE] treatment complete -> resume patrol"));
    return;
  }

  if (controlWasManual) {
    controlWasManual = false;
    // Chuyển động Manual không cập nhật odometry; không lùi theo lịch sử có tọa
    // độ vật lý không còn bảo đảm sau khi người dùng đã lái tay.
    resetTravelPath();
    lastMotionFinish = millis();
    return;
  }

  if (initialScanTurnActive) {
    const uint32_t elapsed = millis() - initialScanTurnStartMs;
    const bool imuFresh = initialScanTurnImuValid && imuReadingFresh(s);
    const float directedProgress =
      initialScanTurnStartYaw - s.yawDeg; // Lần quay thân ban đầu là quay phải.
    const bool imuReached = imuFresh && elapsed >= MPU_TURN_MIN_MS &&
      directedProgress >= MPU_TURN_180_STOP_DEG;
    const uint32_t timeout = imuFresh
      ? 2 * MPU_TURN_MAX_90_MS : SCAN_BODY_TURN_MS;
    if (imuReached || elapsed >= timeout) {
      MotorControl::stop();
      MapMemory::rotateBack(p);
      initialScanTurnActive = false;
      initialScanTurnImuValid = false;
      scanStartHeading = p.heading;
      startBackgroundScan();
      Serial.printf("[RADAR] INITIAL BODY TURN 180 COMPLETE yaw=%.1f source=%s\n",
                    directedProgress,
                    imuReached ? "IMU" : "TIME");
    } else {
      // Tiếp tục phát lệnh quay. Trước đây nhánh !initialMapReady dừng motor
      // ngay sau lệnh đầu, khiến pose đổi nhưng thân xe thực tế không quay.
      MotorControl::right(PWM_TURN);
    }
    return;
  }

  if (backgroundScanActive) {
    runBackgroundScan();
  } else if (!initialMapReady && !initialScanTurnActive) {
    startBackgroundScan();
  }

  // Vòng quét đầu hoàn tất có thể khởi động quay thân 180 độ; không chạy tiếp
  // xuống dưới rồi hủy lệnh ngay trong cùng một nhịp cập nhật.
  if (initialScanTurnActive) return;

  if (motion == AUTO_CRUISE || motion == AVOID_FORWARD) {
    if (forwardRadarBlocked()) {
      handleForwardObstacle(forwardRadarDistanceCm, F("RADAR"), true);
      return;
    }

    if (!sa2ReadingFresh(s)) {
      if (millis() - lastSafetyLogMs >= 1000) {
        lastSafetyLogMs = millis();
        Serial.println(SA2_STOP_ON_INVALID_READING
          ? F("[SAFETY] SA2 lost during forward motion; stopped")
          : F("[SA2] no valid echo while moving; radar/map remains active"));
      }
      if (SA2_STOP_ON_INVALID_READING) {
        MotorControl::stop();
        motion = IDLE;
        avoidPhase = AVOID_NONE;
        pendingHeading = NO_HEADING;
        cruiseCellsRemaining = 0;
        return;
      }
    }

    if (sa2ObstacleConfirmed(s)) {
      registerSa2Obstacle(s.sa2DistanceCm);
      handleForwardObstacle(s.sa2DistanceCm, F("SA2"), true);
      MapMemory::save();
      return;
    }

    maintainForwardHeading(s);
  }

  // Kiểm tra và hoàn tất chuyển động theo thời gian hoặc MPU6050.
  if (motion != IDLE) {
    if (motion == AUTO_CRUISE) {
      if (millis() - motionStart >= AUTO_POSE_CELL_INTERVAL_MS) {
        int16_t nextX = p.x;
        int16_t nextY = p.y;
        advance(nextX, nextY, p.heading);

        if (!inside(nextX, nextY) || MapMemory::isBlocked(nextX, nextY)) {
          Serial.println(F("[NAV] CRUISE PATH BLOCKED -> REPLAN"));
          MotorControl::stop();
          motion = IDLE;
          pendingHeading = NO_HEADING;
          cruiseCellsRemaining = 0;
          plannedVisitedTransit = false;
          return;
        }

        MapMemory::updatePoseForward(p);
        markCurrentVisited();
        recordTravelPose();
        consecutiveTurnsWithoutForward = 0;

        if (cruiseCellsRemaining > 0) --cruiseCellsRemaining;
        if (cruiseCellsRemaining == 0) {
          if (plannedVisitedTransit) {
            // Ô này được đi lại vì BFS chứng minh nó nằm trên đường tới ô FREE
            // còn lại; lập kế hoạch ngay, không quét radar tốn thời gian tại mỗi ô.
            MotorControl::stop();
            motion = IDLE;
            pendingHeading = NO_HEADING;
            plannedVisitedTransit = false;
            lastMotionFinish = millis();
            Serial.println(F("[NAV] REQUIRED VISITED TRANSIT COMPLETE -> REPLAN"));
            return;
          }
          MotorControl::stop();
          motion = IDLE;
          pendingHeading = NO_HEADING;
          plannedVisitedTransit = false;
          lastMotionFinish = millis();
          Serial.println(F("[NAV] LONG FREE RUN COMPLETE -> REPLAN SAVED MAP"));
          return;
        }

        motionStart += AUTO_POSE_CELL_INTERVAL_MS;
      }
      // Giữ PWM tiến liên tục; bộ định thời ô chỉ cập nhật pose.
      return;
    }

    uint32_t required = motion == TURN_BACK ? 2 * TURN_90_MS : TURN_90_MS;
    if (motion == BACK_CELL) {
      required = avoidPhase == AVOID_BACKING
        ? (backingAlongHistory ? AUTO_POSE_CELL_INTERVAL_MS : SA2_AVOID_BACK_MS)
        : AUTO_POSE_CELL_INTERVAL_MS;
    } else if (motion == AVOID_FORWARD) {
      // Thoát đúng một ô đã hiệu chỉnh để đường lưu và vị trí vật lý nhất quán;
      // bộ lập kế hoạch sẽ quyết định ô tiếp theo.
      required = AUTO_POSE_CELL_INTERVAL_MS;
    } else if ((avoidPhase == AVOID_TURNING ||
                avoidPhase == AVOID_ALIGNING_BACKTRACK) &&
               (motion == TURN_LEFT || motion == TURN_RIGHT || motion == TURN_BACK)) {
      required = motion == TURN_BACK
        ? 2 * SA2_AVOID_TURN_90_MS : SA2_AVOID_TURN_90_MS;
    }

    bool motionFinished = millis() - motionStart >= required;
    bool turnImuReached = false;
    const bool isTurn = motion == TURN_LEFT || motion == TURN_RIGHT ||
      motion == TURN_BACK;
    if (isTurn && turnImuValid && imuReadingFresh(s)) {
      const float target = motion == TURN_BACK
        ? MPU_TURN_180_STOP_DEG : MPU_TURN_90_STOP_DEG;
      const uint32_t elapsed = millis() - motionStart;
      turnImuReached = elapsed >= MPU_TURN_MIN_MS &&
        directedTurnProgressDeg(motion, turnStartYaw, s) >= target;
      motionFinished = turnImuReached ||
        elapsed >= (motion == TURN_BACK
          ? 2 * MPU_TURN_MAX_90_MS : MPU_TURN_MAX_90_MS);
    }

    if (motionFinished) {
      if (isTurn) {
        Serial.printf("[NAV] TURN COMPLETE yaw=%.1f source=%s\n",
                      directedTurnProgressDeg(motion, turnStartYaw, s),
                      turnImuReached ? "IMU" : "TIME");
      }
      finishMotion();
    }
    return;
  }

  if (!initialMapReady) {
    MotorControl::stop();
    return;
  }

  // Dùng bản đồ tĩnh gần nhất; SA2 tiếp tục bảo vệ vật cản trực tiếp khi xe chạy.
  runCoverage(s);
}

Pose pose() {
  return p;
}

Mode mode() {
  return current;
}

void forceStop() {
  motion = IDLE;
  pendingHeading = NO_HEADING;
  cruiseCellsRemaining = 0;
  plannedVisitedTransit = false;
  backgroundScanActive = false;

  SensorManager::setUltrasonicScanMode(radarPatrolDisabled);
  ServoScanner::stop();
  MotorControl::stop();
}

bool visited(int16_t x, int16_t y) {
  if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return false;
  return coverageVisited[y][x];
}

} // namespace Navigation
