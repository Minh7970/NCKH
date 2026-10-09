#include "MapMemory.h"
#include "RobotConfig.h"

/*
 * Bản đồ dùng ba trạng thái UNKNOWN, FREE và OBSTACLE. Quan sát mới không ghi
 * thẳng ngay mà tích lũy điểm xác nhận, ngoại trừ dữ liệu radar đã được hai mẫu
 * và cả vòng quét xác nhận. Vật cản được nở theo bán kính an toàn để bộ lập
 * đường tránh cả bề rộng của tủ, tường hoặc vật lớn chứ không chỉ một điểm echo.
 */
#include <EEPROM.h>
#include <string.h>
#include <math.h>

// Namespace ẩn danh: dữ liệu và hàm hỗ trợ chỉ dùng nội bộ trong file này
namespace {
  // Lưới bản đồ: mỗi ô lưu một giá trị CellType (chưa biết / trống / vật cản)
  uint8_t mapGrid[MAP_H][MAP_W];
  // Điểm quan sát của từng ô, dùng để lọc nhiễu:
  // dương = nghiêng về "vật cản", âm = nghiêng về "trống"
  int8_t observationScore[MAP_H][MAP_W];
  // Cờ báo bản đồ đã thay đổi so với bản lưu trong EEPROM (cần lưu lại)
  bool changed = false;
  // Cờ báo bản đồ đã được nạp/lưu hợp lệ từ EEPROM
  bool loaded = false;

  // Kiểm tra tọa độ (x, y) có nằm trong phạm vi bản đồ hay không
  bool inside(int16_t x, int16_t y) {
    return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
  }

  // Ghi trực tiếp giá trị cho một ô, chỉ đánh dấu "changed" khi giá trị thật sự khác
  void writeCell(int16_t x, int16_t y, CellType c) {
    // Bỏ qua nếu tọa độ nằm ngoài bản đồ
    if (!inside(x,y)) return;
    if (mapGrid[y][x] != (uint8_t)c) {
      mapGrid[y][x] = (uint8_t)c;
      changed = true;
    }
  }

  // Ghi nhận một lần quan sát cho ô (x, y) theo cơ chế xác nhận nhiều lần
  // Chỉ khi đủ số lần xác nhận liên tiếp thì mới cập nhật ô vào bản đồ thật
  void observeCell(int16_t x, int16_t y, CellType c) {
    if (!inside(x,y)) return;
    // Tham chiếu tới điểm quan sát của ô này để chỉnh sửa trực tiếp
    int8_t& score = observationScore[y][x];
    if (c == CELL_OBSTACLE) {
      // Quan sát thấy vật cản: tăng điểm (có giới hạn trên)
      if (score < (int8_t)DYNAMIC_OBSTACLE_CONFIRMATIONS) ++score;
      // Đủ số lần xác nhận vật cản thì ghi vào bản đồ
      if (score >= (int8_t)DYNAMIC_OBSTACLE_CONFIRMATIONS)
        writeCell(x, y, CELL_OBSTACLE);
    } else if (c == CELL_FREE) {
      // Quan sát thấy ô trống: giảm điểm (có giới hạn dưới)
      if (score > -(int8_t)DYNAMIC_FREE_CONFIRMATIONS) --score;
      // Đủ số lần xác nhận ô trống thì ghi vào bản đồ
      if (score <= -(int8_t)DYNAMIC_FREE_CONFIRMATIONS)
        writeCell(x, y, CELL_FREE);
    }
  }

  // Echo siêu âm chỉ là một điểm trên bề mặt phản xạ. Sau khi điểm đó vượt bộ
  // lọc xác nhận, nở thành vùng cấm nhỏ để tính bề rộng robot và giúp bộ lập
  // đường nhìn thấy vật lớn như tủ/tủ lạnh trước khi va chạm.
  void observeObstacleFootprint(int16_t obstacleX, int16_t obstacleY,
                                int16_t sensorX, int16_t sensorY,
                                bool confirmedReflection = false) {
    // Mỗi góc radar dùng hai mẫu echo. Khi cả hai trùng nhau, công bố vật cản
    // ngay lúc chốt vòng quét; nguồn dữ liệu khác vẫn qua bộ lọc nhiều quan sát.
    if (confirmedReflection) writeCell(obstacleX, obstacleY, CELL_OBSTACLE);
    else observeCell(obstacleX, obstacleY, CELL_OBSTACLE);
    if (!inside(obstacleX, obstacleY) ||
        mapGrid[obstacleY][obstacleX] != CELL_OBSTACLE) return;

    const int16_t radius = MAP_OBSTACLE_INFLATION_CELLS;
    for (int16_t dy = -radius; dy <= radius; ++dy) {
      for (int16_t dx = -radius; dx <= radius; ++dx) {
        const int16_t x = obstacleX + dx;
        const int16_t y = obstacleY + dy;
        if (!inside(x, y)) continue;
        // Không bao giờ biến chính ô robot đang đứng thành vật cản.
        if (x == sensorX && y == sensorY) continue;
        writeCell(x, y, CELL_OBSTACLE);
      }
    }
  }
}

namespace MapMemory {
  // Khởi tạo module: cấp phát EEPROM, xóa bản đồ rồi thử nạp bản đồ đã lưu
  void begin() {
    EEPROM.begin(EEPROM_SIZE);
    clear();
    loaded = load();
  }

  // Xóa toàn bộ bản đồ về trạng thái "chưa biết" và xóa điểm quan sát
  void clear() {
    memset(mapGrid, CELL_UNKNOWN, sizeof(mapGrid));
    memset(observationScore, 0, sizeof(observationScore));
    changed = true;
    loaded = false;
  }

  // Nạp bản đồ từ EEPROM, trả về đúng nếu dữ liệu hợp lệ.
  bool load() {
    // Các biến chứa phần header đọc từ EEPROM
    uint16_t magic;
    uint8_t ver,w,h,sx,sy;
    uint16_t savedChecksum;
    // p là con trỏ vị trí (địa chỉ) đang đọc trong EEPROM
    int p=0;

    // Đọc header: mã nhận dạng, phiên bản, kích thước bản đồ,
    // vị trí xuất phát và checksum đã lưu
    EEPROM.get(p, magic); p+=2;
    EEPROM.get(p, ver); p++;
    EEPROM.get(p, w); p++;
    EEPROM.get(p, h); p++;
    EEPROM.get(p, sx); p++;
    EEPROM.get(p, sy); p++;
    EEPROM.get(p, savedChecksum); p+=2;

    // Header không khớp (sai mã, sai phiên bản hoặc sai kích thước) thì bỏ qua dữ liệu cũ
    if (magic != EEPROM_MAGIC || ver != EEPROM_VERSION ||
        w != MAP_W || h != MAP_H) return false;

    // Đọc từng ô của bản đồ từ EEPROM vào mảng mapGrid
    for (uint16_t i=0; i<MAP_W*MAP_H; i++) {
      uint8_t v;
      EEPROM.get(p+i, v);
      mapGrid[i/MAP_W][i%MAP_W] = v;
    }

    // Tính lại checksum và so với checksum đã lưu để phát hiện dữ liệu hỏng
    uint16_t calc = checksum();
    if (calc != savedChecksum) {
      // Dữ liệu hỏng: xóa bản đồ để tránh dùng dữ liệu sai
      clear();
      return false;
    }

    // Nạp thành công: bản đồ đồng bộ với EEPROM nên không còn thay đổi chờ lưu
    changed = false;
    memset(observationScore, 0, sizeof(observationScore));
    return true;
  }

  // Lưu phần đầu và dữ liệu các ô vào EEPROM, trả về đúng nếu thành công.
  bool save() {
    int p=0;
    // Chuẩn bị các giá trị header cần ghi
    uint16_t magic=EEPROM_MAGIC;
    uint8_t ver=EEPROM_VERSION,w=MAP_W,h=MAP_H,sx=START_X,sy=START_Y;
    // Tính checksum của bản đồ hiện tại
    uint16_t sum=checksum();

    // Ghi header theo đúng thứ tự mà hàm load() sẽ đọc
    EEPROM.put(p,magic); p+=2;
    EEPROM.put(p,ver); p++;
    EEPROM.put(p,w); p++;
    EEPROM.put(p,h); p++;
    EEPROM.put(p,sx); p++;
    EEPROM.put(p,sy); p++;
    EEPROM.put(p,sum); p+=2;

    // Ghi từng ô của bản đồ, ngay sau phần header
    for (uint16_t i=0;i<MAP_W*MAP_H;i++)
      EEPROM.write(p+i,mapGrid[i/MAP_W][i%MAP_W]);

    // Xác nhận ghi thật sự vào bộ nhớ flash; thất bại thì báo lỗi
    if (!EEPROM.commit()) return false;

    // Lưu xong: bản đồ đã đồng bộ với EEPROM
    changed=false;
    loaded=true;
    return true;
  }

  // Trả về đúng nếu bản đồ đã được nạp hoặc lưu hợp lệ từ EEPROM.
  bool isLoaded() { return loaded; }

  // Lấy loại ô tại (x, y); ngoài phạm vi bản đồ được coi là vật cản
  CellType get(int16_t x, int16_t y) {
    if (!inside(x,y)) return CELL_OBSTACLE;
    return (CellType)mapGrid[y][x];
  }

  // Kiểm tra ô (x, y) có bị chặn (là vật cản) hay không
  bool isBlocked(int16_t x, int16_t y) {
    return get(x, y) == CELL_OBSTACLE;
  }

  // Đặt trực tiếp loại ô tại (x, y), không qua bước xác nhận nhiều lần
  void set(int16_t x, int16_t y, CellType c) {
    writeCell(x,y,c);
  }

  // Chuyển một tia siêu âm theo hướng la bàn (Bắc/Nam/Đông/Tây) thành các ô trống
  // dọc đường đi của tia và một ô vật cản ở điểm cuối
  void markRay(int16_t x, int16_t y, Heading h, uint16_t distanceCm) {
    /*
     * Chuyển một tia đo siêu âm thành chuỗi ô trống và một ô vật cản ở cuối tia.
     * Cách cập nhật này được thiết kế thận trọng: số đo bằng 0 hoặc không hợp lệ
     * tuyệt đối không được xem là khoảng đường trống.
     */
    // Bỏ qua nếu vị trí ngoài bản đồ hoặc số đo bằng 0 (số đo không hợp lệ)
    if (!inside(x,y) || distanceCm == 0) return;

    // Xác định bước di chuyển theo từng ô ứng với hướng của robot
    int dx=0,dy=0;
    if (h==NORTH) dy=-1;
    else if (h==SOUTH) dy=1;
    else if (h==EAST) dx=1;
    else dx=-1;

    // Khoảng cách được coi là trống = khoảng đo trừ đi phần đệm gần vật cản (OBSTACLE_MAP_CM)
    uint16_t freeCm = distanceCm > OBSTACLE_MAP_CM
                    ? distanceCm - OBSTACLE_MAP_CM : 0;
    // Đổi khoảng cách trống sang số ô của lưới
    uint16_t freeCells = freeCm / MAP_CELL_CM;

    // Đi dọc theo tia, ghi nhận từng ô là "trống"
    int cx=x, cy=y;
    for (uint16_t i=0;i<freeCells;i++) {
      cx += dx; cy += dy;
      if (!inside(cx,cy)) break;
      observeCell(cx,cy,CELL_FREE);
    }

    // Chỉ đánh dấu vật cản nếu số đo nhỏ hơn tầm đo tối đa
    // (nếu bằng/lớn hơn nghĩa là tia không chạm vật nào)
    if (distanceCm < (uint16_t)(MAX_RANGE_CM)) {
      // Tính tọa độ ô chứa vật cản (làm tròn theo nửa ô)
      int ox = x + dx * (int)((distanceCm + MAP_CELL_CM/2)/MAP_CELL_CM);
      int oy = y + dy * (int)((distanceCm + MAP_CELL_CM/2)/MAP_CELL_CM);
      observeObstacleFootprint(ox, oy, x, y);
    }
  }


  // Giống markRay nhưng tia có thể theo góc bất kỳ (độ) thay vì chỉ 4 hướng
  void markRayAngle(int16_t x, int16_t y, float angleDeg, uint16_t distanceCm,
                    bool hasReflection, bool confirmedReflection) {
    // Bỏ qua nếu vị trí ngoài bản đồ hoặc số đo không hợp lệ
    if (!inside(x,y) || distanceCm == 0) return;

    // Quy ước 0° = Bắc, 90° = Đông; độ chính xác số thực đủ cho lưới thô 5 cm.
    // Đổi góc từ độ sang radian (0.01745329252 = PI / 180)
    float rad = angleDeg * 0.01745329252f;
    // Thành phần hướng theo trục Đông (x) và trục Bắc (y)
    float sx = sin(rad);      // Thành phần theo hướng Đông.
    float sy = -cos(rad);     // Thành phần theo hướng Bắc.

    // min của Arduino là macro, không phải khuôn hàm C++.
    // Giới hạn khoảng đo không vượt quá tầm đo tối đa
    uint16_t maxCm = distanceCm < MAX_RANGE_CM ? distanceCm : MAX_RANGE_CM;
    // Bước lấy mẫu dọc theo tia bằng đúng kích thước một ô
    uint16_t stepCm = MAP_CELL_CM;
    // Phần khoảng cách được coi là trống, đã trừ đi phần đệm gần vật cản
    uint16_t freeCm = maxCm > OBSTACLE_MAP_CM ? maxCm - OBSTACLE_MAP_CM : 0;

    // Đi dọc theo tia từng bước một ô, đánh dấu các ô đi qua là "trống"
    for (uint16_t d=stepCm; d<=freeCm; d+=stepCm) {
      int cx = x + (int)round((sx*d)/MAP_CELL_CM);
      int cy = y + (int)round((sy*d)/MAP_CELL_CM);
      if (!inside(cx,cy)) break;
      observeCell(cx,cy,CELL_FREE);
    }

    // Nếu tia chạm vật (số đo nhỏ hơn tầm đo tối đa) thì đánh dấu ô cuối là vật cản
    if (hasReflection && distanceCm < MAX_RANGE_CM) {
      int ox = x + (int)round((sx*distanceCm)/MAP_CELL_CM);
      int oy = y + (int)round((sy*distanceCm)/MAP_CELL_CM);
      if (inside(ox,oy)) {
        observeObstacleFootprint(ox, oy, x, y, confirmedReflection);
      }
    }
  }

  // Đánh dấu vật cản phía trước robot dựa trên hướng hiện tại trong Pose
  void markObstacleAhead(const Pose& p, uint16_t distanceCm) {
    if (distanceCm == 0) return;
    markRay(p.x,p.y,p.heading,distanceCm);
  }

  // Cập nhật vị trí robot khi tiến lên một ô theo hướng hiện tại,
  // rồi đánh dấu ô mới đến là "trống" (vì robot đã đi vào được)
  void updatePoseForward(Pose& p) {
    if (p.heading==NORTH) p.y--;
    else if (p.heading==SOUTH) p.y++;
    else if (p.heading==EAST) p.x++;
    else p.x--;
    if (inside(p.x,p.y)) writeCell(p.x,p.y,CELL_FREE);
  }

  // Quay trái 90°: cộng 3 (tương đương trừ 1) trong vòng 4 hướng
  void rotateLeft(Pose& p) {
    p.heading = (Heading)((p.heading + 3) % 4);
  }

  // Quay phải 90°: cộng 1 trong vòng 4 hướng
  void rotateRight(Pose& p) {
    p.heading = (Heading)((p.heading + 1) % 4);
  }

  // Quay ngược lại 180°: cộng 2 trong vòng 4 hướng
  void rotateBack(Pose& p) {
    p.heading = (Heading)((p.heading + 2) % 4);
  }

  // Tính checksum của toàn bộ bản đồ để kiểm tra tính toàn vẹn dữ liệu khi lưu/nạp
  uint16_t checksum() {
    // Giá trị khởi tạo cố định
    uint16_t s=0x5AA5;
    // Trộn từng ô vào checksum bằng phép dịch bit và XOR
    for (uint16_t i=0;i<MAP_W*MAP_H;i++) {
      s = (uint16_t)((s << 5) ^ (s >> 11) ^ mapGrid[i/MAP_W][i%MAP_W]);
    }
    return s;
  }

  // Trả về đúng nếu bản đồ có thay đổi chưa được lưu.
  bool dirty() { return changed; }
  // Xóa cờ thay đổi (gọi sau khi đã lưu)
  void clearDirty() { changed=false; }

  // In bản đồ ra Serial để gỡ lỗi, đánh dấu vị trí robot bằng chữ 'R'
  void print(const Pose& p) {
    Serial.println(F("\n========== ROOM MAP =========="));
    Serial.print(F("Cell=")); Serial.print(MAP_CELL_CM);
    Serial.println(F(" cm, '?' unknown '.' free '#' obstacle 'R' robot"));
    // Duyệt từng hàng (y) rồi từng cột (x) của bản đồ
    for (int y=0;y<MAP_H;y++) {
      for (int x=0;x<MAP_W;x++) {
        // Ô hiện tại là vị trí robot thì in 'R'
        if (x==p.x && y==p.y) Serial.print('R');
        else {
          // Ngược lại in '.' cho ô trống, '#' cho vật cản, '?' cho ô chưa biết
          uint8_t v=mapGrid[y][x];
          Serial.print(v==CELL_FREE?'.':(v==CELL_OBSTACLE?'#':'?'));
        }
      }
      Serial.println();
    }
    Serial.println(F("=============================="));
  }
}
