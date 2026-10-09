# Fire Robot — ESP32 ultrasonic mapping and coverage

## Hardware and modules

- ESP32 DevKit (30-pin / ESP32 Dev Module)
- L298N dual motor driver, differential-drive chassis
- MPU6050 relative-yaw IMU
- SRF04 ultrasonic sensor mounted on an SG90 positional servo
- SA2: fixed forward-facing SRF05 ultrasonic sensor for obstacle avoidance
- Two analog flame sensors and a water pump with a fixed forward-facing tube

The servo SRF04 uses `TRIG=GPIO21` and `ECHO=GPIO22` to scan the surrounding
space and update the map. SA2 uses `TRIG=GPIO4` and `ECHO=GPIO13`; it is read
continuously after the initial map scan and is the sensor used to stop or
redirect the robot around a forward obstacle. Both ultrasonic echo pins must
be level-shifted to 3.3 V before connecting to the ESP32.

The MPU6050 uses `SDA=GPIO23`, `SCL=GPIO5`, `VCC=3V3`, `GND=GND`, and
`AD0=GND` (I2C address `0x68`). Keep the chassis completely still during the
short gyro calibration at startup.

## Wi-Fi map dashboard

Set `WIFI_SSID` and `WIFI_PASSWORD` in `RobotConfig.h` to join a local network.
If they are left blank or the connection times out, the ESP32 starts the
`FireRobot-Map` access point (password `firemap123`). The read-only map API is
`http://<ESP32-IP>/api/map` and returns the occupancy grid and robot pose as
JSON. The dashboard at `D:\NCKH\index.html` polls this endpoint once per second;
enter the ESP32 IP in its connection field. In fallback AP mode use
`http://192.168.4.1` after connecting the computer to `FireRobot-Map`.

The motor dashboard at `D:\NCKH\ESP32_Motor_Control_Web\index.html` uses the
same ESP32 address. It provides `AUTO` and `MANUAL` modes. Manual drive requests
use `/api/control/mode?mode=manual` and
`/api/control/drive?cmd=FORWARD&speed=150`; valid commands are `FORWARD`,
`BACKWARD`, `LEFT`, `RIGHT`, and `STOP`. The browser renews held-button commands
and the ESP32 stops if renewal expires. Forward manual drive is blocked when a
fresh SA2 reading detects an obstacle at 15 cm or closer; fire detection also
stops manual drive. Selecting `AUTO` restarts an SRF04 scan before autonomous
navigation.

## Navigation model

This project implements a practical **grid-SLAM-lite** model for the available
hardware. The map is a 40 × 40 occupancy grid, with one cell representing
5 cm. The robot pose is `(x, y, heading)` and is updated after every calibrated
cell move and 90° turn. On startup, the top-mounted SRF04 maps the front and
rear while the chassis remains still during each sweep. The robot also stops
for a fresh sweep after a corridor run, obstacle avoidance, or repeated turns.
Each valid reading is processed as an ultrasonic ray: cells along the ray
become free and its endpoint becomes an obstacle.

This is not metric-grade SLAM. The MPU6050 measures relative yaw: turns stop at
the requested 90/180-degree angle and a proportional PWM correction holds the
startup heading during straight runs. It has no magnetometer, so NORTH means
the direction in which the chassis points at boot, not geographic north.
Forward distance still uses calibrated motor time and can drift with wheel slip
or battery voltage.

The scanner uses absolute servo angles and reverses between
`SERVO_SCAN_MIN_DEG` and `SERVO_SCAN_MAX_DEG`, so the SRF04 cable cannot wind
around the shaft. It scans `0° -> 180° -> 0°` in 5° increments, waits 35 ms
for settling, and averages two SRF04 readings at each angle. `ScanMapper`
converts valid angle/range readings into occupancy-grid rays. After each full
sweep it prints the accumulated map to Serial Monitor: `?` unknown, `.` free,
`#` obstacle, and `R` robot position. The chassis then turns right 180° for
the second initial scan orientation. The servo can continue reporting live
radar while the chassis moves, but those moving observations are not written
to the occupancy map because the robot has no wheel-encoder position correction. The
live SA2 SRF05 remains the forward obstacle-avoidance sensor.

## Dynamic obstacle updates

To prevent a single noisy echo from changing the map, an ultrasonic result must
be observed twice before it changes a cell. Two free ray observations clear a
previous obstacle, so a chair or other movable object is removed from the map
after the robot observes the cleared path. A confirmed obstacle is inflated by
10 cm in the occupancy grid so wide surfaces and the robot's required clearance
are visible to the planner. The map is stored in ESP32 flash.

## Coverage path

The robot uses a snake/S-pattern preference:

1. Travel along a horizontal lane.
2. At an end-of-lane obstacle, shift one 5 cm cell vertically.
3. Reverse horizontal direction and cover the next parallel lane.
4. If an internal obstacle cuts a lane, breadth-first search finds the nearest
   reachable unvisited cell and coverage continues there.

Adjacent confirmed-free cells in the chosen heading are merged into one straight
motor run. The pose and route history are still updated internally every 5 cm,
without stopping or replanning between cells. The most recent 256 travelled cells
are retained in RAM. On a forward obstacle, the robot first retraces five recorded
cells (25 cm), including recorded corners, then scores the left, right, and rear
routes over a ten-cell lookahead. If all routes are blocked it retreats one cell
at a time and reevaluates, up to ten cells (50 cm). With less than five verified
history cells it only retraces the available safe route. This history is not
restored after reset because the physical robot pose cannot be guaranteed.

## Safety

The robot waits for stationary front and rear startup sweeps before coverage.
SA2 confirms an obstacle at 15 cm or closer from three consecutive fresh
samples; a critical reading at 5 cm or closer stops immediately. It then
retraces up to five to ten travelled cells and replans. A missing SA2 echo is
reported in telemetry but does not immobilize the chassis; the radar and stored
map remain active. Manual forward commands use the same 15 cm threshold.
During a stationary sweep, a radar timeout/no-echo releases only the adjacent
5 cm cell as FREE. This lets the robot enter an open area without incorrectly
assuming the complete 250 cm radar ray is obstacle-free.
The radar keeps its 5-degree map resolution but now settles for 35 ms per step,
uses two ultrasonic samples per angle, and pauses only 80 ms when reversing its
sweep. A separate 200 ms delay is retained for the initial 90-to-0-degree move.
The SRF05/SRF04 minimum range and chassis stopping distance must still be tested
on the real robot before autonomous operation.

The current hardware confirms fire from the analogue AO outputs of two flame
modules: LEFT AO connects to GPIO32 and RIGHT AO connects to GPIO33. The fixed-
angle detector learns the ambient infrared baseline, rejects saturated/steady
sunlight, and requires flame-like signal variation before `fireDetected` becomes
true. Digital DO support remains in the code but is disabled with
`FLAME_USE_DIGITAL_OUTPUT=false`. MWIR remains disabled. MQ-2 support is
compiled but disabled for the current hardware (`MQ2_SENSOR_ENABLED=false`);
its legacy input is GPIO35 if it is installed later. The
pump tube is fixed straight ahead at heading 0°;
there is no nozzle servo. When a flame is found between 5° and 175°, the robot
turns in place until the detecting flame servo reaches the fixed-tube heading.
At 0–5° both flame servos are held toward the fire and the relay can spray;
at 175–180° the robot turns around before spraying. A confirmed fire starts the
pump relay immediately (`PUMP_ON_CONFIRMED_FIRE=true`).

After the coverage planner has visited every reachable cell currently marked
FREE, it saves the map, stops the motors, pump and radar, then enters periodic
ESP32 light sleep. The controller wakes briefly once per second for safety and
telemetry processing. If MQ-2 hardware is enabled, a confirmed gas alarm exits
standby and restarts autonomous operation; otherwise a reset/power cycle is
required to begin another coverage pass.

## Build and upload

```powershell
& "C:\Users\Admin\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM4
```

Update `COM4` if Windows assigns the board a different serial port.
