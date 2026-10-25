// odometry.cpp - 位置追蹤系統實作
#include "odometry.h"
#include "vex.h"

using namespace vex;

// 全域變數定義
RobotPose robot_pose;
vex::task* odometry_task_handle = nullptr;
std::vector<Waypoint> recordedWaypoints;
bool isRecording = false;
double totalDistanceTraveled = 0.0;

// 常數定義
const double PI = 3.14159265358979323846;
const double WHEEL_DIAM_IN = 3.25;
const double EXT_GEAR_RATIO = 0.75;
const double WHEEL_CIRC_IN = PI * WHEEL_DIAM_IN;
const double DEG_TO_INCHES = (EXT_GEAR_RATIO * WHEEL_CIRC_IN) / 360.0;
const double MIN_DRIVE_VELOCITY = 3.5;
const double ROBOT_WIDTH_IN = 12.5;
const double ROBOT_LENGTH_IN = 16.0;
const double TRACK_WIDTH_IN = 10;

// 編碼器相關變數（用於錄製）
static double start_left_encoder = 0.0;
static double start_right_encoder = 0.0;
static double start_x = 0.0;
static double start_y = 0.0;
static double start_heading_value = 0.0;
static double encoder_based_x = 0.0;
static double encoder_based_y = 0.0;
static double encoder_based_heading = 0.0;
static double prev_record_left = 0.0;
static double prev_record_right = 0.0;

// IMU 偏移量
static double imu_heading_offset = 0.0;

// 馬達物件宣告（在 main.cpp 中定義）
extern motor L1, L2, L3, R1, R2, R3;
extern inertial Inertial;

// Waypoint 建構函式
Waypoint::Waypoint() : x(0.0), y(0.0), heading(0.0), dist_to_target(0.0) {}
Waypoint::Waypoint(double x_, double y_, double h_, double d_) : 
  x(x_), y(y_), heading(h_), dist_to_target(d_) {}

// ========== 數學輔助函式 ==========
double clampd(double v, double lo, double hi) { 
  return v < lo ? lo : (v > hi ? hi : v); 
}

double sgn(double x) { 
  return (x >= 0) ? 1.0 : -1.0; 
}

double wrap180(double a) { 
  while(a > 180) a -= 360; 
  while(a < -180) a += 360; 
  return a; 
}

// ========== 編碼器讀取函式 ==========
double get_left_inches() {
  double l_deg = (L1.position(vex::deg) + L2.position(vex::deg) + L3.position(vex::deg)) / 3.0;
  return l_deg * DEG_TO_INCHES;
}

double get_right_inches() {
  double r_deg = (R1.position(vex::deg) + R2.position(vex::deg) + R3.position(vex::deg)) / 3.0;
  return r_deg * DEG_TO_INCHES;
}

double get_avg_inches() {
  double l_deg = std::fabs(L1.position(vex::deg) + L2.position(vex::deg) + L3.position(vex::deg)) / 3.0;
  double r_deg = std::fabs(R1.position(vex::deg) + R2.position(vex::deg) + R3.position(vex::deg)) / 3.0;
  return (l_deg + r_deg) * 0.5 * DEG_TO_INCHES;
}

double get_encoder_heading_delta(double left_inches, double right_inches) {
  double delta_heading_rad = (right_inches - left_inches) / TRACK_WIDTH_IN;
  return delta_heading_rad * 180.0 / PI;
}

// ========== 位置追蹤核心函式 ==========
void set_robot_pose(double x, double y, double heading) {
  // 正規化航向到 0-360
  while(heading < 0.0) heading += 360.0;
  while(heading >= 360.0) heading -= 360.0;
  
  robot_pose.x = x;
  robot_pose.y = y;
  robot_pose.heading = heading;
  
  // 計算 IMU 偏移量
  vex::wait(50, vex::msec);
  double raw_imu_now = Inertial.heading();
  imu_heading_offset = raw_imu_now - heading;
  
  // 正規化偏移量到 0-360
  while(imu_heading_offset < 0.0) imu_heading_offset += 360.0;
  while(imu_heading_offset >= 360.0) imu_heading_offset -= 360.0;
  
  // 除錯資訊
  Brain.Screen.printAt(10, 160, "OFFSET SET: Raw=%.0f Want=%.0f Off=%.0f", 
                       raw_imu_now, heading, imu_heading_offset);
}

RobotPose get_robot_pose() {
  return robot_pose;
}

void update_robot_pose(double delta_left, double delta_right, double current_heading) {
  // 計算平均移動距離
  double delta_center = (delta_left + delta_right) / 2.0;
  
  // 檢測原地旋轉
  double abs_left = std::fabs(delta_left);
  double abs_right = std::fabs(delta_right);
  bool is_turning_in_place = (delta_left * delta_right < 0) &&  // 相反符號
                              (std::fabs(abs_left - abs_right) < std::max(abs_left, abs_right) * 0.3);
  
  // 如果是原地旋轉，只更新航向
  if(is_turning_in_place && std::fabs(delta_center) < 0.1) {
    while(current_heading < 0.0) current_heading += 360.0;
    while(current_heading >= 360.0) current_heading -= 360.0;
    robot_pose.heading = current_heading;
    return;
  }
  
  // 正規化航向
  while(current_heading < 0.0) current_heading += 360.0;
  while(current_heading >= 360.0) current_heading -= 360.0;
  
  // 使用三角函數更新 X/Y 位置
  double heading_rad = current_heading * PI / 180.0;
  robot_pose.x += delta_center * std::sin(heading_rad);
  robot_pose.y -= delta_center * std::cos(heading_rad);
  robot_pose.heading = current_heading;
}

static void set_drive_volt(double leftV, double rightV) {
  L1.spin(vex::fwd, leftV, vex::volt); 
  L2.spin(vex::fwd, leftV, vex::volt); 
  L3.spin(vex::fwd, leftV, vex::volt);
  R1.spin(vex::fwd, rightV, vex::volt); 
  R2.spin(vex::fwd, rightV, vex::volt); 
  R3.spin(vex::fwd, rightV, vex::volt);
}

// ========== 里程計背景任務 ==========
int odometry_task() {
  double prev_left = 0.0;
  double prev_right = 0.0;
  bool encoders_reset = false;
  
  while(true) {
    double curr_left = get_left_inches();
    double curr_right = get_right_inches();
    
    // 檢測編碼器是否被重置
    if(encoders_reset || (std::fabs(curr_left) < 0.1 && std::fabs(curr_right) < 0.1 && 
                          (std::fabs(prev_left) > 1.0 || std::fabs(prev_right) > 1.0))) {
      prev_left = curr_left;
      prev_right = curr_right;
      encoders_reset = false;
      continue;
    }
    
    // 計算變化量
    double delta_left = curr_left - prev_left;
    double delta_right = curr_right - prev_right;
    
    // 安全檢查：避免過大的變化量
    if(std::fabs(delta_left) > 10.0 || std::fabs(delta_right) > 10.0) {
      prev_left = curr_left;
      prev_right = curr_right;
      continue;
    }
    
    // 更新總移動距離
    double delta_dist = (std::fabs(delta_left) + std::fabs(delta_right)) / 2.0;
    totalDistanceTraveled += delta_dist;
    
    // 讀取並調整 IMU 航向
    double raw_imu = Inertial.heading();
    double adjusted_heading = raw_imu - imu_heading_offset;
    
    // 正規化到 0-360
    while(adjusted_heading < 0.0) adjusted_heading += 360.0;
    while(adjusted_heading >= 360.0) adjusted_heading -= 360.0;
    
    // 更新機器人位置
    update_robot_pose(delta_left, delta_right, adjusted_heading);
    
    // 儲存當前值供下一次使用
    prev_left = curr_left;
    prev_right = curr_right;
    
    // 顯示當前位姿（僅在錄製時）
    if(isRecording) {
      Brain.Screen.setPenColor(vex::white);
      Brain.Screen.printAt(10, 20, false, "X: %.2f  Y: %.2f", robot_pose.x, robot_pose.y);
      Brain.Screen.printAt(10, 40, false, "Heading: %.1f  Dist: %.2f", robot_pose.heading, totalDistanceTraveled);
      Brain.Screen.printAt(10, 60, false, "Points: %d", (int)recordedWaypoints.size());
    }
    
    vex::wait(10, vex::msec);
  }
  return 0;
}

void start_odometry_task() {
  if(odometry_task_handle != nullptr) {
    odometry_task_handle->stop();
    delete odometry_task_handle;
    odometry_task_handle = nullptr;
  }
  odometry_task_handle = new vex::task(odometry_task);
}

// ========== 路徑錄製系統 ==========
void startRecording(double start_x_val, double start_y_val, double start_heading_val) {
  // 重置所有狀態
  recordedWaypoints.clear();
  totalDistanceTraveled = 0.0;
  
  // 重置編碼器
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  vex::wait(50, vex::msec);
  
  // 儲存起始狀態
  start_left_encoder = get_left_inches();
  start_right_encoder = get_right_inches();
  start_x = start_x_val;
  start_y = start_y_val;
  start_heading_value = start_heading_val;
  
  // 初始化編碼器位姿追蹤
  encoder_based_x = start_x_val;
  encoder_based_y = start_y_val;
  encoder_based_heading = start_heading_val;
  prev_record_left = start_left_encoder;
  prev_record_right = start_right_encoder;
  
  // 設定起始位置
  set_robot_pose(start_x_val, start_y_val, start_heading_val);
  
  // 記錄起始路徑點
  recordedWaypoints.push_back(Waypoint(start_x_val, start_y_val, start_heading_val, 0.0));
  
  // 開始錄製
  isRecording = true;
  
  // 啟動里程計任務
  start_odometry_task();
  
  // 顯示狀態
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 100, "RECORDING PATH...");
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 120, "A: Record waypoint");
  Brain.Screen.printAt(10, 140, "B: Finish recording");
}

void recordWaypoint() {
  if(!isRecording) return;
  
  // 獲取當前編碼器位置
  double curr_left = get_left_inches();
  double curr_right = get_right_inches();
  
  // 計算變化量
  double delta_left = curr_left - prev_record_left;
  double delta_right = curr_right - prev_record_right;
  
  // 獲取當前航向
  double raw_imu = Inertial.heading();
  double adjusted_heading = raw_imu - imu_heading_offset;
  while(adjusted_heading < 0.0) adjusted_heading += 360.0;
  while(adjusted_heading >= 360.0) adjusted_heading -= 360.0;
  
  // 更新編碼器位姿
  double delta_center = (delta_left + delta_right) / 2.0;
  double abs_left = std::fabs(delta_left);
  double abs_right = std::fabs(delta_right);
  bool is_turning_in_place = (delta_left * delta_right < 0) && 
                              (std::fabs(abs_left - abs_right) < std::max(abs_left, abs_right) * 0.3);
  
  if(is_turning_in_place && std::fabs(delta_center) < 0.1) {
    encoder_based_heading = adjusted_heading;
  } else {
    double heading_rad = adjusted_heading * PI / 180.0;
    encoder_based_x += delta_center * std::sin(heading_rad);
    encoder_based_y -= delta_center * std::cos(heading_rad);
    encoder_based_heading = adjusted_heading;
  }
  
  // 更新總移動距離
  totalDistanceTraveled += (std::fabs(delta_left) + std::fabs(delta_right)) / 2.0;
  
  // 儲存路徑點
  recordedWaypoints.push_back(Waypoint(encoder_based_x, encoder_based_y, 
                                        encoder_based_heading, totalDistanceTraveled));
  
  // 更新前一次編碼器值
  prev_record_left = curr_left;
  prev_record_right = curr_right;
  
  // 控制器震動回饋
  Controller1.rumble(".");
  vex::wait(50, vex::msec);
  
  // 視覺回饋
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 180, false, "Waypoint %d: X=%.1f Y=%.1f H=%.1f", 
                       (int)recordedWaypoints.size(), encoder_based_x, 
                       encoder_based_y, encoder_based_heading);
  Brain.Screen.setPenColor(vex::white);
}

void stopRecording() {
  if(!isRecording) return;
  
  isRecording = false;
  
  // 最終回饋
  Controller1.rumble("-");
  vex::wait(100, vex::msec);
  
  // 視覺回饋
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 20, "RECORDING COMPLETE!");
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 40, "Total waypoints: %d", (int)recordedWaypoints.size());
  
  vex::wait(1, vex::sec);
  
  // 列印錄製的路徑
  printRecordedPath();
}

void printRecordedPath() {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::cyan);
  Brain.Screen.printAt(10, 20, "RECORDED PATH:");
  Brain.Screen.setPenColor(vex::white);
  
  int line = 40;
  for(size_t i = 0; i < recordedWaypoints.size() && i < 10; i++) {
    Waypoint& wp = recordedWaypoints[i];
    Brain.Screen.printAt(10, line, false, "%d: X=%.1f Y=%.1f H=%.0f D=%.1f", 
                         (int)i, wp.x, wp.y, wp.heading, wp.dist_to_target);
    line += 20;
  }
  
  // 輸出到控制台
  printf("\n========== RECORDED PATH ==========\n");
  printf("// Total waypoints: %d\n", (int)recordedWaypoints.size());
  printf("// Format: X, Y, Heading, Distance\n\n");
  
  for(size_t i = 0; i < recordedWaypoints.size(); i++) {
    Waypoint& wp = recordedWaypoints[i];
    printf("// Waypoint %d\n", (int)i);
    printf("const double WP%d_X = %.2f;\n", (int)i, wp.x);
    printf("const double WP%d_Y = %.2f;\n", (int)i, wp.y);
    printf("const double WP%d_H = %.1f;\n", (int)i, wp.heading);
    printf("const double WP%d_D = %.2f;\n\n", (int)i, wp.dist_to_target);
  }
  
  printf("========== END RECORDED PATH ==========\n\n");
}

void recordPath() {
  const double START_X = 22.0;
  const double START_Y = 22.5;
  const double START_HEADING = Inertial.heading();
  
  startRecording(START_X, START_Y, START_HEADING);
  
  // 顯示指令
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("PATH RECORDING MODE");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Drive manually");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("A=Save B=Finish");
  
  bool lastButtonA = false;
  bool lastButtonB = false;
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("RECORDING...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("A=Save B=Stop");
  
  while(isRecording) {
    // 啟動手動駕駛（需要底盤控制）
    chassis.control_tank(100); // 此函式在 main.cpp 中定義
    
    // 按鈕 A：記錄路徑點
    bool currentButtonA = Controller1.ButtonA.pressing();
    if(currentButtonA && !lastButtonA) {
      recordWaypoint();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Saved! Pts:%d", (int)recordedWaypoints.size());
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("A=Save B=Stop");
      vex::wait(250, vex::msec);
    }
    lastButtonA = currentButtonA;
    
    // 按鈕 B：結束錄製
    bool currentButtonB = Controller1.ButtonB.pressing();
    if(currentButtonB && !lastButtonB) {
      stopRecording();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("STOPPED!");
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Total:%d pts", (int)recordedWaypoints.size());
    }
    lastButtonB = currentButtonB;
    
    vex::wait(20, vex::msec);
  }
  
  vex::wait(10, vex::sec);
}

// ========== 移動與導航函式 ==========
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV) {
  double D = std::fabs(distance_in);
  if (D <= 0.0) return;

  // 重置所有馬達位置
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  L1.setStopping(vex::brake); L2.setStopping(vex::brake); L3.setStopping(vex::brake);
  R1.setStopping(vex::brake); R2.setStopping(vex::brake); R3.setStopping(vex::brake);

  const double dir   = sgn(distance_in);
  const double Vmax  = clampd(drive_maxV, 0.0, 12.0);
  const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);

  // 速度曲線參數
  double acc = D * 0.35, dec = D * 0.45;
  if (acc + dec > D) { double s = D / (acc + dec); acc *= s; dec *= s; }
  const double cruise = std::max(0.0, D - acc - dec);

  const int    dt_ms = 10;
  const double kH    = 0.17;
  const double kHi   = 0.003;
  const double i_cap = turn_maxV * 0.5;
  const double settle_in = 0.30;

  double h_i = 0.0;
  double last_s = 0.0;
  int    settle_timer = 0;
  const double trimV = 0.0;

  while (true) {
    double s = get_avg_inches();
    if (s >= D) break;
    
    // 計算速度（餘弦曲線）
    double v;
    if (s < acc) {
      double x = s / std::max(1e-6, acc);
      v = Vmax * 0.5 * (1.0 - std::cos(PI * x));
    } else if (s < acc + cruise) {
      v = Vmax;
    } else {
      double x = (s - (D - dec)) / std::max(1e-6, dec);
      v = Vmax * 0.5 * (1.0 + std::cos(PI * x));
    }
    v = std::max(v, MIN_V) * dir;

    // 航向控制（PID）
    double h_err = wrap180(angle_deg - Inertial.heading());
    h_i += h_err * (dt_ms / 1000.0) * kHi;
    h_i  = clampd(h_i, -i_cap, i_cap);
    double turnV = clampd(kH * h_err + h_i, -turn_maxV, turn_maxV);

    // 計算馬達電壓
    double leftV  = clampd(v + turnV - trimV, -Vmax, Vmax);
    double rightV = clampd(v - turnV + trimV, -Vmax, Vmax);
    set_drive_volt(leftV, rightV);

    // 穩定檢測
    double ds = std::fabs(s - last_s);
    if ((D - s) <= settle_in && ds < 0.02) {
      settle_timer += dt_ms;
      if (settle_timer >= 150) break;
    } else {
      settle_timer = 0;
    }
    last_s = s;

    vex::wait(dt_ms, vex::msec);
  }

  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

double cosineVelocity(double traveled, double totalDistance, double Vmax) {
  double acc = totalDistance * 0.20;
  double dec = totalDistance * 0.40;
  double cruise = std::max(0.0, totalDistance - acc - dec);
  
  const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);
  
  if(traveled < acc) {
    double x = traveled / std::max(1e-6, acc);
    return Vmax * 0.5 * (1.0 - std::cos(PI * x));
  } else if(traveled < acc + cruise) {
    return Vmax;
  } else {
    double x = (traveled - (totalDistance - dec)) / std::max(1e-6, dec);
    double v = Vmax * 0.5 * (1.0 + std::cos(PI * x));
    double remaining = totalDistance - traveled;
    if(remaining > 1.0) {
      v = std::max(v, Vmax * 0.3);
    }
    return std::max(v, 0.0);
  }
}

void cos_move_distance_fused(double distance_in, double angle_deg, double turn_maxV, double drive_maxV) {
  double D = std::fabs(distance_in);
  if (D <= 0.0) return;

  // 重置所有馬達位置
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  L1.setStopping(vex::coast); L2.setStopping(vex::coast); L3.setStopping(vex::coast);
  R1.setStopping(vex::coast); R2.setStopping(vex::coast); R3.setStopping(vex::coast);

  const double dir   = sgn(distance_in);
  const double Vmax  = clampd(drive_maxV, 0.0, 12.0);
  const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);

  double acc = D * 0.35, dec = D * 0.45;
  if (acc + dec > D) { double s = D / (acc + dec); acc *= s; dec *= s; }
  const double cruise = std::max(0.0, D - acc - dec);

  const int    dt_ms = 10;
  const double kH    = 0.17;
  const double kHi   = 0.003;
  const double i_cap = turn_maxV * 0.5;
  const double settle_in = 0.30;

  double h_i = 0.0;
  double last_s = 0.0;
  int    settle_timer = 0;
  const double trimV = 0.0;

  // 感測器融合相關
  double prev_left = 0.0;
  double prev_right = 0.0;
  double raw_imu_init = Inertial.heading();
  double fused_heading = raw_imu_init - imu_heading_offset;
  while(fused_heading < 0.0) fused_heading += 360.0;
  while(fused_heading >= 360.0) fused_heading -= 360.0;

  while (true) {
    double s = get_avg_inches();
    if (s >= D) break;

    // 計算速度
    double v = cosineVelocity(s, D, Vmax) * dir;

    // ========== 感測器融合 ==========
    double curr_left = get_left_inches();
    double curr_right = get_right_inches();
    double delta_left = curr_left - prev_left;
    double delta_right = curr_right - prev_right;
    double encoder_heading_delta = get_encoder_heading_delta(delta_left, delta_right);
    
    double raw_imu = Inertial.heading();
    double adjusted_imu = raw_imu - imu_heading_offset;
    while(adjusted_imu < 0.0) adjusted_imu += 360.0;
    while(adjusted_imu >= 360.0) adjusted_imu -= 360.0;
    
    fused_heading += encoder_heading_delta;
    while(fused_heading < 0.0) fused_heading += 360.0;
    while(fused_heading >= 360.0) fused_heading -= 360.0;
    
    double diff = wrap180(adjusted_imu - fused_heading);
    fused_heading += diff * 0.03;
    while(fused_heading < 0.0) fused_heading += 360.0;
    while(fused_heading >= 360.0) fused_heading -= 360.0;
    
    update_robot_pose(delta_left, delta_right, fused_heading);
    prev_left = curr_left;
    prev_right = curr_right;

    // 航向控制
    double target_err = wrap180(angle_deg - fused_heading);
    double real_err = target_err;
    double h_err = (target_err + real_err) / 2.0;
    
    h_i += h_err * (dt_ms / 1000.0) * kHi;
    h_i  = clampd(h_i, -i_cap, i_cap);
    double turnV = clampd(kH * h_err + h_i, -turn_maxV, turn_maxV);

    double leftV  = clampd(v + turnV - trimV, -Vmax, Vmax);
    double rightV = clampd(v - turnV + trimV, -Vmax, Vmax);
    set_drive_volt(leftV, rightV);

    // 穩定檢測
    double ds = std::fabs(s - last_s);
    if ((D - s) <= settle_in && ds < 0.02) {
      settle_timer += dt_ms;
      if (settle_timer >= 150) break;
    } else {
      settle_timer = 0;
    }
    last_s = s;

    vex::wait(dt_ms, vex::msec);
  }

  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

void driveToXY(double targetX, double targetY, double maxV, double turnMaxV) {
  double dx = targetX - robot_pose.x;
  double dy = targetY - robot_pose.y;
  double targetHeading = std::atan2(dx, -dy) * 180.0 / PI;
  
  // 正規化到 0-360
  while(targetHeading < 0.0) targetHeading += 360.0;
  while(targetHeading >= 360.0) targetHeading -= 360.0;
  
  // 轉向面對目標（需要底盤的 turn_to_angle 函式）
  chassis.turn_to_angle(targetHeading);
  vex::wait(50, vex::msec);
  
  // 計算距離並移動（需要底盤的 drive_distance 函式）
  double distance = std::sqrt(dx * dx + dy * dy);
  chassis.drive_distance(distance, targetHeading, maxV, turnMaxV);
}

void turnToXY(double targetX, double targetY, double turnMaxV) {
  const double settle_error = 2.0;
  const int dt_ms = 10;
  int settle_timer = 0;
  int loop_count = 0;
  const int MAX_LOOPS = 1000;
  
  double prev_heading = robot_pose.heading;
  int no_turn_timer = 0;
  const double MIN_TURN_THRESHOLD = 0.5;
  
  double h_i = 0.0;
  double prev_error = 0.0;
  const double kH = 0.07;
  const double kHi = 0.001;
  const double kHd = 0.08;
  const double i_cap = turnMaxV * 0.25;
  
  L1.setStopping(vex::brake); L2.setStopping(vex::brake); L3.setStopping(vex::brake);
  R1.setStopping(vex::brake); R2.setStopping(vex::brake); R3.setStopping(vex::brake);
  
  while(true) {
    loop_count++;
    
    if(loop_count > MAX_LOOPS) {
      set_drive_volt(0, 0);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("TURN TIMEOUT");
      break;
    }
    
    double dx = targetX - robot_pose.x;
    double dy = targetY - robot_pose.y;
    double targetHeading = std::atan2(dx, -dy) * 180.0 / PI;
    
    while(targetHeading < 0.0) targetHeading += 360.0;
    while(targetHeading >= 360.0) targetHeading -= 360.0;
    
    double current_heading = robot_pose.heading;
    while(current_heading < 0.0) current_heading += 360.0;
    while(current_heading >= 360.0) current_heading -= 360.0;
    
    double headingError = wrap180(targetHeading - current_heading);
    
    // 檢測是否在轉動
    double heading_change = std::fabs(wrap180(current_heading - prev_heading));
    
    if(loop_count % 20 == 0) {
      if(heading_change < MIN_TURN_THRESHOLD && std::fabs(headingError) > 3.0) {
        no_turn_timer += 200;
        if(no_turn_timer >= 500) {
          set_drive_volt(0, 0);
          Controller1.Screen.clearScreen();
          Controller1.Screen.setCursor(1, 1);
          Controller1.Screen.print("STOP: NO TURN");
          Controller1.Screen.setCursor(2, 1);
          Controller1.Screen.print("Err:%.1f H:%.1f", headingError, current_heading);
          vex::wait(500, vex::msec);
          break;
        }
      } else {
        no_turn_timer = 0;
      }
      prev_heading = current_heading;
    }
    
    // 除錯顯示
    static int debug_counter = 0;
    if(debug_counter % 20 == 0) {
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("T:%.0f N:%.0f E:%.0f", targetHeading, current_heading, headingError);
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("dx:%.1f dy:%.1f", dx, dy);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("X:%.1f Y:%.1f", robot_pose.x, robot_pose.y);
    }
    debug_counter++;
    
    // 穩定檢測
    double abs_error = std::fabs(headingError);
    if(abs_error < settle_error) {
      settle_timer += dt_ms;
      if(settle_timer >= 150) {
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("SETTLED");
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("E:%.1f", headingError);
        break;
      }
    } else {
      settle_timer = 0;
    }
    
    // PID 計算
    double p_term = kH * headingError;
    h_i += headingError * (dt_ms / 1000.0) * kHi;
    h_i = clampd(h_i, -i_cap, i_cap);
    double turnV = clampd(p_term + h_i, -turnMaxV, turnMaxV);
    
    // 接近目標時減少功率
    if(std::fabs(headingError) < 5.0) {
      double scale = std::fabs(headingError) / 5.0;
      scale = std::max(0.3, scale);
      turnV *= scale;
    }
    
    // 非常接近時限制功率
    if(std::fabs(headingError) < 2.0) {
      turnV = clampd(turnV, -0.4, 0.4);
    }
    
    if(debug_counter % 20 == 0) {
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("V:%.2f", turnV);
    }
    
    set_drive_volt(turnV, -turnV);
    vex::wait(dt_ms, vex::msec);
  }
  
  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

// ========== 在指定航向下驅動至目標 X 座標 ==========
// 驅動到目標 X 座標，同時保持特定的航向
// 計算出能夠獲得所需航向的 Y 座標
void driveToXAtHeading(double targetX, double targetHeading, double maxV, double turnMaxV) {
    // 計算能夠獲得所需航向的 Y 座標
    // heading = atan2(dx, -dy) * 180 / PI
    // 所以: tan(heading * PI / 180) = dx / (-dy)
    // 所以: -dy = dx / tan(heading * PI / 180)
    // 所以: dy = -dx / tan(heading * PI / 180)
    // 所以: targetY = robot_pose.y + dy = robot_pose.y - dx / tan(heading * PI / 180)
    
    double dx = targetX - robot_pose.x;
    double heading_rad = targetHeading * PI / 180.0;
    
    // 處理垂直航向（90° 或 270°），此時 tan 是未定義的
    if(std::fabs(std::cos(heading_rad)) < 0.001) {
        // 航向大約是 90° 或 270°（純粹在 Y 方向移動）
        // 在這種情況下，我們無法在保持此航向的同時到達特定的 X
        // 只需以該航向直線行駛
        double distance = std::fabs(dx);  // 使用合理的距離
        chassis.turn_to_angle(targetHeading);
        vex::wait(50, vex::msec);
        chassis.drive_distance(distance, targetHeading, maxV, turnMaxV);
        return;
    }
    
    // 計算目標 Y 座標
    double dy = -dx / std::tan(heading_rad);
    double targetY = robot_pose.y + dy;
    
    // 現在驅動到計算出的 X,Y 座標
    driveToXY(targetX, targetY, maxV, turnMaxV);
}

// ========== 向後驅動至目標 XY 座標 ==========
// 驅動到目標 (X, Y) 座標，同時保持當前航向
// 向後移動（負距離）以到達目標
// 適用於倒車進入目標或向後移動同時面向特定方向
void driveToXYBackward(double targetX, double targetY, double maxV, double turnMaxV) {
    // ========== 步驟 1: 計算到目標的距離 ==========
    double dx = targetX - robot_pose.x;
    double dy = targetY - robot_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // ========== 步驟 2: 獲取當前航向 ==========
    // 使用當前機器人航向 - 不要改變它
    double currentHeading = robot_pose.heading;
    
    // ========== 步驟 3: 向後驅動到目標 ==========
    // 向後驅動（負距離）同時保持當前航向
    chassis.drive_distance(-distance, currentHeading, maxV, turnMaxV);
}

// ========== 測試與顯示函式 ==========
void test_update_robot_pose() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("TESTING...");
  wait(0.5, sec);
  
  // 儲存當前位姿
  RobotPose saved_pose = robot_pose;
  
  // 測試 1: 在 0° 向上移動 10"
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 0.0;
  update_robot_pose(10.0, 10.0, 0.0);
  double expected_x_0 = 0.0;
  double expected_y_0 = -10.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T1: 0deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_0, expected_y_0);
  wait(2, sec);
  
  // 測試 2: 在 90° 向右移動 10"
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 90.0;
  update_robot_pose(10.0, 10.0, 90.0);
  double expected_x_90 = 10.0;
  double expected_y_90 = 0.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T2: 90deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_90, expected_y_90);
  wait(2, sec);
  
  // 測試 3: 在 180° 向下移動 10"
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 180.0;
  update_robot_pose(-10.0, -10.0, 180.0);
  double expected_x_180 = 0.0;
  double expected_y_180 = 10.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T3: 180deg -10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_180, expected_y_180);
  wait(2, sec);
  
  // 測試 4: 在 270° 向左移動 10"
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 270.0;
  update_robot_pose(-10.0, -10.0, 270.0);
  double expected_x_270 = -10.0;
  double expected_y_270 = 0.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T4: 270deg -10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_270, expected_y_270);
  wait(2, sec);
  
  // 測試 5: 在 45° 對角線移動
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 45.0;
  update_robot_pose(10.0, 10.0, 45.0);
  double expected_x_45 = 10.0 * std::sin(45.0 * PI / 180.0);
  double expected_y_45 = -10.0 * std::cos(45.0 * PI / 180.0);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T5: 45deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_45, expected_y_45);
  wait(2, sec);
  
  // 測試 6: 實際編碼器測試
  double curr_l = get_left_inches();
  double curr_r = get_right_inches();
  double delta_l = curr_l;
  double delta_r = curr_r;
  double heading_now = robot_pose.heading;
  update_robot_pose(delta_l, delta_r, heading_now);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T6: ACTUAL");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("dL:%.2f dR:%.2f", delta_l, delta_r);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("H:%.1f X:%.2f Y:%.2f", heading_now, robot_pose.x, robot_pose.y);
  wait(2, sec);
  
  // 還原位姿
  robot_pose = saved_pose;
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("TEST COMPLETE");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Check results above");
}

void display_robot_pose() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("X:%.1f Y:%.1f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("H:%.1f", robot_pose.heading);
  
  static int debug_count = 0;
  debug_count++;
  if(debug_count % 5 == 0) {
    double l_inches = get_left_inches();
    double r_inches = get_right_inches();
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("L:%.1f R:%.1f", l_inches, r_inches);
  }
}

void display_robot_pose_brain() {
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 220, false, "Pose: X=%.1f Y=%.1f H=%.1f", 
                       robot_pose.x, robot_pose.y, robot_pose.heading);
}