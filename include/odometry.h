// odometry.h - 位置追蹤系統標頭檔
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "vex.h"
#include <cmath>
#include <algorithm>
#include <vector>

// ========== 位置追蹤結構 ==========
/**
 * 機器人位姿結構，用於里程計追蹤
 * 儲存 X、Y 位置（英吋）和航向（度，0-360）
 */
struct RobotPose {
  double x;        // X座標（英吋）
  double y;        // Y座標（英吋）
  double heading;  // 航向（度，0-360）
  
  RobotPose() : x(0.0), y(0.0), heading(0.0) {}
  RobotPose(double x_, double y_, double h_) : x(x_), y(y_), heading(h_) {}
};

// 路徑點結構
struct Waypoint {
  double x;       // X座標
  double y;       // Y座標
  double heading; // 航向
  double dist_to_target; // 從起點的總移動距離
  
  Waypoint();
  Waypoint(double x_, double y_, double h_, double d_);
};

// 全域變數宣告（在 odometry.cpp 中定義）
extern RobotPose robot_pose;
extern vex::task* odometry_task_handle;
extern std::vector<Waypoint> recordedWaypoints;
extern bool isRecording;
extern double totalDistanceTraveled;

// 數學輔助函式
double clampd(double v, double lo, double hi);
double sgn(double x);
double wrap180(double a);

// 編碼器讀取函式
double get_left_inches();
double get_right_inches();
double get_avg_inches();
double get_encoder_heading_delta(double left_inches, double right_inches);

// ========== XY 導航函式 ==========
/**
 * 設定機器人起始位姿（位置和航向）
 * @param x X座標（英吋）
 * @param y Y座標（英吋）
 * @param heading 航向（度，0-360）
 */
void set_robot_pose(double x, double y, double heading);

/**
 * 獲取當前機器人位姿
 * @return RobotPose結構，包含當前 X、Y 和航向
 */
RobotPose get_robot_pose();

/**
 * 驅動到特定的絕對 X、Y 座標
 * @param targetX 目標 X 座標（英吋）
 * @param targetY 目標 Y 座標（英吋）
 * @param maxV 最大驅動電壓（0-12V）
 * @param turnMaxV 最大轉向電壓（0-12V）
 */
void driveToXY(double targetX, double targetY, double maxV, double turnMaxV);

/**
 * 在指定航向下驅動至目標 X 座標
 * 計算出能夠獲得所需航向的 Y 座標
 * @param targetX 目標 X 座標（英吋）
 * @param targetHeading 期望的航向（度，0° = -Y/上，90° = +X/右）
 * @param maxV 最大驅動電壓（0-12V）
 * @param turnMaxV 最大轉向電壓（0-12V）
 */
void driveToXAtHeading(double targetX, double targetHeading, double maxV, double turnMaxV);

/**
 * 向後驅動至目標 (X, Y) 座標，同時保持當前航向
 * 向後移動（負距離）以到達目標
 * 適用於倒車進入目標或向後移動同時面向特定方向
 * @param targetX 目標 X 座標（英吋）
 * @param targetY 目標 Y 座標（英吋）
 * @param maxV 最大驅動電壓（0-12V）
 * @param turnMaxV 最大轉向電壓（0-12V）
 */
void driveToXYBackward(double targetX, double targetY, double maxV, double turnMaxV);

/**
 * 轉向面對特定的絕對 X、Y 座標
 * @param targetX 目標 X 座標（英吋）
 * @param targetY 目標 Y 座標（英吋）
 * @param turnMaxV 最大轉向電壓（0-12V）
 */
void turnToXY(double targetX, double targetY, double turnMaxV);

/**
 * 平滑餘弦速度移動，帶距離和角度控制
 * @param distance_in 要行駛的距離（英吋）
 * @param angle_deg 目標角度（度）
 * @param turn_maxV 最大轉向電壓
 * @param drive_maxV 最大驅動電壓
 */
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

/**
 * 感測器融合移動（IMU + 編碼器）
 * @param distance_in 要行駛的距離（英吋）
 * @param angle_deg 目標角度（度）
 * @param turn_maxV 最大轉向電壓
 * @param drive_maxV 最大驅動電壓
 */
void cos_move_distance_fused(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

// ========== 里程計輔助函式 ==========
/**
 * 使用編碼器變化量和當前航向更新機器人位姿
 * @param delta_left 左編碼器變化量（英吋）
 * @param delta_right 右編碼器變化量（英吋）
 * @param current_heading 當前航向（度，0-360）
 */
void update_robot_pose(double delta_left, double delta_right, double current_heading);

/**
 * 獲取左側編碼器位置（英吋）
 * @return 左編碼器位置（英吋）
 */
double get_left_inches();

/**
 * 獲取右側編碼器位置（英吋）
 * @return 右編碼器位置（英吋）
 */
double get_right_inches();

/**
 * 測試函式，驗證 update_robot_pose 的計算
 * 在控制器螢幕上顯示測試結果
 */
void test_update_robot_pose();

/**
 * 背景里程計任務（持續執行）
 * 使用編碼器和 IMU 更新機器人位姿
 * @return 任務返回碼（始終為 0）
 */
int odometry_task();

/**
 * 啟動背景里程計任務
 */
void start_odometry_task();

// 路徑錄製系統
void startRecording(double start_x_val, double start_y_val, double start_heading_val);
void recordWaypoint();
void stopRecording();
void printRecordedPath();
void recordPath();

// 輔助函式
double cosineVelocity(double traveled, double totalDistance, double Vmax);

// 測試與顯示函式
void display_robot_pose();
void display_robot_pose_brain();

// 常數定義
extern const double PI;
extern const double WHEEL_DIAM_IN;
extern const double EXT_GEAR_RATIO;
extern const double WHEEL_CIRC_IN;
extern const double DEG_TO_INCHES;
extern const double MIN_DRIVE_VELOCITY;
extern const double ROBOT_WIDTH_IN;
extern const double ROBOT_LENGTH_IN;
extern const double TRACK_WIDTH_IN;

#endif // ODOMETRY_H