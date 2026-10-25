// main.cpp - 完整的 JAR-Template 框架與使用者介面
#include "vex.h"
#include "note.h"
#include "odometry.h"  // 新增的位置追蹤系統標頭檔
#include <cmath>
#include <algorithm>

using namespace vex;

// ========== 全域變數宣告 ==========
extern vex::color selectedTeamColor;
competition Competition;
;

// ========== 底盤驅動物件 ==========
Drive chassis(
    ZERO_TRACKER_ODOM,  // 追蹤模式
    motor_group(L1, L2, L3),  // 左側馬達群組
    motor_group(R1, R2, R3),  // 右側馬達群組
    PORT20,  // 慣性感測器連接埠
    3.25,    // 輪子直徑
    0.75,    // 齒輪比
    360,     // 陀螺儀比例
    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE!                                   */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
    /*  If you are not using position tracking, leave the rest of the values as  */
    /*  they are.                                                                */
    /*---------------------------------------------------------------------------*/
    // 位置追蹤參數
    PORT12, -PORT14,  // LF, RF
    PORT13, -PORT20,  // LB, RB
    5,                // 前追蹤器連接埠
    -3.25,            // 前追蹤器直徑
    5.2,              // 前追蹤器中距
    1,                // 側追蹤器連接埠
    -2.75,            // 側追蹤器直徑
    5.5               // 側追蹤器中距
);

// ========== 全域控制變數 ==========
int current_auton_selection = 0;
bool auto_started = false;
bool airspace = false;
bool ran_auton = false;  // 是否已經跑auto模式

// ========== 氣動控制函式 ==========
void wingSwitch() { wing = !wing; }

void cylinderSwitch() { intakeCylander = !intakeCylander; }

void intakecylanderon() {
  airspace = !airspace;
  intakeCylander = airspace;
}

void intakecylanderoff() { intakeCylander = false; }

void shooterSwitch() { shooter = !shooter; }

void shooterOn() { shooter = true; }

void shooterOff() { shooter = false; }

void pushSwitch() { pushCylinder = !pushCylinder; }

void pushON() { pushCylinder = true; }

void pushOFF() { pushCylinder = false; }

bool shooterPushOn = false;

void shooterPushSwitch() {
  shooterPushOn = !shooterPushOn;
  shooter = shooterPushOn;
  pushCylinder = shooterPushOn;
}

// ========== 光學感測器燈光控制任務 ==========
int opticalLightTask() {
  int lightPower = 100;
  Optical.setLightPower(lightPower, percent);
  
  while(true) {
    // 調整燈光亮度
    if(Controller1.ButtonUp.pressing()) {
      lightPower += 10;
      if(lightPower >= 100) lightPower = 100;
      Optical.setLightPower(lightPower, percent);
      wait(50, msec);
    } else if(Controller1.ButtonDown.pressing()) {
      lightPower -= 10;
      if(lightPower <= 0) lightPower = 0;
      Optical.setLightPower(lightPower, percent);
      wait(50, msec);
    }
    
    // 根據氣動狀態控制燈光
    if(lightPower > 0 && pushCylinder) {
      Optical.setLight(ledState::on);
    } else {
      Optical.setLight(ledState::off);
    }
    
    wait(50, msec);
  }
  return 0;
}

// ========== 儀表板繪製輔助結構與函式 ==========
struct Rect { int x, y, w, h; };

static inline void fillRect(const Rect& r, vex::color fill, vex::color pen = vex::transparent) {
  Brain.Screen.setFillColor(fill);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h);
}

static inline void strokeRect(const Rect& r, vex::color pen) {
  Brain.Screen.setFillColor(vex::transparent);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h, vex::transparent);
}

static inline void drawCenteredText(const Rect& r, const std::string& s, vex::color pen=vex::white) {
  Brain.Screen.setPenColor(pen);
  int tw = Brain.Screen.getStringWidth(s.c_str());
  int th = Brain.Screen.getStringHeight("A");
  int px = r.x + (r.w - tw) / 2;
  int py = r.y + (r.h + th) / 2;  // y 是 baseline
  Brain.Screen.printAt(px, py, false, s.c_str());
}

// 水平條（-100~100）＋內文字置中
static inline void drawAxisBarLabeled(const char* name, int val, const Rect& labelBox, const Rect& barBox) {
  // 左側標籤（置中）
  fillRect(labelBox, vex::color(30,30,30));
  strokeRect(labelBox, vex::color(80,80,80));
  drawCenteredText(labelBox, name, vex::white);

  // 條底
  strokeRect(barBox, vex::color(90,90,90));
  fillRect({barBox.x+1, barBox.y+1, barBox.w-2, barBox.h-2}, vex::color(20,20,20));

  // 0 中線
  int zeroX = barBox.x + barBox.w/2;
  Brain.Screen.setPenColor(vex::color(90,90,90));
  Brain.Screen.drawLine(zeroX, barBox.y+1, zeroX, barBox.y + barBox.h - 2);

  // 值條
  int v = val; if(v>100) v=100; if(v<-100) v=-100;
  int half = (barBox.w-2)/2;
  int pix = (v * half) / 100;
  if (pix != 0) {
    int bx = (pix>0) ? zeroX : (zeroX+pix);
    int bw = (pix>0) ? pix : -pix;
    fillRect({bx, barBox.y+2, bw, barBox.h-4}, (pix>0)?vex::color(0,140,220):vex::color(220,140,0));
  }

  // 內文字置中（顯示數值）
  char buf[16];
  snprintf(buf, sizeof(buf), "%4d", val);
  drawCenteredText(barBox, buf, vex::color(230,230,230));
}

// 按鍵方塊
static inline void drawButtonBox(const char* name, bool pressed, const Rect& r) {
  vex::color fill = pressed ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = pressed ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
}

// 小型指南針（顯示 heading 0~360）
static inline void drawCompass(const Rect& r, double heading_deg) {
  fillRect(r, vex::color(30,30,30)); strokeRect(r, vex::color(100,100,100));
  // 圓
  int cx = r.x + r.w/2, cy = r.y + r.h/2;
  int rad = (r.w<r.h ? r.w : r.h)/2 - 4;
  Brain.Screen.setPenColor(vex::color(160,160,160));
  Brain.Screen.drawCircle(cx, cy, rad);
  // N 標記
  Brain.Screen.printAt(cx-4, r.y+12, "N");
  // 指針（0 度朝上）
  double radian = (heading_deg - 90.0) * 3.1415926535 / 180.0;
  int tipX = cx + int(rad * std::cos(radian));
  int tipY = cy + int(rad * std::sin(radian));
  Brain.Screen.setPenColor(vex::color(255,80,80));
  Brain.Screen.drawLine(cx, cy, tipX, tipY);
  // 數值
  char buf[32]; snprintf(buf, sizeof(buf), "%3.0f°", heading_deg);
  drawCenteredText({r.x, r.y + r.h - 16, r.w, 16}, buf, vex::white);
}

// 小型 ON/OFF 方塊（氣動）
static inline void drawPneuBox(const char* name, bool on, const Rect& r) {
  vex::color fill = on ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = on ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
}

// ========== 分頁系統 ==========
enum DashTab { TAB_INPUTS=0, TAB_MOTORS=1 };

// 分頁列（右側顯示 Auton，不擋內容）
static inline DashTab drawTabs(DashTab current, const char* autonText) {
  Rect bar = {0,0,480,22}; // 矮一點，留內容空間
  fillRect(bar, vex::color(35,35,35)); strokeRect(bar, vex::color(90,90,90));

  Rect t1 = {8,  2, 92, 18};
  Rect t2 = {t1.x + t1.w + 4, 2, 92, 18};

  auto tabColor = [&](DashTab t){ return (t==current)?vex::color(60,100,180):vex::color(55,55,55); };
  fillRect(t1, tabColor(TAB_INPUTS));  strokeRect(t1, vex::color(120,120,120));
  fillRect(t2, tabColor(TAB_MOTORS));  strokeRect(t2, vex::color(120,120,120));
  drawCenteredText(t1, "Inputs", vex::white);
  drawCenteredText(t2, "Motors", vex::white);

  // 右側顯示 Auton
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::color(220,220,220));
  Brain.Screen.printAt(216, 16, false, "Auton: %s", autonText);

  DashTab out = current;
  if (Brain.Screen.pressing()) {
    int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
    if (y>=t1.y && y<=t1.y+t1.h) {
      if (x>=t1.x && x<=t1.x+t1.w) out = TAB_INPUTS;
      else if (x>=t2.x && x<=t2.x+t2.w) out = TAB_MOTORS;
    }
  }
  return out;
}

// 馬達監控陣列
static motor* kMotors[] = { &L1,&R1,&L2,&R2,&L3,&R3,&intake,&intakedown};
static const char* kMotorNames[] = { "L1","R1","L2","R2","L3","R3","INTK","IDWN"};
static const int kMotorCount = sizeof(kMotors)/sizeof(kMotors[0]);

// ========== 儀表板顯示函式 ==========
static inline void show_status_page(int selectedAuton) {
  while (Brain.Screen.pressing()) wait(10, msec);

  const char* labels[10] = {
    "R_right","R_left","R_right_F","R_left_F","R_solo",
    "B_right","B_left","B_right_F","B_left_F","B_solo"
  };

  DashTab tab = TAB_INPUTS;
  DashTab prevTab = TAB_INPUTS;
  Brain.Screen.setFont(vex::fontType::mono12);

  // 版面參數
  const int  MARGIN  = 8;
  const int  TAB_H   = 22;
  const Rect content = { MARGIN, TAB_H + MARGIN, 480 - 2*MARGIN, 240 - (TAB_H + 2*MARGIN) };

  // 各分頁初始化旗標（只在剛切入該分頁時做一次）
  bool inputsInit = false;
  bool motorsInit = false;

  // 先鋪一次全局底色
  fillRect({0,0,480,240}, vex::color(18,18,18));

  while (true) {
    // 先畫分頁列（不清整頁，避免把 LOGO 擦掉）
    tab = drawTabs(tab, labels[selectedAuton]);

    // 分頁切換：只清「內容區」並重置 init
    if (tab != prevTab) {
      fillRect(content, vex::color(18,18,18));
      strokeRect(content, vex::color(60,60,60));
      if (tab == TAB_INPUTS)  inputsInit = false;
      if (tab == TAB_MOTORS)  motorsInit = false;
      prevTab = tab;
    }

    if (tab == TAB_INPUTS) {
      // ===== Inputs 頁 =====
      // 第一次進來：畫 LOGO（之後不再從 SD 讀，避免閃爍）
      if (!inputsInit) {
        const int barH = 20;
        const int gapY = 8;
        const int axTop = content.y;

        // A4 條下面顯示 LOGO.bmp（240x240）
        const int imgX = 35;                                        // 你要的 X
        const int imgY = axTop + 3*(barH+gapY) + barH + 10;         // A4 底下 10px

        if (Brain.SDcard.isInserted() && Brain.SDcard.exists("LOGO.bmp")) {
          Brain.Screen.drawImageFromFile("LOGO.bmp", imgX, imgY);
        } else {
          Brain.Screen.setPenColor(vex::color(200,200,200));
          Brain.Screen.printAt(imgX, imgY + 20, false, "LOGO.bmp not found");
        }
        inputsInit = true;
      }

      // —— 每幀只更新動態元件（不清整頁、也不重畫 LOGO） ——
      const int labelW = 36;
      const int barW   = (int)(content.w * 0.55);
      const int barH   = 20;
      const int gapY   = 8;
      const int axLeft = content.x;
      const int axTop  = content.y;

      int a1 = Controller1.Axis1.position();
      int a2 = Controller1.Axis2.position();
      int a3 = Controller1.Axis3.position();
      int a4 = Controller1.Axis4.position();

      drawAxisBarLabeled("A1", a1, {axLeft,             axTop + 0*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 0*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A2", a2, {axLeft,             axTop + 1*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 1*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A3", a3, {axLeft,             axTop + 2*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 2*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A4", a4, {axLeft,             axTop + 3*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 3*(barH+gapY), barW,   barH});

      // 右上小指南針
      Rect compass = { content.x + content.w - 90, axTop, 80, 80 };
      drawCompass(compass, Inertial.heading(degrees));

      // 右側：按鍵 3×4
      int gridX = content.x + content.w - (3*70 + 2*6);
      int gridY = compass.y + compass.h + 6;
      int bw    = 70, bh = 20, sp = 6;

      drawButtonBox("A",   Controller1.ButtonA.pressing(),   {gridX + 0*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("B",   Controller1.ButtonB.pressing(),   {gridX + 1*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("X",   Controller1.ButtonX.pressing(),   {gridX + 2*(bw+sp), gridY + 0*(bh+sp), bw, bh});

      drawButtonBox("Y",   Controller1.ButtonY.pressing(),   {gridX + 0*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L1",  Controller1.ButtonL1.pressing(),  {gridX + 1*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L2",  Controller1.ButtonL2.pressing(),  {gridX + 2*(bw+sp), gridY + 1*(bh+sp), bw, bh});

      drawButtonBox("R1",  Controller1.ButtonR1.pressing(),  {gridX + 0*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("R2",  Controller1.ButtonR2.pressing(),  {gridX + 1*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("Up",  Controller1.ButtonUp.pressing(),  {gridX + 2*(bw+sp), gridY + 2*(bh+sp), bw, bh});

      drawButtonBox("Down", Controller1.ButtonDown.pressing(), {gridX + 0*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Left", Controller1.ButtonLeft.pressing(), {gridX + 1*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Right",Controller1.ButtonRight.pressing(),{gridX + 2*(bw+sp), gridY + 3*(bh+sp), bw, bh});
    
      // 底部：氣動狀態
      int pneuY = content.y + content.h - 20;
      int pneuW = 68, pneuH = 18, pneuSP = 6;
      // Center the 5 boxes: total width = 5*68 + 4*6 = 364, center offset = (content.w - 364) / 2
      int totalPneuWidth = 5 * pneuW + 4 * pneuSP;
      int pneuX = content.x + (content.w - totalPneuWidth) / 2;

      drawPneuBox("no status", redlight.value(),       {pneuX + 0*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("no status", whitelight.value(),     {pneuX + 1*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("INTK",      intakeCylander.value(), {pneuX + 2*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("PUSH",      pushCylinder.value(),   {pneuX + 3*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("SHOT",      shooter.value(),        {pneuX + 4*(pneuW+pneuSP), pneuY, pneuW, pneuH});
    }
    else {
      // ===== Motors 頁（放大字體 + 逐行清除避免陰影） =====
      if (!motorsInit) {
        // 清內容區一次、畫框與標題
        fillRect(content, vex::color(18,18,18));
        strokeRect({content.x, content.y, content.w, content.h - 2}, vex::color(90,90,90));
        Brain.Screen.setPenColor(vex::color(200,200,200));
        Brain.Screen.setFont(vex::fontType::mono20);    // 放大字體
        // 標題往下擺，避免貼到 tabs
        Brain.Screen.printAt(content.x, content.y + 4, "Motor position (deg)");
        motorsInit = true;
      }

      // 版面配置（兩欄）
      const int colW = (content.w - 16) / 2;
      const int c1x  = content.x + 8;
      const int c2x  = content.x + 8 + colW;

      // 行高比字高大些，整體下移
      const int rowH = 26;                 // 搭配 mono20
      int rowTop = content.y + 28;         // ★ 往下移，避免貼到標題與 tabs

      // 顏色
      vex::color bg = vex::color(18,18,18);
      vex::color fg = vex::white;

      // 逐行更新：先清該行，再用不透明文字印上，避免陰影
      for (int i = 0; i < kMotorCount; ++i) {
        int colX = (i % 2 == 0) ? c1x : c2x;
        int rowY = rowTop + (i / 2) * rowH;

        // 清這一行的小區塊（不影響其它區域）
        Rect rline = { colX, rowY - 18, colW - 8, rowH };
        fillRect(rline, bg);

        const char* name = (i < (int)(sizeof(kMotorNames)/sizeof(kMotorNames[0]))) ? kMotorNames[i] : "M?";
        double posDeg = 0.0;
        if (kMotors[i]) {
            // 注意：這裡使用的是 odometry.cpp 中的 deg_to_inches 函式
            posDeg = kMotors[i]->position(degrees);  // 原始版本使用馬達角度
        }
        Brain.Screen.setPenColor(fg);
        Brain.Screen.setFillColor(bg);
        Brain.Screen.printAt(colX, rowY, /*bOpaque=*/true, "%-6s %8.1f", name, posDeg);
      }
    }

    // ========== SPECIAL: If "Record" mode selected, show START button ==========
    if (selectedAuton == 6) {  // Case 6 = Record mode
      // Draw a big green START RECORDING button
      Rect startButton = { 140, 170, 200, 50 };
      fillRect(startButton, vex::color(0, 150, 0));
      strokeRect(startButton, vex::color(0, 255, 0));
      drawCenteredText(startButton, "START RECORDING", vex::white);
      
      // Check if START button pressed
      if (Brain.Screen.pressing()) {
        int tx = Brain.Screen.xPosition();
        int ty = Brain.Screen.yPosition();
        
        if (tx >= startButton.x && tx <= startButton.x + startButton.w &&
            ty >= startButton.y && ty <= startButton.y + startButton.h) {
          // Button pressed - start recording!
          while (Brain.Screen.pressing()) wait(10, msec);
          recordPath();  // Start recording immediately
          break;  // Exit dashboard after recording finishes
        }
      }
    }

    // 觸控返回（點 tabs 區域只切換，不返回）
    if (Brain.Screen.pressing()) {
      int ty = Brain.Screen.yPosition();
      if (!(ty >= 0 && ty <= TAB_H)) {
        while (Brain.Screen.pressing()) wait(10, msec);
        break;   // 回到 pre_auton 的選單頁
      }
    }

    wait(100, msec);
  }
}

// ========== 進球控制任務 ==========
int intakeControlTask() {
  intake.setMaxTorque(100, percent);

  while (true) {
    // 依優先權：L1 > R1
    if (Controller1.ButtonL1.pressing()) {
      // L1：正向轉動
      intake.spin(forward, 12, volt);
      intakedown.spin(forward, 12, volt);
    } else if (Controller1.ButtonR1.pressing()) {
      // R1：反向轉動
      intake.spin(reverse, 12, volt);
      intakedown.spin(reverse, 12, volt);
    } else {
      // 停止
      intake.stop(coast);
      intakedown.stop(coast);
    }

    wait(20, msec);
  }
  return 0;
}

// ========== 預自主函式（程式初始化與選擇介面） ==========
void pre_auton(void) {
  // 初始化 VEX 系統
  vexcodeInit();
  // default_constants();  // 如果有此函式的話

  // 校準慣性感測器
  Inertial.calibrate();
  while (Inertial.isCalibrating()) {
    whitelight = 0;
  }
  Controller1.Screen.print("ok!");
  redlight = 1;
  whitelight = 1;
  
  // CRITICAL: Reset motors RIGHT BEFORE dashboard
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  wait(0.05, sec);  // Small delay for reset to take effect

  // 顏色定義
  vex::color red   = vex::color::red;
  vex::color blue  = vex::color::blue;
  vex::color white = vex::color::white;

  int previous_selection = -1;

  // 介面佈局參數
  const int cols      = 5;
  const int screen_w  = 480;
  const int screen_h  = 240;
  const int col_width = screen_w / cols;
  const int red_height  = screen_h / 2 + 6;
  const int blue_height = screen_h - red_height;

  // 自動程式標籤
  const char* labels[10] = {
    "Right_43","Left_43","Right_7","Left_7","Solo",
    "Skills","Record","blank1","blank2","blank3"
  };

  Brain.Screen.setFont(vex::fontType::mono20);

  // 主選擇循環
  while (!auto_started) {
    if (current_auton_selection != previous_selection) {
      previous_selection = current_auton_selection;

      // 繪製所有選擇方塊
      for (int i = 0; i < 10; i++) {
        int col = i % cols;
        bool top = (i < cols);

        int x = col * col_width;
        int y = top ? 0 : red_height;
        int h = top ? red_height : blue_height;

        // 決定顏色：選中的白色，否則根據位置紅/藍
        vex::color fillColor = (current_auton_selection == i) ? white : (top ? red : blue);
        vex::color textColor = (current_auton_selection == i) ? red   : white;

        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.drawRectangle(x, y, col_width, h);

        Rect r { x, y, col_width, h };
        drawCenteredText(r, labels[i], textColor);
      }
    }

    // 處理觸控輸入
    if (Brain.Screen.pressing()) {
      int touchX = Brain.Screen.xPosition();
      int touchY = Brain.Screen.yPosition();

      int col = touchX / col_width;
      int row = (touchY < red_height) ? 0 : (touchY < red_height + blue_height ? 1 : -1);

      if (col >= 0 && col < cols && (row == 0 || row == 1)) {
        current_auton_selection = col + row * cols;
        
        // 進入第二頁（儀表板）
        show_status_page(current_auton_selection);
      }

      wait(0.3, sec);
    }

    wait(20, msec);
  }
}

// ========== 自主函式 ==========
void autonomous(void) {
  // CRITICAL: Start odometry task to continuously track robot position
  start_odometry_task();
  
  auto_started = true;
  ran_auton = true;
  
  // 根據選擇的自動任務來決定隊伍顏色
  if (current_auton_selection >= 0 && current_auton_selection <= 4) {
    selectedTeamColor = vex::color::red;  // 紅隊
  } else if (current_auton_selection >= 5 && current_auton_selection <= 9) {
    selectedTeamColor = vex::color::blue;  // 藍隊
  } else {
    selectedTeamColor = vex::color::black;  // 預設為黑隊
  }

  // 執行選擇的自動程式
  switch (current_auton_selection) {
    case 0: Right_43(); break;
    case 1: Left_43(); break;
    case 2: Right_7(); break;
    case 3: Left_7(); break;
    case 4: Solo(); break;
    case 5: Skills(); break;
    case 6: recordPath();  // Path recording mode
            break;
    case 7: blank1(); break;
    case 8: blank2(); break;
    case 9: blank3(); break;
  }
}

// ========== 使用者控制函式 ==========
void usercontrol(void) {
  // CRITICAL: Reset motor encoders when entering driver control
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  // Stop odometry task from autonomous (not needed for driver control)
  if(odometry_task_handle != nullptr) {
    odometry_task_handle->stop();
    delete odometry_task_handle;
    odometry_task_handle = nullptr;
  }
  
  // Start background tasks
  task notetask(autonoteTask, 0);
  task intake(intakeControlTask, 0);
  task opticalLight(opticalLightTask, 0);
  
  // Set up button callbacks
  Controller1.ButtonRight.pressed(shooterSwitch);
  Controller1.ButtonL2.pressed(wingSwitch);
  Controller1.ButtonB.pressed(pushSwitch);
  Controller1.ButtonY.pressed(shooterPushSwitch);
  Controller1.ButtonR2.pressed(cylinderSwitch);
  
  // Main driver control loop
  while (1) {
    chassis.control_tank(100);  // 底盤控制
    wait(20, msec);
  }
}

// ========== 主函式 ==========
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}