#include "vex.h"
#include "note.h"
#include "odometry.h"  // 新增的位置追蹤系統標頭檔"

void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

bool downTaskForward = true;  // true = forward, false = reverse
bool upTaskForward   = true;  // true = forward, false = reverse

void driveTime(int ms)
{
  chassis.drive_with_voltage(12, 12);
  wait(ms, msec);
  chassis.DriveL.stop(brake);
  chassis.DriveR.stop(brake);
}




int info()
{
  while (1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0, 50, "X: %f", chassis.get_X_position()); 
    Brain.Screen.printAt(0, 70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0, 90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0, 110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0, 130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
  return 1;
}

void Right_43()
{
  
  
  // ========== CORRECT COORDINATE MAP ==========
  // Field: 140.41" × 140.41" (0,0) is at top left corner, (140.41, 140.41) is at bottom right
  // Robot: 12.5" wide × 18" long (inertia at center: 6.25" from sides, 9" from front/back)
  // START: (88, 118.3) - Actual starting position on field (robot center)
  // Coordinate system: Top-left origin (0,0), 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
  
  
  // Set initial pose: X, Y, and heading (0° = up/forward, 90° = right, 180° = down, 270° = left)
  set_robot_pose(86, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 10, volt);
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(103, 105, 8.0, 6.0);
  wait(0.05, sec);
  intake.spin(forward, 10, volt);

  // ========== PHASE 2: Score 3 blocks ==========
  driveToXY(93.5, 93.5, 6.0, 6.0); // Approach slowly
  wait(0.05, sec);
  chassis.drive_distance(15, 315, 8.0, 6.0); // Drive to low goal
  intakedown.spin(reverse, 6, volt);
  intake.spin(reverse, 10, volt);
  wait(0.05, sec);
  chassis.drive_distance(4, 315, 6, 6);
  wait(0.5, sec); //Score blocks 
  intakedown.stop();
  intake.stop();  
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 315, 8, 6); // Backup
  wait(0.05, sec);
  driveToXAtHeading(115, 135, 8.0, 6.0); //to loader with x at 115, heading 135
  chassis.turn_to_angle(180); // Prepare to load
  intakeCylander.set(true);
  intakedown.spin(forward, 9, volt);
  intake.spin(forward, 12, volt); 
  chassis.drive_distance(15, 180, 4.0, 4.0); // into goal
  wait(1, sec); // load
  intake.stop();
  wait(0.05, sec);
  
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-10, 180, 4.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  pushCylinder.set(false);
  chassis.turn_to_angle(0);
  wait(0.05, sec);
  
  chassis.drive_distance(20, 180, 4.0, 4.0); // to long goal prepare 
  
  wait(0.05, sec);
  shooter.set(true); 
  pushCylinder.set(true); 
  wait(0.05, sec);
  chassis.drive_distance(4, 0, 6, 6);
  wait(3, sec); // Score blocks
  intake.stop();
  intakedown.stop();
  
  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
}
//----------------------------------------------------------------------

void Left_43()
{
  
  // Set initial pose: X, Y, and heading (0° = up/forward, 90° = right, 180° = down, 270° = left)
  set_robot_pose(54.4, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 10, volt);
  
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(36.4, 104, 8.0, 6.0); //should be safe/not too close to long goal
  wait(0.05, sec);
  

  // ========== PHASE 2: Score 3 blocks ==========
  chassis.turn_to_angle(45);
  intake.spin(forward, 10, volt);
  wait(0.05, sec);
  driveToXY(46.5, 92.5, 8.0, 6.0); // Approach slowly
  wait(0.2, sec);
  chassis.drive_distance(12, 45, 8.0, 6.0); // Drive to upper goal
  wait(0.05, sec);
  pushCylinder.set(true); 
  chassis.drive_distance(1.5, 45, 8, 6);
  wait(0.7, sec); //Score blocks 
  intakedown.stop();
  wait(0.05, sec);
  intake.stop(); 
  wait(0.05, sec); 
  // ========== PHASE 3: back up and load ==========
  pushCylinder.set(false); 
  chassis.drive_distance(-10, 45, 8, 6); // Backup
  wait(0.05, sec);
  intake.stop();
  driveToXAtHeading(25, 225, 8.0, 6.0); //to loader at X=25 while maintaining heading 225°
  chassis.turn_to_angle(180); // Prepare to load
  intakedown.spin(forward, 10, volt);
  intakeCylander.set(true);
  intake.spin(forward, 10, volt); 
  chassis.drive_distance(15, 180, 6.0, 4.0); // into goal
  wait(1, sec); // load
  intake.stop();
  wait(0.05, sec);
  
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-12, 180, 4.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  chassis.turn_to_angle(0);
  intakedown.stop();
  intake.stop();
  wait(0.05, sec);
  
  chassis.drive_distance(5, -3, 4.0, 4.0); // to long goal prepare 
  
  wait(0.05, sec);
  shooter.set(true); 
  //start to score
  chassis.drive_distance(7.5, 0, 6, 6);
  pushCylinder.set(true); 
  wait(0.05, sec);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 10, volt);
  chassis.drive_distance(2.5, 0, 6, 6);
  wait(1.5, sec); // Score blocks
  pushCylinder.set(false);
  intake.stop();
  intakedown.stop();
  
  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
}

//--------------------------------------------------------

void Right_7()
{

  set_robot_pose(86, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 10, volt);
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(93.5, 93.5, 8.0, 6.0);
  wait(0.05, sec);
  intake.spin(forward, 10, volt);

  // ========== PHASE 2: intake 1 block ==========
  wait(0.05, sec);
  driveToXY(110, 82, 8.0, 6.0); // go intake, make sure don't cross alliance line
  wait(0.1, sec);
  driveToXYBackward(93.5, 93.5, 8.0, 6.0); // Drive to low goal backward
  wait(0.05, sec);
  chassis.turn_to_angle(315);
  chassis.drive_distance(15, 315, 8.0, 6.0); // Drive to low goal
  intakedown.spin(reverse, 6, volt);
  intake.spin(reverse, 10, volt);
  wait(0.05, sec);
  chassis.drive_distance(4, 315, 6, 6);
  wait(0.1, sec); //Score blocks 
  intakedown.stop();
  intake.stop();  
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 315, 8, 6); // Backup
  wait(0.05, sec);
  driveToXAtHeading(115, 135, 8.0, 6.0); //to loader with x at 115, heading 135
  chassis.turn_to_angle(180); // Prepare to load
  intakeCylander.set(true);
  intakedown.spin(forward, 9, volt);
  intake.spin(forward, 12, volt); 
  chassis.drive_distance(15, 180, 4.0, 4.0); // into goal
  wait(1, sec); // load
  intake.stop();
  wait(0.05, sec);


}

//--------------------------------------------------------

void Left_7()
{
  // 根據對稱性推算左側座標
  set_robot_pose(54.4, 116.3, 0.0); // 對稱於右側的 (86, 116.3)
  intakedown.spin(forward, 10, volt);
  
  // ========== PHASE 1: Drive to intake position ==========
  wait(0.05, sec);
  // 對稱於右側的 (93.5, 93.5)
  driveToXY(46.5, 93.5, 8.0, 6.0);
  wait(0.05, sec);
  intake.spin(forward, 10, volt);

  // ========== PHASE 2: intake 1 block ==========
  wait(0.05, sec);
  // 對稱於右側的 (110, 82)
  driveToXY(30.4, 82, 8.0, 6.0); // go intake, make sure don't cross alliance line
  wait(0.1, sec);
  
  // 對稱於右側的 (93.5, 93.5)
  driveToXYBackward(46.5, 93.5, 8.0, 6.0); // Drive to upper goal backward
  wait(0.05, sec);
  
  // 右側 315° -> 左側 45°
  chassis.turn_to_angle(45);
  chassis.drive_distance(15, 45, 8.0, 6.0); // Drive to upper goal
  intakedown.spin(reverse, 6, volt);
  intake.spin(reverse, 10, volt);
  wait(0.05, sec);
  chassis.drive_distance(4, 45, 6, 6);
  wait(0.1, sec); //Score blocks 
  intakedown.stop();
  intake.stop();  
  
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 45, 8, 6); // Backup
  wait(0.05, sec);
  
  // 對稱於右側的 (115, 135) -> (25, 225)
  driveToXAtHeading(25, 225, 8.0, 6.0); // to loader with x at 25, heading 225
  chassis.turn_to_angle(180); // Prepare to load
  intakeCylander.set(true);
  intakedown.spin(forward, 9, volt);
  intake.spin(forward, 12, volt); 
  chassis.drive_distance(15, 180, 4.0, 4.0); // into goal
  wait(1, sec); // load
  intake.stop();
  wait(0.05, sec);

  // 如果需要顯示最終位置
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
}

//--------------------------------------------------------

void Solo()
{
  vex::color selectedTeamColor = vex::color::red;
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  //intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  chassis.drive_distance(4, 43, 4, 4);//preintake
  intake.spin(forward, 12, volt);
  chassis.drive_distance(3, 43, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(22, 43, 6, 6);//middle goals
  pushCylinder.set(true);
  chassis.drive_distance(5, 43, 6, 6);//middle goals
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-10, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);

  chassis.turn_to_angle(110);
  wait(0.05,sec);
  chassis.drive_distance(50, 110, 10, 10);//to 2nd pile
  chassis.turn_to_angle(319); 
  chassis.drive_distance(4, 319, 4, 4);//preintake
  wait(0.05,sec);
  chassis.drive_distance(3, 319, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(22, 319, 6, 6);//middle goals
  intake.spin(reverse, 12, volt);
  chassis.drive_distance(5, 319, 6, 6);//middle goals
  intakedown.spin(reverse, 9, volt);
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-8, 319, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);

}
//--------------------------------------------------------

void Skills()
{
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  wait(0.05,sec);
  chassis.drive_distance(7.5, 43, 5, 5);//intake 3 balls
  wait(.3,sec);
  //shooter.set(true);
  pushCylinder.set(true);
  wait(0.05,sec);
  chassis.drive_distance(25.5, 43, 6, 6);//middle goals
  intake.spin(forward, 7, volt);
  wait(2.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-15, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);
  chassis.turn_to_angle(110);
  wait(0.05,sec);
  chassis.drive_distance(65, 110, 4, 4);//turn right
  wait(0.05,sec);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  chassis.drive_distance(10, 300, 4, 4);
  wait(.5,sec);
  intakedown.spin(reverse, 12, volt);
  wait(0.05,sec);
  intake.spin(reverse, 12, volt);
  wait(0.05,sec);
  chassis.drive_distance(-10, 300, 4, 4);
  wait(0.05,sec);
  chassis.turn_to_angle(3);
  wait(0.05,sec);
  chassis.drive_distance(60, 3, 6, 6);
  wait(0.05,sec);
  chassis.turn_to_angle(280);
  wait(0.05,sec);
  chassis.drive_distance(60, 280, 6, 6);
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  pushCylinder.set(true);
  wait(0.05,sec);
  shooter.set(true);
  chassis.drive_distance(14, 180, 6, 6);
  

}
//--------------------------------------------------------
// NOTE: recordPath() is defined in main.cpp (lines 371-403)
// It's called from the autonomous selector (main.cpp line 1310)
//--------------------------------------------------------
void blank1()
{
  odom_test();
  wait(5, sec);
}
//--------------------------------------------------------
void blank2()
{
  // Set starting position and heading to 0
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  set_robot_pose(22, 22.5, 0);

  vex::task odomTask(odometry_task);

  
  driveToXY(22, 54, 6, 1);
  wait(0.05, sec);
  turnToXY(-22, 80, 3);
  wait(0.05, sec);
  driveToXY(-22, 80, 6, 1);
  wait(0.05, sec);
  turnToXY(-13, 88, 3);
  wait(0.05, sec);
  driveToXY(-13, 88, 6, 1);
  
}
//-------------------------------------------------------- 
void blank3()
{
  // ========== SIMPLE MOTOR TEST - DO NOT DELETE ==========
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 20, "Motor Test Starting...");
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Motor Test");
  wait(1, sec);
  
  // Test: Drive forward for 3 seconds at 8 volts
  Brain.Screen.printAt(10, 40, "Driving FORWARD...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Forward 8V...");
  
  L1.spin(forward, 8, volt);
  L2.spin(forward, 8, volt);
  L3.spin(forward, 8, volt);
  R1.spin(forward, 8, volt);
  R2.spin(forward, 8, volt);
  R3.spin(forward, 8, volt);
  wait(3, sec);
  
  // Stop
  L1.stop(brake); 
  L2.stop(brake); 
  L3.stop(brake);
  R1.stop(brake); 
  R2.stop(brake); 
  R3.stop(brake);
  
  Brain.Screen.printAt(10, 60, "Stopped.");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Stopped     ");
  wait(1, sec);
  
  // Test: Drive backward for 3 seconds
  Brain.Screen.printAt(10, 80, "Driving BACKWARD...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Backward 8V...");
  
  L1.spin(reverse, 8, volt);
  L2.spin(reverse, 8, volt);
  L3.spin(reverse, 8, volt);
  R1.spin(reverse, 8, volt);
  R2.spin(reverse, 8, volt);
  R3.spin(reverse, 8, volt);
  wait(3, sec);
  
  // Stop
  L1.stop(brake); 
  L2.stop(brake); 
  L3.stop(brake);
  R1.stop(brake); 
  R2.stop(brake); 
  R3.stop(brake);
  
  Brain.Screen.printAt(10, 100, "Test COMPLETE!");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("DONE!       ");
  Controller1.rumble("--");
}
//--------------------------------------------------------


void B_17022A()
{
  vex::color selectedTeamColor = vex::color::blue;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(10, .5, 0.005, 4, 20);
  chassis.set_heading_constants(10, .25, 0.007, 5, 20);
  chassis.turn_to_angle(316);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3, 316, 6, 6);
  //----------------------------------------------------------------------

  // Reset motor positions
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  set_robot_pose(0, 0, 0);

  vex::task odomTask(odometry_task);

  
  driveToXY(0, 20, 6, 1);
  wait(2, sec);
  turnToXY(10, 20, 4.0);
  wait(2, sec);
  driveToXY(10, 20, 6, 1);
  
}
//-------------------------------------------------------- 
// Removed duplicate blank3() function
//--------------------------------------------------------


// Removed duplicate B_17022A() function - using the one defined earlier

void odom_test()
{
}

void PIDtest()
{
}

void default_constants()
{
  chassis.set_drive_constants(12, .5, 0.004, 5, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 40);
  chassis.set_turn_constants(12, .35, .001, 3, 90);
  chassis.set_swing_constants(12, .5, .001, 2, 15);
  chassis.set_drive_exit_conditions(.5, 0, 2000);
  chassis.set_turn_exit_conditions(4, 0, 3000);
  chassis.set_swing_exit_conditions(10, 0, 3000);
}

void odom_constants()
{
  default_constants();
  chassis.drive_max_voltage = 12;
  chassis.drive_settle_error = 3;
}

void drive_test()
{
  chassis.turn_to_angle(45);
  chassis.drive_distance(30);
  chassis.turn_to_angle(270);
  chassis.drive_distance(20);
  chassis.turn_to_angle(135);
  chassis.drive_distance(30);
  chassis.turn_to_angle(270);
  chassis.drive_distance(20);
  chassis.turn_to_angle(0);
}

void turn_test()
{
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test()
{
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test()
{
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void tank_odom_test()
{
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24, 24);
  chassis.drive_to_point(0, 0);
  chassis.turn_to_angle(0);
}

void holonomic_odom_test()
{
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}