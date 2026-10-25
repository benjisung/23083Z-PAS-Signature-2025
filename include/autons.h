#pragma once

#include "JAR-Template/drive.h"

// Forward declaration
class Drive;

// ========== Global Chassis Object ==========
extern Drive chassis;

// ========== Initialization ==========
void default_constants();

// ========== Competition Autonomous Routines ==========
void Right_43();
void Left_43();
void Right_7();
void Left_7();
void Solo();
void Skills();

// ========== Test/Blank Autonomous Routines ==========
void blank1();
void blank2();
void blank3();
void blank4();

// ========== Additional Routines ==========
void B_17022A();
void skills();

// ========== Test Functions ==========
void PIDtest();
void autoarm();
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
