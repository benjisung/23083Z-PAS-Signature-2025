#pragma once

using namespace vex;

// ========== VEX Brain ==========
extern brain Brain;

// ========== Drive Motors ==========
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;

// ========== Other Motors ==========
extern motor intake;
extern motor intakedown;

// ========== Sensors ==========
extern optical Optical;
extern optical Optical_go;
extern inertial Inertial;
extern controller Controller1;

extern distance BackDis;
extern distance LeftDis;
extern distance RightDis;
extern distance FrontDis;

// ========== Digital Outputs (Pneumatics/Lights) ==========
extern digital_out wing;
extern digital_out redlight;
extern digital_out whitelight;
extern digital_out intakeCylander;
extern digital_out pushCylinder;
extern digital_out shooter;

// Math Functions
extern double AngleWrap(double LeAngle);
extern double Normalize(double LeAngle);
extern double ToRadians(double val);
extern double ToDegrees(double val);
extern double Sign(double val);

// ========== Initialization ==========
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
