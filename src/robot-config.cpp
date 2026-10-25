#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor L1 = motor(PORT18, ratio6_1, true);
motor L2 = motor(PORT17, ratio6_1, false);
motor L3 = motor(PORT19, ratio6_1, true);
motor R1 = motor(PORT7, ratio6_1, false);
motor R2 = motor(PORT9, ratio6_1, true);
motor R3 = motor(PORT8, ratio6_1, false);
inertial Inertial = inertial(PORT20);
controller Controller1 = controller(primary);
motor intake = motor(PORT6, ratio6_1, true);
optical Optical = optical(PORT11);
optical Optical_go = optical(PORT15);
motor intakedown = motor(PORT5, ratio6_1, false);


digital_out shooter = digital_out(Brain.ThreeWirePort.A);
digital_out pushCylinder = digital_out(Brain.ThreeWirePort.B);
digital_out intakeCylander = digital_out(Brain.ThreeWirePort.D);
digital_out wing = digital_out(Brain.ThreeWirePort.C);
digital_out redlight = digital_out(Brain.ThreeWirePort.H);
digital_out whitelight = digital_out(Brain.ThreeWirePort.G);

//MCL Distance
distance BackDis = distance(PORT12);
distance LeftDis = distance(PORT1);
distance RightDis = distance(PORT2);
distance FrontDis = distance(PORT3);

// 88
//  VEXcode generated functions
//  define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  // nothing to initialize
}