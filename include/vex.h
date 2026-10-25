/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once

// Standard C++ libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// VEX V5 libraries
#include "v5.h"
#include "v5_vcs.h"

// Project headers
#include "robot-config.h"
#include "JAR-Template/odom.h"
#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"
#include "autons.h"

// ========== Utility Macros ==========
/**
 * Wait until a condition is true
 * @param condition Boolean condition to wait for
 */
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

/**
 * Repeat a block of code a specified number of times
 * @param iterations Number of times to repeat
 */
#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
