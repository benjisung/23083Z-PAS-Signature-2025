# Path Recording Guide

## Overview
The path recording system lets you manually drive your robot and save waypoints (X, Y, heading) along the path. This is useful for creating autonomous routines by recording the exact path you want the robot to follow.

## How to Start Recording

### Method 1: From Autonomous Selector
1. **Select Path Recording Mode**: In the autonomous selector, choose the path recording option (usually one of the autonomous slots)
2. **Start Autonomous**: The recording will start automatically when you run autonomous

### Method 2: Direct Call (in code)
If you want to call it directly, you can add this to your autonomous function:
```cpp
void my_auton() {
    recordPath();  // Start path recording
}
```

## Recording Process

### Step 1: Initial Setup
- The system will:
  - Reset encoders to zero
  - Set starting position (default: X=22.0, Y=22.5)
  - Use current IMU heading as starting heading
  - Start the odometry tracking task

### Step 2: Drive Manually
- **Use your controller** to drive the robot manually
- The robot will track its position using odometry (encoders + IMU)
- Drive exactly the path you want to record

### Step 3: Record Waypoints
- **Press Button A** on the controller to save a waypoint at the current position
- You'll feel a **triple short rumble** when a waypoint is saved
- The Brain screen will show "Waypoint X recorded!" (where X is the waypoint number)
- The Controller screen will show "Saved! Pts:X" (where X is total waypoints)

**Tips for recording waypoints:**
- Record waypoints at **key points** along your path:
  - Starting position (automatically recorded)
  - Turns
  - Object pickup/drop locations
  - Important navigation points
- Don't record too many waypoints (can be hard to use later)
- Don't record too few waypoints (path won't be accurate)

### Step 4: Finish Recording
- **Press Button B** on the controller to stop recording
- You'll feel a **triple long rumble** when recording stops
- The system will display the recorded path

## Output Format

After recording, the path is printed in two places:

### 1. Brain Screen
- Shows up to 10 waypoints
- Format: `Index: X=value Y=value H=heading D=distance`

### 2. Console/Terminal (for copy-paste)
The path is printed in C++ code format that you can copy directly into your autonomous:

```cpp
========== RECORDED PATH ==========
// Total waypoints: 5
// Format: X, Y, Heading, Distance

// Waypoint 0
const double WP0_X = 22.00;
const double WP0_Y = 22.50;
const double WP0_H = 0.0;
const double WP0_D = 0.00; // Total distance from start

// Waypoint 1
const double WP1_X = 30.50;
const double WP1_Y = 35.20;
const double WP1_H = 45.0;
const double WP1_D = 15.30; // Total distance from start

// ... (more waypoints)
========== END RECORDED PATH ==========
```

## Using Recorded Paths

### Option 1: Use `driveToXY()` for Each Waypoint
```cpp
void my_auton() {
    // Set starting position
    set_robot_pose(22.0, 22.5, 0);
    vex::task odomTask(odometry_task);
    
    // Drive to each waypoint
    driveToXY(WP1_X, WP1_Y, 6, 1);
    driveToXY(WP2_X, WP2_Y, 6, 1);
    driveToXY(WP3_X, WP3_Y, 6, 1);
    // ... etc
}
```

### Option 2: Use `turnToXY()` Before Driving
If you want to face the next waypoint before driving:
```cpp
void my_auton() {
    set_robot_pose(22.0, 22.5, 0);
    vex::task odomTask(odometry_task);
    
    // Turn to face waypoint, then drive
    turnToXY(WP1_X, WP1_Y, 3);
    driveToXY(WP1_X, WP1_Y, 6, 1);
    
    turnToXY(WP2_X, WP2_Y, 3);
    driveToXY(WP2_X, WP2_Y, 6, 1);
    // ... etc
}
```

## Customizing Starting Position

To change the starting position, edit the `recordPath()` function in `main.cpp`:

```cpp
void recordPath() {
    // Change these values to match your starting position
    const double START_X = 22.0;   // Your starting X coordinate
    const double START_Y = 22.5;   // Your starting Y coordinate
    const double START_HEADING = Inertial.heading();  // Current IMU heading
    
    startRecording(START_X, START_Y, START_HEADING);
    // ... rest of function
}
```

## Tips & Best Practices

1. **Drive Slowly**: Drive at a consistent, slow speed to get accurate waypoints
2. **Record Key Points**: Don't record every inch - record important navigation points
3. **Test the Path**: After recording, test the autonomous to see if it follows the path correctly
4. **Adjust Waypoints**: You can manually edit waypoint coordinates if needed
5. **Multiple Paths**: Record different paths for different autonomous routines
6. **Save Output**: Copy the console output and save it in your autonomous code file

## Troubleshooting

### Problem: Waypoints seem inaccurate
- **Solution**: Make sure odometry is working correctly. Check that encoders and IMU are calibrated.

### Problem: Robot doesn't follow the recorded path
- **Solution**: 
  - Check that starting position matches when you run autonomous
  - Verify waypoint coordinates are correct
  - Make sure odometry is initialized before using `driveToXY()`

### Problem: Can't see console output
- **Solution**: 
  - Check the terminal/console in your IDE
  - The output is also shown on the Brain screen (up to 10 waypoints)

### Problem: Recording stops unexpectedly
- **Solution**: 
  - Make sure you're not accidentally pressing Button B
  - Check that the odometry task is running

## Example Workflow

1. **Position robot** at starting position (e.g., X=22, Y=22.5)
2. **Start recording** from autonomous selector
3. **Drive manually** to your first target
4. **Press Button A** to save waypoint
5. **Continue driving** to next point
6. **Press Button A** again to save another waypoint
7. **Repeat** until you've recorded the entire path
8. **Press Button B** to finish recording
9. **Copy the console output** and paste into your autonomous code
10. **Test the autonomous** to verify it follows the path

## What Gets Recorded

Each waypoint contains:
- **X**: X coordinate in inches
- **Y**: Y coordinate in inches  
- **Heading**: Robot's heading in degrees (0-360)
- **Distance**: Total distance traveled from start (in inches)

The odometry system continuously tracks position using:
- Motor encoders (for distance traveled)
- IMU (for heading)
- Sensor fusion (combines both for accuracy)

