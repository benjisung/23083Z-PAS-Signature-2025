# PID Tuning Guide for turnToXY

## What Each Term Does

### kH (Proportional Gain)
- **What it does**: Responds to current error
- **Formula**: `P = kH * headingError`
- **Effect**: 
  - Higher kH = faster response, but can cause overshoot/oscillation
  - Lower kH = slower response, but more stable

### kHi (Integral Gain)
- **What it does**: Accumulates error over time (fixes persistent errors)
- **Formula**: `I = kHi * sum(headingError * dt)`
- **Effect**:
  - Higher kHi = eliminates steady-state error faster, but can cause overshoot
  - Lower kHi = slower to correct persistent errors, but more stable

### kHd (Derivative Gain)
- **What it does**: Predicts future error based on rate of change
- **Formula**: `D = kHd * (error_rate)`
- **Effect**:
  - Higher kHd = more damping, reduces overshoot, but can be sluggish
  - Lower kHd = less damping, faster response, but more overshoot

## Tuning Process (Start with kH only, then add I and D)

### Step 1: Tune kH (Proportional) First
1. Set kHi = 0, kHd = 0 (disable I and D)
2. Start with kH = 0.05
3. Test: Does the robot turn at all?
   - **No** → Increase kH (try 0.10, 0.15, 0.20)
   - **Yes** → Continue
4. Test: Does it overshoot?
   - **Yes, overshoots** → Decrease kH
   - **No overshoot, but too slow** → Increase kH
5. Goal: Find the highest kH that doesn't overshoot

### Step 2: Add kHi (Integral) if needed
1. Keep kH from Step 1
2. Set kHd = 0 (disable D)
3. Start with kHi = 0.001
4. Test: Does it settle at the target?
   - **No, stops before target** → Increase kHi (try 0.002, 0.003)
   - **Yes, but oscillates** → Decrease kHi
5. Goal: Eliminates steady-state error without oscillation

### Step 3: Add kHd (Derivative) to reduce overshoot
1. Keep kH and kHi from Steps 1-2
2. Start with kHd = 0.05
3. Test: Does it still overshoot?
   - **Yes** → Increase kHd (try 0.08, 0.10, 0.12)
   - **No, but too slow** → Decrease kHd
4. Goal: Reduces overshoot without making it sluggish

## Common Symptoms and Fixes

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Robot doesn't turn at all | kH too low | Increase kH |
| Robot overshoots and oscillates | kH too high | Decrease kH |
| Robot overshoots but settles | Need damping | Increase kHd |
| Robot stops before target | Need integral | Increase kHi |
| Robot oscillates around target | kHi too high | Decrease kHi |
| Robot is sluggish/slow | kH too low or kHd too high | Increase kH or decrease kHd |
| Robot turns past target, corrects, overshoots again | Need derivative | Add/increase kHd |

## Recommended Starting Values

For VEX robots with typical drivetrain:
- **kH**: 0.10 - 0.15 (start here)
- **kHi**: 0.001 - 0.002 (add if needed)
- **kHd**: 0.05 - 0.10 (add to reduce overshoot)

## Your Current Values
- kH = 0.12 (good - responsive)
- kHi = 0.002 (good - eliminates steady-state error)
- kHd = 0.08 (defined but not used - add it back if overshooting)

## Quick Tuning Tips

1. **One at a time**: Change only one gain at a time
2. **Small steps**: Change by 20-30% at a time (not 2x)
3. **Test consistently**: Use the same test case (e.g., turnToXY(10, 0))
4. **Watch the behavior**: 
   - Does it overshoot? → Reduce kH or increase kHd
   - Does it undershoot? → Increase kH or kHi
   - Does it oscillate? → Reduce kH or kHi, increase kHd

## Example Tuning Session

1. Start: kH=0.10, kHi=0, kHd=0
   - Result: Turns but overshoots by 10°
   - Action: Add derivative, kHd=0.05

2. Test: kH=0.10, kHi=0, kHd=0.05
   - Result: Still overshoots by 5°
   - Action: Increase kHd to 0.08

3. Test: kH=0.10, kHi=0, kHd=0.08
   - Result: No overshoot, but stops 2° before target
   - Action: Add integral, kHi=0.001

4. Test: kH=0.10, kHi=0.001, kHd=0.08
   - Result: Settles perfectly!
   - Final values: kH=0.10, kHi=0.001, kHd=0.08

