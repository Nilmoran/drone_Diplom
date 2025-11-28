# Drone Movement Issues - Detailed Analysis & Solutions

## Executive Summary

The drone exhibits two primary issues:
1. **Trajectory Deviation**: Not following the designated route accurately
2. **Turn Stalling**: Getting stuck or slowing excessively during turns

## Root Cause Analysis

### 1. Trajectory Following Issues

#### A. Kalman Filter Over-Smoothing
**Location**: `KalmanFilter2D.update()` (lines 169-282)
**Problem**: 
- `smoothing_alpha = 0.75` is too high, causing significant lag
- The filter applies exponential smoothing even after Kalman filtering
- This creates a delay between actual position and displayed position
- During turns, this lag causes the drone to "chase" its own position

**Impact**: 
- Drone appears to overshoot waypoints
- Position updates lag behind actual movement
- Creates oscillatory behavior during corrections

#### B. Hybrid Navigation Conflicts
**Location**: `apply_hybrid_navigation()` (lines 957-1003)
**Problem**:
- Hybrid system blends route position with trilateration measurements
- During turns, trilateration noise can pull drone away from route
- Correction strength calculation may be too conservative (max 30% correction)
- Signal quality dependency can cause inconsistent behavior

**Impact**:
- Drone drifts from route during low signal periods
- Corrections may conflict with control system commands
- Creates "fighting" between navigation systems

#### C. Control System Reset at Waypoints
**Location**: Lines 1269-1296
**Problem**:
- Control system is reset at EVERY waypoint (line 1294: `if turn_angle > 15.0 or True`)
- This clears PID integral terms, causing loss of correction momentum
- Reset happens even on straight segments
- No gradual transition between segments

**Impact**:
- Sudden loss of correction at waypoints
- Drone may overshoot before control system re-engages
- Creates jerky motion at segment boundaries

### 2. Turn Stalling Issues

#### A. Bank Angle Rate Limiting Too Restrictive
**Location**: `FlightControlSystem.compute_control()` (line 714)
**Problem**:
- `MAX_BANK_RATE_DEG_S = 25.0` degrees/second
- For a 90° turn, this takes 3.6 seconds to reach full bank
- During this time, drone may not turn fast enough
- Rate limiter prevents rapid bank angle changes needed for sharp turns

**Impact**:
- Drone takes too long to initiate turns
- May overshoot turn entry point
- Creates "stuck" appearance as drone slowly banks

#### B. Minimum Turn Radius Constraint
**Location**: `FlightControlSystem.constrain_bank_angle()` (lines 648-659)
**Problem**:
- `MIN_TURN_RADIUS_M = 15.0` meters
- At 10 m/s speed, this requires ~25° bank angle minimum
- For tighter turns, system may reject desired bank angle
- Constraint may be too conservative for urban navigation

**Impact**:
- Drone cannot execute tight turns
- May cut corners or overshoot waypoints
- Creates "stuck" behavior when route requires tight turn

#### C. Speed Reduction Too Aggressive
**Location**: Lines 1170-1177
**Problem**:
- Speed reduced by up to 50% for 180° turns
- Minimum speed set to 60% of normal (6 m/s)
- Turn factor calculation: `1.0 - (turn_angle / 180.0) * 0.5`
- Speed reduction starts at 30m before turn (LOOKAHEAD_DISTANCE_M)

**Impact**:
- Drone slows too much before turns
- May appear "stuck" as it crawls through turn
- Recovery after turn is slow

#### D. Turn Anticipation Logic Issues
**Location**: Lines 1146-1184
**Problem**:
- Turn angle calculation uses dot product, which may not detect all turn types
- Bank angle preparation only happens within 15m of turn
- No consideration of turn radius vs. required radius
- Preparation bank angle limited to 30% of max (9°)

**Impact**:
- Insufficient preparation for sharp turns
- Drone enters turn at wrong angle
- Requires correction during turn, causing oscillation

#### E. Segment Progress Calculation
**Location**: Lines 1186-1191
**Problem**:
- Progress increment: `move_distance / segment_distance`
- If segment is very short, progress jumps quickly
- No minimum time per segment
- Progress can exceed 1.0 if velocity is high relative to segment length

**Impact**:
- Drone may skip waypoints on short segments
- Progress may not accurately reflect position
- Can cause "stuck" behavior if progress calculation fails

### 3. Additional Contributing Factors

#### A. Control System Gains
**Location**: `FlightControlSystem.__init__()` (lines 551, 555)
**Current Values**:
- Lateral PID: kp=0.8, ki=0.01, kd=0.3
- Longitudinal PID: kp=0.4, ki=0.005, kd=0.15

**Issues**:
- Derivative gain (kd) may be too high, causing overshoot
- Integral gain (ki) very low, may not correct steady-state errors
- Proportional gain may need tuning for different turn types

#### B. Velocity Smoothing
**Location**: Lines 1205-1211
**Problem**:
- `VELOCITY_SMOOTHING_ALPHA = 0.7` applies exponential smoothing
- This creates lag in velocity vector
- During turns, velocity vector may point wrong direction

**Impact**:
- Delayed response to direction changes
- May contribute to overshoot

#### C. Cross-Track Error Correction
**Location**: Lines 1218-1236
**Problem**:
- Correction only applied when `cross_track_error > 2.0m` and not in turn
- During turns, no correction applied
- Correction factor limited to 15% maximum

**Impact**:
- May allow drift during turns
- Correction may be insufficient for large errors

## Recommended Solutions

### Priority 1: Critical Fixes

1. **Fix Control System Reset Logic**
   - Only reset on significant turns (>30°)
   - Preserve heading and bank angle state
   - Gradual transition instead of hard reset

2. **Improve Turn Anticipation**
   - Increase look-ahead distance for sharp turns
   - Calculate required turn radius vs. available radius
   - Start bank angle preparation earlier (50m instead of 15m)
   - Increase preparation bank angle to 50% of max

3. **Optimize Bank Angle Rate Limiting**
   - Increase `MAX_BANK_RATE_DEG_S` to 35-40 deg/s
   - Allow faster rate changes during turn initiation
   - Reduce rate limiting during turn exit

4. **Fix Segment Progress Calculation**
   - Add minimum segment time
   - Cap progress increment to prevent overshoot
   - Better handling of very short segments

### Priority 2: Performance Improvements

5. **Reduce Kalman Filter Smoothing**
   - Lower `smoothing_alpha` to 0.6
   - Reduce velocity damping
   - Faster response to position updates

6. **Optimize Speed Reduction for Turns**
   - Less aggressive reduction (max 30% instead of 50%)
   - Minimum speed 70% instead of 60%
   - Faster recovery after turn

7. **Improve Hybrid Navigation**
   - Reduce correction during turns
   - Better signal quality weighting
   - Adaptive correction strength

8. **Tune PID Gains**
   - Increase integral gain slightly for better steady-state
   - Reduce derivative gain to prevent overshoot
   - Adaptive gains based on turn angle

### Priority 3: Enhancements

9. **Add Turn Radius Prediction**
   - Calculate required turn radius from route geometry
   - Compare with minimum achievable radius
   - Adjust speed/bank angle accordingly

10. **Improve Waypoint Transition**
    - Smooth interpolation between segments
    - Preserve velocity vector direction
    - Gradual heading change

## Implementation Plan

See code changes in main.py for detailed fixes.

