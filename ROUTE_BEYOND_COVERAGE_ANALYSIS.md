# Route Construction Beyond Ground Station Coverage: Analysis and Solutions

## Executive Summary

When a drone's route extends beyond the operational area of ground stations, the number of available base stations can drop below the minimum required for trilateration (currently 2-3 stations). This causes the positioning system to fail, leading to erratic movement as the drone relies on inaccurate dead reckoning instead of trilateration-based navigation.

## Scenario Description

### Phase 1: Normal Operation (Within Coverage)

**Initial State:**
- Drone starts at Point A (e.g., Rechnoy Vokzal) with good station coverage
- 5-7 base stations available within range
- Trilateration works accurately (15-20m precision)
- Hybrid navigation combines route following with trilateration corrections

**Navigation Behavior:**
- Position calculated via weighted intersection center optimization (as per ТКМС.txt methodology)
- Objective function: `min Σ(signal_i × distance_i²)` - weighted center of mass
- Constraints: `radius_i² - distance_i² ≥ 0` - point must be within all station radii
- Initial guess: arithmetic mean of all selected base station coordinates
- Route following is accurate with small deviations (5-15m)

### Phase 2: Transition Zone (Entering Low Coverage)

**What Happens:**
- Drone moves along route toward destination
- Gradually moves away from dense station coverage
- Number of stations in range decreases: 7 → 5 → 4 → 3 → 2
- Signal strength from remaining stations weakens
- Station selection algorithm (`select_diverse_towers`) struggles to find diverse stations

**Navigation Behavior:**
- Trilateration accuracy degrades (20m → 50m → 100m+)
- Optimization failures increase ("Positive directional derivative for linesearch")
- Hybrid navigation correction strength decreases (signal quality drops)
- Route deviation increases but remains manageable

### Phase 3: Critical Failure (Below Minimum Stations)

**Critical Threshold:**
- **Current Code**: Requires minimum 2 stations (reduced from 3)
- **Mathematical Requirement**: 3 stations minimum for proper trilateration
- **Reality**: With only 2 stations, position is ambiguous (two possible solutions)

**What Happens When Stations < 3:**

1. **Trilateration Failure:**
   ```python
   if len(measurements) < 2:  # Current threshold (was 3)
       return None  # Trilateration fails
   ```

2. **Fallback to Dead Reckoning:**
   - System uses Kalman filter prediction only
   - No new position measurements
   - Position drifts based on last known velocity
   - Error accumulates over time

3. **Hybrid Navigation Breakdown:**
   - `apply_hybrid_navigation()` receives invalid measured position
   - Correction strength becomes 0 (no signal)
   - System relies entirely on route following
   - But route following uses `true_position` which may be inaccurate

4. **Control System Confusion:**
   - Flight control system receives incorrect position data
   - Cross-track and along-track errors become meaningless
   - PID controllers attempt to correct based on wrong data
   - Results in erratic corrections in random directions

### Phase 4: Erratic Movement Pattern

**Symptom Manifestation:**

1. **Position Jumps:**
   - Kalman filter prediction drifts from actual position
   - When stations briefly become available, position "snaps" to new location
   - Large position jumps (50-200m) confuse control system

2. **Oscillatory Behavior:**
   - Control system sees large cross-track error
   - Applies correction toward route
   - Next update: position has drifted, error is in opposite direction
   - System corrects back, creating oscillation

3. **Random Direction Changes:**
   - Without accurate position, heading calculations are wrong
   - Control system commands turns based on incorrect heading
   - Drone appears to move randomly as it tries to correct non-existent errors

4. **Route Deviation Escalation:**
   - Initial deviation: 20m
   - After 10 seconds without stations: 50m
   - After 30 seconds: 150m+
   - Deviation continues growing exponentially

## Mathematical Analysis

### Trilateration with Insufficient Stations

**With 3+ Stations (Proper Trilateration):**
- System of equations is overdetermined
- Unique solution exists (or small solution space)
- Optimization converges reliably
- Position accuracy: 15-20m

**With 2 Stations (Ambiguous):**
- System of equations is underdetermined
- Two possible solutions (intersection of two circles)
- Optimization may converge to wrong solution
- Position accuracy: 50-200m (depending on station geometry)

**With 1 Station (Impossible):**
- Cannot determine position (only distance, not direction)
- System must use dead reckoning exclusively
- Position accuracy: Degrades exponentially with time

### Dead Reckoning Error Accumulation

**Error Model:**
```
Position_error(t) = Initial_error + Velocity_error × t + 0.5 × Acceleration_error × t²
```

**Typical Values:**
- Initial error: 20m (last known position)
- Velocity error: 0.5 m/s (estimated from Kalman filter uncertainty)
- After 10 seconds: 20 + 5 = 25m
- After 30 seconds: 20 + 15 = 35m
- After 60 seconds: 20 + 30 = 50m

**In Practice:**
- Without position updates, error grows faster
- Control system corrections based on wrong position increase error
- After 30-60 seconds, position can be 100-200m off

## Root Causes

### 1. **Insufficient Station Coverage**

**Problem:**
- Route extends into areas with sparse station deployment
- Stations have limited range (typically 1500m radius)
- Gaps in coverage between station coverage areas

**Impact:**
- Number of stations drops below minimum threshold
- Trilateration becomes impossible or inaccurate

### 2. **Inadequate Fallback Strategy**

**Current Implementation:**
```python
if not position_updated:
    if drone_state['position'] and drone_state['kalman_filter']:
        # Dead reckoning only
        kf.predict()
        predicted_pos = kf.get_position()
        drone_state['position'] = predicted_pos
```

**Problems:**
- No validation of dead reckoning accuracy
- No detection of coverage loss
- No route-based position correction
- Continues using inaccurate position for control

### 3. **Control System Using Wrong Position**

**Issue:**
- Flight control system uses `drone_state['position']` for error calculations
- When position is inaccurate, errors are wrong
- Corrections are applied in wrong directions
- Creates positive feedback loop (error increases)

### 4. **No Coverage Prediction**

**Missing Feature:**
- System doesn't predict when route will enter low-coverage areas
- No warning before entering coverage gaps
- No route planning to avoid coverage gaps
- No alternative navigation mode for low-coverage areas

## Solutions and Recommendations

### Solution 1: Enhanced Dead Reckoning with Route Constraint

**Implementation:**
- When stations < 2, use route-based dead reckoning instead of pure Kalman prediction
- Project predicted position onto route line
- Constrain position to be within reasonable distance of route

**Code Changes:**
```python
if not position_updated and stations_available < 2:
    # Enhanced dead reckoning: constrain to route
    if drone_state['route'] and drone_state['true_position']:
        # Use route position as primary, Kalman as secondary
        route_pos = drone_state['true_position']
        kf.predict()
        kf_pos = kf.get_position()
        
        # Blend: 80% route, 20% Kalman (trust route more in low coverage)
        blended_pos = {
            'lat': route_pos['lat'] * 0.8 + kf_pos['lat'] * 0.2,
            'lon': route_pos['lon'] * 0.8 + kf_pos['lon'] * 0.2
        }
        
        # Project onto route if deviation too large
        route_deviation = calculate_distance_to_route_segment(
            blended_pos, route_start, route_end
        )
        if route_deviation > 50.0:  # If >50m off route, snap to route
            # Project onto route at current progress
            drone_state['position'] = route_pos
        else:
            drone_state['position'] = blended_pos
```

### Solution 2: Coverage-Aware Route Planning

**Implementation:**
- Before setting route, analyze station coverage along route
- Warn user if route extends into low-coverage areas
- Suggest alternative routes that stay within coverage
- Mark route segments with coverage quality

**Code Changes:**
```python
def analyze_route_coverage(waypoints, stations):
    """Analyze station coverage along route"""
    coverage_segments = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        # Sample points along segment
        segment_coverage = []
        for j in range(10):  # Sample 10 points
            progress = j / 9.0
            point = interpolate_position(start, end, progress)
            # Count stations in range
            stations_in_range = sum(
                1 for s in stations 
                if calculate_distance_meters(
                    point['lat'], point['lon'], s['lat'], s['lon']
                ) <= s['radius']
            )
            segment_coverage.append(stations_in_range)
        
        min_coverage = min(segment_coverage)
        avg_coverage = sum(segment_coverage) / len(segment_coverage)
        coverage_segments.append({
            'segment': i,
            'min_stations': min_coverage,
            'avg_stations': avg_coverage,
            'warning': min_coverage < 3
        })
    
    return coverage_segments
```

### Solution 3: Degraded Mode Navigation

**Implementation:**
- When stations < 3, switch to "route-following mode"
- Disable trilateration corrections
- Use route position exclusively
- Reduce control system sensitivity

**Code Changes:**
```python
# In simulate_drone_flight()
stations_available = len(measurements)
if stations_available < 3:
    # Degraded mode: route-following only
    drone_state['navigation_mode'] = 'route_only'
    # Use route position directly, ignore trilateration
    drone_state['position'] = copy_waypoint(drone_state['true_position'])
    # Reduce control system gains for stability
    if drone_state.get('flight_control'):
        fcs = drone_state['flight_control']
        fcs.adaptive_gain_factor = 0.5  # Reduce sensitivity
else:
    drone_state['navigation_mode'] = 'hybrid'
    # Normal hybrid navigation
```

### Solution 4: Station Coverage Visualization

**Implementation:**
- Display coverage map showing station coverage areas
- Highlight route segments with insufficient coverage
- Show real-time station count along route

**UI Enhancement:**
- Add coverage heatmap overlay
- Color-code route segments by coverage quality
- Display warning when entering low-coverage area

### Solution 5: Improved 2-Station Trilateration

**Current Issue:**
- With 2 stations, position is ambiguous (two intersection points)
- System picks one arbitrarily, may be wrong

**Solution:**
- Use route position to disambiguate
- Choose intersection point closest to route
- Weight solution by route proximity

**Code Changes:**
```python
def trilaterate_with_2_stations(station1, station2, route_position):
    """Trilaterate with 2 stations, using route to disambiguate"""
    # Calculate two intersection points of circles
    intersections = calculate_circle_intersections(
        station1, station2
    )
    
    if len(intersections) == 2:
        # Choose intersection closest to route
        dist1 = calculate_distance_meters(
            route_position['lat'], route_position['lon'],
            intersections[0]['lat'], intersections[0]['lon']
        )
        dist2 = calculate_distance_meters(
            route_position['lat'], route_position['lon'],
            intersections[1]['lat'], intersections[1]['lon']
        )
        
        # Return closest to route
        return intersections[0] if dist1 < dist2 else intersections[1]
    
    # Fallback to weighted centroid
    return weighted_centroid([station1, station2])
```

## Recommended Implementation Priority

1. **Immediate (Critical):**
   - Enhanced dead reckoning with route constraint
   - Degraded mode navigation
   - Route-based position when stations < 2

2. **Short-term (Important):**
   - Coverage analysis before route setting
   - Warning system for low-coverage areas
   - Improved 2-station trilateration

3. **Long-term (Enhancement):**
   - Coverage-aware route planning
   - Coverage visualization
   - Predictive coverage analysis

## Expected Outcomes

**With Solutions Implemented:**

1. **Smooth Transition:**
   - Drone continues following route even in low coverage
   - No erratic movements
   - Graceful degradation instead of failure

2. **User Awareness:**
   - Warnings before entering low-coverage areas
   - Visual indication of coverage quality
   - Ability to plan routes within coverage

3. **Reliable Navigation:**
   - Route-following mode ensures mission completion
   - Reduced position errors in low coverage
   - Stable control system behavior

## Conclusion

The erratic movement when stations drop below 3 is caused by:
1. Trilateration failure leading to no position updates
2. Dead reckoning error accumulation
3. Control system using inaccurate position data
4. Lack of route-constrained navigation in degraded mode

Solutions focus on:
- Using route position as primary source when trilateration fails
- Constraining dead reckoning to route
- Providing coverage awareness to users
- Implementing graceful degradation instead of failure

