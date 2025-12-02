# Drone Accuracy Analysis: Trilateration vs Route-Following

## Problem Statement

**Observed Behavior:**
- ✅ **WITHOUT ground stations**: Drone flies perfectly along trajectory
- ❌ **WITH ground stations**: Drone exhibits significant inaccuracies

**Expected Behavior:**
- ✅ **WITH ground stations**: Should improve accuracy via trilateration
- ⚠️ **WITHOUT ground stations**: Should degrade gracefully but still follow route

## Root Cause Analysis

### 1. **Trilateration Measurement Noise**

**Location**: `measure_ranges_from_stations()` (lines 531-595)

**Problem**:
- Base noise: 15 meters (`SIGNAL_NOISE_METERS = 15`)
- Distance-dependent noise: `noise_std = base_noise * (1.0 + distance/500.0)`
- At 100m: ~18m noise
- At 1000m: ~45m noise
- **Result**: Trilateration measurements have 15-45m random error

**Code**:
```python
base_noise = SIGNAL_NOISE_METERS  # 15m
distance_factor = 1.0 + (true_distance / 500.0)
noise_std = base_noise * distance_factor
noise = random.gauss(0, noise_std)  # Random error
measured_distance = max(0, true_distance + noise)
```

### 2. **Hybrid Navigation Over-Trusts Trilateration**

**Location**: `apply_hybrid_navigation()` (lines 1106-1143)

**Problem**:
- Correction strength based on signal quality: `signal_factor = signal_quality / 100.0`
- With high signal quality (80-100%), applies up to 30% correction
- **Issue**: High signal quality doesn't mean low measurement error!
- Signal quality is based on signal strength, not measurement accuracy
- Trilateration can have 20-30m error even with high signal quality

**Code**:
```python
signal_factor = signal_quality / 100.0  # 0.8-1.0 for good signal
correction_strength = signal_factor * (1.0 - deviation_factor * 0.5)
# Applies up to 30% correction (line 1129-1130)
corrected_lat = route_pos['lat'] + correction['lat'] * correction_strength * 0.3
```

**Result**: Noisy trilateration measurements pull drone away from accurate route

### 3. **Feedback Loop: Route Correction**

**Location**: Lines 1659-1667

**Problem**:
- When trilateration shows large deviation (>10m), system pulls route toward measured position
- This creates a feedback loop:
  1. Trilateration has noise → shows drone 20m off route
  2. Hybrid navigation applies correction → moves drone toward noisy measurement
  3. Route correction pulls route toward noisy measurement
  4. Next cycle: route is now wrong, trilateration shows different error
  5. Process repeats → drone oscillates around route

**Code**:
```python
if hybrid_result['deviation'] > 10.0 and hybrid_result['correction_applied']:
    # Pulls route toward measured position (which has noise!)
    correction_factor = min(0.1, hybrid_result['deviation'] / 100.0)
    drone_state['true_position'] = {
        'lat': drone_state['true_position']['lat'] + 
               (hybrid_result['lat'] - drone_state['true_position']['lat']) * correction_factor,
        ...
    }
```

### 4. **Route-Constrained Dead Reckoning (Works Perfectly)**

**Location**: Lines 1700-1740 (fallback when stations < 2)

**Why It Works**:
- Uses 85% route position, 15% Kalman prediction
- If deviation > 30m, snaps to route
- **No trilateration noise** → position stays on accurate route
- Route is the ground truth → following it exactly = perfect accuracy

**Code**:
```python
# Blend: 85% route, 15% Kalman (trust route more in low coverage)
blended_pos = {
    'lat': route_pos['lat'] * 0.85 + kf_pos['lat'] * 0.15,
    'lon': route_pos['lon'] * 0.85 + kf_pos['lon'] * 0.15
}
# If >30m off route, snap to route
if route_deviation > 30.0:
    drone_state['position'] = route_pos  # Perfect route following!
```

## Mathematical Analysis

### Trilateration Error Model

**Measurement Error**:
```
σ_measurement = 15m × (1 + distance/500m)
```

**Position Error** (from 3+ stations):
```
σ_position ≈ σ_measurement / √N_stations
```

**Example**:
- 3 stations at 500m: σ_measurement ≈ 30m, σ_position ≈ 17m
- 5 stations at 200m: σ_measurement ≈ 21m, σ_position ≈ 9m

**But**: Optimization may not converge perfectly, adding more error.

### Hybrid Navigation Correction

**Current Formula**:
```
correction = (measured_pos - route_pos) × signal_factor × 0.3
```

**Problem**: `signal_factor` doesn't account for measurement uncertainty!

**Should Be**:
```
correction = (measured_pos - route_pos) × trust_factor × 0.3
where trust_factor = f(signal_quality, measurement_uncertainty, deviation)
```

## Solutions

### Solution 1: Trust Route When Trilateration Deviates

**Principle**: If trilateration shows large deviation, it's likely measurement error, not route error.

**Implementation**:
- Reduce correction strength when deviation > threshold
- Trust route more when trilateration conflicts with route
- Don't apply route corrections based on noisy measurements

### Solution 2: Use Measurement Uncertainty in Trust Calculation

**Principle**: Signal quality ≠ measurement accuracy

**Implementation**:
- Calculate measurement uncertainty from noise model
- Use uncertainty to weight correction strength
- High uncertainty → low trust → minimal correction

### Solution 3: Remove Route Correction Feedback Loop

**Principle**: Route is ground truth, don't modify it based on noisy measurements

**Implementation**:
- Remove or significantly reduce route correction logic
- Only apply route corrections when deviation is consistently large over time
- Use moving average of deviations, not single measurement

### Solution 4: Increase Route Constraint in Hybrid Mode

**Principle**: Even with trilateration, keep position close to route

**Implementation**:
- Reduce maximum correction from 30% to 10-15%
- Add route constraint: if hybrid result deviates >20m from route, snap to route
- Trust route more when trilateration shows large deviations

## Recommended Fix Priority

1. **Immediate (Critical)**: 
   - Remove route correction feedback loop (lines 1659-1667)
   - Reduce correction strength when deviation > 15m
   - Add route constraint in hybrid navigation

2. **Short-term (Important)**:
   - Use measurement uncertainty in trust calculation
   - Reduce maximum correction from 30% to 15%
   - Add deviation-based trust factor

3. **Long-term (Enhancement)**:
   - Implement moving average of deviations
   - Add trilateration quality metric (not just signal strength)
   - Adaptive correction based on historical accuracy

## Expected Outcomes

**After Fixes**:
- ✅ **WITH stations**: Drone follows route accurately (within 5-10m)
- ✅ **WITHOUT stations**: Drone continues following route (within 10-15m)
- ✅ No feedback loops or oscillations
- ✅ Smooth transitions between coverage areas

