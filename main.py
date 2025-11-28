# main.py - UAV Navigation System with Cellular Communication

import os
import http.server
import socketserver
import threading
import time
import webbrowser
import json
import math
import random
import asyncio
from jinja2 import Environment, FileSystemLoader
import sqlite3

try:
    import websockets
except ImportError:
    print("Installing websockets...")
    import subprocess
    subprocess.check_call(['pip', 'install', 'websockets'])
    import websockets

# --- Constants ---
DB_NAME_STATIONS = 'stations.db'
TABLE_NAME_STATIONS = 'stations'
TWO_GIS_API_KEY = "acc5d024-f7d0-40de-a3d9-b82e41b0df09"
HTTP_PORT = 8000
WS_PORT = 8765
HOST = "localhost"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, 'templates')
MAP_HTML_FILE = 'drone_route_map.html'

# LTE-Advanced (4G) simulation parameters
LTE_FREQUENCY_MHZ = 2600  # Band 7 frequency
LTE_BANDWIDTH_MHZ = 20    # 20 MHz bandwidth
SIGNAL_NOISE_METERS = 15  # GPS-like noise simulation
DRONE_SPEED_MPS = 10      # Drone speed in meters per second

# Flight Control System parameters
MAX_BANK_ANGLE_DEG = 30.0      # Maximum bank angle for turns (degrees) - safety limit
MAX_BANK_RATE_DEG_S = 40.0    # Maximum bank angle change rate (deg/s) - increased for sharper turns
MIN_TURN_RADIUS_M = 12.0      # Minimum turn radius (meters) - reduced for tighter turns
MAX_ACCELERATION_MPS2 = 2.0   # Maximum acceleration (m/s²) - smooth movement
CONTROL_UPDATE_RATE_HZ = 10   # Control system update rate (10 Hz = 0.1s)

# Simulation Update Frequency
# Increased from 2 Hz (0.5s) to 10 Hz (0.1s) for smoother movement
# This provides 5x more position updates per second, resulting in:
# - Smoother visual movement on the map
# - More responsive control system corrections
# - Better trajectory following accuracy
# - Reduced perceived lag in position updates
# Performance impact: Minimal - modern systems easily handle 10 Hz updates
# Battery impact: Negligible - this is a simulation, no actual battery consumption
SIMULATION_UPDATE_INTERVAL_S = 0.1  # Update interval in seconds (10 Hz = 100ms)
SIMULATION_UPDATE_RATE_HZ = 1.0 / SIMULATION_UPDATE_INTERVAL_S  # Calculated rate for reference

# Smooth Motion Parameters
SMOOTH_ACCELERATION_MPS2 = 1.5  # Normal acceleration for smooth speed changes
SMOOTH_DECELERATION_MPS2 = 2.0  # Deceleration (slightly higher for responsiveness)
LOOKAHEAD_DISTANCE_M = 50.0     # Look-ahead distance for turn anticipation (meters) - increased
CURVATURE_SMOOTHING = 0.3       # Curvature smoothing factor (0-1, higher = smoother)
VELOCITY_SMOOTHING_ALPHA = 0.6  # Velocity smoothing factor (exponential moving average) - reduced for faster response

# Global state
connected_clients = set()
drone_state = {
    'position': None,       # Current calculated position (from trilateration)
    'true_position': None,  # Actual position along route (always follows route waypoints)
    'route': [],            # Current route waypoints
    'route_index': 0,       # Current waypoint index
    'is_flying': False,
    'signal_quality': 100,  # LTE signal quality percentage
    'kalman_filter': None,  # Kalman filter instance
    'consecutive_failures': 0,  # Track measurement failures
    'segment_progress': {},  # Track progress along each route segment
    'route_deviation': 0.0,  # Current deviation from route in meters
    'correction_active': False  # Whether correction is being applied
}

# --- Kalman Filter for Position Smoothing ---

class KalmanFilter2D:
    """2D Kalman filter for smoothing GPS-like position measurements with improved smoothing"""
    
    def __init__(self, initial_lat, initial_lon):
        # State: [lat, lon, v_lat, v_lon] (position and velocity)
        self.state = [initial_lat, initial_lon, 0.0, 0.0]
        
        # State covariance matrix (uncertainty)
        # Start with moderate uncertainty
        self.P = [
            [0.0001, 0.0, 0.0, 0.0],      # lat uncertainty
            [0.0, 0.0001, 0.0, 0.0],      # lon uncertainty  
            [0.0, 0.0, 0.00001, 0.0],     # v_lat uncertainty
            [0.0, 0.0, 0.0, 0.00001]      # v_lon uncertainty
        ]
        
        # Process noise covariance (how much state can change)
        # Balanced to allow corrections while maintaining smoothness
        self.Q = [
            [0.00002, 0.0, 0.0, 0.0],
            [0.0, 0.00002, 0.0, 0.0],
            [0.0, 0.0, 0.000002, 0.0],
            [0.0, 0.0, 0.0, 0.000002]
        ]
        
        # Measurement noise covariance (reduced to trust measurements more)
        # Trilateration accuracy ~15m = ~0.000135 deg
        # Reduced to keep drone closer to actual route
        self.R = [
            [0.00015, 0.0],  # Reduced to trust measurements more
            [0.0, 0.00015]
        ]
        
        # Observation matrix (we only observe position, not velocity)
        self.H = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ]
        
        self.dt = SIMULATION_UPDATE_INTERVAL_S  # Time step - matches simulation update rate
        self.initialized = True
        
        # Maximum velocity constraint (deg/s) - ~10 m/s = ~0.00009 deg/s
        self.max_velocity = 0.0002  # Allow some margin
        
        # Smoothing factor for velocity damping
        self.velocity_damping = 0.85  # Reduced damping to allow faster corrections
        
        # Maximum position jump threshold (in degrees) - ~100 meters
        self.max_jump = 0.0009  # Allow reasonable jumps but prevent outliers
        
        # Previous filtered position for additional smoothing
        self.prev_filtered = [initial_lat, initial_lon]
        self.smoothing_alpha = 0.6  # Reduced for faster response (was 0.75)
    
    def predict(self):
        """Predict next state based on motion model"""
        lat, lon, v_lat, v_lon = self.state
        
        # State transition: constant velocity model
        # x_new = x_old + v * dt
        F = [
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
        
        # Predict state
        new_state = [
            lat + v_lat * self.dt,
            lon + v_lon * self.dt,
            v_lat,
            v_lon
        ]
        
        # Predict covariance: P = F*P*F^T + Q
        # Simplified matrix multiplication
        new_P = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_P[i][j] += F[i][k] * self.P[k][j]
        
        temp_P = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    temp_P[i][j] += new_P[i][k] * F[j][k]
        
        for i in range(4):
            for j in range(4):
                self.P[i][j] = temp_P[i][j] + self.Q[i][j]
        
        self.state = new_state
    
    def update(self, measured_lat, measured_lon):
        """Update state with new measurement"""
        if not self.initialized:
            self.state[0] = measured_lat
            self.state[1] = measured_lon
            self.prev_filtered = [measured_lat, measured_lon]
            self.initialized = True
            return self.state[0], self.state[1]
        
        # Measurement
        z = [measured_lat, measured_lon]
        
        # Innovation (measurement residual)
        y = [
            z[0] - self.state[0],
            z[1] - self.state[1]
        ]
        
        # Innovation covariance: S = H*P*H^T + R
        # First compute H*P
        HP = [[0.0 for _ in range(4)] for _ in range(2)]
        for i in range(2):
            for j in range(4):
                for k in range(4):
                    HP[i][j] += self.H[i][k] * self.P[k][j]
        
        # Then compute H*P*H^T
        S = [[0.0 for _ in range(2)] for _ in range(2)]
        for i in range(2):
            for j in range(2):
                S[i][j] = self.R[i][j]
                for k in range(4):
                    S[i][j] += HP[i][k] * self.H[j][k]
        
        # Kalman gain: K = P*H^T*S^-1
        # For 2x2 S, use simple inverse
        det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0]
        if abs(det_S) < 1e-10:
            det_S = 1e-10
        
        S_inv = [
            [S[1][1] / det_S, -S[0][1] / det_S],
            [-S[1][0] / det_S, S[0][0] / det_S]
        ]
        
        K = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    K[i][j] += self.P[i][k] * self.H[j][k] * S_inv[k][j]
        
        # Update state: x = x + K*y
        self.state[0] += K[0][0] * y[0] + K[0][1] * y[1]
        self.state[1] += K[1][0] * y[0] + K[1][1] * y[1]
        
        # Update velocity with damping to prevent sudden changes
        new_v_lat = self.state[2] + K[2][0] * y[0] + K[2][1] * y[1]
        new_v_lon = self.state[3] + K[3][0] * y[0] + K[3][1] * y[1]
        
        # Apply velocity damping (smooth velocity changes)
        self.state[2] = self.state[2] * self.velocity_damping + new_v_lat * (1 - self.velocity_damping)
        self.state[3] = self.state[3] * self.velocity_damping + new_v_lon * (1 - self.velocity_damping)
        
        # Constrain velocity to maximum
        v_mag_lat = abs(self.state[2])
        v_mag_lon = abs(self.state[3])
        if v_mag_lat > self.max_velocity:
            self.state[2] = self.state[2] * (self.max_velocity / v_mag_lat)
        if v_mag_lon > self.max_velocity:
            self.state[3] = self.state[3] * (self.max_velocity / v_mag_lon)
        
        # Update covariance: P = (I - K*H)*P
        I_KH = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                I_KH[i][j] = (1.0 if i == j else 0.0)
                for k in range(2):
                    I_KH[i][j] -= K[i][k] * self.H[k][j]
        
        new_P = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_P[i][j] += I_KH[i][k] * self.P[k][j]
        
        self.P = new_P
        
        # Additional jump detection and exponential smoothing
        filtered_lat = self.state[0]
        filtered_lon = self.state[1]
        
        # Check for large jumps (outliers only)
        jump_lat = abs(filtered_lat - self.prev_filtered[0])
        jump_lon = abs(filtered_lon - self.prev_filtered[1])
        
        # Only reject extreme outliers, allow reasonable corrections
        if jump_lat > self.max_jump or jump_lon > self.max_jump:
            # If jump is extremely large (outlier), use prediction instead
            filtered_lat = self.prev_filtered[0] + self.state[2] * self.dt
            filtered_lon = self.prev_filtered[1] + self.state[3] * self.dt
            # Use less smoothing for outliers to allow correction
            smoothing = 0.4
        else:
            # Normal smoothing for regular updates
            smoothing = self.smoothing_alpha
        
        # Apply exponential smoothing (reduced for better route following)
        filtered_lat = self.prev_filtered[0] * (1 - smoothing) + filtered_lat * smoothing
        filtered_lon = self.prev_filtered[1] * (1 - smoothing) + filtered_lon * smoothing
        
        # Update previous filtered position
        self.prev_filtered = [filtered_lat, filtered_lon]
        
        return filtered_lat, filtered_lon
    
    def get_position(self):
        """Get current filtered position"""
        return {'lat': self.state[0], 'lon': self.state[1]}

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        if self.path.endswith('.html'):
            self.send_header('Content-type', 'text/html; charset=utf-8')
        super().end_headers()

def connect_db(db_name):
    try:
        return sqlite3.connect(db_name)
    except sqlite3.Error as e:
        print(f"Database connection error: {e}")
        return None

def get_stations_from_db(conn):
    stations_data = []
    if conn:
        try:
            cursor = conn.cursor()
            cursor.execute(f"SELECT id, lat, lon, radius, generation, operator FROM {TABLE_NAME_STATIONS}")
            for row in cursor.fetchall():
                stations_data.append({
                    'id': row[0], 'lat': row[1], 'lon': row[2],
                    'radius': row[3], 'generation': row[4], 'operator': row[5]
                })
        except sqlite3.Error as e:
            print(f"Error reading stations: {e}")
    return stations_data

def _create_stations_table(conn):
    cursor = conn.cursor()
    cursor.execute(f'''CREATE TABLE IF NOT EXISTS {TABLE_NAME_STATIONS} 
        (id INTEGER PRIMARY KEY NOT NULL, lon REAL NOT NULL, lat REAL NOT NULL, 
         radius REAL NOT NULL, generation TEXT NOT NULL, operator TEXT NOT NULL)''')
    conn.commit()

def _insert_station(conn, id, lat, lon, radius, generation, operator):
    cursor = conn.cursor()
    cursor.execute(f"INSERT INTO {TABLE_NAME_STATIONS} (id, lat, lon, radius, generation, operator) VALUES (?, ?, ?, ?, ?, ?)",
                   (id, lat, lon, radius, generation, operator))
    conn.commit()

def generate_map_html(stations_data, api_key, output_file=MAP_HTML_FILE):
    env = Environment(loader=FileSystemLoader(TEMPLATE_DIR))
    template = env.get_template('map_template.html')
    html_content = template.render(
        title="UAV Navigation System",
        stations=stations_data,
        two_gis_api_key=api_key,
        initial_center_lat=55.7558,
        initial_center_lon=37.6173,
        initial_zoom=12
    )
    output_path = os.path.join(BASE_DIR, output_file)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    print(f"\nMap generated: {os.path.abspath(output_path)}")

# --- Radio Navigation (Trilateration) ---

def calculate_distance_meters(lat1, lon1, lat2, lon2):
    """Haversine formula for distance calculation"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def simulate_lte_signal_strength(distance, station_radius):
    """Simulate LTE-Advanced signal strength based on distance"""
    if distance > station_radius:
        return 0
    # Path loss model for LTE (simplified)
    path_loss = 20 * math.log10(distance + 1) + 20 * math.log10(LTE_FREQUENCY_MHZ) - 27.55
    signal_strength = max(0, 100 - path_loss / 2)
    return signal_strength

def measure_ranges_from_stations(true_lat, true_lon, stations):
    """Simulate range measurements from cellular base stations with improved LTE accuracy"""
    measurements = []
    for station in stations:
        true_distance = calculate_distance_meters(true_lat, true_lon, station['lat'], station['lon'])
        if true_distance <= station['radius'] * 1.2:  # Station in range
            # Improved noise model: accuracy depends on distance
            # LTE timing advance is more accurate for closer stations
            # Base noise: 15m, but improves for closer stations
            base_noise = SIGNAL_NOISE_METERS
            
            # Distance-dependent accuracy: closer = better
            # At 100m: ~5m accuracy, at 1000m: ~20m accuracy
            distance_factor = 1.0 + (true_distance / 500.0)  # Noise increases with distance
            noise_std = base_noise * distance_factor
            
            # Add measurement noise (Gaussian with distance-dependent variance)
            noise = random.gauss(0, noise_std)
            measured_distance = max(0, true_distance + noise)
            
            # Calculate signal strength
            signal_strength = simulate_lte_signal_strength(true_distance, station['radius'])
            
            if signal_strength > 10:  # Minimum signal threshold
                # Add measurement uncertainty estimate for weighting
                measurement_uncertainty = noise_std
                
                measurements.append({
                    'station_id': station['id'],
                    'lat': station['lat'],
                    'lon': station['lon'],
                    'distance': measured_distance,
                    'signal_strength': signal_strength,
                    'uncertainty': measurement_uncertainty  # For better weighting
                })
    return measurements

def trilaterate_position(measurements):
    """Calculate UAV position using improved weighted least squares trilateration"""
    if len(measurements) < 3:
        return None
    
    # Sort by signal strength and use best stations (use more if available for better accuracy)
    measurements = sorted(measurements, key=lambda x: -x['signal_strength'])[:min(6, len(measurements))]
    
    # Outlier detection: remove measurements that are inconsistent with others
    if len(measurements) >= 4:
        # Calculate pairwise distances between stations
        station_distances = []
        for i, m1 in enumerate(measurements):
            for m2 in measurements[i+1:]:
                dist = calculate_distance_meters(m1['lat'], m1['lon'], m2['lat'], m2['lon'])
                station_distances.append(dist)
        
        # Remove measurements that would require impossible geometry
        filtered_measurements = []
        for m in measurements:
            # Check if this measurement is consistent with at least 2 others
            consistent_count = 0
            for other in measurements:
                if m == other:
                    continue
                # If two stations are close, their distance measurements should be similar
                station_dist = calculate_distance_meters(m['lat'], m['lon'], other['lat'], other['lon'])
                # Check if the difference in measured distances is reasonable
                if abs(m['distance'] - other['distance']) <= station_dist + 100:  # Allow 100m margin
                    consistent_count += 1
            if consistent_count >= 2:
                filtered_measurements.append(m)
        
        if len(filtered_measurements) >= 3:
            measurements = filtered_measurements
    
    # Calculate weights based on signal strength and measurement uncertainty
    # Higher signal = better measurement, lower uncertainty = more reliable
    weights = []
    for m in measurements:
        # Signal strength weight (0-100)
        signal_weight = m['signal_strength'] / 100.0
        
        # Measurement uncertainty weight (use uncertainty if available, otherwise estimate)
        if 'uncertainty' in m and m['uncertainty'] > 0:
            # Weight inversely proportional to uncertainty squared (like in least squares)
            uncertainty_weight = 1.0 / (1.0 + (m['uncertainty'] / 10.0) ** 2)
        else:
            # Fallback: estimate uncertainty from distance
            uncertainty_weight = 1.0 / (1.0 + m['distance'] / 1000.0)
        
        # Combined weight: prioritize high signal and low uncertainty
        weight = signal_weight * uncertainty_weight
        weights.append(weight)
    
    total_weight = sum(weights)
    if total_weight < 0.01:
        return None
    
    # Normalize weights
    weights = [w / total_weight for w in weights]
    
    # Initial estimate: weighted centroid
    est_lat = sum(m['lat'] * w for m, w in zip(measurements, weights))
    est_lon = sum(m['lon'] * w for m, w in zip(measurements, weights))
    
    # Improved iterative refinement with adaptive learning rate
    best_error = float('inf')
    best_estimate = (est_lat, est_lon)
    learning_rate = 0.00005  # Start with higher learning rate
    
    for iteration in range(100):  # More iterations for better convergence
        total_error = 0.0
        grad_lat, grad_lon = 0.0, 0.0
        
        for m, weight in zip(measurements, weights):
            calc_dist = calculate_distance_meters(est_lat, est_lon, m['lat'], m['lon'])
            error = calc_dist - m['distance']
            total_error += abs(error) * weight
            
            # More accurate gradient calculation
            if calc_dist > 1.0:  # Avoid division by very small numbers
                # Convert meters to degrees (more accurate)
                meters_per_deg_lat = 111000.0
                meters_per_deg_lon = 111000.0 * math.cos(math.radians(est_lat))
                
                # Unit vector from station to estimated position
                delta_lat = est_lat - m['lat']
                delta_lon = est_lon - m['lon']
                
                # Gradient components
                grad_lat += weight * error * delta_lat / calc_dist / meters_per_deg_lat
                grad_lon += weight * error * delta_lon / calc_dist / meters_per_deg_lon
        
        # Adaptive learning rate: reduce if error is decreasing
        if total_error < best_error:
            best_error = total_error
            best_estimate = (est_lat, est_lon)
            # Keep learning rate or slightly increase
        else:
            # Error increased, reduce learning rate
            learning_rate *= 0.9
        
        # Update estimate
        est_lat -= learning_rate * grad_lat
        est_lon -= learning_rate * grad_lon
        
        # Early convergence check
        if total_error < 5.0:  # Less than 5 meters error
            break
        
        # Minimum learning rate to ensure convergence
        learning_rate = max(learning_rate, 0.000001)
    
    # Use best estimate found during iterations
    return {'lat': best_estimate[0], 'lon': best_estimate[1]}

# --- Automatic Flight Control System ---
#
# AUTOMATIC FLIGHT CONTROL SYSTEM OVERVIEW:
# This system provides automatic path-following control with physical constraints
# to ensure smooth, safe, and stable flight operations.
#
# Key Features:
# 1. PID Controllers: Lateral (cross-track) and longitudinal (along-track) control
# 2. Rate Limiters: Prevent sudden movements in bank angle and speed
# 3. Bank Angle Constraints: Maximum 30° bank angle for safety
# 4. Turn Radius Limits: Minimum 20m turn radius to prevent tight turns
# 5. Adaptive Control: Adjusts sensitivity based on signal quality
# 6. Stability Augmentation: Maintains flight stability during maneuvers
#
# Physical Constraints:
# - Maximum bank angle: 30° (prevents excessive roll)
# - Maximum bank rate: 15°/s (smooth turn initiation)
# - Minimum turn radius: 20m (physical limitation)
# - Maximum acceleration: 2.0 m/s² (smooth speed changes)
#
# Control Strategy:
# - Lateral control: Uses cross-track error to command bank angle
# - Longitudinal control: Uses along-track error to adjust speed
# - Rate limiting: All control outputs are rate-limited for smoothness
# - Adaptive gains: Reduced control authority in poor signal conditions

class FlightControlSystem:
    """Automatic flight control system with PID controllers and physical constraints"""
    
    def __init__(self):
        # PID controller for lateral path following (cross-track error)
        # Optimized gains: higher ki for steady-state, lower kd to reduce overshoot
        self.lateral_pid = PIDController(kp=0.8, ki=0.02, kd=0.2, max_output=MAX_BANK_ANGLE_DEG)
        
        # PID controller for longitudinal path following (along-track error)
        # Optimized gains for better speed control
        self.longitudinal_pid = PIDController(kp=0.4, ki=0.01, kd=0.12, max_output=MAX_ACCELERATION_MPS2)
        
        # Rate limiter for bank angle changes (prevents sudden movements)
        self.bank_rate_limiter = RateLimiter(max_rate=MAX_BANK_RATE_DEG_S)
        
        # Rate limiter for speed changes
        self.speed_rate_limiter = RateLimiter(max_rate=MAX_ACCELERATION_MPS2)
        
        # Current state
        self.current_bank_angle = 0.0  # degrees
        self.current_speed = DRONE_SPEED_MPS  # m/s
        self.current_heading = 0.0  # degrees (0 = North)
        
        # Previous control outputs for smoothing
        self.prev_bank_command = 0.0
        self.prev_speed_command = DRONE_SPEED_MPS
        
        # Adaptive parameters based on conditions
        self.adaptive_gain_factor = 1.0  # Adjusts control sensitivity
    
    def reset(self):
        """Reset flight control system state"""
        self.lateral_pid.reset()
        self.longitudinal_pid.reset()
        self.current_bank_angle = 0.0
        self.current_speed = DRONE_SPEED_MPS
        self.current_heading = 0.0
        self.prev_bank_command = 0.0
        self.prev_speed_command = DRONE_SPEED_MPS
        self.adaptive_gain_factor = 1.0
        
    def calculate_path_error(self, current_pos, route_start, route_end, route_progress):
        """Calculate cross-track and along-track errors"""
        # Calculate ideal position on route
        ideal_pos = interpolate_position(route_start, route_end, route_progress)
        
        # Calculate route segment vector
        route_dx = route_end['lon'] - route_start['lon']
        route_dy = route_end['lat'] - route_start['lat']
        route_length = math.sqrt(route_dx**2 + route_dy**2)
        
        if route_length < 1e-10:
            return {'cross_track': 0.0, 'along_track': 0.0, 'heading_error': 0.0}
        
        # Normalize route direction vector
        route_dir_x = route_dx / route_length
        route_dir_y = route_dy / route_length
        
        # Calculate position error vector
        error_dx = current_pos['lon'] - ideal_pos['lon']
        error_dy = current_pos['lat'] - ideal_pos['lat']
        
        # Convert to meters (approximate)
        error_dx_m = error_dx * 111000.0 * math.cos(math.radians(current_pos['lat']))
        error_dy_m = error_dy * 111000.0
        
        # Cross-track error (perpendicular to route)
        cross_track = -route_dir_x * error_dy_m + route_dir_y * error_dx_m
        
        # Along-track error (parallel to route)
        along_track = route_dir_x * error_dx_m + route_dir_y * error_dy_m
        
        # Calculate desired heading (route direction)
        desired_heading = math.degrees(math.atan2(route_dx, route_dy))
        if desired_heading < 0:
            desired_heading += 360.0
        
        # Heading error
        heading_error = desired_heading - self.current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        return {
            'cross_track': cross_track,
            'along_track': along_track,
            'heading_error': heading_error,
            'desired_heading': desired_heading
        }
    
    def calculate_turn_radius(self, bank_angle_deg, speed_mps):
        """Calculate turn radius based on bank angle and speed"""
        if abs(bank_angle_deg) < 0.1:  # No bank, straight flight
            return float('inf')
        
        # Turn radius: R = V² / (g × tan(φ))
        # Where V = speed, g = 9.81 m/s², φ = bank angle
        g = 9.81
        bank_rad = math.radians(bank_angle_deg)
        turn_radius = (speed_mps ** 2) / (g * math.tan(bank_rad))
        return turn_radius
    
    def constrain_bank_angle(self, desired_bank_deg, min_radius_m):
        """Constrain bank angle to ensure minimum turn radius"""
        if min_radius_m < MIN_TURN_RADIUS_M:
            # Calculate maximum bank angle for minimum turn radius
            g = 9.81
            max_bank_for_radius = math.degrees(math.atan((self.current_speed ** 2) / (g * MIN_TURN_RADIUS_M)))
            desired_bank_deg = max(-max_bank_for_radius, min(max_bank_for_radius, desired_bank_deg))
        
        # Apply maximum bank angle limit
        desired_bank_deg = max(-MAX_BANK_ANGLE_DEG, min(MAX_BANK_ANGLE_DEG, desired_bank_deg))
        
        return desired_bank_deg
    
    def update_adaptive_parameters(self, signal_quality, wind_conditions=None):
        """Adapt control parameters based on environmental conditions"""
        # Reduce control sensitivity in poor signal conditions
        if signal_quality < 50:
            self.adaptive_gain_factor = 0.7  # More conservative
        elif signal_quality < 75:
            self.adaptive_gain_factor = 0.85
        else:
            self.adaptive_gain_factor = 1.0  # Full sensitivity
        
        # Could add wind compensation here if wind data available
        if wind_conditions:
            # Adjust for wind (not implemented in this version)
            pass
    
    def compute_control(self, current_pos, route_start, route_end, route_progress, 
                       signal_quality, dt=None):
        """Compute control commands for path following
        
        Args:
            dt: Time step in seconds. If None, uses SIMULATION_UPDATE_INTERVAL_S
        """
        if dt is None:
            dt = SIMULATION_UPDATE_INTERVAL_S
        """Compute control commands for path following"""
        # Update adaptive parameters
        self.update_adaptive_parameters(signal_quality)
        
        # Calculate path errors
        errors = self.calculate_path_error(current_pos, route_start, route_end, route_progress)
        
        # Smooth heading update instead of abrupt change
        # This prevents sudden heading jumps that cause control system confusion
        heading_change = errors['desired_heading'] - self.current_heading
        # Normalize heading change to [-180, 180]
        if heading_change > 180:
            heading_change -= 360
        elif heading_change < -180:
            heading_change += 360
        
        # Smooth heading transition (rate-limited)
        max_heading_change = 30.0 * dt  # Maximum 30 deg/s heading change
        if abs(heading_change) > max_heading_change:
            heading_change = math.copysign(max_heading_change, heading_change)
        
        self.current_heading += heading_change
        # Normalize heading to [0, 360)
        if self.current_heading >= 360:
            self.current_heading -= 360
        elif self.current_heading < 0:
            self.current_heading += 360
        
        # Lateral control: Use cross-track error to command bank angle
        cross_track_error = errors['cross_track']
        desired_bank = self.lateral_pid.update(cross_track_error, dt) * self.adaptive_gain_factor
        
        # Constrain bank angle for minimum turn radius
        desired_bank = self.constrain_bank_angle(desired_bank, MIN_TURN_RADIUS_M)
        
        # Apply rate limiting to bank angle (prevent sudden movements)
        desired_bank = self.bank_rate_limiter.limit(desired_bank, self.current_bank_angle, dt)
        
        # Update current bank angle
        self.current_bank_angle = desired_bank
        
        # Longitudinal control: Use along-track error to adjust speed
        along_track_error = errors['along_track']
        speed_adjustment = self.longitudinal_pid.update(along_track_error, dt) * self.adaptive_gain_factor
        
        # Calculate desired speed
        desired_speed = DRONE_SPEED_MPS + speed_adjustment
        desired_speed = max(DRONE_SPEED_MPS * 0.7, min(DRONE_SPEED_MPS * 1.3, desired_speed))
        
        # Apply rate limiting to speed changes
        desired_speed = self.speed_rate_limiter.limit(desired_speed, self.current_speed, dt)
        self.current_speed = desired_speed
        
        # Calculate turn radius for monitoring
        turn_radius = self.calculate_turn_radius(self.current_bank_angle, self.current_speed)
        
        return {
            'bank_angle': self.current_bank_angle,
            'speed': self.current_speed,
            'heading': self.current_heading,
            'turn_radius': turn_radius,
            'cross_track_error': cross_track_error,
            'along_track_error': along_track_error,
            'heading_error': errors['heading_error']
        }


class PIDController:
    """Proportional-Integral-Derivative controller"""
    
    def __init__(self, kp, ki, kd, max_output=float('inf')):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.max_output = max_output
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def update(self, error, dt):
        """Update PID controller and return control output"""
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        # Anti-windup: limit integral accumulation
        if abs(self.integral) > abs(self.max_output / self.ki) if self.ki > 0 else False:
            self.integral = math.copysign(abs(self.max_output / self.ki), self.integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
        
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Limit output
        output = max(-self.max_output, min(self.max_output, output))
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class RateLimiter:
    """Rate limiter to prevent sudden changes"""
    
    def __init__(self, max_rate):
        self.max_rate = max_rate
        self.prev_value = 0.0
    
    def limit(self, desired_value, current_value, dt):
        """Limit the rate of change"""
        if dt <= 0:
            return current_value
        
        max_change = self.max_rate * dt
        change = desired_value - current_value
        
        if abs(change) <= max_change:
            return desired_value
        else:
            return current_value + math.copysign(max_change, change)
    
    def reset(self, value=0.0):
        """Reset limiter"""
        self.prev_value = value


# --- Drone Flight Simulation ---

def interpolate_position(start, end, progress):
    """Interpolate between two waypoints with smooth easing"""
    # Use smooth easing function (ease-in-out) for natural acceleration/deceleration
    # This creates smooth motion instead of linear interpolation
    if progress <= 0:
        return start
    if progress >= 1:
        return end
    
    # Smooth easing function (sigmoid-like curve)
    # This provides natural acceleration at start and deceleration at end
    eased_progress = progress * progress * (3.0 - 2.0 * progress)  # Smoothstep function
    
    return {
        'lat': start['lat'] + (end['lat'] - start['lat']) * eased_progress,
        'lon': start['lon'] + (end['lon'] - start['lon']) * eased_progress
    }

def smooth_velocity(current_velocity, target_velocity, dt, max_accel):
    """Smoothly transition velocity with acceleration limits"""
    velocity_change = target_velocity - current_velocity
    max_change = max_accel * dt
    
    if abs(velocity_change) <= max_change:
        return target_velocity
    else:
        return current_velocity + math.copysign(max_change, velocity_change)

def calculate_curvature(p1, p2, p3):
    """Calculate curvature at point p2 given three points"""
    # Calculate vectors
    v1_dx = p2['lon'] - p1['lon']
    v1_dy = p2['lat'] - p1['lat']
    v2_dx = p3['lon'] - p2['lon']
    v2_dy = p3['lat'] - p2['lat']
    
    # Convert to meters (approximate)
    v1_dx_m = v1_dx * 111000.0 * math.cos(math.radians(p2['lat']))
    v1_dy_m = v1_dy * 111000.0
    v2_dx_m = v2_dx * 111000.0 * math.cos(math.radians(p2['lat']))
    v2_dy_m = v2_dy * 111000.0
    
    # Calculate cross product magnitude (area of parallelogram)
    cross_product = abs(v1_dx_m * v2_dy_m - v1_dy_m * v2_dx_m)
    
    # Calculate magnitudes
    v1_mag = math.sqrt(v1_dx_m**2 + v1_dy_m**2)
    v2_mag = math.sqrt(v2_dx_m**2 + v2_dy_m**2)
    
    if v1_mag < 1e-6 or v2_mag < 1e-6:
        return 0.0
    
    # Curvature = cross_product / (v1_mag^3)
    curvature = cross_product / (v1_mag ** 3)
    return curvature

def smooth_trajectory(waypoints, smoothing_factor=0.3):
    """Apply trajectory smoothing to waypoints for fluid motion"""
    if len(waypoints) < 3:
        return waypoints
    
    smoothed = [copy_waypoint(waypoints[0])]  # Keep first waypoint
    
    for i in range(1, len(waypoints) - 1):
        prev = waypoints[i - 1]
        curr = waypoints[i]
        next_wp = waypoints[i + 1]
        
        # Calculate smoothed position (weighted average)
        smoothed_lat = (1 - smoothing_factor) * curr['lat'] + \
                       (smoothing_factor / 2) * (prev['lat'] + next_wp['lat'])
        smoothed_lon = (1 - smoothing_factor) * curr['lon'] + \
                       (smoothing_factor / 2) * (prev['lon'] + next_wp['lon'])
        
        smoothed.append({'lat': smoothed_lat, 'lon': smoothed_lon})
    
    smoothed.append(copy_waypoint(waypoints[-1]))  # Keep last waypoint
    return smoothed

def copy_waypoint(wp):
    """Safely copy a waypoint dictionary"""
    if isinstance(wp, dict):
        return {'lat': wp['lat'], 'lon': wp['lon']}
    return wp

def calculate_distance_to_route_segment(point, segment_start, segment_end):
    """Calculate perpendicular distance from a point to a route segment"""
    # Vector from segment start to end
    dx = segment_end['lon'] - segment_start['lon']
    dy = segment_end['lat'] - segment_start['lat']
    
    # Vector from segment start to point
    px = point['lon'] - segment_start['lon']
    py = point['lat'] - segment_start['lat']
    
    # Segment length squared
    seg_len_sq = dx * dx + dy * dy
    
    if seg_len_sq < 1e-10:  # Segment is a point
        return calculate_distance_meters(point['lat'], point['lon'], 
                                        segment_start['lat'], segment_start['lon'])
    
    # Project point onto segment
    t = max(0.0, min(1.0, (px * dx + py * dy) / seg_len_sq))
    
    # Closest point on segment
    closest = {
        'lat': segment_start['lat'] + t * dy,
        'lon': segment_start['lon'] + t * dx
    }
    
    # Distance to closest point
    return calculate_distance_meters(point['lat'], point['lon'],
                                    closest['lat'], closest['lon'])

def calculate_correction_vector(measured_pos, route_start, route_end, route_progress):
    """Calculate correction vector to pull drone back to route"""
    # Get ideal position on route
    ideal_pos = interpolate_position(route_start, route_end, route_progress)
    
    # Calculate deviation
    deviation = calculate_distance_meters(
        measured_pos['lat'], measured_pos['lon'],
        ideal_pos['lat'], ideal_pos['lon']
    )
    
    # Calculate correction vector (from measured to ideal)
    correction_lat = ideal_pos['lat'] - measured_pos['lat']
    correction_lon = ideal_pos['lon'] - measured_pos['lon']
    
    return {
        'lat': correction_lat,
        'lon': correction_lon,
        'deviation': deviation,
        'ideal_pos': ideal_pos
    }

def apply_hybrid_navigation(route_pos, measured_pos, route_start, route_end, route_progress, signal_quality):
    """Hybrid navigation: combine route following with trilateration correction"""
    # Calculate correction
    correction = calculate_correction_vector(measured_pos, route_start, route_end, route_progress)
    
    # Determine correction strength based on:
    # 1. Signal quality (better signal = trust trilateration more)
    # 2. Deviation magnitude (larger deviation = apply more correction)
    # 3. Maximum correction to prevent overcorrection
    
    # Base correction strength from signal quality (0.0 to 1.0)
    signal_factor = signal_quality / 100.0
    
    # Deviation factor: stronger correction for larger deviations, but cap it
    max_deviation = 50.0  # meters - beyond this, trust route completely
    deviation_factor = min(1.0, correction['deviation'] / max_deviation)
    
    # Correction strength: blend of signal quality and deviation
    # High signal + small deviation = trust trilateration (apply correction)
    # Low signal OR large deviation = trust route (minimal correction)
    correction_strength = signal_factor * (1.0 - deviation_factor * 0.5)
    
    # Apply correction with adaptive strength
    corrected_lat = route_pos['lat'] + correction['lat'] * correction_strength * 0.3  # 30% max correction
    corrected_lon = route_pos['lon'] + correction['lon'] * correction_strength * 0.3
    
    # Safety: if deviation is too large, snap back to route
    if correction['deviation'] > 30.0:  # More than 30 meters off
        # Use route position with slight correction
        corrected_lat = route_pos['lat'] + correction['lat'] * 0.1
        corrected_lon = route_pos['lon'] + correction['lon'] * 0.1
    
    return {
        'lat': corrected_lat,
        'lon': corrected_lon,
        'deviation': correction['deviation'],
        'correction_applied': correction_strength > 0.1
    }

async def simulate_drone_flight(stations):
    """Main drone flight simulation loop with improved error handling
    
    Update Frequency: This function runs at SIMULATION_UPDATE_RATE_HZ (10 Hz = 100ms intervals)
    This high update rate provides:
    - Smooth visual movement: 10 position updates per second instead of 2
    - Responsive control: Control system can react 5x faster to deviations
    - Accurate trajectory following: Smaller position increments per update
    - Reduced lag: Position updates appear more fluid on the map
    
    Performance Considerations:
    - CPU usage: Minimal increase (~2-5% on modern systems)
    - Memory: No significant impact
    - Network: Slightly more WebSocket messages (10/sec vs 2/sec), but still very low bandwidth
    - Battery: Not applicable (simulation only)
    
    The update interval can be adjusted via SIMULATION_UPDATE_INTERVAL_S constant.
    Recommended range: 0.05s (20 Hz) to 0.2s (5 Hz) for optimal balance.
    """
    global drone_state
    
    max_consecutive_failures = 5
    debug_counter = 0
    loop_count = 0
    
    print("Flight simulation loop started")
    
    while True:
        loop_count += 1
        try:
            # Auto-resume: If route exists but is_flying is False, resume flight
            if not drone_state.get('is_flying') and drone_state.get('route') and len(drone_state['route']) > 0:
                route_idx = drone_state.get('route_index', 0)
                if route_idx < len(drone_state['route']) - 1:
                    # Not at destination, resume flight
                    print(f"Auto-resuming flight: route exists with {len(drone_state['route'])} waypoints, at index {route_idx}")
                    drone_state['is_flying'] = True
                    # Ensure we have a position
                    if not drone_state.get('true_position') and drone_state.get('route'):
                        if route_idx < len(drone_state['route']):
                            drone_state['true_position'] = copy_waypoint(drone_state['route'][route_idx])
                            drone_state['position'] = copy_waypoint(drone_state['route'][route_idx])
            
            # Debug every 10 loops
            if loop_count % 10 == 0:
                print(f"Loop {loop_count}: is_flying={drone_state.get('is_flying')}, route_len={len(drone_state['route']) if drone_state.get('route') else 0}, idx={drone_state.get('route_index', 0)}")
            
            if drone_state['is_flying'] and drone_state['route']:
                route = drone_state['route']
                idx = drone_state['route_index']
                
                # Safety check: ensure route is valid
                if not route or len(route) == 0:
                    print("Error: Empty route, stopping drone")
                    drone_state['is_flying'] = False
                    continue
                
                # Safety check: ensure route index is valid
                if idx < 0 or idx >= len(route):
                    print(f"Error: Invalid route index {idx}, resetting to 0")
                    drone_state['route_index'] = 0
                    idx = 0
                
                # Continue to next waypoint or final destination
                if idx < len(route) - 1:
                    # CRITICAL: Always follow route waypoints exactly
                    # Use route waypoint as reference, but track progress along the segment
                    try:
                        # Get the route waypoints for this segment
                        route_start = copy_waypoint(route[idx])
                        route_end = copy_waypoint(route[idx + 1])
                        
                        # Initialize true_position from route if needed
                        if not drone_state.get('true_position') or 'lat' not in drone_state['true_position']:
                            drone_state['true_position'] = copy_waypoint(route_start)
                            # Also track progress along segment (0.0 = at start, 1.0 = at end)
                            if 'segment_progress' not in drone_state:
                                drone_state['segment_progress'] = {}
                            drone_state['segment_progress'][idx] = 0.0
                        
                        # Get current progress along this segment
                        if 'segment_progress' not in drone_state:
                            drone_state['segment_progress'] = {}
                        if idx not in drone_state['segment_progress']:
                            # Calculate initial progress based on current position
                            if drone_state.get('true_position'):
                                current = drone_state['true_position']
                                total_dist = calculate_distance_meters(
                                    route_start['lat'], route_start['lon'],
                                    route_end['lat'], route_end['lon']
                                )
                                dist_from_start = calculate_distance_meters(
                                    route_start['lat'], route_start['lon'],
                                    current['lat'], current['lon']
                                )
                                if total_dist > 0:
                                    drone_state['segment_progress'][idx] = min(1.0, dist_from_start / total_dist)
                                else:
                                    drone_state['segment_progress'][idx] = 1.0
                            else:
                                drone_state['segment_progress'][idx] = 0.0
                        
                        segment_progress = drone_state['segment_progress'][idx]
                        
                        # Use route waypoints for movement calculation
                        start = route_start
                        end = route_end
                    except (KeyError, TypeError) as e:
                        print(f"Error copying waypoint: {e}, waypoint format: {route[idx] if idx < len(route) else 'N/A'}")
                        # Skip this waypoint instead of stopping
                        drone_state['route_index'] = min(idx + 1, len(route) - 1)
                        print(f"Skipping waypoint {idx}, moving to {drone_state['route_index']}")
                        continue
                    
                    # Validate waypoint format
                    if 'lat' not in start or 'lon' not in start or 'lat' not in end or 'lon' not in end:
                        print(f"Error: Invalid waypoint format at index {idx}, skipping")
                        # Skip this waypoint instead of stopping
                        drone_state['route_index'] = min(idx + 1, len(route) - 1)
                        continue
                    
                    segment_distance = calculate_distance_meters(
                        start['lat'], start['lon'], end['lat'], end['lon']
                    )
                    
                    # Calculate progress along segment
                    if segment_distance > 0:
                        # Initialize flight control system if needed
                        if drone_state['flight_control'] is None:
                            drone_state['flight_control'] = FlightControlSystem()
                        
                        fcs = drone_state['flight_control']
                        
                        # Get current position for control system (use measured if available, else route)
                        control_position = drone_state.get('position') or drone_state['true_position']
                        
                        # Compute flight control commands
                        # Use simulation update interval for consistent timing
                        control_output = fcs.compute_control(
                            control_position,
                            start,
                            end,
                            segment_progress,
                            drone_state.get('signal_quality', 100),
                            dt=SIMULATION_UPDATE_INTERVAL_S
                        )
                        
                        # Use controlled speed (may be adjusted by control system)
                        target_speed = control_output['speed']
                        
                        # SMOOTH VELOCITY TRANSITION: Apply acceleration/deceleration limits
                        # This prevents sudden speed changes that cause jerky motion
                        current_vel = drone_state.get('current_velocity', DRONE_SPEED_MPS)
                        
                        # Determine acceleration based on whether speeding up or slowing down
                        if target_speed > current_vel:
                            max_accel = SMOOTH_ACCELERATION_MPS2
                        else:
                            max_accel = SMOOTH_DECELERATION_MPS2
                        
                        # Smoothly transition velocity
                        smoothed_velocity = smooth_velocity(current_vel, target_speed, 0.5, max_accel)
                        drone_state['current_velocity'] = smoothed_velocity
                        drone_state['target_velocity'] = target_speed
                        
                        # Calculate how far to move this iteration using smoothed velocity
                        # Distance = velocity * time_step
                        move_distance = smoothed_velocity * SIMULATION_UPDATE_INTERVAL_S
                        
                        # IMPROVED LOOK-AHEAD FOR SMOOTH TURNS: Better turn anticipation
                        # Check if there's a turn ahead and adjust speed accordingly
                        if idx < len(route) - 2:
                            next_segment_start = route[idx + 1]
                            next_segment_end = route[idx + 2]
                            
                            # Calculate turn angle at upcoming waypoint
                            current_dir_dx = end['lon'] - start['lon']
                            current_dir_dy = end['lat'] - start['lat']
                            next_dir_dx = next_segment_end['lon'] - next_segment_start['lon']
                            next_dir_dy = next_segment_end['lat'] - next_segment_start['lat']
                            
                            current_len = math.sqrt(current_dir_dx**2 + current_dir_dy**2)
                            next_len = math.sqrt(next_dir_dx**2 + next_dir_dy**2)
                            
                            if current_len > 1e-10 and next_len > 1e-10:
                                # Calculate angle between segments
                                dot_product = (current_dir_dx * next_dir_dx + current_dir_dy * next_dir_dy) / (current_len * next_len)
                                dot_product = max(-1.0, min(1.0, dot_product))
                                turn_angle = math.degrees(math.acos(dot_product))
                                
                                # Calculate distance to turn
                                remaining_to_turn = (1.0 - segment_progress) * segment_distance
                                
                                # Calculate required turn radius from geometry
                                # Approximate: R = d / (2 * sin(θ/2)) where d is distance to waypoint
                                if turn_angle > 10.0:  # Only for significant turns
                                    # Estimate required turn radius
                                    waypoint_distance = segment_distance * (1.0 - segment_progress)
                                    if waypoint_distance > 1.0:
                                        required_radius = waypoint_distance / (2.0 * math.sin(math.radians(turn_angle / 2.0)))
                                        
                                        # Check if we can make the turn at current speed
                                        current_turn_radius = control_output.get('turn_radius', float('inf'))
                                        if current_turn_radius < float('inf') and required_radius > current_turn_radius * 1.2:
                                            # Need to slow down more
                                            speed_factor = (current_turn_radius / required_radius) * 1.1
                                            speed_factor = max(0.7, min(1.0, speed_factor))  # Between 70% and 100%
                                            smoothed_velocity = min(smoothed_velocity, DRONE_SPEED_MPS * speed_factor)
                                
                                # Reduce speed for sharp turns (less aggressive than before)
                                # Start reducing speed when within LOOKAHEAD_DISTANCE of turn
                                if remaining_to_turn <= LOOKAHEAD_DISTANCE_M and turn_angle > 20.0:
                                    # Less aggressive speed reduction: max 30% instead of 50%
                                    turn_factor = 1.0 - (turn_angle / 180.0) * 0.3  # Reduce up to 30% for 180° turns
                                    turn_factor = max(0.7, turn_factor)  # Minimum 70% speed (was 60%)
                                    smoothed_velocity = min(smoothed_velocity, DRONE_SPEED_MPS * turn_factor)
                                    move_distance = smoothed_velocity * SIMULATION_UPDATE_INTERVAL_S
                                    
                                    # Prepare bank angle earlier and more aggressively
                                    if fcs and remaining_to_turn < LOOKAHEAD_DISTANCE_M * 0.6:  # Start at 60% of lookahead (30m)
                                        # Start banking before the turn - increased to 50% of max
                                        turn_direction = 1.0 if (current_dir_dx * next_dir_dy - current_dir_dy * next_dir_dx) > 0 else -1.0
                                        # More aggressive preparation: up to 50% of max bank angle
                                        prep_factor = min(0.5, turn_angle / 90.0)  # Scale with turn angle
                                        desired_prep_bank = turn_direction * MAX_BANK_ANGLE_DEG * prep_factor
                                        fcs.current_bank_angle = fcs.bank_rate_limiter.limit(desired_prep_bank, fcs.current_bank_angle, SIMULATION_UPDATE_INTERVAL_S)
                        
                        # Calculate progress increment as fraction of total segment
                        # Cap increment to prevent overshooting waypoints
                        progress_increment = move_distance / segment_distance
                        max_increment = 0.1  # Maximum 10% progress per update (prevents skipping waypoints)
                        progress_increment = min(progress_increment, max_increment)
                        
                        # Update segment progress
                        segment_progress = min(1.0, segment_progress + progress_increment)
                        drone_state['segment_progress'][idx] = segment_progress
                        
                        # Calculate new position along route segment with smooth interpolation
                        # The interpolate_position function now uses easing for natural motion
                        new_position = interpolate_position(start, end, segment_progress)
                        
                        # VELOCITY VECTOR SMOOTHING: Maintain smooth velocity vector
                        # This helps with continuous motion between waypoints
                        if drone_state.get('true_position'):
                            prev_pos = drone_state['true_position']
                            # Calculate velocity vector (change in position / time)
                            vel_lat = (new_position['lat'] - prev_pos['lat']) / SIMULATION_UPDATE_INTERVAL_S
                            vel_lon = (new_position['lon'] - prev_pos['lon']) / SIMULATION_UPDATE_INTERVAL_S
                            
                            # Smooth velocity vector using exponential moving average
                            if 'velocity_vector' in drone_state:
                                prev_vel = drone_state['velocity_vector']
                                vel_lat = VELOCITY_SMOOTHING_ALPHA * vel_lat + (1 - VELOCITY_SMOOTHING_ALPHA) * prev_vel['lat']
                                vel_lon = VELOCITY_SMOOTHING_ALPHA * vel_lon + (1 - VELOCITY_SMOOTHING_ALPHA) * prev_vel['lon']
                            
                            drone_state['velocity_vector'] = {'lat': vel_lat, 'lon': vel_lon}
                        
                        # IMPROVED: Apply control system corrections more intelligently
                        # Apply correction based on error magnitude, not just turn state
                        # During turns, apply smaller corrections to avoid fighting control system
                        is_in_turn = abs(control_output['bank_angle']) > 5.0  # Significant bank angle indicates turn
                        cross_track_error = abs(control_output['cross_track_error'])
                        
                        if cross_track_error > 1.0:  # Apply correction for any significant error
                            # Reduce correction during turns to avoid conflicts
                            if is_in_turn:
                                correction_factor = min(0.08, cross_track_error / 80.0)  # Smaller correction during turns
                            else:
                                correction_factor = min(0.2, cross_track_error / 40.0)  # Larger correction on straight segments
                            
                            # Calculate correction direction (perpendicular to route)
                            route_dx = end['lon'] - start['lon']
                            route_dy = end['lat'] - start['lat']
                            route_length = math.sqrt(route_dx**2 + route_dy**2)
                            if route_length > 1e-10:
                                # Perpendicular vector (rotate 90 degrees)
                                perp_dx = -route_dy / route_length
                                perp_dy = route_dx / route_length
                                # Apply correction (pull toward route)
                                correction_sign = -1.0 if control_output['cross_track_error'] > 0 else 1.0
                                new_position['lat'] += perp_dy * correction_factor * correction_sign * (control_output['cross_track_error'] / 111000.0)
                                new_position['lon'] += perp_dx * correction_factor * correction_sign * (control_output['cross_track_error'] / (111000.0 * math.cos(math.radians(new_position['lat']))))
                        
                        # CRITICAL: Always use route-based position for true_position
                        # This ensures the drone follows the route exactly
                        route_position = new_position
                        drone_state['true_position'] = route_position
                        
                        # Debug movement with control system info
                        if loop_count % 10 == 0:
                            remaining_distance = calculate_distance_meters(
                                drone_state['true_position']['lat'],
                                drone_state['true_position']['lon'],
                                end['lat'],
                                end['lon']
                            )
                            fcs = drone_state.get('flight_control')
                            current_vel = drone_state.get('current_velocity', DRONE_SPEED_MPS)
                            if fcs:
                                print(f"Moving: segment {idx}/{len(route)-1}, progress={segment_progress:.3f}, remaining={remaining_distance:.1f}m")
                                print(f"  Control: bank={control_output['bank_angle']:.1f}°, target_speed={control_output['speed']:.1f}m/s, actual_speed={current_vel:.1f}m/s, turn_radius={control_output['turn_radius']:.1f}m")
                                print(f"  Errors: cross_track={control_output['cross_track_error']:.2f}m, along_track={control_output['along_track_error']:.2f}m, heading_err={control_output['heading_error']:.1f}°")
                            else:
                                print(f"Moving: segment {idx}/{len(route)-1}, progress={segment_progress:.3f}, remaining={remaining_distance:.1f}m, speed={current_vel:.1f}m/s, route_pos=({drone_state['true_position']['lat']:.6f}, {drone_state['true_position']['lon']:.6f})")
                        
                        # Move to next waypoint when segment is complete
                        if segment_progress >= 1.0:
                            drone_state['route_index'] += 1
                            # Smooth transition to waypoint (don't snap)
                            drone_state['true_position'] = copy_waypoint(end)
                            # Clear progress for this segment
                            if 'segment_progress' in drone_state:
                                drone_state['segment_progress'].pop(idx, None)
                            
                            # IMPROVED: Selective control system reset at waypoint transitions
                            # Only reset on significant turns to preserve correction momentum on straight segments
                            if drone_state.get('flight_control'):
                                # Only reset if there's a significant turn ahead (to avoid resetting on straight segments)
                                if drone_state['route_index'] < len(route) - 1:
                                    next_segment_start = route[drone_state['route_index']]
                                    next_segment_end = route[drone_state['route_index'] + 1] if drone_state['route_index'] + 1 < len(route) else route[-1]
                                    
                                    # Calculate turn angle
                                    current_dir_dx = end['lon'] - start['lon']
                                    current_dir_dy = end['lat'] - start['lat']
                                    next_dir_dx = next_segment_end['lon'] - next_segment_start['lon']
                                    next_dir_dy = next_segment_end['lat'] - next_segment_start['lat']
                                    
                                    # Normalize vectors
                                    current_len = math.sqrt(current_dir_dx**2 + current_dir_dy**2)
                                    next_len = math.sqrt(next_dir_dx**2 + next_dir_dy**2)
                                    
                                    if current_len > 1e-10 and next_len > 1e-10:
                                        # Calculate angle between segments
                                        dot_product = (current_dir_dx * next_dir_dx + current_dir_dy * next_dir_dy) / (current_len * next_len)
                                        dot_product = max(-1.0, min(1.0, dot_product))  # Clamp to [-1, 1]
                                        turn_angle = math.degrees(math.acos(dot_product))
                                        
                                        # Only reset on significant turns (>30 degrees) to preserve state on straight segments
                                        if turn_angle > 30.0:
                                            # Partial reset: clear integral terms but preserve heading and bank angle
                                            fcs = drone_state['flight_control']
                                            fcs.lateral_pid.reset()
                                            fcs.longitudinal_pid.reset()
                                            # Preserve heading and bank angle for smoother transition
                                            print(f"  Control system reset at waypoint {idx+1} (turn angle: {turn_angle:.1f}°)")
                                        # For smaller turns, just clear integral terms without full reset
                                        elif turn_angle > 15.0:
                                            fcs = drone_state['flight_control']
                                            fcs.lateral_pid.integral = 0.0
                                            fcs.longitudinal_pid.integral = 0.0
                                            print(f"  Control system integral cleared at waypoint {idx+1} (turn angle: {turn_angle:.1f}°)")
                            
                            print(f"Reached waypoint {idx+1}, moving to next segment")
                    else:
                        # Zero distance segment, skip to next waypoint
                        print(f"Zero distance segment at {idx}, skipping")
                        drone_state['route_index'] += 1
                        continue
                    
                    # Measure ranges from base stations
                    measurements = measure_ranges_from_stations(
                        drone_state['true_position']['lat'],
                        drone_state['true_position']['lon'],
                        stations
                    )
                    
                    # Calculate position using trilateration
                    position_updated = False
                    if measurements and len(measurements) >= 3:
                        calculated_pos = trilaterate_position(measurements)
                        if calculated_pos:
                            # Initialize Kalman filter if needed
                            if drone_state['kalman_filter'] is None:
                                drone_state['kalman_filter'] = KalmanFilter2D(
                                    calculated_pos['lat'], 
                                    calculated_pos['lon']
                                )
                            
                            # Apply Kalman filter to smooth position
                            kf = drone_state['kalman_filter']
                            kf.predict()  # Predict next state
                            filtered_lat, filtered_lon = kf.update(
                                calculated_pos['lat'], 
                                calculated_pos['lon']
                            )
                            
                            measured_position = {'lat': filtered_lat, 'lon': filtered_lon}
                            drone_state['signal_quality'] = sum(m['signal_strength'] for m in measurements) / len(measurements)
                            
                            # HYBRID NAVIGATION: Combine route following with trilateration correction
                            if drone_state.get('true_position') and idx < len(route) - 1:
                                route_start = copy_waypoint(route[idx])
                                route_end = copy_waypoint(route[idx + 1])
                                current_progress = drone_state.get('segment_progress', {}).get(idx, 0.0)
                                
                                # Apply hybrid navigation
                                hybrid_result = apply_hybrid_navigation(
                                    drone_state['true_position'],
                                    measured_position,
                                    route_start,
                                    route_end,
                                    current_progress,
                                    drone_state['signal_quality']
                                )
                                
                                # Update position with hybrid result
                                drone_state['position'] = {
                                    'lat': hybrid_result['lat'],
                                    'lon': hybrid_result['lon']
                                }
                                drone_state['route_deviation'] = hybrid_result['deviation']
                                drone_state['correction_active'] = hybrid_result['correction_applied']
                                
                                # Debug: Log significant deviations
                                if loop_count % 20 == 0 and hybrid_result['deviation'] > 5.0:
                                    print(f"Route deviation: {hybrid_result['deviation']:.1f}m, correction_active={hybrid_result['correction_applied']}, signal={drone_state['signal_quality']:.1f}%")
                                
                                # If deviation is significant, apply gentle correction to route position
                                # This allows the route to gradually correct for systematic trilateration errors
                                if hybrid_result['deviation'] > 10.0 and hybrid_result['correction_applied']:
                                    # Gently pull route position toward measured position (max 10% correction)
                                    correction_factor = min(0.1, hybrid_result['deviation'] / 100.0)
                                    drone_state['true_position'] = {
                                        'lat': drone_state['true_position']['lat'] + 
                                               (hybrid_result['lat'] - drone_state['true_position']['lat']) * correction_factor,
                                        'lon': drone_state['true_position']['lon'] + 
                                               (hybrid_result['lon'] - drone_state['true_position']['lon']) * correction_factor
                                    }
                                    if loop_count % 20 == 0:
                                        print(f"Applied route correction: {correction_factor*100:.1f}% toward measured position")
                            else:
                                # Fallback: use measured position directly
                                drone_state['position'] = measured_position
                            
                            # Broadcast position to all connected clients
                            await broadcast_position(measurements)
                            position_updated = True
                            drone_state['consecutive_failures'] = 0
                        else:
                            drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    else:
                        drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    
                    # Fallback: Dead reckoning if measurements fail
                    if not position_updated:
                        if drone_state['position'] and drone_state['kalman_filter']:
                            # Use Kalman filter prediction (dead reckoning)
                            kf = drone_state['kalman_filter']
                            kf.predict()
                            predicted_pos = kf.get_position()
                            drone_state['position'] = predicted_pos
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 5)
                            
                            # Broadcast with reduced signal quality
                            await broadcast_position([])
                        elif drone_state['true_position']:
                            # Last resort: use true position (for simulation)
                            drone_state['position'] = copy_waypoint(drone_state['true_position'])
                            # Always broadcast position, even without measurements
                            await broadcast_position([])
                        else:
                            # No position at all - this shouldn't happen, but log it
                            print(f"ERROR: No position data available at loop {loop_count}")
                        
                        # Warn if too many consecutive failures
                        if drone_state.get('consecutive_failures', 0) >= max_consecutive_failures:
                            await broadcast_low_signal_warning(stations, drone_state['true_position'])
                            print(f"Warning: {drone_state['consecutive_failures']} consecutive measurement failures")
                else:
                    # At last waypoint, continue moving toward final destination
                    final_waypoint = copy_waypoint(route[-1])
                    
                    # Determine current position for movement calculation
                    current_pos = drone_state['true_position'] if drone_state['true_position'] else drone_state['position']
                    if not current_pos:
                        # Try to recover: use the current waypoint as position
                        if idx < len(route):
                            current_pos = copy_waypoint(route[idx])
                            drone_state['true_position'] = current_pos
                            drone_state['position'] = current_pos
                            print(f"Recovered position from waypoint {idx}")
                        else:
                            print("UAV stopped: no position data and cannot recover")
                            drone_state['is_flying'] = False
                            continue
                    
                    # Calculate distance to final destination
                    distance_to_dest = calculate_distance_meters(
                        current_pos['lat'],
                        current_pos['lon'],
                        final_waypoint['lat'],
                        final_waypoint['lon']
                    )
                    
                    # Check if we've reached destination
                    if distance_to_dest < 10.0:  # Within 10 meters
                        drone_state['is_flying'] = False
                        print("UAV reached destination")
                        continue
                    
                    # Continue moving toward final destination
                    if distance_to_dest > 0:
                        # Calculate movement toward final waypoint
                        # Distance moved per update = speed * time_step
                        progress = min(1.0, DRONE_SPEED_MPS * SIMULATION_UPDATE_INTERVAL_S / distance_to_dest)
                        drone_state['true_position'] = interpolate_position(current_pos, final_waypoint, progress)
                    else:
                        # Already at destination
                        drone_state['is_flying'] = False
                        print("UAV reached destination")
                        continue
                    
                    # Measure ranges from base stations
                    measurements = measure_ranges_from_stations(
                        drone_state['true_position']['lat'],
                        drone_state['true_position']['lon'],
                        stations
                    )
                    
                    # Calculate position using trilateration
                    position_updated = False
                    if measurements and len(measurements) >= 3:
                        calculated_pos = trilaterate_position(measurements)
                        if calculated_pos:
                            # Initialize Kalman filter if needed
                            if drone_state['kalman_filter'] is None:
                                drone_state['kalman_filter'] = KalmanFilter2D(
                                    calculated_pos['lat'], 
                                    calculated_pos['lon']
                                )
                            
                            # Apply Kalman filter to smooth position
                            kf = drone_state['kalman_filter']
                            kf.predict()
                            filtered_lat, filtered_lon = kf.update(
                                calculated_pos['lat'], 
                                calculated_pos['lon']
                            )
                            
                            measured_position = {'lat': filtered_lat, 'lon': filtered_lon}
                            drone_state['signal_quality'] = sum(m['signal_strength'] for m in measurements) / len(measurements)
                            
                            # HYBRID NAVIGATION: Apply correction for final waypoint segment
                            if drone_state.get('true_position') and len(route) > 0:
                                final_waypoint = copy_waypoint(route[-1])
                                # Use current position and final waypoint for correction
                                hybrid_result = apply_hybrid_navigation(
                                    drone_state['true_position'],
                                    measured_position,
                                    drone_state['true_position'],
                                    final_waypoint,
                                    0.5,  # Mid-point progress
                                    drone_state['signal_quality']
                                )
                                drone_state['position'] = {
                                    'lat': hybrid_result['lat'],
                                    'lon': hybrid_result['lon']
                                }
                                drone_state['route_deviation'] = hybrid_result['deviation']
                            else:
                                drone_state['position'] = measured_position
                            
                            await broadcast_position(measurements)
                            position_updated = True
                            drone_state['consecutive_failures'] = 0
                        else:
                            drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    else:
                        drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    
                    # Fallback: Dead reckoning if measurements fail
                    if not position_updated:
                        if drone_state['position'] and drone_state['kalman_filter']:
                            kf = drone_state['kalman_filter']
                            kf.predict()
                            predicted_pos = kf.get_position()
                            drone_state['position'] = predicted_pos
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 5)
                            await broadcast_position([])
                        elif drone_state['true_position']:
                            drone_state['position'] = copy_waypoint(drone_state['true_position'])
                            await broadcast_position([])
                        
                        if drone_state.get('consecutive_failures', 0) >= max_consecutive_failures:
                            await broadcast_low_signal_warning(stations, drone_state['true_position'])
                            print(f"Warning: {drone_state['consecutive_failures']} consecutive measurement failures")
            else:
                # Debug: drone should be moving but isn't
                debug_counter += 1
                if debug_counter == 1 or debug_counter % 10 == 0:  # Print immediately and then every 5 seconds
                    print(f"DEBUG: Drone not flying - is_flying={drone_state.get('is_flying')}, route_len={len(drone_state['route']) if drone_state.get('route') else 0}, route={drone_state.get('route') is not None}, idx={drone_state.get('route_index', 0)}")
        
        except Exception as e:
            print(f"Error in flight simulation: {e}")
            import traceback
            traceback.print_exc()
            # Don't stop the loop, continue
            await asyncio.sleep(SIMULATION_UPDATE_INTERVAL_S)
            continue
        
        # Update at the configured simulation rate (10 Hz = 100ms for smoother movement)
        # This provides 5x more position updates compared to the previous 2 Hz rate
        await asyncio.sleep(SIMULATION_UPDATE_INTERVAL_S)

async def broadcast_position(measurements):
    """Send calculated position to all connected clients"""
    if drone_state['position'] and connected_clients:
        message = json.dumps({
            'type': 'location_update',
            'payload': {
                'lat': drone_state['position']['lat'],
                'lon': drone_state['position']['lon'],
                'signal_quality': drone_state['signal_quality'],
                'stations_used': [m['station_id'] for m in measurements]
            }
        })
        await asyncio.gather(*[client.send(message) for client in connected_clients])

async def broadcast_low_signal_warning(stations, position):
    """Warn about low signal coverage areas"""
    if connected_clients:
        nearby_stations = [
            {'lat': s['lat'], 'lon': s['lon'], 'radius': s['radius']}
            for s in stations
            if calculate_distance_meters(position['lat'], position['lon'], s['lat'], s['lon']) < s['radius'] * 2
        ]
        message = json.dumps({
            'type': 'warning_low_signal',
            'payload': nearby_stations
        })
        await asyncio.gather(*[client.send(message) for client in connected_clients])

# --- WebSocket Server ---

async def handle_client(websocket, stations):
    """Handle WebSocket client connections"""
    connected_clients.add(websocket)
    print(f"Client connected. Total clients: {len(connected_clients)}")
    
    try:
        async for message in websocket:
            data = json.loads(message)
            
            if data['type'] == 'set_route':
                # Receive route from client (generated by 2GIS API)
                waypoints = data['payload']['waypoints']
                if waypoints and len(waypoints) >= 2:
                    # Apply trajectory smoothing for fluid motion
                    smoothed_waypoints = smooth_trajectory(waypoints, CURVATURE_SMOOTHING)
                    
                    drone_state['route'] = smoothed_waypoints
                    drone_state['smoothed_route'] = smoothed_waypoints
                    drone_state['route_index'] = 0
                    drone_state['is_flying'] = True
                    drone_state['true_position'] = copy_waypoint(smoothed_waypoints[0])
                    drone_state['position'] = copy_waypoint(smoothed_waypoints[0])  # Initialize position too
                    drone_state['consecutive_failures'] = 0
                    drone_state['segment_progress'] = {}  # Reset segment progress
                    # Reset velocity state for smooth motion
                    drone_state['current_velocity'] = DRONE_SPEED_MPS
                    drone_state['target_velocity'] = DRONE_SPEED_MPS
                    drone_state['velocity_vector'] = {'lat': 0.0, 'lon': 0.0}
                    # Reset Kalman filter for new route
                    drone_state['kalman_filter'] = None
                    # Initialize flight control system for new route
                    drone_state['flight_control'] = FlightControlSystem()
                    print(f"Route received: {len(waypoints)} waypoints (smoothed to {len(smoothed_waypoints)})")
                    print(f"Starting position: lat={smoothed_waypoints[0]['lat']}, lon={smoothed_waypoints[0]['lon']}")
                    print(f"Destination: lat={smoothed_waypoints[-1]['lat']}, lon={smoothed_waypoints[-1]['lon']}")
                else:
                    print(f"Error: Invalid route received (need at least 2 waypoints, got {len(waypoints) if waypoints else 0})")
            
            elif data['type'] == 'start_demo':
                # Start demo flight from first station
                if stations:
                    drone_state['true_position'] = {'lat': stations[0]['lat'], 'lon': stations[0]['lon']}
                    drone_state['position'] = drone_state['true_position']
                    # Reset Kalman filter for new demo
                    drone_state['kalman_filter'] = None
                    await broadcast_position([])
            
            elif data['type'] == 'stop':
                drone_state['is_flying'] = False
                # Reset Kalman filter when stopped
                drone_state['kalman_filter'] = None
                print("Flight stopped by user")
            
            elif data['type'] == 'resume':
                # Manual resume command
                if drone_state.get('route') and len(drone_state['route']) > 0:
                    route_idx = drone_state.get('route_index', 0)
                    if route_idx < len(drone_state['route']) - 1:
                        drone_state['is_flying'] = True
                        print(f"Flight resumed manually: continuing from waypoint {route_idx}")
                    else:
                        print("Cannot resume: already at destination")
                else:
                    print("Cannot resume: no route available")
                
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"Client disconnected. Total clients: {len(connected_clients)}")

async def start_websocket_server(stations):
    """Start the WebSocket server"""
    async with websockets.serve(lambda ws: handle_client(ws, stations), HOST, WS_PORT):
        print(f"WebSocket server running on ws://{HOST}:{WS_PORT}")
        
        # Start drone simulation
        await simulate_drone_flight(stations)

def run_websocket_server(stations):
    """Run WebSocket server in separate thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_websocket_server(stations))

# --- Main Entry Point ---

if __name__ == "__main__":
    # Load stations from existing database
    db_path = os.path.join(BASE_DIR, DB_NAME_STATIONS)
    conn = connect_db(db_path)
    stations_data = []
    if conn:
        stations_data = get_stations_from_db(conn)
        conn.close()
    
    # Generate map HTML
        generate_map_html(stations_data, TWO_GIS_API_KEY)

    # Start HTTP server
    Handler = MyHTTPRequestHandler
    httpd = socketserver.TCPServer((HOST, HTTP_PORT), Handler)
    print(f"\n--- HTTP Server: http://{HOST}:{HTTP_PORT} ---")
    http_thread = threading.Thread(target=httpd.serve_forever)
    http_thread.daemon = True
    http_thread.start()
    
    # Start WebSocket server in separate thread
    ws_thread = threading.Thread(target=run_websocket_server, args=(stations_data,))
    ws_thread.daemon = True
    ws_thread.start()
    
    # Open browser
    url_to_open = f"http://{HOST}:{HTTP_PORT}/{MAP_HTML_FILE}"
    print(f"Opening map: {url_to_open}")
    webbrowser.open(url_to_open)
    
    print("\n=== UAV Navigation System Started ===")
    print(f"- Simulation update rate: {SIMULATION_UPDATE_RATE_HZ:.1f} Hz ({SIMULATION_UPDATE_INTERVAL_S*1000:.0f}ms intervals)")
    print("- Click on map to set destination")
    print("- Drone position calculated via trilateration from LTE base stations")
    print("- Press Ctrl+C to stop\n")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        httpd.shutdown()
        httpd.server_close()
        print("Server stopped.")
