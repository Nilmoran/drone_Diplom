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
        
        self.dt = 0.5  # Time step (500ms)
        self.initialized = True
        
        # Maximum velocity constraint (deg/s) - ~10 m/s = ~0.00009 deg/s
        self.max_velocity = 0.0002  # Allow some margin
        
        # Smoothing factor for velocity damping
        self.velocity_damping = 0.85  # Reduced damping to allow faster corrections
        
        # Maximum position jump threshold (in degrees) - ~100 meters
        self.max_jump = 0.0009  # Allow reasonable jumps but prevent outliers
        
        # Previous filtered position for additional smoothing
        self.prev_filtered = [initial_lat, initial_lon]
        self.smoothing_alpha = 0.75  # High alpha to follow route closely (less smoothing lag)
    
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

# --- Drone Flight Simulation ---

def interpolate_position(start, end, progress):
    """Interpolate between two waypoints"""
    return {
        'lat': start['lat'] + (end['lat'] - start['lat']) * progress,
        'lon': start['lon'] + (end['lon'] - start['lon']) * progress
    }

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
    """Main drone flight simulation loop with improved error handling"""
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
                        # Calculate how far to move this iteration (in meters)
                        move_distance = DRONE_SPEED_MPS * 0.5  # Move 5 meters per iteration (10 m/s * 0.5s)
                        
                        # Calculate progress increment as fraction of total segment
                        progress_increment = move_distance / segment_distance
                        
                        # Update segment progress
                        segment_progress = min(1.0, segment_progress + progress_increment)
                        drone_state['segment_progress'][idx] = segment_progress
                        
                        # Calculate new position along route segment (always from route waypoints)
                        new_position = interpolate_position(start, end, segment_progress)
                        
                        # CRITICAL: Always use route-based position for true_position
                        # This ensures the drone follows the route exactly
                        route_position = new_position
                        drone_state['true_position'] = route_position
                        
                        # Debug movement
                        if loop_count % 10 == 0:
                            remaining_distance = calculate_distance_meters(
                                drone_state['true_position']['lat'],
                                drone_state['true_position']['lon'],
                                end['lat'],
                                end['lon']
                            )
                            print(f"Moving: segment {idx}/{len(route)-1}, progress={segment_progress:.3f}, remaining={remaining_distance:.1f}m, route_pos=({drone_state['true_position']['lat']:.6f}, {drone_state['true_position']['lon']:.6f})")
                        
                        # Move to next waypoint when segment is complete
                        if segment_progress >= 1.0:
                            drone_state['route_index'] += 1
                            # Snap to waypoint when reached
                            drone_state['true_position'] = copy_waypoint(end)
                            # Clear progress for this segment
                            if 'segment_progress' in drone_state:
                                drone_state['segment_progress'].pop(idx, None)
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
                        progress = min(1.0, DRONE_SPEED_MPS * 0.5 / distance_to_dest)
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
            await asyncio.sleep(0.5)
            continue
        
        await asyncio.sleep(0.5)  # Update every 500ms

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
                    drone_state['route'] = waypoints
                    drone_state['route_index'] = 0
                    drone_state['is_flying'] = True
                    drone_state['true_position'] = copy_waypoint(waypoints[0])
                    drone_state['position'] = copy_waypoint(waypoints[0])  # Initialize position too
                    drone_state['consecutive_failures'] = 0
                    drone_state['segment_progress'] = {}  # Reset segment progress
                    # Reset Kalman filter for new route
                    drone_state['kalman_filter'] = None
                    print(f"Route received: {len(waypoints)} waypoints")
                    print(f"Starting position: lat={waypoints[0]['lat']}, lon={waypoints[0]['lon']}")
                    print(f"Destination: lat={waypoints[-1]['lat']}, lon={waypoints[-1]['lon']}")
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
