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
import csv
from datetime import datetime
from jinja2 import Environment, FileSystemLoader
import sqlite3

try:
    import websockets
except ImportError:
    print("Installing websockets...")
    import subprocess
    subprocess.check_call(['pip', 'install', 'websockets'])
    import websockets

try:
    import numpy as np
    from scipy.optimize import minimize
except ImportError:
    print("Installing numpy and scipy for advanced trilateration...")
    import subprocess
    subprocess.check_call(['pip', 'install', 'numpy', 'scipy'])
    import numpy as np
    from scipy.optimize import minimize

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

# Predefined drone starting locations in Moscow
MOSCOW_DRONE_LOCATIONS = [
    {'id': 'rechnoy_vokzal', 'name': 'Rechnoy Vokzal', 'lat': 55.8364, 'lon': 37.5376, 'description': 'SAO - Rechnoy Vokzal'},
    {'id': 'kremlin', 'name': 'Kremlin', 'lat': 55.7520, 'lon': 37.6156, 'description': 'Moscow Kremlin'},
    {'id': 'sparrow_hills', 'name': 'Sparrow Hills', 'lat': 55.7108, 'lon': 37.5532, 'description': 'Sparrow Hills observation point'},
    {'id': 'vdnh', 'name': 'VDNKh', 'lat': 55.8304, 'lon': 37.6214, 'description': 'VDNKh Exhibition Center'},
    {'id': 'kutuzovsky', 'name': 'Kutuzovsky Prospect', 'lat': 55.7444, 'lon': 37.5364, 'description': 'Kutuzovsky Prospect area'},
    {'id': 'tverskaya', 'name': 'Tverskaya Street', 'lat': 55.7558, 'lon': 37.6056, 'description': 'Tverskaya Street - main street'},
    {'id': 'gorky_park', 'name': 'Gorky Park', 'lat': 55.7320, 'lon': 37.6014, 'description': 'Gorky Central Park of Culture'},
    {'id': 'ostankino', 'name': 'Ostankino Tower', 'lat': 55.8197, 'lon': 37.6117, 'description': 'Ostankino TV Tower area'},
    {'id': 'luzhniki', 'name': 'Luzhniki Stadium', 'lat': 55.7158, 'lon': 37.5536, 'description': 'Luzhniki Olympic Complex'},
    {'id': 'sheremetyevo', 'name': 'Sheremetyevo Airport', 'lat': 55.9736, 'lon': 37.4145, 'description': 'Sheremetyevo International Airport'},
    {'id': 'domodedovo', 'name': 'Domodedovo Airport', 'lat': 55.4143, 'lon': 37.9005, 'description': 'Domodedovo Airport area'},
    {'id': 'izmailovo', 'name': 'Izmailovo Park', 'lat': 55.7879, 'lon': 37.7704, 'description': 'Izmailovo Park and Market'},
    {'id': 'kolomenskoye', 'name': 'Kolomenskoye', 'lat': 55.6670, 'lon': 37.6680, 'description': 'Kolomenskoye Museum-Reserve'},
    {'id': 'tsaritsyno', 'name': 'Tsaritsyno Park', 'lat': 55.6200, 'lon': 37.6820, 'description': 'Tsaritsyno Museum-Reserve'},
]

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
    'correction_active': False,  # Whether correction is being applied
    'selected_start_location': None  # Selected starting location from predefined list
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
        initial_zoom=12,
        drone_locations=MOSCOW_DRONE_LOCATIONS
    )
    output_path = os.path.join(BASE_DIR, output_file)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    print(f"\nMap generated: {os.path.abspath(output_path)}")

# --- Radio Navigation (Trilateration) ---

# Station Selection Parameters
MAX_TOWERS_TO_SELECT = 7  # Maximum number of towers to select for calculation
METERS_IN_DEGREE = 111000.0  # Approximate meters in one degree of latitude

def calculate_distance(point_1, point_2):
    """Calculate Euclidean distance between points (in degrees)."""
    return np.sqrt((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)

def select_diverse_towers(towers, num_to_select):
    """
    Select N most diverse (spread out) towers from the list.
    
    Uses a greedy algorithm to maximize minimum distance between selected towers.
    This ensures better geometric diversity for trilateration accuracy.
    
    Args:
        towers: List of tower tuples (id, lon, lat, radius, generation, operator, signal)
        num_to_select: Maximum number of towers to select
        
    Returns:
        List of selected tower tuples
    """
    if not towers or len(towers) <= num_to_select:
        return towers
    
    selected_towers = [towers[0]]
    remaining_towers = towers[1:]
    
    while len(selected_towers) < num_to_select and remaining_towers:
        best_candidate, max_min_dist = None, -1
        
        for candidate in remaining_towers:
            candidate_coords = (candidate[1], candidate[2])  # (lon, lat)
            min_dist_to_selected = min([
                calculate_distance(candidate_coords, (s[1], s[2])) 
                for s in selected_towers
            ])
            
            if min_dist_to_selected > max_min_dist:
                max_min_dist, best_candidate = min_dist_to_selected, candidate
        
        if best_candidate:
            selected_towers.append(best_candidate)
            remaining_towers.remove(best_candidate)
    
    return selected_towers

def find_weighted_intersection_center(towers_with_signals):
    """
    Find a point within the intersection area, weighted toward towers with stronger signals.
    
    Uses constrained optimization to find the optimal position that:
    1. Minimizes weighted sum of squared distances (weighted by signal strength)
    2. Ensures the point is within all tower coverage radii
    
    Args:
        towers_with_signals: List of tuples (id, lon, lat, radius_meters, generation, operator, signal_strength)
        
    Returns:
        Dictionary with 'point' (lon, lat) tuple and 'success' boolean
    """
    if not towers_with_signals:
        return {'point': None, 'success': False}
    
    # Convert all data to comparable units (degrees)
    towers_in_degrees = []
    for id, lon, lat, r_meters, g, o, signal in towers_with_signals:
        r_degrees = r_meters / METERS_IN_DEGREE
        towers_in_degrees.append((id, lon, lat, r_degrees, g, o, signal))
    
    # Objective function: minimize weighted sum of squared distances (in degrees)
    # Higher signal strength = higher weight = point moves closer to that tower
    def objective(point):
        return np.sum([
            signal * ((point[0] - lon)**2 + (point[1] - lat)**2) 
            for id, lon, lat, r_deg, g, o, signal in towers_in_degrees
        ])
    
    # Constraints: point must be within radius of each tower (in degrees)
    # FIXED: Lambda closure bug - use default arguments to capture values correctly
    constraints = []
    for id, lon, lat, r_deg, g, o, signal in towers_in_degrees:
        constraints.append({
            'type': 'ineq', 
            'fun': lambda p, r=r_deg, lon_val=lon, lat_val=lat: r**2 - ((p[0]-lon_val)**2 + (p[1]-lat_val)**2)
        })
    
    # Initial guess: mean of all tower positions
    initial_guess = np.mean(np.array([(c[1], c[2]) for c in towers_in_degrees]), axis=0)
    
    try:
        # FIXED: Use more lenient tolerance and options to reduce optimization failures
        result = minimize(
            objective, 
            initial_guess, 
            method='SLSQP', 
            constraints=constraints, 
            tol=1e-6,  # Reduced from 1e-9 for better convergence
            options={'maxiter': 100, 'ftol': 1e-6}  # Limit iterations and tolerance
        )
        
        if result.success:
            final_point = result.x
            
            # Additional validation: check constraints with small tolerance
            for const in constraints:
                if const['fun'](final_point) < -1e-6:
                    print(f"Warning: Optimizer reported success but point violates constraints. Using centroid fallback.")
                    # Fallback to simple centroid
                    centers = np.array([(c[1], c[2]) for c in towers_in_degrees])
                    return {'point': tuple(np.mean(centers, axis=0)), 'success': True}
            
            return {'point': tuple(final_point), 'success': True}
        else:
            print(f"Optimization failed: {result.message}. Using centroid fallback.")
            # Fallback to simple centroid
            centers = np.array([(c[1], c[2]) for c in towers_in_degrees])
            return {'point': tuple(np.mean(centers, axis=0)), 'success': True}
    except Exception as e:
        print(f"Optimization error: {e}. Using centroid fallback.")
        # Fallback to simple centroid
        centers = np.array([(c[1], c[2]) for c in towers_in_degrees])
        return {'point': tuple(np.mean(centers, axis=0)), 'success': True}

def calculate_distance_meters(lat1, lon1, lat2, lon2):
    """Haversine formula for distance calculation"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def simulate_lte_signal_strength(distance, station_radius):
    """Simulate LTE-Advanced signal strength based on distance with urban path loss"""
    if distance > station_radius:
        return 0
    
    # Urban path loss model for LTE (учитывает городские условия)
    # Free space path loss: 20*log10(d) + 20*log10(f) - 27.55
    free_space_loss = 20 * math.log10(distance + 1) + 20 * math.log10(LTE_FREQUENCY_MHZ) - 27.55
    
    # Дополнительные городские потери (из-за зданий, препятствий, отражений)
    # Потери увеличиваются с расстоянием (больше препятствий на пути)
    urban_loss_factor = 1.0 + (distance / station_radius) * 0.8  # Дополнительные 0-80% потерь
    urban_loss = 10 * math.log10(urban_loss_factor)  # Преобразуем в дБ
    
    # Общие потери = свободное пространство + городские потери
    total_path_loss = free_space_loss + urban_loss
    
    # Нормализация к шкале 0-100%
    # At close range (50m): ~95-100%
    # At medium range (500m): ~60-70% (с учетом городских потерь)
    # At far range (1500m): ~20-30% (с учетом городских потерь)
    baseline_path_loss = 20 * math.log10(50) + 20 * math.log10(LTE_FREQUENCY_MHZ) - 27.55  # Reference at 50m
    path_loss_diff = total_path_loss - baseline_path_loss
    
    # Масштабирование с учетом городских потерь
    # Более агрессивное затухание для городской среды
    signal_strength = max(0, min(100, 100 - path_loss_diff / 0.9))  # Более агрессивное затухание
    
    # Гарантируем, что на границе (1500м) сигнал все еще ненулевой (минимум 15-20%)
    # Это важно для навигации - станция должна быть доступна до границы радиуса
    if distance >= station_radius * 0.9:  # Последние 10% радиуса
        signal_strength = max(signal_strength, 15)  # Минимум 15% на границе
    
    return signal_strength

def measure_ranges_from_stations(true_lat, true_lon, stations):
    """
    Simulate range measurements from cellular base stations with improved LTE accuracy.
    Returns measurements with signal strength for station selection.
    """
    measurements = []
    towers_with_signals = []  # For station selection: (id, lon, lat, radius, generation, operator, signal)
    
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
                
                # Store for station selection (with signal strength)
                towers_with_signals.append((
                    station['id'],
                    station['lon'],
                    station['lat'],
                    station['radius'],
                    station.get('generation', '4g'),
                    station.get('operator', 'Unknown'),
                    signal_strength
                ))
                
                measurements.append({
                    'station_id': station['id'],
                    'lat': station['lat'],
                    'lon': station['lon'],
                    'distance': measured_distance,
                    'signal_strength': signal_strength,
                    'uncertainty': measurement_uncertainty  # For better weighting
                })
    
    # Select diverse towers for better trilateration accuracy
    selected_towers = select_diverse_towers(towers_with_signals, MAX_TOWERS_TO_SELECT)
    selected_station_ids = {t[0] for t in selected_towers}  # Set of selected station IDs
    
    # Filter measurements to only include selected stations
    filtered_measurements = [m for m in measurements if m['station_id'] in selected_station_ids]
    
    # Store selected towers info for map display
    return {
        'measurements': filtered_measurements,
        'selected_stations': selected_towers,  # (id, lon, lat, radius, generation, operator, signal)
        'all_measurements': measurements  # Keep for reference
    }

def trilaterate_position(measurement_data):
    """
    Calculate UAV position using weighted intersection center optimization.
    
    Uses the selected diverse stations to find the optimal position within
    the intersection of all tower coverage areas, weighted by signal strength.
    
    Args:
        measurement_data: Dictionary with 'measurements' and 'selected_stations'
        
    Returns:
        Dictionary with 'lat' and 'lon', or None if insufficient data
    """
    if isinstance(measurement_data, dict):
        measurements = measurement_data.get('measurements', [])
        selected_stations = measurement_data.get('selected_stations', [])
    else:
        # Backward compatibility: if old format (list), convert
        measurements = measurement_data
        selected_stations = []
    
    if len(measurements) < 3:
        return None
    
    # If we have selected stations with signals, use weighted intersection method
    if selected_stations and len(selected_stations) >= 3:
        result = find_weighted_intersection_center(selected_stations)
        if result['success'] and result['point']:
            lon, lat = result['point']
            return {'lat': lat, 'lon': lon}
    
    # Fallback to original method if optimization fails or no selected stations
    # This ensures backward compatibility
    if len(measurements) < 3:
        return None
    
    # Original weighted least squares method as fallback
    weights = []
    for m in measurements:
        signal_weight = m['signal_strength'] / 100.0
        if 'uncertainty' in m and m['uncertainty'] > 0:
            uncertainty_weight = 1.0 / (1.0 + (m['uncertainty'] / 10.0) ** 2)
        else:
            uncertainty_weight = 1.0 / (1.0 + m['distance'] / 1000.0)
        weight = signal_weight * uncertainty_weight
        weights.append(weight)
    
    total_weight = sum(weights)
    if total_weight < 0.01:
        return None
    
    weights = [w / total_weight for w in weights]
    est_lat = sum(m['lat'] * w for m, w in zip(measurements, weights))
    est_lon = sum(m['lon'] * w for m, w in zip(measurements, weights))
    
    return {'lat': est_lat, 'lon': est_lon}

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
        # Enhanced gains for improved trajectory accuracy: higher kp for faster response, tuned ki/kd
        self.lateral_pid = PIDController(kp=1.0, ki=0.03, kd=0.25, max_output=MAX_BANK_ANGLE_DEG)  # Increased kp from 0.8 to 1.0
        
        # PID controller for longitudinal path following (along-track error)
        # Enhanced gains for better speed control and position accuracy
        self.longitudinal_pid = PIDController(kp=0.5, ki=0.015, kd=0.15, max_output=MAX_ACCELERATION_MPS2)  # Increased kp from 0.4 to 0.5
        
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
    
    def update_adaptive_parameters(self, signal_quality, wind_conditions=None, stations_available=None):
        """Adapt control parameters based on environmental conditions"""
        # CRITICAL: Reduce control sensitivity in poor signal conditions or low station count
        # This prevents erratic corrections when position is inaccurate
        if stations_available is not None and stations_available < 3:
            # Low station count: significantly reduce sensitivity to prevent erratic movement
            self.adaptive_gain_factor = 0.4  # Very conservative - prevents erratic corrections
        elif signal_quality < 30:
            self.adaptive_gain_factor = 0.5  # Very poor signal
        elif signal_quality < 50:
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
                       signal_quality, dt=None, stations_available=None):
        """Compute control commands for path following
        
        Args:
            dt: Time step in seconds. If None, uses SIMULATION_UPDATE_INTERVAL_S
            stations_available: Number of base stations available for trilateration
        """
        if dt is None:
            dt = SIMULATION_UPDATE_INTERVAL_S
        
        # Update adaptive parameters (pass station count for degraded mode detection)
        self.update_adaptive_parameters(signal_quality, stations_available=stations_available)
        
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
    """Hybrid navigation: combine route following with trilateration correction
    
    CRITICAL FIX: Trust route more when trilateration shows large deviations.
    Large deviations are likely measurement noise, not route errors.
    """
    # Calculate correction
    correction = calculate_correction_vector(measured_pos, route_start, route_end, route_progress)
    deviation = correction['deviation']
    
    # CRITICAL FIX: Trust route when trilateration deviates significantly
    # Large deviations (>15m) are likely measurement noise, not route errors
    # Small deviations (<5m) might be real, apply small correction
    
    # Base correction strength from signal quality (0.0 to 1.0)
    signal_factor = signal_quality / 100.0
    
    # CRITICAL: Deviation-based trust factor
    # Small deviation (<5m): trust trilateration (might be accurate)
    # Medium deviation (5-15m): reduce trust (likely noise)
    # Large deviation (>15m): minimal trust (definitely noise)
    if deviation < 5.0:
        # Small deviation: might be accurate, apply moderate correction
        deviation_trust = 0.6  # Trust trilateration 60%
    elif deviation < 15.0:
        # Medium deviation: likely noise, minimal correction
        deviation_trust = 0.2  # Trust trilateration 20%
    else:
        # Large deviation: definitely noise, trust route
        deviation_trust = 0.05  # Trust trilateration 5% (minimal)
    
    # Correction strength: blend of signal quality and deviation trust
    # High signal + small deviation = trust trilateration
    # Low signal OR large deviation = trust route
    correction_strength = signal_factor * deviation_trust
    
    # CRITICAL FIX: Reduced maximum correction from 30% to 12%
    # This prevents noisy trilateration from pulling drone too far from route
    max_correction_factor = 0.12  # Reduced from 0.3
    
    # Apply correction with adaptive strength
    corrected_lat = route_pos['lat'] + correction['lat'] * correction_strength * max_correction_factor
    corrected_lon = route_pos['lon'] + correction['lon'] * correction_strength * max_correction_factor
    
    # CRITICAL FIX: Route constraint - if result deviates too much, snap to route
    # This prevents trilateration noise from pulling drone away from accurate route
    result_deviation = calculate_distance_meters(
        corrected_lat, corrected_lon,
        route_pos['lat'], route_pos['lon']
    )
    
    if result_deviation > 15.0:  # If hybrid result is >15m from route
        # Snap back to route - trilateration is too noisy
        corrected_lat = route_pos['lat']
        corrected_lon = route_pos['lon']
        correction_strength = 0.0  # No correction applied
    
    return {
        'lat': corrected_lat,
        'lon': corrected_lon,
        'deviation': deviation,
        'correction_applied': correction_strength > 0.05
    }

# CSV статистика полета
FLIGHT_STATS_CSV = 'flight_statistics.csv'
_flight_stats_file = None
_flight_stats_writer = None
_flight_stats_initialized = False

def init_flight_statistics():
    """Инициализация CSV файла для статистики полета"""
    global _flight_stats_file, _flight_stats_writer, _flight_stats_initialized
    
    if _flight_stats_initialized:
        return
    
    # Создаем CSV файл с заголовками
    file_exists = os.path.exists(FLIGHT_STATS_CSV)
    
    try:
        _flight_stats_file = open(FLIGHT_STATS_CSV, 'a', newline='', encoding='utf-8')
        _flight_stats_writer = csv.writer(_flight_stats_file)
        
        # Записываем заголовки только если файл новый
        if not file_exists:
            headers = [
                'timestamp',
                'true_lat', 'true_lon',
                'measured_lat', 'measured_lon',
                'position_error_m',
                'num_stations',
                'avg_signal_strength',
                'min_distance_to_station',
                'max_distance_to_station',
                'route_deviation_m',
                'trilateration_success'
            ]
            _flight_stats_writer.writerow(headers)
            _flight_stats_file.flush()
        
        _flight_stats_initialized = True
        print(f"Flight statistics initialized: {FLIGHT_STATS_CSV}")
    except Exception as e:
        print(f"Error initializing flight statistics: {e}")

def write_flight_statistics(
    true_pos, measured_pos, position_error,
    num_stations, avg_signal, distances_to_stations,
    route_deviation, trilateration_success
):
    """Запись статистики полета в CSV файл"""
    global _flight_stats_file, _flight_stats_writer
    
    if not _flight_stats_initialized:
        init_flight_statistics()
    
    if not _flight_stats_writer:
        return
    
    try:
        min_distance = min(distances_to_stations) if distances_to_stations else 0
        max_distance = max(distances_to_stations) if distances_to_stations else 0
        
        row = [
            datetime.now().isoformat(),
            true_pos['lat'] if true_pos else '',
            true_pos['lon'] if true_pos else '',
            measured_pos['lat'] if measured_pos else '',
            measured_pos['lon'] if measured_pos else '',
            position_error,
            num_stations,
            avg_signal,
            min_distance,
            max_distance,
            route_deviation,
            1 if trilateration_success else 0
        ]
        
        _flight_stats_writer.writerow(row)
        _flight_stats_file.flush()
    except Exception as e:
        print(f"Error writing flight statistics: {e}")

def close_flight_statistics():
    """Закрытие CSV файла статистики"""
    global _flight_stats_file, _flight_stats_initialized
    
    if _flight_stats_file:
        try:
            _flight_stats_file.close()
        except:
            pass
        _flight_stats_file = None
        _flight_stats_initialized = False

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
                        # CRITICAL: Pass station count to control system for adaptive behavior
                        # Get station count from drone state (will be updated after measurements)
                        stations_available = drone_state.get('last_station_count', 3)
                        control_output = fcs.compute_control(
                            control_position,
                            start,
                            end,
                            segment_progress,
                            drone_state.get('signal_quality', 100),
                            dt=SIMULATION_UPDATE_INTERVAL_S,
                            stations_available=stations_available
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
                        
                        # ENHANCED: Calculate progress increment with improved precision
                        # Cap increment to prevent overshooting waypoints while maintaining accuracy
                        progress_increment = move_distance / segment_distance
                        max_increment = 0.15  # Increased from 0.1 to 15% for smoother progress (still safe)
                        progress_increment = min(progress_increment, max_increment)
                        
                        # Update segment progress with precision
                        segment_progress = min(1.0, segment_progress + progress_increment)
                        drone_state['segment_progress'][idx] = segment_progress
                        
                        # Calculate new position along route segment with smooth interpolation
                        # The interpolate_position function uses easing for natural motion
                        new_position = interpolate_position(start, end, segment_progress)
                        
                        # ENHANCED: Verify position accuracy - ensure we're exactly on the route
                        # FIXED: Only project if deviation is extreme, and preserve forward progress
                        # Calculate distance from calculated position to route line
                        route_deviation = calculate_distance_to_route_segment(new_position, start, end)
                        if route_deviation > 10.0:  # Increased threshold from 0.5m to 10.0m to prevent constant projection
                            # Project position back onto route
                            route_dx = end['lon'] - start['lon']
                            route_dy = end['lat'] - start['lat']
                            route_length = math.sqrt(route_dx**2 + route_dy**2)
                            if route_length > 1e-10:
                                pos_dx = new_position['lon'] - start['lon']
                                pos_dy = new_position['lat'] - start['lat']
                                # Project onto route
                                t = (pos_dx * route_dx + pos_dy * route_dy) / (route_length * route_length)
                                t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
                                
                                # CRITICAL FIX: Only update progress if projected position is ahead
                                # This prevents regression that causes deadlock
                                if t > segment_progress - 0.01:  # Allow small backward correction but prevent major regression
                                    # Recalculate position exactly on route
                                    new_position = {
                                        'lat': start['lat'] + t * route_dy,
                                        'lon': start['lon'] + t * route_dx
                                    }
                                    # Update progress to match corrected position (only if ahead or slightly behind)
                                    segment_progress = max(segment_progress - 0.01, t)  # Prevent major regression
                                    drone_state['segment_progress'][idx] = segment_progress
                                # If projection would cause major regression, keep current progress and just correct position
                                else:
                                    # Only correct position, don't change progress
                                    # Project current progress position onto route
                                    current_progress_pos = interpolate_position(start, end, segment_progress)
                                    route_dev = calculate_distance_to_route_segment(current_progress_pos, start, end)
                                    if route_dev > 10.0:
                                        # Minimal correction: move toward route but preserve progress
                                        route_dir_x = route_dx / route_length
                                        route_dir_y = route_dy / route_length
                                        # Project current position onto route at current progress
                                        proj_lat = start['lat'] + segment_progress * route_dy
                                        proj_lon = start['lon'] + segment_progress * route_dx
                                        # Blend: 90% route position, 10% current position (minimal correction)
                                        new_position = {
                                            'lat': proj_lat * 0.9 + new_position['lat'] * 0.1,
                                            'lon': proj_lon * 0.9 + new_position['lon'] * 0.1
                                        }
                        
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
                        
                        # ENHANCED: Improved trajectory following with precise waypoint tracking
                        # Apply correction based on error magnitude and route geometry
                        # During turns, apply smaller corrections to avoid fighting control system
                        is_in_turn = abs(control_output['bank_angle']) > 5.0  # Significant bank angle indicates turn
                        cross_track_error = abs(control_output['cross_track_error'])
                        along_track_error = abs(control_output['along_track_error'])
                        
                        # Enhanced correction logic for better trajectory accuracy
                        if cross_track_error > 0.5:  # Apply correction for any significant error (reduced threshold from 1.0m)
                            # Adaptive correction factor based on error magnitude and turn state
                            if is_in_turn:
                                # During turns: smaller correction to avoid overcorrection
                                correction_factor = min(0.12, cross_track_error / 60.0)  # Increased from 0.08 for better response
                            else:
                                # On straight segments: larger correction for precise following
                                correction_factor = min(0.25, cross_track_error / 30.0)  # Increased from 0.2 for better accuracy
                            
                            # Calculate correction direction (perpendicular to route)
                            route_dx = end['lon'] - start['lon']
                            route_dy = end['lat'] - start['lat']
                            route_length = math.sqrt(route_dx**2 + route_dy**2)
                            if route_length > 1e-10:
                                # Perpendicular vector (rotate 90 degrees)
                                perp_dx = -route_dy / route_length
                                perp_dy = route_dx / route_length
                                # Apply correction (pull toward route) with improved precision
                                correction_sign = -1.0 if control_output['cross_track_error'] > 0 else 1.0
                                # Convert meters to degrees more accurately
                                lat_correction = perp_dy * correction_factor * correction_sign * (control_output['cross_track_error'] / 111000.0)
                                lon_correction = perp_dx * correction_factor * correction_sign * (control_output['cross_track_error'] / (111000.0 * math.cos(math.radians(new_position['lat']))))
                                new_position['lat'] += lat_correction
                                new_position['lon'] += lon_correction
                        
                        # Along-track correction: ensure we're at the right progress point
                        # FIXED: Increased threshold and prevent progress regression to avoid deadlock
                        if along_track_error > 20.0:  # Increased from 2.0m to 20.0m to prevent constant triggering
                            # Adjust progress to match actual position, but never reduce progress
                            route_dx = end['lon'] - start['lon']
                            route_dy = end['lat'] - start['lat']
                            route_length = math.sqrt(route_dx**2 + route_dy**2)
                            if route_length > 1e-10:
                                # Calculate actual progress based on position
                                pos_dx = new_position['lon'] - start['lon']
                                pos_dy = new_position['lat'] - start['lat']
                                # Project position onto route vector
                                actual_progress = (pos_dx * route_dx + pos_dy * route_dy) / (route_length * route_length)
                                actual_progress = max(0.0, min(1.0, actual_progress))
                                
                                # CRITICAL FIX: Only adjust if actual_progress is ahead of current progress
                                # This prevents the feedback loop that causes deadlock
                                if actual_progress > segment_progress + 0.01:  # Only if significantly ahead
                                    # Smoothly adjust segment progress toward actual progress
                                    segment_progress = segment_progress * 0.8 + actual_progress * 0.2
                                    segment_progress = max(0.0, min(1.0, segment_progress))
                                    drone_state['segment_progress'][idx] = segment_progress
                                    # Recalculate position with corrected progress
                                    new_position = interpolate_position(start, end, segment_progress)
                                # If behind, don't reduce progress - allow forward movement to continue
                        
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
                    
                    # Measure ranges from base stations and select best diverse stations
                    measurement_result = measure_ranges_from_stations(
                        drone_state['true_position']['lat'],
                        drone_state['true_position']['lon'],
                        stations
                    )
                    
                    # Extract measurements and selected stations
                    measurements = measurement_result.get('measurements', [])
                    selected_stations = measurement_result.get('selected_stations', [])
                    
                    # Store selected stations for map display
                    drone_state['selected_stations'] = selected_stations
                    
                    # Calculate position using trilateration with selected stations
                    position_updated = False
                    
                    # CRITICAL: Handle degraded mode when stations < 3
                    # With 2 stations, trilateration is ambiguous but possible
                    # With < 2 stations, must use route-constrained dead reckoning
                    if stations_available >= 2:
                        calculated_pos = trilaterate_position(measurement_result)
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
                            drone_state['signal_quality'] = sum(m['signal_strength'] for m in measurements) / len(measurements) if measurements else 0
                            
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
                                
                                # CRITICAL FIX: Removed route correction feedback loop
                                # This was causing the drone to drift away from accurate route
                                # Route is ground truth - don't modify it based on noisy trilateration measurements
                                # 
                                # Previous code pulled route toward noisy measurements, creating feedback loop:
                                # 1. Trilateration has noise → shows 20m deviation
                                # 2. Route correction pulls route toward noisy measurement
                                # 3. Next cycle: route is wrong, trilateration shows different error
                                # 4. Process repeats → drone oscillates
                                #
                                # Solution: Trust route as ground truth, only use trilateration for small corrections
                                # Route position remains accurate, trilateration provides minor adjustments only
                            else:
                                # Fallback: use measured position directly
                                drone_state['position'] = measured_position
                            
                            # Broadcast position to all connected clients with selected station IDs
                            selected_station_ids = [m['station_id'] for m in measurements]
                            await broadcast_position(measurements, selected_station_ids)
                            position_updated = True
                            drone_state['consecutive_failures'] = 0
                            
                            # Записываем статистику полета в CSV
                            try:
                                # Вычисляем ошибку позиционирования
                                position_error = calculate_distance_meters(
                                    drone_state['true_position']['lat'],
                                    drone_state['true_position']['lon'],
                                    drone_state['position']['lat'],
                                    drone_state['position']['lon']
                                )
                                
                                # Вычисляем реальные расстояния до станций (без шума измерения)
                                distances_to_stations = []
                                for m in measurements:
                                    # Вычисляем реальное расстояние от истинной позиции до станции
                                    station_lat = m.get('lat')
                                    station_lon = m.get('lon')
                                    if station_lat and station_lon:
                                        true_dist = calculate_distance_meters(
                                            drone_state['true_position']['lat'],
                                            drone_state['true_position']['lon'],
                                            station_lat,
                                            station_lon
                                        )
                                        distances_to_stations.append(true_dist)
                                
                                # Средняя сила сигнала
                                avg_signal = drone_state.get('signal_quality', 0)
                                
                                # Отклонение истинной позиции от маршрута (должно быть ~0, так как true_position всегда на маршруте)
                                # Вычисляем для статистики, но обычно это будет близко к нулю
                                route_deviation = 0.0
                                if drone_state.get('route') and idx < len(route) - 1:
                                    route_start = copy_waypoint(route[idx])
                                    route_end = copy_waypoint(route[idx + 1])
                                    route_deviation = calculate_distance_to_route_segment(
                                        drone_state['true_position'],
                                        route_start,
                                        route_end
                                    )
                                
                                # Записываем статистику
                                write_flight_statistics(
                                    true_pos=drone_state['true_position'],
                                    measured_pos=drone_state['position'],
                                    position_error=position_error,
                                    num_stations=len(measurements),
                                    avg_signal=avg_signal,
                                    distances_to_stations=distances_to_stations,
                                    route_deviation=route_deviation,
                                    trilateration_success=True
                                )
                            except Exception as e:
                                # Не прерываем полет из-за ошибки записи статистики
                                if loop_count % 100 == 0:
                                    print(f"Warning: Failed to write flight statistics: {e}")
                        else:
                            drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    else:
                        drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    
                    # Fallback: Enhanced dead reckoning with route constraint when measurements fail
                    if not position_updated:
                        # CRITICAL FIX: Use route-constrained navigation when stations insufficient
                        # This prevents erratic movement by constraining position to route
                        if drone_state.get('route') and drone_state.get('true_position') and idx < len(route) - 1:
                            # Enhanced dead reckoning: blend route position with Kalman prediction
                            route_pos = copy_waypoint(drone_state['true_position'])
                            
                            if drone_state['position'] and drone_state['kalman_filter']:
                                # Use Kalman filter prediction (dead reckoning)
                                kf = drone_state['kalman_filter']
                                kf.predict()
                                kf_pos = kf.get_position()
                                
                                # Blend: 85% route, 15% Kalman (trust route more in low coverage)
                                # This prevents position from drifting away from route
                                blended_pos = {
                                    'lat': route_pos['lat'] * 0.85 + kf_pos['lat'] * 0.15,
                                    'lon': route_pos['lon'] * 0.85 + kf_pos['lon'] * 0.15
                                }
                                
                                # CRITICAL: Project onto route if deviation too large
                                # This prevents erratic movement by keeping position on route
                                route_start = copy_waypoint(route[idx])
                                route_end = copy_waypoint(route[idx + 1])
                                route_deviation = calculate_distance_to_route_segment(
                                    blended_pos, route_start, route_end
                                )
                                
                                if route_deviation > 30.0:  # If >30m off route, snap to route
                                    # Use route position directly to prevent erratic movement
                                    drone_state['position'] = route_pos
                                    if loop_count % 20 == 0:
                                        print(f"Low coverage (stations={stations_available}): Using route position (deviation was {route_deviation:.1f}m)")
                                else:
                                    # Use blended position (slightly off route but smooth)
                                    drone_state['position'] = blended_pos
                            else:
                                # No Kalman filter: use route position directly
                                drone_state['position'] = route_pos
                            
                            # Signal quality degrades slowly in low coverage
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 0.5)  # Slower degradation
                            
                            # Broadcast with reduced signal quality (no selected stations)
                            await broadcast_position([], [])
                        elif drone_state['position'] and drone_state['kalman_filter']:
                            # Fallback: pure Kalman prediction (no route available)
                            kf = drone_state['kalman_filter']
                            kf.predict()
                            predicted_pos = kf.get_position()
                            drone_state['position'] = predicted_pos
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 1)
                            await broadcast_position([], [])
                        elif drone_state['true_position']:
                            # Last resort: use true position (for simulation)
                            drone_state['position'] = copy_waypoint(drone_state['true_position'])
                            await broadcast_position([], [])
                        else:
                            # No position at all - this shouldn't happen, but log it
                            print(f"ERROR: No position data available at loop {loop_count}")
                        
                        # FIXED: Only warn every 10 failures to reduce log spam, and increase threshold
                        consecutive_failures = drone_state.get('consecutive_failures', 0)
                        if consecutive_failures >= max_consecutive_failures:
                            if consecutive_failures % 10 == 0:  # Only warn every 10 failures
                                await broadcast_low_signal_warning(stations, drone_state.get('true_position') or drone_state.get('position'))
                                print(f"Warning: {consecutive_failures} consecutive measurement failures")
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
                    
                    # Measure ranges from base stations and select best diverse stations
                    measurement_result = measure_ranges_from_stations(
                        drone_state['true_position']['lat'],
                        drone_state['true_position']['lon'],
                        stations
                    )
                    
                    # Extract measurements and selected stations
                    measurements = measurement_result.get('measurements', [])
                    selected_stations = measurement_result.get('selected_stations', [])
                    
                    # Store selected stations for map display
                    drone_state['selected_stations'] = selected_stations
                    
                    # Calculate position using trilateration with selected stations
                    position_updated = False
                    stations_available = len(measurements) if measurements else 0
                    
                    # FIXED: Reduce measurement requirement from 3 to 2 for better coverage
                    # CRITICAL: Handle degraded mode when stations < 3
                    if stations_available >= 2:
                        calculated_pos = trilaterate_position(measurement_result)
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
                            drone_state['signal_quality'] = sum(m['signal_strength'] for m in measurements) / len(measurements) if measurements else 0
                            
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
                            
                            # Broadcast position with selected station IDs
                            selected_station_ids = [m['station_id'] for m in measurements]
                            await broadcast_position(measurements, selected_station_ids)
                            position_updated = True
                            drone_state['consecutive_failures'] = 0
                        else:
                            drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    else:
                        drone_state['consecutive_failures'] = drone_state.get('consecutive_failures', 0) + 1
                    
                    # Fallback: Enhanced dead reckoning with route constraint for final waypoint
                    if not position_updated:
                        # CRITICAL FIX: Use route-constrained navigation when stations insufficient
                        if drone_state.get('route') and drone_state.get('true_position'):
                            # Use route position as primary source in low coverage
                            route_pos = copy_waypoint(drone_state['true_position'])
                            
                            if drone_state['position'] and drone_state['kalman_filter']:
                                kf = drone_state['kalman_filter']
                                kf.predict()
                                kf_pos = kf.get_position()
                                
                                # Blend: 85% route, 15% Kalman (trust route more)
                                blended_pos = {
                                    'lat': route_pos['lat'] * 0.85 + kf_pos['lat'] * 0.15,
                                    'lon': route_pos['lon'] * 0.85 + kf_pos['lon'] * 0.15
                                }
                                
                                # Check deviation from route to final waypoint
                                final_waypoint = copy_waypoint(route[-1])
                                route_deviation = calculate_distance_meters(
                                    blended_pos['lat'], blended_pos['lon'],
                                    route_pos['lat'], route_pos['lon']
                                )
                                
                                if route_deviation > 30.0:  # If >30m off, use route position
                                    drone_state['position'] = route_pos
                                else:
                                    drone_state['position'] = blended_pos
                            else:
                                drone_state['position'] = route_pos
                            
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 0.5)
                            await broadcast_position([], [])
                        elif drone_state['position'] and drone_state['kalman_filter']:
                            kf = drone_state['kalman_filter']
                            kf.predict()
                            predicted_pos = kf.get_position()
                            drone_state['position'] = predicted_pos
                            drone_state['signal_quality'] = max(0, drone_state['signal_quality'] - 1)
                            await broadcast_position([], [])
                        elif drone_state['true_position']:
                            drone_state['position'] = copy_waypoint(drone_state['true_position'])
                            await broadcast_position([], [])
                        
                        # FIXED: Only warn every 10 failures to reduce log spam
                        consecutive_failures = drone_state.get('consecutive_failures', 0)
                        if consecutive_failures >= max_consecutive_failures:
                            if consecutive_failures % 10 == 0:  # Only warn every 10 failures
                                await broadcast_low_signal_warning(stations, drone_state.get('true_position') or drone_state.get('position'))
                                print(f"Warning: {consecutive_failures} consecutive measurement failures")
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

async def broadcast_position(measurements, selected_station_ids=None):
    """
    Send calculated position to all connected clients
    
    Args:
        measurements: List of measurement dictionaries (can be empty for fallback)
        selected_station_ids: List of selected station IDs for display (optional)
    """
    if drone_state['position'] and connected_clients:
        # Extract station IDs from measurements if not provided
        if selected_station_ids is None:
            selected_station_ids = [m['station_id'] for m in measurements] if measurements else []
        
        message = json.dumps({
            'type': 'location_update',
            'payload': {
                'lat': drone_state['position']['lat'],
                'lon': drone_state['position']['lon'],
                'signal_quality': drone_state['signal_quality'],
                'stations_used': selected_station_ids,  # Only selected stations
                'selected_stations': selected_station_ids  # For map display
            }
        })
        
        # FIXED: Handle disconnected clients gracefully to prevent crashes
        disconnected_clients = set()
        for client in connected_clients:
            try:
                await client.send(message)
            except (websockets.exceptions.ConnectionClosed, websockets.exceptions.ConnectionClosedOK, 
                    websockets.exceptions.ConnectionClosedError, Exception) as e:
                # Mark for removal but don't remove during iteration
                disconnected_clients.add(client)
                # Don't print every disconnection to reduce log spam
                if len(disconnected_clients) == 1:  # Only log first disconnection
                    pass  # Silent removal
        
        # Remove disconnected clients
        connected_clients.difference_update(disconnected_clients)

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
            
            if data['type'] == 'set_start_location':
                # Set starting location from predefined list
                location_id = data['payload'].get('location_id')
                if location_id:
                    location = next((loc for loc in MOSCOW_DRONE_LOCATIONS if loc['id'] == location_id), None)
                    if location:
                        drone_state['selected_start_location'] = location
                        drone_state['true_position'] = {'lat': location['lat'], 'lon': location['lon']}
                        drone_state['position'] = {'lat': location['lat'], 'lon': location['lon']}
                        # Reset Kalman filter for new location
                        drone_state['kalman_filter'] = None
                        # Reset flight state
                        drone_state['is_flying'] = False
                        drone_state['route'] = []
                        drone_state['route_index'] = 0
                        drone_state['signal_quality'] = 100  # Reset signal quality
                        drone_state['consecutive_failures'] = 0
                        # Initialize flight control system
                        drone_state['flight_control'] = FlightControlSystem()
                        # Broadcast position immediately so it appears on map
                        await broadcast_position([], [])
                        print(f"Starting location set to: {location['name']} ({location['lat']}, {location['lon']})")
                    else:
                        print(f"Error: Unknown location ID: {location_id}")
                else:
                    print("Error: No location_id provided in set_start_location")
            
            elif data['type'] == 'set_route':
                # Receive route from client (generated by 2GIS API)
                try:
                    waypoints = data['payload'].get('waypoints', [])
                    
                    # FIXED: Validate waypoints format
                    if not waypoints or not isinstance(waypoints, list):
                        print(f"Error: Invalid waypoints format: {type(waypoints)}")
                        return
                    
                    # Validate each waypoint has required fields
                    valid_waypoints = []
                    for wp in waypoints:
                        if isinstance(wp, dict) and 'lat' in wp and 'lon' in wp:
                            try:
                                lat = float(wp['lat'])
                                lon = float(wp['lon'])
                                if not (math.isnan(lat) or math.isnan(lon)):
                                    valid_waypoints.append({'lat': lat, 'lon': lon})
                            except (ValueError, TypeError):
                                continue
                    
                    if len(valid_waypoints) < 2:
                        print(f"Error: Not enough valid waypoints ({len(valid_waypoints)} < 2)")
                        return
                    
                    waypoints = valid_waypoints
                    
                    # FIXED: Limit route waypoints to prevent crashes on large routes
                    MAX_WAYPOINTS = 500  # Maximum waypoints to prevent memory/performance issues
                    if len(waypoints) > MAX_WAYPOINTS:
                        print(f"Warning: Route has {len(waypoints)} waypoints, limiting to {MAX_WAYPOINTS} for performance")
                        # Sample waypoints evenly across the route (FIXED: prevent index out of bounds)
                        original_waypoints = waypoints.copy()
                        step = (len(waypoints) - 1) / (MAX_WAYPOINTS - 1)  # Ensure we can reach last index
                        sampled_waypoints = []
                        for i in range(MAX_WAYPOINTS - 1):
                            idx = int(i * step)
                            if idx < len(original_waypoints):
                                sampled_waypoints.append(original_waypoints[idx])
                        # Always keep last waypoint
                        if original_waypoints:
                            sampled_waypoints.append(original_waypoints[-1])
                        waypoints = sampled_waypoints
                        print(f"Route simplified from {len(original_waypoints)} to {len(waypoints)} waypoints")
                    
                    if waypoints and len(waypoints) >= 2:
                        # Use selected start location if available, otherwise use first waypoint
                        start_pos = None
                        if drone_state.get('selected_start_location'):
                            start_pos = {
                                'lat': drone_state['selected_start_location']['lat'],
                                'lon': drone_state['selected_start_location']['lon']
                            }
                            # Replace first waypoint with selected start location for accurate trajectory
                            waypoints[0] = start_pos
                            print(f"Using selected start location: {drone_state['selected_start_location']['name']}")
                        else:
                            start_pos = copy_waypoint(waypoints[0])
                            print("No start location selected, using first waypoint from route")
                        
                        # Apply trajectory smoothing for fluid motion
                        smoothed_waypoints = smooth_trajectory(waypoints, CURVATURE_SMOOTHING)
                        
                        # Ensure first waypoint is the selected start location
                        if start_pos:
                            smoothed_waypoints[0] = copy_waypoint(start_pos)
                        
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
                except Exception as e:
                    print(f"Error processing route: {e}")
                    import traceback
                    traceback.print_exc()
            
            elif data['type'] == 'start_demo':
                # Start demo flight from first station
                if stations and len(stations) > 0:
                    # Set position to first station
                    start_pos = {'lat': stations[0]['lat'], 'lon': stations[0]['lon']}
                    drone_state['true_position'] = copy_waypoint(start_pos)
                    drone_state['position'] = copy_waypoint(start_pos)
                    
                    # Create a simple demo route: fly to second station if available, otherwise fly 1km north
                    if len(stations) > 1:
                        # Route to second station
                        demo_route = [copy_waypoint(start_pos), copy_waypoint({'lat': stations[1]['lat'], 'lon': stations[1]['lon']})]
                    else:
                        # Route 1km north from first station
                        demo_route = [
                            copy_waypoint(start_pos),
                            {'lat': start_pos['lat'] + 0.009, 'lon': start_pos['lon']}  # ~1km north
                        ]
                    
                    # Apply trajectory smoothing
                    smoothed_route = smooth_trajectory(demo_route, CURVATURE_SMOOTHING)
                    smoothed_route[0] = copy_waypoint(start_pos)  # Ensure start is exact
                    
                    # Initialize flight state
                    drone_state['route'] = smoothed_route
                    drone_state['smoothed_route'] = smoothed_route
                    drone_state['route_index'] = 0
                    drone_state['is_flying'] = True
                    drone_state['consecutive_failures'] = 0
                    drone_state['segment_progress'] = {}
                    drone_state['signal_quality'] = 100
                    drone_state['current_velocity'] = DRONE_SPEED_MPS
                    drone_state['target_velocity'] = DRONE_SPEED_MPS
                    drone_state['velocity_vector'] = {'lat': 0.0, 'lon': 0.0}
                    
                    # Reset Kalman filter for new demo
                    drone_state['kalman_filter'] = None
                    # Initialize flight control system
                    drone_state['flight_control'] = FlightControlSystem()
                    
                    # Broadcast position immediately
                    await broadcast_position([], [])
                    print(f"Demo started: Flying from station {stations[0]['id']} to {'station ' + str(stations[1]['id']) if len(stations) > 1 else '1km north'}")
                else:
                    print("Error: No stations available for demo")
            
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
