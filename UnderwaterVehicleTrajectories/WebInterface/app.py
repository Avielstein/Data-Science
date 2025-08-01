"""
Underwater Vehicle Trajectory Planning Web Interface

A clean, professional web interface for visualizing and comparing
trajectory planning algorithms for underwater vehicles.
"""

from flask import Flask, render_template, request, jsonify, send_from_directory
import os
import json
import numpy as np
from datetime import datetime
import threading
import time

from trajectory_visualizer import TrajectoryVisualizer, generate_sample_results
import sys
import os

# Add the Dubins algorithms to the path
dubins_path = os.path.join(os.path.dirname(__file__), '..', 'Algorithms', 'DubinsCurves')
sys.path.insert(0, dubins_path)

try:
    from dubins_3d import Dubins3DPlanner, BioInspiredDubinsPlanner, Configuration3D
    print("✅ Successfully imported Dubins planning modules")
    DUBINS_AVAILABLE = True
except ImportError as e:
    print(f"❌ Failed to import Dubins modules: {e}")
    print("🔧 Using fallback path planning")
    DUBINS_AVAILABLE = False

app = Flask(__name__)
app.config['SECRET_KEY'] = 'underwater_trajectory_planning_2025'

# Initialize visualizer
visualizer = TrajectoryVisualizer()

# Global state for real-time updates
current_results = {}
current_scenarios = []
custom_scenarios = []
processing_status = {"status": "idle", "progress": 0, "message": "Ready"}

@app.route('/')
def index():
    """Simple drag-and-drop trajectory planner."""
    return render_template('simple_planner.html')

@app.route('/api/scenarios')
def get_scenarios():
    """Get available test scenarios."""
    scenarios = [
        {
            'id': 'simple',
            'name': 'Simple Navigation',
            'description': 'Open water navigation with minimal obstacles',
            'start': [0, 0, -2],
            'goal': [10, 8, -6],
            'obstacles': [],
            'difficulty': 'Easy',
            'environment_type': 'Open Water'
        },
        {
            'id': 'moderate',
            'name': 'Moderate Complexity',
            'description': 'Navigation with moderate obstacle density',
            'start': [0, 0, -2],
            'goal': [15, 12, -8],
            'obstacles': [
                {'position': [5, 4, -4], 'radius': 1.5, 'type': 'coral'},
                {'position': [10, 8, -6], 'radius': 1.0, 'type': 'rock'}
            ],
            'difficulty': 'Medium',
            'environment_type': 'Coral Reef'
        },
        {
            'id': 'complex',
            'name': 'Complex Environment',
            'description': 'Dense obstacle field requiring advanced planning',
            'start': [0, 0, -2],
            'goal': [20, 15, -10],
            'obstacles': [
                {'position': [5, 3, -3], 'radius': 1.2, 'type': 'coral'},
                {'position': [10, 7, -5], 'radius': 1.8, 'type': 'rock'},
                {'position': [15, 12, -8], 'radius': 1.0, 'type': 'coral'},
                {'position': [8, 10, -7], 'radius': 1.5, 'type': 'kelp'}
            ],
            'difficulty': 'Hard',
            'environment_type': 'Kelp Forest'
        },
        {
            'id': 'bio_inspired',
            'name': 'Bio-Inspired Challenge',
            'description': 'Scenario optimized for SALP/jellyfish movement patterns',
            'start': [0, 0, -3],
            'goal': [25, 20, -12],
            'obstacles': [
                {'position': [8, 6, -5], 'radius': 2.0, 'type': 'thermal_vent'},
                {'position': [12, 10, -7], 'radius': 1.5, 'type': 'current'},
                {'position': [18, 15, -9], 'radius': 1.8, 'type': 'predator_zone'}
            ],
            'difficulty': 'Expert',
            'environment_type': 'Deep Ocean',
            'bio_features': {
                'currents': True,
                'thermal_layers': True,
                'predator_zones': True
            }
        }
    ]
    return jsonify(scenarios)

@app.route('/api/create_scenario', methods=['POST'])
def create_custom_scenario():
    """Create a custom scenario."""
    global custom_scenarios
    
    data = request.json
    
    # Validate required fields
    required_fields = ['name', 'start', 'goal']
    for field in required_fields:
        if field not in data:
            return jsonify({'error': f'Missing required field: {field}'}), 400
    
    # Create custom scenario
    custom_scenario = {
        'id': f"custom_{len(custom_scenarios) + 1}",
        'name': data['name'],
        'description': data.get('description', 'Custom scenario'),
        'start': data['start'],
        'goal': data['goal'],
        'obstacles': data.get('obstacles', []),
        'difficulty': data.get('difficulty', 'Custom'),
        'environment_type': data.get('environment_type', 'Custom'),
        'bio_features': data.get('bio_features', {})
    }
    
    custom_scenarios.append(custom_scenario)
    
    return jsonify({
        'message': 'Custom scenario created successfully',
        'scenario': custom_scenario
    })

@app.route('/api/scenarios/custom')
def get_custom_scenarios():
    """Get custom scenarios."""
    return jsonify(custom_scenarios)

@app.route('/api/algorithms')
def get_algorithms():
    """Get available algorithms."""
    algorithms = [
        {
            'id': 'traditional_dubins',
            'name': 'Traditional Dubins',
            'description': 'Classical Dubins curves for smooth path planning',
            'type': 'Classical',
            'strengths': ['Fast planning', 'Smooth paths', 'Predictable'],
            'weaknesses': ['Limited obstacle handling', 'Fixed turn radius']
        },
        {
            'id': 'bio_inspired_dubins',
            'name': 'Bio-Inspired Dubins',
            'description': 'SALP-inspired jet propulsion planning',
            'type': 'Bio-Inspired',
            'strengths': ['Energy-aware', 'Discrete propulsion', 'Biomimetic'],
            'weaknesses': ['Higher energy cost', 'Complex dynamics']
        },
        {
            'id': 'rrt_star',
            'name': 'RRT* Underwater',
            'description': 'Rapidly-exploring Random Tree with underwater adaptations',
            'type': 'Sampling-Based',
            'strengths': ['Obstacle handling', 'Probabilistic completeness', 'Adaptable'],
            'weaknesses': ['Slower planning', 'Non-deterministic']
        }
    ]
    return jsonify(algorithms)

@app.route('/api/run_comparison', methods=['POST'])
def run_comparison():
    """Run trajectory planning comparison."""
    global processing_status, current_results, current_scenarios
    
    data = request.json
    selected_scenarios = data.get('scenarios', [])
    selected_algorithms = data.get('algorithms', [])
    
    if not selected_scenarios or not selected_algorithms:
        return jsonify({'error': 'Please select at least one scenario and algorithm'}), 400
    
    # Start processing in background
    def process_comparison():
        global processing_status, current_results, current_scenarios
        
        try:
            processing_status = {"status": "running", "progress": 10, "message": "Initializing..."}
            
            # Get scenario details
            all_scenarios = [
                {
                    'name': 'Simple Navigation',
                    'start': [0, 0, -2],
                    'goal': [10, 8, -6],
                    'obstacles': []
                },
                {
                    'name': 'Moderate Complexity',
                    'start': [0, 0, -2],
                    'goal': [15, 12, -8],
                    'obstacles': [
                        {'position': [5, 4, -4], 'radius': 1.5},
                        {'position': [10, 8, -6], 'radius': 1.0}
                    ]
                },
                {
                    'name': 'Complex Environment',
                    'start': [0, 0, -2],
                    'goal': [20, 15, -10],
                    'obstacles': [
                        {'position': [5, 3, -3], 'radius': 1.2},
                        {'position': [10, 7, -5], 'radius': 1.8},
                        {'position': [15, 12, -8], 'radius': 1.0},
                        {'position': [8, 10, -7], 'radius': 1.5}
                    ]
                }
            ]
            
            # Filter scenarios
            scenario_map = {'simple': 0, 'moderate': 1, 'complex': 2}
            filtered_scenarios = [all_scenarios[scenario_map[s]] for s in selected_scenarios if s in scenario_map]
            current_scenarios = filtered_scenarios
            
            processing_status = {"status": "running", "progress": 30, "message": "Generating trajectories..."}
            
            # Generate sample results (in real implementation, this would run actual algorithms)
            results = generate_sample_results()
            
            # Filter results by selected algorithms and scenarios
            filtered_results = {}
            for alg in selected_algorithms:
                if alg in results:
                    filtered_results[alg] = {}
                    for scenario in filtered_scenarios:
                        scenario_name = scenario['name']
                        if scenario_name in results[alg]:
                            filtered_results[alg][scenario_name] = results[alg][scenario_name]
            
            current_results = filtered_results
            
            processing_status = {"status": "running", "progress": 60, "message": "Creating visualizations..."}
            
            # Generate visualizations
            trajectory_plot = visualizer.create_3d_trajectory_plot(filtered_scenarios, selected_algorithms)
            dashboard_plot = visualizer.create_performance_dashboard(filtered_results)
            
            processing_status = {"status": "running", "progress": 90, "message": "Finalizing results..."}
            
            # Store plot filenames
            current_results['plots'] = {
                'trajectory': trajectory_plot,
                'dashboard': dashboard_plot
            }
            
            processing_status = {"status": "complete", "progress": 100, "message": "Analysis complete!"}
            
        except Exception as e:
            processing_status = {"status": "error", "progress": 0, "message": f"Error: {str(e)}"}
    
    # Start background processing
    thread = threading.Thread(target=process_comparison)
    thread.daemon = True
    thread.start()
    
    return jsonify({'message': 'Comparison started', 'status': 'running'})

@app.route('/api/status')
def get_status():
    """Get current processing status."""
    return jsonify(processing_status)

@app.route('/api/results')
def get_results():
    """Get comparison results."""
    global current_results
    
    if not current_results:
        return jsonify({'error': 'No results available'}), 404
    
    # Format results for frontend
    formatted_results = {
        'algorithms': list(current_results.keys()),
        'scenarios': list(current_results[list(current_results.keys())[0]].keys()) if current_results else [],
        'data': current_results,
        'plots': current_results.get('plots', {}),
        'timestamp': datetime.now().isoformat()
    }
    
    return jsonify(formatted_results)

@app.route('/api/algorithm_analysis/<algorithm>')
def get_algorithm_analysis(algorithm):
    """Get detailed analysis for specific algorithm."""
    global current_scenarios
    
    if not current_scenarios:
        return jsonify({'error': 'No scenarios available'}), 404
    
    try:
        # Generate deep dive analysis
        plot_filename = visualizer.create_algorithm_deep_dive(algorithm, current_scenarios)
        
        return jsonify({
            'algorithm': algorithm,
            'plot': plot_filename,
            'timestamp': datetime.now().isoformat()
        })
    
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/plots/<filename>')
def serve_plot(filename):
    """Serve generated plot images."""
    return send_from_directory(visualizer.save_dir, filename)

def generate_proper_dubins_path(start_pos, end_pos, start_dir, end_dir, turning_radius):
    """Generate a proper Dubins path using LSL, RSR, LSR, or RSL patterns."""
    
    # Convert to numpy arrays and radians
    start_pos = np.array(start_pos[:2])  # Use only x, y for 2D Dubins
    end_pos = np.array(end_pos[:2])
    start_angle = np.radians(start_dir)
    end_angle = np.radians(end_dir)
    
    # Calculate all possible Dubins paths and choose the shortest
    paths = []
    
    # LSL (Left-Straight-Left)
    lsl_path = calculate_lsl_path(start_pos, end_pos, start_angle, end_angle, turning_radius)
    if lsl_path:
        paths.append(('LSL', lsl_path))
    
    # RSR (Right-Straight-Right)  
    rsr_path = calculate_rsr_path(start_pos, end_pos, start_angle, end_angle, turning_radius)
    if rsr_path:
        paths.append(('RSR', rsr_path))
    
    # LSR (Left-Straight-Right)
    lsr_path = calculate_lsr_path(start_pos, end_pos, start_angle, end_angle, turning_radius)
    if lsr_path:
        paths.append(('LSR', lsr_path))
    
    # RSL (Right-Straight-Left)
    rsl_path = calculate_rsl_path(start_pos, end_pos, start_angle, end_angle, turning_radius)
    if rsl_path:
        paths.append(('RSL', rsl_path))
    
    if not paths:
        # Fallback to simple path if no Dubins path found
        return generate_fallback_path(start_pos, end_pos, start_angle, end_angle)
    
    # Choose shortest path
    best_path = min(paths, key=lambda x: x[1][1])  # x[1][1] is path_length
    path_type, (path_points, path_length) = best_path
    
    # Convert back to 3D by adding z coordinate
    z_coord = start_pos[2] if len(start_pos) > 2 else -2
    path_points_3d = [[p[0], p[1], z_coord] for p in path_points]
    
    return path_points_3d, path_length

def calculate_lsl_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Left-Straight-Left Dubins path."""
    # Left turn centers
    C1 = start_pos + R * np.array([-np.sin(start_angle), np.cos(start_angle)])
    C2 = end_pos + R * np.array([-np.sin(end_angle), np.cos(end_angle)])
    
    # Distance between centers
    d = np.linalg.norm(C2 - C1)
    
    if d < 2 * R:
        return None  # No valid LSL path
    
    # Calculate tangent points
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    
    # Tangent points
    T1 = C1 + R * np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    T2 = C2 + R * np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    
    # Generate path points
    path_points = []
    
    # First arc (left turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    
    # Ensure we turn left (counterclockwise)
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    # Straight segment
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])  # Skip first point to avoid duplication
    
    # Second arc (left turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    
    # Ensure we turn left (counterclockwise)
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])  # Skip first point to avoid duplication
    
    # Calculate total length
    path_length = calculate_path_length_2d(path_points)
    
    return path_points, path_length

def calculate_rsr_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Right-Straight-Right Dubins path."""
    # Right turn centers
    C1 = start_pos + R * np.array([np.sin(start_angle), -np.cos(start_angle)])
    C2 = end_pos + R * np.array([np.sin(end_angle), -np.cos(end_angle)])
    
    # Distance between centers
    d = np.linalg.norm(C2 - C1)
    
    if d < 2 * R:
        return None  # No valid RSR path
    
    # Calculate tangent points
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    
    # Tangent points
    T1 = C1 + R * np.array([np.cos(theta - np.pi/2), np.sin(theta - np.pi/2)])
    T2 = C2 + R * np.array([np.cos(theta - np.pi/2), np.sin(theta - np.pi/2)])
    
    # Generate path points
    path_points = []
    
    # First arc (right turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    
    # Ensure we turn right (clockwise)
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    # Straight segment
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (right turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    
    # Ensure we turn right (clockwise)
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    # Calculate total length
    path_length = calculate_path_length_2d(path_points)
    
    return path_points, path_length

def calculate_lsr_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Left-Straight-Right Dubins path."""
    # Turn centers
    C1 = start_pos + R * np.array([-np.sin(start_angle), np.cos(start_angle)])  # Left
    C2 = end_pos + R * np.array([np.sin(end_angle), -np.cos(end_angle)])       # Right
    
    # Distance between centers
    d = np.linalg.norm(C2 - C1)
    
    if d < 2 * R:
        return None  # No valid LSR path
    
    # Calculate external tangent
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    alpha = np.arccos(2 * R / d)
    
    # Tangent points
    T1 = C1 + R * np.array([np.cos(theta + alpha), np.sin(theta + alpha)])
    T2 = C2 + R * np.array([np.cos(theta + alpha), np.sin(theta + alpha)])
    
    # Generate path points
    path_points = []
    
    # First arc (left turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    # Straight segment
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (right turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length_2d(path_points)
    return path_points, path_length

def calculate_rsl_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Right-Straight-Left Dubins path."""
    # Turn centers
    C1 = start_pos + R * np.array([np.sin(start_angle), -np.cos(start_angle)])  # Right
    C2 = end_pos + R * np.array([-np.sin(end_angle), np.cos(end_angle)])       # Left
    
    # Distance between centers
    d = np.linalg.norm(C2 - C1)
    
    if d < 2 * R:
        return None  # No valid RSL path
    
    # Calculate external tangent
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    alpha = np.arccos(2 * R / d)
    
    # Tangent points
    T1 = C1 + R * np.array([np.cos(theta - alpha), np.sin(theta - alpha)])
    T2 = C2 + R * np.array([np.cos(theta - alpha), np.sin(theta - alpha)])
    
    # Generate path points
    path_points = []
    
    # First arc (right turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    # Straight segment
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (left turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length_2d(path_points)
    return path_points, path_length

def generate_arc_points(center, radius, start_angle, end_angle, num_points):
    """Generate points along a circular arc."""
    angles = np.linspace(start_angle, end_angle, num_points)
    points = []
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        points.append([x, y])
    return points

def generate_straight_points(start, end, num_points):
    """Generate points along a straight line."""
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        points.append([x, y])
    return points

def calculate_path_length_2d(points):
    """Calculate total path length for 2D points."""
    if len(points) < 2:
        return 0.0
    
    length = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        length += np.sqrt(dx**2 + dy**2)
    return length

def generate_fallback_path(start_pos, end_pos, start_angle, end_angle):
    """Fallback path when Dubins calculation fails."""
    # Simple straight line
    num_points = 50
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start_pos[0] + t * (end_pos[0] - start_pos[0])
        y = start_pos[1] + t * (end_pos[1] - start_pos[1])
        points.append([x, y])
    
    length = np.linalg.norm(end_pos - start_pos)
    return points, length

@app.route('/api/calculate_dubins_path', methods=['POST'])
def calculate_dubins_path():
    """Calculate Dubins path using Python algorithms."""
    try:
        data = request.json
        
        # Extract parameters
        start_pos = data.get('start_position', [0, 0, 0])
        end_pos = data.get('end_position', [10, 10, 0])
        start_dir = data.get('start_direction', 0)  # degrees
        end_dir = data.get('end_direction', 90)     # degrees
        turning_radius = data.get('turning_radius', 1.5)
        algorithm = data.get('algorithm', 'traditional_dubins')
        
        # Convert to 3D if needed (add depth for 2D interface)
        if len(start_pos) == 2:
            start_pos = [start_pos[0], start_pos[1], -2]
        if len(end_pos) == 2:
            end_pos = [end_pos[0], end_pos[1], -2]
        
        if DUBINS_AVAILABLE:
            # Use the actual Dubins implementation
            # Convert direction angles to unit vectors
            start_angle_rad = np.radians(start_dir)
            end_angle_rad = np.radians(end_dir)
            
            start_velocity = np.array([np.cos(start_angle_rad), np.sin(start_angle_rad), 0])
            end_velocity = np.array([np.cos(end_angle_rad), np.sin(end_angle_rad), 0])
            
            # Create configurations
            start_config = Configuration3D(
                position=np.array(start_pos),
                orientation=np.array([0, 0, start_angle_rad]),
                velocity_direction=start_velocity
            )
            
            end_config = Configuration3D(
                position=np.array(end_pos),
                orientation=np.array([0, 0, end_angle_rad]),
                velocity_direction=end_velocity
            )
            
            # Choose planner based on algorithm
            if algorithm == 'bio_inspired_dubins':
                planner = BioInspiredDubinsPlanner(min_turn_radius=turning_radius)
                path = planner.plan_jet_propulsion_path(start_config, end_config, max_jets=8)
            else:
                planner = Dubins3DPlanner(min_turn_radius=turning_radius)
                path = planner.plan_csc_curve(start_config, end_config)
            
            # Convert path points to list for JSON serialization
            path_points = path.path_points.tolist() if path.path_points is not None else []
            path_length = float(path.path_length)
            path_type = path.curve_type if hasattr(path, 'curve_type') else 'CSC'
            
        else:
            # Use proper Dubins implementation
            path_points, path_length = generate_proper_dubins_path(
                start_pos, end_pos, start_dir, end_dir, turning_radius
            )
            path_type = 'Dubins-CSC'
        
        # Determine if path is feasible
        feasible = len(path_points) > 1 and path_length > 0
        
        # Determine path type based on algorithm and feasibility
        if not feasible:
            path_type = 'Infeasible'
        elif algorithm == 'bio_inspired_dubins' and DUBINS_AVAILABLE:
            path_type = 'Bio-Inspired'
        
        return jsonify({
            'success': True,
            'feasible': feasible,
            'path_points': path_points,
            'path_length': float(path_length),
            'path_type': path_type,
            'algorithm': algorithm,
            'parameters': {
                'turning_radius': turning_radius,
                'start_direction': start_dir,
                'end_direction': end_dir
            }
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'feasible': False,
            'path_points': [],
            'path_length': 0,
            'path_type': 'Error'
        }), 500

@app.route('/api/export_results')
def export_results():
    """Export results as JSON."""
    global current_results
    
    if not current_results:
        return jsonify({'error': 'No results to export'}), 404
    
    export_data = {
        'timestamp': datetime.now().isoformat(),
        'results': current_results,
        'scenarios': current_scenarios,
        'metadata': {
            'version': '1.0',
            'description': 'Underwater Vehicle Trajectory Planning Results'
        }
    }
    
    return jsonify(export_data)

if __name__ == '__main__':
    # Ensure directories exist
    os.makedirs('WebInterface/static/plots', exist_ok=True)
    os.makedirs('WebInterface/templates', exist_ok=True)
    
    print("🌊 Starting Underwater Vehicle Trajectory Planning Web Interface...")
    print("📊 Dashboard will be available at: http://localhost:5000")
    
    app.run(debug=True, host='0.0.0.0', port=5000)
