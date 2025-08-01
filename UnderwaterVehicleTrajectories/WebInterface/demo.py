"""
Simple Dubins Path Demo Interface

A clean, minimal interface for viewing and testing Dubins paths.
"""

from flask import Flask, render_template, request, jsonify
import numpy as np
import os
import sys

# Add Dubins algorithms to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Algorithms', 'DubinsCurves'))

app = Flask(__name__)

def calculate_dubins_path(start_pos, end_pos, start_angle, end_angle, turning_radius):
    """Calculate proper Dubins path using LSL, RSR, LSR, or RSL patterns."""
    
    # Convert to numpy arrays and radians
    start_pos = np.array(start_pos[:2])  # Use only x, y for 2D
    end_pos = np.array(end_pos[:2])
    start_angle = np.radians(start_angle)
    end_angle = np.radians(end_angle)
    
    # Calculate all possible Dubins paths
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
        return None, 0, 'No feasible path'
    
    # Choose shortest path
    best_path = min(paths, key=lambda x: x[1][1])
    path_type, (path_points, path_length) = best_path
    
    return path_points, path_length, path_type

def calculate_lsl_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Left-Straight-Left Dubins path."""
    # Left turn centers
    C1 = start_pos + R * np.array([-np.sin(start_angle), np.cos(start_angle)])
    C2 = end_pos + R * np.array([-np.sin(end_angle), np.cos(end_angle)])
    
    d = np.linalg.norm(C2 - C1)
    if d < 2 * R:
        return None
    
    # Calculate tangent points
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    T1 = C1 + R * np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    T2 = C2 + R * np.array([np.cos(theta + np.pi/2), np.sin(theta + np.pi/2)])
    
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
    
    # Second arc (left turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length(path_points)
    return path_points, path_length

def calculate_rsr_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Right-Straight-Right Dubins path."""
    # Right turn centers
    C1 = start_pos + R * np.array([np.sin(start_angle), -np.cos(start_angle)])
    C2 = end_pos + R * np.array([np.sin(end_angle), -np.cos(end_angle)])
    
    d = np.linalg.norm(C2 - C1)
    if d < 2 * R:
        return None
    
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    T1 = C1 + R * np.array([np.cos(theta - np.pi/2), np.sin(theta - np.pi/2)])
    T2 = C2 + R * np.array([np.cos(theta - np.pi/2), np.sin(theta - np.pi/2)])
    
    path_points = []
    
    # First arc (right turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (right turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length(path_points)
    return path_points, path_length

def calculate_lsr_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Left-Straight-Right Dubins path."""
    C1 = start_pos + R * np.array([-np.sin(start_angle), np.cos(start_angle)])
    C2 = end_pos + R * np.array([np.sin(end_angle), -np.cos(end_angle)])
    
    d = np.linalg.norm(C2 - C1)
    if d < 2 * R:
        return None
    
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    alpha = np.arccos(2 * R / d)
    
    T1 = C1 + R * np.array([np.cos(theta + alpha), np.sin(theta + alpha)])
    T2 = C2 + R * np.array([np.cos(theta + alpha), np.sin(theta + alpha)])
    
    path_points = []
    
    # First arc (left turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (right turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length(path_points)
    return path_points, path_length

def calculate_rsl_path(start_pos, end_pos, start_angle, end_angle, R):
    """Calculate Right-Straight-Left Dubins path."""
    C1 = start_pos + R * np.array([np.sin(start_angle), -np.cos(start_angle)])
    C2 = end_pos + R * np.array([-np.sin(end_angle), np.cos(end_angle)])
    
    d = np.linalg.norm(C2 - C1)
    if d < 2 * R:
        return None
    
    theta = np.arctan2(C2[1] - C1[1], C2[0] - C1[0])
    alpha = np.arccos(2 * R / d)
    
    T1 = C1 + R * np.array([np.cos(theta - alpha), np.sin(theta - alpha)])
    T2 = C2 + R * np.array([np.cos(theta - alpha), np.sin(theta - alpha)])
    
    path_points = []
    
    # First arc (right turn)
    start_arc_angle = np.arctan2(start_pos[1] - C1[1], start_pos[0] - C1[0])
    end_arc_angle = np.arctan2(T1[1] - C1[1], T1[0] - C1[0])
    if end_arc_angle > start_arc_angle:
        end_arc_angle -= 2 * np.pi
    
    arc1_points = generate_arc_points(C1, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc1_points)
    
    straight_points = generate_straight_points(T1, T2, 20)
    path_points.extend(straight_points[1:])
    
    # Second arc (left turn)
    start_arc_angle = np.arctan2(T2[1] - C2[1], T2[0] - C2[0])
    end_arc_angle = np.arctan2(end_pos[1] - C2[1], end_pos[0] - C2[0])
    if end_arc_angle < start_arc_angle:
        end_arc_angle += 2 * np.pi
    
    arc2_points = generate_arc_points(C2, R, start_arc_angle, end_arc_angle, 20)
    path_points.extend(arc2_points[1:])
    
    path_length = calculate_path_length(path_points)
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

def calculate_path_length(points):
    """Calculate total path length."""
    if len(points) < 2:
        return 0.0
    
    length = 0.0
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        length += np.sqrt(dx**2 + dy**2)
    return length

@app.route('/')
def index():
    """Main demo interface."""
    return render_template('demo.html')

@app.route('/api/calculate_path', methods=['POST'])
def api_calculate_path():
    """API endpoint for path calculation."""
    try:
        data = request.json
        
        start_pos = data.get('start_position', [0, 0])
        end_pos = data.get('end_position', [10, 10])
        start_angle = data.get('start_angle', 0)
        end_angle = data.get('end_angle', 90)
        turning_radius = data.get('turning_radius', 2.0)
        
        path_points, path_length, path_type = calculate_dubins_path(
            start_pos, end_pos, start_angle, end_angle, turning_radius
        )
        
        if path_points is None:
            return jsonify({
                'success': False,
                'error': 'No feasible path found',
                'path_points': [],
                'path_length': 0,
                'path_type': 'Infeasible'
            })
        
        return jsonify({
            'success': True,
            'path_points': path_points,
            'path_length': float(path_length),
            'path_type': path_type
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'path_points': [],
            'path_length': 0,
            'path_type': 'Error'
        })

if __name__ == '__main__':
    print("🌊 Starting Dubins Path Demo...")
    print("📊 Demo available at: http://localhost:3000")
    app.run(debug=True, host='0.0.0.0', port=3000)
