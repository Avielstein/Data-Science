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
    """Main dashboard page."""
    return render_template('index.html')

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
