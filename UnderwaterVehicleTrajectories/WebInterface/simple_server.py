#!/usr/bin/env python3
"""
Simple, reliable server for the Underwater Vehicle Trajectory Planning Web Interface

This version uses a minimal approach to avoid connection issues.
"""

import os
import sys
import socket
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json
import urllib.parse
import threading
import time

# Add current directory to path
current_dir = Path(__file__).parent
os.chdir(current_dir)

# Import our modules
sys.path.append(str(current_dir))
from trajectory_visualizer import TrajectoryVisualizer, generate_sample_results

class TrajectoryHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler for trajectory planning interface."""
    
    def __init__(self, *args, **kwargs):
        # Initialize visualizer
        self.visualizer = TrajectoryVisualizer(save_dir="static/plots")
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/' or self.path == '/index.html':
            self.serve_index()
        elif self.path == '/api/scenarios':
            self.serve_scenarios()
        elif self.path == '/api/algorithms':
            self.serve_algorithms()
        elif self.path == '/api/status':
            self.serve_status()
        elif self.path == '/api/results':
            self.serve_results()
        elif self.path.startswith('/plots/'):
            self.serve_plot()
        else:
            super().do_GET()
    
    def do_POST(self):
        """Handle POST requests."""
        if self.path == '/api/run_comparison':
            self.handle_comparison()
        else:
            self.send_error(404)
    
    def serve_index(self):
        """Serve the main HTML page."""
        try:
            with open('templates/index.html', 'r') as f:
                content = f.read()
            
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(content.encode())
        except Exception as e:
            self.send_error(500, f"Error serving index: {e}")
    
    def serve_scenarios(self):
        """Serve scenarios API."""
        scenarios = [
            {
                'id': 'simple',
                'name': 'Simple Navigation',
                'description': 'Open water navigation with minimal obstacles',
                'difficulty': 'Easy'
            },
            {
                'id': 'moderate',
                'name': 'Moderate Complexity',
                'description': 'Navigation with moderate obstacle density',
                'difficulty': 'Medium'
            },
            {
                'id': 'complex',
                'name': 'Complex Environment',
                'description': 'Dense obstacle field requiring advanced planning',
                'difficulty': 'Hard'
            }
        ]
        
        self.send_json_response(scenarios)
    
    def serve_algorithms(self):
        """Serve algorithms API."""
        algorithms = [
            {
                'id': 'traditional_dubins',
                'name': 'Traditional Dubins',
                'description': 'Classical Dubins curves for smooth path planning',
                'type': 'Classical'
            },
            {
                'id': 'bio_inspired_dubins',
                'name': 'Bio-Inspired Dubins',
                'description': 'SALP-inspired jet propulsion planning',
                'type': 'Bio-Inspired'
            },
            {
                'id': 'rrt_star',
                'name': 'RRT* Underwater',
                'description': 'Rapidly-exploring Random Tree with underwater adaptations',
                'type': 'Sampling-Based'
            }
        ]
        
        self.send_json_response(algorithms)
    
    def serve_status(self):
        """Serve status API."""
        # Simple status - always ready for demo
        status = {
            'status': 'idle',
            'progress': 0,
            'message': 'Ready to run analysis'
        }
        self.send_json_response(status)
    
    def serve_results(self):
        """Serve results API."""
        # Generate sample results
        results = generate_sample_results()
        
        # Create some plots
        scenarios = [
            {'name': 'Simple Navigation', 'start': [0, 0, -2], 'goal': [10, 8, -6], 'obstacles': []},
            {'name': 'Moderate Complexity', 'start': [0, 0, -2], 'goal': [15, 12, -8], 'obstacles': [{'position': [5, 4, -4], 'radius': 1.5}]},
            {'name': 'Complex Environment', 'start': [0, 0, -2], 'goal': [20, 15, -10], 'obstacles': [{'position': [5, 3, -3], 'radius': 1.2}]}
        ]
        
        algorithms = ['traditional_dubins', 'bio_inspired_dubins', 'rrt_star']
        
        try:
            trajectory_plot = self.visualizer.create_3d_trajectory_plot(scenarios, algorithms)
            dashboard_plot = self.visualizer.create_performance_dashboard(results)
            
            formatted_results = {
                'algorithms': algorithms,
                'scenarios': list(results[algorithms[0]].keys()),
                'data': results,
                'plots': {
                    'trajectory': trajectory_plot,
                    'dashboard': dashboard_plot
                },
                'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S')
            }
            
            self.send_json_response(formatted_results)
        except Exception as e:
            self.send_error(500, f"Error generating results: {e}")
    
    def serve_plot(self):
        """Serve plot images."""
        plot_name = self.path[7:]  # Remove '/plots/'
        plot_path = Path('static/plots') / plot_name
        
        if plot_path.exists():
            self.send_response(200)
            self.send_header('Content-type', 'image/png')
            self.end_headers()
            
            with open(plot_path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, "Plot not found")
    
    def handle_comparison(self):
        """Handle comparison request."""
        try:
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())
            
            # Simple response - just acknowledge
            response = {'message': 'Comparison started', 'status': 'running'}
            self.send_json_response(response)
            
        except Exception as e:
            self.send_error(500, f"Error handling comparison: {e}")
    
    def send_json_response(self, data):
        """Send JSON response."""
        json_data = json.dumps(data).encode()
        
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Content-length', str(len(json_data)))
        self.end_headers()
        self.wfile.write(json_data)
    
    def log_message(self, format, *args):
        """Override to reduce log spam."""
        pass

def find_free_port():
    """Find an available port."""
    ports_to_try = [8080, 8000, 3000, 5001, 8888, 9000]
    for port in ports_to_try:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('', port))
                return port
        except OSError:
            continue
    return 8080

def get_local_ip():
    """Get local IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

def main():
    """Main function."""
    print("🌊 Underwater Vehicle Trajectory Planning - Simple Server")
    print("=" * 60)
    
    # Setup directories
    Path("static/plots").mkdir(parents=True, exist_ok=True)
    Path("templates").mkdir(parents=True, exist_ok=True)
    print("✅ Directories created")
    
    # Find port and IP
    port = find_free_port()
    local_ip = get_local_ip()
    
    print(f"\n🚀 Starting simple HTTP server...")
    print(f"📊 Dashboard available at:")
    print(f"   🏠 Local:    http://localhost:{port}")
    print(f"   🌐 Network:  http://{local_ip}:{port}")
    print("🔄 Press Ctrl+C to stop the server")
    print("-" * 60)
    
    # Start server
    try:
        server = HTTPServer(('0.0.0.0', port), TrajectoryHandler)
        print("✅ Server started successfully")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n👋 Server stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    main()
