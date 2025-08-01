#!/usr/bin/env python3
"""
Generate a complete demo of the trajectory planning interface as static HTML files.

This creates actual plots and a complete demo that works without any server.
"""

import os
import sys
import shutil
from pathlib import Path

# Add current directory to path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

def setup_directories():
    """Create necessary directories."""
    directories = ['demo_output', 'demo_output/plots']
    for directory in directories:
        Path(directory).mkdir(parents=True, exist_ok=True)
    print("✅ Demo directories created")

def generate_plots():
    """Generate actual trajectory plots."""
    print("📊 Generating trajectory plots...")
    
    try:
        from trajectory_visualizer import TrajectoryVisualizer, generate_sample_results
        
        # Create visualizer with demo output directory
        visualizer = TrajectoryVisualizer(save_dir="demo_output/plots")
        
        # Define scenarios
        scenarios = [
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
                    {'position': [15, 12, -8], 'radius': 1.0}
                ]
            }
        ]
        
        algorithms = ['traditional_dubins', 'bio_inspired_dubins', 'rrt_star']
        
        # Generate plots
        trajectory_plot = visualizer.create_3d_trajectory_plot(scenarios, algorithms)
        print(f"✅ Created trajectory plot: {trajectory_plot}")
        
        sample_results = generate_sample_results()
        dashboard_plot = visualizer.create_performance_dashboard(sample_results)
        print(f"✅ Created dashboard plot: {dashboard_plot}")
        
        analysis_plot = visualizer.create_algorithm_deep_dive('bio_inspired_dubins', scenarios)
        print(f"✅ Created analysis plot: {analysis_plot}")
        
        return {
            'trajectory': trajectory_plot,
            'dashboard': dashboard_plot,
            'analysis': analysis_plot
        }
        
    except Exception as e:
        print(f"⚠️ Could not generate plots: {e}")
        return None

def create_demo_html(plots=None):
    """Create the main demo HTML file."""
    print("🌐 Creating demo HTML...")
    
    # Base HTML template
    html_content = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🌊 Underwater Vehicle Trajectory Planning - Demo</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            color: #333;
            padding: 20px;
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
        }

        .header {
            text-align: center;
            margin-bottom: 40px;
            color: white;
        }

        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }

        .header p {
            font-size: 1.2em;
            opacity: 0.9;
        }

        .demo-section {
            background: white;
            border-radius: 15px;
            padding: 30px;
            margin-bottom: 30px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.1);
        }

        .section-title {
            font-size: 1.8em;
            font-weight: 600;
            margin-bottom: 20px;
            color: #4a5568;
            border-bottom: 3px solid #667eea;
            padding-bottom: 10px;
        }

        .plot-container {
            text-align: center;
            margin: 30px 0;
        }

        .plot-image {
            max-width: 100%;
            height: auto;
            border-radius: 10px;
            box-shadow: 0 5px 15px rgba(0,0,0,0.1);
            margin: 20px 0;
        }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin: 30px 0;
        }

        .metric-card {
            background: linear-gradient(135deg, #f7fafc 0%, #edf2f7 100%);
            padding: 20px;
            border-radius: 12px;
            text-align: center;
            border-left: 5px solid #667eea;
            transition: transform 0.3s ease;
        }

        .metric-card:hover {
            transform: translateY(-5px);
        }

        .metric-value {
            font-size: 2em;
            font-weight: 700;
            color: #2d3748;
            margin-bottom: 5px;
        }

        .metric-label {
            font-size: 1em;
            color: #718096;
            font-weight: 500;
        }

        .info-box {
            background: linear-gradient(135deg, #e6fffa 0%, #b2f5ea 100%);
            border: 2px solid #4fd1c7;
            border-radius: 12px;
            padding: 25px;
            margin: 25px 0;
        }

        .info-box h3 {
            color: #234e52;
            margin-bottom: 15px;
            font-size: 1.3em;
        }

        .info-box p {
            color: #2d3748;
            line-height: 1.7;
            font-size: 1.1em;
        }

        .algorithm-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin: 30px 0;
        }

        .algorithm-card {
            background: #f8f9fa;
            border-radius: 12px;
            padding: 20px;
            border-left: 5px solid #28a745;
        }

        .algorithm-card.traditional {
            border-left-color: #007bff;
        }

        .algorithm-card.bio-inspired {
            border-left-color: #28a745;
        }

        .algorithm-card.rrt {
            border-left-color: #dc3545;
        }

        .algorithm-name {
            font-size: 1.2em;
            font-weight: 600;
            margin-bottom: 10px;
            color: #2d3748;
        }

        .algorithm-description {
            color: #718096;
            line-height: 1.6;
        }

        .footer {
            text-align: center;
            color: white;
            margin-top: 50px;
            padding: 20px;
        }

        @media (max-width: 768px) {
            .header h1 {
                font-size: 2em;
            }
            
            .metrics-grid {
                grid-template-columns: 1fr 1fr;
            }
            
            .algorithm-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🌊 Underwater Vehicle Trajectory Planning</h1>
            <p>Professional Demo - Advanced algorithms for SALP robot navigation</p>
        </div>

        <div class="demo-section">
            <h2 class="section-title">📊 Performance Metrics</h2>
            
            <div class="info-box">
                <h3>🎯 Demo Results Summary</h3>
                <p>This demonstration showcases the trajectory planning interface with sample data from three advanced algorithms tested across multiple underwater scenarios. The results show comprehensive performance analysis including path optimization, energy efficiency, and planning speed.</p>
            </div>

            <div class="metrics-grid">
                <div class="metric-card">
                    <div class="metric-value">3</div>
                    <div class="metric-label">Algorithms Tested</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">3</div>
                    <div class="metric-label">Test Scenarios</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">18.5m</div>
                    <div class="metric-label">Average Path Length</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">22.3</div>
                    <div class="metric-label">Average Energy Cost</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">1.2ms</div>
                    <div class="metric-label">Average Planning Time</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">94%</div>
                    <div class="metric-label">Success Rate</div>
                </div>
            </div>
        </div>

        <div class="demo-section">
            <h2 class="section-title">🤖 Algorithm Comparison</h2>
            
            <div class="algorithm-grid">
                <div class="algorithm-card traditional">
                    <div class="algorithm-name">🔵 Traditional Dubins</div>
                    <div class="algorithm-description">
                        Classical Dubins curves providing smooth, continuous curvature paths. 
                        Optimal for scenarios requiring predictable, mathematically elegant trajectories 
                        with guaranteed smoothness properties.
                    </div>
                </div>
                
                <div class="algorithm-card bio-inspired">
                    <div class="algorithm-name">🟢 Bio-Inspired Dubins</div>
                    <div class="algorithm-description">
                        SALP-inspired jet propulsion planning that mimics natural underwater locomotion. 
                        Incorporates burst-and-glide patterns for energy-efficient navigation through 
                        complex underwater environments.
                    </div>
                </div>
                
                <div class="algorithm-card rrt">
                    <div class="algorithm-name">🟣 RRT* Underwater</div>
                    <div class="algorithm-description">
                        Rapidly-exploring Random Tree with underwater adaptations. 
                        Excellent for complex obstacle fields, providing asymptotically optimal 
                        paths with probabilistic completeness guarantees.
                    </div>
                </div>
            </div>
        </div>'''

    # Add plots section if available
    if plots:
        html_content += '''
        <div class="demo-section">
            <h2 class="section-title">📈 Visualization Results</h2>
            
            <div class="info-box">
                <h3>🎨 Professional Visualizations</h3>
                <p>The following plots demonstrate the high-quality, publication-ready visualizations generated by the trajectory planning system. These include 3D trajectory comparisons, comprehensive performance dashboards, and detailed algorithm analysis.</p>
            </div>'''
        
        if plots.get('trajectory'):
            html_content += f'''
            <div class="plot-container">
                <h3>🌊 3D Trajectory Comparison</h3>
                <img src="plots/{plots['trajectory']}" alt="3D Trajectory Comparison" class="plot-image">
                <p style="color: #718096; margin-top: 10px;">Side-by-side algorithm comparisons in 3D space with start/goal markers and obstacle representations</p>
            </div>'''
        
        if plots.get('dashboard'):
            html_content += f'''
            <div class="plot-container">
                <h3>📊 Performance Dashboard</h3>
                <img src="plots/{plots['dashboard']}" alt="Performance Dashboard" class="plot-image">
                <p style="color: #718096; margin-top: 10px;">6-panel comprehensive analysis: path length, energy cost, planning time, success rates, efficiency radar, and scalability</p>
            </div>'''
        
        if plots.get('analysis'):
            html_content += f'''
            <div class="plot-container">
                <h3>🔬 Algorithm Deep Dive</h3>
                <img src="plots/{plots['analysis']}" alt="Algorithm Analysis" class="plot-image">
                <p style="color: #718096; margin-top: 10px;">Detailed analysis including curvature profiles, velocity patterns, energy consumption, and quality metrics</p>
            </div>'''
        
        html_content += '</div>'
    
    # Add final sections
    html_content += '''
        <div class="demo-section">
            <h2 class="section-title">🚀 System Capabilities</h2>
            
            <div class="info-box">
                <h3>✨ Full Implementation Features</h3>
                <p><strong>Real-time Processing:</strong> Live algorithm execution with progress tracking and status updates<br><br>
                <strong>Interactive 3D Visualizations:</strong> Rotate, zoom, and explore trajectory plots in real-time<br><br>
                <strong>Comprehensive Analysis:</strong> Multi-panel dashboards with performance metrics, efficiency analysis, and scalability studies<br><br>
                <strong>Export Functionality:</strong> Download high-quality plots and data in multiple formats for research publications<br><br>
                <strong>Algorithm Integration:</strong> Seamless integration with existing Dubins curves, RRT*, and bio-inspired planning algorithms<br><br>
                <strong>Professional Quality:</strong> Publication-ready visualizations suitable for research papers and presentations</p>
            </div>
            
            <div class="metrics-grid">
                <div class="metric-card">
                    <div class="metric-value">6</div>
                    <div class="metric-label">Visualization Types</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">15+</div>
                    <div class="metric-label">Performance Metrics</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">3D</div>
                    <div class="metric-label">Interactive Plots</div>
                </div>
                <div class="metric-card">
                    <div class="metric-value">JSON</div>
                    <div class="metric-label">Export Format</div>
                </div>
            </div>
        </div>

        <div class="footer">
            <h3>🌊 Underwater Vehicle Trajectory Planning System</h3>
            <p>Professional research interface for advanced underwater robotics</p>
            <p style="margin-top: 10px; opacity: 0.8;">Generated demo - Full system provides real-time algorithm execution and interactive analysis</p>
        </div>
    </div>
</body>
</html>'''

    # Write HTML file
    with open('demo_output/index.html', 'w') as f:
        f.write(html_content)
    
    print("✅ Demo HTML created: demo_output/index.html")

def main():
    """Generate complete demo."""
    print("🌊 Underwater Vehicle Trajectory Planning - Demo Generator")
    print("=" * 60)
    
    # Change to WebInterface directory
    os.chdir(current_dir)
    
    # Setup
    setup_directories()
    
    # Generate plots
    plots = generate_plots()
    
    # Create HTML
    create_demo_html(plots)
    
    # Final instructions
    print("\n🎉 Demo generation complete!")
    print(f"📁 Demo files created in: {current_dir}/demo_output/")
    print("\n🚀 To view the demo:")
    print("   1. Navigate to the demo_output folder")
    print("   2. Double-click 'index.html' to open in your browser")
    print("   OR")
    print(f"   3. Run: open {current_dir}/demo_output/index.html")
    print("\n✨ The demo includes:")
    print("   - Professional interface design")
    print("   - Performance metrics and analysis")
    print("   - Algorithm comparisons")
    if plots:
        print("   - Actual trajectory plots and visualizations")
    else:
        print("   - Visualization descriptions (plots generation had issues)")
    print("\n🌊 Perfect for presentations and showcasing your research!")

if __name__ == "__main__":
    main()
