"""
Enhanced Trajectory Visualization System

Creates beautiful, interactive visualizations for underwater vehicle trajectory planning
with comprehensive performance analysis and comparison tools.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-GUI backend for thread safety
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
from typing import Dict, List, Tuple, Optional
import sys
import os
import json
from datetime import datetime

# Add paths for existing algorithms
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(os.path.join(parent_dir, 'Algorithms', 'DubinsCurves'))
sys.path.append(os.path.join(parent_dir, 'Algorithms', 'RRT_Variants'))
sys.path.append(os.path.join(parent_dir, 'Simulations'))

# For now, we'll create mock implementations since the actual modules may not be available
class MockConfiguration3D:
    def __init__(self, position, orientation, velocity_direction):
        self.position = position
        self.orientation = orientation
        self.velocity_direction = velocity_direction

class MockPath:
    def __init__(self, points):
        self.path_points = points
        self.path_length = sum(np.linalg.norm(np.array(points[i+1]) - np.array(points[i])) 
                              for i in range(len(points)-1)) if len(points) > 1 else 0

class MockPlanner:
    def __init__(self, min_turn_radius=1.0):
        self.min_turn_radius = min_turn_radius
    
    def plan_path(self, start_config, goal_config):
        # Simple straight line path
        points = [start_config.position, goal_config.position]
        return MockPath(points)
    
    def plan_jet_propulsion_path(self, start_config, goal_config, max_jets=5):
        # Simple path with some intermediate points for jet propulsion
        start = start_config.position
        goal = goal_config.position
        
        # Create path with intermediate jet points
        points = [start]
        for i in range(1, max_jets):
            t = i / max_jets
            point = start + t * (goal - start)
            # Add some variation for bio-inspired behavior
            point += np.random.normal(0, 0.5, 3)
            points.append(point)
        points.append(goal)
        
        return MockPath(points)

# Try to import real modules, fall back to mocks
try:
    from dubins_3d import Configuration3D, BioInspiredDubinsPlanner
    from underwater_simulation_environment import UnderwaterEnvironment, Obstacle
    print("✅ Successfully imported trajectory planning modules")
except ImportError:
    print("⚠️ Using mock trajectory planning modules for demonstration")
    Configuration3D = MockConfiguration3D
    BioInspiredDubinsPlanner = MockPlanner
    TraditionalDubinsPlanner = MockPlanner

# Set style for professional plots
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")

class TrajectoryVisualizer:
    """Enhanced visualization system for trajectory planning algorithms."""
    
    def __init__(self, save_dir: str = "WebInterface/static/plots"):
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        
        # Initialize planners
        try:
            self.dubins_planner = BioInspiredDubinsPlanner(min_turn_radius=1.0)
            self.traditional_planner = TraditionalDubinsPlanner(min_turn_radius=1.0)
        except:
            print("Warning: Planners not available")
            self.dubins_planner = None
            self.traditional_planner = None
    
    def create_3d_trajectory_plot(self, scenarios: List[Dict], algorithms: List[str]) -> str:
        """Create comprehensive 3D trajectory comparison plot."""
        
        fig = plt.figure(figsize=(20, 12))
        
        # Create subplots for each scenario
        num_scenarios = len(scenarios)
        
        for i, scenario in enumerate(scenarios):
            ax = fig.add_subplot(2, 3, i + 1, projection='3d')
            
            start_pos = np.array(scenario['start'])
            goal_pos = np.array(scenario['goal'])
            obstacles = scenario.get('obstacles', [])
            
            # Plot obstacles
            for obs in obstacles:
                self._plot_sphere(ax, obs['position'], obs['radius'], alpha=0.3, color='red')
            
            # Plot trajectories for each algorithm
            colors = ['blue', 'green', 'orange', 'purple']
            
            for j, algorithm in enumerate(algorithms):
                if algorithm == 'traditional_dubins':
                    path = self._generate_traditional_dubins_path(start_pos, goal_pos)
                elif algorithm == 'bio_inspired_dubins':
                    path = self._generate_bio_inspired_path(start_pos, goal_pos)
                elif algorithm == 'rrt_star':
                    path = self._generate_rrt_path(start_pos, goal_pos, obstacles)
                else:
                    continue
                
                if path is not None and len(path) > 1:
                    path_array = np.array(path)
                    ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2], 
                           color=colors[j % len(colors)], linewidth=2, 
                           label=algorithm.replace('_', ' ').title(), alpha=0.8)
            
            # Plot start and goal
            ax.scatter(*start_pos, color='green', s=100, marker='o', label='Start')
            ax.scatter(*goal_pos, color='red', s=100, marker='*', label='Goal')
            
            # Formatting
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'{scenario["name"]}', fontsize=12, fontweight='bold')
            ax.legend(fontsize=8)
            
            # Set equal aspect ratio
            max_range = max(
                np.ptp([start_pos[0], goal_pos[0]]),
                np.ptp([start_pos[1], goal_pos[1]]),
                np.ptp([start_pos[2], goal_pos[2]])
            )
            ax.set_xlim(start_pos[0] - max_range/2, start_pos[0] + max_range/2)
            ax.set_ylim(start_pos[1] - max_range/2, start_pos[1] + max_range/2)
            ax.set_zlim(start_pos[2] - max_range/2, start_pos[2] + max_range/2)
        
        # Add overall title
        fig.suptitle('🌊 Underwater Vehicle Trajectory Planning Comparison', 
                    fontsize=16, fontweight='bold', y=0.95)
        
        plt.tight_layout()
        
        # Save plot
        filename = f"trajectory_comparison_3d_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        filepath = os.path.join(self.save_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
        
        return filename
    
    def create_performance_dashboard(self, results: Dict) -> str:
        """Create comprehensive performance analysis dashboard."""
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('📊 Performance Analysis Dashboard', fontsize=16, fontweight='bold')
        
        # Extract data
        algorithms = list(results.keys())
        scenarios = list(results[algorithms[0]].keys())
        
        # 1. Path Length Comparison
        ax = axes[0, 0]
        path_lengths = []
        for scenario in scenarios:
            scenario_lengths = [results[alg][scenario]['path_length'] for alg in algorithms]
            path_lengths.append(scenario_lengths)
        
        x = np.arange(len(scenarios))
        width = 0.2
        for i, alg in enumerate(algorithms):
            lengths = [results[alg][scenario]['path_length'] for scenario in scenarios]
            ax.bar(x + i*width, lengths, width, label=alg.replace('_', ' ').title())
        
        ax.set_xlabel('Scenarios')
        ax.set_ylabel('Path Length (m)')
        ax.set_title('Path Length Comparison')
        ax.set_xticks(x + width * (len(algorithms)-1) / 2)
        ax.set_xticklabels(scenarios, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 2. Energy Cost Comparison
        ax = axes[0, 1]
        for i, alg in enumerate(algorithms):
            costs = [results[alg][scenario]['energy_cost'] for scenario in scenarios]
            ax.bar(x + i*width, costs, width, label=alg.replace('_', ' ').title())
        
        ax.set_xlabel('Scenarios')
        ax.set_ylabel('Energy Cost')
        ax.set_title('Energy Cost Comparison')
        ax.set_xticks(x + width * (len(algorithms)-1) / 2)
        ax.set_xticklabels(scenarios, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3. Planning Time Comparison (Log Scale)
        ax = axes[0, 2]
        for i, alg in enumerate(algorithms):
            times = [results[alg][scenario]['planning_time'] * 1000 for scenario in scenarios]  # Convert to ms
            ax.bar(x + i*width, times, width, label=alg.replace('_', ' ').title())
        
        ax.set_xlabel('Scenarios')
        ax.set_ylabel('Planning Time (ms)')
        ax.set_title('Planning Time Comparison')
        ax.set_yscale('log')
        ax.set_xticks(x + width * (len(algorithms)-1) / 2)
        ax.set_xticklabels(scenarios, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 4. Success Rate Heatmap
        ax = axes[1, 0]
        success_matrix = []
        for alg in algorithms:
            success_row = [results[alg][scenario]['success_rate'] for scenario in scenarios]
            success_matrix.append(success_row)
        
        im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=1)
        ax.set_xticks(range(len(scenarios)))
        ax.set_yticks(range(len(algorithms)))
        ax.set_xticklabels(scenarios, rotation=45)
        ax.set_yticklabels([alg.replace('_', ' ').title() for alg in algorithms])
        ax.set_title('Success Rate Heatmap')
        
        # Add text annotations
        for i in range(len(algorithms)):
            for j in range(len(scenarios)):
                text = ax.text(j, i, f'{success_matrix[i][j]:.2f}',
                             ha="center", va="center", color="black", fontweight='bold')
        
        plt.colorbar(im, ax=ax)
        
        # 5. Efficiency Radar Chart
        ax = axes[1, 1]
        
        # Normalize metrics for radar chart
        metrics = ['Path Length', 'Energy Cost', 'Planning Time', 'Success Rate']
        
        angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
        angles += angles[:1]  # Complete the circle
        
        for alg in algorithms:
            # Calculate average normalized scores
            avg_path = np.mean([results[alg][scenario]['path_length'] for scenario in scenarios])
            avg_energy = np.mean([results[alg][scenario]['energy_cost'] for scenario in scenarios])
            avg_time = np.mean([results[alg][scenario]['planning_time'] for scenario in scenarios])
            avg_success = np.mean([results[alg][scenario]['success_rate'] for scenario in scenarios])
            
            # Normalize (lower is better for path, energy, time; higher is better for success)
            max_path = max([np.mean([results[a][s]['path_length'] for s in scenarios]) for a in algorithms])
            max_energy = max([np.mean([results[a][s]['energy_cost'] for s in scenarios]) for a in algorithms])
            max_time = max([np.mean([results[a][s]['planning_time'] for s in scenarios]) for a in algorithms])
            
            values = [
                1 - (avg_path / max_path),  # Inverted: lower path length is better
                1 - (avg_energy / max_energy),  # Inverted: lower energy is better
                1 - (avg_time / max_time),  # Inverted: lower time is better
                avg_success  # Higher success rate is better
            ]
            values += values[:1]  # Complete the circle
            
            ax.plot(angles, values, 'o-', linewidth=2, label=alg.replace('_', ' ').title())
            ax.fill(angles, values, alpha=0.25)
        
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metrics)
        ax.set_ylim(0, 1)
        ax.set_title('Algorithm Efficiency Radar')
        ax.legend()
        ax.grid(True)
        
        # 6. Scalability Analysis
        ax = axes[1, 2]
        
        # Plot path length vs distance for scalability
        distances = []
        path_ratios = []
        
        for scenario in scenarios:
            # Calculate straight-line distance
            start = np.array([0, 0, -2])  # Assuming standard start
            if 'Simple' in scenario:
                goal = np.array([10, 8, -6])
            elif 'Moderate' in scenario:
                goal = np.array([15, 12, -8])
            else:
                goal = np.array([20, 15, -10])
            
            straight_distance = np.linalg.norm(goal - start)
            distances.append(straight_distance)
            
            for alg in algorithms:
                path_length = results[alg][scenario]['path_length']
                ratio = path_length / straight_distance
                path_ratios.append(ratio)
        
        # Create scatter plot
        colors = plt.cm.Set1(np.linspace(0, 1, len(algorithms)))
        for i, alg in enumerate(algorithms):
            alg_ratios = [results[alg][scenario]['path_length'] / distances[j] 
                         for j, scenario in enumerate(scenarios)]
            ax.scatter(distances, alg_ratios, c=[colors[i]], 
                      label=alg.replace('_', ' ').title(), s=100, alpha=0.7)
        
        ax.set_xlabel('Straight-line Distance (m)')
        ax.set_ylabel('Path Length Ratio')
        ax.set_title('Scalability Analysis')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        filename = f"performance_dashboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        filepath = os.path.join(self.save_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
        
        return filename
    
    def create_algorithm_deep_dive(self, algorithm: str, scenarios: List[Dict]) -> str:
        """Create detailed analysis for a specific algorithm."""
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'🔬 Deep Dive: {algorithm.replace("_", " ").title()}', 
                    fontsize=16, fontweight='bold')
        
        # Generate paths for analysis
        paths_data = []
        for scenario in scenarios:
            start_pos = np.array(scenario['start'])
            goal_pos = np.array(scenario['goal'])
            
            if algorithm == 'traditional_dubins':
                path = self._generate_traditional_dubins_path(start_pos, goal_pos)
            elif algorithm == 'bio_inspired_dubins':
                path = self._generate_bio_inspired_path(start_pos, goal_pos)
            else:
                path = self._generate_rrt_path(start_pos, goal_pos, scenario.get('obstacles', []))
            
            if path:
                paths_data.append({
                    'scenario': scenario['name'],
                    'path': np.array(path),
                    'length': self._calculate_path_length(path),
                    'smoothness': self._calculate_path_smoothness(path),
                    'energy': self._estimate_energy_cost(path)
                })
        
        # 1. Path Curvature Analysis
        ax = axes[0, 0]
        for data in paths_data:
            curvatures = self._calculate_curvature(data['path'])
            ax.plot(curvatures, label=data['scenario'], linewidth=2)
        
        ax.set_xlabel('Path Segment')
        ax.set_ylabel('Curvature (1/m)')
        ax.set_title('Path Curvature Analysis')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 2. Velocity Profile
        ax = axes[0, 1]
        for data in paths_data:
            velocities = self._calculate_velocity_profile(data['path'])
            ax.plot(velocities, label=data['scenario'], linewidth=2)
        
        ax.set_xlabel('Path Segment')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3. Energy Consumption Over Path
        ax = axes[1, 0]
        for data in paths_data:
            energy_profile = self._calculate_energy_profile(data['path'])
            ax.plot(np.cumsum(energy_profile), label=data['scenario'], linewidth=2)
        
        ax.set_xlabel('Path Segment')
        ax.set_ylabel('Cumulative Energy')
        ax.set_title('Energy Consumption Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 4. Path Efficiency Metrics
        ax = axes[1, 1]
        
        metrics = ['Length Ratio', 'Smoothness', 'Energy Efficiency']
        scenario_names = [data['scenario'] for data in paths_data]
        
        # Calculate metrics
        length_ratios = []
        smoothness_scores = []
        energy_scores = []
        
        for data in paths_data:
            # Length ratio (path length / straight line distance)
            start = data['path'][0]
            end = data['path'][-1]
            straight_distance = np.linalg.norm(end - start)
            length_ratios.append(data['length'] / straight_distance)
            
            # Smoothness (inverse of average curvature)
            smoothness_scores.append(1 / (data['smoothness'] + 0.01))
            
            # Energy efficiency (inverse of energy per meter)
            energy_scores.append(1 / (data['energy'] / data['length'] + 0.01))
        
        x = np.arange(len(scenario_names))
        width = 0.25
        
        ax.bar(x - width, length_ratios, width, label='Length Ratio', alpha=0.8)
        ax.bar(x, smoothness_scores, width, label='Smoothness', alpha=0.8)
        ax.bar(x + width, energy_scores, width, label='Energy Efficiency', alpha=0.8)
        
        ax.set_xlabel('Scenarios')
        ax.set_ylabel('Normalized Score')
        ax.set_title('Path Quality Metrics')
        ax.set_xticks(x)
        ax.set_xticklabels(scenario_names, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        filename = f"algorithm_deepdive_{algorithm}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        filepath = os.path.join(self.save_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight', facecolor='white')
        plt.close()
        
        return filename
    
    def _plot_sphere(self, ax, center, radius, alpha=0.3, color='red'):
        """Plot a sphere in 3D."""
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, alpha=alpha, color=color)
    
    def _generate_traditional_dubins_path(self, start, goal):
        """Generate traditional Dubins path."""
        if self.traditional_planner is None:
            return None
        
        try:
            start_config = Configuration3D(
                position=start,
                orientation=np.zeros(3),
                velocity_direction=np.array([1, 0, 0])
            )
            
            goal_config = Configuration3D(
                position=goal,
                orientation=np.zeros(3),
                velocity_direction=np.array([1, 0, 0])
            )
            
            path = self.traditional_planner.plan_path(start_config, goal_config)
            return path.path_points if hasattr(path, 'path_points') else None
        except:
            return None
    
    def _generate_bio_inspired_path(self, start, goal):
        """Generate bio-inspired Dubins path."""
        if self.dubins_planner is None:
            return None
        
        try:
            start_config = Configuration3D(
                position=start,
                orientation=np.zeros(3),
                velocity_direction=np.array([1, 0, 0])
            )
            
            goal_config = Configuration3D(
                position=goal,
                orientation=np.zeros(3),
                velocity_direction=np.array([1, 0, 0])
            )
            
            path = self.dubins_planner.plan_jet_propulsion_path(start_config, goal_config, max_jets=5)
            return path.path_points if hasattr(path, 'path_points') else None
        except:
            return None
    
    def _generate_rrt_path(self, start, goal, obstacles):
        """Generate RRT* path."""
        # Simplified RRT path generation for visualization
        # In practice, this would use the actual RRT* implementation
        
        # Simple straight line with some deviation around obstacles
        path = [start]
        
        # Add intermediate points
        direction = goal - start
        distance = np.linalg.norm(direction)
        direction = direction / distance
        
        num_points = max(5, int(distance / 2))
        for i in range(1, num_points):
            t = i / num_points
            point = start + t * (goal - start)
            
            # Add some deviation to avoid obstacles
            for obs in obstacles:
                obs_pos = np.array(obs['position'])
                obs_radius = obs['radius']
                
                dist_to_obs = np.linalg.norm(point - obs_pos)
                if dist_to_obs < obs_radius + 2.0:  # Safety margin
                    # Deviate perpendicular to obstacle
                    to_obs = obs_pos - point
                    if np.linalg.norm(to_obs) > 0:
                        to_obs = to_obs / np.linalg.norm(to_obs)
                        perpendicular = np.array([-to_obs[1], to_obs[0], to_obs[2]])
                        point += perpendicular * (obs_radius + 2.0 - dist_to_obs)
            
            path.append(point)
        
        path.append(goal)
        return path
    
    def _calculate_path_length(self, path):
        """Calculate total path length."""
        if len(path) < 2:
            return 0
        
        total_length = 0
        for i in range(1, len(path)):
            total_length += np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        
        return total_length
    
    def _calculate_path_smoothness(self, path):
        """Calculate path smoothness (average curvature)."""
        if len(path) < 3:
            return 0
        
        curvatures = self._calculate_curvature(np.array(path))
        return np.mean(curvatures)
    
    def _calculate_curvature(self, path):
        """Calculate curvature at each point along the path."""
        if len(path) < 3:
            return [0]
        
        curvatures = []
        for i in range(1, len(path) - 1):
            p1, p2, p3 = path[i-1], path[i], path[i+1]
            
            # Calculate curvature using three points
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                v1_norm = v1 / np.linalg.norm(v1)
                v2_norm = v2 / np.linalg.norm(v2)
                
                # Curvature approximation
                cross_product = np.linalg.norm(np.cross(v1_norm, v2_norm))
                curvature = cross_product / (np.linalg.norm(v1) + np.linalg.norm(v2)) * 2
                curvatures.append(curvature)
            else:
                curvatures.append(0)
        
        return curvatures
    
    def _calculate_velocity_profile(self, path):
        """Calculate velocity profile along path."""
        if len(path) < 2:
            return [0]
        
        velocities = []
        for i in range(len(path) - 1):
            segment_length = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
            # Assume constant time step, velocity proportional to segment length
            velocity = segment_length * 2.0  # Scaling factor
            velocities.append(velocity)
        
        return velocities
    
    def _calculate_energy_profile(self, path):
        """Calculate energy consumption profile."""
        if len(path) < 2:
            return [0]
        
        energy_profile = []
        for i in range(len(path) - 1):
            segment_length = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
            # Energy proportional to distance and curvature
            if i > 0 and i < len(path) - 2:
                curvature = self._calculate_curvature(np.array(path[max(0, i-1):min(len(path), i+3)]))[0]
                energy = segment_length * (1 + curvature * 2)  # Higher energy for curved segments
            else:
                energy = segment_length
            
            energy_profile.append(energy)
        
        return energy_profile
    
    def _estimate_energy_cost(self, path):
        """Estimate total energy cost for path."""
        energy_profile = self._calculate_energy_profile(path)
        return sum(energy_profile)

def generate_sample_results():
    """Generate sample results for demonstration."""
    
    algorithms = ['traditional_dubins', 'bio_inspired_dubins', 'rrt_star']
    scenarios = ['Simple Navigation', 'Moderate Complexity', 'Complex Environment']
    
    results = {}
    
    for alg in algorithms:
        results[alg] = {}
        for scenario in scenarios:
            # Generate realistic sample data
            base_length = 15 if 'Simple' in scenario else 25 if 'Moderate' in scenario else 35
            
            if alg == 'traditional_dubins':
                path_length = base_length * np.random.uniform(0.9, 1.1)
                energy_cost = path_length * np.random.uniform(0.8, 1.0)
                planning_time = np.random.uniform(0.001, 0.003)
                success_rate = np.random.uniform(0.85, 0.95)
            elif alg == 'bio_inspired_dubins':
                path_length = base_length * np.random.uniform(0.85, 1.05)
                energy_cost = path_length * np.random.uniform(1.1, 1.3)
                planning_time = np.random.uniform(0.002, 0.004)
                success_rate = np.random.uniform(0.90, 0.98)
            else:  # rrt_star
                path_length = base_length * np.random.uniform(1.0, 1.2)
                energy_cost = path_length * np.random.uniform(0.9, 1.1)
                planning_time = np.random.uniform(0.5, 2.0)
                success_rate = np.random.uniform(0.95, 0.99)
            
            results[alg][scenario] = {
                'path_length': path_length,
                'energy_cost': energy_cost,
                'planning_time': planning_time,
                'success_rate': success_rate
            }
    
    return results

if __name__ == "__main__":
    # Test the visualizer
    print("🎨 Testing Enhanced Trajectory Visualizer...")
    
    visualizer = TrajectoryVisualizer()
    
    # Define test scenarios
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
                {'position': [15, 12, -8], 'radius': 1.0},
                {'position': [8, 10, -7], 'radius': 1.5}
            ]
        }
    ]
    
    algorithms = ['traditional_dubins', 'bio_inspired_dubins', 'rrt_star']
    
    # Generate visualizations
    print("📊 Creating 3D trajectory comparison...")
    trajectory_plot = visualizer.create_3d_trajectory_plot(scenarios, algorithms)
    print(f"✅ Saved: {trajectory_plot}")
    
    print("📈 Creating performance dashboard...")
    sample_results = generate_sample_results()
    dashboard_plot = visualizer.create_performance_dashboard(sample_results)
    print(f"✅ Saved: {dashboard_plot}")
    
    print("🔬 Creating algorithm deep dive...")
    deepdive_plot = visualizer.create_algorithm_deep_dive('bio_inspired_dubins', scenarios)
    print(f"✅ Saved: {deepdive_plot}")
    
    print("\n🎉 Visualization system test completed!")
    print(f"📁 All plots saved to: {visualizer.save_dir}")
