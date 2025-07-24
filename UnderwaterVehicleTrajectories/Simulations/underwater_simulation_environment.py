"""
Comprehensive Underwater Vehicle Simulation Environment

Integrates 3D Dubins curves, improved RRT*, and bio-inspired vehicle dynamics
for testing and validation of trajectory planning algorithms.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from typing import List, Tuple, Dict, Optional
import time
import json
from dataclasses import dataclass, asdict
import sys
import os

# Add parent directories to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Algorithms', 'DubinsCurves'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Algorithms', 'RRT_Variants'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'VehicleModels', 'SALP_Dynamics'))

try:
    from dubins_3d import Configuration3D, DubinsPath3D, BioInspiredDubinsPlanner
    from rrt_star_underwater import (ImprovedRRTStar, BioInspiredRRTStar, 
                                   UnderwaterEnvironment, Obstacle, RRTNode)
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure all algorithm files are in the correct directories.")

@dataclass
class SimulationConfig:
    """Configuration for simulation parameters"""
    environment_bounds: Tuple[Tuple[float, float], ...] = ((-10, 20), (-10, 20), (-15, 0))
    num_obstacles: int = 8
    obstacle_radius_range: Tuple[float, float] = (0.8, 2.0)
    moving_obstacle_ratio: float = 0.3
    max_obstacle_velocity: float = 0.2
    
    # Vehicle parameters
    min_turn_radius: float = 1.0
    jet_cycle_time: float = 1.0
    max_velocity: float = 2.0
    
    # Planning parameters
    rrt_max_iterations: int = 1000
    rrt_step_size: float = 1.5
    rrt_goal_bias: float = 0.15
    rrt_rewire_radius: float = 2.5
    
    # Simulation parameters
    time_step: float = 0.1
    max_simulation_time: float = 60.0
    animation_speed: float = 1.0

@dataclass
class SimulationResults:
    """Results from simulation run"""
    algorithm_name: str
    planning_time: float
    path_length: float
    energy_cost: float
    success: bool
    iterations_used: int
    path_smoothness: float
    collision_free: bool
    execution_time: float

class UnderwaterVehicleSimulator:
    """Main simulation environment for underwater vehicle trajectory planning"""
    
    def __init__(self, config: SimulationConfig = None):
        self.config = config or SimulationConfig()
        self.environment = self._create_environment()
        self.results_history: List[SimulationResults] = []
        
        # Initialize planners
        self.dubins_planner = BioInspiredDubinsPlanner(
            min_turn_radius=self.config.min_turn_radius,
            jet_cycle_time=self.config.jet_cycle_time
        )
        
        self.rrt_standard = ImprovedRRTStar(
            environment=self.environment,
            step_size=self.config.rrt_step_size,
            goal_bias=self.config.rrt_goal_bias,
            rewire_radius=self.config.rrt_rewire_radius,
            max_iterations=self.config.rrt_max_iterations
        )
        
        self.rrt_bio = BioInspiredRRTStar(
            environment=self.environment,
            step_size=self.config.rrt_step_size,
            goal_bias=self.config.rrt_goal_bias,
            rewire_radius=self.config.rrt_rewire_radius,
            max_iterations=self.config.rrt_max_iterations,
            jet_cycle_time=self.config.jet_cycle_time,
            energy_weight=0.4
        )
    
    def _create_environment(self) -> UnderwaterEnvironment:
        """Create underwater environment with random obstacles"""
        obstacles = []
        
        for i in range(self.config.num_obstacles):
            # Random position within bounds
            center = np.array([
                np.random.uniform(bounds[0] + 2, bounds[1] - 2)
                for bounds in self.config.environment_bounds
            ])
            
            # Random radius
            radius = np.random.uniform(*self.config.obstacle_radius_range)
            
            # Random velocity for moving obstacles
            velocity = None
            if np.random.random() < self.config.moving_obstacle_ratio:
                velocity = np.random.uniform(
                    -self.config.max_obstacle_velocity,
                    self.config.max_obstacle_velocity,
                    3
                )
                velocity[2] *= 0.3  # Limit vertical movement
            
            obstacles.append(Obstacle(center, radius, velocity))
        
        return UnderwaterEnvironment(self.config.environment_bounds, obstacles)
    
    def run_comparative_study(self, start_pos: np.ndarray, goal_pos: np.ndarray,
                            num_runs: int = 10) -> Dict[str, List[SimulationResults]]:
        """Run comparative study of different algorithms"""
        
        results = {
            'Dubins_Traditional': [],
            'Dubins_BioInspired': [],
            'RRT_Standard': [],
            'RRT_BioInspired': []
        }
        
        print(f"Running comparative study with {num_runs} runs...")
        print(f"Start: {start_pos}, Goal: {goal_pos}")
        print("=" * 60)
        
        for run in range(num_runs):
            print(f"Run {run + 1}/{num_runs}")
            
            # Recreate environment for each run to ensure fairness
            self.environment = self._create_environment()
            self.rrt_standard.environment = self.environment
            self.rrt_bio.environment = self.environment
            
            # Test traditional Dubins curves
            result = self._test_dubins_traditional(start_pos, goal_pos)
            results['Dubins_Traditional'].append(result)
            
            # Test bio-inspired Dubins curves
            result = self._test_dubins_bio_inspired(start_pos, goal_pos)
            results['Dubins_BioInspired'].append(result)
            
            # Test standard RRT*
            result = self._test_rrt_standard(start_pos, goal_pos)
            results['RRT_Standard'].append(result)
            
            # Test bio-inspired RRT*
            result = self._test_rrt_bio_inspired(start_pos, goal_pos)
            results['RRT_BioInspired'].append(result)
        
        self._print_comparative_results(results)
        return results
    
    def _test_dubins_traditional(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> SimulationResults:
        """Test traditional Dubins curve planning"""
        start_config = Configuration3D(
            position=start_pos,
            orientation=np.zeros(3),
            velocity_direction=np.array([1, 0, 0])
        )
        
        goal_config = Configuration3D(
            position=goal_pos,
            orientation=np.zeros(3),
            velocity_direction=np.array([1, 0, 0])
        )
        
        start_time = time.time()
        path = self.dubins_planner.plan_csc_curve(start_config, goal_config)
        planning_time = time.time() - start_time
        
        # Check collision
        collision_free = self._check_path_collision_free(path.path_points)
        
        # Calculate metrics
        smoothness = self._calculate_path_smoothness(path.path_points)
        energy_cost = path.path_length  # For traditional, energy = distance
        
        return SimulationResults(
            algorithm_name="Dubins_Traditional",
            planning_time=planning_time,
            path_length=path.path_length,
            energy_cost=energy_cost,
            success=collision_free,
            iterations_used=1,  # Direct computation
            path_smoothness=smoothness,
            collision_free=collision_free,
            execution_time=planning_time
        )
    
    def _test_dubins_bio_inspired(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> SimulationResults:
        """Test bio-inspired Dubins curve planning"""
        start_config = Configuration3D(
            position=start_pos,
            orientation=np.zeros(3),
            velocity_direction=np.array([1, 0, 0])
        )
        
        goal_config = Configuration3D(
            position=goal_pos,
            orientation=np.zeros(3),
            velocity_direction=np.array([1, 0, 0])
        )
        
        start_time = time.time()
        path = self.dubins_planner.plan_jet_propulsion_path(start_config, goal_config, max_jets=8)
        planning_time = time.time() - start_time
        
        # Check collision
        collision_free = self._check_path_collision_free(path.path_points)
        
        # Calculate metrics
        smoothness = self._calculate_path_smoothness(path.path_points)
        geometric_length = self._calculate_geometric_length(path.path_points)
        
        return SimulationResults(
            algorithm_name="Dubins_BioInspired",
            planning_time=planning_time,
            path_length=geometric_length,
            energy_cost=path.path_length,  # Includes energy cost
            success=collision_free,
            iterations_used=1,  # Direct computation
            path_smoothness=smoothness,
            collision_free=collision_free,
            execution_time=planning_time
        )
    
    def _test_rrt_standard(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> SimulationResults:
        """Test standard RRT* planning"""
        start_time = time.time()
        path = self.rrt_standard.plan(start_pos, goal_pos)
        planning_time = time.time() - start_time
        
        if path is None:
            return SimulationResults(
                algorithm_name="RRT_Standard",
                planning_time=planning_time,
                path_length=0,
                energy_cost=0,
                success=False,
                iterations_used=self.rrt_standard.iterations_used,
                path_smoothness=0,
                collision_free=False,
                execution_time=planning_time
            )
        
        # Extract path points
        path_points = np.array([node.position for node in path])
        
        # Calculate metrics
        path_length = self._calculate_geometric_length(path_points)
        smoothness = self._calculate_path_smoothness(path_points)
        energy_cost = path_length  # For standard RRT*, energy = distance
        
        return SimulationResults(
            algorithm_name="RRT_Standard",
            planning_time=planning_time,
            path_length=path_length,
            energy_cost=energy_cost,
            success=True,
            iterations_used=self.rrt_standard.iterations_used,
            path_smoothness=smoothness,
            collision_free=True,  # RRT* ensures collision-free paths
            execution_time=planning_time
        )
    
    def _test_rrt_bio_inspired(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> SimulationResults:
        """Test bio-inspired RRT* planning"""
        start_time = time.time()
        result = self.rrt_bio.plan_with_jet_timing(start_pos, goal_pos)
        planning_time = time.time() - start_time
        
        if result is None:
            return SimulationResults(
                algorithm_name="RRT_BioInspired",
                planning_time=planning_time,
                path_length=0,
                energy_cost=0,
                success=False,
                iterations_used=self.rrt_bio.iterations_used,
                path_smoothness=0,
                collision_free=False,
                execution_time=planning_time
            )
        
        path, jet_times = result
        
        # Extract path points
        path_points = np.array([node.position for node in path])
        
        # Calculate metrics
        path_length = self._calculate_geometric_length(path_points)
        smoothness = self._calculate_path_smoothness(path_points)
        energy_cost = path[-1].cost  # Bio-inspired cost includes energy
        
        return SimulationResults(
            algorithm_name="RRT_BioInspired",
            planning_time=planning_time,
            path_length=path_length,
            energy_cost=energy_cost,
            success=True,
            iterations_used=self.rrt_bio.iterations_used,
            path_smoothness=smoothness,
            collision_free=True,  # RRT* ensures collision-free paths
            execution_time=planning_time
        )
    
    def _check_path_collision_free(self, path_points: np.ndarray) -> bool:
        """Check if path is collision-free"""
        for i in range(len(path_points) - 1):
            if not self.environment.is_path_collision_free(path_points[i], path_points[i+1]):
                return False
        return True
    
    def _calculate_geometric_length(self, path_points: np.ndarray) -> float:
        """Calculate geometric path length"""
        if len(path_points) < 2:
            return 0.0
        
        distances = np.linalg.norm(np.diff(path_points, axis=0), axis=1)
        return np.sum(distances)
    
    def _calculate_path_smoothness(self, path_points: np.ndarray) -> float:
        """Calculate path smoothness (inverse of total curvature)"""
        if len(path_points) < 3:
            return 1.0
        
        # Calculate curvature at each point
        curvatures = []
        for i in range(1, len(path_points) - 1):
            p1, p2, p3 = path_points[i-1], path_points[i], path_points[i+1]
            
            # Vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Avoid division by zero
            if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
                continue
            
            # Angle between vectors
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_angle = np.clip(cos_angle, -1, 1)
            angle = np.arccos(cos_angle)
            
            # Curvature approximation
            curvature = angle / (np.linalg.norm(v1) + np.linalg.norm(v2))
            curvatures.append(curvature)
        
        if not curvatures:
            return 1.0
        
        avg_curvature = np.mean(curvatures)
        return 1.0 / (1.0 + avg_curvature)  # Higher smoothness for lower curvature
    
    def _print_comparative_results(self, results: Dict[str, List[SimulationResults]]):
        """Print comparative analysis of results"""
        print("\n" + "=" * 80)
        print("COMPARATIVE ANALYSIS RESULTS")
        print("=" * 80)
        
        for algorithm, result_list in results.items():
            if not result_list:
                continue
            
            successful_runs = [r for r in result_list if r.success]
            success_rate = len(successful_runs) / len(result_list) * 100
            
            if successful_runs:
                avg_planning_time = np.mean([r.planning_time for r in successful_runs])
                avg_path_length = np.mean([r.path_length for r in successful_runs])
                avg_energy_cost = np.mean([r.energy_cost for r in successful_runs])
                avg_smoothness = np.mean([r.path_smoothness for r in successful_runs])
                avg_iterations = np.mean([r.iterations_used for r in successful_runs])
                
                print(f"\n{algorithm}:")
                print(f"  Success Rate: {success_rate:.1f}%")
                print(f"  Avg Planning Time: {avg_planning_time:.3f}s")
                print(f"  Avg Path Length: {avg_path_length:.2f}")
                print(f"  Avg Energy Cost: {avg_energy_cost:.2f}")
                print(f"  Avg Smoothness: {avg_smoothness:.3f}")
                print(f"  Avg Iterations: {avg_iterations:.0f}")
            else:
                print(f"\n{algorithm}:")
                print(f"  Success Rate: {success_rate:.1f}%")
                print("  No successful runs to analyze")
    
    def visualize_comparison(self, start_pos: np.ndarray, goal_pos: np.ndarray,
                           save_path: str = None):
        """Create comprehensive visualization of algorithm comparison"""
        
        # Plan paths with all algorithms
        print("Planning paths for visualization...")
        
        # Traditional Dubins
        start_config = Configuration3D(start_pos, np.zeros(3), np.array([1, 0, 0]))
        goal_config = Configuration3D(goal_pos, np.zeros(3), np.array([1, 0, 0]))
        dubins_trad = self.dubins_planner.plan_csc_curve(start_config, goal_config)
        dubins_bio = self.dubins_planner.plan_jet_propulsion_path(start_config, goal_config, max_jets=6)
        
        # RRT* variants
        rrt_standard_path = self.rrt_standard.plan(start_pos, goal_pos)
        rrt_bio_result = self.rrt_bio.plan_with_jet_timing(start_pos, goal_pos)
        
        # Create visualization
        fig = plt.figure(figsize=(20, 15))
        
        # Environment overview
        ax1 = fig.add_subplot(231, projection='3d')
        self._plot_environment(ax1)
        ax1.scatter(*start_pos, color='green', s=200, label='Start', marker='o')
        ax1.scatter(*goal_pos, color='red', s=200, label='Goal', marker='s')
        ax1.set_title('Environment Overview')
        ax1.legend()
        
        # Traditional Dubins
        ax2 = fig.add_subplot(232, projection='3d')
        self._plot_environment(ax2)
        ax2.plot(dubins_trad.path_points[:, 0], dubins_trad.path_points[:, 1], 
                dubins_trad.path_points[:, 2], 'b-', linewidth=3, label='Traditional Dubins')
        ax2.scatter(*start_pos, color='green', s=100, marker='o')
        ax2.scatter(*goal_pos, color='red', s=100, marker='s')
        ax2.set_title(f'Traditional Dubins\nLength: {dubins_trad.path_length:.2f}')
        ax2.legend()
        
        # Bio-inspired Dubins
        ax3 = fig.add_subplot(233, projection='3d')
        self._plot_environment(ax3)
        ax3.plot(dubins_bio.path_points[:, 0], dubins_bio.path_points[:, 1], 
                dubins_bio.path_points[:, 2], 'g-', linewidth=3, label='Bio-Inspired Dubins')
        ax3.scatter(dubins_bio.path_points[:, 0], dubins_bio.path_points[:, 1], 
                   dubins_bio.path_points[:, 2], color='orange', s=30, label='Jet Points')
        ax3.scatter(*start_pos, color='green', s=100, marker='o')
        ax3.scatter(*goal_pos, color='red', s=100, marker='s')
        ax3.set_title(f'Bio-Inspired Dubins\nEnergy Cost: {dubins_bio.path_length:.2f}')
        ax3.legend()
        
        # Standard RRT*
        ax4 = fig.add_subplot(234, projection='3d')
        self._plot_environment(ax4)
        if rrt_standard_path:
            path_points = np.array([node.position for node in rrt_standard_path])
            ax4.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
                    'purple', linewidth=3, label='Standard RRT*')
            path_length = self._calculate_geometric_length(path_points)
            ax4.set_title(f'Standard RRT*\nLength: {path_length:.2f}')
        else:
            ax4.set_title('Standard RRT*\nNo path found')
        ax4.scatter(*start_pos, color='green', s=100, marker='o')
        ax4.scatter(*goal_pos, color='red', s=100, marker='s')
        ax4.legend()
        
        # Bio-inspired RRT*
        ax5 = fig.add_subplot(235, projection='3d')
        self._plot_environment(ax5)
        if rrt_bio_result:
            bio_path, jet_times = rrt_bio_result
            path_points = np.array([node.position for node in bio_path])
            ax5.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 
                    'orange', linewidth=3, label='Bio-Inspired RRT*')
            # Mark jet points
            jet_indices = np.linspace(0, len(path_points)-1, len(jet_times), dtype=int)
            ax5.scatter(path_points[jet_indices, 0], path_points[jet_indices, 1], 
                       path_points[jet_indices, 2], color='red', s=30, label='Jet Events')
            ax5.set_title(f'Bio-Inspired RRT*\nEnergy Cost: {bio_path[-1].cost:.2f}')
        else:
            ax5.set_title('Bio-Inspired RRT*\nNo path found')
        ax5.scatter(*start_pos, color='green', s=100, marker='o')
        ax5.scatter(*goal_pos, color='red', s=100, marker='s')
        ax5.legend()
        
        # Performance comparison
        ax6 = fig.add_subplot(236)
        algorithms = ['Trad. Dubins', 'Bio Dubins', 'Std RRT*', 'Bio RRT*']
        
        # Collect metrics
        lengths = [
            dubins_trad.path_length,
            self._calculate_geometric_length(dubins_bio.path_points),
            self._calculate_geometric_length(np.array([node.position for node in rrt_standard_path])) if rrt_standard_path else 0,
            self._calculate_geometric_length(np.array([node.position for node in rrt_bio_result[0]])) if rrt_bio_result else 0
        ]
        
        energy_costs = [
            dubins_trad.path_length,
            dubins_bio.path_length,
            lengths[2],  # Standard RRT* energy = distance
            rrt_bio_result[0][-1].cost if rrt_bio_result else 0
        ]
        
        x = np.arange(len(algorithms))
        width = 0.35
        
        ax6.bar(x - width/2, lengths, width, label='Path Length', alpha=0.7)
        ax6.bar(x + width/2, energy_costs, width, label='Energy Cost', alpha=0.7)
        
        ax6.set_xlabel('Algorithm')
        ax6.set_ylabel('Cost')
        ax6.set_title('Performance Comparison')
        ax6.set_xticks(x)
        ax6.set_xticklabels(algorithms, rotation=45)
        ax6.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Visualization saved to {save_path}")
        
        plt.show()
    
    def _plot_environment(self, ax):
        """Plot environment obstacles"""
        for obs in self.environment.obstacles:
            # Create sphere
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = obs.center[0] + obs.radius * np.outer(np.cos(u), np.sin(v))
            y = obs.center[1] + obs.radius * np.outer(np.sin(u), np.sin(v))
            z = obs.center[2] + obs.radius * np.outer(np.ones(np.size(u)), np.cos(v))
            
            color = 'red' if obs.velocity is None else 'darkred'
            ax.plot_surface(x, y, z, alpha=0.3, color=color)
        
        # Set axis properties
        bounds = self.config.environment_bounds
        ax.set_xlim(bounds[0])
        ax.set_ylim(bounds[1])
        ax.set_zlim(bounds[2])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

def main():
    """Main simulation demonstration"""
    print("Underwater Vehicle Trajectory Planning Simulation")
    print("=" * 60)
    
    # Create simulation
    config = SimulationConfig(
        num_obstacles=6,
        rrt_max_iterations=800,
        moving_obstacle_ratio=0.2
    )
    
    simulator = UnderwaterVehicleSimulator(config)
    
    # Define test scenario
    start_pos = np.array([0, 0, -2])
    goal_pos = np.array([15, 15, -12])
    
    print(f"Test scenario:")
    print(f"  Start: {start_pos}")
    print(f"  Goal: {goal_pos}")
    print(f"  Environment: {config.num_obstacles} obstacles")
    print()
    
    # Run single comparison
    print("Running single algorithm comparison...")
    simulator.visualize_comparison(
        start_pos, goal_pos, 
        save_path='UnderwaterVehicleTrajectories/Simulations/algorithm_comparison.png'
    )
    
    # Run statistical study
    print("\nRunning statistical comparison study...")
    results = simulator.run_comparative_study(start_pos, goal_pos, num_runs=5)
    
    # Save results
    results_file = 'UnderwaterVehicleTrajectories/Simulations/simulation_results.json'
    with open(results_file, 'w') as f:
        # Convert results to serializable format
        serializable_results = {}
        for alg, result_list in results.items():
            serializable_results[alg] = [asdict(r) for r in result_list]
        json.dump(serializable_results, f, indent=2)
    
    print(f"\nResults saved to {results_file}")
    print("Simulation completed successfully!")

if __name__ == "__main__":
    main()
