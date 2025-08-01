"""
3D Dubins Curves for Underwater Vehicle Trajectory Planning

Implementation based on Pan et al. (2025) research with adaptations for bio-inspired vehicles.
Includes neural network acceleration for fast curve length estimation.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, List, Optional
import tensorflow as tf
from dataclasses import dataclass

@dataclass
class Configuration3D:
    """3D configuration with position and orientation"""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # [roll, pitch, yaw]
    velocity_direction: np.ndarray  # unit vector

@dataclass
class DubinsPath3D:
    """3D Dubins path representation"""
    start_config: Configuration3D
    end_config: Configuration3D
    path_points: np.ndarray
    path_length: float
    curve_type: str  # 'CSC', 'CCC', etc.
    
class NeuralDubinsEstimator:
    """Neural network for fast 3D Dubins curve length estimation"""
    
    def __init__(self):
        self.model = self._build_model()
        self.is_trained = False
    
    def _build_model(self) -> tf.keras.Model:
        """Build BPNN architecture from Pan et al. (2025)"""
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(30, activation='tanh', input_shape=(6,)),
            tf.keras.layers.Dense(22, activation='tanh'),
            tf.keras.layers.Dense(1, activation='linear')
        ])
        
        model.compile(
            optimizer='adam',
            loss='mse',
            metrics=['mae']
        )
        
        return model
    
    def create_geometric_descriptor(self, config1: Configuration3D, 
                                  config2: Configuration3D, 
                                  r1: float, r2: float) -> np.ndarray:
        """Create geometric descriptor vector Λ_T = [A, B1, B2, B12, r1, r2]"""
        delta_p = config2.position - config1.position
        
        A = np.linalg.norm(delta_p) ** 2
        B1 = np.dot(config1.velocity_direction, delta_p)
        B2 = np.dot(config2.velocity_direction, delta_p)
        B12 = np.dot(config1.velocity_direction, config2.velocity_direction)
        
        return np.array([A, B1, B2, B12, r1, r2])
    
    def estimate_length(self, config1: Configuration3D, config2: Configuration3D,
                       r1: float = 1.0, r2: float = 1.0) -> float:
        """Fast length estimation using neural network"""
        if not self.is_trained:
            print("Warning: Model not trained. Using analytical computation.")
            return self._analytical_length(config1, config2, r1, r2)
        
        descriptor = self.create_geometric_descriptor(config1, config2, r1, r2)
        length = self.model.predict(descriptor.reshape(1, -1), verbose=0)[0, 0]
        return max(0, length)  # Ensure non-negative
    
    def _analytical_length(self, config1: Configuration3D, config2: Configuration3D,
                          r1: float, r2: float) -> float:
        """Analytical computation for comparison/fallback"""
        # Simplified analytical computation
        delta_p = config2.position - config1.position
        straight_distance = np.linalg.norm(delta_p)
        
        # Rough approximation including turning costs
        turn_cost1 = r1 * np.pi / 2  # Quarter circle approximation
        turn_cost2 = r2 * np.pi / 2
        
        return straight_distance + turn_cost1 + turn_cost2
    
    def train_model(self, training_data: List[Tuple], epochs: int = 100):
        """Train the neural network on generated 3D Dubins curves"""
        X = []
        y = []
        
        for config1, config2, r1, r2, true_length in training_data:
            descriptor = self.create_geometric_descriptor(config1, config2, r1, r2)
            X.append(descriptor)
            y.append(true_length)
        
        X = np.array(X)
        y = np.array(y)
        
        history = self.model.fit(X, y, epochs=epochs, validation_split=0.1, verbose=1)
        self.is_trained = True
        
        return history

class Dubins3DPlanner:
    """3D Dubins curve planner for underwater vehicles"""
    
    def __init__(self, min_turn_radius: float = 1.0):
        self.min_turn_radius = min_turn_radius
        self.neural_estimator = NeuralDubinsEstimator()
    
    def plan_csc_curve(self, start: Configuration3D, end: Configuration3D) -> DubinsPath3D:
        """Plan CSC (Curve-Straight-Curve) Dubins path"""
        
        # Extract positions and directions
        p1, p2 = start.position, end.position
        v1, v2 = start.velocity_direction, end.velocity_direction
        
        # Calculate geometric parameters
        delta_p = p2 - p1
        
        # Simplified CSC construction (full implementation would solve nonlinear system)
        # This is a basic approximation for demonstration
        
        # Calculate intermediate points for CSC curve
        path_points = self._construct_csc_path(p1, p2, v1, v2)
        
        # Calculate path length
        path_length = self._calculate_path_length(path_points)
        
        return DubinsPath3D(
            start_config=start,
            end_config=end,
            path_points=path_points,
            path_length=path_length,
            curve_type='CSC'
        )
    
    def _construct_csc_path(self, p1: np.ndarray, p2: np.ndarray, 
                           v1: np.ndarray, v2: np.ndarray, 
                           num_points: int = 100) -> np.ndarray:
        """Construct CSC path points (simplified implementation)"""
        
        # For demonstration, create a smooth path with circular arcs and straight segment
        points = []
        
        # First curve (simplified as quarter circle)
        r = self.min_turn_radius
        center1 = p1 + r * self._perpendicular_vector(v1)
        
        # Generate first arc points
        for i in range(num_points // 3):
            angle = i * np.pi / (2 * (num_points // 3))
            point = center1 + r * (np.cos(angle) * (-v1) + np.sin(angle) * self._perpendicular_vector(v1))
            points.append(point)
        
        # Straight segment
        start_straight = points[-1]
        direction = (p2 - start_straight)
        direction = direction / np.linalg.norm(direction)
        
        straight_length = np.linalg.norm(p2 - start_straight) * 0.6  # Approximate
        
        for i in range(num_points // 3):
            t = i / (num_points // 3)
            point = start_straight + t * straight_length * direction
            points.append(point)
        
        # Second curve
        for i in range(num_points // 3):
            angle = i * np.pi / (2 * (num_points // 3))
            # Simplified second arc
            t = i / (num_points // 3)
            point = points[-1] + t * (p2 - points[-1])
            points.append(point)
        
        return np.array(points)
    
    def _perpendicular_vector(self, v: np.ndarray) -> np.ndarray:
        """Generate a perpendicular vector in 3D"""
        if abs(v[2]) < 0.9:
            perp = np.cross(v, np.array([0, 0, 1]))
        else:
            perp = np.cross(v, np.array([1, 0, 0]))
        
        return perp / np.linalg.norm(perp)
    
    def _calculate_path_length(self, path_points: np.ndarray) -> float:
        """Calculate total path length"""
        if len(path_points) < 2:
            return 0.0
        
        distances = np.linalg.norm(np.diff(path_points, axis=0), axis=1)
        return np.sum(distances)
    
    def plan_optimal_path(self, start: Configuration3D, end: Configuration3D) -> DubinsPath3D:
        """Plan optimal 3D Dubins path (currently implements CSC only)"""
        return self.plan_csc_curve(start, end)

class BioInspiredDubinsPlanner(Dubins3DPlanner):
    """Extended Dubins planner for bio-inspired vehicles like SALP robots"""
    
    def __init__(self, min_turn_radius: float = 1.0, jet_cycle_time: float = 1.0):
        super().__init__(min_turn_radius)
        self.jet_cycle_time = jet_cycle_time
        
        # Enhanced SALP/Jellyfish dynamics parameters
        self.body_length = 0.15  # meters
        self.max_volume_ratio = 2.5  # expansion ratio
        self.jet_efficiency = 0.65  # propulsion efficiency
        self.recovery_time = 0.3  # time between jet cycles
        self.contraction_speed = 0.8  # how fast body contracts (0-1)
        
    def salp_volume_profile(self, t: float, phase_offset: float = 0) -> float:
        """Generate realistic SALP body volume over time"""
        cycle_phase = (t / self.jet_cycle_time + phase_offset) % 1.0
        
        if cycle_phase < 0.4:  # Slow expansion phase
            expansion_factor = np.sin(cycle_phase * np.pi / 0.4) ** 0.5
            volume_factor = 0.5 + 0.5 * expansion_factor
        elif cycle_phase < 0.6:  # Rapid contraction (jet phase)
            contraction_progress = (cycle_phase - 0.4) / 0.2
            volume_factor = 0.5 + 0.5 * (1 - contraction_progress ** 2)
        else:  # Recovery phase
            recovery_progress = (cycle_phase - 0.6) / 0.4
            volume_factor = 0.5 + 0.1 * np.sin(recovery_progress * np.pi)
            
        return volume_factor
    
    def calculate_thrust_force(self, volume_rate: float, current_volume: float) -> float:
        """Calculate thrust force based on volume change rate (realistic SALP model)"""
        if volume_rate < -0.01:  # Only during significant contraction
            # Thrust proportional to volume change rate and current volume
            base_thrust = abs(volume_rate) * current_volume * 50  # N per m³/s
            efficiency_factor = self.jet_efficiency * (1 + 0.3 * current_volume)
            return base_thrust * efficiency_factor
        return 0
    
    def jellyfish_swimming_pattern(self, path_points: np.ndarray, num_pulses: int = 8) -> np.ndarray:
        """Apply jellyfish-like pulsing motion to trajectory"""
        enhanced_path = []
        
        for i in range(len(path_points) - 1):
            start_point = path_points[i]
            end_point = path_points[i + 1]
            
            # Create pulsing motion between points
            segment_points = []
            for pulse in range(num_pulses):
                t = pulse / (num_pulses - 1) if num_pulses > 1 else 0
                
                # Base interpolation
                base_point = start_point + t * (end_point - start_point)
                
                # Add pulsing motion (perpendicular to direction)
                direction = end_point - start_point
                if np.linalg.norm(direction) > 0:
                    direction = direction / np.linalg.norm(direction)
                    
                    # Create perpendicular vectors for pulsing
                    if abs(direction[2]) < 0.9:
                        perp1 = np.cross(direction, np.array([0, 0, 1]))
                    else:
                        perp1 = np.cross(direction, np.array([1, 0, 0]))
                    perp1 = perp1 / np.linalg.norm(perp1)
                    perp2 = np.cross(direction, perp1)
                    
                    # Pulsing amplitude based on swimming cycle
                    pulse_phase = pulse * 2 * np.pi / num_pulses
                    amplitude = 0.1 * self.body_length * np.sin(pulse_phase)
                    
                    # Add pulsing motion
                    pulse_offset = amplitude * (perp1 * np.cos(pulse_phase) + perp2 * np.sin(pulse_phase))
                    pulsed_point = base_point + pulse_offset
                    
                    segment_points.append(pulsed_point)
                else:
                    segment_points.append(base_point)
            
            enhanced_path.extend(segment_points)
        
        return np.array(enhanced_path)
    
    def plan_jet_propulsion_path(self, start: Configuration3D, end: Configuration3D,
                                max_jets: int = 10) -> DubinsPath3D:
        """Plan path considering discrete jet propulsion events"""
        
        # Get basic Dubins path
        base_path = self.plan_csc_curve(start, end)
        
        # Modify for jet propulsion by inserting jet timing points
        jet_points = self._insert_jet_points(base_path.path_points, max_jets)
        
        # Recalculate length considering jet efficiency
        energy_cost = self._calculate_energy_cost(jet_points, max_jets)
        
        return DubinsPath3D(
            start_config=start,
            end_config=end,
            path_points=jet_points,
            path_length=energy_cost,  # Use energy cost instead of geometric length
            curve_type='JET-CSC'
        )
    
    def _insert_jet_points(self, path_points: np.ndarray, max_jets: int) -> np.ndarray:
        """Insert jet propulsion points along the path"""
        if len(path_points) <= max_jets:
            return path_points
        
        # Select evenly spaced points for jet events
        indices = np.linspace(0, len(path_points) - 1, max_jets, dtype=int)
        return path_points[indices]
    
    def _calculate_energy_cost(self, jet_points: np.ndarray, num_jets: int) -> float:
        """Calculate energy cost considering jet propulsion efficiency"""
        if len(jet_points) < 2:
            return 0.0
        
        # Base distance cost
        distances = np.linalg.norm(np.diff(jet_points, axis=0), axis=1)
        distance_cost = np.sum(distances)
        
        # Jet firing cost (energy per jet event)
        jet_cost = num_jets * 0.5  # Simplified energy model
        
        # Total cost of transport approximation
        return distance_cost + jet_cost

def generate_training_data(num_samples: int = 1000) -> List[Tuple]:
    """Generate training data for neural network"""
    training_data = []
    planner = Dubins3DPlanner()
    
    for _ in range(num_samples):
        # Random start and end configurations
        start_pos = np.random.uniform(-10, 10, 3)
        end_pos = np.random.uniform(-10, 10, 3)
        
        start_dir = np.random.uniform(-1, 1, 3)
        start_dir = start_dir / np.linalg.norm(start_dir)
        
        end_dir = np.random.uniform(-1, 1, 3)
        end_dir = end_dir / np.linalg.norm(end_dir)
        
        start_config = Configuration3D(start_pos, np.zeros(3), start_dir)
        end_config = Configuration3D(end_pos, np.zeros(3), end_dir)
        
        # Random turning radii
        r1 = np.random.uniform(0.5, 3.0)
        r2 = np.random.uniform(0.5, 3.0)
        
        # Calculate true length (using analytical approximation)
        true_length = planner.neural_estimator._analytical_length(start_config, end_config, r1, r2)
        
        training_data.append((start_config, end_config, r1, r2, true_length))
    
    return training_data

def demo_3d_dubins():
    """Demonstration of 3D Dubins curve planning"""
    
    # Create planner
    planner = BioInspiredDubinsPlanner(min_turn_radius=1.0)
    
    # Define start and end configurations
    start = Configuration3D(
        position=np.array([0, 0, 0]),
        orientation=np.array([0, 0, 0]),
        velocity_direction=np.array([1, 0, 0])
    )
    
    end = Configuration3D(
        position=np.array([10, 5, -2]),
        orientation=np.array([0, 0, np.pi/4]),
        velocity_direction=np.array([0.707, 0.707, 0])
    )
    
    # Plan traditional Dubins path
    dubins_path = planner.plan_csc_curve(start, end)
    print(f"Traditional Dubins path length: {dubins_path.path_length:.2f}")
    
    # Plan bio-inspired jet propulsion path
    jet_path = planner.plan_jet_propulsion_path(start, end, max_jets=5)
    print(f"Jet propulsion path cost: {jet_path.path_length:.2f}")
    
    # Visualize paths
    fig = plt.figure(figsize=(12, 5))
    
    # Traditional path
    ax1 = fig.add_subplot(121, projection='3d')
    path_points = dubins_path.path_points
    ax1.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], 'b-', linewidth=2, label='Dubins Path')
    ax1.scatter(*start.position, color='green', s=100, label='Start')
    ax1.scatter(*end.position, color='red', s=100, label='End')
    ax1.set_title('Traditional 3D Dubins Path')
    ax1.legend()
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Jet propulsion path
    ax2 = fig.add_subplot(122, projection='3d')
    jet_points = jet_path.path_points
    ax2.plot(jet_points[:, 0], jet_points[:, 1], jet_points[:, 2], 'r-', linewidth=2, label='Jet Path')
    ax2.scatter(jet_points[:, 0], jet_points[:, 1], jet_points[:, 2], color='orange', s=50, label='Jet Points')
    ax2.scatter(*start.position, color='green', s=100, label='Start')
    ax2.scatter(*end.position, color='red', s=100, label='End')
    ax2.set_title('Bio-Inspired Jet Propulsion Path')
    ax2.legend()
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    
    plt.tight_layout()
    plt.savefig('UnderwaterVehicleTrajectories/Simulations/dubins_3d_demo.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return dubins_path, jet_path

if __name__ == "__main__":
    # Run demonstration
    print("3D Dubins Curves for Underwater Vehicles Demo")
    print("=" * 50)
    
    dubins_path, jet_path = demo_3d_dubins()
    
    print("\nDemo completed. Check Simulations/ folder for visualization.")
