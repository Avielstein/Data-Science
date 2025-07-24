"""
Improved RRT* for Underwater Vehicle Path Planning

Implementation based on Pan et al. (2025) and Hao et al. (2020) research.
Includes bio-inspired adaptations for jet-propelled vehicles like SALP robots.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Optional, Dict, Set
from dataclasses import dataclass, field
from enum import Enum
import random
import time
from collections import deque

class NodeState(Enum):
    """Node states for improved RRT* algorithm"""
    NEW = "new"
    OPEN = "open"
    CLOSE = "close"

@dataclass
class RRTNode:
    """RRT* node with enhanced properties for underwater vehicles"""
    position: np.ndarray
    orientation: float  # Heading angle
    velocity_direction: np.ndarray
    parent: Optional['RRTNode'] = None
    children: List['RRTNode'] = field(default_factory=list)
    cost: float = float('inf')
    state: NodeState = NodeState.NEW
    collision_info: Optional[Dict] = None
    jet_timing: Optional[float] = None  # For bio-inspired vehicles
    
    def __post_init__(self):
        if self.velocity_direction is not None:
            # Ensure velocity direction is normalized
            self.velocity_direction = self.velocity_direction / np.linalg.norm(self.velocity_direction)

@dataclass
class Obstacle:
    """3D spherical obstacle representation"""
    center: np.ndarray
    radius: float
    velocity: Optional[np.ndarray] = None  # For moving obstacles
    
    def is_collision(self, point: np.ndarray, safety_margin: float = 0.1) -> bool:
        """Check collision with point including safety margin"""
        distance = np.linalg.norm(point - self.center)
        return distance <= (self.radius + safety_margin)
    
    def update_position(self, dt: float):
        """Update obstacle position for moving obstacles"""
        if self.velocity is not None:
            self.center += self.velocity * dt

class UnderwaterEnvironment:
    """3D underwater environment with obstacles and boundaries"""
    
    def __init__(self, bounds: Tuple[Tuple[float, float], ...], obstacles: List[Obstacle] = None):
        self.bounds = bounds  # ((x_min, x_max), (y_min, y_max), (z_min, z_max))
        self.obstacles = obstacles or []
        self.current_time = 0.0
    
    def is_valid_position(self, position: np.ndarray, safety_margin: float = 0.1) -> bool:
        """Check if position is valid (within bounds and collision-free)"""
        # Check bounds
        for i, (min_val, max_val) in enumerate(self.bounds):
            if position[i] < min_val or position[i] > max_val:
                return False
        
        # Check obstacles
        for obstacle in self.obstacles:
            if obstacle.is_collision(position, safety_margin):
                return False
        
        return True
    
    def is_path_collision_free(self, start: np.ndarray, end: np.ndarray, 
                              num_checks: int = 10) -> bool:
        """Check if straight line path is collision-free"""
        for i in range(num_checks + 1):
            t = i / num_checks
            point = start + t * (end - start)
            if not self.is_valid_position(point):
                return False
        return True
    
    def update(self, dt: float):
        """Update environment (move obstacles, etc.)"""
        self.current_time += dt
        for obstacle in self.obstacles:
            obstacle.update_position(dt)
    
    def sample_free_space(self) -> np.ndarray:
        """Sample a random point in free space"""
        max_attempts = 100
        for _ in range(max_attempts):
            point = np.array([
                np.random.uniform(bounds[0], bounds[1]) 
                for bounds in self.bounds
            ])
            if self.is_valid_position(point):
                return point
        
        # Fallback: return center of environment
        return np.array([(bounds[0] + bounds[1]) / 2 for bounds in self.bounds])

class ImprovedRRTStar:
    """Improved RRT* algorithm for underwater vehicles"""
    
    def __init__(self, environment: UnderwaterEnvironment, 
                 step_size: float = 1.0,
                 goal_bias: float = 0.1,
                 rewire_radius: float = 2.0,
                 max_iterations: int = 1000):
        
        self.environment = environment
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.rewire_radius = rewire_radius
        self.max_iterations = max_iterations
        
        # Node sets for improved algorithm
        self.nodes: List[RRTNode] = []
        self.V_new: Set[RRTNode] = set()
        self.V_open: Set[RRTNode] = set()
        self.V_close: Set[RRTNode] = set()
        
        # Statistics
        self.planning_time = 0.0
        self.iterations_used = 0
    
    def plan(self, start_pos: np.ndarray, goal_pos: np.ndarray,
             start_heading: float = 0.0, goal_heading: float = 0.0) -> Optional[List[RRTNode]]:
        """Plan path from start to goal using improved RRT*"""
        
        start_time = time.time()
        
        # Initialize with start node
        start_dir = np.array([np.cos(start_heading), np.sin(start_heading), 0])
        start_node = RRTNode(
            position=start_pos.copy(),
            orientation=start_heading,
            velocity_direction=start_dir,
            cost=0.0,
            state=NodeState.OPEN
        )
        
        self.nodes = [start_node]
        self.V_open = {start_node}
        
        goal_dir = np.array([np.cos(goal_heading), np.sin(goal_heading), 0])
        
        for iteration in range(self.max_iterations):
            self.iterations_used = iteration + 1
            
            # Pseudorandom sampling with goal bias
            if random.random() < self.goal_bias:
                sample_pos = goal_pos.copy()
                sample_dir = goal_dir.copy()
            else:
                sample_pos = self._pseudorandom_sampling()
                sample_dir = self._random_direction()
            
            # Find nearest node
            nearest_node = self._find_nearest_node(sample_pos)
            if nearest_node is None:
                continue
            
            # Steer towards sample
            new_pos, new_dir = self._steer(nearest_node, sample_pos, sample_dir)
            
            # Check if new position is valid
            if not self.environment.is_valid_position(new_pos):
                continue
            
            # Check path collision
            if not self.environment.is_path_collision_free(nearest_node.position, new_pos):
                continue
            
            # Create new node
            new_node = RRTNode(
                position=new_pos,
                orientation=np.arctan2(new_dir[1], new_dir[0]),
                velocity_direction=new_dir,
                parent=nearest_node,
                cost=nearest_node.cost + np.linalg.norm(new_pos - nearest_node.position),
                state=NodeState.NEW
            )
            
            # Find nearby nodes for rewiring
            nearby_nodes = self._find_nearby_nodes(new_node)
            
            # Choose best parent
            best_parent = self._choose_best_parent(new_node, nearby_nodes)
            if best_parent is not None:
                new_node.parent = best_parent
                new_node.cost = best_parent.cost + self._calculate_cost(best_parent, new_node)
            
            # Add to tree
            self.nodes.append(new_node)
            if new_node.parent:
                new_node.parent.children.append(new_node)
            
            # Rewire nearby nodes
            self._rewire_nearby_nodes(new_node, nearby_nodes)
            
            # Update node states
            self._update_node_states(new_node, goal_pos)
            
            # Check if goal is reached
            if self._is_goal_reached(new_node, goal_pos):
                path = self._extract_path(new_node)
                self.planning_time = time.time() - start_time
                return path
        
        self.planning_time = time.time() - start_time
        return None  # No path found
    
    def _pseudorandom_sampling(self) -> np.ndarray:
        """Enhanced sampling strategy for obstacle avoidance"""
        # Combine random sampling with obstacle-aware bias
        if len(self.V_new) > 0 and random.random() < 0.3:
            # Sample near nodes that encountered obstacles
            blocked_node = random.choice(list(self.V_new))
            
            # Sample in direction away from nearest obstacle
            nearest_obstacle = self._find_nearest_obstacle(blocked_node.position)
            if nearest_obstacle:
                away_direction = blocked_node.position - nearest_obstacle.center
                away_direction = away_direction / np.linalg.norm(away_direction)
                
                # Sample in cone around away direction
                cone_angle = np.pi / 3  # 60 degrees
                random_angle = np.random.uniform(-cone_angle/2, cone_angle/2)
                
                # Rotate away direction by random angle
                rotation_axis = np.array([0, 0, 1])  # Rotate around z-axis
                cos_angle = np.cos(random_angle)
                sin_angle = np.sin(random_angle)
                
                rotated_dir = (cos_angle * away_direction + 
                             sin_angle * np.cross(rotation_axis, away_direction))
                
                sample_distance = np.random.uniform(1.0, 3.0)
                return blocked_node.position + sample_distance * rotated_dir
        
        # Default random sampling
        return self.environment.sample_free_space()
    
    def _random_direction(self) -> np.ndarray:
        """Generate random unit direction vector"""
        direction = np.random.uniform(-1, 1, 3)
        direction[2] *= 0.3  # Limit vertical component for underwater vehicles
        return direction / np.linalg.norm(direction)
    
    def _find_nearest_node(self, position: np.ndarray) -> Optional[RRTNode]:
        """Find nearest node to given position"""
        if not self.nodes:
            return None
        
        min_distance = float('inf')
        nearest_node = None
        
        for node in self.nodes:
            distance = np.linalg.norm(node.position - position)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
    
    def _steer(self, from_node: RRTNode, to_pos: np.ndarray, 
              to_dir: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Steer from node towards target position and direction"""
        direction = to_pos - from_node.position
        distance = np.linalg.norm(direction)
        
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        
        new_pos = from_node.position + direction
        
        # Interpolate direction
        alpha = min(1.0, self.step_size / distance) if distance > 0 else 1.0
        new_dir = (1 - alpha) * from_node.velocity_direction + alpha * to_dir
        new_dir = new_dir / np.linalg.norm(new_dir)
        
        return new_pos, new_dir
    
    def _find_nearby_nodes(self, node: RRTNode) -> List[RRTNode]:
        """Find nodes within rewiring radius"""
        nearby_nodes = []
        
        for other_node in self.nodes:
            if other_node != node:
                distance = np.linalg.norm(other_node.position - node.position)
                if distance <= self.rewire_radius:
                    nearby_nodes.append(other_node)
        
        return nearby_nodes
    
    def _choose_best_parent(self, node: RRTNode, nearby_nodes: List[RRTNode]) -> Optional[RRTNode]:
        """Choose best parent from nearby nodes"""
        best_parent = node.parent
        best_cost = node.cost
        
        for candidate in nearby_nodes:
            if self.environment.is_path_collision_free(candidate.position, node.position):
                candidate_cost = candidate.cost + self._calculate_cost(candidate, node)
                if candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best_parent = candidate
        
        return best_parent
    
    def _calculate_cost(self, from_node: RRTNode, to_node: RRTNode) -> float:
        """Calculate cost between two nodes"""
        # Basic Euclidean distance
        distance = np.linalg.norm(to_node.position - from_node.position)
        
        # Add orientation change penalty
        orientation_diff = abs(to_node.orientation - from_node.orientation)
        orientation_penalty = min(orientation_diff, 2*np.pi - orientation_diff) * 0.5
        
        return distance + orientation_penalty
    
    def _rewire_nearby_nodes(self, new_node: RRTNode, nearby_nodes: List[RRTNode]):
        """Rewire nearby nodes if better path through new node exists"""
        for node in nearby_nodes:
            if node.parent and self.environment.is_path_collision_free(new_node.position, node.position):
                new_cost = new_node.cost + self._calculate_cost(new_node, node)
                if new_cost < node.cost:
                    # Remove from old parent
                    if node in node.parent.children:
                        node.parent.children.remove(node)
                    
                    # Set new parent
                    node.parent = new_node
                    node.cost = new_cost
                    new_node.children.append(node)
                    
                    # Update costs of descendants
                    self._update_descendant_costs(node)
    
    def _update_descendant_costs(self, node: RRTNode):
        """Update costs of all descendant nodes"""
        for child in node.children:
            child.cost = node.cost + self._calculate_cost(node, child)
            self._update_descendant_costs(child)
    
    def _update_node_states(self, new_node: RRTNode, goal_pos: np.ndarray):
        """Update node states based on goal reachability"""
        # Check if node can reach goal
        if self.environment.is_path_collision_free(new_node.position, goal_pos):
            new_node.state = NodeState.CLOSE
            self.V_close.add(new_node)
        else:
            new_node.state = NodeState.OPEN
            self.V_open.add(new_node)
    
    def _is_goal_reached(self, node: RRTNode, goal_pos: np.ndarray, 
                        tolerance: float = 0.5) -> bool:
        """Check if node is close enough to goal"""
        distance = np.linalg.norm(node.position - goal_pos)
        return distance <= tolerance
    
    def _extract_path(self, goal_node: RRTNode) -> List[RRTNode]:
        """Extract path from start to goal node"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current)
            current = current.parent
        
        return list(reversed(path))
    
    def _find_nearest_obstacle(self, position: np.ndarray) -> Optional[Obstacle]:
        """Find nearest obstacle to given position"""
        if not self.environment.obstacles:
            return None
        
        min_distance = float('inf')
        nearest_obstacle = None
        
        for obstacle in self.environment.obstacles:
            distance = np.linalg.norm(obstacle.center - position)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return nearest_obstacle

class BioInspiredRRTStar(ImprovedRRTStar):
    """RRT* variant adapted for bio-inspired underwater vehicles"""
    
    def __init__(self, environment: UnderwaterEnvironment, 
                 jet_cycle_time: float = 1.0,
                 energy_weight: float = 0.5,
                 **kwargs):
        super().__init__(environment, **kwargs)
        self.jet_cycle_time = jet_cycle_time
        self.energy_weight = energy_weight
    
    def _calculate_cost(self, from_node: RRTNode, to_node: RRTNode) -> float:
        """Calculate cost considering energy efficiency for bio-inspired vehicles"""
        # Base geometric cost
        distance = np.linalg.norm(to_node.position - from_node.position)
        
        # Energy cost based on jet propulsion
        num_jets_needed = max(1, int(distance / (self.jet_cycle_time * 2.0)))  # Approximate
        jet_energy_cost = num_jets_needed * 0.5  # Energy per jet
        
        # Orientation change cost
        orientation_diff = abs(to_node.orientation - from_node.orientation)
        orientation_penalty = min(orientation_diff, 2*np.pi - orientation_diff) * 0.3
        
        # Combined cost (distance + energy + orientation)
        total_cost = distance + self.energy_weight * jet_energy_cost + orientation_penalty
        
        return total_cost
    
    def plan_with_jet_timing(self, start_pos: np.ndarray, goal_pos: np.ndarray,
                           **kwargs) -> Optional[Tuple[List[RRTNode], List[float]]]:
        """Plan path with explicit jet timing information"""
        path = self.plan(start_pos, goal_pos, **kwargs)
        
        if path is None:
            return None
        
        # Calculate jet timing for each segment
        jet_times = []
        for i in range(len(path) - 1):
            segment_length = np.linalg.norm(path[i+1].position - path[i].position)
            jets_needed = max(1, int(segment_length / (self.jet_cycle_time * 2.0)))
            
            # Distribute jets evenly along segment
            segment_jet_times = [
                i * self.jet_cycle_time + j * (segment_length / jets_needed) / 2.0
                for j in range(jets_needed)
            ]
            jet_times.extend(segment_jet_times)
        
        return path, jet_times

def create_test_environment() -> UnderwaterEnvironment:
    """Create test environment with obstacles"""
    bounds = ((-5, 15), (-5, 15), (-10, 0))  # Underwater environment
    
    obstacles = [
        Obstacle(np.array([3, 3, -3]), 1.5),
        Obstacle(np.array([7, 2, -5]), 1.0),
        Obstacle(np.array([5, 8, -4]), 1.2),
        Obstacle(np.array([10, 6, -6]), 1.8),
        Obstacle(np.array([12, 10, -2]), 1.0),
        # Moving obstacle
        Obstacle(np.array([8, 5, -3]), 0.8, velocity=np.array([0.1, 0.1, 0]))
    ]
    
    return UnderwaterEnvironment(bounds, obstacles)

def demo_rrt_star():
    """Demonstration of improved RRT* for underwater vehicles"""
    
    # Create environment
    env = create_test_environment()
    
    # Create planners
    standard_planner = ImprovedRRTStar(env, step_size=1.5, max_iterations=500)
    bio_planner = BioInspiredRRTStar(env, step_size=1.5, max_iterations=500, 
                                   jet_cycle_time=1.0, energy_weight=0.3)
    
    # Define start and goal
    start_pos = np.array([0, 0, -2])
    goal_pos = np.array([12, 12, -8])
    
    print("Planning paths...")
    
    # Plan with standard RRT*
    start_time = time.time()
    standard_path = standard_planner.plan(start_pos, goal_pos)
    standard_time = time.time() - start_time
    
    # Plan with bio-inspired RRT*
    start_time = time.time()
    bio_result = bio_planner.plan_with_jet_timing(start_pos, goal_pos)
    bio_time = time.time() - start_time
    
    # Results
    print(f"\nStandard RRT*:")
    if standard_path:
        path_length = sum(np.linalg.norm(standard_path[i+1].position - standard_path[i].position) 
                         for i in range(len(standard_path)-1))
        print(f"  Path found: {len(standard_path)} nodes, length: {path_length:.2f}")
        print(f"  Planning time: {standard_time:.3f}s")
        print(f"  Iterations: {standard_planner.iterations_used}")
    else:
        print("  No path found")
    
    print(f"\nBio-Inspired RRT*:")
    if bio_result:
        bio_path, jet_times = bio_result
        path_length = sum(np.linalg.norm(bio_path[i+1].position - bio_path[i].position) 
                         for i in range(len(bio_path)-1))
        print(f"  Path found: {len(bio_path)} nodes, length: {path_length:.2f}")
        print(f"  Jet events: {len(jet_times)}")
        print(f"  Planning time: {bio_time:.3f}s")
        print(f"  Iterations: {bio_planner.iterations_used}")
    else:
        print("  No path found")
    
    # Visualize results
    fig = plt.figure(figsize=(15, 5))
    
    # Standard RRT* result
    ax1 = fig.add_subplot(131, projection='3d')
    
    # Plot obstacles
    for obs in env.obstacles:
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = obs.center[0] + obs.radius * np.outer(np.cos(u), np.sin(v))
        y = obs.center[1] + obs.radius * np.outer(np.sin(u), np.sin(v))
        z = obs.center[2] + obs.radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax1.plot_surface(x, y, z, alpha=0.3, color='red')
    
    if standard_path:
        path_positions = np.array([node.position for node in standard_path])
        ax1.plot(path_positions[:, 0], path_positions[:, 1], path_positions[:, 2], 
                'b-', linewidth=2, label='Standard RRT*')
    
    ax1.scatter(*start_pos, color='green', s=100, label='Start')
    ax1.scatter(*goal_pos, color='red', s=100, label='Goal')
    ax1.set_title('Standard RRT*')
    ax1.legend()
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Bio-inspired RRT* result
    ax2 = fig.add_subplot(132, projection='3d')
    
    # Plot obstacles
    for obs in env.obstacles:
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = obs.center[0] + obs.radius * np.outer(np.cos(u), np.sin(v))
        y = obs.center[1] + obs.radius * np.outer(np.sin(u), np.sin(v))
        z = obs.center[2] + obs.radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax2.plot_surface(x, y, z, alpha=0.3, color='red')
    
    if bio_result:
        bio_path, jet_times = bio_result
        path_positions = np.array([node.position for node in bio_path])
        ax2.plot(path_positions[:, 0], path_positions[:, 1], path_positions[:, 2], 
                'g-', linewidth=2, label='Bio-Inspired RRT*')
        
        # Mark jet points
        if len(jet_times) > 0:
            jet_positions = path_positions[::max(1, len(path_positions)//len(jet_times))]
            ax2.scatter(jet_positions[:, 0], jet_positions[:, 1], jet_positions[:, 2], 
                       color='orange', s=30, label='Jet Points')
    
    ax2.scatter(*start_pos, color='green', s=100, label='Start')
    ax2.scatter(*goal_pos, color='red', s=100, label='Goal')
    ax2.set_title('Bio-Inspired RRT*')
    ax2.legend()
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    
    # Comparison plot
    ax3 = fig.add_subplot(133)
    
    methods = ['Standard RRT*', 'Bio-Inspired RRT*']
    times = [standard_time if standard_path else 0, bio_time if bio_result else 0]
    iterations = [standard_planner.iterations_used, bio_planner.iterations_used]
    
    x = np.arange(len(methods))
    width = 0.35
    
    ax3.bar(x - width/2, times, width, label='Planning Time (s)', alpha=0.7)
    ax3.bar(x + width/2, [i/100 for i in iterations], width, label='Iterations (×100)', alpha=0.7)
    
    ax3.set_xlabel('Method')
    ax3.set_ylabel('Value')
    ax3.set_title('Performance Comparison')
    ax3.set_xticks(x)
    ax3.set_xticklabels(methods)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig('UnderwaterVehicleTrajectories/Simulations/rrt_star_demo.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return standard_path, bio_result

if __name__ == "__main__":
    print("Improved RRT* for Underwater Vehicles Demo")
    print("=" * 50)
    
    standard_path, bio_result = demo_rrt_star()
    
    print("\nDemo completed. Check Simulations/ folder for visualization.")
