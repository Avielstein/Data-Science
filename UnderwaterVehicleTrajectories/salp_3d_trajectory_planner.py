"""
3D SALP Trajectory Planner
Specialized trajectory planning for bio-inspired jet propulsion underwater robots
Combines 3D Dubins paths with pulsed jet propulsion and steerable nozzle control

Based on Cynthia Sung's SALP project at UPenn GRASP Lab
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import scipy.optimize as opt
from scipy.spatial.transform import Rotation as R

class SALP3DTrajectoryPlanner:
    """3D trajectory planner for SALP jet propulsion robots"""
    
    def __init__(self, 
                 min_turning_radius=1.0,
                 max_nozzle_angle=30.0,  # degrees
                 jet_thrust=2.0,         # N
                 robot_mass=0.5,         # kg
                 pulse_duration=0.2,     # seconds
                 min_pulse_interval=0.1  # seconds
                ):
        """
        Initialize SALP trajectory planner
        
        Args:
            min_turning_radius: Minimum turning radius (m)
            max_nozzle_angle: Maximum nozzle deflection angle (degrees)
            jet_thrust: Thrust force per pulse (N)
            robot_mass: Robot mass (kg)
            pulse_duration: Duration of each jet pulse (s)
            min_pulse_interval: Minimum time between pulses (s)
        """
        self.min_turning_radius = min_turning_radius
        self.max_nozzle_angle = math.radians(max_nozzle_angle)
        self.jet_thrust = jet_thrust
        self.robot_mass = robot_mass
        self.pulse_duration = pulse_duration
        self.min_pulse_interval = min_pulse_interval
        
        # Derived parameters
        self.max_acceleration = jet_thrust / robot_mass
        self.min_curvature = 1.0 / min_turning_radius
        
    def plan_3d_trajectory(self, start_pose, goal_pose, waypoints=None):
        """
        Plan 3D trajectory from start to goal
        
        Args:
            start_pose: [x, y, z, roll, pitch, yaw] (m, rad)
            goal_pose: [x, y, z, roll, pitch, yaw] (m, rad)
            waypoints: Optional list of intermediate waypoints
            
        Returns:
            dict with trajectory data
        """
        
        print("🐙 SALP 3D TRAJECTORY PLANNER")
        print("=" * 40)
        print(f"Start: {start_pose}")
        print(f"Goal: {goal_pose}")
        
        # Generate 3D Dubins path
        path_segments = self._generate_3d_dubins_path(start_pose, goal_pose)
        
        if not path_segments:
            return {'success': False, 'error': 'No valid 3D path found'}
        
        # Plan jet pulses along the path
        pulse_plan = self._plan_jet_pulses(path_segments)
        
        # Generate nozzle steering commands
        nozzle_commands = self._plan_nozzle_steering(path_segments, pulse_plan)
        
        # Create complete trajectory
        trajectory = self._generate_complete_trajectory(path_segments, pulse_plan, nozzle_commands)
        
        return {
            'success': True,
            'trajectory': trajectory,
            'path_segments': path_segments,
            'pulse_plan': pulse_plan,
            'nozzle_commands': nozzle_commands,
            'total_time': trajectory['time'][-1],
            'total_pulses': len(pulse_plan),
            'path_length': self._calculate_path_length(path_segments)
        }
    
    def _generate_3d_dubins_path(self, start_pose, goal_pose):
        """Generate 3D Dubins path segments"""
        
        # Extract position and orientation
        start_pos = np.array(start_pose[:3])
        goal_pos = np.array(goal_pose[:3])
        start_orient = np.array(start_pose[3:])
        goal_orient = np.array(goal_pose[3:])
        
        # Calculate 3D distance and direction
        displacement = goal_pos - start_pos
        distance = np.linalg.norm(displacement)
        
        print(f"3D distance: {distance:.2f}m")
        
        # For now, implement a simplified 3D path
        # This can be extended to full 3D Dubins paths later
        segments = []
        
        if distance < 0.1:  # Very close - direct path
            segments.append({
                'type': 'straight',
                'start_pos': start_pos,
                'end_pos': goal_pos,
                'start_orient': start_orient,
                'end_orient': goal_orient,
                'length': distance
            })
        else:
            # Create a simple 3-segment path: turn-straight-turn
            # This is a simplified version - full 3D Dubins would be more complex
            
            # Segment 1: Initial turn to align with goal direction
            mid_orient = self._calculate_intermediate_orientation(start_orient, goal_orient, 0.3)
            turn1_length = self._calculate_turn_length(start_orient, mid_orient)
            
            segments.append({
                'type': 'turn',
                'start_pos': start_pos,
                'end_pos': start_pos + self._get_direction_vector(start_orient) * turn1_length * 0.1,
                'start_orient': start_orient,
                'end_orient': mid_orient,
                'length': turn1_length,
                'turn_direction': self._get_turn_direction(start_orient, mid_orient)
            })
            
            # Segment 2: Straight line toward goal
            straight_start = segments[0]['end_pos']
            straight_length = distance * 0.8
            straight_end = straight_start + displacement / distance * straight_length
            
            segments.append({
                'type': 'straight',
                'start_pos': straight_start,
                'end_pos': straight_end,
                'start_orient': mid_orient,
                'end_orient': mid_orient,
                'length': straight_length
            })
            
            # Segment 3: Final turn to goal orientation
            turn2_length = self._calculate_turn_length(mid_orient, goal_orient)
            
            segments.append({
                'type': 'turn',
                'start_pos': straight_end,
                'end_pos': goal_pos,
                'start_orient': mid_orient,
                'end_orient': goal_orient,
                'length': turn2_length,
                'turn_direction': self._get_turn_direction(mid_orient, goal_orient)
            })
        
        return segments
    
    def _plan_jet_pulses(self, path_segments):
        """Plan optimal jet pulse timing along the path"""
        
        pulse_plan = []
        current_time = 0.0
        
        for segment_idx, segment in enumerate(path_segments):
            segment_length = segment['length']
            
            if segment_length < 0.1:  # Skip very short segments
                continue
            
            # Calculate required pulses for this segment
            # Estimate based on physics: F*t = m*v, distance = 0.5*a*t^2
            estimated_velocity = math.sqrt(2 * self.max_acceleration * segment_length)
            estimated_time = estimated_velocity / self.max_acceleration
            
            # Number of pulses needed
            num_pulses = max(1, int(estimated_time / (self.pulse_duration + self.min_pulse_interval)))
            
            # Distribute pulses evenly along segment
            for i in range(num_pulses):
                pulse_time = current_time + i * (estimated_time / num_pulses)
                
                pulse_plan.append({
                    'time': pulse_time,
                    'segment_index': segment_idx,
                    'segment_progress': i / num_pulses,
                    'thrust_vector': self._calculate_thrust_vector(segment, i / num_pulses),
                    'duration': self.pulse_duration
                })
            
            current_time += estimated_time
        
        print(f"Generated {len(pulse_plan)} jet pulses")
        return pulse_plan
    
    def _plan_nozzle_steering(self, path_segments, pulse_plan):
        """Plan nozzle steering angles for each pulse"""
        
        nozzle_commands = []
        
        for pulse in pulse_plan:
            segment = path_segments[pulse['segment_index']]
            progress = pulse['segment_progress']
            
            # Calculate required steering angle
            if segment['type'] == 'turn':
                # For turns, deflect nozzle to create turning moment
                turn_direction = segment.get('turn_direction', [0, 0, 1])
                required_angle = self._calculate_required_nozzle_angle(segment, progress)
                
                # Limit to maximum nozzle angle
                steering_angle = np.clip(required_angle, -self.max_nozzle_angle, self.max_nozzle_angle)
                
            else:  # straight segment
                steering_angle = 0.0  # No steering needed
            
            nozzle_commands.append({
                'time': pulse['time'],
                'steering_angle': steering_angle,
                'steering_axis': self._get_steering_axis(segment),
                'thrust_direction': pulse['thrust_vector']
            })
        
        return nozzle_commands
    
    def _generate_complete_trajectory(self, path_segments, pulse_plan, nozzle_commands):
        """Generate complete trajectory with position, velocity, and control"""
        
        # Time parameters
        total_time = pulse_plan[-1]['time'] + pulse_plan[-1]['duration'] if pulse_plan else 1.0
        dt = 0.01
        time_array = np.arange(0, total_time + dt, dt)
        
        # Initialize trajectory arrays
        trajectory = {
            'time': time_array,
            'position': {'x': [], 'y': [], 'z': []},
            'orientation': {'roll': [], 'pitch': [], 'yaw': []},
            'velocity': {'x': [], 'y': [], 'z': []},
            'angular_velocity': {'roll': [], 'pitch': [], 'yaw': []},
            'thrust': {'x': [], 'y': [], 'z': []},
            'nozzle_angle': [],
            'active_pulse': []
        }
        
        # Initial conditions
        if path_segments:
            pos = np.array(path_segments[0]['start_pos'])
            orient = np.array(path_segments[0]['start_orient'])
        else:
            pos = np.zeros(3)
            orient = np.zeros(3)
        
        vel = np.zeros(3)
        ang_vel = np.zeros(3)
        
        # Simulate trajectory
        for t in time_array:
            # Store current state
            trajectory['position']['x'].append(pos[0])
            trajectory['position']['y'].append(pos[1])
            trajectory['position']['z'].append(pos[2])
            trajectory['orientation']['roll'].append(orient[0])
            trajectory['orientation']['pitch'].append(orient[1])
            trajectory['orientation']['yaw'].append(orient[2])
            trajectory['velocity']['x'].append(vel[0])
            trajectory['velocity']['y'].append(vel[1])
            trajectory['velocity']['z'].append(vel[2])
            trajectory['angular_velocity']['roll'].append(ang_vel[0])
            trajectory['angular_velocity']['pitch'].append(ang_vel[1])
            trajectory['angular_velocity']['yaw'].append(ang_vel[2])
            
            # Check for active pulse
            active_pulse = None
            for pulse in pulse_plan:
                if pulse['time'] <= t <= pulse['time'] + pulse['duration']:
                    active_pulse = pulse
                    break
            
            # Calculate thrust and nozzle angle
            if active_pulse:
                thrust = np.array(active_pulse['thrust_vector']) * self.jet_thrust
                
                # Find corresponding nozzle command
                nozzle_cmd = None
                for cmd in nozzle_commands:
                    if abs(cmd['time'] - active_pulse['time']) < 0.001:
                        nozzle_cmd = cmd
                        break
                
                nozzle_angle = nozzle_cmd['steering_angle'] if nozzle_cmd else 0.0
                trajectory['active_pulse'].append(True)
            else:
                thrust = np.zeros(3)
                nozzle_angle = 0.0
                trajectory['active_pulse'].append(False)
            
            trajectory['thrust']['x'].append(thrust[0])
            trajectory['thrust']['y'].append(thrust[1])
            trajectory['thrust']['z'].append(thrust[2])
            trajectory['nozzle_angle'].append(math.degrees(nozzle_angle))
            
            # Update dynamics
            acceleration = thrust / self.robot_mass
            
            # Simple integration (Euler method)
            vel += acceleration * dt
            pos += vel * dt
            
            # Add some damping to prevent unrealistic velocities
            vel *= 0.99
        
        return trajectory
    
    def _calculate_intermediate_orientation(self, start_orient, goal_orient, t):
        """Calculate intermediate orientation using spherical interpolation"""
        # Simple linear interpolation for now - could use SLERP for better results
        return start_orient + t * (goal_orient - start_orient)
    
    def _calculate_turn_length(self, start_orient, end_orient):
        """Calculate length of turn segment"""
        angle_diff = np.linalg.norm(end_orient - start_orient)
        return angle_diff * self.min_turning_radius
    
    def _get_direction_vector(self, orientation):
        """Get forward direction vector from orientation"""
        # Convert Euler angles to direction vector
        roll, pitch, yaw = orientation
        return np.array([
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch)
        ])
    
    def _get_turn_direction(self, start_orient, end_orient):
        """Get turn direction vector"""
        # Simplified - return normalized difference
        diff = end_orient - start_orient
        norm = np.linalg.norm(diff)
        return diff / norm if norm > 0 else np.array([0, 0, 1])
    
    def _calculate_thrust_vector(self, segment, progress):
        """Calculate thrust vector for given segment and progress"""
        if segment['type'] == 'straight':
            # Thrust in forward direction
            direction = segment['end_pos'] - segment['start_pos']
            norm = np.linalg.norm(direction)
            return direction / norm if norm > 0 else np.array([1, 0, 0])
        else:  # turn
            # Thrust with some lateral component for turning
            forward = self._get_direction_vector(segment['start_orient'])
            lateral = segment.get('turn_direction', np.array([0, 0, 1]))
            
            # Mix forward and lateral thrust
            thrust_vec = 0.8 * forward + 0.2 * lateral
            norm = np.linalg.norm(thrust_vec)
            return thrust_vec / norm if norm > 0 else forward
    
    def _calculate_required_nozzle_angle(self, segment, progress):
        """Calculate required nozzle steering angle"""
        if segment['type'] != 'turn':
            return 0.0
        
        # Simple model: angle proportional to turn sharpness
        turn_angle = np.linalg.norm(segment['end_orient'] - segment['start_orient'])
        max_turn_angle = math.pi / 2  # 90 degrees
        
        # Scale to nozzle range
        normalized_turn = min(turn_angle / max_turn_angle, 1.0)
        return normalized_turn * self.max_nozzle_angle
    
    def _get_steering_axis(self, segment):
        """Get steering axis for nozzle deflection"""
        if segment['type'] == 'turn':
            return segment.get('turn_direction', np.array([0, 0, 1]))
        return np.array([0, 0, 1])  # Default to yaw axis
    
    def _calculate_path_length(self, path_segments):
        """Calculate total path length"""
        return sum(segment['length'] for segment in path_segments)
    
    def visualize_3d_trajectory(self, result):
        """Visualize 3D trajectory with jet pulses and nozzle steering"""
        
        if not result['success']:
            print("❌ Cannot visualize - no valid trajectory")
            return
        
        # Configure matplotlib to avoid font warnings
        import matplotlib
        matplotlib.rcParams['font.family'] = 'DejaVu Sans'
        matplotlib.rcParams['figure.dpi'] = 100
        matplotlib.rcParams['savefig.dpi'] = 300
        import warnings
        warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
        
        trajectory = result['trajectory']
        
        # Create 3D plot
        fig = plt.figure(figsize=(16, 12))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        
        # Plot trajectory
        ax1.plot(trajectory['position']['x'], 
                trajectory['position']['y'], 
                trajectory['position']['z'], 
                'b-', linewidth=2, label='Trajectory')
        
        # Mark start and end
        ax1.scatter(trajectory['position']['x'][0], 
                   trajectory['position']['y'][0], 
                   trajectory['position']['z'][0], 
                   c='green', s=100, label='Start')
        ax1.scatter(trajectory['position']['x'][-1], 
                   trajectory['position']['y'][-1], 
                   trajectory['position']['z'][-1], 
                   c='red', s=100, label='Goal')
        
        # Mark jet pulses
        pulse_times = [i for i, active in enumerate(trajectory['active_pulse']) if active]
        if pulse_times:
            pulse_x = [trajectory['position']['x'][i] for i in pulse_times[::10]]  # Every 10th pulse point
            pulse_y = [trajectory['position']['y'][i] for i in pulse_times[::10]]
            pulse_z = [trajectory['position']['z'][i] for i in pulse_times[::10]]
            ax1.scatter(pulse_x, pulse_y, pulse_z, c='orange', s=30, alpha=0.7, label='Jet Pulses')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.legend()
        ax1.set_title('3D SALP Trajectory')
        
        # Velocity profiles
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(trajectory['time'], trajectory['velocity']['x'], 'r-', label='vx')
        ax2.plot(trajectory['time'], trajectory['velocity']['y'], 'g-', label='vy')
        ax2.plot(trajectory['time'], trajectory['velocity']['z'], 'b-', label='vz')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.set_title('Velocity Profiles')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        
        # Thrust profiles
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(trajectory['time'], trajectory['thrust']['x'], 'r-', label='Fx')
        ax3.plot(trajectory['time'], trajectory['thrust']['y'], 'g-', label='Fy')
        ax3.plot(trajectory['time'], trajectory['thrust']['z'], 'b-', label='Fz')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        ax3.set_title('Thrust Forces')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Force (N)')
        
        # Nozzle steering angle
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(trajectory['time'], trajectory['nozzle_angle'], 'purple', linewidth=2)
        ax4.axhline(math.degrees(self.max_nozzle_angle), color='red', linestyle='--', alpha=0.5, label='Max Angle')
        ax4.axhline(-math.degrees(self.max_nozzle_angle), color='red', linestyle='--', alpha=0.5)
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        ax4.set_title('Nozzle Steering Angle')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angle (degrees)')
        
        # Orientation profiles
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(trajectory['time'], np.degrees(trajectory['orientation']['roll']), 'r-', label='Roll')
        ax5.plot(trajectory['time'], np.degrees(trajectory['orientation']['pitch']), 'g-', label='Pitch')
        ax5.plot(trajectory['time'], np.degrees(trajectory['orientation']['yaw']), 'b-', label='Yaw')
        ax5.grid(True, alpha=0.3)
        ax5.legend()
        ax5.set_title('Orientation')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Angle (degrees)')
        
        # Pulse timing
        ax6 = fig.add_subplot(2, 3, 6)
        pulse_indicator = [1 if active else 0 for active in trajectory['active_pulse']]
        ax6.plot(trajectory['time'], pulse_indicator, 'orange', linewidth=3)
        ax6.fill_between(trajectory['time'], pulse_indicator, alpha=0.3, color='orange')
        ax6.grid(True, alpha=0.3)
        ax6.set_title('Jet Pulse Timing')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Pulse Active')
        ax6.set_ylim(-0.1, 1.1)
        
        plt.tight_layout()
        plt.savefig('salp_3d_trajectory.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # Print summary
        print(f"\n📊 TRAJECTORY SUMMARY")
        print(f"Total time: {result['total_time']:.2f}s")
        print(f"Total pulses: {result['total_pulses']}")
        print(f"Path length: {result['path_length']:.2f}m")
        print(f"Average speed: {result['path_length']/result['total_time']:.2f}m/s")
        
        max_vel = max(np.sqrt(np.array(trajectory['velocity']['x'])**2 + 
                             np.array(trajectory['velocity']['y'])**2 + 
                             np.array(trajectory['velocity']['z'])**2))
        print(f"Maximum speed: {max_vel:.2f}m/s")

def test_salp_3d_planner():
    """Test the SALP 3D trajectory planner"""
    
    print("🐙 TESTING SALP 3D TRAJECTORY PLANNER")
    print("=" * 50)
    
    # Create planner
    planner = SALP3DTrajectoryPlanner(
        min_turning_radius=2.0,
        max_nozzle_angle=25.0,  # degrees
        jet_thrust=3.0,         # N
        robot_mass=0.8,         # kg
        pulse_duration=0.15,    # seconds
        min_pulse_interval=0.05 # seconds
    )
    
    # Test trajectory
    start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    goal_pose = [10.0, 5.0, -2.0, 0.0, 0.2, 1.57]  # Turn and dive
    
    print(f"Planning trajectory from {start_pose} to {goal_pose}")
    
    # Plan trajectory
    result = planner.plan_3d_trajectory(start_pose, goal_pose)
    
    if result['success']:
        print("✅ 3D trajectory planning successful!")
        
        # Visualize
        planner.visualize_3d_trajectory(result)
        
        return True
    else:
        print(f"❌ Planning failed: {result.get('error', 'Unknown error')}")
        return False

if __name__ == "__main__":
    success = test_salp_3d_planner()
    print(f"\nTest {'✅ PASSED' if success else '❌ FAILED'}")
