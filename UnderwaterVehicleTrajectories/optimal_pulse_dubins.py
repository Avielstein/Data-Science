"""
Optimal Control for Pulsed Dubins Paths
Combines optimal control theory with Dubins path planning for pulsed propulsion systems
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from optimal_control import OptimalController
from dubins import plan_dubins_path

class OptimalPulseDubins:
    """Optimal control system for pulsed Dubins-like trajectories"""
    
    def __init__(self, pulse_duration=0.1, pulse_strength=2.0, min_pulse_interval=0.5):
        self.pulse_duration = pulse_duration
        self.pulse_strength = pulse_strength
        self.min_pulse_interval = min_pulse_interval
        self.optimal_controller = OptimalController(max_acceleration=pulse_strength)
        
    def plan_pulsed_trajectory(self, start_pose, goal_pose, turning_radius=2.0):
        """
        Plan a pulsed trajectory using optimal control principles
        
        Args:
            start_pose: [x, y, theta] starting position and orientation
            goal_pose: [x, y, theta] goal position and orientation
            turning_radius: minimum turning radius for the vehicle
            
        Returns:
            dict with trajectory plan and pulse schedule
        """
        
        print(f"🌊 OPTIMAL PULSE DUBINS PLANNING")
        print(f"Start: ({start_pose[0]:.1f}, {start_pose[1]:.1f}) facing {math.degrees(start_pose[2]):.0f}°")
        print(f"Goal: ({goal_pose[0]:.1f}, {goal_pose[1]:.1f}) facing {math.degrees(goal_pose[2]):.0f}°")
        
        # Step 1: Generate reference Dubins path
        curvature = 1.0 / turning_radius
        x_list, y_list, yaw_list, mode, lengths = plan_dubins_path(
            start_pose[0], start_pose[1], start_pose[2],
            goal_pose[0], goal_pose[1], goal_pose[2],
            curvature, step_size=0.1
        )
        
        if x_list is None:
            raise Exception("No valid Dubins path found")
        
        reference_path = {'x': x_list, 'y': y_list, 'yaw': yaw_list}
        total_length = sum(lengths)
        
        print(f"📍 Reference Dubins path: {mode}, length: {total_length:.2f}m")
        
        # Step 2: Break path into segments for optimal control
        segments = self._segment_path(reference_path)
        
        # Step 3: Use optimal control for each segment
        pulse_schedule = []
        optimized_trajectory = {'time': [], 'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': []}
        
        current_time = 0.0
        current_state = [start_pose[0], start_pose[1], 0.0, 0.0]  # [x, y, vx, vy]
        
        for i, segment in enumerate(segments):
            print(f"\n🎯 Optimizing segment {i+1}/{len(segments)}")
            
            # Define target state for this segment
            target_state = [segment['end'][0], segment['end'][1], 0.0, 0.0]
            
            # Use optimal control to find best trajectory
            result = self.optimal_controller.solve(current_state, target_state)
            
            if result['success']:
                print(f"   ✅ Success: {result['error']:.3f}m error")
                
                # Generate detailed trajectory
                trajectory = self.optimal_controller.generate_trajectory(
                    result['parameters'], current_state
                )
                
                # Convert to pulse schedule
                pulses = self._trajectory_to_pulses(trajectory, current_time)
                pulse_schedule.extend(pulses)
                
                # Add to optimized trajectory
                for j, t in enumerate(trajectory['time']):
                    optimized_trajectory['time'].append(current_time + t)
                    optimized_trajectory['x'].append(trajectory['position']['x'][j])
                    optimized_trajectory['y'].append(trajectory['position']['y'][j])
                    optimized_trajectory['vx'].append(trajectory['velocity']['x'][j])
                    optimized_trajectory['vy'].append(trajectory['velocity']['y'][j])
                    
                    # Estimate orientation from velocity
                    vx, vy = trajectory['velocity']['x'][j], trajectory['velocity']['y'][j]
                    if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                        theta = math.atan2(vy, vx)
                    else:
                        theta = optimized_trajectory['theta'][-1] if optimized_trajectory['theta'] else start_pose[2]
                    optimized_trajectory['theta'].append(theta)
                
                # Update for next segment
                current_time += trajectory['time'][-1]
                current_state = [
                    trajectory['position']['x'][-1],
                    trajectory['position']['y'][-1],
                    trajectory['velocity']['x'][-1],
                    trajectory['velocity']['y'][-1]
                ]
                
            else:
                print(f"   ❌ Failed to optimize segment {i+1}")
                # Fall back to simple pulse schedule
                fallback_pulses = self._fallback_pulse_schedule(segment, current_time)
                pulse_schedule.extend(fallback_pulses)
                current_time += segment['duration']
        
        return {
            'reference_path': reference_path,
            'optimized_trajectory': optimized_trajectory,
            'pulse_schedule': pulse_schedule,
            'total_time': current_time,
            'total_pulses': len(pulse_schedule)
        }
    
    def _segment_path(self, reference_path, max_segment_length=3.0):
        """Break reference path into segments for optimization"""
        
        segments = []
        current_start = 0
        
        while current_start < len(reference_path['x']) - 1:
            # Find segment end
            segment_length = 0.0
            segment_end = current_start + 1
            
            while (segment_end < len(reference_path['x']) and 
                   segment_length < max_segment_length):
                dx = reference_path['x'][segment_end] - reference_path['x'][segment_end-1]
                dy = reference_path['y'][segment_end] - reference_path['y'][segment_end-1]
                segment_length += math.sqrt(dx**2 + dy**2)
                segment_end += 1
            
            # Create segment
            segment = {
                'start': [reference_path['x'][current_start], reference_path['y'][current_start]],
                'end': [reference_path['x'][segment_end-1], reference_path['y'][segment_end-1]],
                'length': segment_length,
                'duration': segment_length / 1.0  # Assume 1 m/s average speed
            }
            
            segments.append(segment)
            current_start = segment_end - 1
        
        return segments
    
    def _trajectory_to_pulses(self, trajectory, start_time):
        """Convert optimal control trajectory to discrete pulse schedule"""
        
        pulses = []
        last_pulse_time = -self.min_pulse_interval
        
        for i, t in enumerate(trajectory['time']):
            # Check if we need a pulse based on control magnitude
            ux = trajectory['control']['x'][i]
            uy = trajectory['control']['y'][i]
            control_magnitude = math.sqrt(ux**2 + uy**2)
            
            # Pulse if control is significant and enough time has passed
            if (control_magnitude > 0.1 and 
                t - last_pulse_time >= self.min_pulse_interval):
                
                pulse = {
                    'time': start_time + t,
                    'position': [trajectory['position']['x'][i], trajectory['position']['y'][i]],
                    'direction': math.atan2(uy, ux),
                    'magnitude': min(control_magnitude, self.pulse_strength)
                }
                
                pulses.append(pulse)
                last_pulse_time = t
        
        return pulses
    
    def _fallback_pulse_schedule(self, segment, start_time):
        """Simple fallback pulse schedule if optimization fails"""
        
        # Simple: one pulse at start, one at middle, one at end
        dx = segment['end'][0] - segment['start'][0]
        dy = segment['end'][1] - segment['start'][1]
        direction = math.atan2(dy, dx)
        
        pulses = [
            {
                'time': start_time,
                'position': segment['start'],
                'direction': direction,
                'magnitude': self.pulse_strength
            },
            {
                'time': start_time + segment['duration'] / 2,
                'position': [(segment['start'][0] + segment['end'][0]) / 2,
                           (segment['start'][1] + segment['end'][1]) / 2],
                'direction': direction,
                'magnitude': self.pulse_strength
            }
        ]
        
        return pulses
    
    def visualize_plan(self, plan, title="Optimal Pulse Dubins Plan"):
        """Visualize the complete plan"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        
        # Main trajectory plot
        ax = axes[0, 0]
        
        # Reference Dubins path
        ref = plan['reference_path']
        ax.plot(ref['x'], ref['y'], 'k--', linewidth=1, alpha=0.5, label='Reference Dubins')
        
        # Optimized trajectory
        opt = plan['optimized_trajectory']
        if opt['x']:
            ax.plot(opt['x'], opt['y'], 'b-', linewidth=2, label='Optimized Path')
        
        # Pulse locations
        for i, pulse in enumerate(plan['pulse_schedule']):
            x, y = pulse['position']
            direction = pulse['direction']
            magnitude = pulse['magnitude']
            
            # Pulse arrow
            dx = 0.3 * magnitude * math.cos(direction)
            dy = 0.3 * magnitude * math.sin(direction)
            ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.1, 
                    fc='red', ec='red', alpha=0.8)
            
            # Pulse number
            if i < 10:  # Don't overcrowd
                ax.text(x + 0.2, y + 0.2, str(i+1), fontsize=8, 
                       bbox=dict(boxstyle="round,pad=0.1", facecolor="white", alpha=0.7))
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Trajectory with Pulse Schedule')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        
        # Pulse timing
        ax = axes[0, 1]
        pulse_times = [p['time'] for p in plan['pulse_schedule']]
        pulse_magnitudes = [p['magnitude'] for p in plan['pulse_schedule']]
        
        ax.stem(pulse_times, pulse_magnitudes, basefmt=' ')
        ax.grid(True, alpha=0.3)
        ax.set_title('Pulse Schedule')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Pulse Magnitude')
        
        # Velocity profile
        ax = axes[1, 0]
        if opt['time']:
            speed = [math.sqrt(vx**2 + vy**2) for vx, vy in zip(opt['vx'], opt['vy'])]
            ax.plot(opt['time'], speed, 'g-', linewidth=2)
            ax.grid(True, alpha=0.3)
            ax.set_title('Speed Profile')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Speed (m/s)')
        
        # Statistics
        ax = axes[1, 1]
        ax.axis('off')
        
        stats_text = f"""
📊 PLAN STATISTICS:
        
Total Time: {plan['total_time']:.1f}s
Total Pulses: {plan['total_pulses']}
Pulse Rate: {plan['total_pulses']/plan['total_time']:.2f} Hz

Reference Path: {len(plan['reference_path']['x'])} points
Optimized Path: {len(opt['x'])} points

Pulse Strength: {self.pulse_strength:.1f}
Pulse Duration: {self.pulse_duration:.1f}s
Min Interval: {self.min_pulse_interval:.1f}s
        """
        
        ax.text(0.1, 0.9, stats_text, transform=ax.transAxes, fontsize=10,
               verticalalignment='top', fontfamily='monospace')
        
        plt.suptitle(title, fontsize=14)
        plt.tight_layout()
        return fig

def test_optimal_pulse_dubins():
    """Test the optimal pulse Dubins system"""
    
    print("🌊 OPTIMAL PULSE DUBINS TEST")
    print("=" * 40)
    
    # Test cases
    test_cases = [
        {
            'name': 'Simple Turn',
            'start': [0.0, 0.0, 0.0],  # [x, y, theta]
            'goal': [5.0, 3.0, math.pi/4],
            'description': 'Move and turn 45°'
        },
        {
            'name': 'Sharp Turn',
            'start': [0.0, 0.0, 0.0],
            'goal': [3.0, 3.0, math.pi/2],
            'description': 'Move and turn 90°'
        }
    ]
    
    planner = OptimalPulseDubins(
        pulse_duration=0.1,
        pulse_strength=2.0,
        min_pulse_interval=0.5
    )
    
    for i, case in enumerate(test_cases):
        print(f"\n🎯 Test {i+1}: {case['name']}")
        print(f"   {case['description']}")
        
        try:
            plan = planner.plan_pulsed_trajectory(case['start'], case['goal'])
            
            print(f"   ✅ Success!")
            print(f"   Total time: {plan['total_time']:.1f}s")
            print(f"   Total pulses: {plan['total_pulses']}")
            
            # Visualize the first successful case
            if i == 0:
                fig = planner.visualize_plan(plan, f"Test: {case['name']}")
                plt.savefig('optimal_pulse_dubins_test.png', dpi=300, bbox_inches='tight')
                plt.show()
                
        except Exception as e:
            print(f"   ❌ Failed: {str(e)}")

if __name__ == "__main__":
    test_optimal_pulse_dubins()
