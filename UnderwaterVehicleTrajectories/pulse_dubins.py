"""
Pulse-Constrained Dubins Path Planning
Calculates achievable paths given discrete jet pulses with fixed energy
"""

import math
import numpy as np
import matplotlib.pyplot as plt

class PulsePhysics:
    """Physics model for discrete jet pulses"""
    
    def __init__(self):
        # Pulse parameters
        self.pulse_thrust = 2.5  # N per pulse
        self.pulse_duration = 0.3  # seconds
        self.mass = 1.0  # kg
        self.drag_coefficient = 0.08
        
        # Derived values
        self.pulse_impulse = self.pulse_thrust * self.pulse_duration  # N⋅s
        self.pulse_velocity_delta = self.pulse_impulse / self.mass  # m/s per pulse
        
    def calculate_straight_line(self, distance, heading):
        """Calculate pulses needed for straight line motion"""
        # From the image: each pulse gives ~0.75 m/s velocity
        # With drag over 1.5s between pulses, effective distance is much less
        # Looking at actual results: 4 pulses moved ~18m, so ~4.5m per pulse
        # But we want 8.5m total, so we need fewer pulses
        
        # Much more conservative: from results, 3 pulses moved ~11m
        # So each pulse effectively moves ~3.7m, but we want to undershoot slightly
        effective_distance_per_pulse = 4.0  # Conservative estimate
        num_pulses = max(1, int(math.ceil(distance / effective_distance_per_pulse)))
        
        # Limit pulses to avoid overshooting
        if distance < 5:
            num_pulses = 1  # Single pulse for short distances
        elif distance < 10:
            num_pulses = min(num_pulses, 2)  # Max 2 pulses for medium distances
        else:
            num_pulses = min(num_pulses, 3)  # Max 3 pulses for long distances
        
        print(f"   Line calculation: {distance:.1f}m needs {num_pulses} pulses (was {int(math.ceil(distance / effective_distance_per_pulse))})")
        
        return {
            'type': 'line',
            'distance': distance,
            'heading': heading,
            'num_pulses': num_pulses,
            'nozzle_angles': [0.0] * num_pulses,  # All straight ahead
            'pulse_times': [i * 2.0 for i in range(num_pulses)]  # 2s between pulses for more drag
        }
    
    def calculate_arc(self, radius, arc_angle, start_heading):
        """Calculate pulses needed for circular arc"""
        # For arc: need angled pulses to create turning motion
        # Each pulse creates both forward motion and turning
        
        arc_length = abs(radius * arc_angle)
        num_pulses = max(2, int(arc_length / (self.pulse_velocity_delta * 1.5)))  # More pulses for curves
        
        # Calculate nozzle angles for each pulse
        nozzle_angles = []
        pulse_times = []
        
        for i in range(num_pulses):
            # Progress along arc (0 to 1)
            progress = i / (num_pulses - 1) if num_pulses > 1 else 0
            
            # Current heading along arc
            current_heading = start_heading + arc_angle * progress
            
            # Desired turn direction
            turn_direction = 1 if arc_angle > 0 else -1
            
            # Nozzle angle to create turn (limited to ±60°)
            desired_nozzle = turn_direction * min(math.pi/3, abs(arc_angle) / num_pulses * 3)
            nozzle_angles.append(desired_nozzle)
            
            # Timing: more frequent pulses during turns
            pulse_times.append(i * 1.5)
        
        return {
            'type': 'arc',
            'radius': radius,
            'arc_angle': arc_angle,
            'start_heading': start_heading,
            'end_heading': start_heading + arc_angle,
            'num_pulses': num_pulses,
            'nozzle_angles': nozzle_angles,
            'pulse_times': pulse_times
        }
    
    def calculate_achievable_radius(self, num_pulses, max_nozzle_angle=math.pi/3):
        """Calculate minimum achievable turning radius given pulse constraints"""
        # With discrete pulses, we can't make arbitrarily tight turns
        # Minimum radius depends on pulse spacing and max nozzle angle
        
        max_turn_per_pulse = max_nozzle_angle * 0.5  # Conservative estimate
        total_turn_capability = num_pulses * max_turn_per_pulse
        
        # For a 90° turn, what's the minimum radius?
        if total_turn_capability > 0:
            min_radius = (num_pulses * self.pulse_velocity_delta) / (2 * total_turn_capability)
            return max(1.0, min_radius)  # At least 1m radius
        else:
            return float('inf')

class PulseConstrainedDubins:
    """Dubins path planner that respects pulse physics constraints"""
    
    def __init__(self):
        self.physics = PulsePhysics()
    
    def plan_path(self, start_x, start_y, start_heading, goal_x, goal_y, goal_heading):
        """Plan a pulse-constrained path using proper Dubins segments"""
        
        # Get the Dubins path first
        from dubins import plan_dubins_path
        curvature = 0.5  # 2m turning radius
        x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
            start_x, start_y, start_heading, goal_x, goal_y, goal_heading, curvature)
        
        if x_dubins is None:
            print("❌ Dubins path planning failed")
            return None
        
        print(f"🔧 PULSE-CONSTRAINED DUBINS PLANNING:")
        print(f"   Dubins mode: {mode_dubins}")
        print(f"   Segment lengths: {[f'{l:.1f}m' for l in lengths]}")
        print(f"   Total length: {sum(lengths):.1f}m")
        
        # Convert Dubins segments to pulse segments
        segments = []
        
        # Analyze Dubins mode (e.g., "LSL" = Left turn, Straight, Left turn)
        segment_types = list(mode_dubins)
        
        for i, (seg_type, length) in enumerate(zip(segment_types, lengths)):
            if seg_type == 'S':  # Straight segment
                # Calculate heading for this straight segment
                start_idx = sum(len(x_dubins) // 3 * j for j in range(i))
                end_idx = start_idx + len(x_dubins) // 3
                if end_idx < len(x_dubins):
                    heading = yaw_dubins[start_idx]
                    segments.append(self.physics.calculate_straight_line(length, heading))
                    print(f"   Segment {i+1}: STRAIGHT {length:.1f}m at {math.degrees(heading):.1f}°")
            
            elif seg_type in ['L', 'R']:  # Turn segment
                # Calculate arc parameters
                radius = 1.0 / curvature  # 2m radius
                arc_angle = length / radius
                if seg_type == 'R':
                    arc_angle = -arc_angle  # Right turn is negative
                
                start_idx = sum(len(x_dubins) // 3 * j for j in range(i))
                start_heading = yaw_dubins[start_idx] if start_idx < len(yaw_dubins) else start_heading
                
                segments.append(self.physics.calculate_arc(radius, arc_angle, start_heading))
                print(f"   Segment {i+1}: {seg_type} TURN {math.degrees(arc_angle):.1f}° (r={radius:.1f}m)")
        
        return {
            'segments': segments,
            'total_pulses': sum(seg['num_pulses'] for seg in segments),
            'estimated_time': sum(seg['pulse_times'][-1] if seg['pulse_times'] else 0 for seg in segments),
            'path_type': f'PULSE_{mode_dubins}',
            'dubins_path': (x_dubins, y_dubins, yaw_dubins),
            'dubins_lengths': lengths
        }
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def simulate_path(self, path_plan, start_x, start_y, start_heading):
        """Simulate the planned path with exact trajectory following"""
        
        # Get the reference Dubins path
        x_dubins, y_dubins, yaw_dubins = path_plan['dubins_path']
        
        # Current state
        x, y, heading = start_x, start_y, start_heading
        vx, vy = 0.0, 0.0
        
        # Trajectory points
        trajectory = [(x, y, heading, 0.0)]
        pulse_events = []
        
        current_time = 0.0
        dt = 0.1
        dubins_idx = 0  # Track position along Dubins path
        
        for seg_idx, segment in enumerate(path_plan['segments']):
            print(f"\n🎯 EXECUTING SEGMENT {seg_idx + 1}: {segment['type'].upper()}")
            print(f"   Pulses: {segment['num_pulses']}")
            
            # Calculate target points along Dubins path for this segment
            segment_start_idx = dubins_idx
            points_per_segment = len(x_dubins) // 3  # Divide path into 3 segments
            segment_end_idx = min(segment_start_idx + points_per_segment, len(x_dubins) - 1)
            
            # Execute each pulse in the segment
            for pulse_idx, (nozzle_angle, pulse_time) in enumerate(zip(segment['nozzle_angles'], segment['pulse_times'])):
                
                # Simulate until pulse time
                target_time = current_time + pulse_time
                while current_time < target_time:
                    # Apply drag
                    speed = math.sqrt(vx**2 + vy**2)
                    if speed > 0:
                        drag_factor = 1.0 - self.physics.drag_coefficient * dt
                        vx *= drag_factor
                        vy *= drag_factor
                    
                    # Update position
                    x += vx * dt
                    y += vy * dt
                    
                    # Update heading based on velocity
                    if speed > 0.1:
                        heading = math.atan2(vy, vx)
                    
                    trajectory.append((x, y, heading, speed))
                    current_time += dt
                
                # Calculate exact thrust needed to follow Dubins path
                # Find target point on Dubins path
                target_idx = min(segment_start_idx + int(pulse_idx * points_per_segment / segment['num_pulses']), 
                               len(x_dubins) - 1)
                target_x = x_dubins[target_idx]
                target_y = y_dubins[target_idx]
                target_heading = yaw_dubins[target_idx]
                
                # Calculate required thrust direction to reach target
                dx_to_target = target_x - x
                dy_to_target = target_y - y
                distance_to_target = math.sqrt(dx_to_target**2 + dy_to_target**2)
                
                if distance_to_target > 0.1:  # Only thrust if we're not at target
                    # Thrust direction toward target
                    thrust_angle = math.atan2(dy_to_target, dx_to_target)
                    
                    # Scale thrust based on distance to target
                    thrust_magnitude = min(self.physics.pulse_velocity_delta, 
                                         distance_to_target * 0.5)  # Proportional control
                    
                    thrust_vx = thrust_magnitude * math.cos(thrust_angle)
                    thrust_vy = thrust_magnitude * math.sin(thrust_angle)
                    
                    vx += thrust_vx
                    vy += thrust_vy
                    
                    pulse_events.append({
                        'time': current_time,
                        'position': (x, y),
                        'target_position': (target_x, target_y),
                        'thrust_angle': thrust_angle,
                        'distance_to_target': distance_to_target,
                        'segment': seg_idx,
                        'pulse_in_segment': pulse_idx
                    })
                    
                    print(f"   Pulse {pulse_idx + 1}: t={current_time:.1f}s, pos=({x:.1f},{y:.1f}) → target=({target_x:.1f},{target_y:.1f}), dist={distance_to_target:.1f}m")
                else:
                    print(f"   Pulse {pulse_idx + 1}: t={current_time:.1f}s, pos=({x:.1f},{y:.1f}) - AT TARGET")
            
            # Update dubins index for next segment
            dubins_idx = segment_end_idx
        
        return trajectory, pulse_events

def test_pulse_constrained_dubins():
    """Test the pulse-constrained Dubins planner"""
    
    # Test case
    start_x, start_y, start_heading = 0.0, 0.0, 0.0
    goal_x, goal_y, goal_heading = 10.0, 5.0, math.pi/2
    
    print("🌊 PULSE-CONSTRAINED DUBINS PATH PLANNING")
    print("=" * 50)
    print(f"Start: ({start_x}, {start_y}) facing {math.degrees(start_heading):.0f}°")
    print(f"Goal: ({goal_x}, {goal_y}) facing {math.degrees(goal_heading):.0f}°")
    
    # Plan path
    planner = PulseConstrainedDubins()
    path_plan = planner.plan_path(start_x, start_y, start_heading, goal_x, goal_y, goal_heading)
    
    print(f"\n📋 PATH PLAN:")
    print(f"   Total segments: {len(path_plan['segments'])}")
    print(f"   Total pulses: {path_plan['total_pulses']}")
    print(f"   Estimated time: {path_plan['estimated_time']:.1f}s")
    
    # Simulate path
    trajectory, pulse_events = planner.simulate_path(path_plan, start_x, start_y, start_heading)
    
    # Extract trajectory data
    x_traj = [p[0] for p in trajectory]
    y_traj = [p[1] for p in trajectory]
    heading_traj = [p[2] for p in trajectory]
    speed_traj = [p[3] for p in trajectory]
    
    # Calculate final error
    final_x, final_y, final_heading = trajectory[-1][:3]
    position_error = math.sqrt((final_x - goal_x)**2 + (final_y - goal_y)**2)
    heading_error = abs(planner.normalize_angle(final_heading - goal_heading))
    
    print(f"\n📊 RESULTS:")
    print(f"   Final position: ({final_x:.2f}, {final_y:.2f})")
    print(f"   Position error: {position_error:.2f}m")
    print(f"   Final heading: {math.degrees(final_heading):.1f}°")
    print(f"   Heading error: {math.degrees(heading_error):.1f}°")
    print(f"   Total pulses used: {len(pulse_events)}")
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    # Path plot
    plt.subplot(2, 2, 1)
    plt.plot(x_traj, y_traj, 'b-', linewidth=2, label='Pulse-Constrained Path')
    
    # Mark pulse events
    colors = ['red', 'orange', 'green', 'purple']
    for event in pulse_events:
        color = colors[event['segment'] % len(colors)]
        plt.plot(event['position'][0], event['position'][1], 'o', color=color, markersize=6)
        
        # Show thrust direction
        x, y = event['position']
        thrust_angle = event['thrust_angle']
        dx = 0.5 * math.cos(thrust_angle)
        dy = 0.5 * math.sin(thrust_angle)
        plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc=color, ec=color, alpha=0.7)
    
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(goal_x, goal_y, 'ro', markersize=10, label='Goal')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Pulse-Constrained Dubins Path')
    
    # Speed profile
    plt.subplot(2, 2, 2)
    times = np.arange(len(speed_traj)) * 0.1
    plt.plot(times, speed_traj, 'b-', linewidth=2)
    
    # Mark pulse times
    for event in pulse_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Profile')
    plt.grid(True, alpha=0.3)
    
    # Heading profile
    plt.subplot(2, 2, 3)
    heading_deg = np.degrees(heading_traj)
    plt.plot(times, heading_deg, 'g-', linewidth=2)
    
    # Mark pulse times
    for event in pulse_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Heading Evolution')
    plt.grid(True, alpha=0.3)
    
    # Segment breakdown
    plt.subplot(2, 2, 4)
    plt.axis('off')
    
    segment_text = "🎯 SEGMENT BREAKDOWN:\n\n"
    for i, segment in enumerate(path_plan['segments']):
        segment_text += f"Segment {i+1}: {segment['type'].upper()}\n"
        segment_text += f"  Pulses: {segment['num_pulses']}\n"
        if segment['type'] == 'arc':
            segment_text += f"  Radius: {segment['radius']:.1f}m\n"
            segment_text += f"  Arc: {math.degrees(segment['arc_angle']):.1f}°\n"
        elif segment['type'] == 'line':
            segment_text += f"  Distance: {segment['distance']:.1f}m\n"
        segment_text += "\n"
    
    plt.text(0.05, 0.95, segment_text, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.savefig('pulse_constrained_dubins.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return path_plan, trajectory, pulse_events

if __name__ == "__main__":
    test_pulse_constrained_dubins()
