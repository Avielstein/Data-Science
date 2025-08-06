"""
Rocket Jellyfish Physics Simulation
Proper momentum-based physics for underwater vehicle with rear nozzle
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class RocketJellyfish:
    """
    Physics simulation of jellyfish-like vehicle with rear nozzle
    
    Key physics:
    - Momentum conservation (like rocket in space)
    - Rotational inertia and angular momentum
    - Water drag forces
    - Vectored thrust from rear nozzle
    """
    
    def __init__(self, x=0, y=0, heading=0, mass=1.0, moment_inertia=0.1):
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading  # radians
        
        # Linear motion
        self.vx = 0.0  # velocity in x
        self.vy = 0.0  # velocity in y
        
        # Angular motion
        self.angular_velocity = 0.0  # rad/s
        
        # Physical properties
        self.mass = mass
        self.moment_inertia = moment_inertia
        
        # Drag coefficients (reduced for better performance)
        self.linear_drag = 0.05  # linear drag coefficient (less drag)
        self.angular_drag = 0.02  # angular drag coefficient (less drag)
        
        # Jet system
        self.jet_recharge_time = 2.0  # seconds
        self.last_jet_time = -10.0  # allow immediate first jet
        
        # History for plotting
        self.history_x = [x]
        self.history_y = [y]
        self.history_heading = [heading]
        self.jet_events = []
    
    def can_fire_jet(self, current_time):
        """Check if enough time has passed since last jet"""
        return (current_time - self.last_jet_time) >= self.jet_recharge_time
    
    def fire_jet(self, thrust_magnitude, nozzle_angle, current_time, duration=0.5):
        """
        Fire jet with given thrust and nozzle angle
        
        thrust_magnitude: Force magnitude [N]
        nozzle_angle: Angle of nozzle relative to body axis [rad]
                     0 = straight back (forward thrust)
                     +angle = nozzle points right (turn left)
                     -angle = nozzle points left (turn right)
        """
        if not self.can_fire_jet(current_time):
            return False
        
        # Thrust direction in world coordinates
        thrust_world_angle = self.heading + nozzle_angle + math.pi  # +pi because thrust is opposite to nozzle
        
        # Linear thrust (Newton's 2nd law: F = ma)
        thrust_x = thrust_magnitude * math.cos(thrust_world_angle)
        thrust_y = thrust_magnitude * math.sin(thrust_world_angle)
        
        # Apply linear impulse
        impulse_x = thrust_x * duration
        impulse_y = thrust_y * duration
        
        self.vx += impulse_x / self.mass
        self.vy += impulse_y / self.mass
        
        # Torque from off-center thrust (if nozzle is angled)
        # Assume nozzle is at rear of vehicle (distance = 0.5m from center)
        lever_arm = 0.5
        torque = thrust_magnitude * math.sin(nozzle_angle) * lever_arm
        
        # Apply angular impulse (τ = I * α)
        angular_impulse = torque * duration
        self.angular_velocity += angular_impulse / self.moment_inertia
        
        # Record jet event
        self.jet_events.append({
            'time': current_time,
            'position': (self.x, self.y),
            'thrust_magnitude': thrust_magnitude,
            'nozzle_angle': nozzle_angle,
            'heading': self.heading
        })
        
        self.last_jet_time = current_time
        return True
    
    def update_physics(self, dt):
        """Update physics for one time step"""
        
        # Apply drag forces
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > 0:
            drag_force = self.linear_drag * speed**2
            drag_x = -drag_force * (self.vx / speed)
            drag_y = -drag_force * (self.vy / speed)
            
            # Apply drag acceleration
            self.vx += drag_x * dt / self.mass
            self.vy += drag_y * dt / self.mass
        
        # Apply angular drag
        angular_drag_torque = -self.angular_drag * self.angular_velocity**2 * np.sign(self.angular_velocity)
        self.angular_velocity += angular_drag_torque * dt / self.moment_inertia
        
        # Update position and orientation
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.heading += self.angular_velocity * dt
        
        # Normalize heading
        while self.heading > math.pi:
            self.heading -= 2 * math.pi
        while self.heading < -math.pi:
            self.heading += 2 * math.pi
        
        # Record history
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_heading.append(self.heading)

def plan_rocket_jellyfish_path(start_x, start_y, start_heading, goal_x, goal_y, goal_heading):
    """
    Plan a path for rocket jellyfish using Dubins path as reference trajectory
    """
    
    # Get optimal Dubins path as reference
    from dubins import plan_dubins_path
    curvature = 0.5
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_heading, goal_x, goal_y, goal_heading, curvature)
    
    if x_dubins is None:
        print("❌ Failed to generate Dubins reference path")
        return None
    
    print(f"🚀 Planning rocket jellyfish path using {mode_dubins} Dubins reference...")
    print(f"Start: ({start_x:.1f}, {start_y:.1f}) heading {math.degrees(start_heading):.0f}°")
    print(f"Goal: ({goal_x:.1f}, {goal_y:.1f}) heading {math.degrees(goal_heading):.0f}°")
    print(f"Reference path length: {sum(lengths):.2f}m")
    
    # Create jellyfish
    jellyfish = RocketJellyfish(start_x, start_y, start_heading)
    
    # Simulation parameters
    dt = 0.1  # time step
    max_time = 30.0  # increased time limit
    current_time = 0.0
    
    # Control parameters
    position_tolerance = 1.0  # meters
    heading_tolerance = 0.3  # radians
    path_following_distance = 3.0  # how close to follow Dubins path
    
    # Track progress along Dubins path
    dubins_index = 0
    
    while current_time < max_time:
        # Check if we've reached the goal
        distance_to_goal = math.sqrt((jellyfish.x - goal_x)**2 + (jellyfish.y - goal_y)**2)
        heading_error = goal_heading - jellyfish.heading
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        if distance_to_goal < position_tolerance and abs(heading_error) < heading_tolerance:
            print(f"✅ Goal reached at t={current_time:.1f}s")
            print(f"   Final position: ({jellyfish.x:.2f}, {jellyfish.y:.2f})")
            print(f"   Final heading: {math.degrees(jellyfish.heading):.1f}°")
            print(f"   Position error: {distance_to_goal:.2f}m")
            print(f"   Heading error: {math.degrees(abs(heading_error)):.1f}°")
            break
        
        # Find closest point on Dubins path for reference
        min_dist = float('inf')
        closest_idx = dubins_index
        
        # Search ahead on the path
        search_range = min(50, len(x_dubins) - dubins_index)
        for i in range(dubins_index, dubins_index + search_range):
            if i >= len(x_dubins):
                break
            dist = math.sqrt((jellyfish.x - x_dubins[i])**2 + (jellyfish.y - y_dubins[i])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Update tracking index (move forward along path)
        dubins_index = max(dubins_index, closest_idx)
        
        # Look ahead on Dubins path for target point
        lookahead_distance = 4.0  # meters
        target_idx = dubins_index
        
        for i in range(dubins_index, len(x_dubins)):
            path_dist = math.sqrt((x_dubins[i] - x_dubins[dubins_index])**2 + 
                                (y_dubins[i] - y_dubins[dubins_index])**2)
            if path_dist >= lookahead_distance:
                target_idx = i
                break
        else:
            target_idx = len(x_dubins) - 1  # Use goal if near end
        
        # Target point on Dubins path
        target_x = x_dubins[target_idx]
        target_y = y_dubins[target_idx]
        target_heading = yaw_dubins[target_idx]
        
        # Control strategy using Dubins path guidance
        if jellyfish.can_fire_jet(current_time):
            distance_to_target = math.sqrt((jellyfish.x - target_x)**2 + (jellyfish.y - target_y)**2)
            
            print(f"   t={current_time:.1f}s: Following Dubins path, target at ({target_x:.1f}, {target_y:.1f}), dist={distance_to_target:.1f}m")
            
            # Calculate desired thrust direction toward Dubins path target
            direction_to_target = math.atan2(target_y - jellyfish.y, target_x - jellyfish.x)
            
            # Determine thrust magnitude based on distance and progress
            if distance_to_goal > 5.0:
                thrust_magnitude = 10.0  # Strong thrust when far
            elif distance_to_goal > 2.0:
                thrust_magnitude = 7.0   # Medium thrust
            else:
                thrust_magnitude = 5.0   # Gentle thrust when close
            
            # Calculate nozzle angle for desired thrust direction
            desired_thrust_angle = direction_to_target
            nozzle_angle = desired_thrust_angle - jellyfish.heading - math.pi
            
            # Normalize nozzle angle
            while nozzle_angle > math.pi:
                nozzle_angle -= 2 * math.pi
            while nozzle_angle < -math.pi:
                nozzle_angle += 2 * math.pi
            
            # Add heading correction component
            heading_to_target = target_heading - jellyfish.heading
            while heading_to_target > math.pi:
                heading_to_target -= 2 * math.pi
            while heading_to_target < -math.pi:
                heading_to_target += 2 * math.pi
            
            # Blend position and heading control
            if distance_to_target > 2.0:
                # Focus on position when far from path
                position_weight = 0.8
                heading_weight = 0.2
            else:
                # Focus more on heading when close to path
                position_weight = 0.6
                heading_weight = 0.4
            
            # Heading correction component
            heading_correction = heading_weight * heading_to_target * 0.5
            nozzle_angle = position_weight * nozzle_angle + heading_correction
            
            # Limit nozzle angle (realistic constraint)
            max_nozzle_angle = math.pi/2  # 90 degrees max
            nozzle_angle = max(-max_nozzle_angle, min(max_nozzle_angle, nozzle_angle))
            
            jellyfish.fire_jet(thrust_magnitude, nozzle_angle, current_time)
        
        # Update physics
        jellyfish.update_physics(dt)
        current_time += dt
    
    if current_time >= max_time:
        print(f"⏰ Time limit reached at t={current_time:.1f}s")
        print(f"   Final position: ({jellyfish.x:.2f}, {jellyfish.y:.2f})")
        print(f"   Distance to goal: {distance_to_goal:.2f}m")
    
    return jellyfish

def compare_dubins_vs_rocket():
    """Compare traditional Dubins path vs rocket jellyfish"""
    
    # Test case
    start_x, start_y, start_heading = 0.0, 0.0, 0.0
    goal_x, goal_y, goal_heading = 10.0, 5.0, math.pi/2
    
    # Plan rocket jellyfish path
    jellyfish = plan_rocket_jellyfish_path(start_x, start_y, start_heading, 
                                          goal_x, goal_y, goal_heading)
    
    # Get traditional Dubins path for comparison
    from dubins import plan_dubins_path
    curvature = 0.5
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_heading, goal_x, goal_y, goal_heading, curvature)
    
    # Plot comparison
    plt.figure(figsize=(15, 10))
    
    # Path comparison
    plt.subplot(2, 2, 1)
    plt.plot(x_dubins, y_dubins, 'b-', linewidth=2, label='Dubins Path')
    plt.plot(jellyfish.history_x, jellyfish.history_y, 'r-', linewidth=2, label='Rocket Jellyfish')
    
    # Mark jet events
    for i, event in enumerate(jellyfish.jet_events):
        plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8, alpha=0.7)
        if i == 0:
            plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8, alpha=0.7, label='Jet Events')
    
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(goal_x, goal_y, 'ko', markersize=10, label='Goal')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Path Comparison')
    
    # Velocity over time
    plt.subplot(2, 2, 2)
    times = np.arange(len(jellyfish.history_x)) * 0.1
    velocities = []
    for i in range(1, len(jellyfish.history_x)):
        dx = jellyfish.history_x[i] - jellyfish.history_x[i-1]
        dy = jellyfish.history_y[i] - jellyfish.history_y[i-1]
        v = math.sqrt(dx**2 + dy**2) / 0.1
        velocities.append(v)
    
    plt.plot(times[1:], velocities, 'r-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Over Time')
    plt.grid(True, alpha=0.3)
    
    # Mark jet times
    for event in jellyfish.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    # Heading over time
    plt.subplot(2, 2, 3)
    headings_deg = [math.degrees(h) for h in jellyfish.history_heading]
    plt.plot(times, headings_deg, 'g-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Heading Over Time')
    plt.grid(True, alpha=0.3)
    
    # Mark jet times
    for event in jellyfish.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    # Jet event details
    plt.subplot(2, 2, 4)
    plt.axis('off')
    
    text_info = "🚀 JET EVENTS:\n\n"
    for i, event in enumerate(jellyfish.jet_events):
        nozzle_deg = math.degrees(event['nozzle_angle'])
        heading_deg = math.degrees(event['heading'])
        text_info += f"Jet {i+1}: t={event['time']:.1f}s\n"
        text_info += f"  Position: ({event['position'][0]:.1f}, {event['position'][1]:.1f})\n"
        text_info += f"  Thrust: {event['thrust_magnitude']:.1f}N\n"
        text_info += f"  Nozzle angle: {nozzle_deg:+.1f}°\n"
        text_info += f"  Body heading: {heading_deg:.1f}°\n\n"
    
    plt.text(0.1, 0.9, text_info, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.savefig('rocket_jellyfish_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"\n📊 RESULTS:")
    print(f"Dubins path length: {sum(lengths):.2f}m")
    print(f"Rocket jellyfish path length: {sum(math.sqrt((jellyfish.history_x[i+1]-jellyfish.history_x[i])**2 + (jellyfish.history_y[i+1]-jellyfish.history_y[i])**2) for i in range(len(jellyfish.history_x)-1)):.2f}m")
    print(f"Number of jet pulses: {len(jellyfish.jet_events)}")
    print(f"Total mission time: {len(jellyfish.history_x) * 0.1:.1f}s")

if __name__ == "__main__":
    compare_dubins_vs_rocket()
