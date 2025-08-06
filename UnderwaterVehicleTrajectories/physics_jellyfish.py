"""
Physics-Based Jellyfish Trajectory Planning
Uses proper orbital mechanics and physics calculations for jet propulsion
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from dubins import plan_dubins_path

class PhysicsJellyfish:
    """
    Physics-based jellyfish with proper trajectory calculations
    """
    
    def __init__(self, x=0, y=0, heading=0, mass=1.0):
        # State
        self.x = x
        self.y = y
        self.heading = heading
        self.vx = 0.0
        self.vy = 0.0
        self.angular_velocity = 0.0
        
        # Physical properties
        self.mass = mass
        self.moment_inertia = 0.1
        self.drag_coeff = 0.02
        
        # Jet system
        self.max_thrust = 10.0  # N
        self.max_nozzle_angle = math.pi/3  # 60 degrees
        self.jet_duration = 0.5  # seconds
        self.recharge_time = 2.0  # seconds
        self.last_jet_time = -10.0
        
        # History
        self.history = [(x, y, heading, 0.0)]
        self.jet_events = []
    
    def can_fire_jet(self, time):
        return (time - self.last_jet_time) >= self.recharge_time
    
    def calculate_required_impulse(self, target_x, target_y, target_vx=0, target_vy=0):
        """
        Calculate the impulse needed to reach target position with target velocity
        Using basic orbital mechanics principles
        """
        # Position difference
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Velocity difference needed
        dvx = target_vx - self.vx
        dvy = target_vy - self.vy
        
        # Required impulse (momentum change)
        impulse_x = self.mass * dvx
        impulse_y = self.mass * dvy
        
        return impulse_x, impulse_y
    
    def calculate_jet_parameters(self, impulse_x, impulse_y):
        """
        Calculate thrust magnitude and nozzle angle for desired impulse
        """
        # Total impulse magnitude
        impulse_mag = math.sqrt(impulse_x**2 + impulse_y**2)
        
        # Required thrust magnitude
        thrust_mag = min(impulse_mag / self.jet_duration, self.max_thrust)
        
        # Desired thrust direction (world coordinates)
        thrust_angle = math.atan2(impulse_y, impulse_x)
        
        # Convert to nozzle angle (relative to body, opposite direction)
        nozzle_angle = thrust_angle - self.heading - math.pi
        
        # Normalize angle
        while nozzle_angle > math.pi:
            nozzle_angle -= 2 * math.pi
        while nozzle_angle < -math.pi:
            nozzle_angle += 2 * math.pi
        
        # Limit nozzle angle
        nozzle_angle = max(-self.max_nozzle_angle, min(self.max_nozzle_angle, nozzle_angle))
        
        return thrust_mag, nozzle_angle
    
    def fire_jet(self, thrust_mag, nozzle_angle, time):
        """Fire jet with calculated parameters"""
        if not self.can_fire_jet(time):
            return False
        
        # Thrust direction in world coordinates
        thrust_world_angle = self.heading + nozzle_angle + math.pi
        
        # Apply impulse
        impulse_x = thrust_mag * self.jet_duration * math.cos(thrust_world_angle)
        impulse_y = thrust_mag * self.jet_duration * math.sin(thrust_world_angle)
        
        self.vx += impulse_x / self.mass
        self.vy += impulse_y / self.mass
        
        # Angular impulse from off-center thrust
        lever_arm = 0.5
        torque = thrust_mag * math.sin(nozzle_angle) * lever_arm
        angular_impulse = torque * self.jet_duration
        self.angular_velocity += angular_impulse / self.moment_inertia
        
        # Record event
        self.jet_events.append({
            'time': time,
            'position': (self.x, self.y),
            'thrust': thrust_mag,
            'nozzle_angle': nozzle_angle,
            'impulse': (impulse_x, impulse_y)
        })
        
        self.last_jet_time = time
        return True
    
    def predict_trajectory(self, time_horizon):
        """Predict where jellyfish will be after time_horizon with current velocity"""
        # Account for drag
        drag_factor = math.exp(-self.drag_coeff * time_horizon)
        
        # Predicted position
        pred_x = self.x + self.vx * time_horizon * drag_factor
        pred_y = self.y + self.vy * time_horizon * drag_factor
        
        # Predicted velocity
        pred_vx = self.vx * drag_factor
        pred_vy = self.vy * drag_factor
        
        return pred_x, pred_y, pred_vx, pred_vy
    
    def update_physics(self, dt):
        """Update physics with drag"""
        # Apply drag
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > 0:
            drag_force = self.drag_coeff * speed
            self.vx -= drag_force * (self.vx / speed) * dt
            self.vy -= drag_force * (self.vy / speed) * dt
        
        # Angular drag
        self.angular_velocity *= (1 - self.drag_coeff * dt)
        
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
        speed = math.sqrt(self.vx**2 + self.vy**2)
        self.history.append((self.x, self.y, self.heading, speed))

def plan_physics_jellyfish_path(start_x, start_y, start_heading, goal_x, goal_y, goal_heading):
    """
    Plan path using physics-based calculations and Dubins reference
    """
    
    # Get Dubins reference path
    curvature = 0.5
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_heading, goal_x, goal_y, goal_heading, curvature)
    
    if x_dubins is None:
        return None
    
    print(f"🧮 Physics-based jellyfish planning using {mode_dubins} reference")
    print(f"Start: ({start_x:.1f}, {start_y:.1f}) @ {math.degrees(start_heading):.0f}°")
    print(f"Goal: ({goal_x:.1f}, {goal_y:.1f}) @ {math.degrees(goal_heading):.0f}°")
    
    # Create jellyfish
    jellyfish = PhysicsJellyfish(start_x, start_y, start_heading)
    
    # Simulation parameters
    dt = 0.1
    max_time = 25.0
    current_time = 0.0
    
    # Control parameters
    position_tolerance = 0.8
    heading_tolerance = 0.2
    
    # Plan waypoints along Dubins path
    waypoints = []
    waypoint_spacing = 3.0  # meters between waypoints
    
    current_dist = 0
    for i in range(1, len(x_dubins)):
        segment_dist = math.sqrt((x_dubins[i] - x_dubins[i-1])**2 + (y_dubins[i] - y_dubins[i-1])**2)
        current_dist += segment_dist
        
        if current_dist >= waypoint_spacing or i == len(x_dubins) - 1:
            waypoints.append({
                'x': x_dubins[i],
                'y': y_dubins[i],
                'heading': yaw_dubins[i],
                'index': i
            })
            current_dist = 0
    
    print(f"Generated {len(waypoints)} waypoints along Dubins path")
    
    current_waypoint = 0
    
    while current_time < max_time and current_waypoint < len(waypoints):
        # Current target waypoint
        target = waypoints[current_waypoint]
        
        # Check if reached current waypoint
        dist_to_waypoint = math.sqrt((jellyfish.x - target['x'])**2 + (jellyfish.y - target['y'])**2)
        
        if dist_to_waypoint < 2.0:  # Close enough to waypoint
            current_waypoint += 1
            if current_waypoint >= len(waypoints):
                break
            print(f"   Reached waypoint {current_waypoint}, moving to next")
            continue
        
        # Physics-based control
        if jellyfish.can_fire_jet(current_time):
            # Predict where we'll be when next jet is available
            next_jet_time = current_time + jellyfish.recharge_time
            pred_x, pred_y, pred_vx, pred_vy = jellyfish.predict_trajectory(jellyfish.recharge_time)
            
            # Calculate required velocity to reach target from predicted position
            time_to_target = 3.0  # seconds to reach target
            required_vx = (target['x'] - pred_x) / time_to_target
            required_vy = (target['y'] - pred_y) / time_to_target
            
            # Calculate required impulse
            impulse_x, impulse_y = jellyfish.calculate_required_impulse(
                pred_x, pred_y, required_vx, required_vy)
            
            # Calculate jet parameters
            thrust_mag, nozzle_angle = jellyfish.calculate_jet_parameters(impulse_x, impulse_y)
            
            print(f"   t={current_time:.1f}s: Firing jet toward waypoint {current_waypoint+1}")
            print(f"      Target: ({target['x']:.1f}, {target['y']:.1f}), dist={dist_to_waypoint:.1f}m")
            print(f"      Thrust: {thrust_mag:.1f}N, nozzle: {math.degrees(nozzle_angle):+.1f}°")
            
            jellyfish.fire_jet(thrust_mag, nozzle_angle, current_time)
        
        # Update physics
        jellyfish.update_physics(dt)
        current_time += dt
    
    # Check final goal
    final_dist = math.sqrt((jellyfish.x - goal_x)**2 + (jellyfish.y - goal_y)**2)
    final_heading_error = abs(goal_heading - jellyfish.heading)
    
    if final_dist < position_tolerance and final_heading_error < heading_tolerance:
        print(f"✅ Goal reached! Final error: {final_dist:.2f}m, {math.degrees(final_heading_error):.1f}°")
    else:
        print(f"⚠️  Close to goal: {final_dist:.2f}m error, {math.degrees(final_heading_error):.1f}° heading error")
    
    return jellyfish, x_dubins, y_dubins

def compare_physics_approach():
    """Compare physics-based approach"""
    
    # Test case
    start_x, start_y, start_heading = 0.0, 0.0, 0.0
    goal_x, goal_y, goal_heading = 10.0, 5.0, math.pi/2
    
    # Plan path
    jellyfish, x_dubins, y_dubins = plan_physics_jellyfish_path(
        start_x, start_y, start_heading, goal_x, goal_y, goal_heading)
    
    if jellyfish is None:
        print("❌ Planning failed")
        return
    
    # Extract history
    history = np.array(jellyfish.history)
    x_history = history[:, 0]
    y_history = history[:, 1]
    heading_history = history[:, 2]
    speed_history = history[:, 3]
    
    # Plot results
    plt.figure(figsize=(15, 10))
    
    # Path comparison
    plt.subplot(2, 2, 1)
    plt.plot(x_dubins, y_dubins, 'b-', linewidth=2, alpha=0.7, label='Dubins Reference')
    plt.plot(x_history, y_history, 'r-', linewidth=2, label='Physics Jellyfish')
    
    # Mark jet events
    for i, event in enumerate(jellyfish.jet_events):
        plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8)
        if i == 0:
            plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8, label='Jet Events')
    
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(goal_x, goal_y, 'ko', markersize=10, label='Goal')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Physics-Based Jellyfish Path')
    
    # Speed over time
    plt.subplot(2, 2, 2)
    times = np.arange(len(speed_history)) * 0.1
    plt.plot(times, speed_history, 'r-', linewidth=2)
    
    # Mark jet times
    for event in jellyfish.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Profile')
    plt.grid(True, alpha=0.3)
    
    # Heading over time
    plt.subplot(2, 2, 3)
    heading_deg = np.degrees(heading_history)
    plt.plot(times, heading_deg, 'g-', linewidth=2)
    
    # Mark jet times
    for event in jellyfish.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Heading Evolution')
    plt.grid(True, alpha=0.3)
    
    # Performance metrics
    plt.subplot(2, 2, 4)
    plt.axis('off')
    
    # Calculate path length
    path_length = sum(math.sqrt((x_history[i+1] - x_history[i])**2 + (y_history[i+1] - y_history[i])**2) 
                     for i in range(len(x_history)-1))
    
    dubins_length = sum(math.sqrt((x_dubins[i+1] - x_dubins[i])**2 + (y_dubins[i+1] - y_dubins[i])**2) 
                       for i in range(len(x_dubins)-1))
    
    info_text = f"""📊 PERFORMANCE METRICS

Dubins Reference:
  Length: {dubins_length:.2f}m
  
Physics Jellyfish:
  Path length: {path_length:.2f}m
  Mission time: {len(x_history) * 0.1:.1f}s
  Jet pulses: {len(jellyfish.jet_events)}
  Efficiency: {path_length/dubins_length:.1f}x reference
  
Final State:
  Position: ({x_history[-1]:.2f}, {y_history[-1]:.2f})
  Heading: {math.degrees(heading_history[-1]):.1f}°
  Speed: {speed_history[-1]:.2f} m/s
"""
    
    plt.text(0.1, 0.9, info_text, transform=plt.gca().transAxes, 
             fontsize=11, verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.savefig('physics_jellyfish_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"\n📊 FINAL RESULTS:")
    print(f"Dubins reference: {dubins_length:.2f}m")
    print(f"Physics jellyfish: {path_length:.2f}m ({path_length/dubins_length:.1f}x)")
    print(f"Jet pulses: {len(jellyfish.jet_events)}")
    print(f"Mission time: {len(x_history) * 0.1:.1f}s")

if __name__ == "__main__":
    compare_physics_approach()
