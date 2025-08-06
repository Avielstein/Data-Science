"""
Bio-Inspired Jet Swimmer with Rear Nozzle Physics
Proper momentum-based simulation with realistic nozzle constraints
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from dubins import plan_dubins_path

class JetSwimmer:
    """
    Bio-inspired swimmer with rear nozzle
    - Nozzle can only point backward (180° range)
    - Cannot slow down, only accelerate or turn
    - Momentum-based physics between jets
    """
    
    def __init__(self, x=0, y=0, heading=0):
        # Position and orientation
        self.x = x
        self.y = y
        self.heading = heading  # radians
        
        # Velocity (momentum)
        self.vx = 0.0
        self.vy = 0.0
        
        # Physical properties
        self.mass = 1.0
        self.drag_coeff = 0.05  # water drag
        
        # Jet system constraints
        self.max_nozzle_angle = math.pi/2  # ±90° from straight back
        self.jet_thrust = 5.0  # N
        self.jet_duration = 0.3  # seconds
        self.recharge_time = 2.0  # seconds between jets
        self.last_jet_time = -10.0  # allow immediate first jet
        
        # History
        self.history = [(x, y, heading, 0.0)]  # (x, y, heading, speed)
        self.jet_events = []
    
    def can_fire_jet(self, time):
        """Check if jet can fire (recharge time elapsed)"""
        return (time - self.last_jet_time) >= self.recharge_time
    
    def get_nozzle_world_angle(self, nozzle_angle):
        """Convert nozzle angle to world coordinates"""
        # Nozzle angle is relative to rear of vehicle
        # 0° = straight back, +angle = nozzle points right, -angle = nozzle points left
        return self.heading + math.pi + nozzle_angle
    
    def fire_jet(self, nozzle_angle, time):
        """
        Fire jet with given nozzle angle
        nozzle_angle: angle relative to rear of vehicle [-π/2, +π/2]
        """
        if not self.can_fire_jet(time):
            return False
        
        # Clamp nozzle angle to realistic range
        nozzle_angle = max(-self.max_nozzle_angle, min(self.max_nozzle_angle, nozzle_angle))
        
        # Thrust direction (opposite to nozzle direction)
        thrust_world_angle = self.get_nozzle_world_angle(nozzle_angle) + math.pi
        
        # Apply impulse (change in momentum)
        impulse_x = self.jet_thrust * self.jet_duration * math.cos(thrust_world_angle)
        impulse_y = self.jet_thrust * self.jet_duration * math.sin(thrust_world_angle)
        
        self.vx += impulse_x / self.mass
        self.vy += impulse_y / self.mass
        
        # Record jet event
        self.jet_events.append({
            'time': time,
            'position': (self.x, self.y),
            'nozzle_angle': nozzle_angle,
            'thrust_direction': thrust_world_angle,
            'velocity_after': (self.vx, self.vy)
        })
        
        self.last_jet_time = time
        return True
    
    def update_physics(self, dt):
        """Update physics - momentum and drag"""
        # Apply drag
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > 0:
            drag_force = self.drag_coeff * speed
            drag_x = -drag_force * (self.vx / speed) * dt
            drag_y = -drag_force * (self.vy / speed) * dt
            
            self.vx += drag_x
            self.vy += drag_y
        
        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Update heading based on velocity direction (like a rocket)
        if speed > 0.1:  # Only update heading if moving
            self.heading = math.atan2(self.vy, self.vx)
        
        # Record history
        self.history.append((self.x, self.y, self.heading, speed))

def plan_jet_swimmer_path(start_x, start_y, start_heading, goal_x, goal_y, goal_heading):
    """
    Plan path for jet swimmer using physics-based approach
    """
    
    print(f"🌊 JET SWIMMER PLANNING")
    print(f"Start: ({start_x:.1f}, {start_y:.1f}) facing {math.degrees(start_heading):.0f}°")
    print(f"Goal: ({goal_x:.1f}, {goal_y:.1f}) facing {math.degrees(goal_heading):.0f}°")
    
    # Create swimmer
    swimmer = JetSwimmer(start_x, start_y, start_heading)
    
    # Simulation parameters
    dt = 0.1  # time step
    max_time = 20.0  # maximum simulation time
    current_time = 0.0
    
    # Control parameters
    position_tolerance = 1.0  # meters
    heading_tolerance = 0.3  # radians
    
    while current_time < max_time:
        # Check if reached goal
        distance_to_goal = math.sqrt((swimmer.x - goal_x)**2 + (swimmer.y - goal_y)**2)
        heading_error = abs(goal_heading - swimmer.heading)
        
        if distance_to_goal < position_tolerance and heading_error < heading_tolerance:
            print(f"✅ Goal reached at t={current_time:.1f}s")
            break
        
        # Control logic
        if swimmer.can_fire_jet(current_time):
            # Calculate desired direction to goal
            direction_to_goal = math.atan2(goal_y - swimmer.y, goal_x - swimmer.x)
            current_speed = math.sqrt(swimmer.vx**2 + swimmer.vy**2)
            
            if current_speed < 0.5:
                # Low speed - fire jet toward goal
                # We want thrust in direction_to_goal
                # Nozzle points opposite to thrust direction
                # Nozzle angle is relative to rear of vehicle (heading + π)
                nozzle_world_angle = direction_to_goal + math.pi  # Point nozzle opposite to desired thrust
                nozzle_angle = nozzle_world_angle - (swimmer.heading + math.pi)
                
                # Normalize nozzle angle
                while nozzle_angle > math.pi:
                    nozzle_angle -= 2 * math.pi
                while nozzle_angle < -math.pi:
                    nozzle_angle += 2 * math.pi
                
                print(f"   t={current_time:.1f}s: LOW SPEED - thrust toward goal")
                print(f"      Distance: {distance_to_goal:.1f}m, desired thrust: {math.degrees(direction_to_goal):+.1f}°")
                print(f"      Nozzle angle: {math.degrees(nozzle_angle):+.1f}°")
                
            else:
                # Moving - calculate course correction
                current_direction = math.atan2(swimmer.vy, swimmer.vx)
                direction_error = direction_to_goal - current_direction
                
                # Normalize direction error
                while direction_error > math.pi:
                    direction_error -= 2 * math.pi
                while direction_error < -math.pi:
                    direction_error += 2 * math.pi
                
                # Small course correction - use nozzle to nudge direction
                # Positive direction_error means we need to turn left (counterclockwise)
                # Negative direction_error means we need to turn right (clockwise)
                nozzle_angle = -direction_error * 0.3  # Proportional control (negative because nozzle is opposite)
                
                print(f"   t={current_time:.1f}s: COURSE CORRECTION")
                print(f"      Speed: {current_speed:.1f}m/s, current dir: {math.degrees(current_direction):+.1f}°")
                print(f"      Desired dir: {math.degrees(direction_to_goal):+.1f}°, error: {math.degrees(direction_error):+.1f}°")
                print(f"      Nozzle angle: {math.degrees(nozzle_angle):+.1f}°")
            
            # Fire jet
            swimmer.fire_jet(nozzle_angle, current_time)
        
        # Update physics
        swimmer.update_physics(dt)
        current_time += dt
    
    if current_time >= max_time:
        print(f"⏰ Time limit reached")
        print(f"   Final position: ({swimmer.x:.2f}, {swimmer.y:.2f})")
        print(f"   Distance to goal: {distance_to_goal:.2f}m")
    
    return swimmer

def compare_dubins_vs_jet_swimmer():
    """Compare traditional Dubins vs jet swimmer"""
    
    # Test case
    start_x, start_y, start_heading = 0.0, 0.0, 0.0
    goal_x, goal_y, goal_heading = 10.0, 5.0, math.pi/2
    curvature = 0.5
    
    print("🌊 DUBINS vs JET SWIMMER COMPARISON")
    print("=" * 50)
    print(f"Start: ({start_x}, {start_y}) facing {math.degrees(start_heading):.0f}°")
    print(f"End: ({goal_x}, {goal_y}) facing {math.degrees(goal_heading):.0f}°")
    print(f"Turning radius: {1/curvature:.1f}m")
    print()
    
    # Plan Dubins path
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_heading, goal_x, goal_y, goal_heading, curvature)
    
    # Plan jet swimmer path
    swimmer = plan_jet_swimmer_path(start_x, start_y, start_heading, goal_x, goal_y, goal_heading)
    
    if x_dubins is None or swimmer is None:
        print("❌ Path planning failed")
        return
    
    # Extract swimmer history
    history = np.array(swimmer.history)
    x_swimmer = history[:, 0]
    y_swimmer = history[:, 1]
    heading_swimmer = history[:, 2]
    speed_swimmer = history[:, 3]
    
    # Calculate path lengths
    dubins_length = sum(lengths)
    swimmer_length = sum(math.sqrt((x_swimmer[i+1] - x_swimmer[i])**2 + (y_swimmer[i+1] - y_swimmer[i])**2) 
                         for i in range(len(x_swimmer)-1))
    
    # Plot comparison
    plt.figure(figsize=(15, 10))
    
    # Path comparison
    plt.subplot(2, 2, 1)
    plt.plot(x_dubins, y_dubins, 'b-', linewidth=2, label='Dubins Path')
    plt.plot(x_swimmer, y_swimmer, 'r-', linewidth=2, label='Jet Swimmer')
    
    # Mark jet events
    for i, event in enumerate(swimmer.jet_events):
        plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8)
        if i == 0:
            plt.plot(event['position'][0], event['position'][1], 'ro', markersize=8, label='Jet Events')
        
        # Show nozzle direction
        x, y = event['position']
        nozzle_world_angle = swimmer.get_nozzle_world_angle(event['nozzle_angle'])
        dx = 0.5 * math.cos(nozzle_world_angle)
        dy = 0.5 * math.sin(nozzle_world_angle)
        plt.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc='orange', ec='orange', alpha=0.7)
    
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(goal_x, goal_y, 'ko', markersize=10, label='Goal')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Path Comparison')
    
    # Speed over time
    plt.subplot(2, 2, 2)
    times = np.arange(len(speed_swimmer)) * 0.1
    plt.plot(times, speed_swimmer, 'r-', linewidth=2)
    
    # Mark jet times
    for event in swimmer.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed Profile')
    plt.grid(True, alpha=0.3)
    
    # Heading over time
    plt.subplot(2, 2, 3)
    heading_deg = np.degrees(heading_swimmer)
    plt.plot(times, heading_deg, 'g-', linewidth=2)
    
    # Mark jet times
    for event in swimmer.jet_events:
        plt.axvline(event['time'], color='red', alpha=0.5, linestyle='--')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Heading Evolution')
    plt.grid(True, alpha=0.3)
    
    # Results summary
    plt.subplot(2, 2, 4)
    plt.axis('off')
    
    results_text = f"""📊 RESULTS:

Traditional Dubins ({mode_dubins}):
  Path length: {dubins_length:.2f}m
  Planning time: <1ms
  
Jet Swimmer:
  Path length: {swimmer_length:.2f}m
  Mission time: {len(swimmer.history) * 0.1:.1f}s
  Jet pulses: {len(swimmer.jet_events)}
  Efficiency: {swimmer_length/dubins_length:.1f}x reference

🚀 JET EVENTS:
"""
    
    for i, event in enumerate(swimmer.jet_events):
        nozzle_deg = math.degrees(event['nozzle_angle'])
        if abs(nozzle_deg) < 5:
            direction = "Straight back"
        elif nozzle_deg > 0:
            direction = f"Right {nozzle_deg:.1f}°"
        else:
            direction = f"Left {abs(nozzle_deg):.1f}°"
        
        results_text += f"  Jet {i+1}: t={event['time']:.1f}s - {direction}\n"
    
    plt.text(0.1, 0.9, results_text, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.savefig('jet_swimmer_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"\n📊 FINAL RESULTS:")
    print(f"Dubins path: {dubins_length:.2f}m")
    print(f"Jet swimmer: {swimmer_length:.2f}m ({swimmer_length/dubins_length:.1f}x)")
    print(f"Jet pulses: {len(swimmer.jet_events)}")
    print(f"Mission time: {len(swimmer.history) * 0.1:.1f}s")
    print("\n✅ Comparison complete!")

# For compatibility with animation
def plan_bio_inspired_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature):
    """Compatibility function for animation"""
    swimmer = plan_jet_swimmer_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw)
    
    if swimmer is None:
        return None, None, None, None, None, None
    
    history = np.array(swimmer.history)
    x_path = history[:, 0]
    y_path = history[:, 1]
    yaw_path = history[:, 2]
    
    # Calculate energy cost (simplified)
    path_length = sum(math.sqrt((x_path[i+1] - x_path[i])**2 + (y_path[i+1] - y_path[i])**2) 
                     for i in range(len(x_path)-1))
    energy_cost = path_length + len(swimmer.jet_events) * 2.0  # base + jet costs
    
    return x_path, y_path, yaw_path, "JET", energy_cost, swimmer.jet_events

if __name__ == "__main__":
    compare_dubins_vs_jet_swimmer()
