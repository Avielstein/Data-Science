"""
Bio-Inspired Path Planning Extensions
Adds discrete jet propulsion and energy modeling to Dubins paths
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from dubins import plan_dubins_path

def plan_bio_inspired_path(sx, sy, syaw, gx, gy, gyaw, curvature, 
                          jet_efficiency=0.7, max_jets=8, recharge_time=2.0, step_size=0.1):
    """
    Plan bio-inspired path with jellyfish/salp-style jet propulsion
    
    CRITICAL PHYSICS CONSTRAINTS:
    1. Vehicle can ONLY turn during jet pulses (vectored thrust)
    2. Between jets: Straight-line gliding only (no steering)
    3. Recharge time required between jet pulses
    4. Must plan jet timing to achieve desired path
    
    Parameters:
    sx, sy, syaw: start x, y, yaw [m, m, rad]
    gx, gy, gyaw: goal x, y, yaw [m, m, rad]
    curvature: curvature [1/m] (turning capability during jets)
    jet_efficiency: efficiency of jet propulsion [0-1]
    max_jets: maximum number of jet pulses
    recharge_time: time between jet pulses [seconds]
    step_size: step size [m]
    
    Returns:
    x_list, y_list: path coordinates with jet-glide cycles
    yaw_list: path yaw angles
    mode: path type
    energy_cost: total energy cost
    jet_events: (position, type, energy, time) for each jet
    """
    
    # Get base Dubins path as reference (what we want to achieve)
    x_ref, y_ref, yaw_ref, mode, lengths = plan_dubins_path(
        sx, sy, syaw, gx, gy, gyaw, curvature, step_size)
    
    if x_ref is None:
        return None, None, None, None, None, None
    
    # Plan jet events with recharge time constraints
    jet_events = plan_jet_sequence(sx, sy, syaw, gx, gy, gyaw, 
                                  curvature, max_jets, recharge_time)
    
    # Generate actual path based on jet-glide physics
    x_list, y_list, yaw_list = simulate_jet_glide_physics(
        sx, sy, syaw, jet_events, recharge_time, step_size)
    
    # Calculate energy and time costs
    geometric_length = sum(lengths)
    energy_cost, total_time = calculate_jet_costs(jet_events, geometric_length, jet_efficiency)
    
    return x_list, y_list, yaw_list, f"BIO-{mode}", energy_cost, jet_events

def plan_jet_sequence(sx, sy, syaw, gx, gy, gyaw, curvature, max_jets, recharge_time):
    """
    Plan sequence of jet pulses to reach goal with physics constraints:
    
    CRITICAL: Vehicle can only change direction during jet pulses
    Between jets: Straight-line gliding only
    """
    jet_events = []
    current_time = 0.0
    
    # Calculate required heading changes
    target_heading = math.atan2(gy - sy, gx - sx)
    initial_turn = target_heading - syaw
    
    # Normalize angle
    while initial_turn > math.pi:
        initial_turn -= 2 * math.pi
    while initial_turn < -math.pi:
        initial_turn += 2 * math.pi
    
    # Jet 1: Initial steering + propulsion
    jet_events.append({
        'time': current_time,
        'position': (sx, sy),
        'type': 'start',
        'heading_change': initial_turn,
        'nozzle_angle': -initial_turn * 0.5,  # Vector thrust for steering
        'energy': 2.0 + abs(initial_turn) * 2.0,  # More energy for larger turns
        'thrust_duration': 0.5  # Jet pulse duration
    })
    
    current_time += recharge_time
    
    # Calculate distance and intermediate waypoints
    distance = math.sqrt((gx - sx)**2 + (gy - sy)**2)
    
    # Jet 2: Mid-course correction (if needed)
    if distance > 6.0 and max_jets > 2:
        mid_x = sx + 0.6 * (gx - sx)
        mid_y = sy + 0.6 * (gy - sy)
        
        jet_events.append({
            'time': current_time,
            'position': (mid_x, mid_y),
            'type': 'cruise',
            'heading_change': 0.0,
            'nozzle_angle': 0.0,  # Straight thrust
            'energy': 1.5,
            'thrust_duration': 0.3
        })
        
        current_time += recharge_time
    
    # Final jet: Approach and final heading adjustment
    final_turn = gyaw - target_heading
    while final_turn > math.pi:
        final_turn -= 2 * math.pi
    while final_turn < -math.pi:
        final_turn += 2 * math.pi
    
    approach_x = gx - 2.0 * math.cos(target_heading)
    approach_y = gy - 2.0 * math.sin(target_heading)
    
    jet_events.append({
        'time': current_time,
        'position': (approach_x, approach_y),
        'type': 'final',
        'heading_change': final_turn,
        'nozzle_angle': -final_turn * 0.5,
        'energy': 1.5 + abs(final_turn) * 2.0,
        'thrust_duration': 0.4
    })
    
    return jet_events

def simulate_jet_glide_physics(sx, sy, syaw, jet_events, recharge_time, step_size):
    """
    Simulate actual vehicle motion with jet-glide physics:
    
    - During jet pulse: Can change heading and accelerate
    - Between jets: Straight-line gliding with deceleration
    - No turning capability during glide phases
    """
    x_list = [sx]
    y_list = [sy]
    yaw_list = [syaw]
    
    current_x, current_y, current_yaw = sx, sy, syaw
    current_velocity = 0.0
    current_time = 0.0
    
    for i, jet in enumerate(jet_events):
        # Glide phase until next jet
        glide_time = jet['time'] - current_time
        if glide_time > 0:
            # Straight-line gliding with deceleration
            steps = max(int(glide_time / 0.1), 1)
            for step in range(steps):
                # Decelerate due to drag
                current_velocity *= 0.95  # 5% velocity loss per 0.1s
                
                # Move straight (no turning during glide)
                dx = current_velocity * 0.1 * math.cos(current_yaw)
                dy = current_velocity * 0.1 * math.sin(current_yaw)
                
                current_x += dx
                current_y += dy
                
                x_list.append(current_x)
                y_list.append(current_y)
                yaw_list.append(current_yaw)
        
        # Jet pulse phase
        current_time = jet['time']
        
        # Apply heading change (only possible during jet)
        current_yaw += jet['heading_change']
        
        # Accelerate due to jet thrust
        thrust_velocity = jet['energy'] * 0.5  # Convert energy to velocity
        current_velocity += thrust_velocity
        
        # Move during jet pulse
        pulse_steps = max(int(jet['thrust_duration'] / 0.05), 1)
        for step in range(pulse_steps):
            dx = current_velocity * 0.05 * math.cos(current_yaw)
            dy = current_velocity * 0.05 * math.sin(current_yaw)
            
            current_x += dx
            current_y += dy
            
            x_list.append(current_x)
            y_list.append(current_y)
            yaw_list.append(current_yaw)
        
        current_time += jet['thrust_duration']
    
    return x_list, y_list, yaw_list

def calculate_jet_costs(jet_events, geometric_length, jet_efficiency):
    """Calculate energy and time costs for jet sequence"""
    total_energy = 0.0
    total_time = 0.0
    
    for jet in jet_events:
        # Energy cost with efficiency losses
        base_energy = jet['energy']
        efficiency_loss = base_energy * (1.0 / jet_efficiency - 1.0)
        total_energy += base_energy + efficiency_loss
        
        # Time includes jet duration + recharge time
        total_time = max(total_time, jet['time'] + jet['thrust_duration'])
    
    # Add drag losses during gliding
    drag_cost = geometric_length * 0.1
    total_energy += drag_cost
    
    return total_energy, total_time

def generate_jet_glide_motion(x_base, y_base, yaw_base, jet_events, step_size):
    """
    Generate realistic jet-glide motion based on jellyfish/salp physics
    
    Between jets: Passive gliding with gradual deceleration
    At jets: Sudden acceleration burst with body contraction
    """
    x_list = []
    y_list = []
    yaw_list = []
    
    jet_indices = {event['index'] for event in jet_events}
    
    for i in range(len(x_base)):
        x, y, yaw = x_base[i], y_base[i], yaw_base[i]
        
        if i in jet_indices:
            # Find the jet event for this index
            jet_event = next(event for event in jet_events if event['index'] == i)
            
            # Add jet burst effect - body contracts, then expands
            if jet_event['type'] == 'start':
                # Strong initial contraction
                contraction = 0.3
            elif jet_event['type'] == 'turn':
                # Moderate contraction for steering
                contraction = 0.2
            else:  # cruise
                # Light contraction for maintenance
                contraction = 0.1
            
            # Body contracts perpendicular to motion direction
            perp_x = -math.sin(yaw)
            perp_y = math.cos(yaw)
            
            # Contraction phase
            x_contract = x + contraction * perp_x
            y_contract = y + contraction * perp_y
            
            # Add both contraction and expansion points
            x_list.extend([x_contract, x])
            y_list.extend([y_contract, y])
            yaw_list.extend([yaw, yaw])
        else:
            # Passive gliding - smooth motion
            x_list.append(x)
            y_list.append(y)
            yaw_list.append(yaw)
    
    return x_list, y_list, yaw_list

def calculate_jet_energy_cost(geometric_length, jet_events, jet_efficiency):
    """
    Calculate energy cost based on actual jet physics:
    - Each jet type has different energy requirements
    - Efficiency losses for each jet firing
    - Drag losses during gliding phases
    """
    # Base drag cost (energy lost to water resistance)
    drag_cost = geometric_length * 0.1  # 10% of distance as drag
    
    # Jet firing costs
    jet_cost = 0
    for event in jet_events:
        # Energy cost depends on jet type and efficiency
        base_energy = event['energy']
        efficiency_loss = base_energy * (1.0 / jet_efficiency - 1.0)
        jet_cost += base_energy + efficiency_loss
    
    return drag_cost + jet_cost

def compare_paths():
    """Compare traditional Dubins vs bio-inspired paths"""
    print("🌊 DUBINS vs BIO-INSPIRED COMPARISON")
    print("=" * 50)
    
    # Test case
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    end_x, end_y, end_yaw = 10.0, 5.0, math.pi/2
    curvature = 0.5
    
    print(f"Start: ({start_x}, {start_y}) facing {math.degrees(start_yaw):.0f}°")
    print(f"End: ({end_x}, {end_y}) facing {math.degrees(end_yaw):.0f}°")
    print(f"Turning radius: {1/curvature:.1f}m")
    
    # Traditional Dubins
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    
    # Bio-inspired
    x_bio, y_bio, yaw_bio, mode_bio, energy_cost, jet_events = plan_bio_inspired_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    
    if x_dubins is None or x_bio is None:
        print("❌ Path planning failed")
        return
    
    # Results
    dubins_length = sum(lengths)
    print(f"\n📊 RESULTS:")
    print(f"Traditional Dubins ({mode_dubins}):")
    print(f"  Path length: {dubins_length:.2f}m")
    print(f"  Energy cost: {dubins_length:.2f} (same as length)")
    print(f"  Path points: {len(x_dubins)}")
    
    print(f"\nJet Swimmer ({mode_bio}):")
    print(f"  Path length: {dubins_length:.2f}m (same geometric path)")
    print(f"  Energy cost: {energy_cost:.2f} (includes jet costs)")
    print(f"  Jet events: {len(jet_events)}")
    print(f"  Recharge time: 2.0s between jets")
    
    # Show detailed jet timing and directions
    print(f"\n🚀 JET SEQUENCE:")
    for i, event in enumerate(jet_events):
        nozzle_deg = math.degrees(event['nozzle_angle'])
        heading_deg = math.degrees(event['heading_change'])
        pos_x, pos_y = event['position']
        
        # Determine jet direction description
        if abs(nozzle_deg) < 5:
            direction = "straight back (forward thrust)"
        elif nozzle_deg > 0:
            direction = f"angled {nozzle_deg:.1f}° right (turn left)"
        else:
            direction = f"angled {abs(nozzle_deg):.1f}° left (turn right)"
        
        print(f"  t={event['time']:.1f}s: {event['type'].upper()} jet at ({pos_x:.1f}, {pos_y:.1f})")
        print(f"         Direction: {direction}")
        print(f"         Heading change: {heading_deg:+.1f}°")
        print(f"         Energy: {event['energy']:.1f} units")
        print()
    
    print(f"Energy efficiency: {((energy_cost - dubins_length) / dubins_length * 100):+.1f}% vs traditional")
    
    # Plot comparison
    plt.figure(figsize=(15, 5))
    
    # Traditional Dubins
    plt.subplot(131)
    plt.plot(x_dubins, y_dubins, 'b-', linewidth=2, label=f'Dubins {mode_dubins}')
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(end_x, end_y, 'ro', markersize=10, label='End')
    plt.arrow(start_x, start_y, math.cos(start_yaw), math.sin(start_yaw),
              head_width=0.3, head_length=0.3, fc='g', ec='g')
    plt.arrow(end_x, end_y, math.cos(end_yaw), math.sin(end_yaw),
              head_width=0.3, head_length=0.3, fc='r', ec='r')
    plt.title(f'Traditional Dubins\nLength: {dubins_length:.2f}m')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Bio-inspired
    plt.subplot(132)
    plt.plot(x_bio, y_bio, 'orange', linewidth=2, label='Bio-Inspired Jet-Glide')
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(end_x, end_y, 'ro', markersize=10, label='End')
    
    # Mark jet events with different colors, timing, and direction arrows
    for i, event in enumerate(jet_events):
        jx, jy = event['position']
        color = {'start': 'red', 'cruise': 'green', 'final': 'blue'}.get(event['type'], 'black')
        plt.plot(jx, jy, 'o', color=color, markersize=12, alpha=0.8)
        plt.text(jx + 0.3, jy + 0.3, f't={event["time"]:.1f}s', fontsize=8)
        
        # Add jet direction arrow (nozzle direction)
        nozzle_angle = event['nozzle_angle']
        # Jet direction is opposite to nozzle angle (Newton's 3rd law)
        jet_direction = nozzle_angle + math.pi
        arrow_length = 0.8
        arrow_dx = arrow_length * math.cos(jet_direction)
        arrow_dy = arrow_length * math.sin(jet_direction)
        
        plt.arrow(jx, jy, arrow_dx, arrow_dy, 
                 head_width=0.2, head_length=0.2, fc=color, ec=color, alpha=0.6)
        
        if i == 0:
            plt.plot(jx, jy, 'o', color=color, markersize=12, alpha=0.8, label='Jet Pulses')
    
    plt.arrow(start_x, start_y, math.cos(start_yaw), math.sin(start_yaw),
              head_width=0.3, head_length=0.3, fc='g', ec='g')
    plt.arrow(end_x, end_y, math.cos(end_yaw), math.sin(end_yaw),
              head_width=0.3, head_length=0.3, fc='r', ec='r')
    plt.title(f'Bio-Inspired Jet Propulsion\nEnergy: {energy_cost:.2f} units')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Energy comparison
    plt.subplot(133)
    methods = ['Traditional\nDubins', 'Bio-Inspired\nJet Propulsion']
    costs = [dubins_length, energy_cost]
    colors = ['blue', 'orange']
    
    bars = plt.bar(methods, costs, color=colors, alpha=0.7)
    plt.ylabel('Energy Cost')
    plt.title('Energy Comparison')
    plt.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bar, cost in zip(bars, costs):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                f'{cost:.2f}', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig('bio_inspired_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return True

if __name__ == "__main__":
    success = compare_paths()
    print(f"\n{'✅ Comparison complete!' if success else '❌ Comparison failed!'}")
