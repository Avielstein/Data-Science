"""
Animated Jet Swimmer Demonstration
Shows how the bio-inspired vehicle moves with discrete jet pulses
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from jet_swimmer import plan_bio_inspired_path
from dubins import plan_dubins_path

def create_jet_swimmer_animation():
    """Create animation showing jet swimmer motion over time"""
    
    # Test case
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    end_x, end_y, end_yaw = 10.0, 5.0, math.pi/2
    curvature = 0.5
    
    print("🎬 Creating Jet Swimmer Animation...")
    print(f"Start: ({start_x}, {start_y}) facing {math.degrees(start_yaw):.0f}°")
    print(f"End: ({end_x}, {end_y}) facing {math.degrees(end_yaw):.0f}°")
    
    # Get both paths
    x_dubins, y_dubins, yaw_dubins, mode_dubins, lengths = plan_dubins_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    
    x_bio, y_bio, yaw_bio, mode_bio, energy_cost, jet_events = plan_bio_inspired_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    
    if x_dubins is None or x_bio is None:
        print("❌ Path planning failed")
        return
    
    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Setup axes
    for ax in [ax1, ax2]:
        ax.set_xlim(-2, 12)
        ax.set_ylim(-2, 7)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
    
    ax1.set_title('Traditional Dubins Path\n(Continuous Motion)', fontsize=14)
    ax2.set_title('Jet Swimmer\n(Discrete Jet Pulses)', fontsize=14)
    
    # Plot full paths (faded)
    ax1.plot(x_dubins, y_dubins, 'b-', alpha=0.3, linewidth=1, label='Full Path')
    ax2.plot(x_bio, y_bio, 'orange', alpha=0.3, linewidth=1, label='Full Path')
    
    # Plot start/end points
    for ax in [ax1, ax2]:
        ax.plot(start_x, start_y, 'go', markersize=10, label='Start')
        ax.plot(end_x, end_y, 'ro', markersize=10, label='End')
        ax.arrow(start_x, start_y, math.cos(start_yaw), math.sin(start_yaw),
                head_width=0.2, head_length=0.2, fc='g', ec='g')
        ax.arrow(end_x, end_y, math.cos(end_yaw), math.sin(end_yaw),
                head_width=0.2, head_length=0.2, fc='r', ec='r')
    
    # Plot jet event locations
    for event in jet_events:
        jx, jy = event['position']
        color = {'start': 'red', 'cruise': 'green', 'final': 'blue'}.get(event['type'], 'black')
        ax2.plot(jx, jy, 'o', color=color, markersize=8, alpha=0.5)
        ax2.text(jx + 0.3, jy + 0.3, f't={event["time"]:.1f}s', fontsize=8)
    
    # Animation elements
    dubins_trail, = ax1.plot([], [], 'b-', linewidth=3, label='Current Path')
    dubins_vehicle, = ax1.plot([], [], 'bo', markersize=8)
    dubins_arrow = ax1.annotate('', xy=(0, 0), xytext=(0, 0),
                               arrowprops=dict(arrowstyle='->', color='blue', lw=2))
    
    bio_trail, = ax2.plot([], [], 'orange', linewidth=3, label='Current Path')
    bio_vehicle, = ax2.plot([], [], 'o', color='orange', markersize=8)
    bio_arrow = ax2.annotate('', xy=(0, 0), xytext=(0, 0),
                            arrowprops=dict(arrowstyle='->', color='orange', lw=2))
    
    # Jet burst visualization
    jet_burst = ax2.scatter([], [], s=[], c=[], alpha=0.8, cmap='Reds')
    
    # Status text
    status_text = fig.text(0.5, 0.02, '', ha='center', fontsize=12, 
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
    
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')
    
    # Animation parameters
    total_frames = 200
    max_points_dubins = len(x_dubins)
    max_points_bio = len(x_bio)
    
    def animate(frame):
        # Calculate current positions
        dubins_idx = min(int(frame * max_points_dubins / total_frames), max_points_dubins - 1)
        bio_idx = min(int(frame * max_points_bio / total_frames), max_points_bio - 1)
        
        # Current time
        current_time = frame * 6.0 / total_frames  # 6 seconds total
        
        # Update Dubins path
        dubins_trail.set_data(x_dubins[:dubins_idx+1], y_dubins[:dubins_idx+1])
        if dubins_idx < len(x_dubins):
            dubins_vehicle.set_data([x_dubins[dubins_idx]], [y_dubins[dubins_idx]])
            # Vehicle heading arrow
            arrow_len = 0.8
            dx = arrow_len * math.cos(yaw_dubins[dubins_idx])
            dy = arrow_len * math.sin(yaw_dubins[dubins_idx])
            dubins_arrow.set_position((x_dubins[dubins_idx], y_dubins[dubins_idx]))
            dubins_arrow.xy = (x_dubins[dubins_idx] + dx, y_dubins[dubins_idx] + dy)
        
        # Update bio-inspired path
        bio_trail.set_data(x_bio[:bio_idx+1], y_bio[:bio_idx+1])
        if bio_idx < len(x_bio):
            bio_vehicle.set_data([x_bio[bio_idx]], [y_bio[bio_idx]])
            # Vehicle heading arrow
            arrow_len = 0.8
            dx = arrow_len * math.cos(yaw_bio[bio_idx])
            dy = arrow_len * math.sin(yaw_bio[bio_idx])
            bio_arrow.set_position((x_bio[bio_idx], y_bio[bio_idx]))
            bio_arrow.xy = (x_bio[bio_idx] + dx, y_bio[bio_idx] + dy)
        
        # Check for active jet events
        active_jet = None
        for event in jet_events:
            if abs(current_time - event['time']) < 0.5:  # Jet pulse duration
                active_jet = event
                break
        
        # Update jet burst visualization
        if active_jet:
            # Show jet burst
            jx, jy = active_jet['position']
            # Create expanding burst effect
            burst_size = 200 * (1 - abs(current_time - active_jet['time']) / 0.5)
            jet_burst.set_offsets(np.array([[jx, jy]]))
            jet_burst.set_sizes([burst_size])
            jet_burst.set_array(np.array([1]))
            
            # Status text for jet event
            nozzle_deg = math.degrees(active_jet['nozzle_angle'])
            if abs(nozzle_deg) < 5:
                direction = "Forward Thrust"
            elif nozzle_deg > 0:
                direction = f"Turn Left ({nozzle_deg:.1f}°)"
            else:
                direction = f"Turn Right ({abs(nozzle_deg):.1f}°)"
            
            status_text.set_text(f"JET FIRING! t={current_time:.1f}s - {active_jet['type'].upper()} - {direction}")
        else:
            # No active jet - gliding phase
            jet_burst.set_offsets(np.empty((0, 2)))
            jet_burst.set_sizes([])
            jet_burst.set_array(np.array([]))
            
            # Find next jet
            next_jet = None
            for event in jet_events:
                if event['time'] > current_time:
                    next_jet = event
                    break
            
            if next_jet:
                time_to_next = next_jet['time'] - current_time
                status_text.set_text(f"⏱️  Gliding... Next jet in {time_to_next:.1f}s - Recharging energy")
            else:
                status_text.set_text(f"✅ Mission Complete! t={current_time:.1f}s")
        
        return dubins_trail, dubins_vehicle, bio_trail, bio_vehicle, jet_burst, status_text
    
    # Create animation
    print("🎬 Generating animation frames...")
    anim = animation.FuncAnimation(fig, animate, frames=total_frames, 
                                 interval=50, blit=False, repeat=True)
    
    # Save animation
    print("💾 Saving animation as GIF...")
    anim.save('jet_swimmer_animation.gif', writer='pillow', fps=20, dpi=100)
    print("✅ Animation saved as 'jet_swimmer_animation.gif'")
    
    plt.tight_layout()
    plt.show()
    
    return True

if __name__ == "__main__":
    success = create_jet_swimmer_animation()
    if success:
        print("\n🎬 Animation complete! Check 'jet_swimmer_animation.gif'")
    else:
        print("\n❌ Animation failed")
