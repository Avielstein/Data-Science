"""
Simple Working Dubins Path Planner
Based on proven mathematical formulations from literature
"""

import math
import numpy as np
import matplotlib.pyplot as plt

def mod2pi(angle):
    """Normalize angle to [0, 2π)"""
    return angle - 2.0 * math.pi * math.floor(angle / (2.0 * math.pi))

def plan_dubins_path(sx, sy, syaw, gx, gy, gyaw, curvature, step_size=0.1):
    """
    Plan dubins path
    
    Parameters:
    sx, sy, syaw: start x, y, yaw [m, m, rad]
    gx, gy, gyaw: goal x, y, yaw [m, m, rad]  
    curvature: curvature [1/m]
    step_size: step size [m]
    
    Returns:
    x_list, y_list: path coordinates
    yaw_list: path yaw angles
    mode: path type (LSL, RSR, etc.)
    lengths: segment lengths
    """
    
    # Calculate relative position and orientation
    dx = gx - sx
    dy = gy - sy
    D = math.sqrt(dx * dx + dy * dy)
    d = D * curvature
    
    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(syaw - theta)
    beta = mod2pi(gyaw - theta)
    
    # Try all 6 path types and find shortest
    planners = [
        ("LSL", LSL),
        ("RSR", RSR), 
        ("LSR", LSR),
        ("RSL", RSL),
        ("RLR", RLR),
        ("LRL", LRL)
    ]
    
    best_cost = float('inf')
    best_path = None
    
    for mode, planner in planners:
        t, p, q = planner(alpha, beta, d)
        if t is not None:
            cost = abs(t) + abs(p) + abs(q)
            if cost < best_cost:
                best_cost = cost
                best_path = (mode, t, p, q)
    
    if best_path is None:
        return None, None, None, None, None
        
    mode, t, p, q = best_path
    
    # Generate path
    x_list, y_list, yaw_list = generate_course(
        sx, sy, syaw, t, p, q, mode, curvature, step_size)
    
    lengths = [abs(t) / curvature, abs(p) / curvature, abs(q) / curvature]
    
    return x_list, y_list, yaw_list, mode, lengths

def LSL(alpha, beta, d):
    """Left-Straight-Left path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    tmp0 = d + sa - sb
    p_squared = 2 + d*d - 2*c_ab + 2*d*(sa - sb)
    
    if p_squared < 0:
        return None, None, None
        
    tmp1 = math.atan2(cb - ca, tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    
    return t, p, q

def RSR(alpha, beta, d):
    """Right-Straight-Right path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    tmp0 = d - sa + sb
    p_squared = 2 + d*d - 2*c_ab + 2*d*(sb - sa)
    
    if p_squared < 0:
        return None, None, None
        
    tmp1 = math.atan2(ca - cb, tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)
    
    return t, p, q

def LSR(alpha, beta, d):
    """Left-Straight-Right path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    p_squared = -2 + d*d + 2*c_ab + 2*d*(sa + sb)
    
    if p_squared < 0:
        return None, None, None
        
    p = math.sqrt(p_squared)
    tmp2 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)
    
    return t, p, q

def RSL(alpha, beta, d):
    """Right-Straight-Left path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    p_squared = d*d - 2 + 2*c_ab - 2*d*(sa + sb)
    
    if p_squared < 0:
        return None, None, None
        
    p = math.sqrt(p_squared)
    tmp2 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)
    
    return t, p, q

def RLR(alpha, beta, d):
    """Right-Left-Right path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    tmp_rlr = (6.0 - d*d + 2*c_ab + 2*d*(sa - sb)) / 8.0
    
    if abs(tmp_rlr) > 1.0:
        return None, None, None
        
    p = mod2pi(2*math.pi - math.acos(tmp_rlr))
    t = mod2pi(-alpha + math.atan2(ca - cb, d - sa + sb) + p/2.0)
    q = mod2pi(alpha - beta - t + p)
    
    return t, p, q

def LRL(alpha, beta, d):
    """Left-Right-Left path"""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    
    tmp_lrl = (6.0 - d*d + 2*c_ab + 2*d*(-sa + sb)) / 8.0
    
    if abs(tmp_lrl) > 1.0:
        return None, None, None
        
    p = mod2pi(2*math.pi - math.acos(tmp_lrl))
    t = mod2pi(alpha - math.atan2(ca - cb, d + sa - sb) + p/2.0)
    q = mod2pi(beta - alpha - t + p)
    
    return t, p, q

def generate_course(sx, sy, syaw, t, p, q, mode, curvature, step_size):
    """Generate course from path parameters"""
    
    x_list = [sx]
    y_list = [sy]
    yaw_list = [syaw]
    
    # First segment
    if mode[0] == 'L':
        x, y, yaw = generate_arc(sx, sy, syaw, t, curvature, step_size, 'L')
    else:
        x, y, yaw = generate_arc(sx, sy, syaw, t, curvature, step_size, 'R')
    
    x_list.extend(x[1:])
    y_list.extend(y[1:])
    yaw_list.extend(yaw[1:])
    
    # Second segment
    if mode[1] == 'S':
        x, y, yaw = generate_straight(x_list[-1], y_list[-1], yaw_list[-1], 
                                    p / curvature, step_size)
    elif mode[1] == 'L':
        x, y, yaw = generate_arc(x_list[-1], y_list[-1], yaw_list[-1], 
                               p, curvature, step_size, 'L')
    else:
        x, y, yaw = generate_arc(x_list[-1], y_list[-1], yaw_list[-1], 
                               p, curvature, step_size, 'R')
    
    x_list.extend(x[1:])
    y_list.extend(y[1:])
    yaw_list.extend(yaw[1:])
    
    # Third segment
    if mode[2] == 'L':
        x, y, yaw = generate_arc(x_list[-1], y_list[-1], yaw_list[-1], 
                               q, curvature, step_size, 'L')
    else:
        x, y, yaw = generate_arc(x_list[-1], y_list[-1], yaw_list[-1], 
                               q, curvature, step_size, 'R')
    
    x_list.extend(x[1:])
    y_list.extend(y[1:])
    yaw_list.extend(yaw[1:])
    
    return x_list, y_list, yaw_list

def generate_arc(sx, sy, syaw, length, curvature, step_size, direction):
    """Generate arc segment"""
    x_list = []
    y_list = []
    yaw_list = []
    
    arc_length = abs(length) / curvature
    n_points = max(int(arc_length / step_size), 1)
    
    for i in range(n_points + 1):
        s = i * arc_length / n_points
        
        if direction == 'L':
            yaw = syaw + s * curvature
        else:
            yaw = syaw - s * curvature
            
        x = sx + (math.sin(yaw) - math.sin(syaw)) / curvature
        y = sy - (math.cos(yaw) - math.cos(syaw)) / curvature
        
        x_list.append(x)
        y_list.append(y)
        yaw_list.append(yaw)
    
    return x_list, y_list, yaw_list

def generate_straight(sx, sy, syaw, length, step_size):
    """Generate straight segment"""
    x_list = []
    y_list = []
    yaw_list = []
    
    n_points = max(int(length / step_size), 1)
    
    for i in range(n_points + 1):
        s = i * length / n_points
        x = sx + s * math.cos(syaw)
        y = sy + s * math.sin(syaw)
        
        x_list.append(x)
        y_list.append(y)
        yaw_list.append(syaw)
    
    return x_list, y_list, yaw_list

def test_dubins():
    """Test the Dubins path planner"""
    print("🌊 DUBINS PATH PLANNER TEST")
    print("=" * 40)
    
    # Test case
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    end_x, end_y, end_yaw = 10.0, 5.0, math.pi/2
    curvature = 0.5  # 1/turning_radius
    
    print(f"Start: ({start_x}, {start_y}) facing {math.degrees(start_yaw):.0f}°")
    print(f"End: ({end_x}, {end_y}) facing {math.degrees(end_yaw):.0f}°")
    print(f"Turning radius: {1/curvature:.1f}m")
    
    # Plan path
    x_list, y_list, yaw_list, mode, lengths = plan_dubins_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)
    
    if x_list is None:
        print("❌ No valid path found")
        return
    
    total_length = sum(lengths)
    print(f"\n✅ Path found: {mode}")
    print(f"Total length: {total_length:.2f}m")
    print(f"Segment lengths: {[f'{l:.2f}m' for l in lengths]}")
    
    # Check accuracy
    final_x, final_y, final_yaw = x_list[-1], y_list[-1], yaw_list[-1]
    pos_error = math.sqrt((final_x - end_x)**2 + (final_y - end_y)**2)
    yaw_error = abs(final_yaw - end_yaw)
    
    print(f"\nAccuracy check:")
    print(f"Final position: ({final_x:.3f}, {final_y:.3f})")
    print(f"Target position: ({end_x:.3f}, {end_y:.3f})")
    print(f"Position error: {pos_error:.3f}m")
    print(f"Yaw error: {math.degrees(yaw_error):.1f}°")
    
    # Plot
    plt.figure(figsize=(10, 8))
    plt.plot(x_list, y_list, 'b-', linewidth=2, label=f'{mode} Path')
    
    # Start and end points
    plt.plot(start_x, start_y, 'go', markersize=10, label='Start')
    plt.plot(end_x, end_y, 'ro', markersize=10, label='End')
    
    # Direction arrows
    arrow_len = 1.0
    plt.arrow(start_x, start_y, arrow_len*math.cos(start_yaw), arrow_len*math.sin(start_yaw),
              head_width=0.3, head_length=0.3, fc='g', ec='g')
    plt.arrow(end_x, end_y, arrow_len*math.cos(end_yaw), arrow_len*math.sin(end_yaw),
              head_width=0.3, head_length=0.3, fc='r', ec='r')
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title(f'Dubins Path: {mode} (Length: {total_length:.2f}m)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    
    plt.tight_layout()
    plt.savefig('dubins_test.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return pos_error < 0.1 and yaw_error < 0.1

if __name__ == "__main__":
    success = test_dubins()
    print(f"\nTest {'✅ PASSED' if success else '❌ FAILED'}")
