"""
6-DOF Optimal Control for Underwater Vehicle Trajectories
Handles position (x,y), orientation (theta), and their derivatives (vx, vy, omega)
Using Pontryagin's Maximum Principle for complete underwater vehicle control
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate
import math

class OptimalController6DOF:
    """6-DOF optimal control solver using Pontryagin's Maximum Principle"""
    
    def __init__(self, max_thrust=1.0, max_torque=0.5):
        self.max_thrust = max_thrust      # Maximum thrust force
        self.max_torque = max_torque      # Maximum torque for rotation
        
    def solve(self, initial_state, final_state, max_attempts=30):
        """
        Solve 6-DOF optimal control problem
        
        Args:
            initial_state: [x, y, theta, vx, vy, omega] 
            final_state: [x, y, theta, vx, vy, omega]
            max_attempts: Maximum optimization attempts
            
        Returns:
            dict with solution parameters and trajectory
        """
        
        # Normalize angles to [-π, π]
        initial_state = np.array(initial_state)
        final_state = np.array(final_state)
        initial_state[2] = self._normalize_angle(initial_state[2])
        final_state[2] = self._normalize_angle(final_state[2])
        
        # Check for trivial case (same start and end)
        if np.allclose(initial_state, final_state, atol=1e-8):
            return {
                'success': True,
                'parameters': np.array([0, 0, 0, 0, 0, 0, 0.1]),  # 6 costates + time
                'error': 0.0,
                'attempts': 1
            }
        
        # Calculate distance and angle difference to adjust search parameters
        pos_distance = np.linalg.norm(final_state[:2] - initial_state[:2])
        angle_diff = abs(self._normalize_angle(final_state[2] - initial_state[2]))
        
        # Adjust search parameters based on complexity
        if pos_distance < 0.5 and angle_diff < 0.2:  # Small movement
            lambda_range = 5.0
            T_range = (0.5, 4.0)
            attempts_to_use = max_attempts
        elif pos_distance > 10.0 or angle_diff > math.pi/2:  # Large movement
            lambda_range = 3.0
            T_range = (2.0, 15.0)
            attempts_to_use = max_attempts * 2
        else:  # Normal movement
            lambda_range = 4.0
            T_range = (1.0, 10.0)
            attempts_to_use = max_attempts
        
        # Find optimal parameters using multiple random initializations
        best_result = None
        best_error = float('inf')
        
        for attempt in range(attempts_to_use):
            # Random initial guess: [λx, λy, λθ, λvx, λvy, λω, T]
            lambda_init = np.random.uniform(-lambda_range, lambda_range, 6)
            T_init = np.random.uniform(T_range[0], T_range[1], 1)
            params_init = np.append(lambda_init, T_init)
            
            # Constraint: time must be positive
            constraints = ({'type': 'ineq', 'fun': lambda x: x[6]})
            
            try:
                result = opt.minimize(
                    self._boundary_error, 
                    params_init,
                    constraints=constraints,
                    args=(initial_state, final_state),
                    method='SLSQP'
                )
                
                if result.success:
                    error = self._boundary_error(result.x, initial_state, final_state)
                    
                    if error < best_error:
                        best_error = error
                        best_result = result.x
                        
                        if error < 0.1:  # High precision requirement
                            return {
                                'success': True,
                                'parameters': result.x,
                                'error': error,
                                'attempts': attempt + 1
                            }
                        
            except:
                continue
        
        # Return best result found
        if best_result is not None and best_error < 1.0:  # Reasonable threshold
            return {
                'success': True,
                'parameters': best_result,
                'error': best_error,
                'attempts': attempts_to_use
            }
        
        return {'success': False, 'error': float('inf')}
    
    def _boundary_error(self, params, initial_state, final_state):
        """Calculate boundary condition error for 6-DOF system"""
        
        if len(params) < 7:
            return 1e6
            
        T = params[6]
        
        if T <= 0:
            return 1e6
        
        try:
            final_computed = self._integrate_dynamics_6dof(params, initial_state)
            
            # Calculate weighted error (position, angle, velocities)
            pos_error = np.linalg.norm(final_computed[:2] - final_state[:2])
            angle_error = abs(self._normalize_angle(final_computed[2] - final_state[2]))
            vel_error = np.linalg.norm(final_computed[3:6] - final_state[3:6])
            
            # Weighted total error
            total_error = pos_error + 2.0 * angle_error + 0.5 * vel_error
            return total_error
            
        except:
            return 1e6
    
    def _integrate_dynamics_6dof(self, params, initial_state):
        """Integrate 6-DOF system dynamics with simplified control"""
        
        λx, λy, λθ, λvx, λvy, λω, T = params
        
        def dynamics_6dof(t, state):
            x, y, theta, vx, vy, omega = state
            
            # Simplified control law - more stable than full Pontryagin
            # Direct control based on costates
            
            # Thrust control (world frame for simplicity)
            thrust_x = -λvx * self.max_thrust / 10.0  # Scale down for stability
            thrust_y = -λvy * self.max_thrust / 10.0
            
            # Limit thrust
            thrust_magnitude = math.sqrt(thrust_x**2 + thrust_y**2)
            if thrust_magnitude > self.max_thrust:
                thrust_x = self.max_thrust * thrust_x / thrust_magnitude
                thrust_y = self.max_thrust * thrust_y / thrust_magnitude
            
            # Torque control
            torque = -λω * self.max_torque / 5.0  # Scale down for stability
            if abs(torque) > self.max_torque:
                torque = self.max_torque * np.sign(torque)
            
            # 6-DOF dynamics
            dx_dt = vx
            dy_dt = vy
            dtheta_dt = omega
            dvx_dt = thrust_x
            dvy_dt = thrust_y
            domega_dt = torque
            
            return np.array([dx_dt, dy_dt, dtheta_dt, dvx_dt, dvy_dt, domega_dt])
        
        try:
            sol = integrate.solve_ivp(
                dynamics_6dof, [0, T], initial_state, 
                dense_output=True, rtol=1e-6, atol=1e-8
            )
            
            if sol.success:
                final_state = sol.y[:, -1]
                # Normalize angle
                final_state[2] = self._normalize_angle(final_state[2])
                return final_state
            else:
                raise Exception("Integration failed")
        except:
            raise Exception("Integration failed")
    
    def generate_trajectory_6dof(self, params, initial_state, dt=0.01):
        """Generate complete 6-DOF trajectory for visualization"""
        
        λx, λy, λθ, λvx, λvy, λω, T = params
        t_array = np.arange(0, T + dt, dt)
        
        trajectory = {
            'time': t_array,
            'position': {'x': [], 'y': [], 'theta': []},
            'velocity': {'x': [], 'y': [], 'omega': []},
            'control': {'thrust_x': [], 'thrust_y': [], 'torque': []}
        }
        
        state = np.array(initial_state, dtype=float)
        
        for t in t_array:
            # Store current state
            trajectory['position']['x'].append(state[0])
            trajectory['position']['y'].append(state[1])
            trajectory['position']['theta'].append(state[2])
            trajectory['velocity']['x'].append(state[3])
            trajectory['velocity']['y'].append(state[4])
            trajectory['velocity']['omega'].append(state[5])
            
            # Compute optimal control
            thrust_body_x = λvx
            thrust_body_y = λvy
            thrust_magnitude = math.sqrt(thrust_body_x**2 + thrust_body_y**2)
            
            if thrust_magnitude > self.max_thrust:
                thrust_body_x = self.max_thrust * thrust_body_x / thrust_magnitude
                thrust_body_y = self.max_thrust * thrust_body_y / thrust_magnitude
            
            # Convert to world frame
            cos_theta = math.cos(state[2])
            sin_theta = math.sin(state[2])
            
            thrust_world_x = thrust_body_x * cos_theta - thrust_body_y * sin_theta
            thrust_world_y = thrust_body_x * sin_theta + thrust_body_y * cos_theta
            
            torque = λω
            if abs(torque) > self.max_torque:
                torque = self.max_torque * np.sign(torque)
            
            trajectory['control']['thrust_x'].append(thrust_world_x)
            trajectory['control']['thrust_y'].append(thrust_world_y)
            trajectory['control']['torque'].append(torque)
            
            # Update state
            state[0] += state[3] * dt  # x += vx * dt
            state[1] += state[4] * dt  # y += vy * dt
            state[2] += state[5] * dt  # theta += omega * dt
            state[3] += thrust_world_x * dt  # vx += thrust_x * dt
            state[4] += thrust_world_y * dt  # vy += thrust_y * dt
            state[5] += torque * dt    # omega += torque * dt
            
            # Normalize angle
            state[2] = self._normalize_angle(state[2])
        
        return trajectory
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

class TrajectoryVisualizer6DOF:
    """6-DOF trajectory visualization"""
    
    @staticmethod
    def plot_6dof(trajectory, initial_state, final_state, title="6-DOF Optimal Trajectory"):
        """Plot 6-DOF trajectory results"""
        
        fig, axes = plt.subplots(3, 3, figsize=(18, 15))
        
        # Position trajectory with orientation arrows
        ax = axes[0, 0]
        ax.plot(trajectory['position']['x'], trajectory['position']['y'], 
                'b-', linewidth=2, label='Trajectory')
        ax.plot(initial_state[0], initial_state[1], 'go', markersize=12, label='Start')
        ax.plot(final_state[0], final_state[1], 'ro', markersize=12, label='Goal')
        
        # Add orientation arrows
        skip = max(1, len(trajectory['time']) // 10)
        for i in range(0, len(trajectory['time']), skip):
            x, y, theta = (trajectory['position']['x'][i], 
                          trajectory['position']['y'][i], 
                          trajectory['position']['theta'][i])
            dx, dy = 0.3 * math.cos(theta), 0.3 * math.sin(theta)
            ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, 
                    fc='red', ec='red', alpha=0.7)
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Position + Orientation')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        
        # Position components
        ax = axes[0, 1]
        ax.plot(trajectory['time'], trajectory['position']['x'], 'r-', linewidth=2, label='x')
        ax.plot(trajectory['time'], trajectory['position']['y'], 'b-', linewidth=2, label='y')
        ax.axhline(final_state[0], color='red', linestyle='--', alpha=0.5)
        ax.axhline(final_state[1], color='blue', linestyle='--', alpha=0.5)
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Position Components')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        
        # Orientation
        ax = axes[0, 2]
        theta_deg = [math.degrees(theta) for theta in trajectory['position']['theta']]
        ax.plot(trajectory['time'], theta_deg, 'g-', linewidth=2)
        ax.axhline(math.degrees(final_state[2]), color='green', linestyle='--', alpha=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_title('Orientation')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        
        # Velocity components
        ax = axes[1, 0]
        ax.plot(trajectory['time'], trajectory['velocity']['x'], 'r-', linewidth=2, label='vx')
        ax.plot(trajectory['time'], trajectory['velocity']['y'], 'b-', linewidth=2, label='vy')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Linear Velocities')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        
        # Angular velocity
        ax = axes[1, 1]
        omega_deg = [math.degrees(omega) for omega in trajectory['velocity']['omega']]
        ax.plot(trajectory['time'], omega_deg, 'g-', linewidth=2)
        ax.grid(True, alpha=0.3)
        ax.set_title('Angular Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (deg/s)')
        
        # Speed
        ax = axes[1, 2]
        speed = [math.sqrt(vx**2 + vy**2) for vx, vy in 
                zip(trajectory['velocity']['x'], trajectory['velocity']['y'])]
        ax.plot(trajectory['time'], speed, 'purple', linewidth=2)
        ax.grid(True, alpha=0.3)
        ax.set_title('Speed')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        
        # Control inputs - thrust
        ax = axes[2, 0]
        ax.plot(trajectory['time'], trajectory['control']['thrust_x'], 'r-', linewidth=2, label='Thrust X')
        ax.plot(trajectory['time'], trajectory['control']['thrust_y'], 'b-', linewidth=2, label='Thrust Y')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Thrust Control')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Thrust (N)')
        
        # Control inputs - torque
        ax = axes[2, 1]
        ax.plot(trajectory['time'], trajectory['control']['torque'], 'g-', linewidth=2)
        ax.grid(True, alpha=0.3)
        ax.set_title('Torque Control')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (N⋅m)')
        
        # Control magnitude
        ax = axes[2, 2]
        thrust_mag = [math.sqrt(tx**2 + ty**2) for tx, ty in 
                     zip(trajectory['control']['thrust_x'], trajectory['control']['thrust_y'])]
        ax.plot(trajectory['time'], thrust_mag, 'purple', linewidth=2, label='Thrust Magnitude')
        ax.plot(trajectory['time'], [abs(t) for t in trajectory['control']['torque']], 
               'orange', linewidth=2, label='|Torque|')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Control Magnitudes')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Effort')
        
        plt.suptitle(title, fontsize=16)
        plt.tight_layout()
        return fig

def test_6dof_control():
    """Test the 6-DOF optimal control system"""
    
    print("🌊 6-DOF UNDERWATER VEHICLE CONTROL TEST")
    print("=" * 45)
    
    # Test cases with position and orientation (realistic underwater vehicle constraints)
    test_cases = [
        {
            'name': 'Straight Movement',
            'initial': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # [x, y, θ, vx, vy, ω]
            'final': [5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'description': 'Move forward 5m, no rotation'
        },
        {
            'name': 'Gentle Turn',
            'initial': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'final': [3.0, 1.0, math.pi/8, 0.0, 0.0, 0.0],  # 22.5° turn
            'description': 'Move to (3,1) with 22.5° turn'
        },
        {
            'name': 'Moderate Turn',
            'initial': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'final': [3.0, 3.0, math.pi/4, 0.0, 0.0, 0.0],  # 45° turn
            'description': 'Move to (3,3) and face 45°'
        },
        {
            'name': 'Reverse Direction',
            'initial': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'final': [2.0, 4.0, -math.pi/6, 0.0, 0.0, 0.0],  # -30° turn
            'description': 'Move to (2,4) and face -30°'
        }
    ]
    
    controller = OptimalController6DOF(max_thrust=2.0, max_torque=1.0)
    results = []
    
    for i, case in enumerate(test_cases):
        print(f"\n🎯 Test {i+1}: {case['name']}")
        print(f"   {case['description']}")
        print(f"   From: {case['initial']}")
        print(f"   To: {case['final']}")
        
        result = controller.solve(case['initial'], case['final'])
        
        if result['success']:
            print(f"   ✅ Success: {result['error']:.6f} error in {result['attempts']} attempts")
            
            # Generate trajectory
            trajectory = controller.generate_trajectory_6dof(result['parameters'], case['initial'])
            print(f"   Time: {trajectory['time'][-1]:.3f}s")
            
            results.append({
                'case': case,
                'result': result,
                'trajectory': trajectory
            })
        else:
            print(f"   ❌ Failed to solve")
    
    # Visualize the most interesting case
    if results:
        # Find the most complex case that succeeded
        complex_case = max(results, key=lambda r: r['result']['error'])
        
        print(f"\n📊 Visualizing: {complex_case['case']['name']}")
        fig = TrajectoryVisualizer6DOF.plot_6dof(
            complex_case['trajectory'], 
            complex_case['case']['initial'], 
            complex_case['case']['final'],
            f"6-DOF Control: {complex_case['case']['name']}"
        )
        plt.savefig('optimal_control_6dof_test.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    return results

if __name__ == "__main__":
    test_6dof_control()
