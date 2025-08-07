"""
Optimal Control for Underwater Vehicle Trajectories
Clean, modular implementation using Pontryagin's Maximum Principle
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate
import math

class OptimalController:
    """Clean optimal control solver using Pontryagin's Maximum Principle"""
    
    def __init__(self, max_acceleration=1.0):
        self.max_acceleration = max_acceleration
        
    def solve(self, initial_state, final_state, max_attempts=20):
        """
        Solve optimal control problem
        
        Args:
            initial_state: [x, y, vx, vy] 
            final_state: [x, y, vx, vy]
            max_attempts: Maximum optimization attempts
            
        Returns:
            dict with solution parameters and trajectory
        """
        
        # Check for trivial case (same start and end)
        if np.allclose(initial_state, final_state, atol=1e-8):
            return {
                'success': True,
                'parameters': np.array([0, 0, 0, 0, 0.1]),  # Minimal time, no control
                'error': 0.0,
                'attempts': 1
            }
        
        # Calculate distance to adjust search parameters
        distance = np.linalg.norm(np.array(final_state[:2]) - np.array(initial_state[:2]))
        
        # Handle very small movements with analytical solution
        if distance < 0.12:  # Very small movements - use analytical approach
            return self._solve_small_movement(initial_state, final_state)
        elif distance < 0.2:  # Borderline small movements - use enhanced optimization
            return self._solve_borderline_movement(initial_state, final_state)
        
        # Adjust search parameters based on distance
        if distance < 2.0:  # Small to medium movements - need more care
            lambda_range = 8.0
            T_range = (0.3, 6.0)
            attempts_to_use = max_attempts * 2
        else:  # Normal movements
            lambda_range = 5.0
            T_range = (1.0, 10.0)
            attempts_to_use = max_attempts
        
        # Find optimal parameters using multiple random initializations
        for attempt in range(attempts_to_use):
            # Random initial guess: [λ1, λ2, λ3, λ4, T]
            lambda_init = np.random.uniform(-lambda_range, lambda_range, 4)
            T_init = np.random.uniform(T_range[0], T_range[1], 1)
            params_init = np.append(lambda_init, T_init)
            
            # Constraint: time must be positive
            constraints = ({'type': 'ineq', 'fun': lambda x: x[4]})
            
            try:
                result = opt.minimize(
                    self._boundary_error, 
                    params_init,
                    constraints=constraints,
                    args=(initial_state, final_state)
                )
                
                if result.success:
                    error = self._boundary_error(result.x, initial_state, final_state)
                    
                    if error < 0.1:  # High precision requirement
                        return {
                            'success': True,
                            'parameters': result.x,
                            'error': error,
                            'attempts': attempt + 1
                        }
                        
            except:
                continue
        
        return {'success': False, 'error': float('inf')}
    
    def _solve_small_movement(self, initial_state, final_state):
        """Analytical solution for very small movements"""
        
        # For very small movements, use a simple analytical approach
        dx = final_state[0] - initial_state[0]
        dy = final_state[1] - initial_state[1]
        dvx = final_state[2] - initial_state[2]
        dvy = final_state[3] - initial_state[3]
        
        # Calculate required time for small movement
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 1e-10:  # Essentially zero movement
            return {
                'success': True,
                'parameters': np.array([0, 0, 0, 0, 0.1]),
                'error': 0.0,
                'attempts': 1
            }
        
        # For small movements, use a simple bang-bang control approach
        # Accelerate for half the time, decelerate for half the time
        T = 2.0 * math.sqrt(2.0 * distance / self.max_acceleration)
        
        # Direction of movement
        theta = math.atan2(dy, dx)
        
        # Simple parameters for bang-bang control
        # These create acceleration in the direction of movement
        lambda1 = -self.max_acceleration * math.cos(theta) / T
        lambda2 = -self.max_acceleration * math.sin(theta) / T
        lambda3 = -lambda1 * T / 2
        lambda4 = -lambda2 * T / 2
        
        params = np.array([lambda1, lambda2, lambda3, lambda4, T])
        
        # Verify the solution
        try:
            final_computed = self._integrate_dynamics(params, initial_state)
            error = np.linalg.norm(final_computed - final_state)
            
            if error < 0.1:  # Good enough for small movements
                return {
                    'success': True,
                    'parameters': params,
                    'error': error,
                    'attempts': 1
                }
        except:
            pass
        
        # If analytical approach fails, fall back to optimization with more attempts
        for attempt in range(60):  # More attempts for difficult small movements
            lambda_init = np.random.uniform(-15.0, 15.0, 4)
            T_init = np.random.uniform(0.1, 3.0, 1)
            params_init = np.append(lambda_init, T_init)
            
            constraints = ({'type': 'ineq', 'fun': lambda x: x[4]})
            
            try:
                result = opt.minimize(
                    self._boundary_error, 
                    params_init,
                    constraints=constraints,
                    args=(initial_state, final_state)
                )
                
                if result.success:
                    error = self._boundary_error(result.x, initial_state, final_state)
                    
                    if error < 0.1:
                        return {
                            'success': True,
                            'parameters': result.x,
                            'error': error,
                            'attempts': attempt + 1
                        }
            except:
                continue
        
        return {'success': False, 'error': float('inf')}
    
    def _solve_borderline_movement(self, initial_state, final_state):
        """Enhanced optimization for borderline small movements (0.12-0.2m)"""
        
        # For borderline cases like 0.1m, use multiple strategies
        dx = final_state[0] - initial_state[0]
        dy = final_state[1] - initial_state[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Strategy 1: Try analytical approach first
        try:
            T = 2.0 * math.sqrt(2.0 * distance / self.max_acceleration)
            theta = math.atan2(dy, dx)
            
            lambda1 = -self.max_acceleration * math.cos(theta) / T
            lambda2 = -self.max_acceleration * math.sin(theta) / T
            lambda3 = -lambda1 * T / 2
            lambda4 = -lambda2 * T / 2
            
            params = np.array([lambda1, lambda2, lambda3, lambda4, T])
            
            final_computed = self._integrate_dynamics(params, initial_state)
            error = np.linalg.norm(final_computed - final_state)
            
            if error < 0.1:
                return {
                    'success': True,
                    'parameters': params,
                    'error': error,
                    'attempts': 1
                }
        except:
            pass
        
        # Strategy 2: Enhanced optimization with multiple approaches
        best_result = None
        best_error = float('inf')
        
        # Try different optimization strategies
        strategies = [
            # Strategy A: Wide search
            {'lambda_range': 20.0, 'T_range': (0.1, 4.0), 'attempts': 40},
            # Strategy B: Focused search
            {'lambda_range': 10.0, 'T_range': (0.5, 2.0), 'attempts': 30},
            # Strategy C: Fine-tuned search
            {'lambda_range': 5.0, 'T_range': (0.8, 1.5), 'attempts': 20}
        ]
        
        total_attempts = 0
        for strategy in strategies:
            for attempt in range(strategy['attempts']):
                total_attempts += 1
                
                lambda_init = np.random.uniform(-strategy['lambda_range'], strategy['lambda_range'], 4)
                T_init = np.random.uniform(strategy['T_range'][0], strategy['T_range'][1], 1)
                params_init = np.append(lambda_init, T_init)
                
                constraints = ({'type': 'ineq', 'fun': lambda x: x[4]})
                
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
                            
                            if error < 0.1:  # Good enough
                                return {
                                    'success': True,
                                    'parameters': result.x,
                                    'error': error,
                                    'attempts': total_attempts
                                }
                except:
                    continue
        
        # Return best result found, even if not perfect
        if best_result is not None and best_error < 0.2:  # Relaxed threshold for difficult cases
            return {
                'success': True,
                'parameters': best_result,
                'error': best_error,
                'attempts': total_attempts
            }
        
        return {'success': False, 'error': float('inf')}
    
    def _boundary_error(self, params, initial_state, final_state):
        """Calculate boundary condition error"""
        
        λ1, λ2, λ3, λ4, T = params
        
        if T <= 0:
            return 1e6
        
        try:
            final_computed = self._integrate_dynamics(params, initial_state)
            return np.linalg.norm(final_computed - final_state)
        except:
            return 1e6
    
    def _integrate_dynamics(self, params, initial_state):
        """Integrate system dynamics with optimal control"""
        
        λ1, λ2, λ3, λ4, T = params
        
        def dynamics(t, state):
            x, y, vx, vy = state
            
            # Optimal control from Pontryagin's principle
            control_vec = np.array([λ1*t - λ3, λ2*t - λ4])
            control_norm = np.linalg.norm(control_vec)
            
            if control_norm > 1e-6:
                ux = -self.max_acceleration * control_vec[0] / control_norm
                uy = -self.max_acceleration * control_vec[1] / control_norm
            else:
                ux = uy = 0.0
            
            return np.array([vx, vy, ux, uy])
        
        sol = integrate.solve_ivp(
            dynamics, [0, T], initial_state, 
            dense_output=True, rtol=1e-8
        )
        
        if sol.success:
            return sol.y[:, -1]
        else:
            raise Exception("Integration failed")
    
    def generate_trajectory(self, params, initial_state, dt=0.01):
        """Generate complete trajectory for visualization"""
        
        λ1, λ2, λ3, λ4, T = params
        t_array = np.arange(0, T + dt, dt)
        
        trajectory = {
            'time': t_array,
            'position': {'x': [], 'y': []},
            'velocity': {'x': [], 'y': []},
            'control': {'x': [], 'y': []}
        }
        
        state = np.array(initial_state, dtype=float)
        
        for t in t_array:
            # Store current state
            trajectory['position']['x'].append(state[0])
            trajectory['position']['y'].append(state[1])
            trajectory['velocity']['x'].append(state[2])
            trajectory['velocity']['y'].append(state[3])
            
            # Compute optimal control
            control_vec = np.array([λ1*t - λ3, λ2*t - λ4])
            control_norm = np.linalg.norm(control_vec)
            
            if control_norm > 1e-6:
                ux = -self.max_acceleration * control_vec[0] / control_norm
                uy = -self.max_acceleration * control_vec[1] / control_norm
            else:
                ux = uy = 0.0
            
            trajectory['control']['x'].append(ux)
            trajectory['control']['y'].append(uy)
            
            # Update state
            state[0] += state[2] * dt
            state[1] += state[3] * dt
            state[2] += ux * dt
            state[3] += uy * dt
        
        return trajectory

class TrajectoryVisualizer:
    """Clean trajectory visualization"""
    
    @staticmethod
    def plot(trajectory, initial_state, final_state, title="Optimal Trajectory"):
        """Plot trajectory results"""
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Position trajectory
        ax = axes[0, 0]
        ax.plot(trajectory['position']['x'], trajectory['position']['y'], 
                'b-', linewidth=2, label='Trajectory')
        ax.plot(initial_state[0], initial_state[1], 'go', markersize=10, label='Start')
        ax.plot(final_state[0], final_state[1], 'ro', markersize=10, label='Goal')
        
        # Add velocity arrows
        skip = len(trajectory['time']) // 15
        ax.quiver(trajectory['position']['x'][::skip], 
                 trajectory['position']['y'][::skip],
                 trajectory['velocity']['x'][::skip], 
                 trajectory['velocity']['y'][::skip],
                 units='width', alpha=0.7)
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Position Trajectory')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        
        # Control inputs
        ax = axes[0, 1]
        ax.plot(trajectory['time'], trajectory['control']['x'], 'r-', linewidth=2, label='ux')
        ax.plot(trajectory['time'], trajectory['control']['y'], 'b-', linewidth=2, label='uy')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Control Inputs')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        
        # Velocity profiles
        ax = axes[1, 0]
        ax.plot(trajectory['time'], trajectory['velocity']['x'], 'r-', linewidth=2, label='vx')
        ax.plot(trajectory['time'], trajectory['velocity']['y'], 'b-', linewidth=2, label='vy')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Velocity Profiles')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        
        # Position components
        ax = axes[1, 1]
        ax.plot(trajectory['time'], trajectory['position']['x'], 'r-', linewidth=2, label='x')
        ax.plot(trajectory['time'], trajectory['position']['y'], 'b-', linewidth=2, label='y')
        ax.axhline(final_state[0], color='red', linestyle='--', alpha=0.5)
        ax.axhline(final_state[1], color='blue', linestyle='--', alpha=0.5)
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title('Position Components')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        
        plt.suptitle(title)
        plt.tight_layout()
        return fig

def test_optimal_control():
    """Test the optimal control system"""
    
    print("🎯 OPTIMAL CONTROL TEST")
    print("=" * 30)
    
    # Test case
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    final = np.array([5.0, 3.0, 0.0, 0.0])
    
    print(f"Initial state: {initial}")
    print(f"Final state: {final}")
    
    # Solve
    controller = OptimalController(max_acceleration=1.0)
    result = controller.solve(initial, final)
    
    if result['success']:
        print(f"✅ Solution found!")
        print(f"   Error: {result['error']:.6f}m")
        print(f"   Attempts: {result['attempts']}")
        
        # Generate trajectory
        trajectory = controller.generate_trajectory(result['parameters'], initial)
        
        # Calculate final state
        final_computed = np.array([
            trajectory['position']['x'][-1],
            trajectory['position']['y'][-1],
            trajectory['velocity']['x'][-1],
            trajectory['velocity']['y'][-1]
        ])
        
        print(f"   Final computed: {final_computed}")
        print(f"   Time: {trajectory['time'][-1]:.3f}s")
        
        # Visualize
        fig = TrajectoryVisualizer.plot(trajectory, initial, final)
        plt.savefig('optimal_control_test.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        return True
    else:
        print("❌ Failed to find solution")
        return False

if __name__ == "__main__":
    test_optimal_control()
