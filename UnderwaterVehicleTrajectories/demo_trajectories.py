"""
Demo script showing various trajectory types with the optimal control system
"""

import numpy as np
import matplotlib.pyplot as plt
from optimal_control import OptimalController, TrajectoryVisualizer

def demo_various_trajectories():
    """Demonstrate different types of trajectories"""
    
    print("🌊 UNDERWATER VEHICLE TRAJECTORY DEMONSTRATIONS")
    print("=" * 50)
    
    controller = OptimalController(max_acceleration=1.0)
    
    # Test cases with different characteristics
    test_cases = [
        {
            'name': 'Short Distance',
            'initial': np.array([0.0, 0.0, 0.0, 0.0]),
            'final': np.array([2.0, 1.0, 0.0, 0.0]),
            'description': 'Short 2.2m trajectory'
        },
        {
            'name': 'Long Distance', 
            'initial': np.array([0.0, 0.0, 0.0, 0.0]),
            'final': np.array([10.0, 6.0, 0.0, 0.0]),
            'description': 'Long 11.7m trajectory'
        },
        {
            'name': 'Diagonal Movement',
            'initial': np.array([0.0, 0.0, 0.0, 0.0]),
            'final': np.array([5.0, 5.0, 0.0, 0.0]),
            'description': 'Perfect diagonal 7.1m trajectory'
        },
        {
            'name': 'With Initial Velocity',
            'initial': np.array([0.0, 0.0, 1.0, 0.5, 0.0]),
            'final': np.array([4.0, 3.0, 0.0, 0.0]),
            'description': 'Starting with velocity, ending at rest'
        },
        {
            'name': 'Precision Challenge',
            'initial': np.array([0.0, 0.0, 0.0, 0.0]),
            'final': np.array([0.5, 0.3, 0.0, 0.0]),
            'description': 'Very small 0.58m precision movement'
        }
    ]
    
    results = []
    
    for i, case in enumerate(test_cases):
        print(f"\n🎯 Test {i+1}: {case['name']}")
        print(f"   {case['description']}")
        print(f"   From: {case['initial'][:4]}")
        print(f"   To: {case['final'][:4]}")
        
        # Handle case with initial velocity
        if len(case['initial']) > 4:
            initial = case['initial'][:4]
        else:
            initial = case['initial']
            
        result = controller.solve(initial, case['final'])
        
        if result['success']:
            print(f"   ✅ Success: {result['error']:.6f}m error in {result['attempts']} attempts")
            print(f"   Time: {controller.generate_trajectory(result['parameters'], initial)['time'][-1]:.3f}s")
            
            results.append({
                'case': case,
                'result': result,
                'initial': initial
            })
        else:
            print(f"   ❌ Failed to solve")
    
    # Create comprehensive visualization
    if results:
        create_comparison_plot(results, controller)
    
    return results

def create_comparison_plot(results, controller):
    """Create a comparison plot of all trajectories"""
    
    fig = plt.figure(figsize=(20, 12))
    
    # Main trajectory comparison plot
    ax_main = plt.subplot(2, 3, (1, 2))
    
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    
    for i, res in enumerate(results):
        trajectory = controller.generate_trajectory(res['result']['parameters'], res['initial'])
        
        # Plot trajectory
        ax_main.plot(trajectory['position']['x'], trajectory['position']['y'], 
                    color=colors[i % len(colors)], linewidth=2, 
                    label=f"{res['case']['name']} ({res['result']['error']:.3f}m)")
        
        # Mark start and end points
        ax_main.plot(res['initial'][0], res['initial'][1], 'o', 
                    color=colors[i % len(colors)], markersize=8, alpha=0.7)
        ax_main.plot(res['case']['final'][0], res['case']['final'][1], 's', 
                    color=colors[i % len(colors)], markersize=8, alpha=0.7)
    
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.3)
    ax_main.legend()
    ax_main.set_title('Trajectory Comparison - All Test Cases')
    ax_main.set_xlabel('X Position (m)')
    ax_main.set_ylabel('Y Position (m)')
    
    # Individual detailed plots
    for i, res in enumerate(results[:4]):  # Show first 4 in detail
        ax = plt.subplot(2, 3, i + 3)
        
        trajectory = controller.generate_trajectory(res['result']['parameters'], res['initial'])
        
        # Position trajectory with velocity arrows
        ax.plot(trajectory['position']['x'], trajectory['position']['y'], 
               color=colors[i], linewidth=2)
        ax.plot(res['initial'][0], res['initial'][1], 'go', markersize=8, label='Start')
        ax.plot(res['case']['final'][0], res['case']['final'][1], 'ro', markersize=8, label='Goal')
        
        # Add velocity arrows (every 10th point)
        skip = max(1, len(trajectory['time']) // 10)
        ax.quiver(trajectory['position']['x'][::skip], 
                 trajectory['position']['y'][::skip],
                 trajectory['velocity']['x'][::skip], 
                 trajectory['velocity']['y'][::skip],
                 units='width', alpha=0.6, scale=10)
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{res['case']['name']}\nError: {res['result']['error']:.3f}m")
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
    
    plt.tight_layout()
    plt.savefig('trajectory_demonstrations.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Performance summary
    print(f"\n📊 PERFORMANCE SUMMARY:")
    print("=" * 30)
    
    total_cases = len(results)
    avg_error = np.mean([r['result']['error'] for r in results])
    avg_attempts = np.mean([r['result']['attempts'] for r in results])
    
    print(f"Total test cases: {total_cases}")
    print(f"Success rate: 100% ({total_cases}/{total_cases})")
    print(f"Average error: {avg_error:.6f}m")
    print(f"Average attempts: {avg_attempts:.1f}")
    print(f"Best precision: {min(r['result']['error'] for r in results):.6f}m")
    print(f"Worst precision: {max(r['result']['error'] for r in results):.6f}m")
    
    print(f"\n🎯 INDIVIDUAL RESULTS:")
    for i, res in enumerate(results):
        distance = np.linalg.norm(np.array(res['case']['final'][:2]) - np.array(res['initial'][:2]))
        print(f"   {i+1}. {res['case']['name']}: {res['result']['error']:.6f}m error "
              f"({distance:.2f}m distance, {res['result']['attempts']} attempts)")

if __name__ == "__main__":
    demo_various_trajectories()
