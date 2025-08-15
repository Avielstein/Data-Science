"""
SALP 3D Trajectory Planning Demo Scenarios
Comprehensive demonstration of bio-inspired jet propulsion trajectory planning
for various underwater mission scenarios

Based on Cynthia Sung's SALP project at UPenn GRASP Lab
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from salp_3d_trajectory_planner import SALP3DTrajectoryPlanner

class SALPMissionPlanner:
    """Mission planner for SALP robots with various scenario types"""
    
    def __init__(self):
        """Initialize mission planner with different robot configurations"""
        
        # Different SALP configurations for different missions
        self.configurations = {
            'lightweight': SALP3DTrajectoryPlanner(
                min_turning_radius=1.5,
                max_nozzle_angle=35.0,  # More agile
                jet_thrust=2.0,
                robot_mass=0.4,
                pulse_duration=0.12,
                min_pulse_interval=0.03
            ),
            'standard': SALP3DTrajectoryPlanner(
                min_turning_radius=2.0,
                max_nozzle_angle=25.0,
                jet_thrust=3.0,
                robot_mass=0.8,
                pulse_duration=0.15,
                min_pulse_interval=0.05
            ),
            'heavy_payload': SALP3DTrajectoryPlanner(
                min_turning_radius=3.0,
                max_nozzle_angle=20.0,  # Less agile but more stable
                jet_thrust=4.5,
                robot_mass=1.2,
                pulse_duration=0.18,
                min_pulse_interval=0.08
            )
        }
    
    def run_all_scenarios(self):
        """Run all demonstration scenarios"""
        
        print("🌊 SALP MISSION SCENARIOS DEMONSTRATION")
        print("=" * 60)
        
        scenarios = [
            self.scenario_1_surface_to_depth,
            self.scenario_2_obstacle_avoidance,
            self.scenario_3_search_pattern,
            self.scenario_4_multi_waypoint_survey,
            self.scenario_5_emergency_ascent,
            self.scenario_6_precision_docking
        ]
        
        results = {}
        
        for i, scenario in enumerate(scenarios, 1):
            print(f"\n{'='*20} SCENARIO {i} {'='*20}")
            try:
                result = scenario()
                results[f'scenario_{i}'] = result
                print(f"✅ Scenario {i} completed successfully")
            except Exception as e:
                print(f"❌ Scenario {i} failed: {e}")
                results[f'scenario_{i}'] = None
        
        # Create summary visualization
        self.create_summary_visualization(results)
        
        return results
    
    def scenario_1_surface_to_depth(self):
        """Scenario 1: Surface to depth dive with orientation change"""
        
        print("📍 SCENARIO 1: Surface to Depth Dive")
        print("Mission: Dive from surface to 10m depth while changing orientation")
        
        planner = self.configurations['standard']
        
        # Start at surface, end at depth with different orientation
        start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Surface, facing north
        goal_pose = [5.0, 3.0, -10.0, 0.0, -0.3, 1.57]  # 10m deep, pitched down, facing east
        
        result = planner.plan_3d_trajectory(start_pose, goal_pose)
        
        if result['success']:
            print(f"   Dive completed in {result['total_time']:.1f}s with {result['total_pulses']} pulses")
            print(f"   Path length: {result['path_length']:.1f}m")
            
            # Save visualization
            planner.visualize_3d_trajectory(result)
            plt.savefig('salp_scenario_1_dive.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return result
    
    def scenario_2_obstacle_avoidance(self):
        """Scenario 2: Navigate around underwater obstacles"""
        
        print("📍 SCENARIO 2: Obstacle Avoidance")
        print("Mission: Navigate around underwater structures")
        
        planner = self.configurations['lightweight']  # More agile for obstacles
        
        # Navigate around obstacles using waypoints
        start_pose = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]
        waypoints = [
            [3.0, 2.0, -5.0, 0.0, 0.0, 0.5],    # Around obstacle 1
            [6.0, 4.0, -7.0, 0.0, -0.2, 1.0],   # Dive under obstacle 2
            [9.0, 2.0, -5.0, 0.0, 0.1, 1.5]     # Surface slightly
        ]
        goal_pose = [12.0, 0.0, -5.0, 0.0, 0.0, 0.0]
        
        # Plan trajectory through waypoints
        results = []
        current_pose = start_pose
        
        for waypoint in waypoints + [goal_pose]:
            segment_result = planner.plan_3d_trajectory(current_pose, waypoint)
            if segment_result['success']:
                results.append(segment_result)
                current_pose = waypoint
            else:
                print(f"   ❌ Failed to reach waypoint {waypoint}")
                break
        
        if results:
            total_time = sum(r['total_time'] for r in results)
            total_pulses = sum(r['total_pulses'] for r in results)
            total_length = sum(r['path_length'] for r in results)
            
            print(f"   Obstacle course completed in {total_time:.1f}s")
            print(f"   Total pulses: {total_pulses}, Total distance: {total_length:.1f}m")
            
            # Combine and visualize all segments
            self.visualize_multi_segment_trajectory(results, "Obstacle Avoidance")
            plt.savefig('salp_scenario_2_obstacles.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return {'success': len(results) == len(waypoints) + 1, 'segments': results}
    
    def scenario_3_search_pattern(self):
        """Scenario 3: Systematic search pattern"""
        
        print("📍 SCENARIO 3: Search Pattern")
        print("Mission: Execute systematic search pattern for object detection")
        
        planner = self.configurations['standard']
        
        # Create a lawn-mower search pattern
        search_area = {
            'x_min': 0, 'x_max': 15,
            'y_min': 0, 'y_max': 10,
            'depth': -8.0,
            'spacing': 3.0
        }
        
        # Generate search waypoints
        waypoints = []
        x = search_area['x_min']
        y_direction = 1
        
        while x <= search_area['x_max']:
            if y_direction > 0:
                y_start, y_end = search_area['y_min'], search_area['y_max']
            else:
                y_start, y_end = search_area['y_max'], search_area['y_min']
            
            waypoints.append([x, y_start, search_area['depth'], 0.0, 0.0, 0.0])
            waypoints.append([x, y_end, search_area['depth'], 0.0, 0.0, 0.0])
            
            x += search_area['spacing']
            y_direction *= -1
        
        # Execute search pattern
        results = []
        current_pose = [0.0, 0.0, -2.0, 0.0, 0.0, 0.0]  # Start above search area
        
        for waypoint in waypoints:
            segment_result = planner.plan_3d_trajectory(current_pose, waypoint)
            if segment_result['success']:
                results.append(segment_result)
                current_pose = waypoint
        
        if results:
            total_time = sum(r['total_time'] for r in results)
            total_pulses = sum(r['total_pulses'] for r in results)
            total_length = sum(r['path_length'] for r in results)
            
            print(f"   Search pattern completed in {total_time:.1f}s")
            print(f"   Covered {len(waypoints)} waypoints with {total_pulses} pulses")
            print(f"   Total search distance: {total_length:.1f}m")
            
            self.visualize_search_pattern(results, search_area)
            plt.savefig('salp_scenario_3_search.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return {'success': len(results) > 0, 'segments': results, 'waypoints': waypoints}
    
    def scenario_4_multi_waypoint_survey(self):
        """Scenario 4: Multi-waypoint scientific survey"""
        
        print("📍 SCENARIO 4: Multi-Waypoint Survey")
        print("Mission: Visit multiple scientific sampling points")
        
        planner = self.configurations['heavy_payload']  # Carrying scientific instruments
        
        # Scientific sampling points at various depths
        sampling_points = [
            [2.0, 1.0, -3.0, 0.0, 0.0, 0.0],     # Shallow sample
            [5.0, 4.0, -8.0, 0.0, -0.2, 0.5],    # Mid-depth sample
            [8.0, 2.0, -15.0, 0.0, -0.4, 1.0],   # Deep sample
            [12.0, 6.0, -12.0, 0.0, -0.1, 1.5],  # Return to mid-depth
            [15.0, 3.0, -5.0, 0.0, 0.1, 0.0]     # Final shallow sample
        ]
        
        results = []
        current_pose = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]  # Start near surface
        
        for i, point in enumerate(sampling_points):
            print(f"   Planning route to sampling point {i+1}")
            segment_result = planner.plan_3d_trajectory(current_pose, point)
            
            if segment_result['success']:
                results.append(segment_result)
                current_pose = point
                print(f"   ✅ Route to point {i+1} planned successfully")
            else:
                print(f"   ❌ Failed to plan route to point {i+1}")
        
        if results:
            total_time = sum(r['total_time'] for r in results)
            total_pulses = sum(r['total_pulses'] for r in results)
            
            print(f"   Survey mission planned: {len(results)} segments")
            print(f"   Estimated mission time: {total_time:.1f}s")
            print(f"   Total jet pulses required: {total_pulses}")
            
            self.visualize_survey_mission(results, sampling_points)
            plt.savefig('salp_scenario_4_survey.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return {'success': len(results) == len(sampling_points), 'segments': results}
    
    def scenario_5_emergency_ascent(self):
        """Scenario 5: Emergency ascent to surface"""
        
        print("📍 SCENARIO 5: Emergency Ascent")
        print("Mission: Rapid ascent from depth to surface")
        
        planner = self.configurations['lightweight']  # Need speed and agility
        
        # Emergency ascent from deep water
        start_pose = [10.0, 5.0, -20.0, 0.0, -0.5, 2.0]  # Deep, tilted down
        goal_pose = [12.0, 7.0, 0.0, 0.0, 0.3, 2.0]      # Surface, tilted up
        
        result = planner.plan_3d_trajectory(start_pose, goal_pose)
        
        if result['success']:
            ascent_rate = 20.0 / result['total_time']  # 20m depth change
            print(f"   Emergency ascent completed in {result['total_time']:.1f}s")
            print(f"   Ascent rate: {ascent_rate:.1f}m/s")
            print(f"   Pulses used: {result['total_pulses']}")
            
            # Check if ascent rate is safe (typically < 10m/s for emergency)
            if ascent_rate > 10.0:
                print("   ⚠️  WARNING: Ascent rate exceeds safe limits")
            else:
                print("   ✅ Ascent rate within safe parameters")
            
            planner.visualize_3d_trajectory(result)
            plt.savefig('salp_scenario_5_emergency.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return result
    
    def scenario_6_precision_docking(self):
        """Scenario 6: Precision docking maneuver"""
        
        print("📍 SCENARIO 6: Precision Docking")
        print("Mission: Precise approach and docking with underwater station")
        
        planner = self.configurations['standard']
        
        # Approach docking station with precise final orientation
        start_pose = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]
        
        # Multi-stage approach for precision
        approach_waypoints = [
            [8.0, 2.0, -5.0, 0.0, 0.0, 0.0],      # Initial approach
            [9.5, 1.0, -5.0, 0.0, 0.0, 0.0],      # Close approach
            [9.9, 0.5, -5.0, 0.0, 0.0, 0.0],      # Final approach
        ]
        
        # Final docking position (very precise)
        dock_pose = [10.0, 0.0, -5.0, 0.0, 0.0, 0.0]
        
        results = []
        current_pose = start_pose
        
        # Execute approach phases
        for i, waypoint in enumerate(approach_waypoints + [dock_pose]):
            print(f"   Executing approach phase {i+1}")
            
            # Use more precise parameters for final approach
            if i >= len(approach_waypoints) - 1:  # Final phases
                planner.pulse_duration = 0.08  # Shorter pulses for precision
                planner.min_pulse_interval = 0.02
            
            segment_result = planner.plan_3d_trajectory(current_pose, waypoint)
            
            if segment_result['success']:
                results.append(segment_result)
                current_pose = waypoint
                
                # Check precision for final docking
                if i == len(approach_waypoints):  # Final docking
                    final_pos = np.array([
                        segment_result['trajectory']['position']['x'][-1],
                        segment_result['trajectory']['position']['y'][-1],
                        segment_result['trajectory']['position']['z'][-1]
                    ])
                    target_pos = np.array(dock_pose[:3])
                    precision_error = np.linalg.norm(final_pos - target_pos)
                    
                    print(f"   Docking precision: {precision_error:.3f}m error")
                    if precision_error < 0.1:
                        print("   ✅ High precision docking achieved")
                    else:
                        print("   ⚠️  Docking precision could be improved")
        
        if results:
            total_time = sum(r['total_time'] for r in results)
            print(f"   Docking sequence completed in {total_time:.1f}s")
            
            self.visualize_docking_sequence(results, approach_waypoints + [dock_pose])
            plt.savefig('salp_scenario_6_docking.png', dpi=300, bbox_inches='tight')
            plt.close()
        
        return {'success': len(results) == len(approach_waypoints) + 1, 'segments': results}
    
    def visualize_multi_segment_trajectory(self, results, title):
        """Visualize trajectory with multiple segments"""
        
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
        
        for i, result in enumerate(results):
            if result['success']:
                traj = result['trajectory']
                color = colors[i % len(colors)]
                
                ax.plot(traj['position']['x'], 
                       traj['position']['y'], 
                       traj['position']['z'], 
                       color=color, linewidth=2, label=f'Segment {i+1}')
                
                # Mark start and end of each segment
                ax.scatter(traj['position']['x'][0], 
                          traj['position']['y'][0], 
                          traj['position']['z'][0], 
                          c=color, s=100, marker='o')
                ax.scatter(traj['position']['x'][-1], 
                          traj['position']['y'][-1], 
                          traj['position']['z'][-1], 
                          c=color, s=100, marker='s')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title(f'SALP 3D Trajectory: {title}')
        
        plt.tight_layout()
    
    def visualize_search_pattern(self, results, search_area):
        """Visualize search pattern trajectory"""
        
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot search pattern
        for i, result in enumerate(results):
            if result['success']:
                traj = result['trajectory']
                ax.plot(traj['position']['x'], 
                       traj['position']['y'], 
                       traj['position']['z'], 
                       'b-', linewidth=1, alpha=0.7)
        
        # Show search area boundaries
        x_bounds = [search_area['x_min'], search_area['x_max']]
        y_bounds = [search_area['y_min'], search_area['y_max']]
        z_level = search_area['depth']
        
        # Draw search area rectangle
        corners = [
            [x_bounds[0], y_bounds[0], z_level],
            [x_bounds[1], y_bounds[0], z_level],
            [x_bounds[1], y_bounds[1], z_level],
            [x_bounds[0], y_bounds[1], z_level],
            [x_bounds[0], y_bounds[0], z_level]
        ]
        
        corners = np.array(corners)
        ax.plot(corners[:, 0], corners[:, 1], corners[:, 2], 'r--', linewidth=2, label='Search Area')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title('SALP Search Pattern')
        
        plt.tight_layout()
    
    def visualize_survey_mission(self, results, sampling_points):
        """Visualize scientific survey mission"""
        
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot trajectory segments
        for result in results:
            if result['success']:
                traj = result['trajectory']
                ax.plot(traj['position']['x'], 
                       traj['position']['y'], 
                       traj['position']['z'], 
                       'b-', linewidth=2, alpha=0.7)
        
        # Mark sampling points
        points = np.array(sampling_points)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                  c='red', s=150, marker='*', label='Sampling Points')
        
        # Number the sampling points
        for i, point in enumerate(sampling_points):
            ax.text(point[0], point[1], point[2], f'  {i+1}', fontsize=12)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title('SALP Scientific Survey Mission')
        
        plt.tight_layout()
    
    def visualize_docking_sequence(self, results, waypoints):
        """Visualize precision docking sequence"""
        
        fig = plt.figure(figsize=(15, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot approach trajectory
        colors = ['blue', 'green', 'orange', 'red']
        
        for i, result in enumerate(results):
            if result['success']:
                traj = result['trajectory']
                color = colors[i % len(colors)]
                alpha = 0.5 + 0.5 * (i / len(results))  # Increase opacity for final approach
                
                ax.plot(traj['position']['x'], 
                       traj['position']['y'], 
                       traj['position']['z'], 
                       color=color, linewidth=2+i, alpha=alpha, 
                       label=f'Approach Phase {i+1}')
        
        # Mark waypoints
        points = np.array(waypoints)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                  c='red', s=100, marker='o', label='Waypoints')
        
        # Highlight final docking point
        ax.scatter(waypoints[-1][0], waypoints[-1][1], waypoints[-1][2], 
                  c='gold', s=200, marker='*', label='Docking Station')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title('SALP Precision Docking Sequence')
        
        plt.tight_layout()
    
    def create_summary_visualization(self, results):
        """Create summary visualization of all scenarios"""
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        axes = axes.flatten()
        
        scenario_names = [
            "Surface to Depth",
            "Obstacle Avoidance", 
            "Search Pattern",
            "Survey Mission",
            "Emergency Ascent",
            "Precision Docking"
        ]
        
        for i, (scenario_key, result) in enumerate(results.items()):
            ax = axes[i]
            
            if result and result.get('success', False):
                if 'segments' in result:  # Multi-segment scenario
                    total_time = sum(r['total_time'] for r in result['segments'] if r['success'])
                    total_pulses = sum(r['total_pulses'] for r in result['segments'] if r['success'])
                    total_distance = sum(r['path_length'] for r in result['segments'] if r['success'])
                else:  # Single trajectory scenario
                    total_time = result['total_time']
                    total_pulses = result['total_pulses']
                    total_distance = result['path_length']
                
                # Create bar chart of metrics
                metrics = ['Time (s)', 'Pulses', 'Distance (m)']
                values = [total_time, total_pulses, total_distance]
                
                bars = ax.bar(metrics, values, color=['blue', 'orange', 'green'])
                ax.set_title(f'{scenario_names[i]}\n✅ Success')
                ax.set_ylabel('Value')
                
                # Add value labels on bars
                for bar, value in zip(bars, values):
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{value:.1f}', ha='center', va='bottom')
            else:
                ax.text(0.5, 0.5, f'{scenario_names[i]}\n❌ Failed', 
                       ha='center', va='center', transform=ax.transAxes,
                       fontsize=14, color='red')
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
                ax.axis('off')
        
        plt.tight_layout()
        plt.savefig('salp_scenarios_summary.png', dpi=300, bbox_inches='tight')
        plt.show()

def main():
    """Run all SALP demonstration scenarios"""
    
    mission_planner = SALPMissionPlanner()
    results = mission_planner.run_all_scenarios()
    
    # Print final summary
    print("\n" + "="*60)
    print("🎯 FINAL MISSION SUMMARY")
    print("="*60)
    
    successful_scenarios = sum(1 for r in results.values() 
                             if r and r.get('success', False))
    total_scenarios = len(results)
    
    print(f"Scenarios completed: {successful_scenarios}/{total_scenarios}")
    print(f"Success rate: {successful_scenarios/total_scenarios*100:.1f}%")
    
    if successful_scenarios == total_scenarios:
        print("🏆 ALL SCENARIOS COMPLETED SUCCESSFULLY!")
        print("The SALP 3D trajectory planner is ready for real-world missions.")
    else:
        print("⚠️  Some scenarios need refinement for optimal performance.")
    
    return results

if __name__ == "__main__":
    main()
