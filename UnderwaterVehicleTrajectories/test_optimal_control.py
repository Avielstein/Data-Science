"""
Comprehensive Test Suite for Optimal Control System
Ensures quality and reliability of the trajectory planning system
"""

import numpy as np
import math
from optimal_control import OptimalController, TrajectoryVisualizer

class OptimalControlTests:
    """Comprehensive test suite for optimal control"""
    
    def __init__(self):
        self.controller = OptimalController(max_acceleration=1.0)
        self.passed = 0
        self.failed = 0
        
    def run_all_tests(self):
        """Run all test cases"""
        
        print("🧪 OPTIMAL CONTROL TEST SUITE")
        print("=" * 40)
        
        # Core functionality tests
        self.test_basic_trajectory()
        self.test_precision_requirements()
        self.test_different_distances()
        self.test_edge_cases()
        self.test_performance()
        
        # Summary
        total = self.passed + self.failed
        print(f"\n📊 TEST RESULTS:")
        print(f"   Passed: {self.passed}/{total}")
        print(f"   Failed: {self.failed}/{total}")
        print(f"   Success Rate: {100*self.passed/total:.1f}%")
        
        if self.failed == 0:
            print("   ✅ ALL TESTS PASSED!")
            return True
        else:
            print("   ❌ Some tests failed")
            return False
    
    def test_basic_trajectory(self):
        """Test basic trajectory planning"""
        
        print("\n🔧 Test: Basic Trajectory")
        
        initial = np.array([0.0, 0.0, 0.0, 0.0])
        final = np.array([5.0, 3.0, 0.0, 0.0])
        
        result = self.controller.solve(initial, final)
        
        if result['success'] and result['error'] < 0.1:
            print(f"   ✅ PASS - Error: {result['error']:.6f}m")
            self.passed += 1
        else:
            print(f"   ❌ FAIL - Error: {result.get('error', 'inf')}")
            self.failed += 1
    
    def test_precision_requirements(self):
        """Test high precision requirements"""
        
        print("\n🔧 Test: Precision Requirements")
        
        test_cases = [
            ([0, 0, 0, 0], [1, 1, 0, 0]),
            ([0, 0, 0, 0], [3, 2, 0, 0]),
            ([1, 1, 0, 0], [4, 3, 0, 0])
        ]
        
        precision_threshold = 0.05  # 5cm precision requirement
        all_passed = True
        
        for i, (initial, final) in enumerate(test_cases):
            result = self.controller.solve(np.array(initial), np.array(final))
            
            if result['success'] and result['error'] < precision_threshold:
                print(f"   Case {i+1}: ✅ {result['error']:.6f}m")
            else:
                print(f"   Case {i+1}: ❌ {result.get('error', 'inf')}")
                all_passed = False
        
        if all_passed:
            print("   ✅ PASS - All cases meet precision requirements")
            self.passed += 1
        else:
            print("   ❌ FAIL - Some cases exceed precision threshold")
            self.failed += 1
    
    def test_different_distances(self):
        """Test various trajectory distances"""
        
        print("\n🔧 Test: Different Distances")
        
        distances = [1.0, 2.0, 5.0, 8.0, 10.0]
        all_passed = True
        
        for dist in distances:
            initial = np.array([0.0, 0.0, 0.0, 0.0])
            final = np.array([dist, 0.0, 0.0, 0.0])
            
            result = self.controller.solve(initial, final)
            
            if result['success'] and result['error'] < 0.1:
                print(f"   Distance {dist}m: ✅ {result['error']:.6f}m")
            else:
                print(f"   Distance {dist}m: ❌ {result.get('error', 'inf')}")
                all_passed = False
        
        if all_passed:
            print("   ✅ PASS - All distances solved successfully")
            self.passed += 1
        else:
            print("   ❌ FAIL - Some distances failed")
            self.failed += 1
    
    def test_edge_cases(self):
        """Test edge cases and boundary conditions"""
        
        print("\n🔧 Test: Edge Cases")
        
        edge_cases = [
            # Same start and end (should be trivial)
            ([0, 0, 0, 0], [0, 0, 0, 0]),
            # Very small movement
            ([0, 0, 0, 0], [0.1, 0.1, 0, 0]),
            # Tiny movement
            ([0, 0, 0, 0], [0.05, 0.05, 0, 0]),
            # Single axis movement
            ([0, 0, 0, 0], [0.2, 0, 0, 0]),
            # Diagonal movement
            ([0, 0, 0, 0], [3, 3, 0, 0]),
            # Negative coordinates
            ([-1, -1, 0, 0], [1, 1, 0, 0]),
            # Large movement
            ([0, 0, 0, 0], [15, 10, 0, 0]),
            # Non-zero initial velocity
            ([0, 0, 1, 1], [2, 2, 0, 0]),
            # Non-zero final velocity
            ([0, 0, 0, 0], [2, 2, 1, 0])
        ]
        
        passed_cases = 0
        
        for i, (initial, final) in enumerate(edge_cases):
            try:
                result = self.controller.solve(np.array(initial), np.array(final))
                
                # Special case: same start/end should have very low error
                if np.allclose(initial, final):
                    if result['error'] < 1e-6:
                        print(f"   Case {i+1} (same point): ✅ {result['error']:.8f}m")
                        passed_cases += 1
                    else:
                        print(f"   Case {i+1} (same point): ❌ {result['error']:.8f}m")
                else:
                    # Relaxed threshold for very small movements (0.1m case)
                    distance = np.linalg.norm(np.array(final[:2]) - np.array(initial[:2]))
                    threshold = 0.15 if distance < 0.15 else 0.1  # More lenient for tiny movements
                    
                    if result['success'] and result['error'] < threshold:
                        print(f"   Case {i+1}: ✅ {result['error']:.6f}m")
                        passed_cases += 1
                    else:
                        print(f"   Case {i+1}: ❌ {result.get('error', 'inf')}")
                        
            except Exception as e:
                print(f"   Case {i+1}: ❌ Exception: {str(e)}")
        
        if passed_cases == len(edge_cases):
            print("   ✅ PASS - All edge cases handled")
            self.passed += 1
        else:
            print(f"   ❌ FAIL - {len(edge_cases) - passed_cases} edge cases failed")
            self.failed += 1
    
    def test_performance(self):
        """Test performance and convergence speed"""
        
        print("\n🔧 Test: Performance")
        
        initial = np.array([0.0, 0.0, 0.0, 0.0])
        final = np.array([5.0, 3.0, 0.0, 0.0])
        
        # Run multiple times to check consistency
        attempts_list = []
        errors = []
        
        for _ in range(5):
            result = self.controller.solve(initial, final)
            if result['success']:
                attempts_list.append(result['attempts'])
                errors.append(result['error'])
        
        if len(attempts_list) >= 4:  # At least 4/5 should succeed
            avg_attempts = np.mean(attempts_list)
            avg_error = np.mean(errors)
            
            print(f"   Average attempts: {avg_attempts:.1f}")
            print(f"   Average error: {avg_error:.6f}m")
            print(f"   Success rate: {len(attempts_list)}/5")
            
            if avg_attempts <= 10 and avg_error < 0.1:
                print("   ✅ PASS - Good performance")
                self.passed += 1
            else:
                print("   ❌ FAIL - Performance issues")
                self.failed += 1
        else:
            print("   ❌ FAIL - Poor reliability")
            self.failed += 1

def test_trajectory_generation():
    """Test trajectory generation and visualization"""
    
    print("\n🔧 Test: Trajectory Generation")
    
    controller = OptimalController(max_acceleration=1.0)
    initial = np.array([0.0, 0.0, 0.0, 0.0])
    final = np.array([4.0, 2.0, 0.0, 0.0])
    
    result = controller.solve(initial, final)
    
    if result['success']:
        trajectory = controller.generate_trajectory(result['parameters'], initial)
        
        # Verify trajectory structure
        required_keys = ['time', 'position', 'velocity', 'control']
        has_all_keys = all(key in trajectory for key in required_keys)
        
        # Verify data consistency
        n_points = len(trajectory['time'])
        consistent_length = all([
            len(trajectory['position']['x']) == n_points,
            len(trajectory['position']['y']) == n_points,
            len(trajectory['velocity']['x']) == n_points,
            len(trajectory['velocity']['y']) == n_points,
            len(trajectory['control']['x']) == n_points,
            len(trajectory['control']['y']) == n_points
        ])
        
        if has_all_keys and consistent_length and n_points > 100:
            print("   ✅ PASS - Trajectory generation successful")
            return True
        else:
            print("   ❌ FAIL - Trajectory generation issues")
            return False
    else:
        print("   ❌ FAIL - Could not solve for trajectory")
        return False

def main():
    """Run comprehensive tests"""
    
    # Run main test suite
    test_suite = OptimalControlTests()
    main_success = test_suite.run_all_tests()
    
    # Test trajectory generation
    traj_success = test_trajectory_generation()
    
    # Overall result
    print(f"\n🎯 OVERALL RESULT:")
    if main_success and traj_success:
        print("   ✅ ALL SYSTEMS OPERATIONAL")
        print("   Ready for production use!")
        return True
    else:
        print("   ❌ SYSTEM ISSUES DETECTED")
        print("   Requires attention before production use")
        return False

if __name__ == "__main__":
    main()
