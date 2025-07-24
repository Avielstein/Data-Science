#!/usr/bin/env python3
"""
Demo Runner for Underwater Vehicle Trajectory Planning

This script provides easy access to run demonstrations of different algorithms
and simulation environments.
"""

import sys
import os
import argparse
import numpy as np

# Add project directories to path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(project_root, 'Algorithms', 'DubinsCurves'))
sys.path.append(os.path.join(project_root, 'Algorithms', 'RRT_Variants'))
sys.path.append(os.path.join(project_root, 'Simulations'))

def run_dubins_demo():
    """Run 3D Dubins curves demonstration"""
    print("Running 3D Dubins Curves Demo...")
    print("=" * 50)
    
    try:
        from dubins_3d import demo_3d_dubins
        demo_3d_dubins()
    except ImportError as e:
        print(f"Error importing Dubins module: {e}")
        print("Please ensure all dependencies are installed: pip install -r requirements.txt")
    except Exception as e:
        print(f"Error running Dubins demo: {e}")

def run_rrt_demo():
    """Run RRT* demonstration"""
    print("Running Improved RRT* Demo...")
    print("=" * 50)
    
    try:
        from rrt_star_underwater import demo_rrt_star
        demo_rrt_star()
    except ImportError as e:
        print(f"Error importing RRT* module: {e}")
        print("Please ensure all dependencies are installed: pip install -r requirements.txt")
    except Exception as e:
        print(f"Error running RRT* demo: {e}")

def run_simulation_demo():
    """Run comprehensive simulation demonstration"""
    print("Running Comprehensive Simulation Demo...")
    print("=" * 50)
    
    try:
        from underwater_simulation_environment import main
        main()
    except ImportError as e:
        print(f"Error importing simulation module: {e}")
        print("Please ensure all dependencies are installed: pip install -r requirements.txt")
    except Exception as e:
        print(f"Error running simulation demo: {e}")

def run_quick_test():
    """Run a quick test to verify all modules can be imported"""
    print("Running Quick Module Import Test...")
    print("=" * 50)
    
    modules_to_test = [
        ('dubins_3d', 'Algorithms/DubinsCurves'),
        ('rrt_star_underwater', 'Algorithms/RRT_Variants'),
        ('underwater_simulation_environment', 'Simulations')
    ]
    
    all_passed = True
    
    for module_name, module_path in modules_to_test:
        try:
            # Add module path
            full_path = os.path.join(project_root, module_path)
            if full_path not in sys.path:
                sys.path.append(full_path)
            
            # Try to import
            __import__(module_name)
            print(f"✓ {module_name}: Import successful")
            
        except ImportError as e:
            print(f"✗ {module_name}: Import failed - {e}")
            all_passed = False
        except Exception as e:
            print(f"✗ {module_name}: Unexpected error - {e}")
            all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("✓ All modules imported successfully!")
        print("You can now run the individual demos.")
    else:
        print("✗ Some modules failed to import.")
        print("Please install dependencies: pip install -r requirements.txt")
        print("And ensure all algorithm files are in the correct directories.")
    
    return all_passed

def run_basic_algorithm_test():
    """Run basic algorithm functionality test"""
    print("Running Basic Algorithm Functionality Test...")
    print("=" * 50)
    
    try:
        # Test basic numpy functionality
        print("Testing NumPy...")
        test_array = np.array([1, 2, 3])
        print(f"✓ NumPy working: {test_array}")
        
        # Test matplotlib
        print("Testing Matplotlib...")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        plt.close(fig)
        print("✓ Matplotlib working")
        
        # Test basic 3D functionality
        print("Testing 3D plotting...")
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plt.close(fig)
        print("✓ 3D plotting working")
        
        print("\n✓ Basic functionality test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False

def print_project_info():
    """Print project information and available demos"""
    print("Underwater Vehicle Trajectory Planning Project")
    print("=" * 60)
    print()
    print("This project implements and compares trajectory planning algorithms")
    print("for bio-inspired underwater vehicles, particularly SALP robots.")
    print()
    print("Available Demos:")
    print("  1. 3D Dubins Curves - Traditional and bio-inspired path planning")
    print("  2. Improved RRT* - Standard and bio-inspired sampling-based planning")
    print("  3. Comprehensive Simulation - Comparative analysis of all algorithms")
    print("  4. Quick Test - Verify module imports and basic functionality")
    print()
    print("Research Focus:")
    print("  • 3D Dubins curves with neural network acceleration")
    print("  • Improved RRT* with pseudorandom sampling")
    print("  • Bio-inspired adaptations for jet-propelled vehicles")
    print("  • Multi-robot coordination for SALP chains")
    print("  • Energy-efficient trajectory planning")
    print()

def main():
    """Main demo runner with command line interface"""
    parser = argparse.ArgumentParser(
        description="Demo runner for underwater vehicle trajectory planning algorithms",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_demos.py --info                 # Show project information
  python run_demos.py --test                 # Run quick import test
  python run_demos.py --dubins               # Run Dubins curves demo
  python run_demos.py --rrt                  # Run RRT* demo
  python run_demos.py --simulation           # Run full simulation
  python run_demos.py --all                  # Run all demos
        """
    )
    
    parser.add_argument('--info', action='store_true',
                       help='Show project information')
    parser.add_argument('--test', action='store_true',
                       help='Run quick module import test')
    parser.add_argument('--basic', action='store_true',
                       help='Run basic functionality test')
    parser.add_argument('--dubins', action='store_true',
                       help='Run 3D Dubins curves demo')
    parser.add_argument('--rrt', action='store_true',
                       help='Run improved RRT* demo')
    parser.add_argument('--simulation', action='store_true',
                       help='Run comprehensive simulation demo')
    parser.add_argument('--all', action='store_true',
                       help='Run all available demos')
    
    args = parser.parse_args()
    
    # If no arguments provided, show help
    if not any(vars(args).values()):
        print_project_info()
        parser.print_help()
        return
    
    # Show project info
    if args.info:
        print_project_info()
        return
    
    # Run basic test
    if args.basic:
        if not run_basic_algorithm_test():
            return
    
    # Run quick test
    if args.test:
        if not run_quick_test():
            return
    
    # Run individual demos
    if args.dubins or args.all:
        run_dubins_demo()
        print()
    
    if args.rrt or args.all:
        run_rrt_demo()
        print()
    
    if args.simulation or args.all:
        run_simulation_demo()
        print()
    
    if args.all:
        print("All demos completed!")

if __name__ == "__main__":
    main()
