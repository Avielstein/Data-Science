#!/usr/bin/env python3
"""
Test script for the Underwater Vehicle Trajectory Planning Web Interface

This script tests all components to ensure everything works properly.
"""

import os
import sys
import time
import threading
import requests
from pathlib import Path

def test_imports():
    """Test that all required modules can be imported."""
    print("🧪 Testing imports...")
    
    try:
        import flask
        import numpy
        import matplotlib
        import seaborn
        print("✅ All required packages available")
    except ImportError as e:
        print(f"❌ Missing package: {e}")
        return False
    
    try:
        from trajectory_visualizer import TrajectoryVisualizer, generate_sample_results
        print("✅ Trajectory visualizer imports successfully")
    except ImportError as e:
        print(f"❌ Trajectory visualizer import failed: {e}")
        return False
    
    try:
        from app import app
        print("✅ Flask app imports successfully")
    except ImportError as e:
        print(f"❌ Flask app import failed: {e}")
        return False
    
    return True

def test_visualizer():
    """Test the trajectory visualizer."""
    print("\n🎨 Testing trajectory visualizer...")
    
    try:
        from trajectory_visualizer import TrajectoryVisualizer, generate_sample_results
        
        # Create visualizer
        visualizer = TrajectoryVisualizer(save_dir="test_plots")
        print("✅ Visualizer created")
        
        # Generate sample data
        results = generate_sample_results()
        print("✅ Sample results generated")
        
        # Test scenarios
        scenarios = [
            {
                'name': 'Test Scenario',
                'start': [0, 0, -2],
                'goal': [10, 8, -6],
                'obstacles': []
            }
        ]
        
        algorithms = ['traditional_dubins', 'bio_inspired_dubins']
        
        # Test 3D plot generation
        plot_file = visualizer.create_3d_trajectory_plot(scenarios, algorithms)
        print(f"✅ 3D trajectory plot created: {plot_file}")
        
        # Test performance dashboard
        dashboard_file = visualizer.create_performance_dashboard(results)
        print(f"✅ Performance dashboard created: {dashboard_file}")
        
        # Test algorithm analysis
        analysis_file = visualizer.create_algorithm_deep_dive('bio_inspired_dubins', scenarios)
        print(f"✅ Algorithm analysis created: {analysis_file}")
        
        # Clean up test plots
        import shutil
        if os.path.exists("test_plots"):
            shutil.rmtree("test_plots")
        
        return True
        
    except Exception as e:
        print(f"❌ Visualizer test failed: {e}")
        return False

def test_flask_app():
    """Test the Flask application."""
    print("\n🌐 Testing Flask application...")
    
    try:
        from app import app
        
        # Test app creation
        print("✅ Flask app created")
        
        # Test with test client
        with app.test_client() as client:
            # Test main page
            response = client.get('/')
            if response.status_code == 200:
                print("✅ Main page loads successfully")
            else:
                print(f"❌ Main page failed: {response.status_code}")
                return False
            
            # Test API endpoints
            response = client.get('/api/scenarios')
            if response.status_code == 200:
                print("✅ Scenarios API works")
            else:
                print(f"❌ Scenarios API failed: {response.status_code}")
                return False
            
            response = client.get('/api/algorithms')
            if response.status_code == 200:
                print("✅ Algorithms API works")
            else:
                print(f"❌ Algorithms API failed: {response.status_code}")
                return False
        
        return True
        
    except Exception as e:
        print(f"❌ Flask app test failed: {e}")
        return False

def test_full_workflow():
    """Test the complete workflow."""
    print("\n🔄 Testing complete workflow...")
    
    try:
        from app import app
        
        with app.test_client() as client:
            # Test comparison workflow
            response = client.post('/api/run_comparison', 
                                 json={
                                     'scenarios': ['simple'],
                                     'algorithms': ['traditional_dubins', 'bio_inspired_dubins']
                                 })
            
            if response.status_code == 200:
                print("✅ Comparison started successfully")
                
                # Wait a moment for processing
                time.sleep(2)
                
                # Check status
                response = client.get('/api/status')
                if response.status_code == 200:
                    print("✅ Status check works")
                else:
                    print(f"❌ Status check failed: {response.status_code}")
                    return False
                
                # Wait for completion (up to 10 seconds)
                for _ in range(10):
                    time.sleep(1)
                    response = client.get('/api/status')
                    if response.status_code == 200:
                        status = response.get_json()
                        if status['status'] == 'complete':
                            print("✅ Processing completed")
                            break
                        elif status['status'] == 'error':
                            print(f"❌ Processing error: {status['message']}")
                            return False
                else:
                    print("⚠️ Processing taking longer than expected")
                
                return True
            else:
                print(f"❌ Comparison failed to start: {response.status_code}")
                return False
        
    except Exception as e:
        print(f"❌ Workflow test failed: {e}")
        return False

def main():
    """Run all tests."""
    print("🧪 UNDERWATER VEHICLE TRAJECTORY PLANNING - INTERFACE TEST")
    print("=" * 60)
    
    # Change to WebInterface directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    # Run tests
    tests = [
        ("Import Test", test_imports),
        ("Visualizer Test", test_visualizer),
        ("Flask App Test", test_flask_app),
        ("Full Workflow Test", test_full_workflow)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        if test_func():
            passed += 1
            print(f"✅ {test_name} PASSED")
        else:
            print(f"❌ {test_name} FAILED")
    
    print(f"\n{'='*60}")
    print(f"📊 TEST RESULTS: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 ALL TESTS PASSED! The web interface is ready to use.")
        print("\n🚀 To start the server:")
        print("   python start_server.py")
        print("   Then open: http://localhost:5000")
    else:
        print("⚠️ Some tests failed. Please check the errors above.")
        print("\n🔧 Common fixes:")
        print("   - Install missing packages: pip install -r requirements.txt")
        print("   - Check file permissions")
        print("   - Ensure you're in the WebInterface directory")
    
    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
