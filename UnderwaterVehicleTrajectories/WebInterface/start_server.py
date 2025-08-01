#!/usr/bin/env python3
"""
Quick start script for the Underwater Vehicle Trajectory Planning Web Interface

This script sets up the environment and starts the Flask web server.
"""

import os
import sys
import subprocess
from pathlib import Path

def check_requirements():
    """Check if required packages are installed."""
    try:
        import flask
        import numpy
        import matplotlib
        import seaborn
        print("✅ All required packages are installed")
        return True
    except ImportError as e:
        print(f"❌ Missing required package: {e}")
        return False

def install_requirements():
    """Install required packages."""
    print("📦 Installing required packages...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("✅ Requirements installed successfully")
        return True
    except subprocess.CalledProcessError:
        print("❌ Failed to install requirements")
        return False

def setup_directories():
    """Create necessary directories."""
    directories = [
        "static/plots",
        "templates"
    ]
    
    for directory in directories:
        Path(directory).mkdir(parents=True, exist_ok=True)
    
    print("✅ Directories created")

def main():
    """Main startup function."""
    print("🌊 Underwater Vehicle Trajectory Planning Web Interface")
    print("=" * 60)
    
    # Change to WebInterface directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    # Setup directories
    setup_directories()
    
    # Check requirements
    if not check_requirements():
        print("\n📦 Installing missing requirements...")
        if not install_requirements():
            print("❌ Failed to install requirements. Please install manually:")
            print("   pip install -r requirements.txt")
            return
    
    # Find available port
    import socket
    def find_free_port():
        ports_to_try = [8080, 8000, 3000, 5001, 8888, 9000]
        for port in ports_to_try:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('', port))
                    return port
            except OSError:
                continue
        return 8080  # fallback
    
    port = find_free_port()
    
    # Get local IP address
    def get_local_ip():
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"
    
    local_ip = get_local_ip()
    
    print("\n🚀 Starting web server...")
    print(f"📊 Dashboard available at:")
    print(f"   🏠 Local:    http://localhost:{port}")
    print(f"   🌐 Network:  http://{local_ip}:{port}")
    print("🔄 Press Ctrl+C to stop the server")
    print("-" * 60)
    
    # Start the Flask app
    try:
        from app import app
        app.run(debug=True, host='0.0.0.0', port=port)
    except KeyboardInterrupt:
        print("\n👋 Server stopped by user")
    except Exception as e:
        print(f"\n❌ Error starting server: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure you're in the WebInterface directory")
        print("2. Check that all requirements are installed: pip install -r requirements.txt")
        print("3. Verify that port 5000 is not in use")

if __name__ == "__main__":
    main()
