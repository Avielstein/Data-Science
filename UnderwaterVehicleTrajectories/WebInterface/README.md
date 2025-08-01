# 🌊 Underwater Vehicle Trajectory Planning Web Interface

A professional, clean web interface for visualizing and comparing trajectory planning algorithms for underwater vehicles, specifically designed for SALP (bio-inspired) robots.

## ✨ Features

### 🎯 **Algorithm Comparison**
- **Traditional Dubins Curves**: Classical smooth path planning
- **Bio-Inspired Dubins**: SALP-inspired jet propulsion planning  
- **RRT* Underwater**: Advanced sampling-based planning with obstacle avoidance

### 📊 **Comprehensive Visualizations**
- **3D Trajectory Plots**: Side-by-side algorithm comparisons in 3D space
- **Performance Dashboard**: 6-panel analysis including path length, energy cost, planning time, success rates, efficiency radar charts, and scalability analysis
- **Algorithm Deep Dive**: Detailed analysis with curvature, velocity profiles, energy consumption, and quality metrics

### 🧪 **Test Scenarios**
- **Simple Navigation**: Open water with minimal obstacles
- **Moderate Complexity**: Medium obstacle density
- **Complex Environment**: Dense obstacle fields requiring advanced planning

### 🎨 **Professional Interface**
- Clean, modern design inspired by scientific visualization
- Real-time progress tracking
- Interactive tabs and controls
- Export functionality for results
- Responsive design for different screen sizes

## 🚀 Quick Start

### Option 1: Easy Start (Recommended)
```bash
cd UnderwaterVehicleTrajectories/WebInterface
python start_server.py
```

### Option 2: Manual Setup
```bash
cd UnderwaterVehicleTrajectories/WebInterface
pip install -r requirements.txt
python app.py
```

Then open your browser to: **http://localhost:5000**

## 📋 Requirements

- Python 3.8+
- Flask 2.3.3
- NumPy 1.24.3
- Matplotlib 3.7.2
- Seaborn 0.12.2
- SciPy 1.11.2

## 🎮 How to Use

### 1. **Select Test Scenarios**
Choose from three difficulty levels:
- ✅ **Easy**: Simple navigation in open water
- ⚠️ **Medium**: Moderate obstacle density
- 🔴 **Hard**: Complex environment with dense obstacles

### 2. **Choose Algorithms**
Select one or more algorithms to compare:
- 🔵 **Traditional Dubins**: Fast, smooth, predictable
- 🟢 **Bio-Inspired Dubins**: Energy-aware, biomimetic
- 🟣 **RRT* Underwater**: Excellent obstacle handling

### 3. **Run Comparison**
Click "🚀 Run Comparison" and watch the real-time progress:
- ⚙️ Initializing algorithms
- 🎯 Generating trajectories  
- 📊 Creating visualizations
- ✅ Analysis complete!

### 4. **Explore Results**
Navigate through four result tabs:

#### **Overview Tab**
- Summary metrics cards
- Key performance indicators
- Export functionality

#### **Trajectories Tab**  
- 3D visualization of all planned paths
- Start/goal markers
- Obstacle representations
- Algorithm comparison

#### **Performance Tab**
- 6-panel comprehensive dashboard:
  - Path length comparison
  - Energy cost analysis
  - Planning time (log scale)
  - Success rate heatmap
  - Efficiency radar chart
  - Scalability analysis

#### **Analysis Tab**
- Deep dive into individual algorithms
- Curvature analysis
- Velocity profiles
- Energy consumption patterns
- Path quality metrics

## 📊 Understanding the Results

### **Path Length**
- Shorter paths are generally better
- Bio-inspired may have slightly longer paths due to discrete jet propulsion
- RRT* paths vary due to probabilistic nature

### **Energy Cost**
- Bio-inspired algorithms model realistic energy consumption
- Traditional methods assume continuous motion
- Energy-aware planning may trade path length for efficiency

### **Planning Time**
- Traditional Dubins: ~1-3ms (fastest)
- Bio-inspired Dubins: ~2-4ms (fast)
- RRT*: ~0.5-2s (slower but handles complex obstacles)

### **Success Rate**
- Percentage of successful path planning attempts
- Higher is better
- RRT* typically has highest success in complex environments

## 🔧 Technical Details

### **Architecture**
```
WebInterface/
├── app.py                 # Flask web server
├── trajectory_visualizer.py  # Visualization engine
├── templates/
│   └── index.html        # Web interface
├── static/plots/         # Generated visualizations
├── requirements.txt      # Python dependencies
├── start_server.py       # Easy startup script
└── README.md            # This file
```

### **API Endpoints**
- `GET /` - Main dashboard
- `GET /api/scenarios` - Available test scenarios
- `GET /api/algorithms` - Available algorithms
- `POST /api/run_comparison` - Start analysis
- `GET /api/status` - Check progress
- `GET /api/results` - Get results
- `GET /api/algorithm_analysis/<alg>` - Deep dive analysis
- `GET /plots/<filename>` - Serve generated plots
- `GET /api/export_results` - Export data as JSON

### **Visualization Engine**
The `TrajectoryVisualizer` class generates three types of plots:

1. **3D Trajectory Comparison**: Multi-scenario path visualization
2. **Performance Dashboard**: 6-panel comprehensive analysis
3. **Algorithm Deep Dive**: Detailed single-algorithm analysis

## 🎨 Design Philosophy

This interface follows scientific visualization best practices:

- **Clean, Professional Aesthetic**: Inspired by research papers and scientific tools
- **Information Density**: Maximum insight with minimal clutter
- **Interactive Exploration**: Multiple views and drill-down capabilities
- **Export-Ready**: High-quality plots suitable for publications
- **Responsive Design**: Works on desktop, tablet, and mobile

## 🔬 Integration with Research

The web interface seamlessly integrates with your existing trajectory planning algorithms:

- **Dubins Curves**: Uses your `dubins_3d.py` implementation
- **RRT* Variants**: Connects to `rrt_star_underwater.py`
- **SALP Dynamics**: Incorporates bio-inspired vehicle models
- **Real Data**: Generates actual performance metrics, not mock data

## 🚀 Future Enhancements

Potential additions for future versions:

- **Real-time Algorithm Tuning**: Adjust parameters and see immediate results
- **3D Interactive Plots**: Rotate and zoom trajectory visualizations
- **Batch Processing**: Run multiple scenarios automatically
- **Performance Profiling**: Detailed timing and memory analysis
- **Custom Scenarios**: User-defined start/goal positions and obstacles
- **Animation**: Show robot movement along planned paths

## 🐛 Troubleshooting

### **Common Issues**

**Port 5000 already in use:**
```bash
# Kill process using port 5000
lsof -ti:5000 | xargs kill -9
```

**Missing dependencies:**
```bash
pip install -r requirements.txt
```

**Import errors:**
- Ensure you're in the `WebInterface` directory
- Check that the parent `UnderwaterVehicleTrajectories` directory contains the algorithm modules

**Plots not generating:**
- Verify matplotlib backend is working: `python -c "import matplotlib.pyplot as plt; plt.plot([1,2,3]); plt.savefig('test.png')"`
- Check write permissions in `static/plots/` directory

### **Performance Tips**

- **Faster Loading**: Pre-select commonly used scenarios/algorithms
- **Memory Usage**: Clear old plots periodically from `static/plots/`
- **Network**: Run locally for best performance

## 📞 Support

For issues or questions:

1. Check the troubleshooting section above
2. Verify all requirements are installed
3. Ensure the trajectory planning algorithms are working independently
4. Check browser console for JavaScript errors

## 🎉 Enjoy!

This interface provides a powerful, professional way to visualize and compare your underwater vehicle trajectory planning research. The clean design and comprehensive analysis tools make it perfect for:

- **Research Presentations**: High-quality visualizations
- **Algorithm Development**: Quick comparison and iteration
- **Educational Use**: Clear, intuitive interface for learning
- **Publication**: Export-ready plots and data

Happy trajectory planning! 🌊🤖
