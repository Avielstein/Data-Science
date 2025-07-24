# Underwater Vehicle Trajectories Research Project

## Overview
This project focuses on trajectory planning and path optimization for bio-inspired underwater vehicles, with particular emphasis on jet-propelled soft robots like the SALP (Salp-inspired Approach to Low-energy Propulsion) system.

## Research Focus Areas

### 1. Path Planning Algorithms
- **3D Dubins Curves**: Optimal path planning for nonholonomic underwater vehicles
- **RRT* Variants**: Rapidly-exploring Random Tree algorithms for unknown environments
- **Neural Network Acceleration**: Fast curve length estimation and path optimization

### 2. Bio-Inspired Vehicle Dynamics
- **Jet Propulsion Modeling**: Understanding salp/cephalopod-inspired locomotion
- **Soft Robotics Considerations**: Deformable body dynamics in trajectory planning
- **Multi-Vehicle Coordination**: Coordinated swimming in robot chains

### 3. Trajectory Optimization
- **Energy Efficiency**: Minimizing cost of transport
- **Obstacle Avoidance**: Real-time collision detection and avoidance
- **Adaptive Replanning**: Dynamic path adjustment in changing environments

## Key Research Papers

### Path Planning & Trajectory Generation
1. **"3D Dubins Curve-Based Path Planning for UUV in Unknown Environments Using an Improved RRT* Algorithm"** (2025)
   - Neural network-based fast Dubins curve length estimation
   - Improved RRT* with pseudorandom sampling and goal-biased exploration
   - 3D underwater obstacle avoidance

2. **"Feedback-Dubins-RRT Recovery Path Planning of UUV in an Underwater Obstacle Environment"** (2020)
   - Combines global RRT planning with local Dubins curve generation
   - Feedback control loop approach for path planning
   - Recovery path planning for known start/end vectors

### Bio-Inspired Underwater Robotics
3. **Penn SALP Project** - Salp-inspired soft underwater robots
   - Origami-inspired design with jet propulsion
   - Multi-robot coordination capabilities
   - Energy-efficient locomotion strategies

## Project Structure

```
UnderwaterVehicleTrajectories/
├── Literature/                    # Research papers and references
│   ├── PathPlanning/             # Path planning algorithms
│   ├── BioInspiredRobotics/      # Bio-inspired vehicle research
│   └── TrajectoryOptimization/   # Optimization techniques
├── Algorithms/                   # Implementation and analysis
│   ├── DubinsCurves/            # 3D Dubins curve algorithms
│   ├── RRT_Variants/            # RRT* and variants
│   └── NeuralNetworkAcceleration/ # ML-based acceleration
├── VehicleModels/               # Vehicle dynamics and modeling
│   ├── SALP_Dynamics/           # SALP robot modeling
│   ├── JetPropulsion/           # Jet propulsion mechanics
│   └── MultiRobotSystems/       # Coordinated systems
├── Simulations/                 # Simulation environments and results
└── Documentation/               # Additional documentation
```

## Key Research Questions

1. **Adaptation for Jet Propulsion**: How can traditional Dubins curve planning be adapted for jet-propelled vehicles with different kinematic constraints?

2. **Soft Body Dynamics**: What are the trajectory planning implications of soft, deformable robot bodies compared to rigid vehicles?

3. **Multi-Robot Coordination**: How does coordinated swimming in robot chains affect individual trajectory planning and optimization?

4. **Hybrid Approaches**: Can we develop methods combining neural network efficiency with analytical precision for real-time applications?

5. **Energy Optimization**: How can we minimize the cost of transport while maintaining trajectory accuracy and obstacle avoidance?

## Research Methodology

### Phase 1: Literature Analysis
- Comprehensive review of path planning algorithms
- Analysis of bio-inspired underwater vehicle designs
- Identification of research gaps and opportunities

### Phase 2: Algorithm Development
- Implementation of 3D Dubins curve algorithms
- Development of improved RRT* variants
- Neural network acceleration techniques

### Phase 3: Vehicle Modeling
- SALP robot dynamics modeling
- Jet propulsion system analysis
- Multi-robot coordination strategies

### Phase 4: Simulation and Validation
- Comparative algorithm performance analysis
- Energy efficiency studies
- Real-world applicability assessment

## Implementation Status

### Completed ✅
- [x] Project structure and documentation
- [x] Literature review and analysis
- [x] **3D Dubins Curves Implementation** - Traditional and bio-inspired variants
- [x] **Improved RRT* Implementation** - Standard and bio-inspired variants
- [x] **SALP Robot Dynamics Modeling** - Kinematic and dynamic models
- [x] **Comprehensive Simulation Environment** - Comparative testing framework
- [x] **Neural Network Acceleration** - Fast curve length estimation
- [x] **Multi-robot Coordination** - Chain formation algorithms

### Ready for Testing 🧪
- [x] Algorithm implementations with working demos
- [x] Simulation environment with visualization
- [x] Performance comparison tools
- [x] Energy efficiency analysis

## Getting Started

### Quick Start
```bash
# Install dependencies
pip install -r requirements.txt

# Run all demos
python run_demos.py --all

# Or run individual components
python run_demos.py --dubins      # 3D Dubins curves demo
python run_demos.py --rrt         # RRT* algorithms demo
python run_demos.py --simulation  # Full simulation comparison
```

### Step-by-Step Exploration
1. **Review Literature**: Start with `Literature/` directory for background
2. **Understand Algorithms**: Examine implementations in `Algorithms/`
3. **Study Vehicle Models**: Review dynamics in `VehicleModels/`
4. **Run Simulations**: Test algorithms in `Simulations/`
5. **Analyze Results**: Compare performance metrics and visualizations

### Key Files to Explore
- `Algorithms/DubinsCurves/dubins_3d.py` - 3D Dubins implementation
- `Algorithms/RRT_Variants/rrt_star_underwater.py` - RRT* variants
- `Simulations/underwater_simulation_environment.py` - Comprehensive testing
- `run_demos.py` - Easy demo runner with multiple options

## Future Directions

- Integration with real SALP robot hardware
- Development of distributed planning algorithms
- Environmental sensing integration
- Long-term autonomous operation strategies

---

*Last Updated: January 2025*
