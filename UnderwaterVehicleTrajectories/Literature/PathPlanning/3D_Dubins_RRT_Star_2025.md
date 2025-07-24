# 3D Dubins Curve-Based Path Planning for UUV in Unknown Environments Using an Improved RRT* Algorithm

**Authors**: Feng Pan, Peng Cui, Bo Cui, Weisheng Yan, Shouxu Zhang  
**Publication**: Journal of Marine Science and Engineering, 2025, 13(7), 1354  
**DOI**: https://doi.org/10.3390/jmse13071354

## Abstract Summary
This paper proposes a novel smooth path planning framework that integrates improved Rapidly-exploring Random Tree* (RRT*) with 3D Dubins curves to efficiently generate feasible and collision-free trajectories for nonholonomic UUVs in unknown underwater environments.

## Key Contributions

### 1. Fast 3D Dubins Curve Length Estimation
- **Backpropagation Neural Network (BPNN)** approach for rapid curve length estimation
- **200x computational speedup** compared to traditional numerical methods
- **Average relative error under 5%** across test groups
- Geometric descriptor vector: Λ_T = [A, B1, B2, B12, r1, r2]

### 2. Improved RRT* Algorithm
- **Pseudorandom sampling strategy** for enhanced obstacle avoidance
- **Terminal node backtracking mechanism** for spatio-temporal feasibility
- **Goal-biased exploration** for improved convergence

### 3. 3D Dubins Curve Analysis
- Focus on **CSC (Curve-Straight-Curve) configuration**
- Mathematical formulation for 3D spatial constraints
- Integration with nonholonomic vehicle dynamics

## Technical Details

### Neural Network Architecture
- **Input Layer**: 6 elements (geometric descriptor)
- **Hidden Layers**: 30 and 22 tansig neurons
- **Output Layer**: 1 purelin neuron for length estimation
- **Training Dataset**: 10,000 3D Dubins curves

### Algorithm Improvements
1. **Pseudorandom Sampling**: Enhanced obstacle avoidance in cluttered environments
2. **Terminal Node Backtracking**: Maintains tree connectivity with moving obstacles
3. **Goal-Biased Exploration**: Activated under specific maturity conditions

### Mathematical Framework
- **Minimum turning radius**: r_min based on UUV kinematic constraints
- **3D Dubins curve construction**: Analytical solution for CSC configuration
- **Collision detection**: Spherical obstacle modeling with safety margins

## Performance Results

### Simulation Scenarios
1. **Scenario 1**: 22 static obstacles in concave configuration
2. **Scenario 2**: 16 static + 4 moving obstacles
3. **Scenario 3**: High-density obstacle environment

### Comparative Performance
- **Path Length**: Significantly shorter than baseline algorithms
- **Computation Time**: 1.54s average (vs 5.54s for 3D RRT*)
- **Success Rate**: High reliability in dense obstacle environments
- **Smoothness**: Lowest average curvature (0.16 m^-1)

## Key Algorithms

### DRRT* (Proposed Method)
- Combines path reuse, rough path planning, and path reshaping
- Reactive planning framework for environmental uncertainty
- Efficient handling of both static and dynamic obstacles

### Path Optimization
- **Path Reuse**: Retains feasible segments from previous planning
- **Rough Path Planning**: Enhanced RRT* for rapid trajectory generation
- **Path Reshaping**: Smooth kinematically feasible maneuvers

## Applications
- **UUV Recovery Operations**: Precise docking with recovery platforms
- **Autonomous Navigation**: Unknown 3D underwater environments
- **Obstacle Avoidance**: Real-time collision-free path generation

## Limitations and Future Work
- Current framework tested on moderate-scale environments
- Scalability challenges for extremely large domains (>1000m range)
- Future integration with advanced perception systems

## Relevance to Bio-Inspired Vehicles
This work provides foundational algorithms that could be adapted for:
- **Jet-propelled vehicles**: Modifying Dubins constraints for different propulsion
- **Soft robotics**: Accounting for deformable body dynamics
- **Multi-robot systems**: Coordinated path planning for robot chains

## Implementation Notes
- MATLAB 2019b implementation
- Tested on Intel i7-9750H CPU, 16GB RAM
- Parameters: r_min = 1m, r_scan = 7m for UUV model
