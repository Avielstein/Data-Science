# SALP Robot Kinematic and Dynamic Modeling

## Overview
The SALP (Salp-inspired Approach to Low-energy Propulsion) robot represents a novel class of bio-inspired underwater vehicles that use jet propulsion through body volume changes. This document analyzes the unique kinematic and dynamic properties relevant to trajectory planning.

## Biological Inspiration

### Salp Locomotion Mechanics
- **Body structure**: Barrel-shaped marine invertebrates
- **Propulsion method**: Rapid body cavity volume changes
- **Water flow**: Intake through front aperture, expulsion through rear funnel
- **Efficiency**: Highly energy-efficient compared to other marine locomotion

### Key Biomechanical Principles
- **Volume-based thrust**: Propulsion force proportional to volume change rate
- **Jet directionality**: Thrust vector determined by funnel orientation
- **Coordination capability**: Natural multi-individual chain formations
- **Energy optimization**: Minimal energy expenditure per unit distance

## SALP Robot Design Evolution

### Version 1: Origami Swimmer (2021)
#### Geometric Properties
- **Shape transformation**: Ellipsoid ↔ Sphere via origami magic ball pattern
- **Volume ratio**: Significant volume change capability
- **Material**: Flexible origami structure with tendon actuation
- **Actuation**: Single tendon mechanism in spine

#### Kinematic Model
**State Variables:**
- Position: p(t) = [x, y, z]ᵀ
- Orientation: θ(t) = [roll, pitch, yaw]ᵀ  
- Body volume: V(t) ∈ [V_min, V_max]
- Volume rate: V̇(t) = dV/dt

**Propulsion Dynamics:**
- Thrust force: F_thrust = ρ × A_exit × V̇² (simplified jet model)
- Thrust direction: Along body longitudinal axis
- Drag force: F_drag = ½ρCd(V)A(V)v² (volume-dependent)

#### Performance Characteristics
- **Swimming speed**: 6.7 cm/s (0.2 body lengths/s)
- **Cost of transport**: 2.0
- **Actuation frequency**: Variable based on desired thrust
- **Turning capability**: Limited to body reorientation

### Version 2: Multi-Robot Platform (2025)
#### Enhanced Capabilities
- **Modular design**: Physical connection capability
- **Improved thrust**: Optimized jetting mechanism
- **Reduced drag**: Streamlined body design
- **Coordination**: Multi-robot synchronization

#### Multi-Robot Dynamics
**Chain Configuration:**
- **Physical coupling**: Rigid or flexible connections
- **Coordinated motion**: Synchronized or asynchronous jetting
- **Formation constraints**: Maintain chain integrity
- **Collective dynamics**: Emergent behavior from individual actions

**Performance Improvements:**
- **Synchronous mode**: 9.0% velocity increase, 16.6% acceleration improvement
- **Asynchronous mode**: Lower cost of transport (COT)
- **Coordination strategies**: Various jet timing patterns

## Kinematic Constraints for Trajectory Planning

### Single Robot Constraints

#### 1. Volume-Based Propulsion
**Constraint**: Discrete thrust events rather than continuous propulsion
- **Jet cycle**: Expansion → Contraction → Recovery
- **Thrust timing**: Intermittent force application
- **Planning implication**: Trajectory segments between thrust events

**Mathematical Model:**
```
V(t) = V_base + A_vol × sin(ωt + φ)
F_thrust(t) = k_thrust × |V̇(t)| × sign(V̇(t))
```

#### 2. Body Shape Variation
**Constraint**: Variable collision boundary during operation
- **Expanded state**: Larger collision radius, higher drag
- **Contracted state**: Smaller collision radius, lower drag
- **Transition dynamics**: Shape change affects maneuverability

**Collision Model:**
```
R_collision(t) = R_base + ΔR × (V(t) - V_min)/(V_max - V_min)
```

#### 3. Limited Steering Authority
**Constraint**: No traditional control surfaces or thrusters
- **Steering mechanism**: Body reorientation between jets
- **Turning radius**: Large compared to body length
- **Maneuverability**: Limited compared to traditional UUVs

### Multi-Robot Chain Constraints

#### 1. Physical Coupling
**Constraint**: Connected robots must maintain formation
- **Connection forces**: Tension/compression in links
- **Relative motion**: Limited degrees of freedom between robots
- **Chain dynamics**: Propagation of forces along chain

**Chain Model:**
```
F_connection,i = k_spring × (L_i - L_nominal) + c_damper × L̇_i
```

#### 2. Coordination Requirements
**Constraint**: Jet timing affects overall chain motion
- **Synchronous mode**: All robots jet simultaneously
- **Asynchronous mode**: Staggered jet timing
- **Phase relationships**: Optimal timing for efficiency

#### 3. Formation Maintenance
**Constraint**: Preserve chain integrity during maneuvers
- **Tension limits**: Avoid chain breakage
- **Collision avoidance**: Inter-robot collision prevention
- **Flexibility**: Allow natural chain deformation

## Dynamic Models

### Single Robot Dynamics
**Equations of Motion:**
```
m × ẍ = F_thrust(t) + F_drag(x̣, V(t)) + F_buoyancy + F_external
I × θ̈ = τ_orientation + τ_drag_rotational + τ_external
```

**Where:**
- m: Robot mass (including added mass)
- I: Moment of inertia tensor
- F_thrust: Jet propulsion force
- F_drag: Hydrodynamic drag (volume-dependent)
- F_buoyancy: Buoyancy force
- τ_orientation: Orientation control torques

### Multi-Robot Chain Dynamics
**Coupled System:**
```
M_chain × q̈ = F_propulsion + F_drag + F_coupling + F_external
```

**Where:**
- M_chain: Chain mass matrix
- q: Generalized coordinates for all robots
- F_coupling: Inter-robot connection forces

## Trajectory Planning Implications

### Modified Dubins Curves
Traditional Dubins curves assume constant velocity and instantaneous turning. SALP robots require:

#### 1. Jet-Propulsion Dubins
- **Discrete thrust segments**: Replace continuous motion with jet cycles
- **Variable velocity**: Account for acceleration/deceleration phases
- **Energy optimization**: Minimize number of jet cycles

#### 2. Volume-Aware Planning
- **Shape-dependent constraints**: Turning radius varies with body volume
- **Collision boundaries**: Dynamic obstacle avoidance regions
- **Drag optimization**: Plan volume states for efficiency

#### 3. Multi-Robot Coordination
- **Chain dynamics**: Consider coupling effects in path planning
- **Synchronization**: Coordinate jet timing for optimal performance
- **Formation control**: Maintain desired chain configuration

### RRT* Adaptations

#### 1. Sampling Strategy
- **Jet timing integration**: Sample considering propulsion cycles
- **Volume state sampling**: Include body shape in configuration space
- **Energy-aware sampling**: Bias toward efficient trajectories

#### 2. Cost Functions
- **Energy-based metrics**: Cost of transport instead of path length
- **Coordination costs**: Multi-robot synchronization penalties
- **Formation maintenance**: Chain integrity preservation

#### 3. Collision Detection
- **Dynamic boundaries**: Volume-dependent collision checking
- **Multi-robot interactions**: Chain collision considerations
- **Soft body modeling**: Deformable collision boundaries

## Control Integration

### Low-Level Control
- **Volume control**: Regulate body expansion/contraction
- **Jet timing**: Control thrust event timing
- **Orientation control**: Body attitude adjustment

### High-Level Planning
- **Trajectory following**: Execute planned paths with jet propulsion
- **Obstacle avoidance**: Real-time path modification
- **Multi-robot coordination**: Synchronize chain behavior

## Experimental Validation

### Performance Metrics
- **Swimming efficiency**: Cost of transport measurement
- **Maneuverability**: Turning radius and response time
- **Coordination quality**: Multi-robot synchronization accuracy
- **Energy consumption**: Power requirements for different maneuvers

### Test Scenarios
- **Single robot navigation**: Basic trajectory following
- **Multi-robot coordination**: Chain formation control
- **Obstacle avoidance**: Dynamic environment navigation
- **Energy optimization**: Efficiency comparison studies

## Research Opportunities

### Current Limitations
1. **Limited maneuverability**: Large turning radius compared to traditional UUVs
2. **Discrete propulsion**: Challenges for smooth trajectory following
3. **Complex multi-robot dynamics**: Difficult coordination in complex environments
4. **Energy optimization**: Need for better understanding of efficiency trade-offs

### Future Directions
1. **Advanced control strategies**: Improved trajectory following algorithms
2. **Adaptive coordination**: Dynamic multi-robot formation control
3. **Energy-optimal planning**: Minimize cost of transport in trajectory design
4. **Environmental adaptation**: Responsive behavior to changing conditions

## Conclusion
SALP robot dynamics present unique challenges and opportunities for trajectory planning. The combination of jet propulsion, volume-based locomotion, and multi-robot coordination requires specialized planning algorithms that account for discrete thrust events, variable body geometry, and chain dynamics. Future research should focus on developing trajectory planning methods that leverage these unique characteristics for energy-efficient, coordinated underwater navigation.
