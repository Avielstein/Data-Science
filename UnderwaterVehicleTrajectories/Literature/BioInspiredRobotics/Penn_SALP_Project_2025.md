# SALP: Salp-inspired Approach to Low-Energy Propulsion

**Institution**: University of Pennsylvania, Sung Robotics Lab  
**Principal Investigator**: Prof. Cynthia Sung  
**Project URL**: https://sung.seas.upenn.edu/research/bio-inspired-soft-underwater-robot-that-swims-via-jet-propulsion/

## Project Overview
The SALP project develops bio-inspired soft underwater robots that mimic salp locomotion through jet propulsion. Salps are barrel-shaped marine invertebrates that swim by rapidly changing body cavity volumes, drawing water through front apertures and expelling it under high pressure through rear funnels.

## Key Research Objectives
- **Energy-efficient underwater locomotion** inspired by biological salps
- **Maneuverable underwater robots** for environmental sensing
- **Multi-robot coordination** through physically connected "salp chains"
- **Soft robotics integration** with jet propulsion mechanisms

## Robot Versions and Evolution

### Version 1: Origami Swimmer (2021)
**Key Innovation**: Origami-inspired design using magic ball pattern

#### Technical Specifications
- **Propulsion**: Jet propulsion via volume change (ellipsoid ↔ sphere)
- **Design**: Origami magic ball pattern for shape transformation
- **Actuation**: Tendon mechanism in spine for length control
- **Performance**: 6.7 cm/s forward speed (0.2 body lengths/s)
- **Efficiency**: Cost of transport = 2.0

#### Advantages
- **Simplified fabrication**: Folds from flat sheets into 3D shapes
- **Easy storage/transport**: Compact flat configuration
- **Rapid assembly**: Few hours assembly time
- **Robust design**: Origami structure provides durability

#### Design Features
- **Magic ball origami pattern**: Transforms between ellipsoidal and spherical shapes
- **Volume-based propulsion**: Expansion/contraction creates water jet
- **Tendon actuation**: Spine-based control mechanism
- **Biomimetic locomotion**: Mimics squid/cephalopod swimming

### Version 2: Multi-Robot SALP Platform (2025)
**Key Innovation**: Modular design for multi-robot coordination

#### Enhanced Capabilities
- **Higher thrust generation**: Optimized jetting mechanism
- **Lower drag design**: Improved hydrodynamic efficiency
- **Modular connectivity**: Manual attachment for different arrangements
- **Coordinated swimming**: Multi-robot jet coordination strategies

#### Multi-Robot Performance
- **Synchronous swimming**: 9.0% increase in steady-state velocity
- **Improved acceleration**: 16.6% improvement in transient acceleration
- **Energy efficiency**: Asynchronous mode shows lower cost of transport (COT)
- **Coordination modes**: Various propulsion strategies through jet timing

## Advanced Developments

### Bidirectional Jet Propulsion (2025)
**Innovation**: Active front and rear valves for enhanced control

#### Technical Features
- **Bidirectional propulsion**: Forward and reverse swimming capabilities
- **Self-sensing capability**: Inductance and induced EMF detection
- **Valve state detection**: Open/closed state monitoring
- **Neighbor detection**: Sensing of adjacent robot jetting
- **Decentralized control**: No external sensors or centralized control needed

### Liebau Pumping Integration (2025)
**Innovation**: Valveless propulsion through asymmetric compression

#### Mechanism Details
- **Valveless design**: No mechanical valves or moving parts
- **Asymmetric compression**: Off-center tube compression
- **Frequency control**: Directional control through frequency switching
- **Performance**: 5.25 cm/s forward at 11 Hz, -1.58 cm/s reverse at 15 Hz
- **Quiet operation**: Noise-minimized alternative to propellers

## Research Applications

### Fluid-Structure Interactions
**Collaborative Project**: Leveraging environmental forces for efficiency
- **Micro-vehicle enhancement**: Improved design and control strategies
- **Environmental force utilization**: Power efficiency through passive transport
- **Morphological adaptation**: Shape changes for different flow conditions
- **Long-term operation**: Extended lifespan through efficient control

### Distributed Sensing
**Application Focus**: Long-term environmental monitoring
- **Multi-robot networks**: Coordinated sensing capabilities
- **Environmental adaptation**: Response to changing conditions
- **Scalable deployment**: Large-scale sensing networks
- **Autonomous operation**: Minimal human intervention required

## Key Publications and Results

### Recent Publications (2025)
1. **"Effect of Jet Coordination on Underwater Propulsion with the Multi-Robot SALP System"**
   - IEEE RoboSoft Conference
   - Multi-robot performance analysis
   - Coordination strategy optimization

2. **"Liebau Pumping Enables Valveless Soft Swimmer Robot"**
   - International Symposium on Experimental Robotics (ISER)
   - Novel valveless propulsion mechanism
   - Frequency-based directional control

3. **"Salp-Inspired Bidirectional Jet Propulsion Swimmer with Self-Sensing"**
   - ICRA Workshop on Unconventional Robots
   - Self-sensing capabilities
   - Decentralized coordination

### Foundational Work (2021)
4. **"Origami-inspired robot that swims via jet propulsion"**
   - IEEE Robotics and Automation Letters
   - Original SALP robot design
   - Origami magic ball implementation

## Technical Specifications

### Performance Metrics
- **Swimming Speed**: Up to 6.7 cm/s (0.2 body lengths/s)
- **Propulsion Efficiency**: Cost of transport = 2.0
- **Multi-robot Enhancement**: 9.0% velocity increase, 16.6% acceleration improvement
- **Liebau Pumping**: 5.25 cm/s forward, bidirectional capability

### Design Parameters
- **Body Shape**: Barrel-shaped, volume-variable
- **Propulsion**: Jet-based through volume change
- **Materials**: Soft silicone, origami structures
- **Actuation**: Solenoid, tendon mechanisms
- **Sensing**: Inductance-based, EMF detection

## Trajectory Planning Implications

### Unique Constraints
- **Volume-based propulsion**: Different kinematic model than traditional UUVs
- **Soft body dynamics**: Deformable structure affects path planning
- **Jet propulsion timing**: Discrete thrust events vs. continuous propulsion
- **Multi-robot coordination**: Coupled dynamics in chain configurations

### Planning Considerations
- **Thrust vectoring**: Limited compared to traditional propellers
- **Energy optimization**: Jet timing and coordination for efficiency
- **Obstacle avoidance**: Soft body collision considerations
- **Formation control**: Maintaining chain integrity during maneuvers

## Research Team

### Current Personnel
- Dongsheng Chen (MEAM PhD)
- Zhiyuan (Annie) Yang (MEAM PhD)
- Ryan Stanford (MEAM Undergrad)
- Yi Lu Zheng (ESE Undergrad)
- Benedict Onyekwe (ROBO Master's)
- Jingshuo Li (MEAM Master's)
- Neel Mulay (MEAM Master's)

### Collaborators
- Prof. M. Ani Hsieh (ScalAR lab, UPenn)
- Prof. Eric Forgoston (Montclair State University)
- Prof. Philip Yecko (The Cooper Union)

## Funding Sources
- **National Science Foundation (NSF)**: Grant No. 2121887
- **Office of Naval Research (ONR)**: Award #N00014-23-1-2068

## Future Directions
- **Hardware integration**: Real SALP robot trajectory planning
- **Distributed algorithms**: Multi-robot coordination strategies
- **Environmental sensing**: Integration with navigation systems
- **Long-term autonomy**: Extended operation capabilities
- **Scalable deployment**: Large-scale robot networks

## Relevance to Trajectory Planning Research
The SALP project provides unique insights for trajectory planning in bio-inspired underwater vehicles:
- **Novel propulsion constraints** requiring adapted path planning algorithms
- **Multi-robot coordination** challenges for chain formations
- **Soft robotics considerations** in collision avoidance and path optimization
- **Energy efficiency optimization** through coordinated jet timing
- **Real-world validation platform** for bio-inspired trajectory algorithms
