# 🌊 Interactive Underwater Vehicle Trajectory Planner

A modern, 3D interactive visualization system for understanding and comparing different underwater vehicle trajectory planning algorithms.

## What This UI Shows You

This interface helps you **visualize and understand** what each algorithm actually does:

### 🎯 **Dubins Curves** (Green Path)
- **What it is**: Smooth paths with minimum turning radius constraints
- **What you'll see**: Curved paths that respect the vehicle's turning limitations
- **Best for**: Vehicles that can't make sharp turns (like torpedoes or AUVs)
- **Visual characteristics**: Smooth, flowing curves with no sharp angles

### 🐙 **Bio-SALP** (Orange Path) 
- **What it is**: Bio-inspired jet propulsion mimicking salp/jellyfish movement
- **What you'll see**: Pulsing, undulating motion with discrete propulsion events
- **Best for**: Soft-bodied robots that use jet propulsion
- **Visual characteristics**: Wavy, pulsing motion that looks like jellyfish swimming

### 🌳 **RRT*** (Purple Path)
- **What it is**: Rapidly-exploring Random Tree for complex obstacle avoidance
- **What you'll see**: Paths that efficiently navigate around obstacles
- **Best for**: Complex environments with many obstacles
- **Visual characteristics**: May look more angular but efficiently avoids obstacles

## 🎮 How to Use

### Getting Started
1. Open `index.html` in a web browser
2. You'll see a beautiful underwater environment with animated water, fish, and kelp
3. The interface loads with helpful instructions in the top-left corner

### Creating Trajectories
1. **Click on the water surface** to place waypoints (underwater vehicle destinations)
2. **First click** = Start point (green sphere)
3. **Subsequent clicks** = Waypoints (orange spheres)
4. The system **automatically plans a path** after you place 2+ waypoints

### Comparing Algorithms
1. Use the **Algorithm buttons** (Dubins/Bio-SALP/RRT*) to switch between planning methods
2. **Watch how the path changes** - this shows you the difference between algorithms!
3. **Adjust the turn radius slider** to see how vehicle constraints affect the path
4. **Add obstacles** with the button to see how RRT* handles complex environments

### Understanding the Results
- **Path Length**: How far the vehicle travels
- **Energy Cost**: Estimated energy consumption (includes turns, depth changes)
- **Planning Time**: How long the algorithm took to compute the path
- **Mission Log**: Real-time updates showing what's happening

### Animation & Visualization
- **Animate Vehicle**: Watch a 3D vehicle follow the planned path
- **Direction Arrows**: White arrows show the direction of travel along the path
- **Different Colors**: Each algorithm uses a different color so you can easily compare

## 🔍 What to Look For

### When Testing Dubins Curves:
- Notice how paths are **always smooth curves**
- Try **reducing the turn radius** - paths become more curved
- **No sharp turns** - the vehicle respects physical constraints

### When Testing Bio-SALP:
- See the **pulsing, wavy motion** - this mimics jellyfish propulsion
- **Higher energy cost** due to the pulsing motion
- **Unique swimming pattern** that's different from traditional vehicles

### When Testing RRT*:
- **Add obstacles** first to see RRT* shine
- Notice how it **finds paths around obstacles**
- May take **longer to plan** but handles complex environments
- **More efficient** in obstacle-rich environments

## 🎯 Learning Exercises

### Exercise 1: Understanding Turning Constraints
1. Place two waypoints far apart
2. Set turn radius to minimum (0.5m) - see tight curves
3. Set turn radius to maximum (5.0m) - see wide, sweeping curves
4. **Question**: How does this affect real underwater vehicles?

### Exercise 2: Algorithm Comparison
1. Create the same waypoint pattern
2. Switch between all three algorithms
3. **Compare**: Path length, energy cost, visual appearance
4. **Question**: Which algorithm would you choose for different missions?

### Exercise 3: Obstacle Navigation
1. Add several obstacles using the "Add Obstacle" button
2. Place waypoints that require navigating around obstacles
3. Compare how Dubins vs RRT* handle the obstacles
4. **Question**: Why might RRT* be better for complex environments?

### Exercise 4: Bio-Inspired Movement
1. Switch to Bio-SALP algorithm
2. Create a long path and animate the vehicle
3. **Observe**: The pulsing motion and how it differs from smooth movement
4. **Question**: What are the advantages/disadvantages of bio-inspired propulsion?

## 🎨 Visual Features

- **Animated water surface** with realistic wave motion
- **Swimming fish** for atmosphere
- **Underwater lighting** with filtered sunlight effects
- **3D vehicle model** with propeller and lights
- **Real-time path visualization** with direction indicators
- **Interactive camera** - drag to rotate, scroll to zoom

## 🚀 Advanced Features

- **Emergency Surface**: Creates immediate path to surface
- **Environment Types**: Switch between Open Water, Coral Reef, Kelp Forest, Thermal Vents
- **Real-time replanning**: Paths update immediately when you change parameters
- **Mission logging**: Track all actions and decisions
- **Keyboard shortcuts**: Space (animate), R (reset camera), C (clear scene)

## 🎓 Educational Value

This tool helps you understand:
- **How different algorithms work** visually
- **Trade-offs** between path smoothness, length, and energy
- **Real-world constraints** like turning radius and obstacles
- **Bio-inspired robotics** concepts
- **Path planning fundamentals** in an intuitive way

## 🔧 Technical Notes

- Built with **Three.js** for 3D graphics
- **Real-time rendering** at 60fps
- **Interactive controls** for immediate feedback
- **Simplified algorithms** optimized for educational visualization
- **Responsive design** works on different screen sizes

---

**Perfect for**: Students, researchers, engineers learning about underwater robotics, path planning algorithms, and bio-inspired systems.

**Goal**: Make complex algorithms intuitive and understandable through interactive visualization!
