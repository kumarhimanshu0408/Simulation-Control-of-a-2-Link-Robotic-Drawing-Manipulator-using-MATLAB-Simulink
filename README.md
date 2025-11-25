# 2-Link Drawing Manipulator - Project Documentation

## Project Overview

This project implements a **2-Link Planar Drawing Manipulator** with a comprehensive MATLAB GUI for trajectory planning, inverse kinematics computation, animation, and Simulink integration. The system allows users to generate trajectories from various sources (built-in shapes, custom paths, or SVG files), compute joint angles using inverse kinematics, visualize robot motion through animation, and export data for Simulink simulation.

### Key Features
- **Interactive GUI** for parameter configuration and trajectory generation
- **Multiple Trajectory Types**: Circle, Line, Square, Custom (click-to-draw), and SVG file import
- **Robust SVG Parser** supporting common SVG path commands (M, L, H, V, C, S, Q, T, A, Z)
- **Inverse Kinematics Solver** with elbow-up/elbow-down configuration
- **Real-time Animation** with time-based motion control
- **Path Smoothing** using quintic B-splines and interpolation
- **Simulink Export** with timeseries objects for control system integration
- **Diagnostic Plots** for motion analysis and validation

---

## System Architecture

The project is organized into modular components within the `Robotc` folder:

```
Robotc/
├── main_gui_final.m          # Main GUI application (top module)
├── inverse_kinematics.m     # IK solver for 2-link manipulator
├── forward_kinematics.m     # FK computation for verification
├── svg_parser.m            # SVG path parsing and curve sampling
├── trajectory_generation.m  # Trajectory creation and resampling
├── smoothing.m             # Joint angle smoothing algorithms
└── plotting.m              # Visualization and diagnostic plots
```

---

## Module Descriptions

### 1. `main_gui_final.m` - Main GUI Application

**Purpose**: Top-level module that creates the user interface and coordinates all system components.

**Key Components**:
- **GUI Setup**: Creates figure window with workspace axes and control panel
- **Parameter Management**: Handles user inputs (arm lengths, trajectory type, sampling parameters, etc.)
- **Callback Functions**: Implements button actions and user interactions

**Main Functions**:
- `onGenerate()`: Generates trajectory based on selected type
- `onComputeIK()`: Computes inverse kinematics for trajectory points
- `onAnimate()`: Animates robot motion with time-based control
- `onMoveOn()`: Exports data to base workspace for Simulink
- `onReset()`: Resets GUI to default parameters
- `onUploadSVG()`: Loads and processes SVG files

**Data Flow**:
1. User sets parameters → GUI stores in `data.params`
2. Generate trajectory → Creates `data.traj.x` and `data.traj.y`
3. Compute IK → Generates `data.joint.theta1` and `data.joint.theta2`
4. Animate → Updates visualization using joint angles
5. Move on → Exports to workspace for external use

---

### 2. `inverse_kinematics.m` - IK Solver

**Purpose**: Computes joint angles (θ₁, θ₂) for a desired end-effector position (px, py).

**Mathematical Model**:
The 2-link manipulator has:
- Link 1: Length L₁, angle θ₁
- Link 2: Length L₂, angle θ₂
- End-effector position: (px, py)

**Algorithm**:
1. **Reachability Check**: 
   - Maximum reach: `r ≤ L₁ + L₂`
   - Minimum reach: `r ≥ |L₁ - L₂|`
   - If outside workspace, return `ok = false`

2. **Joint 2 Calculation**:
   - Using cosine law: `cos(θ₂) = (r² - L₁² - L₂²) / (2·L₁·L₂)`
   - Elbow configuration determines sign:
     - **Elbow down**: `θ₂ = atan2(-√(1-cos²θ₂), cos(θ₂))`
     - **Elbow up**: `θ₂ = atan2(√(1-cos²θ₂), cos(θ₂))`

3. **Joint 1 Calculation**:
   - Geometric relationship: `θ₁ = atan2(py, px) - atan2(k₂, k₁)`
   - Where: `k₁ = L₁ + L₂·cos(θ₂)`, `k₂ = L₂·sin(θ₂)`

**Input**: `(px, py, L1, L2, elbow)`
**Output**: `(theta1, theta2, ok)`

---

### 3. `forward_kinematics.m` - FK Computation

**Purpose**: Computes end-effector position from joint angles (used for verification).

**Mathematical Model**:
```
x₁ = L₁·cos(θ₁)
y₁ = L₁·sin(θ₁)
x₂ = x₁ + L₂·cos(θ₁ + θ₂)
y₂ = y₁ + L₂·sin(θ₁ + θ₂)
```

**Usage**: Validates IK solutions and generates end-effector trajectory for visualization.

---

### 4. `svg_parser.m` - SVG Path Parser

**Purpose**: Parses SVG files and converts path commands into coordinate points.

**Supported SVG Commands**:
- **M/m**: Move to (absolute/relative)
- **L/l**: Line to
- **H/h**: Horizontal line
- **V/v**: Vertical line
- **C/c**: Cubic Bézier curve
- **S/s**: Smooth cubic Bézier
- **Q/q**: Quadratic Bézier
- **T/t**: Smooth quadratic Bézier
- **A/a**: Elliptical arc
- **Z/z**: Close path

**Processing Pipeline**:
1. **XML Parsing**: Reads SVG file using `xmlread()`
2. **Path Extraction**: Collects path data from `<path>`, `<polyline>`, `<polygon>` elements
3. **Tokenization**: Uses regex to separate commands and numeric values
4. **Command Processing**: Interprets each command and generates point samples
5. **Curve Sampling**: 
   - Cubic curves: `sampleCubic()` - Bézier interpolation
   - Quadratic curves: `sampleQuadratic()` - Quadratic interpolation
   - Arcs: `svg_arc_to_poly_fast()` - Elliptical arc approximation
6. **Normalization**: Centers and scales path to unit size

**Helper Functions**:
- `read_numbers()`: Extracts numeric values from token stream
- `safe_append()`: Safely appends points to coordinate arrays
- `sampleCubic()`: Samples cubic Bézier curves
- `sampleQuadratic()`: Samples quadratic Bézier curves
- `svg_arc_to_poly_fast()`: Converts elliptical arcs to polyline

---

### 5. `trajectory_generation.m` - Trajectory Creation

**Purpose**: Generates and processes trajectories for robot motion.

**Functions**:

#### `generate_builtin(type, N)`
Creates predefined trajectories:
- **Circle**: Parametric circle with radius 0.5
- **Line**: Horizontal line from -0.5 to 0.5
- **Square**: Closed square path with equal side distribution
- **Custom**: Interactive point selection using `ginput()`, then spline interpolation

#### `resample_and_smooth_path_param(x, y, N, curveSamples)`
Resamples and smooths trajectory:
1. **Duplicate Removal**: Eliminates consecutive duplicate points
2. **Cumulative Distance**: Creates arc-length parameterization
3. **Unique Parameterization**: Ensures monotonic parameter values
4. **PCHIP Interpolation**: Resamples to N points preserving shape
5. **Smoothing**: Applies moving median and mean filters to reduce noise

#### `translate_and_scale_to_init(x_norm, y_norm, initX, initY, L1, L2)`
Scales and translates normalized paths:
- Computes scale factor to fit within robot workspace (80% of reach)
- Translates to initial position (initX, initY)

---

### 6. `smoothing.m` - Joint Angle Smoothing

**Purpose**: Smooths joint angle trajectories to ensure continuous, differentiable motion.

#### `fill_and_interp_nans(a)`
Handles unreachable points:
- Identifies valid (non-NaN) values
- Uses PCHIP interpolation to fill gaps
- Extrapolates at boundaries

#### `quintic_bspline_joint_smooth(th1, th2, Nout)`
High-order smoothing:
1. **B-spline Method** (if Spline Toolbox available):
   - Creates 6th-order B-spline basis
   - Fits smooth curves to joint angles
   - Evaluates at Nout points
2. **Fallback Method**:
   - Uses cubic spline interpolation if B-spline unavailable

**Benefits**:
- Reduces jerk in motion
- Ensures smooth velocity and acceleration profiles
- Prevents sudden angle changes

---

### 7. `plotting.m` - Visualization

**Purpose**: Provides visual feedback and diagnostic analysis.

#### `update_links_plot(d, idx)`
Real-time robot visualization:
- Draws workspace boundaries (outer and inner circles)
- Plots desired trajectory (gray dashed line)
- Shows actual end-effector path (red line)
- Displays current robot pose with links and joints
- Updates joint angles in status text

#### `control_analysis_exhibit(d)`
Comprehensive diagnostic plots (6 subplots):
1. **Initial Pose**: Robot configuration at start
2. **End-effector Path**: Desired vs actual trajectory
3. **Joint Angles**: θ₁ and θ₂ vs time (degrees)
4. **Joint Velocities**: Angular velocities vs time (deg/s)
5. **Tracking Error**: End-effector position error vs time
6. **PD-like Torques**: Estimated control torques (arbitrary units)

---

## Code Logic Flow

### Complete Workflow

```
1. USER INPUT
   ├── Set arm lengths (L1, L2)
   ├── Choose trajectory type
   ├── Configure sampling parameters
   └── Set initial position (initX, initY)

2. TRAJECTORY GENERATION
   ├── Built-in: generate_builtin() → parametric equations
   ├── SVG: parseSVGPath_enhanced_fixed() → path parsing
   └── translate_and_scale_to_init() → workspace fitting
   └── resample_and_smooth_path_param() → uniform sampling

3. INVERSE KINEMATICS
   ├── For each trajectory point:
   │   └── inverse_kinematics() → (θ₁, θ₂)
   ├── fill_and_interp_nans() → handle unreachable points
   ├── unwrap() → remove angle discontinuities
   └── quintic_bspline_joint_smooth() → smooth motion

4. ANIMATION
   ├── Time-based frame selection
   ├── update_links_plot() → visual update
   └── forward_kinematics_traj() → verification

5. EXPORT
   ├── Create time vectors
   ├── Export joint angles (theta1_ref, theta2_ref)
   ├── Export end-effector (ee_ref)
   ├── Create timeseries objects
   └── control_analysis_exhibit() → diagnostics
```

### Key Algorithms

#### Inverse Kinematics Algorithm
```matlab
1. Compute distance: r = √(px² + py²)
2. Check reachability: |L1-L2| ≤ r ≤ L1+L2
3. Solve for θ₂ using cosine law
4. Solve for θ₁ using geometric relationships
5. Return angles with validity flag
```

#### Trajectory Resampling Algorithm
```matlab
1. Remove duplicate points
2. Compute cumulative arc length
3. Create unique parameterization
4. Normalize to [0,1]
5. PCHIP interpolate to N points
6. Apply smoothing filters
```

#### SVG Parsing Algorithm
```matlab
1. Parse XML structure
2. Extract path data strings
3. Tokenize commands and numbers
4. Process each command:
   - Move/Line: Direct point addition
   - Curves: Sample using Bézier equations
   - Arcs: Convert to polyline
5. Normalize coordinates
```

---

## Usage Instructions

### Getting Started

1. **Navigate to Robotc folder**:
   ```matlab
   cd Robotc
   ```

2. **Run the GUI**:
   ```matlab
   main_gui_final
   ```

3. **Basic Workflow**:
   - Set arm lengths (L1, L2)
   - Choose trajectory type
   - Click "Generate Trajectory"
   - Click "Compute IK"
   - Click "Animate" to visualize
   - Click "Move on →" to export for Simulink

### Parameter Configuration

- **Arm L1, L2**: Link lengths in meters (default: 1.0 m each)
- **Trajectory Type**: Circle, Line, Square, Custom, or SVG
- **Initial X, Y**: Translation offset for trajectory (default: 0, 0)
- **Curve Samples**: Points per curve segment for SVG (default: 40)
- **Resample N**: Total trajectory points (default: 2000)
- **Total Time**: Animation duration in seconds (default: 8 s)
- **Elbow Choice**: Elbow-up or elbow-down configuration

### SVG File Import

1. Click "Upload SVG Path"
2. Select SVG file from dialog
3. System automatically:
   - Parses path data
   - Normalizes coordinates
   - Scales to fit workspace
   - Resamples to N points
4. Adjust Initial X, Y if trajectory is outside reach

### Custom Trajectory

1. Select "Custom" from trajectory type
2. Click "Generate Trajectory"
3. Left-click on workspace to add points
4. Press ENTER or right-click to finish
5. System interpolates smooth path through points

---

## Technical Details

### Workspace Limits

The robot workspace is defined by:
- **Outer boundary**: Circle with radius `L₁ + L₂`
- **Inner boundary**: Circle with radius `|L₁ - L₂|` (if L₁ ≠ L₂)
- **Reachable region**: Annular region between boundaries

### Coordinate System

- **Origin**: Base joint (0, 0)
- **X-axis**: Horizontal (right = positive)
- **Y-axis**: Vertical (up = positive)
- **Angles**: Counter-clockwise positive (standard convention)

### Trajectory Processing

1. **Normalization**: SVG paths are centered and scaled to unit size
2. **Scaling**: Paths scaled to 80% of maximum reach
3. **Translation**: Applied to initial position (initX, initY)
4. **Resampling**: Uniform arc-length parameterization
5. **Smoothing**: Moving filters reduce numerical noise

### Joint Angle Smoothing

- **Gap Filling**: PCHIP interpolation for unreachable points
- **Unwrapping**: Removes 2π discontinuities
- **B-spline Smoothing**: 6th-order B-spline for C⁵ continuity
- **Fallback**: Cubic spline if B-spline unavailable

### Animation Control

- **Time-based**: Uses actual elapsed time for frame selection
- **Frame Skipping**: Updates only when time advances significantly
- **Real-time Display**: Shows current joint angles in status text
- **Smooth Motion**: Interpolated joint angles ensure fluid animation

### Simulink Integration

Exported variables:
- `theta1_ref`: [time, angle] array for joint 1
- `theta2_ref`: [time, angle] array for joint 2
- `ee_ref`: [time, x, y] array for end-effector
- `L1_sim`, `L2_sim`: Link lengths
- `Kp`, `Kd`: PD controller gains
- `ts_theta1`, `ts_theta2`, `ts_ee`: Timeseries objects

---

## Mathematical Foundations

### Inverse Kinematics Derivation

For a 2-link planar manipulator:

**Position constraint**:
```
px = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂)
py = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂)
```

**Distance from base**:
```
r² = px² + py² = L₁² + L₂² + 2·L₁·L₂·cos(θ₂)
```

**Solving for θ₂**:
```
cos(θ₂) = (r² - L₁² - L₂²) / (2·L₁·L₂)
θ₂ = ±arccos(cos(θ₂))
```

**Solving for θ₁**:
Using geometric relationships:
```
θ₁ = atan2(py, px) - atan2(L₂·sin(θ₂), L₁ + L₂·cos(θ₂))
```

### Forward Kinematics

**Joint positions**:
```
x₁ = L₁·cos(θ₁)
y₁ = L₁·sin(θ₁)
```

**End-effector position**:
```
x₂ = x₁ + L₂·cos(θ₁ + θ₂)
y₂ = y₁ + L₂·sin(θ₁ + θ₂)
```

### Bézier Curve Sampling

**Cubic Bézier** (4 control points: P₀, P₁, P₂, P₃):
```
P(t) = (1-t)³·P₀ + 3(1-t)²t·P₁ + 3(1-t)t²·P₂ + t³·P₃
```

**Quadratic Bézier** (3 control points: P₀, P₁, P₂):
```
P(t) = (1-t)²·P₀ + 2(1-t)t·P₁ + t²·P₂
```

---

## Error Handling

The system includes robust error handling:

1. **Reachability Checks**: Validates workspace limits before IK computation
2. **NaN Handling**: Interpolates unreachable points
3. **Empty Trajectory**: Prevents computation on invalid data
4. **SVG Parsing Errors**: Catches XML and path parsing exceptions
5. **Toolbox Availability**: Falls back to standard functions if toolboxes unavailable

---

## Dependencies

### Required MATLAB Toolboxes
- **Base MATLAB**: Core functionality
- **Signal Processing Toolbox**: For `movmedian()` and `movmean()` (optional, has fallbacks)

### Optional Toolboxes
- **Spline Toolbox**: For B-spline smoothing (falls back to cubic spline if unavailable)
- **Simulink**: For importing exported data (not required for GUI operation)

---

## Project Structure Summary

| Module | Purpose | Key Functions |
|--------|---------|---------------|
| `main_gui_final.m` | GUI and coordination | Callbacks, UI setup |
| `inverse_kinematics.m` | IK computation | `inverse_kinematics()` |
| `forward_kinematics.m` | FK computation | `forward_kinematics_traj()` |
| `svg_parser.m` | SVG file processing | `parseSVGPath_enhanced_fixed()` |
| `trajectory_generation.m` | Path creation | `generate_builtin()`, `resample_and_smooth_path_param()` |
| `smoothing.m` | Motion smoothing | `fill_and_interp_nans()`, `quintic_bspline_joint_smooth()` |
| `plotting.m` | Visualization | `update_links_plot()`, `control_analysis_exhibit()` |

---

## Future Enhancements

Potential improvements:
- 3D manipulator support
- Dynamic trajectory optimization
- Collision avoidance
- Real-time control interface
- Additional trajectory types
- Export to other formats (ROS, URDF)

---

## Author Notes

This project demonstrates:
- **Robotics fundamentals**: Kinematics, trajectory planning
- **GUI development**: MATLAB App Designer concepts
- **File processing**: SVG parsing and data extraction
- **Numerical methods**: Interpolation, smoothing, optimization
- **System integration**: Simulink data export

The modular architecture allows easy extension and maintenance while keeping the codebase organized and understandable.



# Simulink Model: `robot_tracker.slx` — Components, Functionality, and Integration

**Model file location:**  
`sandbox:/mnt/data/robot_tracker.slx`

This section documents the complete Simulink model used in the project.  
It *adds* to the existing README without removing any content.

---

## ⭐ Purpose of the Simulink Model

The GUI (`main_gui_final.m`) exports joint trajectories, EE trajectories, and robot parameters to the MATLAB base workspace.  
The Simulink model `robot_tracker.slx` uses this data to:

- Track the generated trajectory using a PD controller.
- Simulate joint behavior.
- Compute forward kinematics.
- Visualize the end-effector path.
- Log simulation data for analysis.

---

## 📥 Workspace Variables Used by the Model

The GUI automatically exports the following variables:

| Variable Name | Type | Purpose |
|---------------|-------|---------|
| `theta1_ref` | Nx2 array `[t, θ1]` | Joint 1 reference |
| `theta2_ref` | Nx2 array `[t, θ2]` | Joint 2 reference |
| `ee_ref` | Nx3 array `[t, x, y]` | Desired end‑effector path |
| `ts_theta1` | timeseries | Alt input for Simulink |
| `ts_theta2` | timeseries | Alt input for Simulink |
| `ts_ee` | timeseries | Alt XY reference |
| `L1_sim`, `L2_sim` | scalars | Link lengths |
| `Kp = 10`, `Kd = 2` | scalars | Controller gains |

These variables allow the Simulink model to reproduce and analyze motion.

---

## 🧩 Simulink Model — Block-Level Description

Below is the **complete functional description** of the Simulink model expected for this project.

### 1. **From Workspace Blocks**
These blocks import simulation references:

- **From Workspace (theta1_ref)**  
  Reads `[time, theta1]` for Joint 1.

- **From Workspace (theta2_ref)**  
  Reads Joint 2 trajectory.

- **From Workspace (ee_ref)** *(optional)*  
  Useful for visual comparison in XY plot.

These signals drive the controllers.

---

### 2. **PD Controller Subsystems**

Each joint has a dedicated subsystem:

#### **Joint 1 PD Controller**
- Inputs: `θ1_ref`, `θ1_meas`
- Computes: `u1 = Kp (e) + Kd (de/dt)`
- Where `e = θ1_ref – θ1_meas`

#### **Joint 2 PD Controller**
Same structure for joint 2.

Both use workspace parameters `Kp`, `Kd`.

---

### 3. **Plant Model / Robot Dynamics**

This subsystem simulates joint motion.

Common setups:

- **Simple integration model**:  
  `θ_dot = u`, `θ = ∫θ_dot`

- or a **2nd-order joint model** *(if implemented)*:  
  `θ_dd = (u - damping*θ_dot)/Inertia`

Outputs:
- `θ1_meas`, `θ2_meas`
- `θ1_dot`, `θ2_dot` *(if included)*

---

### 4. **Forward Kinematics (MATLAB Function Block)**

A MATLAB Function block computes:

```matlab
x1 = L1*cos(theta1);
y1 = L1*sin(theta1);
x2 = x1 + L2*cos(theta1 + theta2);
y2 = y1 + L2*sin(theta1 + theta2);
```

Outputs:
- End-effector X, Y
- Intermediate link coordinates

Used by scopes and XY plots.

---

### 5. **XY Graph / Scopes**

Blocks included:

- **XY Graph**  
  Plots `x2` vs `y2` in real-time.

- **Scopes**  
  Show:
  - Joint angles vs references
  - Errors
  - Torques
  - End-effector tracking

---

### 6. **To Workspace Blocks**

For post-simulation analysis:

- `theta1_out`
- `theta2_out`
- `ee_out` (time,x,y)

These support plotting functions such as  
`control_analysis_exhibit(d)` in your GUI.

---

## 🛠 MATLAB Functions Used Inside the Model

Simulink may call or embed the following functions:

### ✔ `forward_kinematics_traj`
Also used in GUI — reused for Simulink FK.

### ✔ Basic math functions  
`cos`, `sin`, `atan2`, `sqrt`, etc.

### ✔ Optional interpolation  
If lookup tables are used:
`interp1`

All functions are supported by MATLAB Function blocks.

---

## ▶ How To Run the Simulink Model

1. Run GUI → Generate Trajectory → Compute IK → **Move on →**
2. This exports all required variables.
3. Open the model:
   ```matlab
   open('robot_tracker.slx');
   ```
4. Press **Run**.
5. Observe end‑effector motion, joint tracking, and scopes.

---

## 📌 Important Notes

- Solver recommended: **ode45** (variable-step) or **ode3** (fixed-step).
- Stop time should match:
  ```matlab
  T = theta1_ref(end,1)
  ```
- Model must reference the exact workspace variable names listed above.

---

## 🎯 Summary

`robot_tracker.slx` acts as the simulation bridge between:

- GUI trajectory generation  
- PD control  
- Robot kinematics  
- Visualization & plotting  

It ensures the software-in-the-loop (SIL) capability of the 2-link drawing robot.

---

