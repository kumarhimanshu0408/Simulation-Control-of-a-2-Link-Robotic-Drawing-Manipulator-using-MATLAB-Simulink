Here is the complete, updated `README.md` file in Markdown format.

```markdown
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
├── main\_gui\_final.m          \# Main GUI application (top module)
├── inverse\_kinematics.m     \# IK solver for 2-link manipulator
├── forward\_kinematics.m     \# FK computation for verification
├── svg\_parser.m            \# SVG path parsing and curve sampling
├── trajectory\_generation.m  \# Trajectory creation and resampling
├── smoothing.m             \# Joint angle smoothing algorithms
├── plotting.m              \# Visualization and diagnostic plots
└── robot\_tracker.slx       \# Simulink model for control simulation

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

````

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

### 8. `robot_tracker.slx` - Simulink Control Simulation

**Purpose**: Provides a dynamic simulation of the robot arm tracking the generated trajectory using closed-loop PD control. It validates the kinematic data exported from the GUI against a continuous plant model.

**Components & Blocks**:

1.  [cite_start]**Input Data (`From Workspace`)**[cite: 20]:
    -   `theta1_ref` & `theta2_ref`: Imports the generated trajectory timeseries.
    -   `L1` & `L2`: Imports arm lengths (`L1_sim`, `L2_sim`) defined in the workspace.

2.  [cite_start]**Controller Subsystems (`PD Controller`)**[cite: 24]:
    -   Implements a Proportional-Derivative control law: $\tau = K_p(e) + K_d(\dot{e})$.
    -   [cite_start]**Gain Blocks**: Uses workspace variables `Kp` and `Kd` to tune performance[cite: 8].
    -   [cite_start]**Derivative Block**: Computes the rate of change of the error signal[cite: 7].
    -   [cite_start]**Sum Blocks**: Calculates error (`ref - actual`) and sums the P and D components[cite: 8, 9].

3.  [cite_start]**Plant Model (`Transfer Fcn`)**[cite: 20]:
    -   Simulates the motor/arm dynamics using a second-order transfer function ($ \frac{1}{s^2 + s} $).
    -   Takes torque ($\tau$) as input and outputs the actual joint angle.

4.  [cite_start]**MATLAB Function Block (`fcn`)**[cite: 161]:
    -   **Purpose**: Computes real-time Forward Kinematics for visualization.
    -   **Logic**:
        ```matlab
        function [x_out, y_out] = fcn(q1, q2, L1, L2)
            % Calculates coordinates for base, joint 1, and end-effector
            x0 = 0; y0 = 0;
            x1 = L1*cos(q1); y1 = L1*sin(q1);
            x2 = x1 + L2*cos(q1+q2); y2 = y1 + L2*sin(q1+q2);
            x_out = [x0, x1, x2];
            y_out = [y0, y1, y2];
        end
        ```
    -   [cite_start]**Outputs**: Vectors `x_out` and `y_out` containing the coordinates of the links for plotting[cite: 164].

5.  [cite_start]**Visualization Tools**[cite: 25, 27]:
    -   **XY Graph**: Plots the actual path of the robot in Cartesian space.
    -   **Scopes**: Displays the reference vs. actual joint angles for both Joint 1 and Joint 2 to monitor tracking performance.

---

## Code Logic Flow

### Complete Workflow

````

1.  USER INPUT
    ├── Set arm lengths (L1, L2)
    ├── Choose trajectory type
    ├── Configure sampling parameters
    └── Set initial position (initX, initY)

2.  TRAJECTORY GENERATION
    ├── Built-in: generate\_builtin() → parametric equations
    ├── SVG: parseSVGPath\_enhanced\_fixed() → path parsing
    └── translate\_and\_scale\_to\_init() → workspace fitting
    └── resample\_and\_smooth\_path\_param() → uniform sampling

3.  INVERSE KINEMATICS
    ├── For each trajectory point:
    │   └── inverse\_kinematics() → (θ₁, θ₂)
    ├── fill\_and\_interp\_nans() → handle unreachable points
    ├── unwrap() → remove angle discontinuities
    └── quintic\_bspline\_joint\_smooth() → smooth motion

4.  ANIMATION
    ├── Time-based frame selection
    ├── update\_links\_plot() → visual update
    └── forward\_kinematics\_traj() → verification

5.  EXPORT & SIMULATION
    ├── Create time vectors
    ├── Export joint angles (theta1\_ref, theta2\_ref)
    ├── Export end-effector (ee\_ref)
    ├── Create timeseries objects
    └── control\_analysis\_exhibit() → diagnostics

<!-- end list -->

````

### Key Algorithms

#### Inverse Kinematics Algorithm
```matlab
1. Compute distance: r = √(px² + py²)
2. Check reachability: |L1-L2| ≤ r ≤ L1+L2
3. Solve for θ₂ using cosine law
4. Solve for θ₁ using geometric relationships
5. Return angles with validity flag
````

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

-----

## Usage Instructions

### Getting Started

1.  **Navigate to Robotc folder**:

    ```matlab
    cd Robotc
    ```

2.  **Run the GUI**:

    ```matlab
    main_gui_final
    ```

3.  **Basic Workflow**:

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

1.  Click "Upload SVG Path"
2.  Select SVG file from dialog
3.  System automatically:
      - Parses path data
      - Normalizes coordinates
      - Scales to fit workspace
      - Resamples to N points
4.  Adjust Initial X, Y if trajectory is outside reach

### Custom Trajectory

1.  Select "Custom" from trajectory type
2.  Click "Generate Trajectory"
3.  Left-click on workspace to add points
4.  Press ENTER or right-click to finish
5.  System interpolates smooth path through points

-----

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

1.  **Normalization**: SVG paths are centered and scaled to unit size
2.  **Scaling**: Paths scaled to 80% of maximum reach
3.  **Translation**: Applied to initial position (initX, initY)
4.  **Resampling**: Uniform arc-length parameterization
5.  **Smoothing**: Moving filters reduce numerical noise

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

-----

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

-----

## Error Handling

The system includes robust error handling:

1.  **Reachability Checks**: Validates workspace limits before IK computation
2.  **NaN Handling**: Interpolates unreachable points
3.  **Empty Trajectory**: Prevents computation on invalid data
4.  **SVG Parsing Errors**: Catches XML and path parsing exceptions
5.  **Toolbox Availability**: Falls back to standard functions if toolboxes unavailable

-----

## Dependencies

### Required MATLAB Toolboxes

  - **Base MATLAB**: Core functionality
  - **Signal Processing Toolbox**: For `movmedian()` and `movmean()` (optional, has fallbacks)

### Optional Toolboxes

  - **Spline Toolbox**: For B-spline smoothing (falls back to cubic spline if unavailable)
  - **Simulink**: For importing exported data (not required for GUI operation)

-----

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
| `robot_tracker.slx` | Control Simulation | `PD Controller`, `MATLAB Function`, `XY Graph` |

-----

## Future Enhancements

Potential improvements:

  - 3D manipulator support
  - Dynamic trajectory optimization
  - Collision avoidance
  - Real-time control interface
  - Additional trajectory types
  - Export to other formats (ROS, URDF)

-----

## Author Notes

This project demonstrates:

  - **Robotics fundamentals**: Kinematics, trajectory planning
  - **GUI development**: MATLAB App Designer concepts
  - **File processing**: SVG parsing and data extraction
  - **Numerical methods**: Interpolation, smoothing, optimization
  - **System integration**: Simulink data export

The modular architecture allows easy extension and maintenance while keeping the codebase organized and understandable.

```
```
