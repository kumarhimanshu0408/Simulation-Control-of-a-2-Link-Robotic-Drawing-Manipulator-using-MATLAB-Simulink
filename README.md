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

This repository contains only the GUI script and the Simulink model:

```
.
├── main_gui_final.m          # Main GUI application (single-file implementation)
└── robot_tracker.slx         # Simulink model for trajectory tracking and visualization
```

**Simulink model file path (local):** `sandbox:/mnt/data/robot_tracker.slx`

> Note: The entire application logic is implemented inside `main_gui_final.m` as one self-contained script containing nested callbacks and a set of subfunctions (no separate `.m` helper files are required).

---

## Function-level Documentation (all functions inside `main_gui_final.m`)

Below is a concise but thorough description of every function present in `main_gui_final.m`. Each entry lists purpose, inputs, outputs, and important implementation details or notes.

### `main_gui_final` (main function)
**Purpose:** Creates the GUI window, initializes parameters, creates controls and axes, registers callbacks, and stores persistent data via `guidata`. This is the entry point; run `main_gui_final` to launch the application.

**Key actions:**
- Initializes `params` (L1, L2, N, curveSamples, totalTime, elbow, initX, initY).
- Builds figure, axes, control `uipanel`, and all UI controls (edits, popup menus, buttons).
- Stores handles and parameters in `guidata`.
- Declares nested callback functions: `onGenerate`, `onComputeIK`, `onAnimate`, `onMoveOn`, `onReset`, `onUploadSVG` (these access and update `guidata`).
- The rest of the file contains helper subfunctions (non-nested) used by callbacks.

**How to run:** `main_gui_final` in MATLAB command window.

---

### Nested Callbacks (defined within `main_gui_final`)

> These are closures that access GUI handles and `guidata`. They orchestrate the GUI flow.

#### `onGenerate(~,~)`
**Purpose:** Generate a trajectory according to GUI selection — built-in shapes or an already-loaded SVG. Resamples and displays the trajectory on the workspace axes.

**Inputs:** none (uses GUI state via `guidata`)

**Behavior & steps:**
1. Reads current GUI parameters: L1, L2, curveSamples, N, initX, initY, and trajectory type.
2. If Trajectory Type is `SVG`, ensures `d.traj_raw` exists (from `onUploadSVG`), translates/scales it to initial position, and calls `resample_and_smooth_path_param`.
3. Otherwise calls `generate_builtin(trajType, N)` to produce a shape, then translates by initX/initY.
4. Stores result in `d.traj` and plots it on the axes.
5. Updates axis limits and title.

**Notes:** Performs safety clamping for sample counts.

#### `onComputeIK(~,~)`
**Purpose:** Compute inverse kinematics for the current trajectory and smooth joint angle sequences for animation and export.

**Inputs:** none (uses `d.traj` and GUI parameters)

**Outputs:** Populates `d.joint.theta1`, `d.joint.theta2` after smoothing.

**Behavior & steps:**
1. Reads elbow selection (up/down), L1/L2.
2. Iterates over all trajectory points and calls `inverse_kinematics(px,py,L1,L2,elbow)` for each to compute `th1`, `th2`.
3. Builds a validity mask `okmask`. If none reachable → error dialog.
4. Fills NaNs with `fill_and_interp_nans`, unwraps angles, and calls `quintic_bspline_joint_smooth` to get smooth joint trajectories of length `params.N`.
5. If smoothing fails or NaNs remain, uses `interp1` fallback.
6. Stores smoothed joint arrays in `d.joint`.
7. Attempts to compute and overlay a forward-kinematics verification track (calls `forward_kinematics_traj`).
8. Calls `update_links_plot(d,1)` to show first pose and notifies the user.

**Notes:** Robust to sparse unreachable points; produces smoothed outputs suitable for animation and Simulink export.

#### `onAnimate(~,~)`
**Purpose:** Time-based animation loop that uses the smoothed joint trajectories to update the workspace plot in real-time.

**Inputs:** none (reads `d.joint` and `d.params`)

**Behavior & steps:**
1. Validates that IK has been computed.
2. Reads N (samples) and target duration T from `d.params.totalTime`.
3. Creates or finds a status text object in axes to display joint angles.
4. Uses `tic`/`toc` loop to track elapsed time and compute `ratio = elapsed / T`.
5. Computes index = round(1 + (N-1)*ratio), clamps it, and calls `update_links_plot(d, idx)` to render the pose for that index.
6. Uses `drawnow limitrate` to throttle updates and ends when elapsed >= T.
7. Ensures last frame is drawn.

**Notes:** Frame skipping ensures the animation completes in the requested time independent of rendering speed.

#### `onMoveOn(~,~)`
**Purpose:** Export joint reference trajectories and end-effector references to MATLAB base workspace (for Simulink) and launch diagnostic plotting.

**Inputs:** none (reads `d.joint`, `d.params`)

**Outputs:** assigns variables in base workspace:
- `theta1_ref` (Nx2: `[t, θ1]`), `theta2_ref`, `ee_ref` (Nx3: `[t, x, y]`)
- `L1_sim`, `L2_sim`, `Kp`, `Kd`
- `ts_theta1`, `ts_theta2`, `ts_ee` (timeseries objects, if timeseries toolbox available)

**Behavior & steps:**
1. Compute `tvec = linspace(0,T,N)'` (T from GUI total time).
2. Build arrays and assign to base workspace using `assignin('base', ...)`.
3. Create `timeseries` objects where possible and assign them.
4. Calls `control_analysis_exhibit(d)` to plot diagnostics.

**Notes:** This function creates the exact workspace variables expected by the Simulink model `robot_tracker.slx`.

#### `onReset(~,~)`
**Purpose:** Reset GUI state and plots to defaults.

**Behavior & steps:**
- Clears axes, resets UI controls to default `params`, and re-initializes `guidata`.

#### `onUploadSVG(~,~)`
**Purpose:** Load an SVG file, parse its path data, normalize/center/scale it, resample using current GUI settings, and display it.

**Behavior & steps:**
1. Prompts user with `uigetfile` to pick an `.svg`.
2. Reads `curveSamples` and `N` from GUI.
3. Calls `parseSVGPath_enhanced_fixed(fullPath, cs)` to parse raw coordinate arrays.
4. Stores raw normalized arrays in `d.traj_raw`.
5. Calls `translate_and_scale_to_init` and `resample_and_smooth_path_param` to produce `d.traj`.
6. Displays resampled trajectory in axes and notifies user.

**Error handling:** Catches XML or parsing errors and shows helpful error dialogs; prints stack trace details to console.

---

## Subfunctions (top-level, non-nested helpers)

These functions are declared after the `main_gui_final` end and are available for reuse or direct calling.

### `control_analysis_exhibit(d)`
**Purpose:** Produce a 6-plot diagnostic figure summarizing initial pose, desired vs actual EE path, joint angles, joint velocities, tracking error, and PD-like torques.

**Inputs:** `d` (GUI data containing `d.joint`, `d.traj`, `d.params`)

**Outputs:** A figure with tiled plots. No workspace assignment.

**Key steps:**
- Extract joint arrays and compute time vector `t`.
- Compute forward kinematics on smooth joint references (`forward_kinematics_traj`).
- Interpolate desired trajectory to match `N` if needed.
- Compute finite-difference velocities and PD-like torque estimates using `Kp`, `Kd`.
- Plot the six subplots with titles, legends, and annotations (max/rms error).

**Notes:** Useful for post-export diagnostics and validation.

---

### `parseSVGPath_enhanced_fixed(svg_file, curve_resolution)`
**Purpose:** Robustly parse common SVG elements (`<path>`, `<polyline>`, `<polygon>`) and sample them into normalized X/Y point arrays.

**Inputs:**
- `svg_file` — path to the svg file
- `curve_resolution` — samples per curve segment (default 40)

**Outputs:** `(xcol, ycol)` column vectors of normalized coordinates (centered and scaled to unit extent).

**Supported commands:** `M m, L l, H h, V v, C c, S s, Q q, T t, A a, Z z`.

**Implementation highlights:**
- Uses `xmlread` to load the SVG.
- Concatenates path and polyline point data into a single path string.
- Tokenizes with regex, iterates commands and numeric values.
- Handles relative and absolute coordinates.
- Calls helper samplers for cubic/quadratic Béziers and arcs: `sampleCubic`, `sampleQuadratic`, `svg_arc_to_poly_fast`.
- Appends all sampled points safely via `safe_append`.
- Removes NaNs and duplicate points, centers coordinates and scales them by their maximum span.

**Failure modes:** Returns empty arrays on parse failure or no path data; caller handles error dialogs.

---

### `resample_and_smooth_path_param(x, y, N, curveSamples)`
**Purpose:** Convert an arbitrary polyline/path into an evenly-sampled, smoothed parametric trajectory with `N` points.

**Inputs:** `x, y` (vectors), `N` (desired point count), `curveSamples` (used earlier in parsing)

**Outputs:** `xr, yr` (row vectors of length `N`)

**Key steps:**
- Removes near-duplicate consecutive points.
- Computes cumulative arc-length (`cumd`) and normalizes to [0,1].
- Ensures unique parameterization (`unique(cumd,'stable')`) to avoid duplicate params.
- Uses `interp1(...,'pchip')` to produce `N` samples preserving shape.
- Applies a small moving median then moving mean smoothing window (window approx 0.3% of N).
- Fills NaNs by local interpolation/extrapolation if needed.

**Notes:** Robust to degenerate inputs (all points identical or too few points).

---

### `translate_and_scale_to_init(x_norm, y_norm, initX, initY, L1, L2)`
**Purpose:** Scale a normalized (unit-extent) path and translate it so it fits inside the robot reach and is positioned at `(initX, initY)`.

**Inputs:** normalized `x_norm, y_norm`, translation `initX, initY`, link lengths `L1, L2`.

**Outputs:** `x_out, y_out` scaled & translated coordinates.

**Scaling rule:** If extent > 0, scale = `0.8 * min(L1+L2,1) / extent` (80% of reach or 1 m reference), otherwise pick safe default scaling.

---

### `generate_builtin(type, N)`
**Purpose:** Create built-in parametric trajectories of `N` points.

**Supported types:**
- `'Circle'` — radius 0.5
- `'Line'` — horizontal line from -0.5 to 0.5
- `'Square'` — closed square path
- `'Custom'` — interactive point collection using `ginput()` then spline interpolation

**Outputs:** `x_out, y_out` row vectors length `N`.

**Notes:** Interactive custom mode provides on-axes feedback and finishes on ENTER or right click.

---

### `read_numbers(tokens, i_start)`
**Purpose:** Lexer helper used by `parseSVGPath_enhanced_fixed` to read numeric tokens until next command token.

**Inputs:** `tokens` (cell array), `i_start` (starting index)

**Outputs:** `vals` numeric vector, `next_i` index of next unconsumed token.

---

### `safe_append(xin,yin,seg)`
**Purpose:** Append a sampled segment `seg` (various shapes) to existing arrays robustly.

**Inputs:** existing `xin,yin` and `seg` (points in Nx2 or 1x2)

**Outputs:** extended `xout,yout`

**Notes:** Accepts `seg` as numeric matrix or cell and reshapes appropriately.

---

### `sampleCubic(p0,c1,c2,p3,res)` and `sampleQuadratic(p0,c,p2,res)`
**Purpose:** Numerically sample cubic and quadratic Bézier curves into `res+1` points.

**Inputs:** control points, resolution `res` (default 40)

**Outputs:** points array `pts` of size `(res+1)x2`.

**Formulas:** Standard Bézier polynomial evaluation.

---

### `svg_arc_to_poly_fast(p1,p2,rx,ry,phi_deg,laf,sf,res)`
**Purpose:** Approximate an SVG elliptical arc command by sampling it into a polyline.

**Inputs:** arc parameters from SVG spec plus resolution

**Outputs:** sampled points array forming the arc

**Implementation details:**
- Implements arc center parameter conversion and angle sweep computation.
- Applies rotation `phi` then parameterizes ellipse points and rotates back.

**Fallback:** If radii zero or degenerate, returns straight line between endpoints.

---

### `fill_and_interp_nans(a)`
**Purpose:** Fill NaN entries in a vector `a` by PCHIP interpolation/extrapolation.

**Inputs:** vector `a` (row/col)

**Outputs:** vector `a` with NaNs replaced by interpolated values or zeros when insufficient valid data.

---

### `quintic_bspline_joint_smooth(th1,th2,Nout)`
**Purpose:** Smooth joint angle sequences (th1, th2) into `Nout` samples using a high-order B-spline or fall back to cubic spline.

**Inputs:** `th1`, `th2` (vectors), desired output length `Nout`

**Outputs:** `t1_s`, `t2_s` (smoothed vectors), `t` (parameter vector)

**Algorithm:**
- If spline toolbox available, constructs knot vector and uses `spap2` to fit 6th-order spline then evaluates at `t`.
- If unavailable, uses `spline` interpolation.

**Notes:** Ensures C⁵-like smoothness where possible, reduces jerk.

---

### `update_links_plot(d, idx)`
**Purpose:** Update the workspace axes to display the manipulator pose corresponding to sample index `idx`.

**Inputs:** `d` (GUI data containing `d.joint`, `d.params`, `d.traj`) and index `idx`.

**Behavior:**
- Clears the axes and draws reachability circles and desired trajectory (if available).
- Plots FK track up to current index (if joint history available).
- Draws links and joints (`p0`, `p1`, `p2`) and marks end-effector.
- Updates axis limits and grid.

**Notes:** Defensive checks ensure function exits silently if axes or joint data missing.

---

### `forward_kinematics_traj(th1,th2,L1,L2)`
**Purpose:** Vectorized forward kinematics for sequences of joint angles.

**Inputs:** `th1, th2` (vectors), `L1, L2` scalars.

**Outputs:** `x1,y1,x2,y2` vectors giving link endpoints and end-effector positions.

**Formula:** Uses cos/sin for each element; returns arrays suitable for plotting.

---

### `inverse_kinematics(px,py,L1,L2,elbow)`
**Purpose:** Compute (θ1, θ2) for a single end-effector point `(px,py)`.

**Inputs:** `px, py, L1, L2, elbow` (`'up'|'down'`)

**Outputs:** `theta1, theta2, ok` (ok boolean for reachability)

**Algorithm:** Cosine rule for θ2 and geometric decomposition for θ1. Clamps `cos_th2` to [-1,1] to avoid numerical errors. Uses `atan2` with appropriate sign for elbow choice.

---

## Simulink Model: `robot_tracker.slx` — Components, Functionality, and Integration

**Model file location (local):**  
`sandbox:/mnt/data/robot_tracker.slx`

This Simulink model consumes variables exported by `onMoveOn` and demonstrates PD tracking, forward kinematics, visualization, and data logging. See the dedicated Simulink section in the README (unchanged) for block-level description, workspace variables expected, and run instructions.

---

## Usage Instructions (summary)

1. Start MATLAB and ensure working directory contains `main_gui_final.m`.
2. Run:  
   ```matlab
   main_gui_final
   ```
3. Generate trajectory → Compute IK → Animate → Move on → Open `robot_tracker.slx` to simulate.

---

## Repository Notes

- The project intentionally keeps everything inside a single top-level GUI script (`main_gui_final.m`) and the Simulink model (`robot_tracker.slx`) for easier grading and distribution.
- If you prefer separate function files for reusability, the helper functions documented above can be split into individual `.m` files following the same function signatures.

---

If you'd like, I will:
- (A) write this updated README back to `/mnt/data/README.md` (overwrite), or
- (B) save as a new file `/mnt/data/README_functions.md` and provide a direct link.

Which option do you want?
