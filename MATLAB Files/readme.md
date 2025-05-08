# PROJECT OVERVIEW
This MATLAB project models and visualizes a modular robot built from stacked Stewart–Gough platforms on a serial RRP backbone. It includes routines for:

- Defining robot parameters  
- Computing forward and inverse kinematics  
- Computing Jacobians  
- Executing resolved‐rate control to drive an end‐effector  
- Visualizing both the serial structure and the parallel legs with gripper and pen  

---

## DIRECTORY STRUCTURE
- `Robot_Desc.mat`  
  Stored robot parameters (DH values, radii, limits)  
- `define_robot.m`  
  Initialize and save robot description  
- `compute_transformation_matrix_serial.m`  
  Compute a single DH homogeneous transform  
- `dirkin_serial.m`  
  Forward kinematics for serial RRP backbone  
- `invkin_parallel.m`  
  Inverse kinematics for Stewart–Gough legs  
- `compute_jacobian_serial.m`  
  Build full 6×n Jacobian for the serial chain  
- `compute_partial_jacobian_parallel.m`  
  Build weighted pseudoinverse for parallel legs  
- `draw_robot_serial.m`  
  3D visualization of serial backbone and triangles  
- `draw_robot_parallel.m`  
  3D visualization of parallel legs, gripper, and pen  
- `res_rates.m`  
  Resolved‐rate control to move pen tip to a target  
- `simulate_robot.m`  
  Run a trajectory sweep and record performance  
- **Supporting helper files**  
  Triangle patches, coordinate axes, link primitives  

---

## REQUIREMENTS
- MATLAB (no additional toolboxes required)

---

## USAGE WORKFLOW
1. **Run** `define_robot` to create `Robot_Desc.mat`.  
2. **Use** `simulate_robot(stack_count, shape)` to execute a path and plot performance metrics (iteration count, run time, leg‐length changes).

## simulate_robot WORKFLOW
1. **Call** `res_rates(P_des, n, q_start)` to drive the pen tip to a desired 3D point.

## res_rates WORKFLOW
1. **Call** `dirkin_serial(q)` to compute serial link frames for joint vector `q`.  
2. **Call** `invkin_parallel(frames_parallel)` to compute leg vectors and lengths.  
3. **Use** `compute_jacobian_serial` or `compute_partial_jacobian_parallel` for velocity mapping.  
4. **Visualize** the structure with `draw_robot_serial` or `draw_robot_parallel`.
---

## FUNCTION SUMMARIES

### `define_robot`
- Clears workspace and saves initial DH parameters, platform radii, and joint limits  
- **Sub‐routines**:  
  - `Generate_DH_Parameters` — set link lengths (`a`), twist angles (`α`), offsets (`d`), and joint angles (`θ`)  
  - `Define_Stewart_Gough_Platform` — set base and moving platform radii  
  - `Define_Limits` — set roll, pitch, and prismatic joint limits  

### `compute_transformation_matrix_serial`
Return a 4×4 homogeneous transform from DH parameters.

### `dirkin_serial`
Compute forward kinematic frames for each RRP module and stack.

### `invkin_parallel`
Compute all leg vectors `Qp`, lengths `qp_val`, base points `qp_base`, and offset matrices `B` and `M`.

### `compute_jacobian_serial`
Assemble the 6×n Jacobian mapping joint velocities to end‐effector velocity.

### `compute_partial_jacobian_parallel`
Build the partial Jacobian relating serial chain motion to leg rate of change.

### `draw_robot_serial`
Render base and moving triangles, cylindrical links, and coordinate axes for the serial backbone.

### `draw_robot_parallel`
Render parallel legs, semi-transparent triangles, gripper body, and pen.

### `res_rates`
Implement a weighted resolved‐rate controller to move the pen tip to a target point; record position and leg‐length change histories.

### `simulate_robot`
Generate a waypoint path (circle or fixed), call `res_rates` for each waypoint, and plot solver performance metrics.
