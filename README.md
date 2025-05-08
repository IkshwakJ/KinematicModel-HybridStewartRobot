# KinematicModel-HybridStewartRobot
MATLAB simulation and kinematic modeling of a hybrid serial-parallel robot composed of stacked Stewart platforms connected by an RRP backbone with redundancy resolution.

The simulation is entirely written in MATLAB and supports:
- Forward kinematics for the serial backbone
- Inverse kinematics of each Stewart platform
- Analytical Jacobian computations for both serial and parallel chains
- Redundancy resolution using a weighted pseudoinverse method
- Simulation of end-effector motion and joint trajectories
- Visualization of the hybrid robot in 3D

This codebase was developed as part of a graduate robotics course project.