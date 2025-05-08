%{
    dirkin_serial: compute forward kinematics for the serial backbone and extract
                   moving‐platform frames for each Stewart–Gough module.
    
    Inputs:
        q               – (3n)×1 vector of joint values for the serial RRP chains,
                          where each consecutive triplet [θᵢ, θᵢ₊₁, dᵢ₊₂] defines one module.
    
    Outputs:
        frames_parallel – 4×4×n array of homogeneous transforms at the top of each
                          Stewart–Gough platform (after the prismatic joint of each module).
        frames_serial   – 4×4×(3n) array of homogeneous transforms for every
                          intermediate serial backbone frame.
    
    Details:
        - Loads DH parameters and platform radii/limits from Robot_Desc.mat.
        - Applies fixed base rotations T_0_0, T_1_1, T_2_2 to align joint axes.
        - Iterates through q in steps of 3, computing:
            • T_0_1: transform after first revolute joint;
            • T_0_2: after second revolute joint;
            • T_0_3: after prismatic joint.
        - Updates T_base for chaining modules and populates both output arrays.
%}
function [frames_parallel, frames_serial] = dirkin_serial(q)
    
    % q are the joint vector for the backbone or the serial manipulator
    % that represent the stewart/gough platform.

    % Length of chain n(Per chain RRP)
    len = length(q);
    n = len / 3;
    
    frames_parallel = zeros(4,4,n);
    frames_serial = zeros(4,4,len);
    
    load('Robot_Desc.mat');
    % Generating transformation matrices for each Stewart/Gough platform
    % As the links for the parallel robot is considered as a prismatic
    % joint that can tilt in 2 DoF wrt the previous joint, the 
    % intermediate frames will be skipped on the transformation matrices 
    % stored for the purpose of calculating the joint values for the 
    % Stewart platform.
    % The position and orientation computed with joints oriented along the
    % x axis(pitch), y axis(roll) and the prismatic(moves along its z axis)
    
    T_base = eye(4);
    
    for i = 1:3:len

        T_0_0 = T_base * compute_transformation_matrix_serial(0, -pi/2, 0, -pi/2);
        T_0_1 = T_0_0 * compute_transformation_matrix_serial(DH_a(1), DH_alpha(1), DH_d(1), q(i));
        frames_serial(:,:,i) = T_0_1;

        T_1_1 = compute_transformation_matrix_serial(0, -pi/2, 0, -pi/2);
        T_0_2 = T_0_1 * T_1_1 * compute_transformation_matrix_serial(DH_a(2), DH_alpha(2), DH_d(2), q(i+1));
        frames_serial(:,:,i+1) = T_0_2;
        
        T_2_2 = compute_transformation_matrix_serial(0, -pi/2, 0, -pi/2);
        T_0_3 = T_0_2 * T_2_2 * compute_transformation_matrix_serial(DH_a(3), DH_alpha(3), q(i+2), DH_d(3));
        frames_serial(:,:,i+2) = T_0_3;
        
        T_base = T_0_3;
        frames_parallel(:,:,((i+2)/3)) = T_base;
        
    end
end
