%{
    define_robot: orchestrate creation of the robot description file.
    
    Inputs:
        none
    
    Outputs:
        Robot_Desc.mat – MAT-file saved to disk containing all robot parameters.
    
    Actions:
        - Clears workspace.
        - Initializes and saves an empty MAT-file.
        - Calls Generate_DH_Parameters to define serial backbone DH parameters.
        - Calls Define_Stewart_Gough_Platform to define Stewart-Gough platform geometry.
        - Calls Define_Limits to define joint and platform motion limits.
%}
function define_robot()    
    % The robot is built up of multiple Stewart/Gough platforms that are
    % identical and stacked upon one other.
    clear;
    save('Robot_Desc.mat');
    Generate_DH_Parameters();
    Define_Stewart_Gough_Platform();
    Define_Limits();
end

%{
    Generate_DH_Parameters: define Denavit–Hartenberg parameters for the serial backbone.
    
    Inputs:
        none
    
    Outputs:
        DH_a      – 3×1 vector of link lengths.
        DH_alpha  – 3×1 vector of link twists.
        DH_d      – 3×1 vector of link offsets (last entry variable).
        DH_theta  – 3×1 vector of joint angles (first two entries variable).
        Saved to Robot_Desc.mat (appended).
    
    Notes:
        - Backbone is modeled as an RRP manipulator.
        - Initial fixed transforms T_0_0, T_1_1, T_2_2 rotate axes so the prismatic joint translates along +z.
        - Users can modify DH vectors within this function to tune the robot geometry.
%}
function Generate_DH_Parameters()
    % Change values within this function to edit DH parameters for the
    % Serial backbone for the Stewart Gough platform.

    % Each link in the backbone is a RRP manipulator with limits.
    
    % The first joint is a revolute joint placed at the origin, rotating
    % about x axis of the origin (z axis of this joint is alligned along 
    % x axis).
    % T_0_0 % This transformation matrix is independent of the joints, and
    % is present to rotate the robot such that the final prismatic joint is
    % able to translate along its +z axis.
    % theta = -pi/2
    % alpha = -pi/2
    % d = 0
    % a = 0
    % The second joint is a revolute joint placed at the origin, rotating
    % about its z axis aligned with the x axis of the previous joint.
    % T_1_1 % This transformation matrix is independent of the joints, and
    % is present to rotate the robot such that the final prismatic joint is
    % able to translate along its +z axis.
    % theta = -pi/2
    % alpha = -pi/2
    % d = 0
    % a = 0
    % The third joint is a prismatic joint placed at the origin,
    % translating about  its z axis aligned with x axis of the previous
    % joint.
    % T_2_2 % This transformation matrix is independent of the joints, and
    % is present to rotate the robot such that the final prismatic joint is
    % able to translate along its +z axis.
    % theta = -pi/2
    % alpha = -pi/2
    % d = 0
    % a = 0
    % After the T_2_2 transformation the start of the 3rd link will be
    % along the same axes as the origin if there is no tilt. 

    % With the T_0_0 matrix defined, the rest of the DH parameters are as
    % follows.

    DH_a = [0;0;0];
    DH_alpha = [0;0;0];
    DH_d = [0;0;0]; % The last entry is a variable.
    DH_theta = [0;0;0]; % The first and second entries are variables. 
    save('Robot_Desc.mat', 'DH_a', 'DH_alpha', 'DH_d', 'DH_theta', '-append');

end

%{
    Define_Stewart_Gough_Platform: specify geometry of the Stewart-Gough moving platform.
    
    Inputs:
        none
    
    Outputs:
        Radius_Base   – scalar radius of the fixed base platform.
        Radius_Moving – scalar radius of the moving platform.
        Saved to Robot_Desc.mat (appended).
    
    Notes:
        - Actuator lengths are not defined here; they are controlled via motion limits.
        - Base and moving platform radii default to 0.3 m.
%}
function Define_Stewart_Gough_Platform()
    % The platform can be defined by the radius of the base, moving
    % platform and the length of the actuators. 
    % The length of the actuators is being skipped in this project to
    % simplify it, and is instead controlled by the limits specified in the 
    % Define_Limits function. 
    
    Radius_Base = 0.3;
    Radius_Moving = 0.3;
    save('Robot_Desc.mat', 'Radius_Moving', 'Radius_Base', '-append');
end

%{
    Define_Limits: set motion limits for backbone prismatic joint and platform attitude.
    
    Inputs:
        none
    
    Outputs:
        Lim_roll      – scalar maximum roll angle (rad).
        Lim_pitch     – scalar maximum pitch angle (rad).
        Lim_len_min   – scalar minimum prismatic joint extension (m).
        Lim_len_max   – scalar maximum prismatic joint extension (m).
        Saved to Robot_Desc.mat (appended).
    
    Notes:
        - Roll and pitch limited to ±15°.
        - Yaw is not implemented in this model.
        - Prismatic joint stroke limited between 0.3 m and 0.5 m to prevent clipping.
%}
function Define_Limits()
    % The limits of the platform can be changed if needed; however, please
    % not that there might be clipping when the limits are stretched too
    % far.

    % Roll (y axis)
    Lim_roll = deg2rad(15);

    % Pitch (x axis)
    Lim_pitch = deg2rad(15);

    % There is no yaw implemented for this model; however, is available on
    % actual models. 

    % Minimum prismatic joint length for the Serial Backbone
    Lim_len_min = 0.3;

    % Maximum prismatic joint length for the Serial Backbone
    Lim_len_max = 0.5;

    save('Robot_Desc.mat', 'Lim_roll', 'Lim_pitch', 'Lim_len_min', 'Lim_len_max', '-append');
end