%{
    compute_jacobian_serial: compute the spatial Jacobian of the serial backbone.
    
    Inputs:
        frames_serial – 4×4×n array of homogeneous transforms for each backbone joint frame.
        T_tool        – (optional) 4×4 homogeneous transform of the tool or end-effector; 
                        defaults to the last frame in frames_serial.
    
    Outputs:
        jacobian_serial – 6×n Jacobian matrix mapping joint rates to end-effector spatial velocity.
                          Rows 1–3 correspond to linear velocity, rows 4–6 to angular velocity.
%}
function jacobian_serial = compute_jacobian_serial(frames_serial, T_tool)
    n = size(frames_serial, 3);
    if nargin < 2
        T_tool = frames_serial(:,:,n);
    end
    p_e = T_tool(1:3, 4);  % Gripper or end effector position in base frame
    
    jacobian_serial = zeros(6,n);
    for i = 1:n
        z_i = frames_serial(1:3, 3, i);
        p_i = frames_serial(1:3, 4, i);
        if (mod(i,3) == 0)
            jacobian_serial(1:3,i) = z_i; % prismatic joint along the z axis of joint i.
            jacobian_serial(4:6,i) = [0;0;0];
        else
            jacobian_serial(1:3, i) = cross(z_i, (p_e - p_i));
            jacobian_serial(4:6, i) = z_i;
        end
    end

end