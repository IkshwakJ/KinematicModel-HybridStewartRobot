%{
    compute_transformation_matrix_serial: compute the homogeneous transformation
    matrix for a serial robot link using standard Denavit–Hartenberg parameters.
    
    Inputs:
        a     – scalar link length (distance along x_{i-1} from z_{i-1} to z_i).
        alpha – scalar link twist (angle between z_{i-1} and z_i about x_{i-1}).
        d     – scalar link offset (distance along z_i from x_{i-1} to x_i).
        theta – scalar joint angle (rotation about z_i from x_{i-1} to x_i).
    
    Outputs:
        T_mat – 4×4 homogeneous transformation matrix representing the pose
                of frame i relative to frame i-1 under the standard DH convention.
%}
function T_mat = compute_transformation_matrix_serial(a, alpha, d, theta)
    T_mat = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0,           sin(alpha),             cos(alpha),            d;
             0,           0,                      0,                     1];
end