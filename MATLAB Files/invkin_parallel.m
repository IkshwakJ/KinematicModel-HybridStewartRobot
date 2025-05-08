%{
    invkin_parallel: compute inverse kinematics for all legs of each parallel module.
    
    Inputs:
        frames_parallel – 4×4×n array of homogeneous transforms at the top of each module.
    
    Outputs:
        Qp       – (6n)×3 matrix of leg direction vectors.
        qp_val   – (6n)×1 vector of leg lengths (norms of Qp rows).
        qp_base  – (6n)×3 matrix of leg base attachment points.
        B        – (3n)×3 matrix of base triangle point offsets from module base origin.
        M        – (3n)×3 matrix of moving platform point offsets from module top origin.
    
    Details:
        - Loads platform radii from Robot_Desc.mat.
        - For each module i:
            • Compute base frame T_base (previous top or world origin).
            • Compute end frame T_end (current top).
            • Retrieve base triangle points via base_triangle_points.
            • Retrieve top triangle points via top_triangle_points.
            • Fill B and M with offsets from respective origins.
            • For each of the six legs (connectivity pattern):
                – Compute Qp as vector from base point to corresponding end point.
                – qp_val as length of that vector.
                – qp_base repeats the base point position.
%}
function [Qp, qp_val, qp_base, B, M] = invkin_parallel(frames_parallel)
    load('Robot_Desc.mat','Radius_Base', 'Radius_Moving');

    n = size(frames_parallel,3);
    qp_val = zeros(n*6,1);
    Qp = zeros(n*6,3);
    qp_base = zeros(n*6,3);
    M = zeros(n*3,3);
    B = zeros(n*3,3);
    for i = 1:n
        if i == 1
            T_base = eye(4);
        else
            T_base = frames_parallel(:,:,i-1);
        end
        T_end = frames_parallel(:,:,i);
        % Leg connectivity:
        %   Leg 1 connects base pt 1 → end pt 2
        %   Leg 2 connects base pt 1 → end pt 3
        %   Leg 3 connects base pt 2 → end pt 3
        %   Leg 4 connects base pt 2 → end pt 1
        %   Leg 5 connects base pt 3 → end pt 1
        %   Leg 6 connects base pt 3 → end pt 2
        tri_base = base_triangle_points(T_base, Radius_Base);
        tri_end = top_triangle_points(T_end, Radius_Moving);
        V_B_1 = (tri_base(1,:));
        V_B_2 = (tri_base(2,:));
        V_B_3 = (tri_base(3,:));
        V_E_1 = (tri_end(1,:));
        V_E_2 = (tri_end(2,:));
        V_E_3 = (tri_end(3,:));
        
        M((i-1)*3 + 1, :) = V_E_1 - transpose(T_end(1:3,4));
        M((i-1)*3 + 2, :) = V_E_2 - transpose(T_end(1:3,4));
        M((i-1)*3 + 3, :) = V_E_3 - transpose(T_end(1:3,4));

        B((i-1)*3 + 1, :) = V_B_1 - transpose(T_base(1:3,4));
        B((i-1)*3 + 2, :) = V_B_2 - transpose(T_base(1:3,4));
        B((i-1)*3 + 3, :) = V_B_3 - transpose(T_base(1:3,4));

        Qp((i-1)*6 + 1, :) = (V_E_2 - V_B_1);
        qp_val((i-1)*6 + 1) = norm(Qp((i-1)*6 + 1, :));
        Qp((i-1)*6 + 2, :) = (V_E_3 - V_B_1);
        qp_val((i-1)*6 + 2) = norm(Qp((i-1)*6 + 2, :));
        Qp((i-1)*6 + 3, :) = (V_E_3 - V_B_2);
        qp_val((i-1)*6 + 3) = norm(Qp((i-1)*6 + 3, :));
        Qp((i-1)*6 + 4, :) = (V_E_1 - V_B_2);
        qp_val((i-1)*6 + 4) = norm(Qp((i-1)*6 + 4, :));
        Qp((i-1)*6 + 5, :) = (V_E_1 - V_B_3);
        qp_val((i-1)*6 + 5) = norm(Qp((i-1)*6 + 5, :));
        Qp((i-1)*6 + 6, :) = (V_E_2 - V_B_3);
        qp_val((i-1)*6 + 6) = norm(Qp((i-1)*6 + 6, :));

        qp_base((i-1)*6 + 1, :) = V_B_1;
        qp_base((i-1)*6 + 2, :) = V_B_1;
        qp_base((i-1)*6 + 3, :) = V_B_2;
        qp_base((i-1)*6 + 4, :) = V_B_2;
        qp_base((i-1)*6 + 5, :) = V_B_3;
        qp_base((i-1)*6 + 6, :) = V_B_3;
    end

end

%{
    base_triangle_points: compute global coordinates of base triangle vertices.
    
    Inputs:
        T – 4×4 homogeneous transform of the module base.
        a – scalar radius controlling triangle side length (side = a√3).
    
    Outputs:
        vertices_global – 3×3 matrix of vertex coordinates in world frame.
    
    Details:
        - Defines an equilateral triangle in local XY plane, centered at origin.
        - Applies T to each vertex to obtain global positions.
%}
function vertices_global = base_triangle_points(T,a)
    side_length = a * (3^0.5);  
    height = (sqrt(3)/2) * side_length;
    vertices = [0,         0,      0;              
                side_length, 0,      0;              
                side_length/2, height, 0];          

    centroid = mean(vertices, 1);
    vertices_centered = vertices - centroid;
    vertices_local_hom = [vertices_centered, ones(3, 1)]';
    vertices_global_hom = T * vertices_local_hom;
    
    vertices_global = vertices_global_hom(1:3, :)';
end

%{
    top_triangle_points: compute global coordinates of moving platform triangle vertices.
    
    Inputs:
        T – 4×4 homogeneous transform of the module top.
        a – scalar radius controlling triangle side length (side = a√3).
    
    Outputs:
        vertices_global – 3×3 matrix of vertex coordinates in world frame.
    
    Details:
        - Defines an equilateral triangle in local XY plane, centered at origin.
        - Flips and inverts local vertices to orient moving platform correctly.
        - Applies T to each vertex to obtain global positions.
%}
function vertices_global = top_triangle_points(T,a)
    side_length = a * (3^0.5);  
    height = (sqrt(3)/2) * side_length;
    vertices = [0,         0,      0;              
                side_length, 0,      0;              
                side_length/2, height, 0];          

    centroid = mean(vertices, 1);
    vertices_centered = vertices - centroid;
    vertices_centered = -vertices_centered;
    vertices_local_hom = [vertices_centered, ones(3, 1)]';
    vertices_global_hom = T * vertices_local_hom;
    
    vertices_global = vertices_global_hom(1:3, :)';
end