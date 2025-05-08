%{
    draw_robot_parallel: visualize the full parallel‐legged robot including
                         base & moving platforms, legs, gripper, and pen.
    
    Inputs:
        Qp               – n×3 matrix of leg direction vectors (rows for legs 1..n).
        qp_base          – n×3 matrix of base attachment point coordinates.
        frames_serial    – 4×4×m array of serial backbone frames.
        T_gripper_end    – 4×4 homogeneous transform of the gripper end.
        T_Pen            – 4×4 homogeneous transform of the pen tip.
    
    Outputs:
        (none) – opens 3D figures showing:
                 • Transparent base and moving platform triangles.
                 • Cylindrical leg links.
                 • Gripper assembly.
                 • Pen drawing link.
    
    Behavior:
        - Draws base triangle at origin and at each module’s base.
        - Draws moving platform at each module’s top.
        - Iterates through Qp to render each leg via DrawLink.
        - Renders gripper and pen via DrawGripper and DrawPen.
%}
function draw_robot_parallel(Qp, qp_base, frames_serial, T_gripper_end, T_Pen)
    len = size(frames_serial,3);
    load('Robot_Desc.mat','Radius_Base', 'Radius_Moving');
    draw_transparent_equilateral_triangle_centered_base_platform(eye(4),Radius_Base);
    for i = 1:3:len
        if( i ~= 1)
            draw_transparent_equilateral_triangle_centered_base_platform(frames_serial(:,:,i-1),Radius_Base);
        end
        draw_transparent_equilateral_triangle_centered_moving_platform(frames_serial(:,:,i+2),Radius_Moving);
    end

    n = size(Qp,1);
    for i = 1:n
        DrawLink(transpose(Qp(i, :)), transpose(qp_base(i,:)), i)
    end
    DrawGripper(frames_serial(:,:,len), T_gripper_end);
    DrawPen(T_gripper_end, T_Pen);
end

%{
    DrawLink: render a cylindrical leg link with hemispherical endcaps.
    
    Inputs:
        Qp      – 3×1 vector from base to moving platform attachment (leg).
        qp_base – 3×1 base attachment point in world coordinates.
        i       – integer index used to select leg color.
        scale   – (optional) scalar scale factor for cylinder radius (default = 1).
    
    Outputs:
        (none) – adds a 3D cylinder and two semispheres to current figure.
    
    Details:
        - Computes cylinder along Qp direction of length ‖Qp‖ minus endcap radii.
        - Uses Rodrigues’ formula to align default z‐axis cylinder to Qp.
        - Colors each link uniquely using MATLAB’s lines colormap.
        - Adds semispherical caps at start and end of the link.
%}
function DrawLink(Qp, qp_base, i, scale)
    if nargin < 4
        scale = 1;
    end
    height_tot = norm(Qp);
    axis_link = (Qp)/height_tot;
    start_pos = qp_base;
    end_pos = qp_base + Qp;
     
    % Define cylinder parameters
    radius = 0.025 * scale; % Adjust as needed
    numPoints = 20; % Number of points around the cylinder

    % Compute the height of the cylinder (distance between origins)
    height = height_tot - 2*radius;

    % Generate a cylinder along the z-axis
    [X, Y, Z] = cylinder(radius, numPoints);

    % By default the generated points are along the z as the axis
    Z = Z * height; % Scale the cylinder height
    z_axis = [0; 0; 1]; % Default cylinder direction

    v = cross(z_axis, axis_link);
    s = norm(v);
    c = dot(z_axis, axis_link);
    Vx = [  0   -v(3)  v(2);
            v(3)  0   -v(1);
           -v(2)  v(1)  0 ];
    R = eye(3) + Vx + (Vx * Vx) * ((1 - c) / (s^2 + eps)); % Rodrigues' formula

    % Transform cylinder points to align with the link direction
    XYZ = R * [X(:)'; Y(:)'; Z(:)']; 
    X = reshape(XYZ(1, :), size(X)) + start_pos(1) + radius*axis_link(1);
    Y = reshape(XYZ(2, :), size(Y)) + start_pos(2) + radius*axis_link(2);
    Z = reshape(XYZ(3, :), size(Z)) + start_pos(3) + radius*axis_link(3);

    % Assign different colors for each link
    colors = lines(10);
    linkColor = colors(mod(i-1, size(colors, 1)) + 1, :);

    % Plot the cylinder
    surf(X, Y, Z, 'FaceColor', linkColor, 'EdgeColor', 0.7*linkColor);

    % Semisphere at the tips of the link with centers at start_pos + radius*axis
    [Xf, Yf, Zf] = sphere(numPoints); % Generate full sphere
    Xe = Xf;
    Ye = Yf;
    Ze = Zf;
    Zf(Zf > 0) = 0;  % Keep only the top hemisphere
    Ze(Ze < 0) = 0;
    % Transform and position semispheres
    semisphere_pts_f = R * [Xf(:)'; Yf(:)'; Zf(:)']; 
    semisphere_pts_e = R * [Xe(:)'; Ye(:)'; Ze(:)'];

    % Place at start and end
    X_start = reshape(semisphere_pts_f(1,:), size(Xf)) * radius + start_pos(1) + radius*axis_link(1);
    Y_start = reshape(semisphere_pts_f(2,:), size(Yf)) * radius + start_pos(2) + radius*axis_link(2);
    Z_start = reshape(semisphere_pts_f(3,:), size(Zf)) * radius + start_pos(3) + radius*axis_link(3);

    X_end = reshape(semisphere_pts_e(1,:), size(Xe)) * radius + end_pos(1) - radius*axis_link(1);
    Y_end = reshape(semisphere_pts_e(2,:), size(Ye)) * radius + end_pos(2) - radius*axis_link(2);
    Z_end = reshape(semisphere_pts_e(3,:), size(Ze)) * radius + end_pos(3) - radius*axis_link(3);

    % Plot semispheres
    surf(X_start, Y_start, Z_start, 'FaceColor', linkColor, 'EdgeColor', 0.7*linkColor);
    surf(X_end, Y_end, Z_end, 'FaceColor', linkColor, 'EdgeColor', 0.7*linkColor);
end

%{
    draw_transparent_equilateral_triangle_centered_base_platform: draw a
                         semi‐transparent equilateral triangle at a given frame.
    
    Inputs:
        T – 4×4 homogeneous transform of triangle centroid.
        a – scalar “radius” controlling triangle side length (side = a√3).
    
    Outputs:
        (none) – patches a gray, semi‐transparent triangle and colored edges.
    
    Details:
        - Constructs triangle in local XY plane.
        - Centers vertices around centroid before transforming.
        - Colors edges magenta, yellow, and cyan.
%}
function draw_transparent_equilateral_triangle_centered_base_platform(T,a)
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
    
    patch('Faces', [1 2 3], 'Vertices', vertices_global, ...
          'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.5, ...  
          'EdgeColor', 'none');
    
    edge_colors = [1 0 1;
                   1 1 0;
                   0 1 1];
    
    edges = [1, 2;
             2, 3;
             3, 1];
    
    % Draw each edge with its specified color
    for i = 1:3
        edge_idx = edges(i, :);
        pts = vertices_global(edge_idx, :);
        line(pts(:,1), pts(:,2), pts(:,3), 'Color', edge_colors(i,:), 'LineWidth', 1.5);
    end
end

%{
    draw_transparent_equilateral_triangle_centered_moving_platform: draw a
                         semi‐transparent equilateral triangle for moving platform.
    
    Inputs:
        T – 4×4 homogeneous transform of triangle centroid.
        a – scalar “radius” controlling triangle side length (side = a√3).
    
    Outputs:
        (none) – patches a yellow, semi‐transparent triangle and colored edges.
    
    Details:
        - Similar to base platform but flips vertex ordering for orientation.
        - Colors edges red, green, and blue.
%}
function draw_transparent_equilateral_triangle_centered_moving_platform(T,a)
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
    
    patch('Faces', [1 2 3], 'Vertices', vertices_global, ...
          'FaceColor', 'yellow', 'FaceAlpha', 0.5, ...  
          'EdgeColor', 'none');
    
    edge_colors = [1 0 0;
                   0 1 0;
                   0 0 1];
    
    edges = [1, 2;
             2, 3;
             3, 1];
    
    % Draw each edge with its specified color
    for i = 1:3
        edge_idx = edges(i, :);
        pts = vertices_global(edge_idx, :);
        line(pts(:,1), pts(:,2), pts(:,3), 'Color', edge_colors(i,:), 'LineWidth', 1.5);
    end
end

%{
    DrawGripper: render the gripper assembly between two serial backbone links.
    
    Inputs:
        T_gripper_base – 4×4 homogeneous transform of link‐2 reference frame.
        T_gripper_end  – 4×4 homogeneous transform of link‐3 reference frame.
    
    Outputs:
        (none) – draws:
                 • Cylindrical wrist segment.
                 • Base and offset gripper plates.
                 • Side, front, back, and top cap walls.
    
    Details:
        - Computes stem height as half the distance between base and end frames.
        - Orients end plate via a π/2 rotation.
        - Builds rectangular prism jaws with thickness fraction of length.
        - Uses semi‐transparent purple shading for all surfaces.
%}
function DrawGripper(T_gripper_base, T_gripper_end)
    %{
        drawGripper: render the gripper geometry between link 2 and link 3.
    
        Inputs:
            T_0_2 – 4×4 homogeneous transform of joint 2 in the base frame.
            T_0_3 – 4×4 homogeneous transform of joint 3 in the base frame.
    
        No outputs. The function:
          1. Computes the gripper’s base plate offset from link 2.
          2. Aligns the gripper’s end plate with link 3 and computes its length.
          3. Builds a cylindrical “wrist” segment at link 2.
          4. Constructs the rectangular gripper jaws:
             • Base and offset faces
             • Outer and inner side walls (left & right)
             • Front and back walls
             • Top caps
          5. Draws all surfaces with semi‐transparent purple shading.
    %}

    facecolor = [0.7, 0, 0.7];
    edgecolor = [0.9, 0, 0.9];
    full_len = norm(T_gripper_base(1:3,4) - T_gripper_end(1:3,4));
    stem_height = 0.5 * full_len;
    T_stem_end = T_gripper_base * [1, 0, 0, 0;
                     0, 1, 0, 0;
                     0, 0, 1, stem_height;
                     0, 0, 0, 1];
    T_gripper_end = T_gripper_end * [cos(pi/2), 0, -sin(pi/2), 0;
                     0, 1, 0, 0;
                     sin(pi/2), 0,  cos(pi/2), 0;
                     0, 0, 0, 1];
    
    length_gripper = norm((T_gripper_end(1:3,4) - T_stem_end(1:3,4)));
    norm_dir = (T_gripper_end(1:3,4) - T_stem_end(1:3,4))/length_gripper;
    width_gripper = 0.4*length_gripper; 
    height_gripper = 0.4*length_gripper;
    thickness_gripper = length_gripper/10;
    T_gripper_end(1:3,4) = T_stem_end(1:3,4) + norm_dir*length_gripper;
    z = T_gripper_end(1:3,3);
    perpend_dir = cross(norm_dir,z);

    % Draw Cylinder Link
    vertical_num_points = 50;
    radius_vect = 0.005 * ones(vertical_num_points, 1);
    num_points = 20;
    [Xc, Yc, Zc] = cylinder(radius_vect, num_points);
    xb = Xc(1, :);
    yb = Yc(1, :);
    zb = Zc(1, :);
    Zc = Zc * stem_height;
    ptsLocal = [ Xc(:)';  Yc(:)';  Zc(:)';  ones(1, numel(Xc))];
    ptsWorld = T_gripper_base * ptsLocal;

    Xw = reshape(ptsWorld(1,:), size(Xc));
    Yw = reshape(ptsWorld(2,:), size(Yc));
    Zw = reshape(ptsWorld(3,:), size(Zc));

    botLocal = [ xb;
                 yb;
                 zb;
                 ones(1, numel(xb)) ];
    
    botWorld = T_gripper_base * botLocal;

    surf(Xw, Yw, Zw, 'FaceColor',facecolor, 'FaceAlpha', 0.3, 'EdgeColor',edgecolor);
    patch( botWorld(1,:), botWorld(2,:), botWorld(3,:), facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor );

    % Base face vertices
    v_0_base = T_stem_end(1:3,4) + perpend_dir*width_gripper/2 + T_gripper_end(1:3,3)*height_gripper/2;
    v_1_base = T_stem_end(1:3,4) + perpend_dir*width_gripper/2 - T_gripper_end(1:3,3)*height_gripper/2;
    v_2_base = T_stem_end(1:3,4) - perpend_dir*width_gripper/2 - T_gripper_end(1:3,3)*height_gripper/2;
    v_3_base = T_stem_end(1:3,4) - perpend_dir*width_gripper/2 + T_gripper_end(1:3,3)*height_gripper/2;
    vertices_base = transpose([v_0_base, v_1_base, v_2_base, v_3_base]);
    patch('Vertices', vertices_base, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Offset base face vertices
    v_0_base_offset = v_0_base + norm_dir*thickness_gripper - perpend_dir*thickness_gripper;
    v_1_base_offset = v_1_base + norm_dir*thickness_gripper - perpend_dir*thickness_gripper;
    v_2_base_offset = v_2_base + norm_dir*thickness_gripper + perpend_dir*thickness_gripper;
    v_3_base_offset = v_3_base + norm_dir*thickness_gripper + perpend_dir*thickness_gripper;
    vertices_offset_base = transpose([v_0_base_offset, v_1_base_offset, v_2_base_offset, v_3_base_offset]); 
    patch('Vertices', vertices_offset_base, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Side wall left
    % v_0_wall_l = v_3_base;
    v_1_wall_l = v_3_base + norm_dir*length_gripper;
    v_2_wall_l = v_2_base + norm_dir*length_gripper;
    % v_3_wall_l = v_2_base;

    vertices_wall_l = transpose([v_3_base, v_1_wall_l, v_2_wall_l, v_2_base]); 
    patch('Vertices', vertices_wall_l, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Side wall right 
    % v_0_wall_r = v_0_base;
    v_1_wall_r = v_0_base + norm_dir*length_gripper;
    v_2_wall_r = v_1_base + norm_dir*length_gripper;
    % v_3_wall_r = v_1_base;
    vertices_wall_r = transpose([v_0_base, v_1_wall_r, v_2_wall_r, v_1_base]); 
    patch('Vertices', vertices_wall_r, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Inner wall left
    % v_0_wall_l_inner = v_3_base_offset;
    v_1_wall_l_inner = v_3_base_offset + norm_dir*(length_gripper - thickness_gripper);
    v_2_wall_l_inner = v_2_base_offset + norm_dir*(length_gripper - thickness_gripper);
    % v_3_wall_l_inner = v_2_base_offset;
    vertices_wall_l_inner = transpose([v_3_base_offset, v_1_wall_l_inner, v_2_wall_l_inner, v_2_base_offset]); 
    patch('Vertices', vertices_wall_l_inner, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Inner wall right
    % v_0_wall_r_inner = v_0_base_offset;
    v_1_wall_r_inner = v_0_base_offset + norm_dir*(length_gripper - thickness_gripper);
    v_2_wall_r_inner = v_1_base_offset + norm_dir*(length_gripper - thickness_gripper);
    % v_3_wall_r_inner = v_1_base_offset;
    vertices_wall_r_inner = transpose([v_0_base_offset, v_1_wall_r_inner, v_2_wall_r_inner, v_1_base_offset]); 
    patch('Vertices', vertices_wall_r_inner, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Front wall
    vertices_front_wall = transpose([v_1_wall_l, v_1_wall_l_inner, v_3_base_offset, v_0_base_offset, v_1_wall_r_inner, v_1_wall_r, v_0_base, v_3_base]);
    patch('Vertices', vertices_front_wall, 'Faces', [1,2,3,4,5,6,7,8], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Back wall
    vertices_back_wall = transpose([v_2_wall_l, v_2_wall_l_inner, v_2_base_offset, v_1_base_offset, v_2_wall_r_inner, v_2_wall_r, v_1_base, v_2_base]);
    patch('Vertices', vertices_back_wall, 'Faces', [1,2,3,4,5,6,7,8], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);
    
    % Top wall left
    vertices_top_wall_left = transpose([v_1_wall_l, v_2_wall_l, v_2_wall_l_inner, v_1_wall_l_inner]);
    patch('Vertices', vertices_top_wall_left, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);

    % Top wall right
    vertices_top_wall_right = transpose([v_1_wall_r, v_2_wall_r, v_2_wall_r_inner, v_1_wall_r_inner]);
    patch('Vertices', vertices_top_wall_right, 'Faces', [1,2,3,4], 'FaceColor', facecolor, 'FaceAlpha', 0.3, 'EdgeColor', edgecolor);
end

%{
    DrawPen: render the pen link between gripper base and pen tip.
    
    Inputs:
        T_pen_base – 4×4 homogeneous transform at pen mount point.
        T_pen_tip  – 4×4 homogeneous transform at pen tip position.
    
    Outputs:
        (none) – draws two scaled cylindrical segments via DrawLink:
                 • Base‐to‐intermediate (scale 0.3).
                 • Intermediate‐to‐tip (scale 0.15).
    
    Details:
        - Offsets the tip back by 5 cm to insert intermediate joint.
        - Leverages existing DrawLink for cylinder rendering.
%}
function DrawPen(T_pen_base,T_pen_tip)
    T_pen_intermediate = T_pen_tip;
    % len = norm(T_0_5(1:3,4) - T_0_4(1:3,4));
    T_pen_intermediate(1:3,4) = T_pen_tip(1:3,4) - 0.05*(T_pen_tip(1:3,4) - T_pen_base(1:3,4));
    vec = T_pen_intermediate(1:3,4) - T_pen_base(1:3,4);
    DrawLink(transpose(vec), transpose(T_pen_base(1:3,4)),1,0.3);
    vec = T_pen_tip(1:3,4) - T_pen_intermediate(1:3,4);
    DrawLink(transpose(vec),transpose(T_pen_intermediate(1:3,4)),1,0.15);
end
