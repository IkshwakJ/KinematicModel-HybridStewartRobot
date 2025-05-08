%{
    draw_robot_serial: visualize the serial backbone with platform bases,
                       links, moving platforms, and coordinate frames.
    
    Inputs:
        frames_serial – 4×4×m array of homogeneous transforms for each serial
                        backbone frame, grouped in triplets per RRP module.
    
    Outputs:
        (none) – opens a 3D figure showing:
                 • Base platform triangles at origin and each module base.
                 • Cylindrical serial links between revolute and prismatic joints.
                 • Moving platform triangles at each module top.
                 • Coordinate axes at every serial frame.
    
    Behavior:
        - Loads base & moving platform radii from Robot_Desc.mat.
        - Draws a transparent base triangle at the world origin.
        - For each module (i=1:3:end):
            • Draws base triangle at the previous frame.
            • Renders the second link segment via DrawLink.
            • Draws moving platform triangle at the top frame.
        - Finally, iterates over all frames to overlay coordinate axes via DrawCoordinateAxes.
%}
function draw_robot_serial(frames_serial)
    len = size(frames_serial,3);
    load('Robot_Desc.mat');
    draw_transparent_equilateral_triangle_centered_base_platform(eye(4),Radius_Base);
    for i = 1:3:len
        if( i ~= 1)
            draw_transparent_equilateral_triangle_centered_base_platform(frames_serial(:,:,i-1),Radius_Base);
        end
        DrawLink(frames_serial(:,:,i+1),frames_serial(:,:,i+2),i);
        draw_transparent_equilateral_triangle_centered_moving_platform(frames_serial(:,:,i+2),Radius_Moving);
    end
    for i = 1:len 
        DrawCoordinateAxes(frames_serial(:,:,i),i);
    end 
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
function DrawLink(T_i_1,T_i,i)
    height_tot = norm(T_i(1:3,4) - T_i_1(1:3,4));
    axis_link = (T_i(1:3,4) - T_i_1(1:3,4))/height_tot;
    start_pos = T_i_1(1:3,4);
    end_pos = T_i(1:3,4);
    
     
    % Define cylinder parameters
    radius = 0.05; % Adjust as needed
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
    DrawCoordinateAxes: render a set of 3D coordinate axes at a given frame.
    
    Inputs:
        T – 4×4 homogeneous transform defining the frame (rotation + translation).
        i – integer index used to label the axes (e.g., x1, y1, z1).
    
    Outputs:
        (none) – adds three colored lines and text labels to the current 3D plot:
                 • X-axis in red
                 • Y-axis in green
                 • Z-axis in blue
    
    Details:
        - Extracts origin and unit direction vectors from T.
        - Uses a fixed scale factor to determine axis length.
        - Draws each axis with a line of width 2.
        - Places text labels at the end of each axis with the index suffix.
%}
function DrawCoordinateAxes(T,i)
    origin = T(1:3,4);
    x_axis = T(1:3,1);
    y_axis = T(1:3,2);
    z_axis = T(1:3,3);
    
    % Define a scale factor for the length of the axes to be drawn
    scale = 1;  
    
    % Plot the coordinate axes
    line([origin(1), origin(1) + scale*x_axis(1)], ...
         [origin(2), origin(2) + scale*x_axis(2)], ...
         [origin(3), origin(3) + scale*x_axis(3)], 'Color', 'r', 'LineWidth', 2);
    line([origin(1), origin(1) + scale*y_axis(1)], ...
         [origin(2), origin(2) + scale*y_axis(2)], ...
         [origin(3), origin(3) + scale*y_axis(3)], 'Color', 'g', 'LineWidth', 2);
    line([origin(1), origin(1) + scale*z_axis(1)], ...
         [origin(2), origin(2) + scale*z_axis(2)], ...
         [origin(3), origin(3) + scale*z_axis(3)], 'Color', 'b', 'LineWidth', 2);
    
    % Optionally, add labels for the axes:
    text(origin(1)+scale*x_axis(1), origin(2)+scale*x_axis(2), origin(3)+scale*x_axis(3), sprintf('x%d',i), 'FontSize', 12, 'Color', 'k');
    text(origin(1)+scale*y_axis(1), origin(2)+scale*y_axis(2), origin(3)+scale*y_axis(3), sprintf('y%d',i), 'FontSize', 12, 'Color', 'k');
    text(origin(1)+scale*z_axis(1), origin(2)+scale*z_axis(2), origin(3)+scale*z_axis(3), sprintf('z%d',i), 'FontSize', 12, 'Color', 'k');
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
