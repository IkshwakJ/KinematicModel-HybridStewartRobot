%{
    simulate_robot: run IK simulation over a predefined path and plot performance.
    
    Inputs:
        stack_count – (opt.) number of RRP modules (default = 3).
        shape       – (opt.) path shape identifier: 
                      1 = half‐circle; 2 = rotated quarter‐circle; 3 = point (default = 1).
    
    Outputs:
        (none) – generates plots of:
                 • Iteration count per path step.
                 • Run time per path step.
                 • Parallel leg‐length change history.
    
    Key steps:
      • Initialize backbone q (prismatic joints = 0.4 m).
      • Generate target points (X,Y,Z) based on shape.
      • Loop over points:
        – Call res_rates to solve IK, record iter count & run time.
      • Plot performance metrics after simulation.
%}
function simulate_robot(stack_count, shape)
    if nargin < 1
        stack_count = 3;
        shape = 1; 
    end
    
    if stack_count < 1 
        msg = "Invalid Number of Stacks";
        error(msg);
    elseif shape < 1 || shape > 3
        msg = "Invalid Shape";
        error(msg);
    end
    
    close all;
    shape = round(shape);

    q_backbone_default = zeros(stack_count*3,1);
    for i = 3:3:stack_count*3
        q_backbone_default(i) = 0.4;
    end
    
    point_count = 250;
    if shape == 1 % Half unit circle about the z axis at the top
        theta = transpose(linspace(-pi, pi, point_count));
        X = 0.25*cos(theta);
        Y = 0.25*sin(theta);
        Z = zeros(point_count,1) + stack_count * 0.4 + 0.25;
    elseif shape == 2 % Quarter unit circle rotated about x axis and shifted
        theta = transpose(linspace(-pi, pi, point_count));
        X = 0.125*cos(theta);
        Y = 0.125*sin(theta);
        Z = zeros(point_count,1) + stack_count * 0.4 + 0.25;
        XYZ = [X'; Y'; Z'; ones(1,point_count)];

        alpha = pi/8;
        T = [1, 0, 0, 0.125;
             0, cos(alpha), -sin(alpha), -0.125;
             0, sin(alpha), cos(alpha), 0;
             0, 0, 0, 1];
        XYZG = T * XYZ;
        X = transpose(XYZG(1,:));
        Y = transpose(XYZG(2,:));
        Z = transpose(XYZG(3,:));
    else
        point_count = 1;
        X = 0.125;
        Y = 0.125;
        Z = stack_count * 0.4 + 0.25;
    end

    % vw = VideoWriter('res_rates_movie.mp4','MPEG-4');
    % vw.FrameRate = 15;    
    % open(vw);
    % assignin('base','VW_RECORDER',vw);

    q_backbone_curr = q_backbone_default;
    iter_his     = zeros(point_count,1);
    run_time_his = zeros(point_count,1);
    p_his        = [];
    delta_qp_his = [];
    
    for i = 1:point_count
        P_des = [X(i); Y(i); Z(i)];
        [q_backbone_curr, it_count, rt, p_his, delta_qp_his] = ...
            res_rates(P_des,stack_count*3 ,q_backbone_curr, p_his, delta_qp_his);
        iter_his(i) = it_count;
        run_time_his(i) = rt;
        % p_his(end+1: end+size(p_vec,1), :) = p_vec;
        % delta_qp_his(end+1) = delta_qp_norm;
    end
    figure(3);
    plot(1:point_count, iter_his, 'LineWidth',1.5);
    xlabel('Step'); ylabel('Iterations'); grid on;
    title('Iteration Count per Step');

    %-- plot 2: run-time history
    figure(4);
    plot(1:point_count, run_time_his, 'LineWidth',1.5);
    xlabel('Step'); ylabel('Run Time (s)'); grid on;
    title('Run Time per Step');

    %-- plot 3: delta‐q̇ parallel history
    figure(5);
    plot(1:size(delta_qp_his,2), delta_qp_his, 'LineWidth',1.5);
    xlabel('SubStep'); ylabel('\Delta q_p'); grid on;
    title('Delta-q Parallel History');

    % vw = evalin('base','VW_RECORDER');
    % close(vw);
    % clear vw
    % disp('Movie saved to res_rates_movie.mp4')
end