%{
    res_rates: drive pen tip to a target via resolved‐rates IK with joint‐limit weighting.
    
    Inputs:
        P_des        – 3×1 desired pen-tip position.
        n            – total number of serial joints (3 per module).
        q_start      – n×1 initial RRP joint vector.
        p_his        – (opt.) history of pen-tip positions (m×3).
        delta_qp_his – (opt.) history of leg-length changes.
    
    Outputs:
        q_curr       – n×1 final joint vector.
        iter         – iteration count.
        run_time     – total time = iter·dt.
        p_his        – updated pen-tip path history.
        delta_qp_his – updated leg-length-change history.
    
    Key steps:
      • Compute FK (dirkin_serial) and parallel IK (invkin_parallel).
      • Loop until position error ≤ eps_p:
        – Compute Cartesian velocity v_d with speed scaling.
        – Build weight matrix W near joint limits.
        – Solve for q_dot via weighted least squares.
        – Integrate q_curr, enforce limits.
        – Update FK, record histories.
      • Optional plotting every 100 iters.
%}
function [q_curr, iter, run_time, p_his, delta_qp_his] = res_rates(P_des,n,q_start, p_his, delta_qp_his)
    if nargin < 1 
        P_des = [-0.25; -0.5; 1.25];
        n = 12;
        q_start = [0;0;0.4;0;0;0.4;0;0;0.4;0;0;0.4];
        p_his = [];
        delta_qp_his = [];
        close all;
    end
    q_curr = q_start;

    dt = 0.001;
    
    eps_p = 0.001; 
    v_min = 0.03;
    v_max = 0.1;
    lambda_trans = 5;
    
    load('Robot_Desc.mat', 'Lim_roll', 'Lim_pitch', 'Lim_len_min', 'Lim_len_max');
    % vw = evalin('base','VW_RECORDER');

    [fp,fs] = dirkin_serial(q_curr);
    [~, qp_val, ~, ~, ~] = invkin_parallel(fp);
    qp_val_new = qp_val;

    gripper_height = 0.15;
    pen_height = 0.1;
    T_n_Gripper = compute_transformation_matrix_serial(0, 0, gripper_height, 0);
    T_Gripper = fs(:,:,n)*T_n_Gripper;
    T_Gripper_Pen = compute_transformation_matrix_serial(0, 0, pen_height, 0);
    T_Pen = T_Gripper * T_Gripper_Pen;
    P_curr = T_Pen(1:3,4);
    P_err = P_des - P_curr;
    p_err = norm(P_err);

    figure(1);
    clf;
    hold on; grid on; axis equal;
    view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    draw_robot_serial(fs);

    iter = 0;
    
    while ((p_err > eps_p)) 
        iter = iter + 1;
        
        p_his((end+1), :) = transpose(T_Pen(1:3,4));
        
        v_d_dir = P_err / p_err;
        if p_err > (eps_p * lambda_trans)
            v_d = v_d_dir * v_max;  
        elseif p_err > eps_p
            v_d = v_d_dir * (v_min + (v_max - v_min)*(p_err - eps_p)/(eps_p*(lambda_trans - 1)));  
        else
            v_d = zeros(3,1);
        end

        % Compute Jacobian at the gripper
        jacobian_full = compute_jacobian_serial(fs);
        jacobian = jacobian_full(1:3,:);
        
        % % Compute joint velocities using pseudoinverse
        % q_dot = pinv(jacobian) * v_d;

        
        W_set = [];
        for i = 1:n/3
            W1 = 1 + abs((((2*Lim_roll)^2)*q_curr((i-1)*3 + 1))/ ...
                        (2 * ((Lim_roll - q_curr((i-1)*3 + 1))^2) * ((-Lim_roll - q_curr((i-1)*3 + 1))^2)));
            W2 = 1 + abs((((2*Lim_pitch)^2)*q_curr((i-1)*3 + 2))/ ...
                        (2 * ((Lim_pitch - q_curr((i-1)*3 + 2))^2) * ((-Lim_pitch - q_curr((i-1)*3 + 2))^2)));
            W3 = 1 + abs((((Lim_len_max - Lim_len_min)^2)*(q_curr((i-1)*3 + 3) - (Lim_len_max + Lim_len_min)/2))/ ...
                        (2 * ((Lim_len_max - q_curr((i-1)*3 + 3))^2) * ((q_curr((i-1)*3 + 3) - Lim_len_min)^2)));
            W_set = [W_set, W1, W2, W3];
        end

        W = diag(W_set);
        q_dot = inv(W) * transpose(jacobian) * inv(jacobian * inv(W) * transpose(jacobian)) * v_d;

        q_curr = q_curr + q_dot * dt;
        for i = 1:n/3
            base = (i-1)*3;
            q1 = q_curr(base + 1);   % roll
            q2 = q_curr(base + 2);   % pitch
            q3 = q_curr(base + 3);   % length
        
            if q1 < -Lim_roll || q1 > Lim_roll ...
              || q2 < -Lim_pitch || q2 > Lim_pitch ...
              || q3 <  Lim_len_min || q3 > Lim_len_max
                disp("Weight Matrix:");
                disp(W);
                error("Beyond Set Limits");
            end
        end

        [fp,fs] = dirkin_serial(q_curr);

        T_Gripper = fs(:,:,n)*T_n_Gripper;
        T_Pen = T_Gripper * T_Gripper_Pen;

        [Qp, qp_val, qp_base, ~, M] = invkin_parallel(fp);

        delta_qp = qp_val_new - qp_val;
        delta_qp_his((end+1))  = norm(delta_qp);

        qp_dot = zeros(n*2,1);
        for i = 1:(n/3)
            if i == 1
                Qs_i_1 = [0;0;0];
            else
                Qs_i_1 = fp(1:3,4,i-1);
                Qs_i_1_dot = q_dot(1 : (i-2)*3 + 3);
            end
            Qs_i = fp(1:3,4,i);
            Qs_i_dot = q_dot(1 : (i-1)*3 + 3);
            
            Qs = transpose(Qs_i - Qs_i_1);
            if (i ~= 1)
                Xs_dot_rel = jacobian_full(:,1:(i-1)*3 + 3) * Qs_i_dot - jacobian_full(:,1:(i-2)*3 + 3) * Qs_i_1_dot;
            else
                Xs_dot_rel = jacobian_full(:,1:3) * Qs_i_dot;
            end

            partial_jacobian_parallel = compute_partial_jacobian_parallel( ...
                                                Qp((i-1)*6 + 1 : (i-1)*6 + 6, :), ...
                                                qp_val((i-1)*6 + 1 : (i-1)*6 + 6), ...
                                                Qs, M((i-1)*3 + 1 : (i-1)*3 + 3, :));
            qp_dot((i-1)*6 + 1 : (i-1)*6 + 6) = partial_jacobian_parallel*Xs_dot_rel;
            
        end

        qp_val_new = qp_val + qp_dot*dt;

        P_curr = T_Pen(1:3,4);
        P_err = P_des - P_curr;
        p_err = norm(P_err);
        
        if mod(iter, 100) == 0
            figure(1);
            clf;
            hold on; grid on; axis equal;
            view(3);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            draw_robot_serial(fs);
            

            figure(2);
            clf;
            hold on; grid on; axis equal;
            view(3);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            plot3(p_his(:,1), p_his(:,2), p_his(:,3), 'c-', 'LineWidth', 2);
            draw_robot_parallel(Qp, qp_base, fs, T_Gripper, T_Pen);
            
            % writeVideo(vw,getframe(1));
        end
    end
    run_time = dt * iter;
    figure(1);
    clf;
    hold on; grid on; axis equal;
    view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    draw_robot_serial(fs);
    

    figure(2);
    clf;
    hold on; grid on; axis equal;
    view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    plot3(p_his(:,1), p_his(:,2), p_his(:,3), 'c-', 'LineWidth', 2);
    draw_robot_parallel(Qp, qp_base, fs, T_Gripper, T_Pen);
    % writeVideo(vw,getframe(1));

end