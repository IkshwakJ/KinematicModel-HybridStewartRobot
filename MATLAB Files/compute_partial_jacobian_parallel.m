%{
    compute_partial_jacobian_parallel: compute the partial Jacobian relating
    parallel‐leg joint velocities to the serial backbone motion.
    
    Inputs:
        Qp       – 6×3 matrix of unit vectors along each leg (rows correspond to legs 1–6).
        qp_val   – 6×1 vector of current leg extension/joint values.
        Qs       – 1×3 vector of the serial‐backbone joint axis direction.
        M        – 3×3 matrix of moving‐platform attachment point vectors (rows for points 1–3).
    
    Outputs:
        partial_jacobian_parallel – 6×(3+3) partial Jacobian matrix mapping
                                     [leg rates; backbone rate] to platform twist,
                                     computed as pinv(A) * B.
    
    Notes:
        - Six legs connect base/end points in the specified leg connectivity pattern:
            Leg 1: base pt 1 → end pt 2
            Leg 2: base pt 1 → end pt 3
            Leg 3: base pt 2 → end pt 3
            Leg 4: base pt 2 → end pt 1
            Leg 5: base pt 3 → end pt 1
            Leg 6: base pt 3 → end pt 2
        - A is the diagonal matrix of leg joint values.
        - B stacks each leg’s unit vector and the moment arm term 
          cross((M(endPt,:) + Qs), Qp(leg,:)).
        - Uses the Moore–Penrose pseudoinverse to invert A.
%}
function partial_jacobian_parallel = compute_partial_jacobian_parallel(Qp, qp_val, Qs, M)
    % There are six legs for the parallel robot and therefore Qp and qp_val
    % will have six rows. There will be 3 vectors in M (stored as rows).
    % Qs is the vector for the backbone joint and should be a row. 
    % Leg connectivity:
        %   Leg 1 connects base pt 1 → end pt 2
        %   Leg 2 connects base pt 1 → end pt 3
        %   Leg 3 connects base pt 2 → end pt 3
        %   Leg 4 connects base pt 2 → end pt 1
        %   Leg 5 connects base pt 3 → end pt 1
        %   Leg 6 connects base pt 3 → end pt 2
    

    % Matrix A is the premultiplier to the joint velocity vector for the
    % parallel robot.
    A = [qp_val(1), 0, 0, 0, 0, 0;
         0, qp_val(2), 0, 0, 0, 0;
         0, 0, qp_val(3), 0, 0, 0;
         0, 0, 0, qp_val(4), 0, 0;
         0, 0, 0, 0, qp_val(5), 0;
         0, 0, 0, 0, 0, qp_val(6)];

    % Matrix B is the premultiplier to the joint velocity vector for the
    % serial robot.
    B = [Qp(1, :), cross((M(2, :) + Qs), Qp(1,:));
         Qp(2, :), cross((M(3, :) + Qs), Qp(2,:));
         Qp(3, :), cross((M(3, :) + Qs), Qp(3,:));
         Qp(4, :), cross((M(1, :) + Qs), Qp(4,:));
         Qp(5, :), cross((M(1, :) + Qs), Qp(5,:));
         Qp(6, :), cross((M(2, :) + Qs), Qp(6,:))];

    partial_jacobian_parallel = pinv(A) * B;
end