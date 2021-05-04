function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j 的起点约束, 
    Aeq_start = zeros(4, n_all_poly);
    % STEP 2.1: write expression of Aeq_start and beq_start
    Aeq_start(1:4,1:8) = getCoeffCons(0);
    beq_start =  start_cond';
    
    %#####################################################
    % p,v,a,j 的终端约束
    Aeq_end = zeros(4, n_all_poly);
    t = ts(end);
    Aeq_end(1:4, end-7:end) = getCoeffCons(t);
    beq_end =  end_cond';
    
    %#####################################################
    % 中点的位置约束
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    
    for k = 0:1:n_seg-2
        beq_wp(k+1, 1) = waypoints(k+2);
        coeff = getCoeffCons(ts(k+1));
        Aeq_wp(k+1, 1+k*8:8+k*8) = coeff(1, :);  
    end
    
    %#####################################################
    % 连续性约束
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    
    for k = 0:1:n_seg-2 
        Aeq_con(1+4*k:4+4*k,1+8*k:8+8*k) = getCoeffCons(ts(k+1));
        Aeq_con(1+4*k:4+4*k,1+8*(k+1):8+8*(k+1)) = -getCoeffCons(0);            
    end
    
    %#####################################################
    % 构造约束矩阵
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end