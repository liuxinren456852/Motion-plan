function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    d_order = (n_order + 1)/2;
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 位置约束

    v = ones(n_all_poly,1);
    Aieq_p = diag(v);
    bieq_p = zeros(n_all_poly,1);
    for i = 0:n_seg - 1
        bieq_p(i *(n_order + 1)+1:i *(n_order + 1)+n_order + 1) = corridor_range(i+1,2);
    end
    
    Aieq_p = [Aieq_p;-Aieq_p];
    bieq_p = [bieq_p;bieq_p];
    
    for i = 0:n_seg - 1
        bieq_p(n_all_poly + i *(n_order + 1)+1:n_all_poly + i *(n_order + 1)+n_order + 1)...
            = (-1) * corridor_range(i + 1,1);
    end
    

    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = zeros(n_order * n_seg, n_all_poly);
    j = 1;
    for i = 1:n_order * n_seg
        Aieq_v(i,j:j+1) = n_order * [-1, 1];
        if mod(j + 1, n_order + 1) == 0
            j = j + 2;
        else
            j = j + 1;
        end
    end
    
    Aieq_v = [Aieq_v;-Aieq_v];
    bieq_v = ones(2 * n_order * n_seg,1)* v_max;

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = zeros((n_order - 1) * n_seg, n_all_poly);
    j = 1;
    for i = 1:(n_order - 1) * n_seg
        Aieq_a(i,j:j+2) = n_order * (n_order - 1)*[1, -2, 1];
        if mod(j + 2, n_order + 1) == 0
            j = j + 3;
        else
            j = j + 1;
        end
    end
    
    Aieq_a = [Aieq_a;-Aieq_a];
    bieq_a = ones((n_order - 1) * n_seg * 2,1)*a_max;
    %#####################################################
    % combine all components to form Aieq and bieq   
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
%     Aieq = Aieq_p;
%     bieq = bieq_p;
end