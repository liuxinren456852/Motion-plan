function M = getM(n_seg, n_order, ts)
% 将多项式系数映射到对应的状态变量和其导数
    coeff = [1,  1,  1,  1,  1,  1,  1,  1,;
             0,  1,  2,  3,  4,  5,  6,  7,;
             0,  0,  2,  6,  12, 20, 30, 42;
             0,  0,  0,  6,  24, 60, 120,210];
    M = [];
    for k = 1:n_seg
        M_k = zeros(8,8);
        %#####################################################
        % 计算每一个M_k 
        t = ts(k);
        % 起点映射
        for i = 0:3
            M_k(i+1, i+1) = coeff(i+1, i+1);
        end
        
        % 末端映射
        for i = 0:3
            for j = i:n_order
                if (i == j)
                    M_k(i+4+1, j+1) = coeff(i+1, j+1) ;
                else
                    M_k(i+4+1, j+1) = coeff(i+1, j+1) * t^(j - i);
                end
            end
        end
        
        
        M = blkdiag(M, M_k);
    end
end