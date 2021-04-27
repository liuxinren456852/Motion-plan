% 获取Q矩阵以及映射矩阵M
function [Q, M] = getQM(n_seg, n_order, ts)
    d_order = (n_order + 1)/2;
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 计算第k段的Q_k 
        Q_k = zeros(n_order + 1, n_order + 1);
        for i = d_order:n_order
            for j = d_order:n_order
                Q_k(i+1,j+1) = factorial(i)/factorial(i-d_order)*...
                    factorial(j)/factorial(j-d_order)/...
                    (i+j-n_order)*ts(k)^(i+j-n_order);
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end