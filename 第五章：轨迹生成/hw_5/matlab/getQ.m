function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = zeros(n_order + 1, n_order + 1);
        %#####################################################
        % STEP 1.1: 计算第k段的矩阵Q_k 
        for i = 4:n_order
            for j = 4:n_order
                Q_k(i+1,j+1) = factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)/(i+j-n_order)*ts(k)^(i+j-n_order);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end