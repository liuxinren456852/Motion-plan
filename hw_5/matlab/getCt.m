function Ct = getCt(n_seg, n_order)
    %#####################################################
    % 计算选择矩阵C_t
    d_order = 4;
    
    ct_rows = d_order * 2 + (n_seg - 1) * 2 * d_order;
    ct_cols = 2 * d_order + (n_seg - 1) * d_order; 
    Ct = zeros(ct_rows, ct_cols);
    
    % 构造哈希表 目的是按照分段，起点，终点和阶数的顺序对每一个变量进行哈希映射
    d_vector = zeros( n_seg * 2 * d_order, 1) ;
    idx = 1;
    for k = 0:n_seg-1           % 第k段
        for t = 0:1             % 起点和终点
            for d = 0:d_order-1 % 微分阶数
                d_vector(idx) = k*100+t*10+d;
                idx = idx + 1;
            end
        end
    end
    
    % 固定起点
    col = 1; % 遍历的起点
    k = 0;
    t = 0;
    for d = 0:d_order-1
        val = k * 100 + t * 10 + d;
        [row,~] = find(d_vector == val);
        Ct(row, col) = 1;
        col = col + 1;
    end
    
    % 固定中间节点位置
    t = 1;
    d = 0;
    if  n_seg - 2 >= 0
        for k = 0:n_seg - 2
            % 前一段的终点
            val = k * 100 + t * 10 + d;
            [row,~] = find(d_vector == val);
            Ct(row, col) = 1;
            % 下一段的起点
            val = (k + 1) * 100 + (t -1) * 10 + d;
            [row,~] = find(d_vector == val);
            Ct(row, col) = 1;
            col = col + 1;
        end
    end
    
    % 固定末端状态
    k = n_seg - 1;
    t = 1;
    for d = 0:d_order-1
        val = k * 100 + t * 10 + d;
        [row,~] = find(d_vector == val);
        Ct(row, col) = 1;
        col = col + 1;
    end
    
    % 映射自由状态，满足连续性约束
    t = 1;
    if  n_seg - 2 >= 0
        for k = 0:n_seg - 2
            for d = 1:d_order-1
            % 前一段的终点
            val = k * 100 + t * 10 + d;
            [row,~] = find(d_vector == val);
            Ct(row, col) = 1;
            % 后一段的起点
            val = (k + 1) * 100 + (t - 1) * 10 + d;
            [row,~] = find(d_vector == val);
            Ct(row, col) = 1;

            col = col + 1;
            end
        end
    end
    
end