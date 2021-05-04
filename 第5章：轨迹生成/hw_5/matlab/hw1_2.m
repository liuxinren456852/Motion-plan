clc;clear;close all;
path = ginput() * 100.0;

n_order = 7;
n_seg = size(path, 1) - 1;
n_poly_perseg = n_order + 1;

ts = zeros(n_seg, 1);

% 使用距离计算每两个点之间的时间分配
dist = zeros(n_seg, 1);
dist_sum = 0;
T = 25;

t_sum = 0;
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum + dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i) / dist_sum * T;
    t_sum = t_sum + ts(i);
end
ts(n_seg) = T - t_sum;

% 简单地设置每个分段的时间间隔为1
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % 获取对应的多项式系数
    Pxi = poly_coef_x((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1); 
    Pyi = poly_coef_y((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1);
    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(flip(Pxi),t);
        Y_n(k)  = polyval(flip(Pyi),t);
        k = k+1;
    end
end

plot(X_n, Y_n ,'Color',[0 1.0 0],'LineWidth',2);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0]';
    end_cond =   [waypoints(end), 0, 0, 0]';
    
    %#####################################################
    % 获取Q矩阵
    Q = getQ(n_seg, n_order, ts);
    
    %#####################################################
    % STEP 1: 计算M矩阵
    M = getM(n_seg, n_order, ts);
    
    %#####################################################
    % STEP 2: 计算C_t
    
    Ct = getCt(n_seg, n_order);
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;
    
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    
    
    %#####################################################
    % STEP 3: 计算dF
    dF = zeros(8+(n_seg-1),1);
    
    % 起点位置约束
    dF(1:4) = start_cond;
    
    
    % 中间节点的位置约束
    for i=1:1:n_seg-1
        dF(4+i) = waypoints(i+1);    

    end
    
    % 末端节点的位置约束
    dF(end-3:end) = end_cond;
    
    % 计算dP
    dP = -inv(R_pp) * R_fp' * dF;
    
    % 还原多项式系数
    poly_coef = inv(M) * Ct * [dF;dP];
end