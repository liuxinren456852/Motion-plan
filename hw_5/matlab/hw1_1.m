clc;clear;close all;
path = ginput() * 100.0;

n_order       = 7;              % 多项式的阶数
n_seg         = size(path,1)-1; % 分段数
n_poly_perseg = (n_order+1);    % 每段的系数个数

ts = zeros(n_seg, 1);

%根据距离长度计算每两个点之间的时间分配
dist     = zeros(n_seg, 1);
dist_sum = 0;
T        = 25;
t_sum    = 0;

for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum+dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i)/dist_sum*T;
    t_sum = t_sum+ts(i);
end
ts(n_seg) = T - t_sum;

% % 简单地设置每一段的时间分配为1
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

% 分别对x和y方向求取对应的多项式系数
poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);


% 用于显示轨迹
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    Pxi = poly_coef_x((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1); 
    Pyi = poly_coef_y((n_order+1)*(i)+1:(n_order+1)*(i)+n_order+1);
    
    for t = 0:tstep:ts(i+1)
        X_n(k)  = polyval(flip(Pxi), t);
        Y_n(k)  = polyval(flip(Pyi), t);
        k = k + 1;
    end
end
 
plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));

% Minisnap求解器
function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    % 起点约束
    start_cond = [waypoints(1), 0, 0, 0];
    % 终点约束
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: 计算Q矩阵
    Q = getQ(n_seg, n_order, ts);
    
    %#####################################################
    % STEP 2: 计算对应的约束矩阵A_beq
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    
    f = zeros(size(Q,1),1);
    % 求解多项式系数
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end