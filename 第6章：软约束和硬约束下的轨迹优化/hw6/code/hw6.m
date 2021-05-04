clc;clear;close all
v_max = 400;    % 最大速度
a_max = 400;    % 最大加速度
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

%% 
path = [50, 50;
       100, 120;
       180, 150;
       250, 80;
       280, 0];             % 飞行走廊中各个矩形区域的中心点
x_length = 100;
y_length = 100;

n_order = 7;                % 8个控制点，对应于七阶多项式，也就是minisnap
n_seg = size(path, 1);      % 分段数，对应于飞行走廊中区域的个数

corridor = zeros(4, n_seg); % 初始化各个飞行走廊区域的坐标和相应的大小
for i = 1:n_seg
    corridor(:, i) = [path(i, 1), path(i, 2), x_length/2, y_length/2]';
end

%% 定义每一段的时间，这里简单的将每段的时间都设置为1
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 0.1;
end
% 获取x和y方向的多项式系数
poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor, ts, n_seg, n_order, v_max, a_max);
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor, ts, n_seg, n_order, v_max, a_max);

%% 显示规划出的轨迹和相应的飞行走廊
plot(path(:,1), path(:,2), '*r'); hold on;
for i = 1:n_seg
    plot_rect([corridor(1,i);corridor(2,i)], corridor(3, i), corridor(4,i));hold on;
end
hold on;
x_pos = [];y_pos = [];
idx = 1;

%% #####################################################
% STEP 4: 绘制相应的贝塞尔曲线
for k = 1:n_seg
    for t = 0:0.01:1
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);   % 此处的nchoosek对应于C_n_i
             x_pos(idx) = x_pos(idx) + poly_coef_x((k-1)*(n_order + 1) + i + 1) * basis_p ;
             y_pos(idx) = y_pos(idx) + poly_coef_y((k-1)*(n_order + 1) + i + 1) * basis_p ;
        end
        idx = idx + 1;
    end
end
scatter(poly_coef_x(1:8),poly_coef_y(1:8),100,"r");
scatter(poly_coef_x(9:16),poly_coef_y(9:16),100,"g");
scatter(poly_coef_x(17:24),poly_coef_y(17:24),100,"b");
scatter(poly_coef_x(25:32),poly_coef_y(25:32),100,"c");
scatter(poly_coef_x(33:40),poly_coef_y(33:40),100,"m");
plot(x_pos',y_pos',"Linewidth",2.0,"color","k");

% 求解贝塞尔多项式系数
function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order, v_max, a_max)
    % 起点和终点条件
    start_cond = [waypoints(1), 0, 0];
    end_cond   = [waypoints(end), 0, 0];   
    
    %% #####################################################
    % STEP 1: 计算对应于普通多项式的Q矩阵和对应于贝塞尔曲线的Q_0矩阵
    [Q, M]  = getQM(n_seg, n_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: 获取相应的等式约束
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: 获取飞行走廊中各个区域的范围以及相应的不等式约束
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as [p1_min, p1_max;
    %                                   p2_min, p2_max;
    %                                   ...,
    %                                   pn_min, pn_max ];
    corridor_range = [];
    for k = 1:n_seg
        corridor_range(k,:) = [corridor(axis,k) - corridor(axis+2,k), corridor(axis,k) + corridor(axis+2,k)];
    end
    % STEP 3.2: 获取相应的不等式约束
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end