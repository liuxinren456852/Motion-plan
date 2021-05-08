function global_best = pso_select(theta,omega,v_ini,p0,last_theta,last_v)

% input
% theta-当前车辆的朝向
% omega-当前车辆的角速度
% v_ini-当前车辆的线速度
% p0-当前车辆的位置
% last_theta-前次的角度
% last_v-前次的速度


% 二维旋转矩阵
R=[cos(theta) -sin(theta);
   sin(theta)  cos(theta)];

% 初始化粒子
N=10;       % 10个粒子
batch=10;   % 迭代10次


% theta,v_end,v_theta,v_vend
% best_theta,best_v,best_cost

P=zeros(N,7);

% best_theta,best_v,best_cost
global_best = zeros(1,3);
global_best(1) = last_theta-theta;
global_best(2) = last_v;
global_best(3) = evaluate(R,omega,p0,last_theta-theta,last_theta-theta,v_ini,last_v);

% 初始化粒子
for i=1:N
    P(i,1)=(rand-0.5)*1.8*pi;
    P(i,2)=(rand-0.5)*4+2;
    P(i,3)=rand-0.5;
    P(i,4)=rand-0.5;
    P(i,5)=P(i,1);
    P(i,6)=P(i,2);
    P(i,7)=inf;
end

% 开始迭代
for j=1:batch
    % 更新每一个粒子
    for i=1:N
        w = 0.95-(0.95-0.4)/batch*j;
        if (j~=1)
            %update the particle position for the i'th particle
            %--------------------------------------------------------------
            % To be finished by the student:
            %--------------------------------------------------------------
            P(i,3) = w* P(i,3)+  0.2*(rand) * (P(i,5) - P(i,1)) + 0.2*(rand) * (global_best(1) - P(i,1));
            P(i,1) = P(i,1) + P(i,3);
            P(i,4) = w * P(i,4)+ 0.2*(rand) * (P(i,6) - P(i,2)) + 0.2*(rand) * (global_best(2) - P(i,2));
            P(i,2) = P(i,2) + P(i,4);
        end
        %evaluate the particles
        cost = evaluate(R,omega,p0,P(i,1),last_theta-theta,v_ini,P(i,2));
        
        %update the local best
        if cost < P(i,7)
            P(i,7) = cost;
            P(i,5)=P(i,1);
            P(i,6)=P(i,2);
        end
        
        %update the global best
        if cost<global_best(3)
            global_best(3)=cost;
            global_best(1)=P(i,1);
            global_best(2)=P(i,2);
        end
    end
end
global_best(1)=global_best(1)+theta;
end