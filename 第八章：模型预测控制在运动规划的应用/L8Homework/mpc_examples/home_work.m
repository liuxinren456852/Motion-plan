clear all;
close all;
clc;

% 初始条件
p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];

% 前向积分20步，每一步0.2秒
K=20;
dt=0.2;

P=[];
V=[];
A=[];


for t=0.2:0.2:40
    %% Construct the reference signal
    % 理想的参考信号
    for i = 1:20
        tref = t + i*0.2;
        r=0.25*tref;
        pt(i,1) = r*sin(0.2*tref);
        vt(i,1) = r*cos(0.2*tref);
        at(i,1) = -r*sin(0.2*tref);
        
        pt(i,2) = r*cos(0.2*tref);
        vt(i,2) = -r*sin(0.2*tref);
        at(i,2) = -r*cos(0.2*tref);
        
        pt(i,3) = 20 - 0.5*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end
    %% Do the MPC
    %% Please follow the example in linear mpc part to fill in the code here to do the tracking
    j(1) = xy_axis_mpc(K,dt,p_0(1),v_0(1),a_0(1),pt(:,1),vt(:,1),at(:,1));
    
    j(2) = xy_axis_mpc(K,dt,p_0(2),v_0(2),a_0(2),pt(:,2),vt(:,2),at(:,2));
    
    j(3) = z_axis_mpc(K,dt,p_0(3),v_0(3),a_0(3),pt(:,3),vt(:,3),at(:,3));

    for i=1:3
       [p_0(i),v_0(i),a_0(i)] = forward(p_0(i),v_0(i),a_0(i),j(i),dt);
    end
    
    %% Log the states
    P = [P;p_0 pt(1,:)];
    V = [V;v_0 vt(1,:)];
    A = [A;a_0 at(1,:)];
end

%% Plot the result
plot(P);
grid on;
legend('x','y','z');
figure;
plot3(P(:,1),P(:,2),P(:,3));
axis equal;
grid on;