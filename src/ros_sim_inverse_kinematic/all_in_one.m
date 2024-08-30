%% 参数设定
clear;clc;close all;

DH_table = [-pi/2, 0,  0.2348,   0;...
             pi/2, 0, -0.0118,   0  ;...
            -pi/2, 0,  0.4208,   0;...
             pi/2, 0, -0.0128,   0;...
            -pi/2, 0,  0.3143,   0;...
             pi/2, 0,  0,        0;...
             0,    0,  0.1673,   0;];
         
for i=1:7
    DH_row = DH_table(i,:);
    alpha = DH_row(1);
    a = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);
    L(i)=Link([theta,d,a,alpha]);
end
robot = SerialLink(L);

figure;
theta_vec = DH_table(:,4)';
robot.plot(theta_vec)

S = [-1 0 0]/100;
Origin = [0 0 0];
Elbow = [28 20 2]/100;
Wrist = [28 -20 26]/100;
Pump = [28+10 10 26]/100;
PumpN = [28+10 -10 21]/100;

HSO = Origin-S;
HOE = Elbow-Origin;
HEW = Wrist-Elbow;
HWP = Pump-Wrist;
HPN = PumpN-Pump;

hold on;
plot3([Origin(1),Elbow(1)],[Origin(2),Elbow(2)],[Origin(3),Elbow(3)], 'r-', 'LineWidth', 2);
plot3([Elbow(1),Wrist(1)],[Elbow(2),Wrist(2)],[Elbow(3),Wrist(3)], 'r-', 'LineWidth', 2);
plot3([Wrist(1),Pump(1)],[Wrist(2),Pump(2)],[Wrist(3),Pump(3)], 'r-', 'LineWidth', 2);

%% T_all

T_all = getT_all(DH_table,theta_vec);

%% inverse kinematic
theta_vec(1) = theta_func(Origin',Elbow',T_all(1));
T_all = getT_all(DH_table,theta_vec);

theta_vec(2) = theta_func(Origin',Elbow',T_all(2))+pi/2;  
T_all = getT_all(DH_table,theta_vec);  

theta_vec(3) = theta_func(Elbow',Wrist',T_all(3));
T_all = getT_all(DH_table,theta_vec);

theta_vec(4) = theta_func(Elbow',Wrist',T_all(4))+pi/2;
T_all = getT_all(DH_table,theta_vec);

theta_vec(5) = theta_func(Wrist',Pump',T_all(5));
T_all = getT_all(DH_table,theta_vec);

theta_vec(6) = theta_func(Wrist',Pump',T_all(6))+pi/2;
T_all = getT_all(DH_table,theta_vec);

theta_vec(7) = theta_func(Pump',PumpN',T_all(7));
T_all = getT_all(DH_table,theta_vec);

%% verify

DH_table = [-pi/2, 0,  0.2348,   0;...
             pi/2, 0, -0.0118,   0  ;...
            -pi/2, 0,  0.4208,   0;...
             pi/2, 0, -0.0128,   0;...
            -pi/2, 0,  0.3143,   0;...
             pi/2, 0,  0,        0;...
             0,    0,  0.1673,   0;];
         
for i=1:7
    DH_row = DH_table(i,:);
    alpha = DH_row(1);
    a = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);
    L(i)=Link([theta,d,a,alpha]);
end
robot = SerialLink(L);

figure;
% theta_vec = [0 pi/2 0 -pi/2 0 pi/2 0];
robot.plot(theta_vec)

hold on;
plot3([Origin(1),Elbow(1)],[Origin(2),Elbow(2)],[Origin(3),Elbow(3)], 'r-', 'LineWidth', 2);
plot3([Elbow(1),Wrist(1)],[Elbow(2),Wrist(2)],[Elbow(3),Wrist(3)], 'r-', 'LineWidth', 2);
plot3([Wrist(1),Pump(1)],[Wrist(2),Pump(2)],[Wrist(3),Pump(3)], 'r-', 'LineWidth', 2);