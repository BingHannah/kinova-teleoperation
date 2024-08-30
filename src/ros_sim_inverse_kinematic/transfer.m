%% inverse kinematics

Origin = [ 0.0699963   1.55088     0.00884627]'
Elbow = [ 0.3120236   1.550911    0.008846  ]';
Wrist = [ 0.52798     1.550938    0.00884578]';
Pump = [-0.6672537   1.55087     0.0081847 ]';
R_Index = [ 0.6014855   1.549217    0.0262001 ]';
R_Middle = [-0.6672537   1.55087     0.0081847 ]';
R_Ring = [-0.6668625   1.549002   -0.00902369]';


Rtrans = [0 -1 0; 0 0 1;-1 0 0];
DH_table = [-pi/2, 0,  0.2348,   0;...
             pi/2, 0, -0.0118,   0;...
            -pi/2, 0,  0.4208,   0;...
             pi/2, 0, -0.0128,   0;...
            -pi/2, 0,  0.3143,   0;...
             pi/2, 0,  0,        0;...
             0,    0,  0.1673,   0;
             0,    0,  88e-3,    0];

Origin = Rtrans *Origin;
Elbow = Rtrans * Elbow;
Wrist = Rtrans * Wrist ;
Pump = Rtrans *  Pump ;
R_Index = Rtrans * R_Index;
R_Middle = Rtrans * R_Middle;
R_Ring = Rtrans *R_Ring ;

PumpN = cross(R_Ring-R_Middle,R_Index-R_Middle );

        

theta_vec = DH_table(:,4)';

T_all = getT_all(DH_table,theta_vec);


 
    j=1;
    theta_vec(1) = theta_func(Origin,Elbow, T_all(:,4*(j-1)+1:4*j));
    T_all = getT_all(DH_table,theta_vec);

    j=j+1;
    theta_vec(2) = theta_func(Origin,Elbow,T_all(:,4*(j-1)+1:4*j))+pi/2;  
    T_all = getT_all(DH_table,theta_vec);  

    j=j+1;
    theta_vec(3) = theta_func(Elbow,Wrist,T_all(:,4*(j-1)+1:4*j));
    T_all = getT_all(DH_table,theta_vec);

    j=j+1;
    theta_vec(4) = theta_func(Elbow,Wrist,T_all(:,4*(j-1)+1:4*j))+pi/2;
    T_all = getT_all(DH_table,theta_vec);

    j=j+1;
    theta_vec(5) = theta_func(Wrist,Pump,T_all(:,4*(j-1)+1:4*j));
    T_all = getT_all(DH_table,theta_vec);

    j=j+1;
    theta_vec(6) = theta_func(Wrist,Pump,T_all(:,4*(j-1)+1:4*j))+pi/2;
    T_all = getT_all(DH_table,theta_vec);

    j=j+1;
    theta_vec(7) = theta_func(Pump,PumpN,T_all(:,4*(j-1)+1:4*j));
    T_all = getT_all(DH_table,theta_vec);

    theta_real = theta_vec;
    theta_real(2) = theta_real(2) + pi/2;
    disp(theta_real);



function T_all = getT_all(DH_table, theta_vec)

DH_table(:,4) = theta_vec; 
DH_size = size(DH_table,1);
T_end = eye(4);
T_all = [T_end];

for i = 1:DH_size
    T = Homegeneous_matrix(DH_table(i,:));
    T_end = T_end*T;
    % disp(['T' num2str(i) '_' num2str(0) ':'])
    % disp(T_end)
    T_all = [T_all, T_end];
end

% for i=1:DH_size+1
%     T_all(:,4*(i-1)+1:4*i)
% end
end

function T = Homegeneous_matrix(DH_row)

theta = DH_row(4);
alpha = DH_row(1);
a = DH_row(2);
d = DH_row(3);

T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),   a*cos(theta);...
     sin(theta),  cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta);...
     0         ,  sin(alpha),            cos(alpha),              d           ;...
     0         ,  0         ,            0,                       1           ];

end

function [theta] = theta_func(P1,P2,T)
% Axis
O = T(1:3,4);
nx = T(1:3,1);
ny =T(1:3,2);
nz = T(1:3,3);

% P1、P2 projection
P1p = P1 - dot(P1 - O, nz) * nz;
P2p = P2 - dot(P2 - O, nz) * nz;

% P1P2 in X Y Component
Vecp = P2p - P1p;
x = dot(Vecp,nx);
y = dot(Vecp,ny);

theta = atan2(y,x);
end