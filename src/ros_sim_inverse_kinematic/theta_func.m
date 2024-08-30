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


