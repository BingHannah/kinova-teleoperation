function T_all = getT_all(DH_table,theta_vec)
DH_table(:,4) = theta_vec; 
DH_size = size(DH_table,1);

T0=eye(4);
T_all = [T0];
for i = 1: DH_size
    clear L robot
   for j = 1:i
    DH_row = DH_table(j,:);
    L(j)=Link([DH_row(4),DH_row(3),DH_row(2),DH_row(1)]);
   end
    robot = SerialLink(L);
    T = robot.fkine(theta_vec(1:i));
    T_all = [T_all T];
end
end