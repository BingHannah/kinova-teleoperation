%% Reading a BVH file and animating it over time
%
% BVH is a text file which contains skeletal data, but its contents needs
% additional processing to draw the wireframe and create the animation.

clear all
close all

 name = 'pick_shake_drop_pick_place_1';
% name = 'pick_place_1';
% name = 'hand_wave';

[skeleton,time] = loadbvh(name);
% Njoints = numel(skeleton);
nn_selected = [18 19 20 30]; 
% Njoints

frame_step  = 10;
write_video = false;

%% display skeleton info
disp(['The number of frames is ' num2str(skeleton(1).Nframes)])
disp('The joints in this skeleton are:')
for i = 1:length(skeleton)
    if skeleton(i).name~=' '
        disp(['The ' num2str(i) 'th joint is:'  skeleton(i).name]);
    end
end

disp('The XYZ coordinates are stored in skeleton(i).Dxyz')

%% inverse kinematics

Rtrans = [0 -1 0; 0 0 1;-1 0 0];
JD = skeleton(18);
Origin = Rtrans * skeleton(18).Dxyz;
JD1 = skeleton(19);
Elbow = Rtrans * skeleton(19).Dxyz;
JD2 = skeleton(20);
Wrist = Rtrans * skeleton(20).Dxyz;
JD3 = skeleton(30);
Pump = Rtrans * skeleton(30).Dxyz;

R_Index = Rtrans *skeleton(25).Dxyz;
R_Middle = Rtrans *skeleton(30).Dxyz;
R_Ring = Rtrans *skeleton(35).Dxyz;
PumpN = cross(R_Ring-R_Middle,R_Index-R_Middle );

DH_table = [-pi/2, 0,  0.2348,   0;...
             pi/2, 0, -0.0118,   0;...
            -pi/2, 0,  0.4208,   0;...
             pi/2, 0, -0.0128,   0;...
            -pi/2, 0,  0.3143,   0;...
             pi/2, 0,  0,        0;...
             0,    0,  0.1673,   0;
             0,    0,  88e-3,    0];
         
for i=1:8
    DH_row = DH_table(i,:);
    alpha = DH_row(1);
    a = DH_row(2);
    d = DH_row(3);
    theta = DH_row(4);
    L(i)=Link([theta,d,a,alpha]);
end
robot = SerialLink(L);

% figure;
theta_vec = DH_table(:,4)';
% robot.plot(theta_vec)

T_all = getT_all(DH_table,theta_vec);

%% create figs for animation

folderPath = 'savefig';
files = dir(fullfile(folderPath, '*.png'));
for i = 1:length(files)
    delete(fullfile(folderPath, files(i).name));
end
disp('All figures in savefig folder have been deleted.');


%%

fig_index = 1;
for ff = 1:frame_step:skeleton(1).Nframes
    % f1=figure('Visible','on');
    f1=figure('Visible','off');
 
    ha=subplot(121);
    hold on
  
    % hf.Color = 'white';
    % ha = gca;
    
    % General view
    title(sprintf('%1.2f seconds (frame %i)',0,ff))

    view(-135,135)
    axis equal
    axis on
    grid on
    
    % Set axes to show all points across all time
    xmax = 0; xmin = min(skeleton(nn_selected(1)).Dxyz(1,:));
    ymax = 0; ymin = min(skeleton(nn_selected(1)).Dxyz(2,:));
    zmax = 0; zmin = min(skeleton(nn_selected(1)).Dxyz(3,:));
    for nn = nn_selected
      xmax = max(xmax,max(skeleton(nn).Dxyz(1,:)));
      ymax = max(ymax,max(skeleton(nn).Dxyz(2,:)));
      zmax = max(zmax,max(skeleton(nn).Dxyz(3,:)));
      xmin = min(xmin,min(skeleton(nn).Dxyz(1,:)));
      ymin = min(ymin,min(skeleton(nn).Dxyz(2,:)));
      zmin = min(zmin,min(skeleton(nn).Dxyz(3,:)));
    end
    scalefactor = 1.2;
    xmax = (xmax+xmin)/2 + scalefactor*(xmax-xmin)/2;
    ymax = (ymax+ymin)/2 + scalefactor*(ymax-ymin)/2;
    zmax = (zmax+zmin)/2 + scalefactor*(zmax-zmin)/2;
    xmin = (xmax+xmin)/2 - scalefactor*(xmax-xmin)/2;
    ymin = (ymax+ymin)/2 - scalefactor*(ymax-ymin)/2;
    zmin = (zmax+zmin)/2 - scalefactor*(zmax-zmin)/2;
    axis([xmin xmax ymin ymax zmin zmax])
    
    ha.XLimMode = 'manual';
    ha.YLimMode = 'manual';
    ha.ZLimMode = 'manual';

  for nn = nn_selected
    hp(nn) = plot3(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(2,ff),skeleton(nn).Dxyz(3,ff),'.','markersize',20);
    parent = skeleton(nn).parent;
    if parent > 0
      hl(nn) = plot3([skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)],...
                     [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)],...
                     [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)]);
    end
  end
%% inverse kinematics
    i=ff;
    theta_vec(1) = theta_func(Origin(:,i),Elbow(:,i),T_all(1));
    T_all = getT_all(DH_table,theta_vec);
    
    theta_vec(2) = theta_func(Origin(:,i),Elbow(:,i),T_all(2))+pi/2;  
    T_all = getT_all(DH_table,theta_vec);  
    
    theta_vec(3) = theta_func(Elbow(:,i),Wrist(:,i),T_all(3));
    T_all = getT_all(DH_table,theta_vec);
    
    theta_vec(4) = theta_func(Elbow(:,i),Wrist(:,i),T_all(4))+pi/2;
    T_all = getT_all(DH_table,theta_vec);
    
    theta_vec(5) = theta_func(Wrist(:,i),Pump(:,i),T_all(5));
    T_all = getT_all(DH_table,theta_vec);
    
    theta_vec(6) = theta_func(Wrist(:,i),Pump(:,i),T_all(6))+pi/2;
    T_all = getT_all(DH_table,theta_vec);
    
    theta_vec(7) = theta_func(Pump(:,i),PumpN(:,i),T_all(7));
    T_all = getT_all(DH_table,theta_vec);
       
    subplot(122)

    % robot.plotopt = {'nobase', 'noshadow', 'nowrist', 'nojaxes'};
    % robot.link{8}.plotopt = {'linkcolor', [0, 0, 1]};  % RGB for blue
    theta_real = theta_vec;
    theta_real(2) = theta_real(2) + pi/2;
    robot.plot(theta_real)

     % view(180,0)
     view(225,45)


    saveas(f1, fullfile('savefig', sprintf('figure_%03d.png', fig_index)), 'png');
    close(f1);
    fig_index = fig_index+1;
end

% Create a video from the saved figures
outputVideo = VideoWriter('robot_video', 'MPEG-4');
outputVideo.FrameRate = 10;  % Change this as needed
open(outputVideo);
for idx = 1:fig_index-1
    img = imread(fullfile('savefig', sprintf('figure_%03d.png', idx)));
    writeVideo(outputVideo, img);
end
close(outputVideo);


                                                                                                                  