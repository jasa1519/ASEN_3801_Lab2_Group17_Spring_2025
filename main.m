


clc;
clear;
close all;

% FUNCTION RotationMatrix321 - Intakes [alpha, beta, gamma] and outputs rotation DCM for Inertial->Body
% FUNCTION RotationMatrix313 - Intakes [alpha, gamma, alpha] and outputs rotation DCM for Inertial->Body

% FUNCTION EulerAngles321 - Intakes 3x3 [DCM] and outputs 3 Euler Angles
% for 321 orientation
% FUNCTION EulerAngles313 - Intakes 3x3 [DCM] and outputs 3 Euler Angles
% for 313 orientation

%% Loading and extracting data

[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData('ASEN_3801_SP25_section001-bakCleanData.csv');
target.position_E = tar_pos_inert;
target.attitude_E = tar_att;
vehicle.position_E = av_pos_inert;
vehicle.attitude_E = av_att;


%% Task 3 Plotting 3d motion in frame N

DCM_E_N = [1,0,0;  %% DCM From Earth Frame to N frame. Extracted from lab doc
           0,-1,0;
           0,0,-1;];
target.position_N = zeros(3,length(t_vec));
vehicle.position_N = zeros(3,length(t_vec));

for i = 1:length(t_vec)
target.position_N(:,i) = DCM_E_N * target.position_E(:,i); 
vehicle.position_N(:,i) = DCM_E_N * vehicle.position_E(:,i);
end

figure()
hold on
plot3(target.position_N(1,:),target.position_N(2,:),target.position_N(3,:))
plot3(vehicle.position_N(1,:),vehicle.position_N(2,:),vehicle.position_N(3,:))

hold off


%% Task 4


% Axis Positions (Frame E)
subplot_label_arr = ["x","y","z"];
figure()
for i = 1:3
subplot(2,2,i)
hold on
plot(t_vec,target.position_E(i,:),"Color",[0,0,1])
plot(t_vec,vehicle.position_E(i,:),"Color",[1,0,0])
title(subplot_label_arr(i))
hold off
end

% Euler 3-2-1 Rel to E
subplot_label_arr = ["\phi","\theta","\psi"];
figure()
for i = 1:3
subplot(2,2,i)
hold on
plot(t_vec,(180/pi).*target.attitude_E(i,:),"Color",[0,0,1])
plot(t_vec,(180/pi).*vehicle.attitude_E(i,:),"Color",[1,0,0])
ylim([-200,200])
title(subplot_label_arr(i))
hold off
end

%% Task 5

% Obtaining 3-1-3 angles from 3-2-1 angles

% initializing vectors
target.attitude_E313 = zeros(3,length(t_vec));
vehicle.attitude_E313 = zeros(3,length(t_vec));

% Loop takes attitude at each point, gets the DCM, and then extracts 313
% Euler angle.
for i = 1:length(t_vec)
vehicle_DCM = RotationMatrix321(vehicle.attitude_E(:,i));
vehicle.attitude_E313(:,i) = EulerAngles313(vehicle_DCM);
target_DCM = RotationMatrix321(target.attitude_E(:,i));
target.attitude_E313(:,i) = EulerAngles313(target_DCM);
end

% Plotting 313 angles.

% Euler 3-1-3 Rel to E
subplot_label_arr = ["\phi","\theta","\psi"];
figure()
for i = 1:3
subplot(2,2,i)
hold on
plot(t_vec,(180/pi).*target.attitude_E313(i,:),"Color",[0,0,1])
plot(t_vec,(180/pi).*vehicle.attitude_E313(i,:),"Color",[1,0,0])
ylim([-200,200])
title(subplot_label_arr(i))
hold off
end



