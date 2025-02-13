


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

[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData('data.csv');
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

figure('Position', [300 300 900 600])
hold on, grid minor;
plot3(target.position_N(1,:),target.position_N(2,:),target.position_N(3,:),'LineWidth',2,'Color','#C70039','LineStyle','--')
plot3(vehicle.position_N(1,:),vehicle.position_N(2,:),vehicle.position_N(3,:),'LineWidth',2,'Color','#0515c4')
view(3);
legend("Target Position","Vehicle Position");
title("TASK 3 - Position Plotting Relative to N-Axis");
xlabel("X Position (mm)");
ylabel("Y Position (mm)");
zlabel("Z Position (mm)");

hold off


%% Task 4


% Axis Positions (Frame E)
subplot_label_arr = ["X","Y","Z"];
TASK4_1 = figure('Position', [400, 400, 800, 600]);
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,target.position_E(i,:),"Color",'#C70039','LineWidth',1.5)
plot(t_vec,vehicle.position_E(i,:),"Color",'#0515c4','LineWidth',1.5)
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Position (mm)")
title(subplot_label_arr(i) + " Coordinates")
legend("Target Position","Vehicle Position",'Location','northeastoutside');
sgtitle("TASK 4 - Position Plotting Relative to N-Axis")
hold off
end
legend("Target Position","Vehicle Position");
saveas(TASK4_1,'TASK4_1','png');


% Euler 3-2-1 Rel to E
subplot_label_arr = ["\phi","\theta","\psi"];
TASK4_2 = figure('Position', [400, 400, 800, 600]);
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,(180/pi).*target.attitude_E(i,:),"Color",'#C70039','LineWidth',1.5)
plot(t_vec,(180/pi).*vehicle.attitude_E(i,:),"Color",'#0515c4','LineWidth',1.5)
ylim([-200,200])
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Angle (deg)")
title(subplot_label_arr(i))
sgtitle("TASK 4 - Attitude Plotting Relative to N-Axis. (3-2-1 Euler Angles)")
legend("Target Attitude","Vehicle Attitude",'Location','northeastoutside');
hold off
end
saveas(TASK4_2,'TASK4_2','png');


%% Task 5

% Obtaining 3-1-3 angles from 3-2-1 angles

% initializing vectors
target.attitude_E313 = zeros(3,length(t_vec));
vehicle.attitude_E313 = zeros(3,length(t_vec));

% Loop takes attitude at each point, gets the DCM, and then extracts 313
% Euler angle.
for i = 1:length(t_vec)
vehicle_DCM = RotationMatrix321((180/pi).*vehicle.attitude_E(:,i));
vehicle.attitude_E313(:,i) = EulerAngles313(vehicle_DCM);
target_DCM = RotationMatrix321((180/pi).*target.attitude_E(:,i));
target.attitude_E313(:,i) = EulerAngles313(target_DCM);
end


% Plotting 313 angles.

% Euler 3-1-3 Rel to E
subplot_label_arr = ["\phi","\theta","\psi"];
TASK5 = figure('Position', [400, 400, 800, 600]);
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,(180/pi).*target.attitude_E313(i,:),"Color",'#C70039','LineWidth',1.5)
plot(t_vec,(180/pi).*vehicle.attitude_E313(i,:),"Color",'#0515c4','LineWidth',1.5)
ylim([-200,200])
title(subplot_label_arr(i))
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Angle (deg)")
sgtitle("TASK 5 - Attitude Plotting Relative to N-Axis. (3-1-3 Euler Angles)")
legend("Target Attitude","Vehicle Attitude",'Location','northeastoutside');
hold off
end
saveas(TASK5,'TASK5','png');

%% Task 6

% Plotting components of relative position vector.

relative_position_E = target.position_E - vehicle.position_E;

subplot_label_arr = ["X","Y","Z"];
TASK6 = figure('Position', [400, 400, 800, 600]);
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,relative_position_E(i,:),"Color",'#C70039','LineWidth',1.5)
title(subplot_label_arr(i))
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Position (mm)")
ylim([-5000,5000]);
sgtitle("TASK 6 - Position of Target Relative to Vehicle, Inertial Coordinates")
legend("Target Position (Relative)",'Location','northeast');
hold off
end
saveas(TASK6,'TASK6','png');

%% Task 7
% Plot relative position vector in vehicle coords.

relative_position_B = zeros(3,length(t_vec));
for i = 1:length(t_vec)
    vehicle_DCM = RotationMatrix321((180/pi).*vehicle.attitude_E(:,i));
    relative_position_B(:,i) = (vehicle_DCM*target.position_E(:,i)-vehicle_DCM*vehicle.position_E(:,i));
    if(0 == mod(i,10))
        teststop = 0;
    end
end
testDCM = RotationMatrix321([30,60,120]);
subplot_label_arr = ["X","Y","Z"];
TASK7 = figure('Position', [400, 400, 800, 600]);
sgtitle("Task 7 (Relative in Vehicle)")
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,relative_position_B(i,:),"Color",'#C70039','LineWidth',1.5)
title(subplot_label_arr(i))
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Position (mm)")
sgtitle("TASK 7 - Position of Target Relative to Vehicle, Body Coordinates")
ylim([-5000,5000]);
legend("Target Position (Relative)",'Location','northeast');
hold off
end
saveas(TASK7,'TASK7','png');

%% Task 8
vehicle_to_target_321 = zeros(3,length(t_vec));

for i = 1:length(t_vec)
    vehicle_DCM = RotationMatrix321(vehicle.attitude_E(:,i)); % Extracts earth to vehicle DCM
    target_DCM = RotationMatrix321(target.attitude_E(:,i)); % Extracts earth to target DCM
     % DCM Rotates from vehicle to earth, then from earth to Target.
     % Combined as one DCM:
    vehicle_to_target_DCM = target_DCM*(vehicle_DCM^-1);
    vehicle_to_target_321(:,i) = EulerAngles321(vehicle_to_target_DCM);
end

subplot_label_arr = ["\phi","\theta","\psi"];
TASK8 = figure('Position', [400, 400, 800, 600]);
for i = 1:3
subplot(3,1,i)
hold on, grid minor;
plot(t_vec,(180/pi).*vehicle_to_target_321(i,:),"Color",'#C70039','LineWidth',1.5)
ylim([-200,200])
title(subplot_label_arr(i))
xlabel("Time (s)")
ylabel(subplot_label_arr(i) + " Angle (deg)")
sgtitle("TASK 8 - 3-2-1 Euler Angles from Body Frame to Target Frame")
hold off
end
saveas(TASK8,'TASK8','png');
