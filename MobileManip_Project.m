%% Subject: Modelado y Control de Sistemas Mecatrónicos y Robots. 2023-24.
%% Deliverable: Supervised Project

clearvars;
close all;

%% Robot models in Robotics Toolbox
% Compute parameters 
%% PUT YOUR ID NUMBER HERE
finalPose=ComputeParameters([X X X X X X X X]);
%%
% Manipulator robot model
% abbIrb120T, abbIrb1600, fanucLRMate200ib, fanucM16ib, universalUR3, universalUR3e, universalUR5, universalUR5e, universalUR10, universalUR10e, universalUR16e
manipulator='abbIrb120';
platform='clearpathHusky';

robot=loadMobileManipulator(manipulator, platform);

% Trajectory settings
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s
% Defining the initial and final pose of the end-effector
currentRobotJConfig = [0 0 0 0 0 0 0 0 0 0 0 0]; % manipulator initial configuration
% Number of joints and Rigid Body Tree frame for the end-effector
numJoints = numel(currentRobotJConfig);
endEffector = 'tool0';
jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);
taskFinal = trvec2tform(finalPose)*axang2tform([0 0 1 pi/2]);  % final pose

[trajTimes, tTask, stateTask]=GenerateCartesianTraj(robot, jointInit, taskInit, taskFinal, timeStep, toolSpeed);



% Mobile robot representation in Cartesian space
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');   % Initial pose
title('Mobile manipulator trajectory')
hold on
axis([-1 3.5 -1 1.5 -0.2 1]);        % appropriate axis dimensions
posesNow = zeros(length(trajTimes), 6);
for i=1:length(trajTimes)
    tNow= trajTimes(i);    % current time
    % Interpolation for the current pose (in time tNow)
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow); % mobile manipulator config in joint space
    poseNow = getTransform(robot,configNow,endEffector);     % end-effector position in transform matrix

    % Transform poseNow frm matrix to pose
    position = poseNow(1:3, 4);
    R = poseNow(1:3, 1:3);
    yaw = atan2(R(2,1), R(1,1));
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    roll = atan2(R(3,2), R(3,3));
    orientation = [roll; pitch; yaw];
    posesNow(i,:) = [position; orientation];


    show(robot, configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end

%% Representar poses del robot del profesor

% Crear el gráfico
figure;
plot(posesNow, '-o'); % Cada columna es una línea
xlabel('Index'); % Etiqueta del eje X
ylabel('Value'); % Etiqueta del eje Y
title('Graph of posesNow Columns');
legend('Column 1', 'Column 2', 'Column 3', 'Column 4', 'Column 5', 'Column 6'); % Leyenda
grid on; % Activar la cuadrícula


