%% Subject: Modelado y Control de Sistemas Mecatrónicos y Robots. 2023-24.
%% Deliverable: Supervised Project

clearvars;
close all;

%% Robot models in Robotics Toolbox
% Compute parameters 
%% PUT YOUR ID NUMBER HERE
finalPose=ComputeParameters([7 7 6 8 4 6 8 2]);
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

timeSteps = tTask(2:end)' - tTask(1:end)';

%% 1 Calcular velocidades de las ruedas

num_points = length(trajTimes);

wheel_velocities = zeros(2, num_points);

for i=1 : num_points
    tNow = trajTimes(i);
    configNow = interp1(tTask, stateTask, tNow);

    phi = configNow(2);
    x_dot = configNow(13);
    y_dot = 0.0;
    phi_dot = configNow(14);
    
    % Calcular la Jacobiana de la base para el instante actual
    J_base = J_base_fcn(phi);
    
    J_base_inv = pinv(J_base);
    
    v_generalized = [x_dot; y_dot; phi_dot];
    
    wheel_velocities(:, i) = J_base_inv * v_generalized;
end

vr_left = wheel_velocities(1, :);
vr_right = wheel_velocities(2, :);

% Mostrar o analizar las velocidades
disp('Velocidades de las ruedas calculadas (izquierda y derecha):');
disp(wheel_velocities');

%% 2. Aplicar la Jacobiana del manipulador móvil para obtener vel. del efector final

% Por cada punto

end_effector_velocities = zeros(6, num_points);

LB = 0;
r = 0.165;
a = 0.560/2;

q_previo = zeros(1, 6);

for i=1 : num_points
    tNow = trajTimes(i);
    configNow = interp1(tTask, stateTask, tNow);
    
    q_i = configNow(7:12);
    q_dot_i = (q_i - q_previo) / timeStep;

    phi = configNow(2);
    vr_left_i = vr_left(i);
    vr_right_i = vr_right(i);

    q = [q_dot_i'; vr_left_i; vr_right_i];
    
    J_base2 = [
        (r/2)*cos(phi) + (r*LB/(2*a))*sin(phi), (r/2)*cos(phi) - (r*LB/(2*a))*sin(phi);
        (r/2)*sin(phi) - (r*LB/(2*a))*cos(phi), (r/2)*sin(phi) + (r*LB/(2*a))*cos(phi);
        zeros(1, 2);
        zeros(1, 2);
        zeros(1, 2);
        -r/(2*a), r/(2*a)
    ];
    
    J_mani = J_mani_fcn(q_i);
    
    J_global = [R_phi_fcn(phi) * J_mani  J_base2];
    
    end_effector_velocities(:, i) = J_global * q;

    q_previo = q_i;
end

%% 3. Integración para obtener posición y orientación del efector final

end_effector_pose = zeros(6, num_points); % [x; y; z; roll; pitch; yaw]

end_effector_pose(:, 1) = [taskInit(1:3,4); 0; 0; 0]; % Pose inicial (puedes ajustarla según el caso)

% Integración para calcular posición y orientación
for i=2 : num_points
    linear_velocity = end_effector_velocities(1:3, i-1); % [vx; vy; vz]
    angular_velocity = end_effector_velocities(4:6, i-1); % [wx; wy; wz]
    
    end_effector_pose(1:3, i) = end_effector_pose(1:3, i-1) + linear_velocity * timeStep;
    
    end_effector_pose(4:6, i) = end_effector_pose(4:6, i-1) + angular_velocity * timeStep;
end

disp('Posición y orientación del efector final en cada instante de tiempo:');
disp(end_effector_pose);

%% Integración con ODE 45

% Definir límites de integración
t_span = [trajTimes(1), trajTimes(end)];

% Pose inicial (x, y, z, roll, pitch, yaw)
pose_inicial = [taskInit(1:3,4); 0; 0; 0]; 

% Llamada a ode45
[t_sol, pose_sol] = ode45(@(t,y) endEffectorDynamics(t, y, trajTimes', end_effector_velocities), ...
                          t_span, ...
                          pose_inicial);

%% Representar poses del robot del profesor

end_effector_pose = end_effector_pose';

% Crear el gráfico
figure;
plot(end_effector_pose, '-o'); % Cada columna es una línea
xlabel('Index'); % Etiqueta del eje X
ylabel('Value'); % Etiqueta del eje Y
title('Graph of posesNow Columns');
legend('Column 1', 'Column 2', 'Column 3', 'Column 4', 'Column 5', 'Column 6'); % Leyenda
grid on; % Activar la cuadrícula

%% 
% Mobile robot representation in Cartesian space
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');   % Initial pose
title('Mobile manipulator trajectory')
hold on
axis([-1 3.5 -1 1.5 -0.2 1]);        % appropriate axis dimensions
for i=1:length(trajTimes)
    tNow= trajTimes(i);    % current time
    % Interpolation for the current pose (in time tNow)
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow); % mobile manipulator config in joint space
    % poseNow = getTransform(robot,configNow,endEffector);     % end-effector position in transform matrix

    show(robot, configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(end_effector_pose(1, i),end_effector_pose(2, i),end_effector_pose(3, i),'b.','MarkerSize',20);
    %if i <= 73
    %    taskSpaceMarker2 = plot3(pose_sol(i, 1),pose_sol(i, 2),pose_sol(i, 3),'r.','MarkerSize',20);
    %end
    drawnow;
end