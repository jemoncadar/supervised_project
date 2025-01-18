%% Subject: Modelado y Control de Sistemas Mecatrónicos y Robots. 2023-24.
%% Deliverable: Supervised Project

clearvars;
close all;

%% Robot models in Robotics Toolbox
% Compute parameters 
%% PUT YOUR ID NUMBER HERE
finalPose=ComputeParameters([7 7 2 0 1 2 5 2]);
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

%% 1 Calcular velocidades de las ruedas

% Valores de la trayectoria
x_dot = stateTask(:,13);
x = stateTask(:,1);
phi = stateTask(:,2);
phi_dot = stateTask(:,14);

% DUDA: 
y_dot = zeros(size(x_dot));

num_points = length(x_dot);
wheel_velocities = zeros(2, num_points);

for i = 1:num_points
    phi_i = phi(i);
    
    % Calcular la Jacobiana de la base para el instante actual
    J_base = J_base_fcn(phi_i);
    
    J_base_inv = pinv(J_base);
    
    v_generalized = [x_dot(i); y_dot(i); phi_dot(i)];
    
    wheel_velocities(:, i) = J_base_inv * v_generalized;
end

vr_left = wheel_velocities(1, :);
vr_right = wheel_velocities(2, :);

% Mostrar o analizar las velocidades
disp('Velocidades de las ruedas calculadas (izquierda y derecha):');
disp(wheel_velocities');

%% 2. Aplicar la Jacobiana del manipulador móvil para obtener vel. del efector final

% Por cada punto

num_points = length(x_dot);
end_effector_velocities = zeros(6, num_points);

LB = 0;
r = 0.165;
a = 0.560/2;

for i = 1:num_points
    
    q_i = stateTask(i,7:12);
    phi_i = phi(i);
    vr_left_i = vr_left(i);
    vr_right_i = vr_right(i);

    q = [q_i'; vr_left_i; vr_right_i];
    
    J_base2 = [
        (r/2)*cos(phi_i) + (r*LB/(2*a))*sin(phi_i), (r/2)*cos(phi_i) - (r*LB/(2*a))*sin(phi_i);
        (r/2)*sin(phi_i) - (r*LB/(2*a))*cos(phi_i), (r/2)*sin(phi_i) + (r*LB/(2*a))*cos(phi_i);
        zeros(1, 2);
        zeros(1, 2);
        zeros(1, 2);
        -r/(2*a), r/(2*a)
    ];
    
    J_mani = J_mani_fcn(q_i);
    
    J_global = [R_phi_fcn(phi_i) * J_mani  J_base2];
    
    end_effector_velocities(:, i) = J_global * q;
end

%% 3. Integración para obtener posición y orientación del efector final

% Inicialización de posición y orientación del efector final
end_effector_pose = zeros(6, num_points); % [x; y; z; roll; pitch; yaw]
end_effector_pose(:, 1) = [taskInit(1:3,4); 0; 0; 0]; % Pose inicial (puedes ajustarla según el caso)

% Integración para calcular posición y orientación
for i = 2:num_points
    % Velocidad lineal y angular en el instante anterior
    linear_velocity = end_effector_velocities(1:3, i-1); % [vx; vy; vz]
    angular_velocity = end_effector_velocities(4:6, i-1); % [wx; wy; wz]
    
    % Integrar posición (x, y, z) usando velocidades lineales
    end_effector_pose(1:3, i) = end_effector_pose(1:3, i-1) + linear_velocity * timeStep;
    
    % Integrar orientación (roll, pitch, yaw) usando velocidades angulares
    end_effector_pose(4:6, i) = end_effector_pose(4:6, i-1) + angular_velocity * timeStep;
end

% Mostrar o analizar los resultados
disp('Posición y orientación del efector final en cada instante de tiempo:');
disp(end_effector_pose);



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
