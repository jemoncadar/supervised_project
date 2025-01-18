function dpose = endEffectorDynamics(t, pose, timeSamples, velocitiesSamples)
    % dpose = endEffectorDynamics(t, pose, timeSamples, velocitiesSamples)
    %
    % t:        tiempo actual en el que ode45 evalúa la función
    % pose:     vector [x; y; z; roll; pitch; yaw] (6x1)
    % timeSamples:   vector con los instantes de tiempo originales (num_points x 1)
    % velocitiesSamples: matriz (6 x num_points) con velocidades en dichos instantes
    %
    % Salida:
    % dpose:    derivada de la pose en el tiempo t, es decir, la velocidad
    %           [vx; vy; vz; wx; wy; wz] interpolada en el tiempo t.
    
    % Interpolación de las 6 velocidades en el tiempo t
    % Como velocitiesSamples es 6 x num_points, para usar interp1, transponemos:
    currentVel = interp1(timeSamples, velocitiesSamples', t, 'linear', 'extrap')';
    % 'linear': interpolación lineal
    % 'extrap': extrapolación (para evitar error si t sale algo fuera de los límites)

    % La derivada de la pose es simplemente la velocidad en cada componente
    % (x_dot = vx, y_dot = vy, z_dot = vz, roll_dot = wx, pitch_dot = wy, yaw_dot = wz)
    dpose = currentVel; 
end
