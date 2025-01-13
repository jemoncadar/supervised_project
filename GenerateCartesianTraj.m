function [trajTimes, tTask, stateTask]=GenerateCartesianTraj(mobilemanipulator, jointInit, taskInit, taskFinal, timeStep, toolSpeed)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  [trajTimes, tTask, stateTask]=GenerateCartesianTraj(mobilemanipulator, jointInit, taskInit, taskFinal, timeStep, toolSpeed)
%
%   function to interpolate trajectory in Cartesian space using ode15s for the model 
%
%   output: trajTimes -> vector time for the complete trajectory
%               tTask       -> times for the interpolated trajectory
%               stateTask-> joint configuration for the interpolated trajectory 
%   input:   mobilemanipulator -> rigid body tree format of the mobile manipulator
%               jointInit                   -> initial mobile manipulator configuration
%               taskInit                   -> Initial pose in Cartesian coordinates
%               taskFinal                -> Final pose in Cartesian coordinates
%               timeStep                -> Desired time step for the complete trajectory
%               toolSpeed              -> tool speed during the trajectory
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Trajectory generation in Cartesian space
% Computing the Cartesian trajectory and intermediate points
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal)); % distance between initial and final pose
% Computing trajectory times using the tool speed 
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

% Motion control in Cartesian space
% Create a motion model in Cartesian space using a PD controller
% The taskSpaceMotionModel object models the Rigid Body Tree motion 
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',mobilemanipulator,'EndEffectorName','tool0');
% Proportional and derivative gains (for 0 value the behavior is based on reference poses)
tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;
% Initial states (joints poses and velocities)
q0 = jointInit; 
qd0 = zeros(size(q0));
% ode15s is used to generate reference states for each instant of time 
% Robot states are generated in stateTask
[tTask,stateTask] = ode15s(@(t,state) TaskInputs4ode(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);

end