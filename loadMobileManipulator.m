function mobilemanipulator = loadMobileManipulator(manipulatorName, platformName)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  mobilemanipulator = loadMobileManipulator(manipulatorName, platformName)
%
%   function to build a mobile manipulator using the System Robotics Toolbox, attaching a 
%   manipulator to a moblie platform and defining 3 DoF for the platform kinematics
%
%   output: mobilemanipulator -> rigid body tree format with the mobile manipulator object
%   input:   manipulatorName  -> name of the manipulator model
%               platformName        -> name of the mobile platform model
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

manipulator = loadrobot(manipulatorName,'DataFormat','row','Gravity',[0 0 -9.81]);  
% Mobile platform model
platform = loadrobot(platformName, 'DataFormat','row');                          % 
platform.BaseName = 'base_link1';
top_plate = 'top_plate_link';
% Adding manipulator to the mobile platform
addSubtree(platform, top_plate, manipulator, ReplaceBase=false); 

% Creating a floating base as a subrobot in Rigid Body Tree format  with
% two degress-of-freedom (a prismatic joint in axis X and a revolute joint in axis Z)
mobilemanipulator = rigidBodyTree();
mobilemanipulator.BaseName = 'world1';                      % Base name for the new robot
mobilemanipulator.DataFormat = 'row';                          % data format in row for every link
jointAxisName = {'X', 'Y', 'Z'};                     % axis names
jointAxisValue = eye(3);                             % value for each axis
parentBodyName = mobilemanipulator.BaseName;       % Parent body for the new robot structure

% Creating the prismatic joint (for the platform motion in x and y coordinates)
N=2; % for differential drive platforms
% N=1 % other platforms
for i = 1:numel(jointAxisName)-N                                                                    
    bodyName = ['base' jointAxisName{i} 'TransBody'];   % Prismatic body names
    jointName = ['base' jointAxisName{i} 'TransJoint'];     % Prismatic joint names
    rb = rigidBody(bodyName);                                         % Adding the body to the subrobot
    rbJoint = rigidBodyJoint(jointName, 'prismatic');         % Create the joint
    rbJoint.JointAxis = jointAxisValue(i,:);                         % Value of the joint axis
    rbJoint.PositionLimits = [-inf inf];                                 % Defining the limits of the joint
    rb.Joint = rbJoint;                                                        % Adding the joint to the rigid body                                                   
    
    % Adding the rigid body to the robot
    mobilemanipulator.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % updating the parent body name
end

% Creating the revolute joint (for the platform orientation)
for i = 3:numel(jointAxisName)
    bodyName = ['base' jointAxisName{i} 'RevBody'];
    jointName = ['base' jointAxisName{i} 'RevJoint'];
    rb = rigidBody(bodyName);
    rbJoint = rigidBodyJoint(jointName, 'revolute');
    rbJoint.JointAxis = jointAxisValue(i,:);
    rbJoint.PositionLimits = [-inf inf];
    rb.Joint = rbJoint;
    
    % Adding to the parent (already updated with prismatic joints) 
    mobilemanipulator.addBody(rb, parentBodyName);
    parentBodyName = rb.Name; % updating the parent body name
end

% Adding the floating base (with prismatic and revolute joints) to the
% mobile manipulator
addSubtree(mobilemanipulator,'baseZRevBody', platform, ReplaceBase=false);


end