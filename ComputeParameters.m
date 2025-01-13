function finalPose = ComputeParameters(dni)
% -------------HELP ABOUT COMPUTEPARAMETERS-------------------
% This function generate random parameters based on ID number of 
% the student. 
% Input: ID number in a vector of size 8. E.g., for ID no. 999999999, 
% this function must be called:
%               f=ComputeParameters([9 9 9 9 9 9 9 9 9])
% Output: 
%               Pose vector for final pose of the mobile manipulator

if length(dni)<8
    fprintf(2, 'ERROR: the ID number is not correct, its length must be of size 8,\n '); 
    fprintf(2,'e.g., for ID no. 99999999, this function must be called: \n');
    fprintf(2,'f=ComputeParameters([9 9 9 9 9 9 9 9])');
else
    seedPose=[2,0.8,0.3];
    finalPose=zeros(size(seedPose));
    
    % number between 1 and 10
    n=1+sum(dni)/length(dni);

    % first component between 1 and 3
    finalPose(1)=seedPose(1)-1+0.2*n;

    % second component between 0.5 and 1
    finalPose(2)=seedPose(2)-0.3+0.05*n;

    % third component between 0.1 and 0.6
    finalPose(3)=seedPose(3)-0.2+0.05*n;
end