function M = MassMatrix(JointAngles, TwistList, TfList, MassList)
% MassMatrix : Calculates Mass Matrix of the robot's equation of motion
%   M = MassMatrix(jAng, Twists, TfList, MassList)
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 


% % Method 1. Use repeated call to inverse dynamics
% nJoint = length(JointAngles);
% % Call inverse dynamics with zero velocity, Fext and gravity,
% % acceleration with [0; 0; 1; 0; ....] pattern
% M = zeros(nJoint, nJoint);
% for i=1:nJoint
%     acc = zeros(nJoint,1);
%     acc(i) = 1;
%     M(:,i) = InvDyn(JointAngles, zeros(size(JointAngles)), acc, zeros(6,1), TwistList, TfList, MassList, zeros(6,1));
% end

% Method 2. Use definition sum(J'*G*J)
nJoint = length(JointAngles);

M = zeros(nJoint, nJoint);
J = zeros(6,nJoint);

for i=1:nJoint
    T = TfList(:,:,i);
    Gb = MassList(:,:,i);
    J(:)= 0 ;    
    J(:,1:i) = JacobianBody(JointAngles(1:i),TwistList(:,1:i),T);
    M = M + J'*Gb*J;
end
