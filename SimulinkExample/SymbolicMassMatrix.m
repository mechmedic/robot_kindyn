function M = SymbolicMassMatrix(JointAngles, TwistList, TfList, MassList)
% SymbolicMassMatrix : Calculates Symbolic Mass Matrix of the robot
%   M = MassMatrix(jAng, Twists, TfList, MassList)
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 

nJoint = length(JointAngles);

M = sym(zeros(nJoint, nJoint));
J = sym(zeros(6,nJoint));

for i=1:nJoint
    T = TfList(:,:,i);
    Gb = MassList(:,:,i);
    J(:)= 0 ;    
    J(:,1:i) = JacobianBody(JointAngles(1:i),TwistList(:,1:i),T);
    M = M + J'*Gb*J;
end
