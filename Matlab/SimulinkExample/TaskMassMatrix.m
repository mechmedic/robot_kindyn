function Mt = TaskMassMatrix(jAng, Twists, TfList, MassList, BodyFrame)
% MassMatrix : Calculates Mass Matrix of the robot's equation of motion
%   M = MassMatrix(jAng, Twists, TfList, MassList, InSpaceOrBody)
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 
%     'BodyFrame' : 4 x 4 transfromation matrix representing body frame 
%                   configuration. If this is given, Mass matrix is 
%                   in this body frame

switch nargin
    case 4
        disp("Space Jacobian");
        J = JacobianSpace(jAng,Twists);
    case 5
        disp("Body Jacobian");
        J = JacobianBody(jAng,Twists,BodyFrame);
end

M = MassMatrix(jAng, Twists, TfList, MassList);

% inv(J')*M*inv(J)
Mt = J'\M/J; 