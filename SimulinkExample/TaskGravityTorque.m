function gt = TaskGravityTorque(jAng, Twists, TfList, MassList, gravity, BodyFrame)
% TaskGravityTorque : Calculate gravity torque in task space
%   gt = TaskGravityTorque(jAng, Twists, TfList, MassList, gravity, BodyFrame)
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 
%     'gravity' : 6 x 1 wrench representing gravitational acceleration
%     'BodyFrame' : 4 x 4 transfromation matrix representing body frame 
%                   configuration. If this is given, gravity torque is 
%                   in this body frame

switch nargin
    case 5
        disp("Space Frame");
        J = JacobianSpace(jAng,Twists);
    case 6
        disp("Body Jacobian");
        J = JacobianBody(jAng,Twists,BodyFrame);
end

    % Call inverse dynamics with zero velocity, acceleration, Fext
    g = InvDyn(jAng, zeros(size(jAng)), zeros(size(jAng)), zeros(6,1), Twists, TfList, MassList, gravity);
    gt = J'\g;
end