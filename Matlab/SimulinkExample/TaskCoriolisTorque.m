function Ctdth = TaskCoriolisTorque(jAng, jVel, Twists, TfList, MassList, BodyFrame)
% TaskCoriolisTorque : Calculates Torque by Coriolis Matrix C(th,dth)*dth,
%                      in task space
%   Ctdth = TaskGravityTorque(jAng, Twists, TfList, MassList, gravity, BodyFrame)
%	  'jAng' : n x 1 vector of joint angles
%	  'jVel' : n x 1 vector of joint velocity
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 
%     'BodyFrame' : 4 x 4 transfromation matrix representing body frame 
%                   configuration. If this is given, gravity torque is 
%                   in this body frame

switch nargin
    case 5
        disp("Space Frame");
        J = JacobianSpace(jAng,Twists);
        dJ = dJacobianSpace(jAng,jVel,Twists);
        M = TaskMassMatrix(jAng,Twists,TfList,MassList);
    case 6
        disp("Body Jacobian");
        J = JacobianBody(jAng,Twists,BodyFrame);
        dJ = dJacobianBody(jAng,jVel,Twists);
        M = TaskMassMatrix(jAng,Twists,TfList,MassList,BodyFrame);
end

Cdth = CoriolisTorque(jAng, jVel, Twists, TfList, MassList);
Ctdth = -M*dJ*jVel + J'\C;


