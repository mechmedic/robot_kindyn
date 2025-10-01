function dJs =  dJacobianSpace(jAng,jVel,Twists)
% dJacobianSpace : Calculate time derivative of Spatial geometric Jacobian.
%   dJs = dJacobianSpace(jAng,jVel,Twists), calculate time derivative
%   of spatial geometric Jacobian of a serial n dof serial manipulator
%	  'jAng' : n x 1 vector of joint angles
%	  'jVel' : n x 1 vector of joint velocities
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.

nJoints = length(jAng);
dJs = zeros(6,nJoints);

Js = JacobianSpace(jAng,Twists);
for i=1:nJoints
    for j=1:i-1
        dJs(:,i) = dJs(:,i) + ad(Js(:,j))*Js(:,i)*jVel(j);
    end
end
