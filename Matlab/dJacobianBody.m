function dJb =  dJacobianBody(jAng, jVel, Twists, M)
% dJacobianBody : Calculate time derivative of Body geometric Jacobian
%   Jb = JacobianBody(jAng, Twists, M), calculate time derivative
%   of body geometric Jacobian of a serial n dof serial manipulator
%	  'jAng' : n x 1 vector of joint angles
%	  'jVel' : n x 1 vector of joint velocities
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'M' : Configuration of the end effector csys. with respect to 
%           fixed space csys at the zero configuration of the robot.

nJoints = length(jAng);
dJb = zeros(6,nJoints);

Jb = JacobianBody(jAng,Twists,M);
for i=1:nJoints
    for j=i+1:nJoints
        dJb(:,i) = dJb(:,i) + ad(Jb(:,i))*Jb(:,j)*jVel(j);
    end
end


