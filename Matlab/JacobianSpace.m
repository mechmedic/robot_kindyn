function Js =  JacobianSpace(JointAngles,Twists)
% JacobianSpace : Calculate Spatial geometric Jacobian.
%   Js = JacobianSpace(jAng,Twists), calculate spatial geometric 
%   Jacobian of a serial n dof serial manipulator
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.

nJoints = length(JointAngles);
if(isa(JointAngles,'sym'))
   Js = sym(zeros(6,nJoints));
else
   Js = zeros(6,nJoints);
end

Js(:,1) = Twists(:,1);
T = eye(4);
for i=2:1:nJoints
    xi = Twists(:,i-1);
    th = JointAngles(i-1);
    T = T*MatrixExp6(VecTose3(xi)*th);
    Js(:,i) = Adjoint(T)*Twists(:,i);
end

