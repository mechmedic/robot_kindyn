function Jb =  JacobianBody(JointAngles, Twists, M)
% JacobianBody : Calculate Body geometric Jacobian
%   Jb = JacobianBody(jAng, Twists, M), calculate body geometric 
%   Jacobian of a serial n dof serial manipulator
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'M' : Configuration of the end effector csys. with respect to 
%           fixed space csys at the zero configuration of the robot.

nJoints = length(JointAngles);
if(isa(JointAngles,'sym'))
   Jb = sym(zeros(6,nJoints));
else
   Jb = zeros(6,nJoints);
end

T = TransInv(M);
Jb(:,nJoints) = Adjoint(T)*Twists(:,nJoints);
for i=(nJoints-1):-1:1
    xi = Twists(:,i+1);
    th = JointAngles(i+1);
    T = T*MatrixExp6(-VecTose3(xi)*th);
    Jb(:,i) = Adjoint(T)*Twists(:,i);
end

% TwistsInBody = Adjoint(TransInv(M))*Twists;
% Jb(:,nJoints) = TwistsInBody(:,nJoints);
% T = eye(4);
% for i=(nJoints-1):-1:1
%     xi = TwistsInBody(:,i+1);
%     th = JointAngles(i+1);
%     T = T*MatrixExp6(-VecTose3(xi)*th);
%     Jb(:,i) = Adjoint(T)*TwistsInBody(:,i);
% end
