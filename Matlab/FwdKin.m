function T =  FwdKin(JointAngles, Twists, M)
% FwdKin : Calculates forward kinematics of the robot.
%   T = FwdKin(jAng, Twists, M) : Calculate product of exponential 
%   forward kinematics of a n dof serial manipulator
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'M' : Configuration of the end effector csys. with respect to 
%           fixed space csys at the zero configuration of the robot.

nJoints = size(Twists,2);	T = eye(4);
for i=1:nJoints
	xi = Twists(:,i);		th = JointAngles(i);
	T = T*MatrixExp6(VecTose3(xi)*th);
end
T = T*M;

