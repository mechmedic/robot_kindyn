function g = SymbolicGravityTorque(jAng, TwistList, TfList, MassList, grav)
% SymbolicGravityTorque : Calculates gravity torque of the robot
%   g = SymbolicGravityTorque(jAng, Twists, TfList, MassList, grav)
%	  'jAng' : n x 1 vector of joint angles
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 
%	  'grav' : 3 x 1 vector of gravity acceleration

nJoint = length(jAng);
g = vpa(zeros(nJoint, 1));
PE = vpa(0);
for i=1:nJoint
    T0 = TfList(:,:,i);
    T = vpa(eye(4));
    G = MassList(:,:,i);
    mc = so3ToVec(G(1:3,4:6));
    for j=1:i
        T = T*MatrixExp6(VecTose3(TwistList(:,j))*jAng(j));
    end
    h = T*T0*[mc; G(4,4)];
    PE = PE - grav'*h(1:3);
end
%g = PE;
for i=1:nJoint
    g(i) = diff(PE,jAng(i));
end