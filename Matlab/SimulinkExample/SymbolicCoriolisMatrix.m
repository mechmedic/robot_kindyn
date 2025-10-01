function C = SymbolicCoriolisMatrix(jAng, jVel, TwistList, TfList, MassList)
% SymbolicCoriolisMatrix : Calculates Coriolis Matrix of the robot
%   C = SymbolicCoriolisMatrix(jAng, jVel, Twists, TfList, MassList)
%	  'jAng' : n x 1 vector of joint angles
%	  'jVel' : n x 1 vector of joint velocities
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 x 4 x n array storing configuration of each link's 
%                body fixed coordinate system at zero configuration. 
%     'MassList' : 6 x 6 x n array storing spatial inertia of each 
%                  link in its body fixed coordinate system 

nJoint = length(jAng);
C = sym(zeros(nJoint, nJoint));
M = SymbolicMassMatrix(jAng, TwistList, TfList, MassList);
for i=1:nJoint
    for j=1:nJoint
        Cij = 0;
        for k=1:nJoint
            Cij = Cij + 0.5*( diff(M(i,j),jAng(k)) + diff(M(i,k),jAng(j)) - diff(M(j,k),jAng(i)) )*jVel(k);
        end
        C(i,j) = Cij;
    end
end