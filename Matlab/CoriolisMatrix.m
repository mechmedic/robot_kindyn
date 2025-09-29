function C = CoriolisMatrix(jAng, jVel, Twists, TfList, MassList)
% CoriolisMatrix : Calculates Coriolis Matrix Numerically
%   C = CoriolisMatrix(theta, dtheta, TwistList, TfList, MassList),
%	  'jAng' : n x 1 vector of joint angles
%	  'jVel' : n x 1 vector of joint velocities
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'TfList' : 4 by 4 by n array of coordinate system fixed to each link. 
%     'MassList' : 6 by 6 by n array of spatial inertia of each link 

nJoints = length(jAng);
C = zeros(6,nJoints);

% Calculate Body Jacobian of each links
Jset = zeros(6,nJoints,nJoints);
for i=1:nJoints
    Jset(:,1:i,i) = JacobianBody(jAng(1:i),Twists(:,1:i),TfList(:,:,i));
end
%Jset

for i=1:nJoints
    for j=1:nJoints
        for idx = max(i,j):nJoints
            G = MassList(:,:,idx);
            Jj = Jset(:,j,idx);
            Ji = Jset(:,i,idx);
            M1 = Jj'*(G*ad(Ji)+ad(Ji)'*G);
            M2 = Ji'*G*ad(Jj);
            for k=1:idx
                Jk = Jset(:,k,idx);
                C(i,j) = C(i,j) + M1*Jk*jVel(k);
                if(k>j)
                    C(i,j) = C(i,j) + M2*Jk*jVel(k);
                else
                    C(i,j) = C(i,j) - M2*Jk*jVel(k);
                end
            end
        end
    end
end

C = C*0.5;
