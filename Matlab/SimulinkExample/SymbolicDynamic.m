function [DynEq, M, C, g] = SymbolicDynamic(q, dq, ddq, Twists, Tfs, Inertias, Gravity)
% SymbolicDynamic : Calculates Symbolic 1. equation of motion 'DynEq', 
%                   2. Mass matrix 'M', 3. Coriolis matrix 'C', and 
%                   4. gravity torque 'g' of n dof robot.
%   [DynEq, M, C, g] = SymbolicDynamic(q, dq, ddq, TwistList, TfList, MassList, Gravity)
%	  'q,dq,ddq' : n x 1 vector of symbols for joint pos, vel, acc. 
%     'Twists' : 6 x n matrix whose columns are Twist parameter of
%                each joints, described in fixed space csys. 
%                at the zero configuration of the robot.
%     'Tfs' : 4 x 4 x (n+1) array of initial configuration of csys.
%             attached to each link at the zero configuration of the robot.
%             Last 4 x 4 matrix is the initial configuration of the 
%             end effector csys.           
%     'Inertias' : 6 x 6 x n array of spatial inertia of each link, 
%                  described in body fixed csys. of each link i.
%     'Gravity' : 3 x 1 vector for direction of gravity

nJoint = length(q);

% Pre calculate e^(-xi*th) and e^(xi*th)for each joints
TinvSet = sym(zeros(4,4,nJoint));
TSet = sym(zeros(4,4,nJoint));
for i=1:nJoint
    TinvSet(:,:,i) = simplify(MatrixExp6(-VecTose3(Twists(:,i))*q(i)));
    TSet(:,:,i) = simplify(MatrixExp6(VecTose3(Twists(:,i))*q(i)));
end

% Calculate Mass matrix M
M = sym(zeros(nJoint, nJoint));
Jb = sym(zeros(6,nJoint));
for i=1:1:nJoint
    J = sym(zeros(6,i));
    T = TransInv(Tfs(:,:,i));
    J(:,i) = Adjoint(T)*Twists(:,i);

    for j=(i-1):-1:1
        T = T*TinvSet(:,:,j+1);
        J(:,j) = Adjoint(T)*Twists(:,j);
    end

    Jb(:)= 0;    
    Jb(:,1:i) = J;
    Gb = Inertias(:,:,i);
    M = M + Jb'*Gb*Jb;
end

% Calculate Coriolis matrix C
C = sym(zeros(nJoint, nJoint));
for i=1:nJoint
    for j=1:nJoint
        Cij = 0;
        for k=1:nJoint
            Cij = Cij + 1/2*( diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)) )*dq(k);
        end
        C(i,j) = Cij;
    end
end

% Calculate Gravity Torque
g = sym(zeros(nJoint, 1));
PE = sym(0);
for i=1:nJoint
    T0 = Tfs(:,:,i);
    T = sym(eye(4));
    Gb = Inertias(:,:,i);
    mc = so3ToVec(Gb(1:3,4:6));
    for j=1:i
        T = T*TSet(:,:,j);
    end
    h = T*T0*[mc; Gb(4,4)];
    PE = PE - Gravity'*h(1:3);
end

for i=1:nJoint
    g(i) = diff(PE,q(i));
end

% Finally Dynamics Equation
DynEq = M*ddq + C*dq + g;
