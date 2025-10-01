% [Tau, F] = InvDyn(theta, dtheta, ddtheta, Fext, TwistList, TfList, MassList, gravity)
% :	Solve inverse dynamics problem using recursive Newton-Euler algorithm
% 1. theta, dtheta, ddtheta : pos, vel, acc of joint
% 2. Fext : Wrench applied to end effector, decribed in the end effector csys.
% 3. TwistList : List of twists of ith joint described in space csys, at initial config. 
% 4. tfList : Configuration of the csys fixed at center of mass, at
%             initial configuration, i+1 element is configuration of the end effector
% 5. MassList : List of 6x6 spatial mass inertia matrix of each link
% 6. gravity : 6x1 acceleration corresponding to gravity. 
%              [0; 0; 0; direction of gravity] 
% returns Torque on joint and Wrench on each joint

function Tau = InvDyn(theta, dtheta, ddtheta, Fext, TwistList, TfList, MassList, gravity)
	

	% Transform twist og joint i to csys of each link i
	% 'xi_i in i' = Adjoint(M_i inverse) * 'xi_i in o'
	nJoint = length(theta);	% number of joint
	TwistInLinkCsys = zeros(6,nJoint);
	for i=1:nJoint
		xi = TwistList(:,i);
		Mi = TfList(:,:,i);
		TwistInLinkCsys(:,i) = Adjoint(TransInv(Mi))*xi;
	end
	
	% Forward Iteration. Calculate Spatial velocity and its derivative
	% for each link, in its csys.
	V = zeros(6,nJoint);
	Vdot = zeros(6,nJoint);
	for i=1:nJoint
		xi = TwistInLinkCsys(:,i);
		th = theta(i);		dth = dtheta(i);	ddth = ddtheta(i);
		
		if (i==1)
			V(:,i) = xi*dth;
			Vdot(:,i) = xi*ddth - gravity;
		else
			Mi = TfList(:,:,i);
			Miprev = TfList(:,:,i-1);
			T = MatrixExp6(-VecTose3(xi)*th)*TransInv(Mi)*Miprev;	% Tiprev_to_i 
			
			V(:,i) = xi*dth + Adjoint(T)*V(:,i-1);
			Vdot(:,i) = xi*ddth + Adjoint(T)*Vdot(:,i-1) + dth*ad(V(:,i))*xi;
		end
	end
    % 
	% V
	% Vdot
	
	% Backward Iteration. Starting from the end, apply NE equation for 
	% each link, find wrench on a joint and corresponding torque
	F = zeros(6,nJoint+1);
	F(:,nJoint+1) = -Fext;
	Tau = zeros(nJoint,1);
	for i=nJoint:-1:1
		Gi = MassList(:,:,i);
		Mi = TfList(:,:,i);				
        Minext = TfList(:,:,i+1);
		if (i==nJoint)
			T = TransInv(Minext)*Mi;
		else
			th = theta(i+1);	
			xi = TwistInLinkCsys(:,i+1);		
			T = MatrixExp6(-VecTose3(xi)*th)*TransInv(Minext)*Mi;	% Ti_to_inext
		end
		F(:,i) = Gi*Vdot(:,i) - ad(V(:,i))'*Gi*V(:,i) + Adjoint(T)'*F(:,i+1);
		Tau(i) = dot(F(:,i),TwistInLinkCsys(:,i));
	end

end

