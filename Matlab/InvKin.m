% [JointAngles, success] = InvKin(Td, M, Twists, InitJang)
% :	Solve Inverse Kinematics to find JointAngles that brings the robot to
%	desired end-effector configuration Td. Initial configuration M, 
%	Twist parameter of the robot, and initial guess for solution 
%	needs to be supplied

function [JointAngles, success] = InvKin(Td, M, Twists, InitJang)
	
	nIter = 1;	jAng = InitJang;
	
	while(1)
		
		if(nIter > 1000)
			error('Did not Converge');
			success = 0;	JointAngles = InitJang;
			break;
		end
		
		currT = FwdKin(jAng, Twists, M);
		Js = JacobianSpace(jAng,Twists);
		err = se3ToVec( MatrixLog6(Td*TransInv(currT)) );
		update = Js\err;
		jAng = jAng+update;

		if( (norm(err) < 0.1) )
			disp(sprintf('Converged in %d iterations',nIter));
			success = 1;	JointAngles = jAng;
			break;
		end
		
		nIter = nIter+1;
	end
	
end

