% ddtheta = FwdDyn(tau, theta, dtheta, Fext, TwistList, TfList, MassList, gravity)
% :	Solve forward dynamics problem using recursive Newton-Euler algorithm
% 1. tau : joint torque
% 2. theta, dtheta, : pos, vel of joint
% 3. Fext : Wrench applied to end effector, decribed in the end effector csys.
% 4. TwistList : List of twists of ith joint described in space csys, at initial config. 
% 5. tfList : Configuration of the csys fixed at center of mass, at
%             initial configuration, i+1 element is configuration of the end effector
% 6. MassList : List of 6x6 spatial mass inertia matrix of each link
% 7. gravity : 6x1 acceleration corresponding to gravity. 
%              [0; 0; 0; direction of gravity] 
% returns Joint acceleration

function ddtheta =  FwdDyn(tau, theta, dtheta, Fext, TwistList, TfList, MassList, gravity)
	% First calculate torque from Coriolois term. C(theta, dtheta)*dtheta
    C = CoriolisTorque(theta, dtheta,TwistList, TfList, MassList);

    % Second calculate torque from gravity. g(theta)
    G = GravityTorque(theta, TwistList, TfList, MassList, gravity);

    % Calculate Mass matrix M(theta)
    M = MassMatrix(theta, TwistList, TfList, MassList);

    % Calculate torque by external wrench
    Teef = TfList(:,:,end);
    Jb = JacobianBody(theta, TwistList, Teef);

    % Solve dynamic equation for ddtheta
    ddtheta = M\(tau + Jb'*Fext - C - G);
end


