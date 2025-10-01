% C = CoriolisTorque(theta, dtheta, tfList, TwistList, MassList)
% :	 Calculates Torque by Coriolis Matrix C(th,dth)*dth

function C = CoriolisTorque(theta, dtheta, TwistList, TfList, MassList)
    % Call inverse dynamics with zero acceleration, Fext and gravity.
    C = InvDyn(theta, dtheta, zeros(size(theta)), zeros(6,1), TwistList, TfList, MassList, zeros(6,1));
end
