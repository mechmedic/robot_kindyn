% g = GravityTorque(theta, TfList, TwistList, MassList, gravity)
% :	 Calculates Torque by gravity g(th)

function g = GravityTorque(theta, TwistList, TfList, MassList, gravity)
    % Method 1. Call inverse dynamics with zero velocity, acceleration, Fext
    % g = InvDyn(theta, zeros(size(theta)), zeros(size(theta)), zeros(6,1), TwistList, TfList, MassList, gravity);
    % disp(g)
    % Spatial Acceleration bu gravity in space frame
    
    % Method 2. Calculate wrench at each link and trnsform it to torque
    g_space = gravity;%[0,0,0,0,0,-9.8]';
    nJoints = length(theta);
    g_torque = zeros(nJoints,1);
    T = eye(4);
    Jb = zeros(6,nJoints);
    for i=1:nJoints
        xi = TwistList(:,i);
        th = theta(i);
        M = TfList(:,:,i);
        T = T*MatrixExp6(VecTose3(xi)*th);
        g_body = Adjoint(TransInv(T*M))*g_space;
        Gb = MassList(:,:,i);
        gravitywrench = Gb*g_body;
        Jb(:)= 0 ;    
        Jb(:,1:i) = JacobianBody(theta(1:i),TwistList(:,1:i),M);
        g_torque = g_torque - Jb'*gravitywrench;
    end

    % disp(g_torque)
    g = g_torque;
end


    %     // CKim - Convert it to body frame
    % SE3 T;          T.SetEye();
    % Inertia Gb;     Jacobian Jb;
    % se3 g_body;     dse3 gravityWrench;
    % for(int i=0; i<m_nJoints; i++)
    % {
    %     T*=Exp(m_TwistSet[i],jA[i]);
    %     g_body = InvAd(T,g_space);
    %     Gb = m_InertiaSet[i];
    %     gravityWrench = Gb*g_body;
    %     Jb = JacobianBody(jA,i);
    %     for(int j=0; j<m_nJoints; j++)
    %     {
    %         g[i] += (Jb[j]*gravityWrench);
    %     }
    % }
    % return g;