function V = InertiaToVec(G)
% InertiaToVec : Compute 6 x 6 spatial inertia from the 10 element
% dynamic parameter vector. [m mcx, mcy, mcz,Ixx, Ixy, Ixz, Iyy, Iyz, Izz];

V = [ G(4,4); so3ToVec(G(1:3,4:6)); G(1,1); G(1,2); G(1,3); G(2,2); G(2,3); G(3,3)];
