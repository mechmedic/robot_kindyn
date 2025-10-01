function W = VecToInertia(G)
% VecToInertia : Compute 6 x 6 spatial inertia from the 10 element
% dynamic parameter vectort

m = G(1);
mc = G(2:4);
I = [G(5), G(6), G(7);
     G(6), G(8), G(9); 
     G(7), G(9), G(10)];
W = [I, VecToso3(mc); -VecToso3(mc), m*eye(3)];