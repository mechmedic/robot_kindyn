function AdT = Adjoint( T )
% Adjoint : Compute 6 x 6 Adjoint transformation matrix from 
%	       rigid transformation matrix T
%   AdT = Adjoint(T) calculates Adjoint transformation matrix 'AdT'
%   from rigid transformation T

[R,p] = TransToRp(T);
AdT = [ R, zeros(3); VecToso3(p)*R, R ];
