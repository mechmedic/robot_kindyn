function T = RpToTrans( R,p )
% RpToTrans : Builds rigid transformation matrix from given rotation
%			  matrix and translation
%   T = RpToTrans[R,p] returns transformation matrix T = [R,p; 0 0 0 1]
%   consisting of rotation matrix 'R', translation 'p'
	
T = [R,p; 0,0,0,1];

