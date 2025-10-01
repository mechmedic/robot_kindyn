function [R,p] = TransToRp( T )
% TransToRp : Extracts rotation matrix R and translation p
%			  from given rigid transformation matrix T
%   [R,p] = TransToRp(T) returns 'R', rotation matrix part of T and
%   'p', translation part of T
	
R = T(1:3,1:3);	
p = T(1:3,4);