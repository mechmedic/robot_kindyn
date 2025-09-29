function invT = TransInv(T)
% TransInv : Inverts rigid trnasformation matrix T
%   invT = TransInv(T). 'invT' is a inverse of transformation matrix 'T'
	
[R,p] = TransToRp(T);
invT = [ R', -R'*p; 0,0,0,1 ];
