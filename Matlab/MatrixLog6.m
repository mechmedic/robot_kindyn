function se3mat = MatrixLog6( T )
% se3mat = MatrixLog6(T)
% :	Calculates exponential coordinate (a se3 matrix)
%	correspoding to rigid transformation matrix T

R = T(1:3,1:3);            t = T(1:3,4); 

if( abs( acos((trace(R)-1)/2) ) < 1e-5)
    xi = [ 0; 0; 0; t ];
else
    so3mat = MatrixLog3(R);		w = so3ToVec(so3mat);
	th = norm(w);				w = w/norm(w);
    A = (eye(3)-R)*VecToso3(w) + w*w'*th;
    xi = [ w; A\t ]*th;
end

se3mat = VecTose3(xi);
