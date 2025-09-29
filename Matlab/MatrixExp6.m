function T = MatrixExp6(se3mat)
% MatrixExp6 : Calculates rigid body transfornmation matrix T using 
%              matrix exponential.
%   T = MatrixExp3(se3mat) calculates transformation matrix corresponding
%   to matrix exponential of given se3 matrix 'se3mat'

xi = se3ToVec(se3mat);
w = xi(1:3);    v = xi(4:6);    th = norm(w);

if(th == 0)
    R = eye(3);
	t = v;
else
    R = MatrixExp3(se3mat(1:3,1:3));
    w = w/th;	v = v/th;
    t = (eye(3)-R)*cross(w,v)+ dot(w,v)*th*w;
end

T = [R,t;[0,0,0,1]];

