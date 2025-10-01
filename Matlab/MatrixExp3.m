function R = MatrixExp3(so3mat)
% MatrixExp3 : Calculates rotation matrix R using matrix exponential.
%   R = MatrixExp3(so3mat) calculates rotation matrix corresponding to 
%   matrix exponential of given so3 matrix 'so3mat'

th = norm( so3ToVec(so3mat) );
hatw = so3mat/th;
w = so3ToVec(hatw);
R = cos(th)*eye(3) + sin(th)*hatw + (1-cos(th))*(w*w');

if(isa(R,'sym'))
   R=simplify(R);
end

% function R = MatrixExp3(so3mat, theta)
% % MatrixExp3 : Calculates rotation matrix R using matrix exponential.
% %   R = MatrixExp3(so3mat) calculates rotation matrix corresponding to 
% %   matrix exponential of given so3 matrix 'so3mat'
% %
% %   R = MatrixExp3(ax, theta) calculates rotation matrix corresponding to 
% %   rotation about an axis 'ax' by angle 'theta', assuming 'ax' is an unit vector.
% 
% if exist('theta','var')
%     th = theta;
%     w = so3mat;
%     hatw = VecToso3(w);    
% else
%     th = norm( so3ToVec(so3mat) );
%     hatw = so3mat/th;
%     w = so3ToVec(hatw);
% end
% %R = eye(3) + sin(th)*hatw + (1-cos(th))*hatw*hatw;
% R = cos(th)*eye(3) + sin(th)*hatw + (1-cos(th))*(w*w');

