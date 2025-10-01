function so3mat = MatrixLog3( R )
% MatrixLog3 : Calculates exponential coordinate (a so3 matrix)
%			   correspoding to rotation matrix R

% if( R-eye(3) == 0)  
if( abs(R-eye(3)) < 1e-8)  
    w = [0;0;0]; 
else
    th = acos((trace(R)-1)/2);
    w = [ R(3,2)- R(2,3), R(1,3)- R(3,1), R(2,1)- R(1,2) ]';
    w = w / (2*sin(th));
    w = w*th;
end
so3mat = VecToso3(w);

