function omg = so3ToVec( so3mat )
% so3ToVec : Returns 3 x 1 vector 'w' corresponding to 
%            a given 3 x 3 skew symmetric matrix 'so3mat', 
%            an element of Lie Algebra so3
%   w = so3ToVec(so3mat)

if(size(so3mat,1)==[3,3])
    omg = [ so3mat(3,2), so3mat(1,3), so3mat(2,1) ]';
else
    error('Invalid Input : Input must be so3');
end

