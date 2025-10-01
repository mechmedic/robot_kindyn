function xi = se3ToVec( se3mat )
% se3ToVec : Returns 6 x 1 vector xi = [w,v] corresponding to a given 
%	         twist matrix 'se3mat', an element of Lie Algebra se3, 
%   xi = so3ToVec(se3mat)

if(size(se3mat,1)==[4,4])
    xi = [ se3mat(3,2), se3mat(1,3), se3mat(2,1), se3mat(1,4), se3mat(2,4), se3mat(3,4) ]';
else
    error('Invalid Input : Input must be se3');
end

