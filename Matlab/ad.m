function adV = ad(xi)
% ad : Compute 6 x 6 matrix from twist representing
% Lie Bracket operation 
%   adV = ad(xi) from twist vector 'xi', computes 6 x 6 matrix 
%   representing LieBracket

if(~isvector(xi))
	error('Not proper input, need twists');
end
w = xi(1:3);	v = xi(4:6);
adV = [VecToso3(w),zeros(3,3); VecToso3(v), VecToso3(w)];
