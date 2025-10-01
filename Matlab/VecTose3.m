function se3mat = VecTose3( xi )
% VecTose3 : Returns twist matrix 'se3mat', an element of Lie Algebra se3,
%            corresponding to a given 6 x 1 vector 'xi'
%   se3mat = VecTose3(xi)

if(size(xi)==[6,1])
	w = xi(1:3);     v = xi(4:6);
	se3mat = [       0, -w(3),  w(2), v(1);
		          w(3),     0, -w(1), v(2);
		         -w(2),  w(1),    0 , v(3);
		             0,     0,    0,     0  ] ;
else
	error('Invalid Input : Input must be 6 x 1 vector');
end
