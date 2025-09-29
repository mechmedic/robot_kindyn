function so3mat = VecToso3( omg )
% VecToso3 : Returns a 3 x 3 skew symmetric matrix 'so3mat', an element of 
%            Lie Algebra so3, corresponding to a given 3 x 1 vector 'w'
%   so3mat = VecToso3(w)

if(size(omg)==[3,1])
    so3mat = [   0,    -omg(3),  omg(2); 
               omg(3),    0,    -omg(1);
              -omg(2),  omg(1),    0     ] ;
else
    error('Invalid Input : Input must be 3 x 1 vector');
end
