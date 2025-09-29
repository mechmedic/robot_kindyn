
% Solves Paden-Kahan subproblem 1, exp(w*th)x = y, given x,y,w

function th = PadenKahanFirst(w,x,y)

if( abs(dot(w,x)-dot(w,y)) > 0.00001 )
    error('No solution to PK1');
end

xprime = x - dot(w,x)*w;        yprime = y - dot(w,y)*w;
th = atan2(dot(w,cross(xprime,yprime)),dot(xprime,yprime));

end