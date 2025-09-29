
% Solves Paden-Kahan subproblem 2, exp(w1*th1)*exp(w2*th2)*x = y, 
% given x,y,w1,w2

function th = PadenKahanSecond(w1,w2,x,y)

	alp = ( dot(w1,w2)*dot(w2,x) - dot(w1,y) ) / ( (dot(w1,w2))^2 - 1 );
	bet = ( dot(w1,w2)*dot(w1,y) - dot(w2,x) ) / ( (dot(w1,w2))^2 - 1 );

    gam = ((norm(x))^2 - alp^2 - bet^2 - 2*alp*bet*dot(w1,w2));

    if( gam < 0 )
        error('No solution to PK2');
    end
    
    gam = sqrt(gam) / norm(cross(w1,w2));    
    
    z1 = alp*w1 + bet*w2 + gam*cross(w1,w2);
    z2 = alp*w1 + bet*w2 - gam*cross(w1,w2);

    th = [ -PadenKahanFirst(w1,y,z1), PadenKahanFirst(w2,x,z1);
           -PadenKahanFirst(w1,y,z2), PadenKahanFirst(w2,x,z2)  ];

end
