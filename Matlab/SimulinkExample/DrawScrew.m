% DrawScrew( handle, se3mat, color )
% :	Draw screw axis and point on a screw correspondign to given se3

function DrawScrew( handle, se3mat, col, len )
	if nargin == 2
		col = [0,0,1];
		len = 3;
	end
	
	if nargin == 3
		len = 3;
	end

	xi = se3ToVec(se3mat);
	
	w = xi(1:3);		v = xi(4:6);	th = norm(w);	
	
	if(th==0)
		error('Translation Joint');
	else
		w = w/th;			v = v/th;
		p = cross(w,v);
		l = w;
		pt = [ p - len*w, p + len*w ];
	end
	
	plot3(handle,pt(1,:),pt(2,:),pt(3,:),'Color', col);
    plot3(handle,p(1),p(2),p(3),'Color', col, 'Marker','*');
end

