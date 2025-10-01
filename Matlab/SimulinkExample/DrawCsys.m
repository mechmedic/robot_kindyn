% DrawCsys( handle, T, scale )
% :	Draw Coordinate System correspondign to given rigid body transformation T

function DrawCsys( handle, T, scale )
	if nargin < 3
		scale = 1;
	end
	Axx = T(1:3,1);		Axy = T(1:3,2);
	Axz = T(1:3,3);		pOrg = T(1:3,4);

	pt = [ pOrg, pOrg+Axx*scale ];
	plot3(handle,pt(1,:),pt(2,:),pt(3,:),'Color', 'r');

	pt = [ pOrg, pOrg+Axy*scale ];
	plot3(handle,pt(1,:),pt(2,:),pt(3,:),'Color', 'g');

	pt = [ pOrg, pOrg+Axz*scale ];
	plot3(handle,pt(1,:),pt(2,:),pt(3,:),'Color', 'b');
	
	plot3(handle,pOrg(1),pOrg(2),pOrg(3),'Color', 'k', 'Marker','*');
end

