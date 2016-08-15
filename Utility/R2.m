function [ R ] = R2( theta )
%R2 Returns the rotation matrix of angle theta in degrees about the 2nd
%axis (y-axis)

R = [cosd(theta) 0 sind(theta); 0 1 0; -sind(theta) 0 cosd(theta)];

end

