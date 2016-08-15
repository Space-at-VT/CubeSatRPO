function [ R ] = R3( theta )
%R3 Returns the rotation matrix of angle theta in degrees about the 3rd
%axis (z-axis)

R = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];

end

