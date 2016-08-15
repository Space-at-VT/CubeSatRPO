function [ R ] = R1( theta )
%R1 Returns the rotation matrix of angle theta in degrees about the 1st
%axis (x-axis)

R = [1 0 0; 0 cosd(theta) -sind(theta); 0 sind(theta) cosd(theta)];

end

