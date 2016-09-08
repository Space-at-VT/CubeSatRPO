function createSTKfile(sat,scenario)

filename = strcat(sat.name,'.e');
fileID = fopen(filename,'w');
fprintf(fileID,'stk.v.8.0\r\n\r\nBEGIN Ephemeris\r\n\r\n');
fprintf(fileID,'NumberOfEphemerisPoints %d\r\n',length(sat.x));
fprintf(fileID,'CoordinateSystem Custom Body Satellite/Origin\r\n\r\n');
fprintf(fileID,'EphemerisTimePosVel\r\n\r\n');

t = 0;
for ii = 1:length(sat.x);
    fprintf(fileID,'%f %f %f %f %f %f %f\r\n',t,sat.y(ii),-sat.z(ii),-sat.x(ii),...
        sat.vy(ii),-sat.vz(ii),-sat.vx(ii));
    t = t+scenario.dt;
end

fprintf(fileID,'END Ephemeris');
fclose(fileID);

end