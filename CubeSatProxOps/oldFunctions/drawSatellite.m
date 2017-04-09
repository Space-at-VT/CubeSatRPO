function drawSatellite(r,h,panel,panAng)
grid on

c = [0.5,0.5,0.5];
ang = pi/8;
theta = linspace(ang,2*pi+ang);

% Top face
topx = r*cos(theta);
topy = r*sin(theta);
topz = h/2*ones(1,length(theta));
patch(topx,topy,topz,c,'EdgeColor','None')

% Bottom face
botx = r*cos(theta);
boty = r*sin(theta);
botz = -h/2*ones(1,length(theta));

patch(botx,boty,botz,c,'EdgeColor','None')

% Panels
panc = [0.6,0.8,1];
panx = [r,r+panel(1),r+panel(1),r];
pany = [-panel(2)/2,-panel(2)/2,panel(2)/2,panel(2)/2]*cosd(panAng);
panz = [-panel(2)/2,-panel(2)/2,panel(2)/2,panel(2)/2]*sind(panAng);
patch(panx,pany,panz,panc)
patch(-panx,pany,panz,panc)

% Sides
for ii = 1:length(theta)-1
   x = [topx(ii),topx(ii+1),botx(ii+1),botx(ii)];
   y = [topy(ii),topy(ii+1),boty(ii+1),boty(ii)];
   z = [topz(ii),topz(ii+1),botz(ii+1),botz(ii)];
   patch(x,y,z,c,'EdgeColor','None')
end

% Plotting
% light('Position',[0 -100 100],'Style','local');