function PlotShuttleApproximationsV2(Ellipsoids,Prisms,Actual)
epsilon = 1e-3;
Npanels = 30;

% Fuselage
xminF = -5.5;
xmaxF = 5;
yminF = -19;
ymaxF = 23;
zminF = -4;
zmaxF = 4;

V = [xminF yminF zminF
     xminF yminF zmaxF
     xminF ymaxF zminF
     xminF ymaxF zmaxF
     xmaxF yminF zminF
     xmaxF ymaxF zminF
     xmaxF yminF zmaxF
     xmaxF ymaxF zmaxF];
F = [1 2 4 3
     5 6 8 7
     1 2 7 5
     2 4 8 7
     3 6 8 4];

[XFuselage,YFuselage,ZFuselage] = ellipsoid(0,0,0,max([xmaxF,xminF]),...
    max([yminF,ymaxF]),max([zminF,zmaxF]),Npanels);

shuttleStructV2.Fuselage.Dimensions = [xminF,yminF,zminF,xmaxF,ymaxF,zmaxF];
shuttleStructV2.Fuselage.Ellipsoid.XFuselage = XFuselage;
shuttleStructV2.Fuselage.Ellipsoid.YFuselage = YFuselage;
shuttleStructV2.Fuselage.Ellipsoid.ZFuselage = ZFuselage;
shuttleStructV2.Fuselage.Vertices = V;
shuttleStructV2.Fuselage.Faces = F;

% Rudder
xmaxR = xmaxF;
xminR = -12;
ymaxR = -7.7;
yminR = -17;
zminR = -1;
zmaxR = 1;

V = [xminR yminR zminR
     xminR yminR zmaxR
     xminR ymaxR zminR
     xminR ymaxR zmaxR
     xmaxR yminR zminR
     xmaxR ymaxR zminR
     xmaxR yminR zmaxR
     xmaxR ymaxR zmaxR];
F = [1 2 4 3
     5 6 8 7
     1 2 7 5
     2 4 8 7
     3 6 8 4];
 
[XRudder,YRudder,ZRudder] = ellipsoid(xmaxR + (xminR - xmaxR)/2,...
    ymaxR + (yminR - ymaxR)/2,zmaxR + (zminR - zmaxR)/2,(xmaxR-xminR)/2,...
    (ymaxR-yminR)/2,(zmaxR-zminR)/2,Npanels);

shuttleStructV2.Rudder.Dimensions = [xminR,yminR,zminR,xmaxR,ymaxR,zmaxR];
shuttleStructV2.Rudder.Ellipsoid.XRudder = XRudder;
shuttleStructV2.Rudder.Ellipsoid.YRudder = YRudder;
shuttleStructV2.Rudder.Ellipsoid.ZRudder = ZRudder;
shuttleStructV2.Rudder.Vertices = V;
shuttleStructV2.Rudder.Faces = F;


% Right Wing- Main
xmaxRWM = xmaxF;
xminRWM = 2.5;
ymaxRWM = 5;
yminRWM = -9.3;
% zminRWM = zmaxF - epsilon;
zminRWM = -14;
zmaxRWM = 14;

V = [xminRWM yminRWM zminRWM
     xminRWM yminRWM zmaxRWM
     xminRWM ymaxRWM zminRWM
     xminRWM ymaxRWM zmaxRWM
     xmaxRWM yminRWM zminRWM
     xmaxRWM ymaxRWM zminRWM
     xmaxRWM yminRWM zmaxRWM
     xmaxRWM ymaxRWM zmaxRWM];
F = [1 2 4 3
     5 6 8 7
     1 2 7 5
     2 4 8 7
     3 6 8 4];
[XRightWing,YRightWing,ZRightWing] = ellipsoid(xmaxRWM + (xminRWM - ...
    xmaxRWM)/2,ymaxRWM + (yminRWM - ymaxRWM)/2,zmaxRWM + (zminRWM - ...
    zmaxRWM)/2,(xmaxRWM - xminRWM)/2,(ymaxRWM - yminRWM)/2,(zmaxRWM - ...
    zminRWM)/2,Npanels);

shuttleStructV2.RightWingMain.Dimensions = [xminRWM,yminRWM,zminRWM,xmaxRWM,ymaxRWM,zmaxRWM];
shuttleStructV2.RightWingMain.Ellipsoid.XRightWingMain = XRightWing;
shuttleStructV2.RightWingMain.Ellipsoid.YRightWingMain = YRightWing;
shuttleStructV2.RightWingMain.Ellipsoid.ZRightWingMain = ZRightWing;
shuttleStructV2.RightWingMain.Vertices = V;
shuttleStructV2.RightWingMain.Faces = F;

% Right Wing- strake
xmaxRWS = xmaxF;
xminRWS = xminRWM;
yminRWS = ymaxRWM - epsilon;
ymaxRWS = 10.5;
zminRWS = -7;
% zminRWS = zmaxF - epsilon;
zmaxRWS = 7;

V = [xminRWS yminRWS zminRWS
     xminRWS yminRWS zmaxRWS
     xminRWS ymaxRWS zminRWS
     xminRWS ymaxRWS zmaxRWS
     xmaxRWS yminRWS zminRWS
     xmaxRWS ymaxRWS zminRWS
     xmaxRWS yminRWS zmaxRWS
     xmaxRWS ymaxRWS zmaxRWS];
F = [1 2 4 3
     5 6 8 7
     1 2 7 5
     2 4 8 7
     3 6 8 4];

[XRightWingStrake,YRightWingStrake,ZRightWingStrake] = ellipsoid(xmaxRWS + (xminRWS - ...
    xmaxRWS)/2,ymaxRWS + (yminRWS - ymaxRWS)/2,zmaxRWS + (zminRWS - ...
    zmaxRWS)/2,(xmaxRWS - xminRWS)/2,(ymaxRWS - yminRWS)/2,(zmaxRWS - ...
    zminRWS)/2,Npanels);

shuttleStructV2.RightWingStrake.Dimensions = [xminRWS,yminRWS,zminRWS,xmaxRWS,ymaxRWS,zmaxRWS];
shuttleStructV2.RightWingStrake.Ellipsoid.XRightWingStrake = XRightWingStrake;
shuttleStructV2.RightWingStrake.Ellipsoid.YRightWingStrake = YRightWingStrake;
shuttleStructV2.RightWingStrake.Ellipsoid.ZRightWingStrake = ZRightWingStrake;
shuttleStructV2.RightWingStrake.Vertices = V;
shuttleStructV2.RightWingStrake.Faces = F;

if Ellipsoids == 1 && Prisms == 1 && Actual == 1
    plotShuttle(0,0,0,0,-pi/2,pi/2,0.0487,1e-3,[1,1,0.5])
    hold on
    shuttleConvexHullApprox(xminF,xmaxF,yminF,ymaxF,zminF,zmaxF,'r')
    surf(XFuselage,YFuselage,ZFuselage)
    shuttleConvexHullApprox(xminR,xmaxR,yminR,ymaxR,zminR,zmaxR,'k')
    surf(XRudder,YRudder,ZRudder)
    shuttleConvexHullApprox(xminRWM,xmaxRWM,yminRWM,ymaxRWM,zminRWM,zmaxRWM,'b')
    surf(XRightWing,YRightWing,ZRightWing)
    shuttleConvexHullApprox(xminRWS,xmaxRWS,yminRWS,ymaxRWS,zminRWS,zmaxRWS,'b')
    surf(XRightWingStrake,YRightWingStrake,ZRightWingStrake)
elseif Ellipsoids == 1 && Prisms == 1 && Actual == 0
    hold on
    shuttleConvexHullApprox(xminF,xmaxF,yminF,ymaxF,zminF,zmaxF,'r')
    surf(XFuselage,YFuselage,ZFuselage)
    shuttleConvexHullApprox(xminR,xmaxR,yminR,ymaxR,zminR,zmaxR,'k')
    surf(XRudder,YRudder,ZRudder)
    shuttleConvexHullApprox(xminRWM,xmaxRWM,yminRWM,ymaxRWM,zminRWM,zmaxRWM,'b')
    surf(XRightWing,YRightWing,ZRightWing)
    shuttleConvexHullApprox(xminRWS,xmaxRWS,yminRWS,ymaxRWS,zminRWS,zmaxRWS,'b')
    surf(XRightWingStrake,YRightWingStrake,ZRightWingStrake)
elseif Ellipsoids == 1 && Prisms == 0 && Actual == 0
    hold on
    surf(XFuselage,YFuselage,ZFuselage)
    surf(XRudder,YRudder,ZRudder)
    surf(XRightWing,YRightWing,ZRightWing)
    surf(XRightWingStrake,YRightWingStrake,ZRightWingStrake)
elseif Ellipsoids == 0 && Prisms == 1 && Actual == 0
    hold on
    shuttleConvexHullApprox(xminF,xmaxF,yminF,ymaxF,zminF,zmaxF,'r')
    shuttleConvexHullApprox(xminR,xmaxR,yminR,ymaxR,zminR,zmaxR,'k')
    shuttleConvexHullApprox(xminRWM,xmaxRWM,yminRWM,ymaxRWM,zminRWM,zmaxRWM,'b')
    shuttleConvexHullApprox(xminRWS,xmaxRWS,yminRWS,ymaxRWS,zminRWS,zmaxRWS,'b')
elseif Ellipsoids == 0 && Prisms == 1 && Actual == 1
    plotShuttle(0,0,0,0,-pi/2,pi/2,0.0487,1e-3,[1,1,0.5])
    hold on
    shuttleConvexHullApprox(xminF,xmaxF,yminF,ymaxF,zminF,zmaxF,'r')
    shuttleConvexHullApprox(xminR,xmaxR,yminR,ymaxR,zminR,zmaxR,'k')
    shuttleConvexHullApprox(xminRWM,xmaxRWM,yminRWM,ymaxRWM,zminRWM,zmaxRWM,'b')
    shuttleConvexHullApprox(xminRWS,xmaxRWS,yminRWS,ymaxRWS,zminRWS,zmaxRWS,'b')
elseif Ellipsoids == 1 && Prisms == 0 && Actual == 1
    plotShuttle(0,0,0,0,-pi/2,pi/2,0.0487,1e-3,[1,1,0.5])
    hold on
    surf(XFuselage,YFuselage,ZFuselage)
    surf(XRudder,YRudder,ZRudder)
    surf(XRightWing,YRightWing,ZRightWing)
    surf(XRightWingStrake,YRightWingStrake,ZRightWingStrake)
else
    plotShuttle(0,0,0,0,-pi/2,pi/2,0.0487,1e-3,[1,1,0.5])
end
xlabel('X');
ylabel('Y');
zlabel('Z');

save('shuttleStructV2.mat','shuttleStructV2')
end