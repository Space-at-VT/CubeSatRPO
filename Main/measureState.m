function [ stateMeas ] = measureState( positionAct)
%measureState Translates the actual modeled state to something the sensors
%might read.
%Sensors read perfect info for now

%% Sensors
% Star Tracker:
%   http://bluecanyontech.com/wp-content/uploads/2015/12/NST-Data-Sheet_4.1.pdf
%   Update: 5 Hz 
%   Bore-sight accuracy: 6 arcsec (1-sigma)
%   Roll axis accuracy: 40 arcsec (1-sigma) 
%   FOV: 10x12 degree (vert x horz)

% Long range Laser Rangefinder:
%   http://cvs.flir.com/mlr5k-lx-laser-rangefinder-datasheet
%   Update: 1 hz
%   Range: 50-5700m
%   Accuracy: +- 1m (95% confidence?)
%   Resolution: 0.1m

% Short range Laser Rangefinder:
%   http://cvs.flir.com/mlr100-laser-rangefinder-datasheet
%   Update: 500 hz
%   Range: ~0cm-100m
%   Accuracy: ????
%   Resolution: <0.2m




range = norm(positionAct);

directVect = (1/range)*positionAct;

% theta = acosd(dot(directVect,[1;0;0]));
% 
% rotateAxis = cross(directVect, [1;0;0]);
% rotateAxis = [1;0;0];
% 
% Rmatrix = cosd(theta)*eye(3)+sind(theta)*skew(rotateAxis) + (1-cosd(theta))*(rotateAxis*rotateAxis');

rollErr = normrnd(0, 400/3600);

boreErrOne = normrnd(0, 60/3600);
boreErrTwo = normrnd(0, 60/3600);

if range>50
    rangeErr = normrnd(0, 0.2551); % 95% confidence bound, +-1m
    rangeMeas = round( (rangeErr+range),1 ); %0.1m resolution
else
    rangeErr = normrnd(0, 0.0255); % 95% confidence bound, +-0.1m
    rangeMeas = 2*round( (rangeErr+range)/2,1 ); %0.2m resolution
end

stateMeas = rangeMeas*R1(rollErr)*R2(boreErrOne)*R3(boreErrTwo)*directVect;

% stateMeas = positionAct;
end

