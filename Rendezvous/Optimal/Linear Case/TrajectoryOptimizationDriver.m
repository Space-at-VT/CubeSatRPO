%%% Satellite Relative Motion Optimal Trajectory Generation


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars; close all; clc; asv(); addPaths(); 
%%% Global Constants
Req     = 6378.1363e3; % Radius of Earth (meters)
mu      = 3.986004415e14; % Gravitational parameter (m^3/s^2)

Initialization();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Gim-Alfriend STM Optimal Control

% Setup the GA problem. Chose options/variables
GASTM_Config();
% Instantiate maneuver class
controller{1} = ConvexSFFManeuver(initStruct);
% Perform trajectory optimization
controller{1}.minimizeFuel();
Name1 = 'GASTM';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% LERM Optimal Control

% % Setup the GA problem. Chose options/variables
% LERM_Config();
% % Instantiate maneuver class
% controller{2} = ConvexSFFManeuver(initStruct);
% % controller{2}.motionModel.propagateState();
% % Perform trajectory optimization
% controller{2}.minimizeFuel();
% Name2 = 'LERM';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% HCW Optimal Control

% Setup the HCW problem. Chose options/variables
HCW_Config();
% Instantiate maneuver class
controller{2} = ConvexSFFManeuver(initStruct);
% Perform trajectory optimization
controller{2}.minimizeFuel();
Name2 = 'HCW';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plotting & Results

% Setup plotting information/variables in this script
Plot_Config_OptControl();

% Instantiate plot class
plotMotion = OrbitPlotter(inputStruct);
% Plot the relative orbits
plotMotion.plot3DOrbit();
% Plot the control histories
plotMotion.plotControls();
% Plot the state histories
plotMotion.plotStates();

% Check diff in propagation/influence
% Akdiff=controller{1}.motionModel.Ak-controller{2}.motionModel.Ak;
% max(max(max(abs(Akdiff))))
% Bkdiff=controller{1}.motionModel.Bk-controller{2}.motionModel.Bk;
% max(max(max(abs(Bkdiff))))