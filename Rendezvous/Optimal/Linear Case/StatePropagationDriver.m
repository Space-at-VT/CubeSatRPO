%%% Satellite Relative Motion State Propagation Comparison


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars; close all; clc; asv(); addPaths(); 
%%% Global Constants
Req     = 6378.1363e3; % Radius of Earth (meters)
mu      = 3.986004415e14; % Gravitational parameter (m^3/s^2)

Initialization();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Gim-Alfriend STM Model

% Setup the GA problem. 
GASTM_Config();
% Instantiate motion model class
GA = GimAlfriendSTM(initStruct);
% Perform state propagation forward in time
GA.propagateState();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Linearized Equations of Relative Motion Model

% Setup the LERM problem. 
LERM_Config();
% Instantiate motion model class
LERM = LERM(initStruct);
% Perform state propagation forward in time
LERM.propagateState();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Hill-Clohessy-Wiltshere Model

% Setup the HCW problem. 
HCW_Config();
% Instantiate motion model class
HCW = HCW(initStruct);
% Perform state propagation forward in time
HCW.propagateState();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Plotting & Results

Plot_Config_StateProp();

% Instantiate plot class
plotMotion = OrbitPlotter(inputStruct);
% % Plot the relative orbits
plotMotion.plot3DOrbit();
% Plot the state histories
plotMotion.plotStates();

% Checks