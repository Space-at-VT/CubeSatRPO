function X = HPOP(root,X0,u,dt)
% Initial state
x = X0(1);
y = X0(2);
z = X0(3);
vx = X0(4);
vy = X0(5);
vz = X0(6);

% Scenario
try
    scenario = root.Children.New('eScenario','HPOP');
catch
    scenario = root.CurrentScenario;
end
root.ExecuteCommand('SetAnalysisTimePeriod * "1 Jul 2012" "2 Jul 2012"');
root.ExecuteCommand('SetEpoch * "1 Jul 2012"');
root.ExecuteCommand('Units_Set * All Distance "Meters"');


% Origin
try
    root.ExecuteCommand('New / */Satellite Origin');
    root.ExecuteCommand('Astrogator */Satellite/Origin SetProp');
    root.ExecuteCommand('Astrogator */Satellite/Origin SetValue MainSequence.SegmentList.Initial_State.InitialState.Epoch 0 EpSec');
    root.ExecuteCommand('Astrogator */Satellite/Origin RunMCS');
    root.ExecuteCommand('Astrogator */Satellite/Origin SetValue MainSequence.SegmentList.Propagate.StoppingConditions.Duration.TripValue 1 sec');
catch   
end

% Satellite
try
    sat = scenario.Children.New('eSatellite','Satellite1');
    root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetProp');
    root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.CoordinateSystem "Satellite/Origin Body"');
    root.ExecuteCommand('ComponentBrowser */ Duplicate "Engine Models" "Constant Thrust and Isp" "CHAMPS"');
    root.ExecuteCommand('ComponentBrowser */ SetValue "Engine Models" "CHAMPS" Thrust 0.25 N');
    root.ExecuteCommand('Astrogator */Satellite/Satellite1 InsertSegment MainSequence.SegmentList.Propagate Maneuver');
    root.ExecuteCommand('Astrogator */Satellite/Satellite1 DeleteSegment MainSequence.SegmentList.Propagate');
catch
    sat = scenario.Children.Item('Satellite1');
end

% Initial state
% root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.Dry_Mass 13 kg')

root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Epoch 0 EpSec');
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.X %f m',z));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.Y %f m',y));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.Z %f m',-x));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.Vx %f m/sec',vz));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.Vy %f m/sec',vy));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Initial_State.InitialState.Cartesian.Vz %f m/sec',-vx));

% Maneuver
root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.MnvrType Finite');
root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.AttitudeControl Thrust Vector');
root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.ThrustAxes "Satellite/Origin Body"');
root.ExecuteCommand('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.CoordType Cartesian');
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.Cartesian.X %f',u(3)));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.Cartesian.Y %f',u(2)));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.FiniteMnvr.Cartesian.Z %f',-u(1)));
root.ExecuteCommand(sprintf('Astrogator */Satellite/Satellite1 SetValue MainSequence.SegmentList.Maneuver.StoppingConditions.Duration.TripValue %f sec',dt'));
root.ExecuteCommand('Astrogator */Satellite/Satellite1 RunMCS');

% New state
RICdp = sat.DataProviders.Item('RIC Coordinates');
RICdp.PreData = 'Satellite/Origin';
dpRes = RICdp.Exec(scenario.StartTime,scenario.StopTime,1);
radial = cell2mat(dpRes.DataSets.GetDataSetByName('Radial').GetValues);
inTrack = cell2mat(dpRes.DataSets.GetDataSetByName('In-Track').GetValues);
crossTrack = cell2mat(dpRes.DataSets.GetDataSetByName('Cross-Track').GetValues);
radialV = cell2mat(dpRes.DataSets.GetDataSetByName('Radial Rate').GetValues);
inTrackV = cell2mat(dpRes.DataSets.GetDataSetByName('In-Track Rate').GetValues);
crossTrackV = cell2mat(dpRes.DataSets.GetDataSetByName('Cross-Track Rate').GetValues);

X = 1e3*[-radial(2),-inTrack(2),crossTrack(2),-radialV(2),-inTrackV(2),crossTrackV(2)]';
end