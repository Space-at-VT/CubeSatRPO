classdef OrbitPlotter < handle
    %%%%% Orbit plotter class
    % Takes in a custom structure. Allows user to call functions to plot
    % the orbit in 3D (LVLH Frame), plot the control signals, and plot the
    % state time-histories
    properties
        States                      % Full states to plot
        StatesQ                     % States to plot with quiver
        Controls                    % Full controls to plot
        ControlsQ                   % Controls to plot with quiver
        Time                        % Time vector over which to plot
        umax                        % Maximum allowable control input
        
        plotLineStates              % Type of lines to plot transfer
        plotLineStatesMods          % Modifications to transfer plot lines
        plotLineStatesSizes         % Size of mods for transfer plot lines
        
        plotLineStatesQ             % Types of lines for thrust vectors
        plotLineStatesQMods         % Mods to thrust vectors
        plotLineStatesQSizes        % Size of mods to thrust vectors
        
        plotLegends                 % Legend entries
        plotIdentifier              % Type of motion model to plot (states are ordered differently)
        plotTitle                   % Title of plot
        plotLabelX                  % X-axis label
        plotLabelY                  % Y-axis label
        plotLabelZ                  % Z-axis label
        axisBounds                  % Bounds to the axes
        
        numInput                    % Number of control inputs
        spaceshuttle                % Variable to determine whether to plot shuttle or not
        fig3d                       % Figure object. Un-needed?
        figControls
        figPositions
        figVelocities
    end
    
    methods
        % Initialization
        function obj = OrbitPlotter(inputStruct)
            % Retrieve time vector
            for ii = 1:length(inputStruct.times)
                obj.Time{ii}            = inputStruct.times{ii};
            end
            % Retrieve states & motion model type
            for ii = 1:length(inputStruct.states.states)
                obj.States{ii}          = inputStruct.states.states{ii};
                obj.plotIdentifier{ii}  = inputStruct.id{ii};
            end
            % Retrieve states of finite thrusts for quiver plot
            if ~isempty(inputStruct.states.statesq)
                for ii = 1:length(inputStruct.states.statesq)
                    obj.StatesQ{ii}         = inputStruct.states.statesq{ii};
                end
            end
            % Retrieve control states
            if ~isempty(inputStruct.controls.controls)
                for ii = 1:length(inputStruct.controls.controls)
                    obj.Controls{ii}        = inputStruct.controls.controls{ii};
                end
            end
            % Retrieve thrust controls for quiver plot
            if ~isempty(inputStruct.controls.controlsq)
                for ii = 1:length(inputStruct.controls.controlsq)
                    obj.ControlsQ{ii}       = inputStruct.controls.controlsq{ii};
                end
            end
            % Retrieve line information for transfer plot
            for ii = 1:length(inputStruct.lines.linestates)
                obj.plotLineStates{ii}          = inputStruct.lines.linestates{ii};
                obj.plotLineStatesMods{ii}      = inputStruct.lines.linemods{ii};
                obj.plotLineStatesSizes(ii)     = inputStruct.lines.linesizes(ii);
            end
            % Retrieve line information for quiver/thrusts
            if ~isempty(inputStruct.lines.linestatesq)
                for ii = 1:length(inputStruct.lines.linestatesq)
                    obj.plotLineStatesQ{ii}         = inputStruct.lines.linestatesq{ii};
                    obj.plotLineStatesQMods{ii}     = inputStruct.lines.linemodsq{ii};
                    obj.plotLineStatesQSizes(ii)    = inputStruct.lines.linesizesq(ii);
                end
            end
            % Retrieve legend
            for ii = 1:length(inputStruct.legends)
                obj.plotLegends{ii}             = inputStruct.legends{ii};
            end
            % Retrieve number of control inputs (2 or 3). 
            if ~isempty(inputStruct.controls.controls)
                for ii = 1:length(inputStruct.controls.controls)
                    obj.numInput{ii}            = size(inputStruct.controls.controls{ii},1);
                end
            else obj.numInput{ii} = []; % For uncontrolled propagation, or only perturbed motion
            end
            
            % Retrieve max allowable control input
            if ~isempty(inputStruct.umax)
                obj.umax = inputStruct.umax;
            end
            
            % Retrieve plot information
            obj.plotTitle  = inputStruct.title;
            obj.axisBounds = inputStruct.bounds;
            obj.plotLabelX = inputStruct.labels{1};
            obj.plotLabelY = inputStruct.labels{2};
            obj.plotLabelZ = inputStruct.labels{3};
            % Decide to plot shuttle body or not
            if strcmp(inputStruct.shuttleFlag,'yes')
                obj.spaceshuttle = inputStruct.shuttleFlag;
            else
                obj.spaceshuttle = [];
            end
        end
        
        % Function plots the transfer in 3D Space with finite thrusts
        function obj = plot3DOrbit(obj)
            obj.fig3d = figure; % Creates new figure
            hold on
            grid on
            % Plots the full transfer in 3D LVLH frame
            for ii = 1:length(obj.States)
                X = obj.States{ii};
                % GA-STM & HCW have different orders of states
                if strcmp(obj.plotIdentifier{ii},'GASTM') == 1
                    plot3(X(1,:),X(3,:),X(5,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
%                     plot3(X(1,:),X(3,:),X(5,:));
                elseif strcmp(obj.plotIdentifier{ii},'LERM') == 1
                    plot3(X(1,:),X(3,:),X(5,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
                else
                    plot3(X(1,:),X(2,:),X(3,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
%                     plot3(X(1,:),X(2,:),X(3,:));
                end
            end
            % Plots the finite thrusts as v ectors using quiver3
            if ~isempty(obj.StatesQ)
                for ii = 1:length(obj.StatesQ)
                    X = obj.StatesQ{ii};
                    U = obj.ControlsQ{ii};
                    switch obj.numInput{ii}
                        case isempty(obj.numInput{ii})
                            nothing();
                        case 2
                            quiver3(X(:,1),X(:,2),X(:,3),zeros(size(X(:,1))),U(:,1),U(:,2),obj.plotLineStatesQ{ii},obj.plotLineStatesQMods{ii},obj.plotLineStatesQSizes(ii));
                        case 3
                            quiver3(X(:,1),X(:,2),X(:,3),U(:,1),U(:,2),U(:,3),0.5,obj.plotLineStatesQ{ii},obj.plotLineStatesQMods{ii},obj.plotLineStatesQSizes(ii));
                    end
                end
            else
                nothing();
            end
            % Create legend for transfer 
            legString = cell(length(obj.plotLegends),1);
            for ii = 1:length(obj.plotLegends)
                legString{ii} = [obj.plotLegends{ii}];
            end
            leg = legend(legString,'Location','Best');
            % Set plot information
            title1 = title(obj.plotTitle);
            xl = xlabel(obj.plotLabelX);
            yl = ylabel(obj.plotLabelY);
            zl = zlabel(obj.plotLabelZ);
            if ~isempty(obj.spaceshuttle)
                makeSpaceShuttle();
            else
            end
            if strcmp(obj.axisBounds,'tight') == 1
                axis tight
            else
                axis(obj.axisBounds)
            end
            set([title1,xl,yl,zl,leg],'interpreter','latex','fontsize',12)
        end
        
        % Function plots the control history of the transfer
        function obj = plotControls(obj)
            % Creates new figure
            obj.figControls = figure;
            % Get legend for control inputs
            legString = cell(length(obj.Controls),1);
            for ii = 1:length(obj.Controls)
                legString{ii} = [obj.plotLegends{ii}];
            end
            
            % Plots the control states for each controller
            for ii = 1:length(obj.Controls)
                % Get controller & time vectors
                U = obj.Controls{ii};
                T = obj.Time{ii};
                % Sets ylabels for different number of control inputs
                switch obj.numInput{ii}
                    case isempty(obj.numInput{ii})
                        return;
                    case 2
                        yaxislab = {'$u_y$, m/s$^{2}$', '$u_{z}$, m/s$^{2}$'};
                    case 3
                        yaxislab = {'$u_x$, m/s$^{2}$'; '$u_y$, m/s$^{2}$'; '$u_z$, m/s$^{2}$'};
                end 
                % Creates a subplot for each control direction
                for jj = 1:obj.numInput{ii}
                    subplot(obj.numInput{ii},1,jj)
                    hold on
                    grid on
                    % Use same plot states/colors/mods from 3D transfer
                    stairs(T,U(jj,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
%                     plot(T,U(jj,:))
                    % Label y-axes differently
                    yl = ylabel(yaxislab{jj});
                    axis([T(1),T(end),-1.2*obj.umax,1.2*obj.umax])
                    set(yl,'interpreter','latex','fontsize',12)
                    % Only plot legend on first subplot
                    if ii == length(obj.Controls) && jj == 1
                        leg = legend(legString,'Location','Best');
                        set(leg,'interpreter','latex','fontsize',12)
                    end
                end
            end
            
            % Just adding control input bounds & axes bounds
            % Legend was weird when I kept it in previous loop
            for jj = 1:obj.numInput{ii}
                subplot(obj.numInput{ii},1,jj)
                % Title only on the first subplot. Xlabel on last
                if jj == 1
                    title1 = title('Optimal Control History');
                    set(title1,'interpreter','latex','fontsize',12)
                elseif jj == obj.numInput{ii}
                    xl = xlabel('Time, s');
                    set(xl,'interpreter','latex','fontsize',12)
                end
                % Plot bounds on control input
                plot([T(1),T(end)],[obj.umax,obj.umax],'k--','lineWidth',2)
                plot([T(1),T(end)],[-obj.umax,-obj.umax],'k--','lineWidth',2)
            end
            
        end
        
        % Function plots the state history of the transfer
        % Position and Velocities are plotted in separate figures
        function obj = plotStates(obj)
            % Don't need to plot the initial/terminal conditions
            numTransfer = length(obj.States)-2;
            % Set ylabels for each plot
            yaxislabPos = {'$X$, m'; '$Y$, m'; '$Z$, m'};
            yaxislabVel = {'v$_x$, m/s'; 'v$_y$, m/s'; 'v$_z$, m/s'};
            % Get legend info
            legString = cell(numTransfer,1);
            for ii = 1:numTransfer
                legString{ii} = [obj.plotLegends{ii}];
            end
            % Plots the position for each controller
            obj.figPositions = figure;
            for ii = 1:numTransfer
                % Get state & time vectors
                X = obj.States{ii};
                T = obj.Time{ii};
                % GA-STM uses different order of state vector
                if strcmp(obj.plotIdentifier{ii},'GASTM') == 1
                    Pos(1,:) = X(1,:);
                    Pos(2,:) = X(3,:);
                    Pos(3,:) = X(5,:);
                elseif strcmp(obj.plotIdentifier{ii},'LERM') == 1
                    Pos(1,:) = X(1,:);
                    Pos(2,:) = X(3,:);
                    Pos(3,:) = X(5,:);
                else 
                    Pos(1,:) = X(1,:);
                    Pos(2,:) = X(2,:);
                    Pos(3,:) = X(3,:);
                end

                % Creates a subplot for each positional direction
                for jj = 1:3
                    subplot(3,1,jj)
                    hold on
                    grid on
                    % Use same plot states/colors/mods from 3D transfer
                    if ~isempty(obj.numInput{ii})
                        plot(T,Pos(jj,1:end-1),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
                    else
                        plot(T,Pos(jj,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
                    end
                    % Label axes differently
                    yl = ylabel(yaxislabPos{jj});
                    axis tight
                    set(yl,'interpreter','latex','fontsize',12)
                    % Plot information
                    if ii == numTransfer && jj == 1
                        % Only plot title/legend on first subplot
                        title1 = title('Optimal Position History');
                        leg = legend(legString,'Location','Best');
                        set([title1,leg],'interpreter','latex','fontsize',12)
                    elseif jj == 3
                        % Only plot xlabel on final subplot
                        xl = xlabel('Time, s');
                        set(xl,'interpreter','latex','fontsize',12)
                    end
                end
            end
            
            % Plot velocity history for each controller
            obj.figVelocities = figure;
            for ii = 1:numTransfer
                % Get state & time vectors
                X = obj.States{ii};
                T = obj.Time{ii};
                % GA-STM uses different orders of state vector
                if strcmp(obj.plotIdentifier{ii},'GASTM') == 1
                    Vel(1,:) = X(2,:);
                    Vel(2,:) = X(4,:);
                    Vel(3,:) = X(6,:);
                elseif strcmp(obj.plotIdentifier{ii},'LERM') == 1
                    Vel(1,:) = X(2,:);
                    Vel(2,:) = X(4,:);
                    Vel(3,:) = X(6,:);
                else
                    Vel(1,:) = X(4,:);
                    Vel(2,:) = X(5,:);
                    Vel(3,:) = X(6,:);
                end
                % Creates a subplot for each velocity direction
                for jj = 1:3
                    subplot(3,1,jj)
                    hold on
                    grid on
                    % Use same plot states/colors/mods from 3D transfer
                    if ~isempty(obj.numInput{ii})
                        plot(T,Vel(jj,1:end-1),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
                    else
                        plot(T,Vel(jj,:),obj.plotLineStates{ii},obj.plotLineStatesMods{ii},obj.plotLineStatesSizes(ii))
                    end
                    % Label axes differently
                    yl = ylabel(yaxislabVel{jj});
                    axis tight
                    set(yl,'interpreter','latex','fontsize',12)
                    % Plot information
                    if ii == numTransfer && jj == 1
                        % Only plot title/legend on first subplot
                        title1 = title('Optimal Velocity History');
                        leg = legend(legString,'Location','Best');
                        set([title1,leg],'interpreter','latex','fontsize',12)
                    elseif jj == 3
                        % Only plot xlabel on final subplot
                        xl = xlabel('Time, s');
                        set(xl,'interpreter','latex','fontsize',12)
                    end
                end
            end
            
        end
        
        %%%% TODO: Make a plotOrbitalElements() function!!!
        
    end
end

% Empty function for special cases
function nothing(~,~)
end

function makeSpaceShuttle
plotShuttle(0,0,0,0,-pi/2,pi/2,0.0487,1e-3,[1,1,0.5])
end

function plotShuttle(x,y,z,pitch,roll,yaw,scale_factor,step,cv)
load shuttle;
V = [-V(:,2) V(:,1) V(:,3)];
V(:,1) = V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2) = V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3) = V(:,3)-round(sum(V(:,3))/size(V,1));

correction = max(abs(V(:,1)));
V = V./(scale_factor*correction);
ii = length(x);
resto = mod(ii,step);

y = y;
z = z;
pitch = pitch;
roll = roll;
yaw = -yaw;

for jj = 1:step:(ii-resto)
    theta = pitch(jj);
    phi = -roll(jj);
    psi = yaw(jj);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Tbe = [cos(psi)*cos(theta), -sin(psi)*cos(theta), sin(theta);
        cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi) ...
        -sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
        -cos(theta)*sin(phi);
        -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
        sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi) ...
        cos(theta)*cos(phi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Vnew = V*Tbe;
    rif = [x(jj) y(jj) z(jj)];
    X0 = repmat(rif,size(Vnew,1),1);
    Vnew = Vnew + X0;
    p = patch('faces', F, 'vertices' ,Vnew);
    set(p, 'facec', cv);
    set(p, 'EdgeColor','none');
    H1 = light('Position',[-100 0 0],'Style','local');
    hold on
    %     lighting phong
    daspect([1 1 1]) ;
    
end
end