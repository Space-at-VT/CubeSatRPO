%% scenario defines the name, epoch, and current time of a simulation
classdef newScenario
    properties
        %% Model
        epoch = datestr(now)
        mu = 3.986004418e14;    %m^3/s^2
        dt = 1                  %s
        T = 15                  %s
        
        %% Origin
        a = 6738e3              %m
        ecc = 0                 %deg
        inc = 0                 %deg
        Om = 0                  %deg
        om = 0                  %deg
        nu = 0                  %deg
    end
    properties (Dependent)
        n                       %1/s
        TP                      %s
    end
    methods
        function obj = scneario(name)
            if nargin > 0
                obj.name = name;
            end
        end
        % Mean anamoly
        function n = get.n(obj)
            n = sqrt(obj.mu/obj.a^3);
        end
        % Orbital period
        function TP = get.TP(obj)
            TP = 2*pi*sqrt(obj.a^3/obj.mu);
        end
    end
end