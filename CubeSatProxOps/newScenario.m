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
        
        %% Solver
        Nobj = 0
        Nslack = 0;

    end
    properties (Dependent)
        n                       %1/s
        TP                      %s
        Nsim
        Nvar
        Neom
        Nbi
        Ntotal
    end
    methods
        function obj = scneario(name,epoch)
            if nargin > 0
                obj.name = name;
                obj.epoch = epoch;
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
        % Number of simulation time steps
        function Nsim = get.Nsim(obj)
            Nsim = length(0:obj.dt:obj.T)-1;
        end
        % Number of control variables
        function Nvar = get.Nvar(obj)
            Nvar = 6*obj.Nsim;
        end
        % Number of hcw acceleration terms
        function Neom = get.Neom(obj)
            Neom = 3*obj.Nsim;
        end
        % Number of collision avoidance binary variables
        function Nbi = get.Nbi(obj)
            Nbi = obj.Nvar*obj.Nobj;
        end
        % Total number of variables
        function Ntotal = get.Ntotal(obj)
            Ntotal = obj.Nvar+obj.Neom+obj.Nslack+obj.Nbi;
        end
    end
end