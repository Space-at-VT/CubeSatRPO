%% scenario defines the name, epoch, and current time of a simulation
classdef hcwScenario
    properties
        name = ''
        epoch = datestr(now)
        mu = 3.986004418e14;    %m^3/s^2
        a = 6738e3              %m
        t = 0                   %s
        dt = 1                  %s
        tmax = 5000             %s
        T = 20                  %s
        ecc = 0;
    end
    properties (Dependent)
        n                       %1/s
        TP                      %s
        Nbi
    end
    properties (Hidden)
        Nobj = 0
    end
    properties (Dependent,Hidden)
        Nsim
        Nvar
        Nhcw
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
            Nsim = length(obj.t:obj.dt:(obj.t+obj.T))-1;
        end
        % Number of control variables
        function Nvar = get.Nvar(obj)
            Nvar = 6*obj.Nsim;
        end
        % Number of hcw acceleration terms
        function Nhcw = get.Nhcw(obj)
            Nhcw = 3*obj.Nsim;
        end
        % Number of collision avoidance binary variables
        function Nbi = get.Nbi(obj)
            Nbi = obj.Nvar*obj.Nobj;
        end
        % Total number of variables
        function Ntotal = get.Ntotal(obj)
            Ntotal = obj.Nvar+obj.Nhcw+3+obj.Nbi;
        end
    end
end