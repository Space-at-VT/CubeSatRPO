        
        %% Non-MPC Long-Range Maneuver
        function sat = phaseManeuver2(sat,scenario,Xf,tf,dtM)
            T0 = scenario.T;
            dt0 = scenario.dt;
            scenario.T = tf;
            scenario.dt = dtM;
            scenario.Nslack = 6;
            
            Nvar = scenario.Nvar;
            Neom = scenario.Neom;
            Nslack = 6;
            
            w1 = 1e-1;          %Control weight
            w2 = 1;             %Postion weight
            w3 = 1e3;           %Velocity weight
            
            % Function coefficients
            f = [w1*dt*ones(Nvar,1) %Control thrusts
                 zeros(Neom,1)
                 w2*ones(Nslack/2,1)
                 w3*ones(Nslack/2,1)];
            
            % Parameter bounds, lower & upper
            lb = [zeros(Nvar,1);   %Control thrusts
                 -inf*ones(Neom,1)
                 zeros(Nslack,1)];
            
            ub = [ones(Nvar,1);   %Control thrusts
                inf*ones(Neom,1)
                1e5*ones(Nslack,1)];
            
            % Equality contraints
            Aeq = []; beq = [];
            [Aeq,beq] = setEOM(Aeq,beq,sat,scenario);
            
            
            % Inequality contraints
            A = [];   b = [];
            [A,b] = setPhaseState([],[],sat,scenario,Xf);
            
            % Integer constraint
            intcon = [];
            
            % options = optimoptions(@linprog);
            [U,fval,exitflag] = intlinprog(f,intcon,A,b,Aeq,beq,lb,ub);
            scenario.dt = dt0;
            scenario.T = T0;
            
            jj = 1;
            for ii = 1:6:Nvar
                for kk = 1:(dt/dt0)
                    iter = length(sat.x);
                    sat.J(iter) = fval;
                    sat.flag(iter) = exitflag;
                    
                    sat.u(1:2:6) = sat.Rbi*U(ii:2:ii+5);
                    sat.u(2:2:6) = sat.Rbi*U(ii+1:2:ii+5);
                    sat.a = U(Nvar+jj:Nvar+jj+2);
                    
                    sat = signalsProp(sat,scenario);
                    sat = attitudeProp(sat,scenario);
                    sat.t(iter+1) = sat.t(iter)+scenario.dt;
                end
                jj = jj+3;
            end
        end
     