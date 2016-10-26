% Setting up plotting class

inputStruct.states.states = {controller{1}.X,controller{2}.X,controller{1}.X0,controller{1}.Xf};
inputStruct.states.statesq = {controller{1}.Xq,controller{2}.Xq};
inputStruct.controls.controls = {controller{1}.U,controller{2}.U};
inputStruct.controls.controlsq = {controller{1}.Uq,controller{2}.Uq};
inputStruct.times = {controller{1}.T,controller{2}.T};
inputStruct.id = {Name1,Name2,Name1,Name1};
inputStruct.lines.linestates = {'r-','b-','k*','g*'};
inputStruct.lines.linemods = {'linewidth','linewidth','markersize','markersize'};
inputStruct.lines.linesizes = [1,1,7,7];
inputStruct.lines.linestatesq = {'r','b'};
inputStruct.lines.linemodsq = {'linewidth','linewidth'};
inputStruct.lines.linesizesq = [2,2];
inputStruct.legends = {'GASTM Min Fuel','LERM Min Fuel','$X_0$','$X_f$','GATSM Min Fuel Thrust','LERM Min Fuel Thrust'};
inputStruct.title = 'Relative Trajectory';
inputStruct.labels = {'Radial, $X$, m','In-Track, $Y$, m','Cross-Track, $Z$, m'};
inputStruct.bounds = 'tight';
inputStruct.shuttleFlag = 'no';
inputStruct.umax = controller{1}.umax;

