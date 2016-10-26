% Setting up plotting class

inputStruct.states.states = {GA.X,LERM.X,HCW.X,GA.X(:,1),GA.X(:,end)};
inputStruct.times = {GA.time,LERM.time,HCW.time};
inputStruct.id = {'GASTM','LERM','HCW','GASTM','GASTM'};
inputStruct.lines.linestates = {'r-','b-','g-','k*','m*'};
inputStruct.lines.linemods = {'linewidth','linewidth','linewidth','markersize','markersize'};
inputStruct.lines.linesizes = [3,2,1,7,7];
inputStruct.legends = {'GASTM','LERM','HCW','$X_0$','$X_f$'};
inputStruct.title = 'Relative Trajectory';
inputStruct.labels = {'Radial, $X$, m','In-Track, $Y$, m','Cross-Track, $Z$, m'};
inputStruct.bounds = 'tight';
inputStruct.shuttleFlag = 'no';
inputStruct.states.statesq = [];
inputStruct.controls.controls = [];
inputStruct.controls.controlsq = [];
inputStruct.lines.linestatesq = {'r','b'};
inputStruct.lines.linemodsq = {'linewidth','linewidth'};
inputStruct.lines.linesizesq = [3,3];
inputStruct.umax = [];