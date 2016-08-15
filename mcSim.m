function [errorScore ] = mcSim(x0)
tic

%Read in the filter variables:
%   xAct : Actual state
%   t    : Time
%   u    : Acceleration control vector (HCW acceleration and acceleration
%          from thrust)
load('FilterVars.mat');

% Number of runs in the monte carlo simulation
numRuns = 1000;

% If given p, q, r, use those
if nargin == 1
    p = x0(1);
    q = x0(2);
    r = x0(3);
else % default p, q, r values
    p = 0.0001;
    q = 0.01;
    r = 0.1;
end

%number of iterations for the kalman filter
len = length(t);
rAct = xAct(1:3,:);
vAct = xAct(4:6,:);

error = zeros(6,len,numRuns);
measError = zeros(3,len,numRuns);

parfor jj = 1:numRuns

    kalman = kalmanSetup(rAct(:,1),vAct(:,1), p, q, r);
    
    for ii = 2:len
       kalman = stateEst(rAct(:,ii), vAct(:,ii), u(:,ii), kalman, t(:,ii));
    end

    error(:,:,jj) = kalman.xEst - xAct;
    measError(:,:,jj) = kalman.xMeas - xAct(1:3,:);

end

errorSTD = std(error, 0, 3);

errorMean = mean(abs(error),3);

measErrorMean = mean(abs(measError),3);

figure(1), clf;
hold on;
% title('x-Position Error');
plot(t,errorMean(1,:),'b');
plot(t,1.96.*errorSTD(1,:)+errorMean(1,:),'g');
plot(t,measErrorMean(1,:),'r.');
legend('Average Estimated Error', '95% Confidence Bound for Estimate', 'Measurement Error');
xlim([-50 2000]);
ylabel('Position Error, meters');
xlabel('Time, seconds');
xlim([0 2000]);
ylim([0 0.13]);
% set(gca, 'FontSize', 16);
saveas(gcf, 'xErrorPlot.png');
saveas(gcf, 'xErrorPlot.fig');

figure(2), clf;
hold on
% title('y-Position Error');
plot(t,errorMean(2,:),'b');
plot(t,1.96.*errorSTD(2,:)+errorMean(2,:),'g');
plot(t,measErrorMean(2,:),'r.');
legend('Average Estimated Error', '95% Confidence Bound for Estimate', 'Measurement Error');
xlim([-50 2000]);
ylabel('Position Error, meters');
xlabel('Time, seconds');
xlim([0 2000]);
% set(gca, 'FontSize', 16);
saveas(gcf, 'yErrorPlot.png');
saveas(gcf, 'yErrorPlot.fig');

figure(3), clf;
hold on
% title('z-Position Error');
plot(t,errorMean(3,:),'b');
plot(t,1.96.*errorSTD(3,:)+errorMean(3,:),'g');
plot(t,measErrorMean(3,:),'r.');
legend('Average Estimated Error', '95% Confidence Bound for Estimate', 'Measurement Error');
xlim([-50 2000]);
ylabel('Position Error, meters');
xlabel('Time, seconds');
xlim([0 2000]);
saveas(gcf, 'zErrorPlot.png');
saveas(gcf, 'zErrorPlot.fig');
% set(gca, 'FontSize', 16);

% figure(2), clf;
% hold on;
% plot(t,rAct(1,:)+errorMean(1,:),'b');
% plot(t,rAct(1,:)+1.96.*errorSTD(1,:)+errorMean(1,:),'g');
% plot(t,rAct(1,:)+measErrorMean(1,:),'r.');

% 
% plot(t,errorSTD(1,:));
% plot(t,errorSTD(2,:));
% plot(t,errorSTD(3,:));
% plot(t,errorSTD(4,:));
% plot(t,errorSTD(5,:));
% plot(t,errorSTD(6,:));
% 
% figure(2), clf;
% hold on;
% title('Average Estimation Error');
% plot(t,errorMean(1,:));
% plot(t,errorMean(2,:));
% plot(t,errorMean(3,:));
% plot(t,errorMean(4,:));
% plot(t,errorMean(5,:));
% plot(t,errorMean(6,:));
% 
% figure(3), clf;
% hold on;
% title('Measurement Error');
% plot(t,measErrorMean(1,:));
% plot(t,measErrorMean(2,:));
% plot(t,measErrorMean(3,:));
% 
errorScore = sum( sqrt(sum(errorMean(1:3,:).^2)));
% measErrorScore = sum(sum(measErrorMean.^2))
toc
end