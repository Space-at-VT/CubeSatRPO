
iter = 0;
for ii = 1:6
    for jj = 1:6
        for kk = 1:6
            iter = iter+1;
            xNot(:,iter) = [10^-ii; 10^-jj; 10^-kk];
            errorScore(iter) = mcSim(xNot(:,iter));
        end
    end
end

x0out = xNot(:, errorScore == min(errorScore));
errorScoreOut = min(errorScore);