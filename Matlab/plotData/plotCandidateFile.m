%% Ploting candidate labels over time

function plotCandidateFile(fileName)
    allData = load(fileName);
    
    [n,m] = size(allData);
    
    probs = [];
    
    for i = 1:n
        probs = [probs; allData(i, 5:end)];
    end
    
    probs = probs';
    
    figure();
    hold on;
    legends = {};
    
    lineStyles = {'-','--','-o', '-*', '--o', '--*'};
    
    for i = 1:length(probs(:,1))
        plot(probs(i,:),lineStyles{floor(i/7)+1});
        legends{i} = strcat('label ', int2str(i-1));
    end
    [h,icons,plots,str] = legend(legends);
    set(icons(:),'LineWidth',2); %// Or whatever
end