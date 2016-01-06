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
    for i = 1:n
        plot(probs(
    end
end