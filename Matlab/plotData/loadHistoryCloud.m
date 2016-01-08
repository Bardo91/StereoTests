function [ vectorizedClouds ] = loadHistoryCloud( fileName )
rawData = importdata(fileName);

[n,m] = size(rawData);

% False if not in nan zone.
nanStatus = false;
vectorizedClouds = {};

auxRow = [];
auxRowIndex = 1;
for i=1:n
    for j =1:m
        if(~isnan(rawData(i,j)))
            nanStatus=false;
            auxRow(auxRowIndex) = rawData(i,j);
            auxRowIndex = auxRowIndex +1;
        else
            if(nanStatus == false)
                vectorizedClouds{length(vectorizedClouds)+1} = auxRow;
            end
            nanStatus = true;
            auxRow = [];
            auxRowIndex = 1;
        end
        
    end    
end

