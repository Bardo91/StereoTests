function [ vectorizedClouds ] = loadHistoryCloud( fileName, offset)

if nargin < 2
    offset = 0;
end
  
rawData = importdata(fileName);

[n,m] = size(rawData);

% False if not in nan zone.
nanStatus = true;
vectorizedClouds = {};

auxRow = [];
auxRowIndex = 1;
for i=1:n
    for j =1:m       
        if(~isnan(rawData(i,j)))
            if nanStatus & j < 1+offset
                continue;
            end
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

