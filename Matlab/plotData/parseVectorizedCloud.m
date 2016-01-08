function [ cloud ] = parseVectorizedCloud( vector )
    cloud = reshape(vector, [3,length(vector)/3]);
end

