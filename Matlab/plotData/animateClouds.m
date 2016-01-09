function [] = animateClouds( clouds, fps )

    if nargin < 2
        fps = 20;
    end

    figure();
    hold on;
    for i =1:length(clouds)
        cla(1)
        
        cloud = [];
        [n, m] = size(clouds{i});
        if(n == 1)
           cloud = parseVectorizedCloud(clouds{i});
        elseif n == 3
           cloud = clouds{i}    
        else
           display('Unknown format of cloud');
        end
        
        plot3(cloud(1,:), cloud(2,:), -cloud(3,:), '*')
        
        %Delaying animation
        pause(1/fps);
    end

end

