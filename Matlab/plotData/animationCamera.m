%% Display an animation of the camera movement
%% To use this function is necessary to have set up peter corke

function animationCamera(fileName, fps)

data = load(fileName);

[n,m] = size(data);

%% Static Data
i2c(1:4,1:4) = 0;
i2c(4,4) = 1;
i2c(1:3,1:3) = rotz(-90*pi/180)*rotx(-158*pi/180);
i2c(1:3,4) = [0.092, 0.124, -0.145]';
i2c2 = i2c;
i2c2(2,4)= -0.124; % Second camera


%% Draw loop
origin = eye(4);
T = eye(4)
for i =1:n
    pos = data(i,1:3);
    quat = data(i,4:7);
    
    cla(1);
    trplot(origin, 'color','k');
    hold on;
    T(1:3,1:3)= quat2rotm(quat);
    T(1:3,4) = pos;
    trplot(T,'color','r');
    
    trplot(i2c*T, 'color', 'b');
    trplot(i2c2*T, 'color', 'b');
    
    %Scaling base plot
    %Preserving axis configuration
    axis([-2,2,-2,2,-2,2]);
    %Delaying animation
    pause(1/fps);
end

end