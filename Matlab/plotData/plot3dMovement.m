function [  ] = plot3dMovement( X, Y, Z )

% 2D graph
figure();
plot(X)
hold on;
plot(Y)
plot(Z)
legend('X','Y','Z')
title('Position')

% 3D graph

figure();
[n, m] = size(X);
if n ~= 1
   X = X';
   Y = Y';
   Z = Z';
end


l = length(X);
d = l:-1:1;
patch([X nan],[Y nan], [Z nan],[d nan], 'EdgeColor', 'interp');

end

