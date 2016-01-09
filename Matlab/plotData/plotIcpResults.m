function [  ] = plotIcpResults( icpFileName )

rawData = load(icpFileName);

figure();
subplot(2,2,1)
plot(rawData(:,1))
title('Icp Score')

subplot(2,2,2)
plot(rawData(:,2))
title('Has converged')

subplot(2,2,3)
plot(rawData(:,3))
title('TranslationChangeGuess')

subplot(2,2,4)
plot(rawData(:,4))
title('AngleChangeGuess')

end

