%Runs the simulation with a certain guide car frequency, randomly
%distributed.
function res = simulate_r(freq,disMatrix,startingSpeedMap,triggerDis)
%freq is the chance of a car being a guide car

%% Parameter definition
Ttot = 600; %Total simultaion time
Ncars = 103;

%% Calculation
x0 = zeros(2*Ncars,1);

for ii = 1:Ncars
   x0(ii) = 8*(Ncars-ii); %Starting Position [m]
   x0(ii+Ncars) = startingSpeedMap(ii); %Starting Velocity [m/s], random
end

guideMap = (rand(Ncars,1) < freq); %randomly introduce guide cars


f = @(t,x) idm_final(t,x,guideMap,disMatrix,triggerDis); %map which of the cars are guide cars


[TOUT,YOUT] = ode45(f,[0 Ttot],x0);

%% Evaluation
pos = 0;
ii = 0;
while pos < 8000
   ii = ii + 1;
   pos = YOUT(ii,Ncars);
end
res = TOUT(ii); %Arrival Time of last car at x = 8000m

%% Plot
ColorMap = [guideMap zeros(Ncars,1) ~guideMap]; %color guide cars red, normal cars blue
set(gca, 'ColorOrder', ColorMap, 'NextPlot', 'replacechildren');
plot(TOUT,YOUT(:,1:Ncars));
title('Position over Time')
end