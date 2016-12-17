%Runs the simulation with a certain guide car frequency, randomly
%distributed.
function res = simulate_r(freq,disMatrix,v0map,triggerDis)
%@param freq The chance of a car being a guide car
%@param dis 1:With disturbance 0:Without disturbance

%Simulation Parameters
Ttot = 600; %Total simultaion time
Ncars = 103;

x0 = zeros(2*Ncars,1);

for ii = 1:Ncars
   x0(ii) = 8*(Ncars-ii); %Starting Position [m]
   x0(ii+Ncars) = v0map(ii); %Starting Velocity [m/s]
end

guideMap = (rand(Ncars,1) < freq); %randomly introduce guide cars

f = @(t,x) idm_final(t,x,guideMap,disMatrix,triggerDis); %map which of the cars are guide cars


[TOUT,YOUT] = ode45(f,[0 Ttot],x0);
res = YOUT(end,Ncars);

%Plot all cars' positions vs. time
ColorMap = [guideMap zeros(Ncars,1) ~guideMap]; %color guide cars red, normal cars blue

set(gca, 'ColorOrder', ColorMap, 'NextPlot', 'replacechildren');
%beep
plot(TOUT,YOUT(:,1:Ncars));
title('position')
end