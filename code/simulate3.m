%Runs the simulation with a certain guide car frequency, randomly
%distributed.
function [res1,res2] = simulate3(freq,disMatrix)
%@param freq The chance of a car being a guide car
%@param dis 1:With disturbance 0:Without disturbance
%PRE: 0<=freq<=1
%% Parameter definition
%Simulation Parameters
Ttot = 10000; %Total simultaion time
Ncars = 1013;

%% Calculation
x0 = zeros(2*Ncars,1);

for ii = 1:Ncars
   x0(ii) = 8*(Ncars-ii); %Starting Position [m]
   x0(ii+Ncars) = 29; %Starting Velocity [m/s]
end

guideMap = (rand(Ncars,1) < freq); %randomly introduce guide cars

f = @(t,x) idm_final(t,x,guideMap,disMatrix,100); %map which of the cars are guide cars

[TOUT,YOUT] = ode45(f,[0 Ttot],x0);

%% Evaluation

%In this section we extract critical values for measuring the car
%throughput
[ycol, yrow] = size(YOUT);
measurement = zeros(3,3); 
taken = zeros(3,3);
for ii = 1:ycol
     if YOUT(ii,5) > 160000 && taken(2,3) == 0
         measurement(2,3) = TOUT(ii);
         taken(2,3) = 1;
     end
     if YOUT(ii,1013) > 160000 && taken(3,3) == 0
         measurement(3,3) = TOUT(ii);
         taken(3,3) = 1;
     end

end

res1 = measurement(3,3);
res2 = measurement(2,3);

%% Plot
ColorMap = [guideMap zeros(Ncars,1) ~guideMap]; %color guide cars red, normal cars blue
set(gca, 'ColorOrder', ColorMap, 'NextPlot', 'replacechildren');
plot(TOUT,YOUT(:,1:Ncars));
title('Position over Time');
end