x = zeros(99,200);
y = zeros(99,1);
v0map = 10*ones(103,1)+19*rand(103,1);
for ii = 0.01:0.01:0.2
    for jj = 1:200
        x(floor(ii*200)-1,jj) = simulate_r(ii,zeros(103,3),v0map,100);
    end
    y(floor(ii*200)-1) = mean(x(floor(ii*200)-1,:));
end

plot(0.01:0.005:0.5,y)