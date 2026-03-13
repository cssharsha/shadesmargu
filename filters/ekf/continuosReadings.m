function continuosReadings()
% Function to read from continuous sensor readings.

fid  = fopen('inputs.txt','r');
formatSpec = '%f %f';
sizeu = [2 Inf];
u = fscanf(fid,formatSpec,sizeu);
fclose(fid);

fid = fopen('sensor_readings.txt','r');
formatSpec = '%f %f %f';
sizez = [3 Inf];
z = fscanf(fid,formatSpec,sizez);
fclose(fid);

mu = [0 0 0];
sigma = [0.001 0 0;0 0.001 0;0 0 0.001];
t = length(z);
sequence = zeros(t,3);
for i = 1:t
    [mu,sigma] = ekf(mu,sigma,u(:,i),z(:,i),0.5,1);
    disp('Claculated mu:');
    disp(mu);
    disp('Calculated sigma:');
    disp(sigma);
    sequence(i,:) = mu;
    % Plotting for only x and y of the state space
    plotErrorEllipsoid(mu(1:2),sigma(1:2,1:2));
end
disp(i);

plot(sequence(:,1),sequence(:,2));hold on;


end

