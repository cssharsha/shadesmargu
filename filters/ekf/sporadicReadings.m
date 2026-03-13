function sporadicReadings( )
% Function to read from sporadic sensor readings file.

fid  = fopen('inputs.txt','r');
formatSpec = '%f %f';
sizeu = [2 Inf];
u = fscanf(fid,formatSpec,sizeu);
fclose(fid);

fid = fopen('sporadic_sensor_readings.txt','r');
formatSpec = '%f %f %f %f';
sizez = [4 Inf];
z = fscanf(fid,formatSpec,sizez);
fclose(fid);


mu = [0 0 0];
sigma = [0.001 0 0;0 0.001 0;0 0 0.001];
t = length(u);
sequence = zeros(t,3);
j = 1;
inp_time = 0.5;
for i = 1:t
    read_time = z(1,j);
    % Sensor reading available at this time.
    if (read_time == inp_time)
        % Do update along with prediction - Flag set to 1
        [mu,sigma] = ekf(mu,sigma,u(:,i),z(2:4,j),0.5,1);
        j = j + 1;
    else
        % Do only prediction - Flag set to 0
        [mu,sigma] = ekf(mu,sigma,u(:,i),z(2:4,j),0.5,0);
    end
    disp('Claculated mu:');
    disp(mu);
    disp('Calculated sigma:');
    disp(sigma);
    sequence(i,:) = mu;
    inp_time = inp_time + 0.5;
    % Plotting for only x and y of the state space
    plotErrorEllipsoid(mu(1:2),sigma(1:2,1:2));
end
disp(j);
disp(i);
plot(sequence(:,1),sequence(:,2));

end

