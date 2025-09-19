% Define waypoints that the rover will pass through.
sim_freq = 10;
final_time = 1e2;
n_sim = final_time * sim_freq;
t = linspace(0, final_time, n_sim);
tc = t * 3 * pi/ (2 * t(end));
tm = 2 * t / t(end);
waypoints = [sin(tc); tm.*cos(tc); 0*tc]';
% Create a trajectory from the information above.
trajectory = waypointTrajectory(waypoints, t);
% Get the elements from the Trajectory.
count = 1;
while(~isDone(trajectory))
   [position(count,:),orientation(count),velocity(count,:),acceleration(count,:),angularVelocity(count,:)] = trajectory();
   count = count + 1;
end

% Generate noisy GNSS measurements of the trajectory.
gnssNoiseStd = 0.05; % Standard deviation of the noise
numMeasurements = size(position, 1);
gnssMeasurements = position + gnssNoiseStd * randn(numMeasurements, 3);
gnss_freq = 0.2; % Rate of GNSS messages in hz.
gnssSample = gnssMeasurements(1:round(sim_freq/gnss_freq):end, :);


% Generate noisy IMU measurements of the trajectory.
% ...  tbd

% Plot the trajectory.
figure(1);clf;axis equal;hold on;
scatter(waypoints(:, 1), waypoints(:, 2), 'b');
plot(position(:, 1), position(:, 2), 'g--', 'LineWidth', 1.5);
scatter(gnssSample(:, 1), gnssSample(:,2), "r")
xlabel('X Position');
ylabel('Y Position');
title('Rover Trajectory');
legend('Waypoints', 'Trajectory', 'GNSS');
grid on;