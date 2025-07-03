clc;
clear;
close all;

% === Parameters ===
scene_id = 101;              % Dynamic obstacle map ID
num_steps = 10;              % Number of dynamic steps
sampleTime = 0.05;           % Simulation time step
plotting = 0;                % Disable tree plotting

initial_pose = [2, 2];       % AGV initial position
current_position = initial_pose;

% Use stronger settings for dynamic maps
memory_allocation = 5000;    % Large tree size for narrow passage
steering_resolution = 0.1;   % Finer local steering
max_retry = 3;               % Retry times per dynamic frame

% Initialize display
figure;
axis equal;
hold on;

% Accumulate results
total_distance = 0;
total_time = 0;
final_trajectory = current_position;

goal_reached_flag = false;

% === Main dynamic loop ===
for step = 0:num_steps
    fprintf('Step %d: Dynamic obstacle moving...\n', step);

    % Create scene with moving obstacle
    environment = createScene(scene_id, false, step);
    environment.start = current_position;

    % Early termination if close enough
    if norm(current_position - environment.goal) < 3
        fprintf('✅ AGV reached goal vicinity at step %d.\n', step);
        goal_reached_flag = true;
        break;
    end

    % Initialize RRT with dynamic settings
    rrt = RRT(environment, ...
        'memory_allocation', memory_allocation, ...
        'steering_resolution', steering_resolution);

    goal_reached = false;

    for retry = 1:max_retry
        goal_reached = rrt.solve(plotting);
        if goal_reached
            break;
        end
    end

    % Plot current tree + path
    clf;
    environment.plot;
    hold on;

    if goal_reached
        path_indices = rrt.reconstructPath();
        positions = vertcat(rrt.nodes(path_indices).position);
        plot(positions(:,1), positions(:,2), 'r-', 'LineWidth', 2);

        % Simulate AGV partial motion
        [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime);
        step_distance = norm(next_position - current_position);

        total_distance = total_distance + step_distance;
        total_time = total_time + step_time;
        current_position = next_position;
        final_trajectory = [final_trajectory; current_position];

        title(sprintf('Step %d: AGV moved to (%.2f, %.2f)', step, current_position(1), current_position(2)));
    else
        warning('❌ No path found at step %d. Terminating.', step);
        goal_reached_flag = false;
        break;
    end

    drawnow;
    pause(0.05);
end

% === Final results ===
fprintf('\n=== AGV Dynamic Single Run Summary ===\n');
fprintf('Total time used:      %.2f s\n', total_time);
fprintf('Total distance moved: %.2f units\n', total_distance);
fprintf('Final position:       (%.2f, %.2f)\n', current_position(1), current_position(2));

% === Show full trajectory playback ===
if size(final_trajectory, 1) > 1
    figure;
    environment = createScene(scene_id, false, step);  % Last frame
    environment.plot;
    hold on;
    simulatePathFollowing(final_trajectory);
    title(sprintf('AGV Final Trajectory Playback - Scene %d', scene_id));
    axis equal;
    xlim([0 30]); ylim([0 30]);
else
    warning('No valid trajectory to playback.');
end
