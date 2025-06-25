clc;
clear;
close all;

scene_id = 100;              % Dynamic obstacle map ID
num_steps = 10;              % Number of dynamic steps
sampleTime = 0.05;           % Simulation time step
plotting = 0;                % Disable RRT tree plotting

initial_pose = [2, 2];       % Starting position of AGV
current_position = initial_pose;

% Initialize display
figure;
axis equal;
hold on;

% Accumulators
total_distance = 0;
total_time = 0;
final_trajectory = current_position;  % To record all AGV positions

for step = 0:num_steps
    fprintf('Step %d: Dynamic obstacle moving...\n', step);

    % Generate scene with dynamic obstacles
    environment = createScene(scene_id, false, step);
    environment.start = current_position;

    % Check if AGV is within 3 units of goal
    if norm(current_position - environment.goal) < 3
        fprintf('✅ AGV reached goal vicinity at step %d.\n', step);
        break;
    end

    % Run RRT
    rrt = RRT(environment);
    goal_reached = rrt.solve(plotting);

    % Visualize environment + tree + path
    clf;
    environment.plot;
    hold on;

    if goal_reached
        path_indices = rrt.reconstructPath();
        positions = vertcat(rrt.nodes(path_indices).position);
        plot(positions(:,1), positions(:,2), 'r-', 'LineWidth', 2);

        % Simulate AGV motion and update state
        [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime);
        step_distance = norm(next_position - current_position);

        % Accumulate metrics
        total_distance = total_distance + step_distance;
        total_time = total_time + step_time;
        current_position = next_position;
        final_trajectory = [final_trajectory; current_position];  % Record trajectory

        title(sprintf('Step %d: AGV moved to (%.2f, %.2f)', step, current_position(1), current_position(2)));
    else
        warning('❌ No path found at step %d. Terminating simulation.', step);
        fprintf('\nSimulation failed due to unreachable goal.\n');
        return;
    end

    drawnow;
    pause(0.5);
end

% === Summary output ===
fprintf('\n=== AGV Dynamic Navigation Summary ===\n');
fprintf('Total time taken:     %.2f seconds\n', total_time);
fprintf('Total distance moved: %.2f units\n', total_distance);
fprintf('Final position:       (%.2f, %.2f)\n', current_position(1), current_position(2));

% === Final trajectory simulation ===
if size(final_trajectory, 1) > 1
    figure;
    environment = createScene(scene_id, false, step);  % Last frame
    environment.plot;
    hold on;
    simulatePathFollowing(final_trajectory);  % Full trajectory playback
    title('AGV Final Full Trajectory Playback');
else
    warning('Final trajectory is empty or invalid.');
end
