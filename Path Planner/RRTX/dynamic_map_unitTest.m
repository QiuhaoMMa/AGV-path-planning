clc;
clear;
close all;

scene_id = 100;              % Dynamic obstacle map ID
num_steps = 10;              % Number of dynamic steps
sampleTime = 0.05;           % Time step for simulation
plotting = 0;                % Turn off RRT plotting for speed

initial_pose = [2, 2];       % Starting position
current_position = initial_pose;

% Initialize figure
figure;
axis equal;
hold on;

% Accumulators
total_distance = 0;
total_time = 0;

for step = 0:num_steps
    fprintf('Step %d: Dynamic obstacle moving to the right...\n', step);
    
    % Update environment with moving obstacle and current AGV location
    environment = createScene(scene_id, false, step);
    environment.start = current_position;

    % If AGV is already close to goal, stop early
    if norm(current_position - environment.goal) < 3
        fprintf('✅ AGV reached within 3 units of goal at step %d.\n', step);
        break;
    end

    % Replan with RRT
    rrt = RRT(environment);
    goal_reached = rrt.solve(plotting);

    clf;
    environment.plot;
    hold on;

    if goal_reached
        % Reconstruct and visualize path
        path_indices = rrt.reconstructPath();
        positions = vertcat(rrt.nodes(path_indices).position);
        plot(positions(:,1), positions(:,2), 'r-', 'LineWidth', 2);

        % Simulate motion and update AGV state
        [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime);
        step_distance = norm(next_position - current_position);

        current_position = next_position;
        total_distance = total_distance + step_distance;
        total_time = total_time + step_time;

        title(sprintf('Step %d: AGV moved to (%.2f, %.2f)', step, current_position(1), current_position(2)));
    else
        % Abort simulation if no path found
        warning('❌ Path not found at step %d. Terminating simulation.', step);
        fprintf('\nSimulation failed due to unreachable goal.\n');
        return;
    end
    
    drawnow;
    pause(0.5);
end

% Final summary
fprintf('\n=== AGV Dynamic Navigation Summary ===\n');
fprintf('Total time taken:     %.2f s\n', total_time);
fprintf('Total distance moved: %.2f units\n', total_distance);
fprintf('Final position:       (%.2f, %.2f)\n', current_position(1), current_position(2));
