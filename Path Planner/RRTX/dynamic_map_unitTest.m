clc;
clear;
close all;

scene_id = 102;              % Dynamic obstacle map ID
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


%%
clc;
clear;
close all;

save_path = 'C:\Users\97895\Desktop\AGV_RRT_maps';

if ~exist(save_path, 'dir')
    mkdir(save_path);
end

% ==== Parameters ====
scene_ids = 100:102;   % Dynamic maps: easy/medium/hard
num_steps = 10;        % How many dynamic obstacle frames
num_repeat = 10;       % Repeat runs for stats
sampleTime = 0.05;

% ==== Loop over each scene ====
for scene_id = scene_ids

    fprintf('\n=== Running Dynamic Test for Scene %d ===\n', scene_id);

    for run_idx = 1:num_repeat
        fprintf(' --- Repeat %d/%d --- \n', run_idx, num_repeat);

        % Initialize AGV state
        initial_pose = [2, 2];
        current_position = initial_pose;
        total_distance = 0;
        total_time = 0;
        final_trajectory = current_position;

        figure;
        hold on;
        axis equal;

        goal_reached_flag = false;

        for step = 0:num_steps

            % Create dynamic environment at this step
            environment = createScene(scene_id, false, step);
            environment.start = current_position;

            % If close enough to goal, finish early
            if norm(current_position - environment.goal) < 3
                fprintf('✅ Reached goal vicinity at step %d.\n', step);
                goal_reached_flag = true;
                break;
            end

            % Solve RRT
            rrt = RRT(environment);
            goal_reached = rrt.solve(0);

            clf;
            environment.plot;
            hold on;

            if goal_reached
                path_indices = rrt.reconstructPath();
                positions = vertcat(rrt.nodes(path_indices).position);
                plot(positions(:,1), positions(:,2), 'r-', 'LineWidth', 2);

                % Simulate AGV partial follow
                [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime);
                step_distance = norm(next_position - current_position);

                current_position = next_position;
                final_trajectory = [final_trajectory; current_position];
                total_distance = total_distance + step_distance;
                total_time = total_time + step_time;

                title(sprintf('Scene %d - Step %d', scene_id, step));
            else
                warning('❌ Path not found at step %d! Terminating.', step);
                goal_reached_flag = false;
                break;
            end

            drawnow;
            pause(0.1);
        end

        % ==== Save final path planning figure (last frame) ====
        filename1 = fullfile(save_path, sprintf('AGV_RRT_PathPlanning_Scene%d_Run%d.png', scene_id, run_idx));
        saveas(gcf, filename1);
        close(gcf);

        % ==== Save final differential drive trajectory ====
        if size(final_trajectory, 1) > 1
            fig2 = figure;
            environment = createScene(scene_id, false, step); % Last dynamic step
            environment.plot;
            hold on;
            simulatePathFollowing(final_trajectory);
            title(sprintf('AGV Full Differential Drive - Scene %d Run %d', scene_id, run_idx));
            axis equal;
            xlim([0 30]); ylim([0 30]);

            filename2 = fullfile(save_path, sprintf('AGV_RRT_DifferentialDrive_Scene%d_Run%d.png', scene_id, run_idx));
            saveas(fig2, filename2);
            close(fig2);
        else
            warning('No final trajectory to plot!');
        end

        % ==== Print summary ====
        if goal_reached_flag
            fprintf('✅ FINAL: Scene %d Run %d SUCCESS - Total Distance: %.2f, Time: %.2f s\n', ...
                scene_id, run_idx, total_distance, total_time);
        else
            fprintf('❌ FINAL: Scene %d Run %d FAILED to reach goal.\n', scene_id, run_idx);
        end
    end
end

fprintf('\n=== All Dynamic Tests Completed ===\n');
