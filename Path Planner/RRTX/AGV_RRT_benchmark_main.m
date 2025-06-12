clc;
clear;
close all;

num_test = 10;                % Number of trials per map
scene_ids = 34:39;            % Map IDs to test
plotting = 0;                 % Set to 1 to enable plotting

% Initialize result storage
results.RRT.data = zeros(length(scene_ids), num_test, 2); % [time, path_length]
results.RRT.success = zeros(length(scene_ids), num_test); % Success flags

% Test all specified maps
for s = 1:length(scene_ids)
    scene_id = scene_ids(s);
    
    for i = 1:num_test
        fprintf('Scene %d, Trial %d\n', scene_id, i);
        
        try
            environment = createScene(scene_id, false);  % Create map environment
            rrt = RRT(environment);                      % Initialize RRT planner
            
            goal_reached = rrt.solve(plotting);          % Solve the planning problem
            
            if goal_reached
                path_indices = rrt.reconstructPath();
                positions = vertcat(rrt.nodes(path_indices).position);
                path_length = sum(vecnorm(diff(positions), 2, 2));
                results.RRT.data(s, i, :) = [rrt.computation_time, path_length];
                results.RRT.success(s, i) = 1;
            else
                results.RRT.data(s, i, :) = [rrt.computation_time, NaN];  % NaN for failed path
                results.RRT.success(s, i) = 0;
            end
            
        catch ME
            warning('Scene %d, Trial %d failed: %s', scene_id, i, ME.message);
            results.RRT.data(s, i, :) = [NaN, NaN];
            results.RRT.success(s, i) = 0;
        end
    end
end

% Statistics
mean_time = mean(results.RRT.data(:, :, 1), 2, 'omitnan');
std_time = std(results.RRT.data(:, :, 1), 0, 2, 'omitnan');
mean_length = mean(results.RRT.data(:, :, 2), 2, 'omitnan');
std_length = std(results.RRT.data(:, :, 2), 0, 2, 'omitnan');
success_rate = sum(results.RRT.success, 2) / num_test;

% Grouped result display
fprintf('\n===== RRT Benchmark Summary (Grouped by Layout) =====\n');

layout_names = {'Manufacturing Cell Layout', 'Amazon Warehouse Layout'};
layout_ranges = {[34, 35, 36], [37, 38, 39]};

for l = 1:2
    range = layout_ranges{l};
    fprintf('\n--- %s ---\n', layout_names{l});
    fprintf('Scene\tSuccess\tTime (mean±std)\tPath Length (mean±std)\n');
    
    for scene_id = range
        idx = scene_id - 33;  % Offset for indexing
        fprintf('%2d\t%.2f\t%.2f ± %.2f\t%.2f ± %.2f\n', ...
            scene_id, success_rate(idx), ...
            mean_time(idx), std_time(idx), ...
            mean_length(idx), std_length(idx));
    end
end

% Save workspace results
save('rrt_34to39_results.mat', 'results', 'mean_time', 'std_time', 'mean_length', 'std_length', 'success_rate');
