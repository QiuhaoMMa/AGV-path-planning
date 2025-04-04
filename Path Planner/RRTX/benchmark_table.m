%% 
clc
% Assuming the `results` structure is already populated with statistics
% data from the previous runs.


metrics = {'Computation Time (s)', 'Path Length', 'Iterations'};

% Pre-allocate a cell array for the table
table_data = cell(length(scene_ids) * length(methods_names), 4);  % 4 columns: Scene, Method, Mean, Std

% Fill table_data with the statistics
row = 1;
for scene_id = scene_ids
    for method_name = methods_names(1:numel(methods))
        method_data = results.(method_name{1}).statistics(:,:,scene_id);
        table_data{row, 1} = scene_id;  % Scene ID
        table_data{row, 2} = method_name{1};  % Method
        table_data{row, 3} = sprintf('%.3f | %.2f | %.0f', method_data(1,:));  % Mean (Time, Path, Iterations)
        table_data{row, 4} = sprintf('%.3f | %.2f | %.0f', method_data(2,:));  % Std (Time, Path, Iterations)
        row = row + 1;
    end
end

% Convert to a table and display
table_output = cell2table(table_data, 'VariableNames', {'Scene', 'Method', 'Mean [Time (sec) | Path | Iter]', 'Std [Time | Path | Iter]'});
disp(table_output);
