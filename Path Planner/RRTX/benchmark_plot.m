% Create a custom plot for each metric across all methods and scenes
close all;
figure;

% Colors for each method (using parula for smooth transitions)
colors = parula(length(methods_names))*0.7;  % Use a visually pleasing colormap

% Predefine marker styles for better differentiation
marker_styles = {'o', 's', 'd', '^', 'v', 'p', 'h'};  % Different markers for each method

% Preallocate handles for the legend
h1 = []; h2 = [];

% Define opacity levels based on the number of scenes
opacities = linspace(1, 0.3, numel(scene_ids));

% Loop through each scene and method
for scene_id = scene_ids
    for method_idx = 1:length(methods_names)
        method_data = results.(methods_names{method_idx}).statistics(:, :, scene_id);
        means = method_data(1, :);
        stds = method_data(2, :);
        
        % Set plot options for color, transparency, and marker style
        plot_options = {
            'Color', [colors(method_idx, :) opacities(scene_id)], ... % Use color with opacity
            'LineWidth', scene_id*2, ...  % Line width
            'MarkerEdgeColor', colors(method_idx, :), ...  % Marker edge color
            'MarkerFaceColor', colors(method_idx, :)
        };

        % --- First Subplot: Path Length vs Computation Time ---
        subplot(1, 2, 1);
        hold on;
        % Plot Path Length (x) vs Computation Time (y) with error bars
        h1(method_idx, :) = plotBarXY(means(2), means(1), stds(1), 'v', marker_styles{mod(method_idx-1, length(marker_styles))+1} , plot_options{:});
        plotBarXY(means(1), means(2), stds(2), 'h', marker_styles{mod(method_idx-1, length(marker_styles))+1} , plot_options{:});

        % --- Second Subplot: Iterations vs Computation Time ---
        subplot(1, 2, 2);
        hold on;
        % Plot Iterations (x) vs Computation Time (y) with error bars
        h2(method_idx, :) = plotBarXY(means(3), means(1), stds(1), 'v', marker_styles{mod(method_idx-1, length(marker_styles))+1}, plot_options{:});
        plotBarXY(means(1), means(3), stds(3), 'h', marker_styles{mod(method_idx-1, length(marker_styles))+1}, plot_options{:});
    end
end

% Customize first subplot (Path Length vs Computation Time)
subplot(1, 2, 1);
ylabel('Computation Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Path Length', 'FontSize', 12, 'FontWeight', 'bold');
title('Path Length vs Computation Time', 'FontSize', 14, 'FontWeight', 'bold');
legend(h1(:, end), methods_names, 'Location', 'northeast', 'FontSize', 16);  % Place legend outside
grid on;  % Enable grid for better readability
% set(gca, 'YScale', 'log')
% set(gca, 'XScale', 'log')


% Customize second subplot (Iterations vs Computation Time)
subplot(1, 2, 2);
ylabel('Computation Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Number of Iterations', 'FontSize', 12, 'FontWeight', 'bold');
title('Iterations vs Computation Time', 'FontSize', 14, 'FontWeight', 'bold');
legend(h2(:, end), methods_names, 'Location', 'northeast', 'FontSize', 16);  % Place legend outside
grid on;  % Enable grid for better readability
% set(gca, 'YScale', 'log')
% set(gca, 'XScale', 'log')

% Set the figure size to accommodate both subplots and the legends
set(gcf, 'Position', [100, 100, 1400, 600]);

