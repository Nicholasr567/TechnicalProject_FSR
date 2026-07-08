%% CAR-LIKE ROBOT: Planning, Control & Localization
clear; clc; close all;

%% Parameters
global length_step radius_curve num_iterations points counter occupancy_matrix map sample_points connections curve_points legend_flag
l = 0.6;
Delta = 0.2;
Ts = 0.01;

%% Map Processing
addpath(genpath("Images"));
img = imread('mia_mappa_ruotata.png'); 

% Image conversion from RGB to grey
if size(img, 3) == 3
    img_gray = rgb2gray(img);
else
    img_gray = img;
end

% Binarization 
% Roads =1, Obstacles=0
BW = imbinarize(img_gray);

% To invert colors in case the map load had roads darker than obstacles
BW = ~BW;

% Transponse matrix to align it at the coordinates (x,y)
map = BW'; 

length_step = 7;            % Length of the step forward
radius_curve = 7;           % Bending radius for curves
num_iterations = 500;       % Number of iterations

% =========================================================================
% MAIN LOOP
% =========================================================================
run_full_simulation = true;
while run_full_simulation

    %% Start and goal point coordinates for the maps
    %  Manual Point Selection
    figure('Name', 'Map Selection'); 
    imshow(map'); 
    title('\fontsize{14}\color{red}Click START point, then GOAL point');
    [x_click, y_click] = ginput(2);
    close;

    % Assign clicked values to te variables
    Qs = [round(x_click(1)), round(y_click(1))];
    Qg = [round(x_click(2)), round(y_click(2))];

    % Print Start and Goal Points 
    disp(['START point set to: ', num2str(Qs)]);
    disp(['GOAL point set to: ', num2str(Qg)]);

    % Check obstacles
    if map(Qs(1), Qs(2)) == 0
        error('ERROR: START point is on an obstacle! Restart and click on a white area.');
    end
    if map(Qg(1), Qg(2)) == 0
        error('ERROR: GOAL point is on an obstacle! Restart and click on a white area.');
    end

    % Variable initialization (Reset for each new simulation)
    legend_flag = 1;           
    curve_points = struct('start_node', [], 'end_node', [], 'points', {});
    sample_points = [];
    connections = [];

    % Vector for points
    points = zeros(((num_iterations + 1)^3), 2);
    points(1, :) = Qs;

    % Occupancy matrix
    occupancy_matrix = zeros(size(map));
    occupancy_matrix(Qs(1), Qs(2)) = 1;         % Mark start point as visited
    occupancy_matrix(map == 0) = 1;             % Mark cells with obstacles as visited

    % Point counter
    counter = 1;

    % Queue for points to explore
    queue = [Qs, 0, 1];                         % [x, y, angle, iteration]

    % Plot initialization
    figure('Name', 'Path Planning');
    imshow(map');
    hold on;
    % Plotting the start and goal points on the map
    plot(Qs(1), Qs(2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);                      % Start point (red)
    plot(Qg(1), Qg(2), 'go', 'MarkerSize', 10, 'LineWidth', 1.5);                      % Goal point (green)

    %% Graph Generation via RSC.
    disp('Searching for path, please wait...');
    while ~isempty(queue)
        % Extract the current point from the queue
        current_point = queue(1, 1:2);
        current_angle = queue(1, 3);
        iteration = queue(1, 4);
        queue(1, :) = [];                   % Remove the item from the queue

        % Calculate new points and add them to the queue if not visited
        new_points = calculate_points(current_point, current_angle, iteration);
        queue = [queue; new_points];
    end
    disp('Graph generated.');

    % notes on the plot figure
    title('\textbf{Primitive Reed-Shepp Curves Iterative}', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('x [m]', 'Interpreter', 'latex');
    ylabel('y [m]', 'Interpreter', 'latex');
    axis equal;
    grid on;

    % Calculates the adjacency matrix, in input the nodes and a vector indicating the connections between the nodes
    sample_points = [Qs; sample_points];
    adjacency_matrix = create_adjacency_matrix(sample_points, connections);

    %% Optimal Path Search via Dijkstra
    disp('Running Dijkstra algorithm...');
    shortest_path = dijkstra(adjacency_matrix, sample_points, Qs, Qg);
    disp('Path found!');

    % Plots the previously obtained path
    collected_points = plot_path(shortest_path, sample_points, 'm');

    %% Reference Trajectory generation
    [xd, yd, thetad, phid, vd, omegad, q0, T_max, k, landmarks, v_max_ref, omega_max_ref] = des_trajectory(collected_points);

    q_goal_finale = [xd.Data(end); yd.Data(end); thetad.Data(end)];

    % --- Final Point (y_1d_goal, y_2d_goal) ---
    xd_end    = xd.Data(end);
    yd_end    = yd.Data(end);
    thetad_end = thetad.Data(end);
    phid_end   = phid.Data(end);

    y_1d_goal = xd_end + l*cos(thetad_end) + Delta*cos(thetad_end + phid_end);
    y_2d_goal = yd_end + l*sin(thetad_end) + Delta*sin(thetad_end + phid_end);

    % --- EKF and LIDAR Configuration ---
    P0 = eye(4) * 0.1; % Incertezza iniziale dello stato
    Q_ekf = diag([0.01, 0.01]); % Incertezza odometria (v, omega)
    R_ekf = diag([0.05, 0.02]); % Incertezza sensore LIDAR (distanza, angolo)

    qd = timeseries([xd.Data(:), yd.Data(:), thetad.Data(:), phid.Data(:)], xd.Time);
    velocities_d = timeseries([vd.Data(:), omegad.Data(:)], xd.Time);

    % =========================================================================
    % Secondary Loop: same simulation, different method
    % =========================================================================
    run_same_path = true;
    while run_same_path
        
        % =========================================================================
        %  --- SIMULATION MENU & MANUAL WORKFLOW ---
        %  =========================================================================

        mode_list = {'Nominal (Ideal Feedback)', ...
                     'Odometry (Euler)', ...
                     'Odometry (Runge-Kutta 2)', ...
                     'Active Localization (EKF)'};
                 
        choice = menu('Choose Localization Method:', mode_list{:});

        if choice == 0
            disp('Simulation aborted by user.');
            run_same_path = false;
            run_full_simulation = false;
            break;
        end

        % Nomi per le cartelle corrispondenti ai metodi
        method_folder_names = {'Nominal', 'Eulero', 'RungeKutta', 'EKF'};
        current_method_suffix = method_folder_names{choice};

        % Set localization variable for Simulink Multiport Switch
        localization = choice - 1; 
        current_mode_name = mode_list{choice};

        disp(' ');
        disp(['Selected Localization Mode: ', current_mode_name]);
        disp('======================================================');
        disp(' MANUAL SIMULATION WORKFLOW:');
        disp(' 1. Open your Simulink model (CarLike_Robot.slx).');
        disp(' 2. Press the RUN button in Simulink.');
        disp(' 3. Wait for the simulation to finish completely.');
        disp(' 4. Return to this MATLAB window and press ENTER to generate plots.');
        disp('======================================================');

        pause;

        disp('Simulation completed. Generating plots...');

        % =========================================================================
        %  --- POST-SIMULATION PLOTS ---
        %  =========================================================================

        % Extracting Data from Workspace variable 'out'
        t_sim   = out.x.Time;
        act_x   = out.x.Data;
        act_y   = out.y.Data;
        act_th  = out.theta.Data;
        act_phi = out.phi.Data;

        des_x   = out.x_d.Data;
        des_y   = out.y_d.Data;
        des_th  = out.theta_d.Data;
        des_phi = out.phi_d.Data;

        cmd_v     = out.v.Data;
        cmd_omega = out.omega.Data;

        % Calculating Errors
        err_x   = des_x - act_x;
        err_y   = des_y - act_y;
        err_th  = des_th - act_th;
        err_th  = atan2(sin(err_th), cos(err_th)); % Normalize angle error [-pi, pi]
        err_phi = des_phi - act_phi;

        % Tracking Error (Euclidean Distance)
        err_dist = sqrt(err_x.^2 + err_y.^2);

        % --- PLOT: STATE ERRORS ---
        fig_err_x = figure('Name', ['Error X - ', current_mode_name], 'Color', 'w');
        plot(t_sim, err_x, 'r', 'LineWidth', 1.5);
        title(['\textbf{Error on } $x$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$e_x$ [m]', 'Interpreter', 'latex'); 
        grid on;

        fig_err_y = figure('Name', ['Error Y - ', current_mode_name], 'Color', 'w');
        plot(t_sim, err_y, 'g', 'LineWidth', 1.5);
        title(['\textbf{Error on } $y$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$e_y$ [m]', 'Interpreter', 'latex'); 
        grid on;

        fig_err_th = figure('Name', ['Error Theta - ', current_mode_name], 'Color', 'w');
        plot(t_sim, err_th, 'b', 'LineWidth', 1.5);
        title(['\textbf{Error on } $\theta$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$e_\theta$ [rad]', 'Interpreter', 'latex'); 
        grid on;

        fig_err_phi = figure('Name', ['Error Phi - ', current_mode_name], 'Color', 'w');
        plot(t_sim, err_phi, 'k', 'LineWidth', 1.5);
        title(['\textbf{Error on } $\phi$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$e_\phi$ [rad]', 'Interpreter', 'latex'); 
        grid on;

        % --- PLOT: CONTROL INPUTS ---
        crop_idx = t_sim > 0;
        t_sim_crop = t_sim(crop_idx);
        cmd_v_crop = cmd_v(crop_idx);
        cmd_omega_crop = cmd_omega(crop_idx);

        fig_v = figure('Name', ['Control Input v - ', current_mode_name], 'Color', 'w');
        plot(t_sim_crop, cmd_v_crop, 'b', 'LineWidth', 1.5);
        title(['\textbf{Computed Linear Velocity } $v$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$v$ [m/s]', 'Interpreter', 'latex'); 
        grid on;

        fig_omega = figure('Name', ['Control Input omega - ', current_mode_name], 'Color', 'w');
        plot(t_sim_crop, cmd_omega_crop, 'r', 'LineWidth', 1.5);
        title(['\textbf{Computed Steering Angular Velocity } $\omega$ \textbf{ (', current_mode_name, ')}'], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$\omega$ [rad/s]', 'Interpreter', 'latex'); 
        grid on;

        % --- PLOT: TRACKING ERROR (Euclidean Distance) ---
        fig_err_dist = figure('Name', ['Tracking Error - ', current_mode_name], 'Color', 'w');
        plot(t_sim, err_dist, 'm', 'LineWidth', 1.5);
        title(['\textbf{Path Tracking Error (Euclidean Distance) - }', current_mode_name], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('Time [s]', 'Interpreter', 'latex');
        ylabel('$e_{dist} = \sqrt{e_x^2 + e_y^2}$ [m]', 'Interpreter', 'latex');
        grid on;

        % --- PLOT: XY TRAJECTORY COMPARISON ---
        fig_traj = figure('Name', ['Trajectory Comparison - ', current_mode_name], 'Color', 'w');
        plot(des_x, des_y, 'g--', 'LineWidth', 2); hold on;
        plot(act_x, act_y, 'b', 'LineWidth', 1.5);
        plot(des_x(1), des_y(1), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        plot(des_x(end), des_y(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        title(['\textbf{Trajectory Tracking: }', current_mode_name], 'Interpreter', 'latex', 'FontSize', 14);
        xlabel('$x$ [m]', 'Interpreter', 'latex');
        ylabel('$y$ [m]', 'Interpreter', 'latex');
        legend({'Reference $(x_d, y_d)$', 'Actual $(x, y)$', 'Start', 'Goal'}, 'Interpreter', 'latex', 'Location', 'best');
        axis equal;
        grid on;

        % =========================================================================
        %  --- SALVATAGGIO PDF DEI GRAFICI ---
        %  =========================================================================
        
        post_sim_figs = [fig_err_x, fig_err_y, fig_err_th, fig_err_phi, fig_v, fig_omega, fig_err_dist, fig_traj];
        base_names = {'Error_X', 'Error_Y', 'Error_Theta', 'Error_Phi', 'Control_v', 'Control_omega', 'Tracking_Error', 'Trajectory_Comparison'};

        save_folder = fullfile('images', current_method_suffix);
        if ~exist(save_folder, 'dir')
            mkdir(save_folder);
        end

        disp(['Salvataggio dei grafici in formato PDF nella cartella: ', save_folder, ' ...']);
        for i_fig = 1:length(post_sim_figs)
            pdf_filename = fullfile(save_folder, sprintf('%s_%s.pdf', base_names{i_fig}, current_method_suffix));
            try
                exportgraphics(post_sim_figs(i_fig), pdf_filename, 'ContentType', 'vector');
            catch
                % Fallback per versioni di MATLAB precedenti alla R2020a
                set(post_sim_figs(i_fig), 'Units', 'Inches');
                pos = get(post_sim_figs(i_fig), 'Position');
                set(post_sim_figs(i_fig), 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)]);
                print(post_sim_figs(i_fig), pdf_filename, '-dpdf', '-r0');
            end
        end
        disp('Salvataggio completato con successo.');

        % =========================================================================
        %  --- POST-SIMULATION USER MENU ---
        %  =========================================================================

        user_choice = menu('Simulation Completed. what s next?', ...
            'New Simulation (Choose new point on the map)', ...
            'Same simulation, change localization method', ...
            'End');

        if user_choice == 1
            run_same_path = false; 
            close all; 
        elseif user_choice == 2
            try close(post_sim_figs); catch; end 
        else
            run_same_path = false;
            run_full_simulation = false;
        end

    end % End of While run_same_path
end % End of While run_full_simulation

%% FUNCTIONS ===============================================================

% Graph generation via RSC method.
function new_points = calculate_points(current_point, current_angle, iteration)
    % Global variables
    global length_step radius_curve num_iterations points counter occupancy_matrix sample_points connections curve_points legend_flag

    % Initialization of the new points
    new_points = [];

    % Condition of termination of recursion
    if iteration > num_iterations
        return;
    end

    % Calculation of the next point forward
    point_forward = round(current_point + length_step * [cos(current_angle), sin(current_angle)]);
    end_point_forward = point_forward;

    % Center of the circumference for the right turn
    center_right = current_point + radius_curve * [cos(current_angle - pi/2), sin(current_angle - pi/2)];

    % Center of circumference for left turn
    center_left = current_point + radius_curve * [cos(current_angle + pi/2), sin(current_angle + pi/2)];

    % Angles to generate quarter circles
    theta_right = linspace(current_angle + pi/2, current_angle, 1001);
    theta_left  = linspace(current_angle - pi/2, current_angle, 1001);

    % Calculation of the next point on the right
    x_right = center_right(1) + radius_curve * cos(theta_right);
    y_right = center_right(2) + radius_curve * sin(theta_right);
    end_point_right = round([x_right(end), y_right(end)]);

    % Calculation of the next point on the left
    x_left = center_left(1) + radius_curve * cos(theta_left);
    y_left = center_left(2) + radius_curve * sin(theta_left);
    end_point_left = round([x_left(end), y_left(end)]);

    % Adding points and curves to the plot only if they do not match previous points
    hold on;

    if is_within_bounds(end_point_forward) && ~is_point_visited(end_point_forward)
        x_points = round(linspace(current_point(1), end_point_forward(1), 300));
        y_points = round(linspace(current_point(2), end_point_forward(2), 300));
        if ~is_collision_forward(current_point, end_point_forward)
            counter = counter + 1;
            points(counter, :) = end_point_forward;
            sample_points = [sample_points; end_point_forward];
            occupancy_matrix(x_points, y_points) = 1;
            connections = [connections; current_point, end_point_forward];
            plot([current_point(1), end_point_forward(1)], [current_point(2), end_point_forward(2)], 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
            plot(end_point_forward(1), end_point_forward(2), 'b.', 'MarkerSize', 10, 'HandleVisibility','off');
            new_points = [new_points; end_point_forward, current_angle, iteration + 1];
            % In order to add to legend 
            if legend_flag == 1
                plot(end_point_forward(1), end_point_forward(2), 'b.', 'MarkerSize', 10, 'DisplayName', 'Nodes');
                plot([current_point(1), end_point_forward(1)], [current_point(2), end_point_forward(2)], 'k-', 'LineWidth', 0.5, 'DisplayName', 'RSC Graph');
                legend_flag = 0;
            end
        end
    end

    if is_within_bounds(end_point_right) && ~is_point_visited(end_point_right)
        if ~is_collision_curve(x_right, y_right)
            counter = counter + 1;
            points(counter, :) = end_point_right;
            sample_points = [sample_points; end_point_right];
            occupancy_matrix(round(x_right), round(y_right)) = 1;
            connections = [connections; current_point, end_point_right];
            plot(x_right, y_right, 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
            plot(x_right(end), y_right(end), 'b.', 'MarkerSize', 10, 'HandleVisibility','off');
            new_points = [new_points; end_point_right, current_angle - pi/2, iteration + 1];
        
            % Save the right corner points
            curve_points(end+1).start_node = current_point;
            curve_points(end).end_node = end_point_right;
            curve_points(end).points = [x_right', y_right'];
        end
    end

    if is_within_bounds(end_point_left) && ~is_point_visited(end_point_left)
        if ~is_collision_curve(x_left, y_left) 
            counter = counter + 1;
            points(counter, :) = end_point_left;
            sample_points = [sample_points; end_point_left];
            occupancy_matrix(round(x_left), round(y_left)) = 1;
            connections = [connections; current_point, end_point_left];
            plot(x_left, y_left, 'k', 'LineWidth', 0.5, 'HandleVisibility','off');
            plot(x_left(end), y_left(end), 'b.', 'MarkerSize', 10, 'HandleVisibility','off');
            new_points = [new_points; end_point_left, current_angle + pi/2, iteration + 1];
            
            % Save the left corner points
            curve_points(end+1).start_node = current_point;
            curve_points(end).end_node = end_point_left;
            curve_points(end).points = [x_left', y_left'];
        end
    end
end

% Check if a point has already been visited
function visited = is_point_visited(point)
    global occupancy_matrix
    visited = false;
    if occupancy_matrix(point(1), point(2)) == 1
        visited = true;
    end
end

% Check if a point is within the limits of the matrix
function within_bounds = is_within_bounds(point)
    global occupancy_matrix
    within_bounds = point(1) > 0 && point(2) > 0 && point(1) <= size(occupancy_matrix, 1) && point(2) <= size(occupancy_matrix, 2);
end

% Check if the generated curves have collisions with obstacles (forward)
function collision_forward = is_collision_forward(current_point, end_point_forward)
    global map
    collision_forward = false;
    % Calculate points equidistant between start and end
    x_points = round(linspace(current_point(1), end_point_forward(1), 300));
    y_points = round(linspace(current_point(2), end_point_forward(2), 300));
    for i = 2:(length(x_points)-1)
        if map(x_points(i), y_points(i)) == 0
            collision_forward = true;
            break
        end
    end
end

% Check if the generated curves have collisions with obstacles (curves)
function collision_curve = is_collision_curve(x, y)
    global map
    collision_curve = false;
    for i = 2:(length(x)-1)
        if map(round(x(i)), round(y(i))) == 0
            collision_curve = true;
            break
        end
    end
end

% Ajacency Matrix Computation
function adjacency_matrix = create_adjacency_matrix(nodes, connections)
    num_nodes = size(nodes, 1);
    adjacency_matrix = zeros(num_nodes);
    for i = 1:size(connections, 1)
        point_1 = connections(i, 1:2);
        point_2 = connections(i, 3:4);
        
        idx_1 = find(ismember(nodes, point_1, 'rows'));
        idx_2 = find(ismember(nodes, point_2, 'rows'));
        
        if ~isempty(idx_1) && ~isempty(idx_2)
            adjacency_matrix(idx_1, idx_2) = 1;
            adjacency_matrix(idx_2, idx_1) = 1;
        end
    end
end

% Path Search via Dijkstra
function path = dijkstra(adjacency_matrix, sample_points, Qs, Qg)
    
    start_node = find(ismember(sample_points, Qs, 'rows'), 1);
    if isempty(start_node)
        start_node = 1;
    end
    
    distances = sqrt((sample_points(:,1) - Qg(1)).^2 + (sample_points(:,2) - Qg(2)).^2);
    [min_dist, goal_node] = min(distances);
    
    if min_dist > 25
        error('GOAL Point too far from map.');
    end
    
    num_nodes = size(adjacency_matrix, 1);
    unvisited = 1:num_nodes;
    dist = inf(1, num_nodes);
    prev = nan(1, num_nodes);
    dist(start_node) = 0;
    
    while ~isempty(unvisited)
        [min_d, idx] = min(dist(unvisited));
        current = unvisited(idx);
        
        if isinf(min_d)
            break; 
        end
        if current == goal_node
            break;
        end
        
        unvisited(idx) = [];
        neighbors = find(adjacency_matrix(current, :));
        
        for neighbor = neighbors
            alt = dist(current) + adjacency_matrix(current, neighbor);
            if alt < dist(neighbor)
                dist(neighbor) = alt;
                prev(neighbor) = current;
            end
        end
    end
    
    if isinf(dist(goal_node)) || isnan(prev(goal_node))
        error('Path not found.');
    end
    
    path = [];
    u = goal_node;
    while ~isnan(u)
        path = [u path];
        u = prev(u);
    end
end

% Plot lines along nodes and check if curve
function collected_points = plot_path(path, nodes, color)
    global curve_points;
    collected_points = [];
    for i = 1:length(path)-1
        start_node = path(i);
        end_node = path(i+1);
        
        curve_index = find_curve_index(nodes(start_node,:), nodes(end_node,:));
        if ~isempty(curve_index)
            curve = curve_points(curve_index).points;
            plot(curve(:,1), curve(:,2), 'Color', color, 'LineWidth', 2, 'HandleVisibility','off');
            if isempty(collected_points)
                collected_points = curve;
            else
                collected_points = [collected_points; curve(2:end, :)];
            end
        else
            straight_line_points(:,1) = linspace(nodes(start_node,1), nodes(end_node,1), 1001);
            straight_line_points(:,2) = linspace(nodes(start_node,2), nodes(end_node,2), 1001);
            plot(straight_line_points(:,1), straight_line_points(:,2), 'Color', color, 'LineWidth', 2, 'HandleVisibility','off');
            if isempty(collected_points)
                collected_points = straight_line_points;
            else
                collected_points = [collected_points; straight_line_points(2:end, :)];
            end
        end
    end
    plot(straight_line_points(:,1), straight_line_points(:,2), 'Color', color, 'LineWidth', 1.5, 'DisplayName', 'Found path');
    plot(collected_points(1,1), collected_points(1,2), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');                      
    plot(collected_points(end,1), collected_points(end,2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal point');                      
    leg = legend('show');
    set(leg, 'Interpreter','latex', 'fontsize', 9,'Location', 'best')
end

function index = find_curve_index(start_node, end_node)
    global curve_points;
    index = [];
    for i = 1:length(curve_points)
        curve = curve_points(i);
        if (isequal(curve.start_node, start_node) && isequal(curve.end_node, end_node)) || ...
           (isequal(curve.start_node, end_node) && isequal(curve.end_node, start_node))
            index = i;
            break;
        end
    end
end

%% Trajectory computation based on the path
function [xd, yd, thetad, phid, vd, omegad, q0, T_max, k, landmarks, v_max_ref, omega_max_ref] = des_trajectory(collected_points)
    global map
    num_rows_map = size(map,2);
    l = 0.6;
    Ts = 0.01;
    x = collected_points(:, 1);
    y = num_rows_map-collected_points(:, 2);

    % --- 1. Spatial Parametrization Real (ARC-LENGTH) ---
    dx = diff(x);
    dy = diff(y);
    dist = sqrt(dx.^2 + dy.^2);
    arc_lengths = [0; cumsum(dist)];
    S_total = arc_lengths(end); % Total path lenght (metres)
    
    s_sample = arc_lengths / S_total; 
    
    % Remove eventual double points
    [s_sample, unique_idx] = unique(s_sample, 'stable');
    x = x(unique_idx);
    y = y(unique_idx);
    
    s_meters = s_sample * S_total; % Vettore delle distanze reali in metri

    % --- 2. Compute path geometric curvature ---
    ds_steps = gradient(s_meters);
    ds_steps(ds_steps == 0) = 1e-6;
    
    % Derivate prime rispetto allo spazio
    dx_ds = gradient(x) ./ ds_steps;
    dy_ds = gradient(y) ./ ds_steps;
    
    % Derivate seconde rispetto allo spazio
    ddx_ds2 = gradient(dx_ds) ./ ds_steps;
    ddy_ds2 = gradient(dy_ds) ./ ds_steps;
    
    % Curvatura geometrica: kappa = |x'*y'' - y'*x''| / (x'^2 + y'^2)^1.5
    kappa = abs(dx_ds .* ddy_ds2 - dy_ds .* ddx_ds2) ./ (dx_ds.^2 + dy_ds.^2 + 1e-6).^(1.5);

    % --- 3. Velocity Profile Generation ---
    v_straight = 2.5; % Velocità massima nei rettilinei (m/s)
    v_curve = 0.6;    % Velocità ridotta nelle curve (m/s)
    
    % Default: max velocity
    v_target = ones(size(x)) * v_straight;
    
    % Where the geometric curvature is greather than threshodl apply curve
    % velocity
    v_target(kappa > 0.04) = v_curve;
    
    % Start and End Velocity imposed to 0
    v_target(1) = 0;
    v_target(end) = 0;

    % --- 4. Gaussian Filter ---
    window_size = round(length(x) * 0.06);
    v_smooth = smoothdata(v_target, 'gaussian', window_size);

    % ---START/STOP RAMP  ---
    N = length(v_smooth);
    frac_ramp = 0.04;                          % 4% dei punti per ciascuna rampa
    n_ramp = max(round(N * frac_ramp), 5);      % almeno 5 campioni
    n_ramp = min(n_ramp, floor(N/2) - 1);       % non sovrapporre le due rampe

    blend = ones(N, 1);                        % fattore moltiplicativo (1 = nessuna attenuazione)

    % Rampa di accelerazione iniziale: blend va da 0 a 1 con tangente nulla agli estremi
    idx_start = (1:n_ramp)';
    blend(idx_start) = 0.5 * (1 - cos(pi * (idx_start - 1) / (n_ramp - 1)));

    % Rampa di frenata finale: blend va da 1 a 0 con tangente nulla agli estremi
    idx_end = (N-n_ramp+1:N)';
    blend(idx_end) = 0.5 * (1 + cos(pi * (idx_end - (N-n_ramp+1)) / (n_ramp - 1)));

    v_smooth = v_smooth .* blend;

    % Sicurezza anti-divisione-per-zero nella parte centrale
    core_mask = true(N,1);
    core_mask([idx_start; idx_end]) = false;
    v_smooth(core_mask) = max(v_smooth(core_mask), 0.01);

    % Negli ultimissimi campioni delle rampe il blend porta v_smooth verso 0; per evitare
    % dt = ds/v_avg -> infinito, imponiamo un v_min proporzionale alla densità
    % spaziale dei campioni vicino agli estremi, in modo che dt resti dell'ordine di Ts.
    ds_start_local = mean(diff(s_meters(1:min(10,N))));
    ds_end_local   = mean(diff(s_meters(max(1,N-9):N)));
    v_min_start = max(ds_start_local / Ts, 1e-3); % v minima che dà dt ~ Ts vicino al via
    v_min_end   = max(ds_end_local   / Ts, 1e-3); % idem vicino all'arrivo
    v_smooth(idx_start) = max(v_smooth(idx_start), v_min_start);
    v_smooth(idx_end)   = max(v_smooth(idx_end), v_min_end);

    % --- 5. NUMERICAL INTEGRTION FROM SPACE TO TIME ---
    % dt = ds / v
    ds_vec = [0; diff(s_meters)];
    dt = zeros(size(x));
    for i = 2:length(x)
        v_avg = 0.5 * (v_smooth(i-1) + v_smooth(i));
        dt(i) = ds_vec(i) / v_avg;
    end
    t_profile = cumsum(dt);
    tf = t_profile(end); % final time

    % --- 6. Uniform Sampling ---
    T = linspace(0, tf, 10000);
    x_t = interp1(t_profile, x, T, 'pchip');
    y_t = interp1(t_profile, y, T, 'pchip');
    
    
    x_dot = gradient(x_t, T);
    y_dot = gradient(y_t, T);
    x_ddot = gradient(x_dot, T);
    y_ddot = gradient(y_dot, T); 
    x_dddot = gradient(x_ddot, T);
    y_dddot = gradient(y_ddot, T);

    theta = atan2(y_dot, x_dot);
    v = sqrt(x_dot.^2 + y_dot.^2);
    
    v_safe = v;
    v_safe(abs(v_safe) < 1e-4) = 1e-4 * sign(v_safe(abs(v_safe) < 1e-4) + 1e-12);
    
    phi = atan(l * (y_ddot .* x_dot - x_ddot .* y_dot) ./ v_safe.^3);
    omega = l * v .* ((y_dddot .* x_dot - x_dddot .* y_dot) .* v_safe.^2 - 3 * ...
        (y_ddot .* x_dot - x_ddot .* y_dot) .* (x_dot .* x_ddot + y_dot .* y_ddot)) ./ ...
        (v_safe.^6 + l^2 * (y_ddot .* x_dot - x_ddot .* y_dot).^2);
        
    % Pulizia numerica ai limiti per evitare oscillazioni a veicolo quasi fermo
    soglia_v = 0.05; 
    indici_fermo = v < soglia_v;
    phi(indici_fermo) = 0;
    omega(indici_fermo) = 0;

    % --- 6bis. START AND END VELOCITY V=0 (raised-cosine) ---
    Nv = length(v);
    n_close = max(round(Nv * 0.01), 5);   % 1% dei campioni, almeno 5
    n_close = min(n_close, floor(Nv/2)-1);

    close_blend = ones(1, Nv);  
    i0 = (1:n_close);
    close_blend(i0) = 0.5 * (1 - cos(pi * (i0 - 1) / (n_close - 1)));
    i1 = (Nv-n_close+1:Nv);
    close_blend(i1) = 0.5 * (1 + cos(pi * (i1 - (Nv-n_close+1)) / (n_close - 1)));

    v = v .* close_blend;
    omega = omega .* close_blend;   % a v->0 anche omega->0 con la stessa dolcezza

    % Forza l'esatto azzeramento al primo e ultimo campione
    v(1) = 0;  v(end) = 0;
    omega(1) = 0; omega(end) = 0;
    
    %% UNIFORM SCALING
    v_max = 3;         
    omega_max = 2.0;
    
    v_peak = max(abs(v));
    omega_peak = max(abs(omega));
    
    cv = v_peak / v_max;
    comega = omega_peak / omega_max;
    
    k = max([1, cv, comega]); 
    
    if k > 1
        disp(['Speed limits violated. Applying Uniform Scaling with k = ', num2str(k)]);
        T_new = k * T; 
        v = v / k;
        omega = omega / k;
    else
        disp('All speed limits respected. k = 1');
        T_new = T;
    end
    
    % --- AUTOMATIC LANDMARKS GENERATION ---
    num_landmarks = 8; 
    idx_lm = round(linspace(1, length(x_t), num_landmarks));
    landmarks = zeros(num_landmarks, 2);
    offset_dist = 15; 
    
    for i = 1:num_landmarks
        idx = idx_lm(i);
        x_pos = x_t(idx);
        y_pos = y_t(idx);
        th = theta(idx); 
        
        lx = x_pos + offset_dist * cos(th + pi/2);
        ly = y_pos + offset_dist * sin(th + pi/2);
        
        lx = max(1, min(size(map,1), lx));
        ly = max(1, min(size(map,2), ly));
        
        landmarks(i,:) = [lx, ly];
    end
    
    xd = timeseries(x_t, T_new);
    yd = timeseries(y_t, T_new);
    thetad = timeseries(theta, T_new);
    phid = timeseries(phi, T_new);
    vd = timeseries(v, T_new);         
    omegad = timeseries(omega, T_new); 
    T_max = T_new(end) + 10;
    
    % Estrazione primo campione
    xd0 = xd.Data(1);
    yd0 = yd.Data(1);
    thetad0 = thetad.Data(1);
    phid0 = phid.Data(1);
    q0 = [xd0; yd0; thetad0; phid0];
    
    v_max_ref = max(abs(v));
    omega_max_ref = max(abs(omega));
    
    % --- PLOT GRAFICI ---
    figure('Name', 'Generated Reference');
    plot(x, y, 'bo', 'DisplayName', 'Path points');
    hold on;
    plot(x_t, y_t, 'm', 'lineWidth', 1.2, 'LineWidth', 2, 'DisplayName', 'Generated Trajectory');
    plot(x(1), y(1), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Start point');           
    plot(x(end), y(end), 'go', 'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Goal point');        
    plot(landmarks(:,1), landmarks(:,2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'DisplayName', 'Landmarks (EKF)');
    title('\textbf{Trajectory}','fontsize',12,'Interpreter','latex');
    xlabel('x [m]','Interpreter','latex','fontsize',10);
    ylabel('y [m]','Interpreter','latex','fontsize',10);
    legend('Location', 'best', 'Interpreter','latex');
    grid on; axis equal;

    figure('Name', 'Feedforward Linear Velocity');
    plot(T_new, v, 'LineWidth', 2);
    title('\textbf{Reference Linear Velocity}','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('$v$ [m/s]','Interpreter','latex','FontSize',10);
    xlim([0 T_max]); grid on;

    figure('Name', 'Feedforward Angular Velocity');
    plot(T_new, omega, 'LineWidth', 2);
    title('\textbf{Reference Angular Velocity}','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',10);
    xlim([0 T_max]); grid on;

    figure('Name', 'Reference State X');
    plot(T_new, x_t, 'b', 'LineWidth', 2);
    title('\textbf{Evolution of } $x_d(t)$','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('$x_d$ [m]','Interpreter','latex','FontSize',10);
    grid on;

    figure('Name', 'Reference State Y');
    plot(T_new, y_t, 'r', 'LineWidth', 2);
    title('\textbf{Evolution of } $y_d(t)$','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('$y_d$ [m]','Interpreter','latex','FontSize',10);
    grid on;

    figure('Name', 'Reference State Theta');
    plot(T_new, theta, 'g', 'LineWidth', 2);
    title('\textbf{Evolution of } $\theta_d(t)$','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('\theta_d$ [rad]','Interpreter','latex','FontSize',10);
    grid on;

    figure('Name', 'Reference State Phi');
    plot(T_new, phi, 'k', 'LineWidth', 2);
    title('\textbf{Evolution of } $\phi_d(t)$','FontSize',10,'Interpreter','latex');
    xlabel('t [s]','Interpreter','latex','FontSize',10);
    ylabel('$\phi_d$ [rad]','Interpreter','latex','FontSize',10);
    grid on;
end