function build_test_model()
    % BUILD_TEST_MODEL Create a minimal Simulink model to test pipeline_wrapper
    %
    % This script builds a simple closed-loop test model programmatically:
    % - Synthetic tick generator (From Workspace blocks with pre-computed data)
    % - Pipeline wrapper MATLAB Function block
    % - Display/logging of outputs
    % - Runs for 20 ticks to verify Simulink integration
    %
    % Usage:
    %   >> build_test_model()
    %   >> sim('plan_a_test_model')
    %
    % The model is saved to models/plan_a_test_model.slx

    fprintf('=== Building Simulink Test Model ===\n\n');

    % Model name and path
    model_name = 'plan_a_test_model';
    model_path = fullfile(pwd, 'models', model_name);

    % Check if model exists
    if bdIsLoaded(model_name)
        fprintf('Model %s already loaded, closing it...\n', model_name);
        close_system(model_name, 0);
    end

    % === STEP 1: Pre-generate synthetic data (20 ticks) ===
    fprintf('[1/4] Pre-generating synthetic tick data (20 ticks)...\n');

    n_ticks = 20;
    MAX_DET = 10;
    MAX_LIDAR = 500;
    IMG_H = 600;
    IMG_W = 800;

    % Pre-allocate storage for all ticks
    tick_data = struct();
    tick_data.det_mat = zeros(n_ticks, MAX_DET, 5);
    tick_data.det_class_ids = zeros(n_ticks, MAX_DET);
    tick_data.n_det = zeros(n_ticks, 1);
    tick_data.lidar_xyz = zeros(n_ticks, MAX_LIDAR, 3, 'single');
    tick_data.n_lidar = zeros(n_ticks, 1);
    tick_data.ego_vec = zeros(n_ticks, 6);
    tick_data.seg_tags = zeros(n_ticks, IMG_H, IMG_W);

    % Generate one tick of data
    for t = 1:n_ticks
        % Use the synthetic generator (slightly modified for this context)
        det_mat = ones(MAX_DET, 5) * NaN;
        det_class_ids = zeros(MAX_DET, 1);

        % Pedestrian: moving across the road
        ped_x = 14.71;
        ped_y = 3.01 - (t - 1) * 0.05 * 1.0;  % moving at -1 m/s in y
        ped_conf = 0.87;
        det_mat(1, :) = [350 + t*2, 200, 420 + t*2, 400, ped_conf];
        det_class_ids(1) = 0;  % pedestrian

        % Animal: wandering
        animal_x = 25.03;
        animal_y = -0.96 + sin(t * 0.1) * 0.5;
        animal_conf = 0.92;
        det_mat(2, :) = [600, 100 + t, 650, 280 + t, animal_conf];
        det_class_ids(2) = 1;  % animal
        n_det = 2;

        % LiDAR: random points around objects
        lidar_xyz = single(randn(MAX_LIDAR, 3) * 5 + [10, 0, 0]);
        n_lidar = 50;

        % Ego state: stationary, goal ahead
        ego_vec = [0.0, 0.0, 0.0, 5.0, 25.0, 0.0];

        % Segmentation: bottom half is road
        seg_tags = ones(IMG_H, IMG_W);
        seg_tags(IMG_H/2:IMG_H, :) = 7;

        % Store in structure
        tick_data.det_mat(t, :, :) = det_mat;
        tick_data.det_class_ids(t, :) = det_class_ids;
        tick_data.n_det(t) = n_det;
        tick_data.lidar_xyz(t, :, :) = lidar_xyz;
        tick_data.n_lidar(t) = n_lidar;
        tick_data.ego_vec(t, :) = ego_vec;
        tick_data.seg_tags(t, :, :) = seg_tags;
    end

    fprintf('  ✓ Generated %d ticks of synthetic data\n\n', n_ticks);

    % === STEP 2: Create Simulink model ===
    fprintf('[2/4] Creating Simulink model structure...\n');

    new_system(model_name);
    open_system(model_name);

    % Set up model parameters
    set_param(model_name, 'Solver', 'FixedStepDiscrete', ...
                          'FixedStep', '0.05', ...  % 50ms tick = 20 Hz
                          'SimulationMode', 'normal');

    % === STEP 3: Add blocks ===
    fprintf('[3/4] Adding Simulink blocks...\n');

    % Use absolute coordinates for clarity
    x_col1 = 50;
    x_col2 = 250;
    x_col3 = 550;

    y_row = 100;
    y_spacing = 80;

    % 3.1: From Workspace blocks (inputs)
    % These will be driven by the pre-generated tick_data

    % det_mat From Workspace
    add_block('simulink/Sources/From Workspace', ...
              [model_name '/det_mat'], ...
              'Position', [x_col1, y_row, x_col1+80, y_row+40], ...
              'VariableName', 'det_mat_ts', ...
              'SampleTime', '0.05', ...
              'Interpolate', 'off');

    % n_det From Workspace
    add_block('simulink/Sources/From Workspace', ...
              [model_name '/n_det'], ...
              'Position', [x_col1, y_row + y_spacing*1, x_col1+80, y_row + y_spacing*1 + 40], ...
              'VariableName', 'n_det_ts', ...
              'SampleTime', '0.05', ...
              'Interpolate', 'off');

    % lidar_xyz From Workspace
    add_block('simulink/Sources/From Workspace', ...
              [model_name '/lidar_xyz'], ...
              'Position', [x_col1, y_row + y_spacing*2, x_col1+80, y_row + y_spacing*2 + 40], ...
              'VariableName', 'lidar_xyz_ts', ...
              'SampleTime', '0.05', ...
              'Interpolate', 'off');

    % n_lidar From Workspace
    add_block('simulink/Sources/From Workspace', ...
              [model_name '/n_lidar'], ...
              'Position', [x_col1, y_row + y_spacing*3, x_col1+80, y_row + y_spacing*3 + 40], ...
              'VariableName', 'n_lidar_ts', ...
              'SampleTime', '0.05', ...
              'Interpolate', 'off');

    % 3.2: Pipeline Wrapper MATLAB Function block
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [model_name '/pipeline_wrapper_fcn'], ...
              'Position', [x_col2, y_row, x_col2+150, y_row + y_spacing*3 + 40]);

    % Edit the function block to call pipeline_wrapper
    fcn_block = [model_name '/pipeline_wrapper_fcn'];
    set_param(fcn_block, 'MATLABFunctionDescription', ...
              ['function [wp_xy, n_wp, is_valid, replanned, total_ms, ' ...
               'nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ' ...
               'pipeline_wrapper_fcn(det_mat, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags)' newline ...
               '[wp_xy, n_wp, is_valid, replanned, total_ms, nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ' ...
               'pipeline_wrapper(det_mat, int32(zeros(10,5)), n_det, lidar_xyz, n_lidar, ego_vec, int32(seg_tags));']);

    % 3.3: Output display blocks
    add_block('simulink/Sinks/Scope', ...
              [model_name '/latency_scope'], ...
              'Position', [x_col3, y_row, x_col3+100, y_row + 60]);

    add_block('simulink/Sinks/Display', ...
              [model_name '/waypoint_count'], ...
              'Position', [x_col3, y_row + y_spacing*1, x_col3+100, y_row + y_spacing*1 + 40]);

    add_block('simulink/Sinks/Display', ...
              [model_name '/nearest_obstacle'], ...
              'Position', [x_col3, y_row + y_spacing*2, x_col3+100, y_row + y_spacing*2 + 40]);

    fprintf('  ✓ Added %d blocks\n\n', 9);

    % === STEP 4: Create time series data for From Workspace blocks ===
    fprintf('[4/4] Preparing workspace time series data...\n');

    % Create time vector
    t_vec = (0:n_ticks-1)' * 0.05;

    % Create time series objects for each input
    % Note: From Workspace expects either a timeseries or [time data] format

    % This approach: we'll pass simple arrays and let From Workspace sample them
    % (Simulink will use the first column as time, subsequent as data)

    assignin('base', 'det_mat_ts', tick_data.det_mat);
    assignin('base', 'n_det_ts', tick_data.n_det);
    assignin('base', 'lidar_xyz_ts', tick_data.lidar_xyz);
    assignin('base', 'n_lidar_ts', tick_data.n_lidar);

    % Also assign constant blocks for ego_vec and seg_tags
    assignin('base', 'ego_vec_const', zeros(n_ticks, 6));  % placeholder
    assignin('base', 'seg_tags_const', zeros(n_ticks, IMG_H, IMG_W));  % placeholder

    fprintf('  ✓ Time series data prepared\n\n');

    % === Save the model ===
    save_system(model_name, model_path);
    fprintf('✓ Model saved to: %s\n\n', model_path);

    fprintf('=== MODEL BUILD COMPLETE ===\n');
    fprintf('\nTo run the model:\n');
    fprintf('  1. open_system(''plan_a_test_model'')\n');
    fprintf('  2. sim(''plan_a_test_model'')\n');
    fprintf('\nThe model will run for %d ticks (%.2f seconds sim time)\n', n_ticks, n_ticks * 0.05);
    fprintf('Output: Latency, waypoint counts, and nearest obstacle info\n');
end
