function build_full_model()
    % BUILD_FULL_MODEL Create complete Simulink model for Plan A (Phase 2)
    
    fprintf('=== BUILDING FULL SIMULINK MODEL (Phase 2) ===\n\n');
    
    model_name = 'plan_a_full_model';
    
    % Close if already open
    if bdIsLoaded(model_name)
        fprintf('[0/3] Closing existing model...\n');
        close_system(model_name, 0);
    end
    
    % ===== STEP 1: CREATE MODEL =====
    fprintf('[1/3] Creating model and configuring solver...\n');
    
    new_system(model_name);
    open_system(model_name);
    
    set_param(model_name, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.05', 'StopTime', '1.0');
    
    fprintf('  ✓ Model created with dt=0.05s (20 Hz, 20 ticks)\n\n');
    
    % ===== STEP 2: GENERATE SYNTHETIC DATA =====
    fprintf('[2/3] Pre-generating synthetic input data (20 ticks)...\n');
    
    n_ticks = 20;
    MAX_DET = 10;
    MAX_LIDAR = 500;
    IMG_H = 600;
    IMG_W = 800;
    
    det_mat = ones(n_ticks, MAX_DET, 5) * NaN;
    det_class_ids = zeros(n_ticks, MAX_DET);
    n_det_vec = zeros(n_ticks, 1);
    lidar_xyz = zeros(n_ticks, MAX_LIDAR, 3, 'single');
    n_lidar_vec = zeros(n_ticks, 1);
    ego_vec = zeros(n_ticks, 6);
    seg_tags = zeros(n_ticks, IMG_H, IMG_W, 'uint8');
    
    for t = 1:n_ticks
        det_mat(t, 1, :) = [350 + t*3, 200, 420 + t*3, 400, 0.87];
        det_class_ids(t, 1) = 0;
        det_mat(t, 2, :) = [600, 100 + t*0.5, 650, 280 + t*0.5, 0.92];
        det_class_ids(t, 2) = 1;
        n_det_vec(t) = 2;
        lidar_xyz(t, 1:50, :) = single(randn(50, 3) * 5 + [10 + t*0.1, 0, 0]);
        n_lidar_vec(t) = 50;
        ego_vec(t, :) = [0, 0, 0, 5.0 + t*0.1, 25.0, 0];
        seg_tags(t, :, :) = 1;
        seg_tags(t, IMG_H/2:IMG_H, :) = 7;
    end
    
    assignin('base', 'det_mat_ts', det_mat);
    assignin('base', 'n_det_ts', n_det_vec);
    assignin('base', 'lidar_xyz_ts', lidar_xyz);
    assignin('base', 'n_lidar_ts', n_lidar_vec);
    assignin('base', 'ego_vec_ts', ego_vec);
    assignin('base', 'seg_tags_ts', seg_tags);
    
    fprintf('  ✓ Generated synthetic data for all 20 ticks\n\n');
    
    % ===== STEP 3: ADD BLOCKS =====
    fprintf('[3/3] Adding Simulink blocks...\n');
    
    x_col1 = 50; x_col2 = 350; x_col3 = 650;
    y_base = 100; y_spacing = 80;
    
    % Input blocks (From Workspace)
    add_block('simulink/Sources/From Workspace', [model_name '/det_mat'], ...
        'VariableName', 'det_mat_ts', 'SampleTime', '0.05', ...
        'Position', [x_col1, y_base, x_col1+80, y_base+30]);
    
    add_block('simulink/Sources/From Workspace', [model_name '/n_det'], ...
        'VariableName', 'n_det_ts', 'SampleTime', '0.05', ...
        'Position', [x_col1, y_base+y_spacing, x_col1+80, y_base+y_spacing+30]);
    
    add_block('simulink/Sources/From Workspace', [model_name '/lidar_xyz'], ...
        'VariableName', 'lidar_xyz_ts', 'SampleTime', '0.05', ...
        'Position', [x_col1, y_base+2*y_spacing, x_col1+80, y_base+2*y_spacing+30]);
    
    add_block('simulink/Sources/From Workspace', [model_name '/ego_vec'], ...
        'VariableName', 'ego_vec_ts', 'SampleTime', '0.05', ...
        'Position', [x_col1, y_base+3*y_spacing, x_col1+80, y_base+3*y_spacing+30]);
    
    add_block('simulink/Sources/From Workspace', [model_name '/seg_tags'], ...
        'VariableName', 'seg_tags_ts', 'SampleTime', '0.05', ...
        'Position', [x_col1, y_base+4*y_spacing, x_col1+80, y_base+4*y_spacing+30]);
    
    % Pipeline wrapper MATLAB Function
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [model_name '/wrapper'], ...
              'Position', [x_col2, y_base, x_col2+150, y_base+180]);
    
    % Stateflow chart
    add_block('stateflow/Chart', [model_name '/DecisionLogic'], ...
              'Position', [x_col3, y_base, x_col3+200, y_base+200]);
    
    % Output displays
    add_block('simulink/Sinks/Display', [model_name '/latency'], ...
              'Position', [1050, y_base+50, 1150, y_base+90]);
    
    add_block('simulink/Sinks/Display', [model_name '/waypoints'], ...
              'Position', [1050, y_base+150, 1150, y_base+190]);
    
    fprintf('  ✓ Added 5 input blocks (From Workspace)\n');
    fprintf('  ✓ Added pipeline wrapper MATLAB Function block\n');
    fprintf('  ✓ Added Stateflow DecisionLogic chart\n');
    fprintf('  ✓ Added 2 Display blocks for outputs\n\n');
    
    % ===== SAVE =====
    fprintf('Saving model...\n');
    
    save_system(model_name, fullfile('models', [model_name '.slx']));
    
    fprintf('  ✓ Saved to: models/%s.slx\n\n', model_name);
    
    fprintf('=== MODEL BUILD COMPLETE ===\n\n');
    fprintf('✓ Model created with all blocks\n');
    fprintf('✓ Synthetic data (20 ticks) pre-loaded in workspace\n');
    fprintf('✓ Ready for Stateflow chart editing and wiring\n\n');
    fprintf('Next steps:\n');
    fprintf('  1. open_system(''plan_a_full_model'')\n');
    fprintf('  2. Edit wrapper block (double-click) to call pipeline_wrapper()\n');
    fprintf('  3. Edit DecisionLogic chart (double-click) - see BUILD_STATEFLOW_MANUALLY.md\n');
    fprintf('  4. Wire blocks together (det_mat → wrapper → decision, etc.)\n');
    fprintf('  5. Run: sim(''plan_a_full_model'')\n\n');
end
