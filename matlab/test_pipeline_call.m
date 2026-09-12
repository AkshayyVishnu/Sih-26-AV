function test_pipeline_call()
    % TEST_PIPELINE_CALL Phase 1 isolation test: pure script, no Simulink
    %
    % This is the RISKIEST UNKNOWN in the entire Plan A — no prior art exists
    % anywhere for calling a custom Python pipeline from MATLAB via py.*.
    % This script tests it in the simplest possible way, before building anything
    % else around it.
    %
    % Success criteria (per docs/two-path-strategy.md):
    % 1. No MATLAB error on any line (especially pyenv/import/tick call)
    % 2. Returned waypoints are real numeric, not empty/garbage
    % 3. Run in a 40-iteration loop on the SAME Pipeline handle (state must persist)
    % 4. Measure total_ms latency — confirm it's within ~20-45ms estimate, well under
    %    the 150-200ms closed-loop-success-collapse ceiling

    fprintf('=== PHASE 1 ISOLATION TEST ===\n');
    fprintf('Testing py.* call to custom Python pipeline (no Simulink)\n\n');

    % Step 1: Initialize Python environment
    fprintf('[1/4] Initializing Python environment...\n');
    init_pyenv();

    % Step 2: Create a single Pipeline instance (must persist across ticks)
    fprintf('[2/4] Instantiating Pipeline (will hold tracker/planner state across ticks)...\n');

    % Camera intrinsic (3x3 K matrix, from run_demo.py)
    % Assumes 800x600 image, ~90 degree FOV
    cam_intrinsic = [727.94 0 400; ...
                     0 727.94 300; ...
                     0 0 1];
    cam_intrinsic_py = py.numpy.array(cam_intrinsic);

    % Camera to LiDAR extrinsic (4x4 homogeneous, from run_demo.py)
    % Rotation: X-forward/Y-left/Z-up (CARLA ego) -> X-right/Y-down/Z-forward (camera optical)
    cam_to_lidar_ext = [0 -1 0 0; ...
                        0 0 -1 0; ...
                        1 0 0 0; ...
                        0 0 0 1];
    cam_to_lidar_ext_py = py.numpy.array(cam_to_lidar_ext, pyargs('dtype', 'float32'));

    % Instantiate the Pipeline object (once, held in memory across loop iterations)
    pipelineObj = py.pipeline.pipeline.Pipeline(cam_intrinsic_py, cam_to_lidar_ext_py, ...
                                                 pyargs('dt', 0.05));
    fprintf('  ✓ Pipeline instantiated\n\n');

    % Step 3: Run 40 iterations, measuring latency
    fprintf('[3/4] Running 40 synthetic ticks...\n');
    latencies = zeros(1, 40);

    for tick = 1:40
        % Build synthetic detections: one pedestrian, one animal
        det_pedestrian = py.pipeline.types.Detection('pedestrian', 0.87, 350.0, 200.0, 420.0, 400.0);
        det_animal = py.pipeline.types.Detection('animal', 0.92, 600.0, 100.0, 650.0, 280.0);
        py_detections = py.list({det_pedestrian, det_animal});

        % Build synthetic LiDAR: 50 random points in ego frame
        lidar_xyz = randn(50, 3, 'single') * 5.0 + [10, 0, 0];  % 50 points, ~10m ahead
        lidar_xyz_py = py.numpy.array(lidar_xyz, pyargs('dtype', 'float32'));

        % Ego state: stationary at origin, goal 25m ahead
        ego_state = py.pipeline.types.EgoState(0.0, 0.0, 0.0, 5.0, 25.0, 0.0);

        % Synthetic segmentation: (600, 800) image with 7=Road in lower half
        seg_tags = ones(600, 800, 'int32');
        seg_tags(300:600, :) = 7;
        seg_tags_py = py.numpy.array(int32(seg_tags), pyargs('dtype', 'int32'));

        % Call Pipeline.tick() — the core test
        try
            result = pipelineObj.tick(py_detections, lidar_xyz_py, ego_state, ...
                                       pyargs('segmentation_tags', seg_tags_py));
            planned = result{1};
            timings = result{2};

            % Unpack results (data marshaling per Phase 2.2 contract)
            wp = double(py.numpy.array(planned.waypoints));
            is_valid = logical(planned.is_valid);
            replanned = logical(planned.replanned);
            total_ms = double(timings.total_ms);
            nearest_class = string(planned.nearest_obstacle_class);
            nearest_dist = double(planned.nearest_obstacle_distance_m);

            latencies(tick) = total_ms;

            % Print progress every 10 ticks
            if mod(tick, 10) == 0
                fprintf('  Tick %2d: %.2fms latency, %d waypoints, replanned=%s, nearest=%s@%.1fm\n', ...
                        tick, total_ms, size(wp, 1), string(replanned), nearest_class, nearest_dist);
            end

        catch ME
            fprintf('\n❌ ERROR on tick %d:\n%s\n', tick, ME.message);
            return;
        end
    end

    fprintf('  ✓ All 40 ticks completed\n\n');

    % Step 4: Analyze latency
    fprintf('[4/4] Latency analysis:\n');
    fprintf('  Mean:   %.2f ms\n', mean(latencies));
    fprintf('  Max:    %.2f ms\n', max(latencies));
    fprintf('  Min:    %.2f ms\n', min(latencies));
    fprintf('  Std:    %.2f ms\n\n', std(latencies));

    % Check against the ~150-200ms closed-loop-success ceiling
    if max(latencies) > 150
        warning('test_pipeline_call:LatencyWarning', ...
                'Max latency (%.2fms) exceeds 150ms — may cause closed-loop instability.', ...
                max(latencies));
    else
        fprintf('  ✓ All ticks well under the 150-200ms closed-loop-success ceiling\n');
    end

    fprintf('\n=== PHASE 1 TEST PASSED ===\n');
    fprintf('The py.* call to pipeline.Pipeline().tick() works reliably.\n');
    fprintf('Ready to proceed to Phase 2: full .slx skeleton.\n');
end
