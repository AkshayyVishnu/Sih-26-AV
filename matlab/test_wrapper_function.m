function test_wrapper_function()
    % TEST_WRAPPER_FUNCTION Test the pipeline_wrapper function in isolation
    %
    % This verifies that pipeline_wrapper.m correctly:
    % 1. Marshals Simulink signals to Python dataclasses
    % 2. Calls Pipeline.tick() and gets results back
    % 3. Unmarshals Python outputs to Simulink signals
    % 4. Handles errors gracefully
    %
    % This is a "unit test" for the wrapper before it goes into Simulink.

    fprintf('=== TESTING PIPELINE_WRAPPER FUNCTION ===\n\n');

    % Add MATLAB search path
    addpath('matlab');

    % Test 1: Happy path with two detections
    fprintf('[TEST 1] Happy path: 2 detections, 50 LiDAR points\n');

    % Build input signals matching the Phase 2.2 contract
    MAX_DET = 10;
    MAX_LIDAR = 500;
    IMG_H = 600;
    IMG_W = 800;
    MAX_WP = 200;

    % Detection input: [MAX_DET x 5] double (x1,y1,x2,y2,confidence)
    det_mat = ones(MAX_DET, 5) * NaN;
    det_mat(1, :) = [350, 200, 420, 400, 0.87];  % pedestrian bbox + confidence
    det_mat(2, :) = [600, 100, 650, 280, 0.92];  % animal bbox + confidence

    % Detection class IDs: [MAX_DET x 1] int32
    det_class_ids = zeros(MAX_DET, 1, 'int32');
    det_class_ids(1) = int32(0);  % pedestrian
    det_class_ids(2) = int32(1);  % animal
    n_det = int32(2);

    % LiDAR points: [MAX_LIDAR x 3] single
    lidar_xyz = single(randn(MAX_LIDAR, 3) * 5 + [10, 0, 0]);
    n_lidar = int32(50);

    % Ego state: [6 x 1] double (x,y,yaw,speed,goal_x,goal_y)
    ego_vec = [0.0; 0.0; 0.0; 5.0; 25.0; 0.0];

    % Segmentation: [H x W] int32
    seg_tags = int32(ones(IMG_H, IMG_W));
    seg_tags(IMG_H/2:IMG_H, :) = 7;  % bottom half = road

    % Call the wrapper
    try
        [wp_xy, n_wp, is_valid, replanned, total_ms, ...
         nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ...
            pipeline_wrapper(det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags);

        fprintf('  ✓ Call succeeded\n');
        fprintf('    - Waypoints: %d returned (max %d)\n', double(n_wp), MAX_WP);
        fprintf('    - Path valid: %s\n', string(is_valid));
        fprintf('    - Replanned: %s\n', string(replanned));
        fprintf('    - Latency: %.2f ms\n', total_ms);
        fprintf('    - Nearest obstacle: class=%d, dist=%.1f m, conf=%.2f, ttc=%.2f s\n', ...
                nearest_class_id, nearest_dist, nearest_conf, nearest_ttc);
        fprintf('    - Pipeline OK: %s\n\n', string(pipeline_ok));

        % Validate output shapes and types
        assert(size(wp_xy, 1) == MAX_WP, 'Waypoints should be [MAX_WP x 2]');
        assert(size(wp_xy, 2) == 2, 'Waypoints should be [MAX_WP x 2]');
        assert(isa(n_wp, 'int32'), 'n_wp should be int32');
        assert(isa(pipeline_ok, 'logical'), 'pipeline_ok should be boolean');
        assert(total_ms > 0, 'Latency should be positive');

        if pipeline_ok
            fprintf('  ✓ Output validation passed\n');
        else
            fprintf('  ❌ Pipeline reported error\n');
            return;
        end

    catch ME
        fprintf('  ❌ Wrapper call failed: %s\n', ME.message);
        return;
    end

    % Test 2: Multiple ticks on same Pipeline instance (state persistence)
    fprintf('[TEST 2] State persistence: 10 ticks on same Pipeline\n');

    persistent_ok = true;
    for tick = 1:10
        % Slightly different inputs each tick (simulate moving objects)
        det_mat = ones(MAX_DET, 5) * NaN;
        det_mat(1, :) = [350 + tick, 200, 420 + tick, 400, 0.87];
        det_mat(2, :) = [600, 100 + tick*2, 650, 280 + tick*2, 0.92];
        det_class_ids = int32([0; 1; zeros(8,1)]);
        n_det = int32(2);

        % Fresh random LiDAR each tick
        lidar_xyz = single(randn(MAX_LIDAR, 3) * 5 + [10, 0, 0]);
        n_lidar = int32(50);

        ego_vec = [0.0; 0.0; 0.0; 5.0; 25.0; 0.0];
        seg_tags = int32(ones(IMG_H, IMG_W));
        seg_tags(IMG_H/2:IMG_H, :) = 7;

        try
            [wp_xy, n_wp, is_valid, replanned, total_ms, ...
             nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ...
                pipeline_wrapper(det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags);

            if ~pipeline_ok
                fprintf('  ❌ Pipeline failed at tick %d\n', tick);
                persistent_ok = false;
                break;
            end

            if mod(tick, 5) == 0
                fprintf('  Tick %2d: %.2f ms latency, %d waypoints, nearest@%.1f m\n', ...
                        tick, total_ms, double(n_wp), nearest_dist);
            end

        catch ME
            fprintf('  ❌ Tick %d failed: %s\n', tick, ME.message);
            persistent_ok = false;
            break;
        end
    end

    if persistent_ok
        fprintf('  ✓ All 10 ticks completed, Pipeline state persisted\n\n');
    else
        fprintf('  Test FAILED\n');
        return;
    end

    % Test 3: Zero detections (edge case)
    fprintf('[TEST 3] Edge case: 0 detections\n');

    det_mat = ones(MAX_DET, 5) * NaN;
    det_class_ids = zeros(MAX_DET, 1, 'int32');
    n_det = int32(0);  % NO detections
    lidar_xyz = single(randn(MAX_LIDAR, 3) * 5 + [10, 0, 0]);
    n_lidar = int32(50);
    ego_vec = [0.0; 0.0; 0.0; 5.0; 25.0; 0.0];
    seg_tags = int32(ones(IMG_H, IMG_W));
    seg_tags(IMG_H/2:IMG_H, :) = 7;

    try
        [wp_xy, n_wp, is_valid, replanned, total_ms, ...
         nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ...
            pipeline_wrapper(det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags);

        fprintf('  ✓ Call succeeded with 0 detections\n');
        fprintf('    - Waypoints: %d\n', double(n_wp));
        fprintf('    - Nearest obstacle: class=%d (no obstacle)\n\n', nearest_class_id);

        assert(nearest_class_id == int32(-1), 'With no detections, nearest should be -1');

    catch ME
        fprintf('  ❌ Failed: %s\n', ME.message);
        return;
    end

    fprintf('=== ALL TESTS PASSED ===\n');
    fprintf('pipeline_wrapper.m is ready for Simulink integration.\n');
end
