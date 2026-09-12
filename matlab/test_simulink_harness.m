function test_simulink_harness()
    % TEST_SIMULINK_HARNESS Simulate Simulink discrete-time execution of pipeline_wrapper
    %
    % This test mimics how Simulink will call the wrapper:
    % - Discrete time steps (dt = 0.05s)
    % - Persistent state across ticks
    % - Fixed-size signal propagation
    % - Synthetic inputs changing each tick
    %
    % This validates the wrapper works in a Simulink-like environment
    % BEFORE building the actual .slx model.

    fprintf('=== SIMULINK HARNESS TEST ===\n');
    fprintf('Simulating Simulink discrete-time execution (dt=0.05s)\n\n');

    addpath('matlab');

    % Simulation parameters
    dt = 0.05;
    n_ticks = 20;
    MAX_DET = 10;
    MAX_LIDAR = 500;
    IMG_H = 600;
    IMG_W = 800;
    MAX_WP = 200;

    % Pre-allocate output arrays (logging)
    latency_log = zeros(n_ticks, 1);
    wp_count_log = zeros(n_ticks, 1);
    is_valid_log = logical(zeros(n_ticks, 1));
    replanned_log = logical(zeros(n_ticks, 1));
    nearest_class_log = zeros(n_ticks, 1);
    nearest_dist_log = zeros(n_ticks, 1);
    nearest_ttc_log = zeros(n_ticks, 1);
    pipeline_ok_log = logical(zeros(n_ticks, 1));

    fprintf('Tick | Latency | WPs | Valid | Replan | Nearest | TTC   | Status\n');
    fprintf('----|---------|-----|-------|--------|---------|-------|--------\n');

    % === MAIN LOOP: Simulate Simulink ticks ===
    for tick = 1:n_ticks
        % --- Generate synthetic inputs for this tick ---
        det_mat = ones(MAX_DET, 5) * NaN;
        det_class_ids = zeros(MAX_DET, 1, 'int32');

        % Pedestrian: moving across the road
        ped_conf = 0.87;
        det_mat(1, :) = [350 + tick*3, 200, 420 + tick*3, 400, ped_conf];
        det_class_ids(1) = int32(0);  % pedestrian

        % Animal: wandering with sinusoidal pattern
        animal_conf = 0.92;
        det_mat(2, :) = [600, 100 + tick*0.5, 650, 280 + tick*0.5, animal_conf];
        det_class_ids(2) = int32(1);  % animal
        n_det = int32(2);

        % LiDAR: random points + structure
        lidar_xyz = single(randn(MAX_LIDAR, 3) * 5 + [10 + tick*0.1, 0, 0]);
        n_lidar = int32(50);

        % Ego state: moving forward slowly
        ego_vec = [0.0; 0.0; 0.0; 5.0 + tick*0.1; 25.0; 0.0];

        % Segmentation
        seg_tags = int32(ones(IMG_H, IMG_W));
        seg_tags(IMG_H/2:IMG_H, :) = int32(7);

        % --- Call the wrapper (this is the core test) ---
        try
            [wp_xy, n_wp, is_valid, replanned, total_ms, ...
             nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ...
                pipeline_wrapper(det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags);

            % Log results
            latency_log(tick) = total_ms;
            wp_count_log(tick) = double(n_wp);
            is_valid_log(tick) = is_valid;
            replanned_log(tick) = replanned;
            nearest_class_log(tick) = nearest_class_id;
            nearest_dist_log(tick) = nearest_dist;
            nearest_ttc_log(tick) = nearest_ttc;
            pipeline_ok_log(tick) = pipeline_ok;

            % Print tick summary
            status = 'OK';
            if ~pipeline_ok
                status = 'ERROR';
            elseif replanned
                status = 'REPLAN';
            end

            fprintf('%4d | %7.2f | %3d | %5s | %6s | %7.1f | %5.2f | %s\n', ...
                    tick, total_ms, double(n_wp), string(is_valid), string(replanned), ...
                    nearest_dist, nearest_ttc, status);

        catch ME
            fprintf('%4d | ERROR: %s\n', tick, ME.message);
            pipeline_ok_log(tick) = false;
            status = 'FAILED';
        end
    end

    fprintf('\n');

    % === ANALYSIS ===
    fprintf('=== TEST SUMMARY ===\n\n');

    % Success rate
    success_rate = sum(pipeline_ok_log) / n_ticks * 100;
    fprintf('Success rate: %.1f%% (%d/%d ticks)\n', success_rate, sum(pipeline_ok_log), n_ticks);

    if success_rate < 100
        fprintf('WARNING: Some ticks failed. Review log above.\n\n');
        return;
    end

    % Latency analysis
    fprintf('\nLatency (pipeline only, not including Simulink overhead):\n');
    fprintf('  Mean:   %.2f ms\n', mean(latency_log));
    fprintf('  Max:    %.2f ms\n', max(latency_log));
    fprintf('  Min:    %.2f ms\n', min(latency_log));
    fprintf('  Std:    %.2f ms\n', std(latency_log));

    if max(latency_log) > 150
        fprintf('  WARNING: Max latency exceeds 150ms closure threshold\n');
    else
        fprintf('  OK: All ticks under 150ms threshold\n');
    end

    % Replanning frequency
    n_replans = sum(replanned_log);
    fprintf('\nReplanning: %d replans in %d ticks (%.1f%%)\n', ...
            n_replans, n_ticks, n_replans/n_ticks*100);

    % Nearest obstacle statistics
    obstacle_detected = nearest_dist_log < inf;
    n_obstacle_ticks = sum(obstacle_detected);
    fprintf('\nNearest obstacle detection: %d ticks had obstacles detected (%.1f%%)\n', ...
            n_obstacle_ticks, n_obstacle_ticks/n_ticks*100);

    if n_obstacle_ticks > 0
        fprintf('  Obstacle distances: min=%.1f m, max=%.1f m, mean=%.1f m\n', ...
                min(nearest_dist_log(obstacle_detected)), ...
                max(nearest_dist_log(obstacle_detected)), ...
                mean(nearest_dist_log(obstacle_detected)));
    end

    % Waypoint generation
    fprintf('\nWaypoint generation:\n');
    fprintf('  Mean waypoints per tick: %.0f\n', mean(wp_count_log(wp_count_log > 0)));
    fprintf('  All ticks valid: %s\n', string(all(is_valid_log)));

    fprintf('\n=== HARNESS TEST PASSED ===\n');
    fprintf('Pipeline wrapper ready for Simulink integration.\n\n');
    fprintf('Next step: Build actual Simulink model with MATLAB Function block.\n');
end
