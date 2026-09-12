function [wp_xy, n_wp, is_valid, replanned, total_ms, ...
          nearest_class_id, nearest_conf, nearest_dist, nearest_ttc, pipeline_ok] = ...
          pipeline_wrapper(det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags)
    % PIPELINE_WRAPPER MATLAB Function block wrapper around Python Pipeline.tick()
    %
    % This is the single, critical interface between Simulink signals (fixed-size,
    % fixed-type) and the Python pipeline (dynamic dataclasses, list outputs).
    % Called once per Simulink tick, it:
    % 1. Marshals fixed-size Simulink inputs → Python objects
    % 2. Calls pipelineObj.tick(...)
    % 3. Unmarshals variable-size Python outputs → fixed-size Simulink outputs
    % 4. Catches all errors gracefully (feeds SAFETY_SUPERVISOR watchdog)
    %
    % Per Phase 2.2 data-marshaling contract, this wrapper handles all the
    % type conversions (MATLAB double ↔ Python float, MATLAB int32 ↔ Python int, etc.).
    %
    % INPUT SIGNALS (from Simulink):
    %   det_mat:       [MAX_DET x 5] double matrix (x1,y1,x2,y2,confidence)
    %   det_class_ids: [MAX_DET x 1] int32 vector (0=pedestrian, 1=animal, etc.)
    %   n_det:         int32 scalar, actual number of detections
    %   lidar_xyz:     [MAX_LIDAR x 3] single matrix (x,y,z) in ego frame
    %   n_lidar:       int32 scalar, actual number of LiDAR points
    %   ego_vec:       [6 x 1] double (x,y,yaw,speed,goal_x,goal_y)
    %   seg_tags:      [H x W] int32 segmentation image (7=Road, 1=non-drivable)
    %
    % OUTPUT SIGNALS (to Simulink):
    %   wp_xy:           [MAX_WP x 2] double, padded waypoints
    %   n_wp:            int32 scalar, actual waypoint count
    %   is_valid:        boolean, path valid
    %   replanned:       boolean, triggered replan this tick
    %   total_ms:        double, total pipeline latency (ms)
    %   nearest_class_id: int32, class ID of nearest obstacle (0=pedestrian, 1=animal, etc.; -1=none)
    %   nearest_conf:    double, confidence of nearest detection
    %   nearest_dist:    double, distance to nearest obstacle (m)
    %   nearest_ttc:     double, time-to-collision (s)
    %   pipeline_ok:     boolean, no Python exception occurred

    % Constants
    MAX_WP = int32(200);
    MAX_DET = int32(10);
    MAX_LIDAR = int32(500);

    % === STEP 1: Initialize Python environment (first call only) ===
    persistent pipelineObj pyenv_initialized

    if isempty(pyenv_initialized)
        init_pyenv();

        % Camera intrinsic (3x3 K matrix, from run_demo.py baseline)
        % Assumes 800x600 image, ~90 degree FOV
        % TODO: Replace with real CARLA camera intrinsics once Phase 3 wires CARLA connection
        cam_intrinsic = [727.94 0 400; ...
                         0 727.94 300; ...
                         0 0 1];
        cam_intrinsic_py = py.numpy.array(cam_intrinsic);

        % Camera to LiDAR extrinsic (4x4 homogeneous)
        % Rotation: X-forward/Y-left/Z-up (CARLA ego) -> X-right/Y-down/Z-forward (camera optical)
        % TODO: Calibrate against real CARLA vehicle/camera setup
        cam_to_lidar_ext = [0 -1 0 0; ...
                            0 0 -1 0; ...
                            1 0 0 0; ...
                            0 0 0 1];
        cam_to_lidar_ext_py = py.numpy.array(cam_to_lidar_ext, pyargs('dtype', 'float32'));

        % Instantiate Pipeline (ONCE, held in memory across ticks)
        % It owns the tracker and planner state — DO NOT recreate each tick
        pipelineObj = py.pipeline.pipeline.Pipeline(cam_intrinsic_py, cam_to_lidar_ext_py, ...
                                                     pyargs('dt', 0.05));
        pyenv_initialized = true;
    end

    % === STEP 2: Initialize outputs with sensible defaults ===
    % (in case of early error return)
    wp_xy = ones(MAX_WP, 2) * NaN;
    n_wp = int32(0);
    is_valid = false;
    replanned = false;
    total_ms = 0.0;
    nearest_class_id = int32(-1);  % -1 = no obstacle
    nearest_conf = 0.0;
    nearest_dist = inf;
    nearest_ttc = inf;
    pipeline_ok = false;

    % === STEP 3: Marshal Simulink inputs → Python objects ===
    try
        % Build list of Detection objects from input matrix
        py_detections = py.list();
        for i = 1:double(n_det)
            class_id = det_class_ids(i);
            class_name = ClassIdMap.id_to_name(class_id);
            confidence = det_mat(i, 5);
            x1 = det_mat(i, 1);
            y1 = det_mat(i, 2);
            x2 = det_mat(i, 3);
            y2 = det_mat(i, 4);

            det = py.pipeline.types.Detection(class_name, confidence, x1, y1, x2, y2);
            py_detections.append(det);
        end

        % Convert LiDAR points to numpy array (N,3 float32)
        lidar_data = lidar_xyz(1:double(n_lidar), :);
        lidar_py = py.numpy.array(single(lidar_data), pyargs('dtype', 'float32'));

        % Build EgoState object
        x = ego_vec(1);
        y = ego_vec(2);
        yaw = ego_vec(3);
        speed = ego_vec(4);
        goal_x = ego_vec(5);
        goal_y = ego_vec(6);
        ego_py = py.pipeline.types.EgoState(x, y, yaw, speed, goal_x, goal_y);

        % Convert segmentation tags to numpy array (H,W int32)
        seg_py = py.numpy.array(int32(seg_tags), pyargs('dtype', 'int32'));

    catch ME
        warning('pipeline_wrapper:MarshalError', 'Failed to marshal Simulink inputs to Python: %s', ME.message);
        pipeline_ok = false;
        return;
    end

    % === STEP 4: Call Pipeline.tick() ===
    % This is the core py.* call — all error handling happens here
    try
        result = pipelineObj.tick(py_detections, lidar_py, ego_py, ...
                                   pyargs('segmentation_tags', seg_py));
        planned = result{1};
        timings = result{2};

    catch ME
        % Python exception during tick() — set fault signal for safety watchdog
        warning('pipeline_wrapper:TickError', 'Pipeline.tick() failed: %s', ME.message);
        pipeline_ok = false;
        return;
    end

    % === STEP 5: Unmarshal Python outputs → Simulink signals ===
    try
        % Extract and convert waypoints (list of tuples → 2D array)
        wp_list = planned.waypoints;
        n_waypoints = length(wp_list);

        if n_waypoints > 0
            % Convert to numpy array for easier indexing
            wp_np = py.numpy.array(wp_list);
            wp_data = double(wp_np);  % Convert to MATLAB double

            % Pad/truncate to MAX_WP
            n_to_copy = min(n_waypoints, double(MAX_WP));
            wp_xy(1:n_to_copy, :) = wp_data(1:n_to_copy, :);
            n_wp = int32(n_to_copy);
        else
            n_wp = int32(0);
        end

        % Extract PlannedPath fields
        is_valid = logical(planned.is_valid);
        replanned = logical(planned.replanned);

        % Extract TickTimings
        total_ms = double(timings.total_ms);

        % Extract nearest-obstacle fields (populated by pipeline.py:tick())
        nearest_class = string(planned.nearest_obstacle_class);
        nearest_class_id = ClassIdMap.name_to_id(nearest_class);
        nearest_conf = double(planned.nearest_obstacle_confidence);
        nearest_dist = double(planned.nearest_obstacle_distance_m);
        nearest_ttc = double(planned.nearest_obstacle_ttc_s);

        % Success
        pipeline_ok = true;

    catch ME
        warning('pipeline_wrapper:UnmarshalError', 'Failed to unmarshal Python outputs: %s', ME.message);
        pipeline_ok = false;
        return;
    end

end
