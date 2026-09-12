function [det_mat, det_class_ids, n_det, lidar_xyz, n_lidar, ego_vec, seg_tags] = generate_synthetic_tick(tick)
    % GENERATE_SYNTHETIC_TICK MATLAB port of run_demo.py's synthetic generator
    %
    % Used in Phase 2 to drive the full MATLAB Function block pipeline without
    % needing a real CARLA server. Produces the same synthetic scenario:
    % - One pedestrian drifting across the road (starting 14.71m ahead)
    % - One animal (cow) wandering (starting 25.03m ahead)
    % - Both move over time so the tracker/planner adapt
    %
    % Args:
    %   tick: int32, iteration number (1-40 typically)
    %
    % Returns:
    %   det_mat: [MAX_DET x 5] double matrix, NaN-padded (x1,y1,x2,y2,confidence)
    %   det_class_ids: [MAX_DET x 1] int32 vector (0=pedestrian, 1=animal, etc.)
    %   n_det: int32, actual number of detections this tick
    %   lidar_xyz: [MAX_LIDAR x 3] single matrix, (x,y,z) in ego frame, zero-padded
    %   n_lidar: int32, actual number of LiDAR points
    %   ego_vec: [6 x 1] double (x, y, yaw, speed, goal_x, goal_y)
    %   seg_tags: [H x W] int32 segmentation image (7=Road, 1=non-drivable)

    % Constants
    MAX_DET = 10;
    MAX_LIDAR = 500;
    IMG_H = 600;
    IMG_W = 800;

    % Initialize outputs with zero/NaN padding
    det_mat = ones(MAX_DET, 5) * NaN;
    det_class_ids = zeros(MAX_DET, 1, 'int32');
    lidar_xyz = zeros(MAX_LIDAR, 3, 'single');
    seg_tags = ones(IMG_H, IMG_W, 'int32');  % 1 = non-drivable by default

    % Segmentation: bottom half is road (tag=7)
    seg_tags(IMG_H/2:IMG_H, :) = 7;

    % --- Synthetic objects: pedestrian and animal moving over time ---
    % Pedestrian: starts at (14.71, 3.01), moves left (toward y=0) at 1 m/s
    ped_x = 14.71;
    ped_y = 3.01 - (tick - 1) * 0.05 * 1.0;  % -1 m/s in y, dt=0.05s per tick
    ped_conf = 0.87;

    % Animal (cow): starts at (25.03, -0.96), wanders slowly
    animal_x = 25.03;
    animal_y = -0.96 + sin(tick * 0.1) * 0.5;  % small sinusoidal wander
    animal_conf = 0.92;

    % Camera intrinsics (same as test_pipeline_call.m)
    fx = 727.94;
    fy = 727.94;
    cx = 400;
    cy = 300;

    % --- Project 3D positions to 2D image space ---
    % Simple projection: (x_img, y_img) = (fx * y/x + cx, fy * z/x + cy)
    % assuming x is forward, y is left, z is up (CARLA/ego frame)

    % Pedestrian bounding box (0.5m width, 1.7m height, z=0.85m center)
    ped_bbox = project_3d_to_bbox(ped_x, ped_y, 0.85, 0.5, 1.7, fx, fy, cx, cy);

    % Animal bounding box (1.0m width, 1.2m height, z=0.6m center)
    animal_bbox = project_3d_to_bbox(animal_x, animal_y, 0.6, 1.0, 1.2, fx, fy, cx, cy);

    % Pack detections into output matrix
    % [x1, y1, x2, y2, confidence]
    det_mat(1, :) = [ped_bbox(1), ped_bbox(2), ped_bbox(3), ped_bbox(4), ped_conf];
    det_class_ids(1) = int32(0);  % 0 = pedestrian

    det_mat(2, :) = [animal_bbox(1), animal_bbox(2), animal_bbox(3), animal_bbox(4), animal_conf];
    det_class_ids(2) = int32(1);  % 1 = animal

    n_det = int32(2);

    % --- LiDAR points: cluster around the 3D object positions ---
    % Pedestrian cluster: 20 points around (ped_x, ped_y)
    ped_lidar = randn(20, 3) .* [0.1 0.1 0.5] + [ped_x ped_y 0.85];
    ped_lidar = single(ped_lidar);

    % Animal cluster: 20 points around (animal_x, animal_y)
    animal_lidar = randn(20, 3) .* [0.15 0.15 0.4] + [animal_x animal_y 0.6];
    animal_lidar = single(animal_lidar);

    % Background points: 200 random points ahead
    background_lidar = single(randn(200, 3) .* [3 3 2] + [20 0 1]);

    % Concatenate all LiDAR points
    all_lidar = [ped_lidar; animal_lidar; background_lidar];
    n_lidar_total = min(size(all_lidar, 1), MAX_LIDAR);

    lidar_xyz(1:n_lidar_total, :) = all_lidar(1:n_lidar_total, :);
    n_lidar = int32(n_lidar_total);

    % --- Ego state: stationary at origin, goal 25m ahead ---
    % (x, y, yaw, speed, goal_x, goal_y)
    ego_vec = [0.0; 0.0; 0.0; 5.0; 25.0; 0.0];
end

function bbox = project_3d_to_bbox(x, y, z, width, height, fx, fy, cx, cy)
    % PROJECT_3D_TO_BBOX Project a 3D box to 2D image bounding box
    % Simplified: project the 8 corners of the 3D box and find the AABB in image space

    % Box corners in ego frame (offset from center)
    corners_3d = [
        -width/2, -height/2;  % back-bottom
        -width/2,  height/2;  % back-top
         width/2, -height/2;  % front-bottom
         width/2,  height/2;  % front-top
    ];

    % Project each corner
    x1_img = inf;
    y1_img = inf;
    x2_img = -inf;
    y2_img = -inf;

    for i = 1:size(corners_3d, 1)
        dy = corners_3d(i, 1);
        dz = corners_3d(i, 2);

        px_img = fx * y / x + cx;  % same x for all corners (front-view assumption)
        py_img = fy * (z + dz) / x + cy;

        x1_img = min(x1_img, px_img);
        x2_img = max(x2_img, px_img);
        y1_img = min(y1_img, py_img);
        y2_img = max(y2_img, py_img);
    end

    bbox = [x1_img, y1_img, x2_img, y2_img];

    % Clamp to image bounds
    bbox(1) = max(0, bbox(1));
    bbox(2) = max(0, bbox(2));
    bbox(3) = min(800, bbox(3));
    bbox(4) = min(600, bbox(4));
end
