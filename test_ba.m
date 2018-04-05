clc
%%
data = load('0405_run1.mat');

vs = data.Map.covisibilityGraph;

cameraParams = load_camera_params('dataset/sequences/00/calib.txt', 0);

figure(10)
clf()

%%
tracks = vs.findTracks();
camposes = vs.poses();
xyz = triangulateMultiview(tracks, camposes, cameraParams);

traj1 = cell2mat(camposes.Location);
x1 = traj1(:, 1);
z1 = traj1(:, 3);

[xyz, camposes, reprojectionErrors] = bundleAdjustment(xyz, ...
    tracks, camposes, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true, ...
    'RelativeTolerance', 1e-8, ...
    'MaxIterations', 500, ...
    'AbsoluteTolerance', 0.1);

traj2 = cell2mat(camposes.Location);
x2 = traj2(:, 1);
z2 = traj2(:, 3);

%%

plot(x1, z1, 'b', x2, z2, 'r')
axis equal
