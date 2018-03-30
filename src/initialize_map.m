function isInitialized = initialize_map(matchedPoints1, matchedPoints2)

global Map
global State
global Params
global Debug


% Calculate features: (frames) -> features
% Match features between frames
% Get homography and fundamental matrix - RANSAC
% Calculate ratio to decide which we use, homography and fundamental matrix
% Motion recovery: parallax - See reference 23
% Bundle adjustment - see matlab's bundleAdjustment function

[F, inlierIdx] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC');

inlierPoints1 = matchedPoints1(inlierIdx, :);
inlierPoints2 = matchedPoints2(inlierIdx, :);

[orient, loc] = relativeCameraPose(F, Params.cameraParams, inlierPoints1, inlierPoints2);

camMatrix1 = cameraMatrix(Params.cameraParams, eye(3), zeros(1, 3));

[R, t] = cameraPoseToExtrinsics(orient, loc);
camMatrix2 = cameraMatrix(Params.cameraParams, R, t);

points3D = triangulate(inlierPoints1, inlierPoints2, camMatrix1, camMatrix2);

figure(1)
clf()
scatter3(points3D(:, 1), points3D(:, 2), points3D(:, 3), '.')
hold on
plotCamera('size', 0.5, 'color', 'r')
plotCamera('size', 0.5, 'color', 'b', 'location', loc, 'orientation', orient)
axis equal
axis([-50, 50, -20, 20, -10, 100])

isInitialized = false;

end