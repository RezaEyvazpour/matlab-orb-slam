function test_sfm(seq)

if ~exist('seq', 'var')
    seq = 4;
end

imageDir = num2str(seq, 'dataset/sequences/%02d/image_0');

imageFiles = dir([imageDir, '/*.png']);
imageFiles = imageFiles(1:5:end);

calibFile = num2str(seq, 'dataset/sequences/%02d/calib.txt');
cameraID = 0;

cameraParams = load_camera_params(calibFile, cameraID);

numImages = min(150, length(imageFiles));

%%
features = cell(numImages, 1);
points = cell(numImages, 1);

frame = imread([imageFiles(1).folder, '/', imageFiles(1).name]);
pt = detectSURFFeatures(frame);
[features{1}, points{1}] = extractFeatures(frame, pt);

%%
vSet = viewSet();
vSet = addView(vSet, 1, 'Points', points{1}, ...
    'Orientation', eye(3), 'Location', zeros(1, 3));

figure(1)
clf()
hold on

for k = 2:numImages
    disp(k)
    
    frame = imread([imageFiles(k).folder, '/', imageFiles(k).name]);
    pt = detectSURFFeatures(frame);
    [features{k}, points{k}] = extractFeatures(frame, pt);
    
    matchedIdx = matchFeatures(features{k - 1}, features{k});
    
    matchedPoints1 = points{k - 1}(matchedIdx(:, 1));
    matchedPoints2 = points{k}(matchedIdx(:, 2));
    
    [relativeOrient, relativeLoc, inlierIdx] = estimate_relative_motion(...
        matchedPoints1, matchedPoints2, cameraParams);
    
    vSet = addView(vSet, k, 'Points', points{k});
    
    vSet = addConnection(vSet, k - 1, k, 'Matches', matchedIdx(inlierIdx,:));
    
    prevPose = poses(vSet, k - 1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation = prevPose.Location{1};
    
    orientation = relativeOrient * prevOrientation;
    location = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, k, ...
        'Orientation', orientation, 'Location', location);
    
    % local BA
    viewIds = max(k - 10, 1):k;
    tracks = findTracks(vSet, viewIds);
    
    camPoses = poses(vSet, viewIds);
    
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);
    
    vSet = updateView(vSet, camPoses);
    
end

% full BA
tracks = findTracks(vSet);
camPoses = poses(vSet);
xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

plotCamera(camPoses, 'Size', 0.2);
grid on

validIdx = sqrt(xyzPoints(:, 1).^2 + xyzPoints(:, 2).^2 + xyzPoints(:, 3).^2) < 100;
validIdx = validIdx & (xyzPoints(:, 3) > 0);

pcshow(xyzPoints(validIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

end

function cameraParams = load_camera_params(path, camera_id)

P = dlmread(path, ' ', 0, 1);
P = reshape(P(camera_id + 1, :), [4, 3]);

K = P(1:3, 1:3);
t = P(4, :) / K;

cameraParams = cameraParameters('WorldUnits', 'm',...
    'IntrinsicMatrix', K, ...
    'RotationVectors', zeros(1, 3), ...
    'TranslationVectors', t);

end

function [orient, loc, inlierIdx] = estimate_relative_motion(matchedPoints1, matchedPoints2, cameraParams)
[F, inlierIdx] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC');

inlierPoints1 = matchedPoints1(inlierIdx, :);
inlierPoints2 = matchedPoints2(inlierIdx, :);

for i = 1:100
    [orient, loc, validPointsFraction] = relativeCameraPose(...
        F, cameraParams, inlierPoints1, inlierPoints2);
    
    if validPointsFraction > 0.8
        return
    end
end

orient = eye(3);
loc = zeros(1, 3);

% camMatrix1 = cameraMatrix(cameraParams, eye(3), zeros(1, 3));
%
% [R, t] = cameraPoseToExtrinsics(orient, loc);
% camMatrix2 = cameraMatrix(cameraParams, R, t);
%
% points3D = triangulate(inlierPoints1, inlierPoints2, camMatrix1, camMatrix2);
end
