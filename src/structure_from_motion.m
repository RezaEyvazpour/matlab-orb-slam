function structure_from_motion(frame_prev, frame_curr)

global Map
global State
global Params
global Debug

[features_prev, validPoints_prev] = extract_features(frame_prev);
[features_curr, validPoints_curr] = extract_features(frame_curr);

matchedIdx = matchFeatures(features_prev, features_curr, 'Unique', true, ...
	'Method', 'Approximate', 'MatchThreshold', .8);
    
matchedPoints1 = validPoints_prev(matchedIdx(:, 1));
matchedPoints2 = validPoints_curr(matchedIdx(:, 2));


[relativeOrient, relativeLoc, inlierIdx] = estimate_relative_motion(...
	matchedPoints1, matchedPoints2, Params.cameraParams);

k = Map.covisibilityGraph.NumViews + 1;
Map.covisibilityGraph = addView(Map.covisibilityGraph, k, features_curr, validPoints_curr, 'Points', validPoints_curr);
Map.covisibilityGraph = addConnection(Map.covisibilityGraph, k - 1, k, 'Matches', matchedIdx(inlierIdx,:));

prevPose = poses(Map.covisibilityGraph, k - 1);
prevOrientation = prevPose.Orientation{1};
prevLocation = prevPose.Location{1};

orientation = relativeOrient * prevOrientation;
location = prevLocation + relativeLoc * prevOrientation;
Map.covisibilityGraph = updateView(Map.covisibilityGraph, k, ...
	'Orientation', orientation, 'Location', location);

% Connect every past view to the current view
for i = 1:k-2
	connect_views(i,k)
end



% local BA
viewIds = max(k - 10, 1):k;
tracks = findTracks(Map.covisibilityGraph, viewIds);

camPoses = poses(Map.covisibilityGraph, viewIds);

xyzPoints = triangulateMultiview(tracks, camPoses, Params.cameraParams);

[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
	tracks, camPoses, Params.cameraParams, 'FixedViewId', 1, ...
	'PointsUndistorted', true);

Map.covisibilityGraph = updateView(Map.covisibilityGraph, camPoses);

end

function connect_views(viewIdx1, viewIdx2)

	global Map
	global State
	global Params
	global Debug

	features1 = Map.covisibilityGraph.Descriptors{viewIdx1};
	points1 = Map.covisibilityGraph.Points{viewIdx1};
	features2 = Map.covisibilityGraph.Descriptors{viewIdx2};
	points2 = Map.covisibilityGraph.Points{viewIdx2};

	matchedIdx = matchFeatures(features1, features2, 'Unique', true, ...
		'Method', 'Approximate', 'MatchThreshold', .8);

	matchedPoints1 = points1(matchedIdx(:, 1));
	matchedPoints2 = points2(matchedIdx(:, 2));


	[~, ~, inlierIdx] = estimate_relative_motion(...
		matchedPoints1, matchedPoints2, Params.cameraParams);

	Map.covisibilityGraph = addConnection(Map.covisibilityGraph, viewIdx1, viewIdx2, 'Matches', matchedIdx(inlierIdx,:));
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

end