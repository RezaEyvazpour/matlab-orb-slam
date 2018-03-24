function keyFrame = tracking(frame)

global Map
global State
global Params
global Debug

keyFrame = frame;

[features, validPoints] = extract_features(frame);

% Pose estimation: (features, validPoints) -> new mu and Sigma?
% 	Bundle adjustment? See Matlab's bundleAdjustment
%		- Requires camera calibration
%	If lost, place recognition occurs
% Track local map
%	Local visibility map
%	Optimize map with matches from covisability graph
% New KeyFrame decision

end

function [features, validPoints] = extract_features(frame)

	global Debug

	if isColor(frame)
		frame = rgb2gray(frame);
	end
	points = detectSURFFeatures(frame);
	[features, validPoints] = extractFeatures(frame, points);
	
	if Debug.displayFeaturesOnImages
		figure; imshow(frame); hold on; plot(validPoints); hold off;
	end
end

function result = isColor(image)
	result = size(image, 3) == 3;
end