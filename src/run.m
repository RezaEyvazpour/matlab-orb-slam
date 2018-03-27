clear all;
close all;
clc;

%% User setup

imageDir = 'images';
imageExt = '.jpg';

%% Get feature vocabulary

% if no vocabulary exists
	% Load set of images
	% Create vocabulary
	% Save vocabulary
% else
	% Load vocabulary
% end


%% Setup Global variables

global Map;
Map.mapPoints = [];
	% Contains
	%	1) pose3D
	%	2) viewing direction (average of all viewing directions)
	%	3) Descriptor
	%	4) dmax and dmin, determined by invariance limits of SURF
	% and (I suggest) number of times observed (for viewing direction)
	%
	% Make a function to create new mapPoints

Map.keyFrames = [];
	% Contains
	%	1) Camera pose T_iw: rigid body transformation from world to camera
	%	2) Camera intrinsics: focal length, principal point)
	%	3) All features extracted in that frame, associated with or not to a map point
	%
	% Make a function to create new keyFrames

Map.covisibilityGraph = [];
	% An undirected graph where nodes are keyframes and edges are shared
	% observations of map points (at least theta)
	
Map.spanningTree = []; % Subset of the essential graph that contains edges with high covisability (theta_min)

global State;
State.mu = [0;0;0];
State.Sigma = zeros(length(State.mu));

global Params;
Params.theta = 15; % Number of shared observations a keyframe must have to be considered the same map points
Params.theta_min = 100; % Defines high covisability for spanning tree
Params.keyFramePercentOfBestScoreThreshold = 75; % bag of words returns keyframes that are more than this percentage of the best match
% ADD number of features to say we didn't lose localization
% ADD angle threshold between v and n
% ADD scale invariance region - perhaps set from data set

% Don't know if we'll like it, figured I'd ask - Audrow
global Debug;
Debug.displayFeaturesOnImages = false;


%% Run ORB-SLAM


imagesFiles = dir([imageDir, filesep, '*', imageExt]);
frames = cell([1 length(imagesFiles)]);
for i = 1:3
	frames{i} = imread([imagesFiles(i).folder, filesep, imagesFiles(i).name]);
end

for i = 1:length(frames)
	
	if iscell(frames)
		frame = frames{i};
	else
		frame = frames(i);
	end
	
	orb_slam(frame);
end