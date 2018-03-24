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
Map.keyFrames = [];
Map.covisibilityGraph = [];
Map.spanningTree = []; % Subset of the essential graph that contains edges with high covisability (theta_min)

global State;
State.mu = [0;0;0];
State.Sigma = zeros(length(State.mu));

global Params;
Params.theta = 15; % Number of shared observations a keyframe must have to be considered the same map points
Params.theta_min = 100; % Defines high covisability for spanning tree
Params.keyFramePercentOfBestScoreThreshold = 75; % bag of words returns keyframes that are more than this percentage of the best match

% Don't know if we'll like it, figured I'd ask - Audrow
global Debug;
Debug.displayFeaturesOnImages = true;


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