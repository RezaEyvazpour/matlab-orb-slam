clear all;
close all;
clc;

%% User setup
seq = 4;

imageDir = num2str(seq, '../dataset/sequences/%02d/image_0');
imageExt = '.png';

imageFiles = dir([imageDir, '/*', imageExt]);

calibFile = num2str(seq, '../dataset/sequences/%02d/calib.txt');
cameraID = 0;

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
State.isInitialized = false;

global Params;
Params.theta = 15; % Number of shared observations a keyframe must have to be considered the same map points
Params.theta_min = 100; % Defines high covisability for spanning tree
Params.keyFramePercentOfBestScoreThreshold = 75; % bag of words returns keyframes that are more than this percentage of the best match
% ADD number of features to say we didn't lose localization
% ADD angle threshold between v and n
% ADD scale invariance region - perhaps set from data set

Params.cameraParams = load_camera_params(calibFile, cameraID);

% Don't know if we'll like it, figured I'd ask - Audrow
global Debug;
Debug.displayFeaturesOnImages = false;

%% Run ORB-SLAM

for i = 1:(length(imageFiles) - 1)
    prevFrame = imread([imageFiles(i).folder, '/', imageFiles(i).name]);
    currFrame = imread([imageFiles(i + 1).folder, '/', imageFiles(i + 1).name]);
	orb_slam(prevFrame, currFrame);
end