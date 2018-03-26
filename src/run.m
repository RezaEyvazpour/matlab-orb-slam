clear all;
close all;
clc;

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

Params.maxRansacIter = 100;


% Don't know if we'll like it, figured I'd ask - Audrow
global Debug;
Debug.displayFeaturesOnImages = false;

%% User setup
global Data
Data.dir = '../dataset/sequences/04/image_0';
Data.ext = '.png';
Data.height = 370;
Data.width = 1226;

Data.files = dir([Data.dir, '/*', Data.ext]);

Data.numFrames = length(Data.files);

Data.prevFrame = imread([Data.files(1).folder, '/', Data.files(1).name]);
Data.currFrame = imread([Data.files(2).folder, '/', Data.files(2).name]);
Data.frameIdx = 2;

Data.Surf.features = {};
Data.Surf.points = {};

points = detectSURFFeatures(Data.prevFrame);
[Data.Surf.features{1}, Data.Surf.points{1}] = extractFeatures(Data.prevFrame, points);

%% Run ORB-SLAM


for i = 1:(Data.numFrames - 1)
	orb_slam();
end