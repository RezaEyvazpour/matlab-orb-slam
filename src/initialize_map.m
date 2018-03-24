function isInitialized = initialize_map()

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

isInitialized = false;

end