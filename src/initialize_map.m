function isInitialized = initialize_map()

global Map
global State
global Params
global Debug

% tic
% % Match features between frames
% features1 = Data.Surf.features{Data.frameIdx - 1};
% points1 = Data.Surf.points{Data.frameIdx - 1};
% features2 = Data.Surf.features{Data.frameIdx};
% points2 = Data.Surf.points{Data.frameIdx};
% 
% matchedIdx = matchFeatures(features1, features2);
% matchedPoints1 = points1(matchedIdx(:, 1), :);
% matchedPoints2 = points2(matchedIdx(:, 2), :);
% 
% % Get fundamental matrix - RANSAC
% F = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC');

% Calculate ratio to decide which we use, homography and fundamental matrix -- Follow MATLAB's structure from motion tutorial
% https://www.mathworks.com/help/vision/examples/structure-from-motion-from-multiple-views.html
% Camera calib information from dataset

% Motion recovery: parallax - See reference 23 -- USE MATLAB TO DO THIS
% Bundle adjustment - see matlab's bundleAdjustment function
% See above tutorial

% toc
isInitialized = false;

end