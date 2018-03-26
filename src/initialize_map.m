function isInitialized = initialize_map()

global Map
global State
global Params
global Debug
global Data

% Calculate features: (frames) -> features
% Match features between frames
% Get homography and fundamental matrix - RANSAC
% Calculate ratio to decide which we use, homography and fundamental matrix
% Motion recovery: parallax - See reference 23
% Bundle adjustment - see matlab's bundleAdjustment function

tic

features1 = Data.Surf.features{Data.frameIdx - 1};
points1 = Data.Surf.points{Data.frameIdx - 1};
features2 = Data.Surf.features{Data.frameIdx};
points2 = Data.Surf.points{Data.frameIdx};

matchedIdx = matchFeatures(features1, features2);
matchedPoints1 = points1(matchedIdx(:, 1), :);
matchedPoints2 = points2(matchedIdx(:, 2), :);

F = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, 'Method', 'RANSAC');

scoreF = 0;
for k = 1:size(matchedIdx, 1)
    x1 = matchedPoints1.Location(k, :);
    x2 = matchedPoints2.Location(k, :);
    
    l2 = F * [x1, 1]';
    num2 = [x2, 1] * l2;
    d2 = num2^2 / (l2(1)^2 + l2(2)^2);
    if d2 < 3.84
        scoreF = scoreF + 3.84 - d2;
    end
    
    l1 = [x2, 1] * F;
    num1 = l1 * [x1, 1]';
    d1 = num1^2 / (l1(1)^2 + l1(2)^2);
    if d1 < 3.84
        scoreF = scoreF + 3.84 - d1;
    end
end

toc
isInitialized = false;

end