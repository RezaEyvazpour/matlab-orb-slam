function orb_slam(prevFrame, currFrame)

global Map
global State
global Params
global Debug


[prevFeatures, prevPoints] = extract_features(prevFrame);
[currFeatures, currPoints] = extract_features(currFrame);

matchedIdx = matchFeatures(prevFeatures, currFeatures);
prevMatchedPoints = prevPoints(matchedIdx(:, 1), :);
currMatchedPoints = currPoints(matchedIdx(:, 2), :);

if ~State.isInitialized
    State.isInitialized = initialize_map(prevMatchedPoints, currMatchedPoints);
else
    keyFrame = tracking(prevFrame, currFrame);
    local_mapping(keyFrame);
    loop_closing(keyFrame);
end

end

