function local_mapping(keyFrame)

global Map
global State
global Params
global Debug

% KeyFrame insertion
%   Update covisability graph: 
%       add keyframe as a node
%       update edge weights that node is connected
%   Update spanning tree graph
%       Find frame with highest number of matching features
%   Compute bag of words representation using precomputed vocabulary
% Recent MapPoints culling
%   Pass conditions
%   QUESTION: 25% of predicted poses means what?
% New points creation
%   Look for points that are not matched
%   Check that those points satisfy 
%       Epipolar constraints
%       Positive depth
%       Reprojection error
%       Scale consistency
%    Add to Map.mapPoints
% Local BA
%   Wrap matlabs implementation
% Local KeyFrames culling
%   For all keyframes
%       Check if 90% of map points are present in at least 3 other key
%       frames at the same or finer scale

end

