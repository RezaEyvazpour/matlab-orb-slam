function keyFrame = tracking(frame_prev, frame_cur)

global Map
global State
global Params
global Debug

keyFrame = frame_cur;

[features_prev, validPoints_prev] = extract_features(frame_prev);
[features_cur, validPoints_cur] = extract_features(frame_cur);

% Initial pose estimation with constant velocity model
%   Calculate homogeneous frame - keep this in Map
% If number of matched points is less than threshold
%   Use a wider search 
%   If still not enough
%       Relocalize
% Track local map
%   Pick K1 (keyframes that we're going to use)
%   Pick K2: neighboring keyframes of K1
%   Pick K_ref shares the most map points with the current frame - use in Step E
%   K = K1 U K2
%   Reproject union of points to frame_curr
%       Discard points outside of image
%       Discard points with greater angle than 60 degrees
%       Discard if it outside the scale invariance region
%   Normalize scale: d/dmin
%   Optimize to find best pose (back slash)
% Decide if keyframe or not - check conditions in the paper

end