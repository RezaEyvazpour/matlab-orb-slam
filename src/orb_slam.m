function orb_slam(frame_curr)

global Map
global State
global Params
global Debug

persistent frame_prev
if ~isempty(frame_prev)
	
    structure_from_motion(frame_prev, frame_curr);
	
	keyFrame = tracking(frame_prev, frame_curr);
	local_mapping(keyFrame);
	loop_closing(keyFrame);
else
	% Initialize
	[descriptors, points] = extract_features(frame_curr);
	Map.covisibilityGraph = addView(Map.covisibilityGraph, 1, descriptors, 'Points', points, ...
        'Orientation', eye(3), 'Location', zeros(1, 3));
    
    Map.bow(1, :) = calc_bow_repr(descriptors, Params.kdtree, Params.numCodewords);
end

	frame_prev = frame_curr;
end

