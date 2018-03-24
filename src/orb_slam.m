function orb_slam(frame)

global Map
global State
global Params
global Debug

persistent mapIsNotInitialized
if isempty(mapIsNotInitialized)
	initialize_map()
	mapIsNotInitialized = true;
end

keyFrame = tracking(frame);
local_mapping(keyFrame);
loop_closing(keyFrame);
end

