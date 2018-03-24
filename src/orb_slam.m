function orb_slam(frame)

global Map
global State
global Params
global Debug

keyFrame = tracking(frame);
local_mapping(keyFrame);
loop_closing(keyFrame);
end

