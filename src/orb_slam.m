function orb_slam(frame_curr)

global Map
global State
global Params
global Debug

persistent isMapInitialized
if isempty(isMapInitialized)
    isMapInitialized = false;
end

persistent frame_prev
if ~isempty(frame_prev)
    if isMapInitialized
        frame_ref = frame_prev;
        isMapInitialized = initialize_map(frame_ref, frame_curr);
    else
        keyFrame = tracking(frame_prev, frame_curr);
        local_mapping(keyFrame);
        loop_closing(keyFrame);
    end
end
frame_prev = frame_curr;
end

