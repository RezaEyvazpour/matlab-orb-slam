function orb_slam()

global Map
global State
global Params
global Debug
global Data


if ~State.isInitialized
    State.isInitialized = initialize_map();
else
    keyFrame = tracking(Data.prevFrame, Data.currFrame);
    local_mapping(keyFrame);
    loop_closing(keyFrame);
end

Data.frameIdx = Data.frameIdx + 1;
Data.prevFrame = Data.currFrame;
Data.currFrame = imread([Data.files(Data.frameIdx).folder, ...
    '/', Data.files(Data.frameIdx).name]);
end

