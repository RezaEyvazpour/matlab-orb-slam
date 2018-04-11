function local_mapping()

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

vs = Map.covisibilityGraph;
i=1;
check_map_point=[];
check_match_map_point=[];
while(1)
    keyframe1=vs.Views.ViewId(i);
    num_key_frames=vs.NumViews
    [descriptors_all, points_all] = vs.getAllMapPoints(keyframe1);
    map_points_keyframe1=points_all.Count;
    check_map_point=[check_map_point; [map_points_keyframe1, keyframe1]];
    count=0;
    for j=i+1:num_key_frames
        keyframe2=vs.Views.ViewId(j);
        connections = vs.getAllConnections();
        [conn_row, conn_col]=size(connections);
        check_conn=repmat([keyframe1,keyframe2],conn_row,1);
        Lia = ismember(connections,check_conn,'rows');
        if(~nnz(Lia))
            break;
        end
        [matches] = vs.getMatches(keyframe1, keyframe2);
        [match_map_points,~]=size(matches);


        check_match_map_point=[check_match_map_point; [match_map_points,keyframe1,keyframe2]];


        if ((match_map_points/map_points_keyframe1)*100 >= 0.9*map_points_keyframe1)
            count=count+1;
        end
        flag_del=0;
        if(count==3)
            vs = vs.deleteKeyFrame(keyframe1);
            i=i;
            flag_del=1;
            break;
        end
    end
    if (flag_del==0)
        i=i+1;
    end
    if (i>vs.NumViews)
        break;
    end
end

Map.covisibilityGraph=vs;

end

