function test_bow(seq)

if ~exist('seq', 'var')
    seq = 4;
end

data = load('dataset/codewords.mat');

kdtree_mdl = KDTreeSearcher(data.codewords);

num_codewords = size(data.codewords, 1);

%%

if exist(num2str(seq, 'dataset/sequences/%02d/image_0.mat'), 'file')
    data = load(num2str(seq, 'dataset/sequences/%02d/image_0.mat'));
    features = data.features;
    points = data.validPoints;
    
    numImages = length(points);
    
    bow = zeros(numImages, num_codewords);
    parfor k = 1:numImages       
        bow(k, :) = calc_bow_repr(features{k}, kdtree_mdl, num_codewords);
    end
else
    imageFiles = dir(num2str(seq, 'dataset/sequences/%02d/image_0/*.png'));
    
    numImages = min(1000, length(imageFiles));
    
    bow = zeros(numImages, num_codewords);
    features = cell(numImages, 1);
    points = cell(numImages, 1);
    parfor k = 1:numImages
        frame = imread([imageFiles(k).folder, '/', imageFiles(k).name]);
        
        points{k} = detectSURFFeatures(frame);
        [features{k}, points{k}] = extractFeatures(frame, points{k});
        
        bow(k, :) = calc_bow_repr(features{k}, kdtree_mdl, num_codewords);
    end
end

%%
loop_closure_proposal = zeros(numImages, 1);
num_frames_apart = 50;

for i = (num_frames_apart + 1):numImages
    d2 = hist_diff(bow(1:(i - num_frames_apart), :), bow(i, :));
    
    [~, j] = min(d2);
    
    matchedIdx = matchFeatures(features{i},features{j}, 'unique', true);
    
    ni = length(features{i});
    nj = length(features{j});
    matchRatio = 2 * numel(matchedIdx) / (ni + nj);
    
    if matchRatio > 0.5
        loop_closure_proposal(i) = j;
    end
end

%%
poses = load_gt_poses(seq);
x = poses(1:numImages, 1, 4);
z = poses(1:numImages, 3, 4);

figure(1)
clf()
plot(x, z, 'k.-')
hold on
axis equal
axis(axis() + [-10, 10, -10, 10])

for i = 1:numImages
    j = loop_closure_proposal(i);
    if j > 0
        plot([x(i), x(j)], [z(i), z(j)], 'r')
    end
end
end

%%
function repr = calc_bow_repr(features, kdtree_mdl, num_codewords)
idx = knnsearch(kdtree_mdl, features);

repr = histcounts(idx, 1:(num_codewords + 1));
repr = repr / sum(repr);
end

%%
function d2 = hist_diff(h1, h2)
d2 = sum((h1 - h2).^2 ./ (h1 + h2 + 1e-6), 2);
end

%%
function poses = load_gt_poses(seq)
%   Load ground truth poses as a sequence of homogeneous transforms.
%
%   Input:
%   -   sequence
%       int
%       A sequence ID in 'dataset/poses'.
%
%   Output:
%   -   poses
%       [N, 3, 4] double
%       poses(i, :, :) = [R(i), t(i)], where R(i) is a 3x3 rotation matrix
%       and t(i) is a 3x1 translation vector.

poses = dlmread(num2str(seq, 'dataset/poses/%02d.txt'));
poses = reshape(poses, [size(poses, 1), 4, 3]);
poses = permute(poses, [1, 3, 2]);
end
