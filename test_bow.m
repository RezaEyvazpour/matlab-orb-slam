function test_bow(seq)

if ~exist('seq', 'var')
    seq = 4;
end

data = load('dataset/codewords.mat');

kdtree_mdl = KDTreeSearcher(data.codewords);

num_codewords = size(data.codewords, 1);

imageFiles = dir(num2str(seq, 'dataset/sequences/%02d/image_0/*.png'));

numImages = min(500, length(imageFiles));

bow = zeros(numImages, num_codewords);
for k = 1:numImages
    tic
    frame = imread([imageFiles(k).folder, '/', imageFiles(k).name]);
    
    points = detectSURFFeatures(frame);
    [features, ~] = extractFeatures(frame, points);
    
    bow(k, :) = calc_bow_repr(features, kdtree_mdl, num_codewords);
    toc
end

figure(1)
clf()

contourf(bow', 64, 'edgecolor', 'none')

end

function repr = calc_bow_repr(features, kdtree_mdl, num_codewords)
idx = knnsearch(kdtree_mdl, features);

repr = histcounts(idx, 1:(num_codewords + 1));
repr = repr / sum(repr);
end

