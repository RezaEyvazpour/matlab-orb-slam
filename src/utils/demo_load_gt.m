clc
close all
%%
for k = 1:10
    figure(k)
    poses = load_gt_poses(k);
    x = squeeze(poses(:, 1, 4));
    z = squeeze(poses(:, 3, 4));
    plot(x, z)
    axis equal
end