clc
%%
%data = load('0406_seq00.mat');
data = load('mydata.mat');
vs = data.Map.covisibilityGraph;

cameraParams = load_camera_params('../dataset/sequences/00/calib.txt', 0);

%%
numPoses = size(vs.Views, 1);
poses = zeros(numPoses, 7);
for k = 1:numPoses
    R = vs.Views.Orientation{k};
    t = vs.Views.Location{k};
    
    poses(k, 1) = log(det(R));
    poses(k, 2:4) = rotationMatrixToVector(R);
    poses(k, 5:7) = t;
end

%%
numConnections = size(vs.Connections, 1);
odom = zeros(numConnections, 7);
for k = 1:numConnections
    R = data.Map.vOdom.rot{k};
    t = data.Map.vOdom.trans{k};
    
    odom(k, 1) = log(det(R));
    odom(k, 2:4) = rotationMatrixToVector(R);
    odom(k, 5:7) = t;
end

%%
%{
f = @(xx)calc_cost(xx, odom, vs.Connections);
options = optimoptions(@lsqnonlin, ...
    'Algorithm', 'levenberg-marquardt', ...
    'FunctionTolerance', 1e-3, ...
    'MaxIterations', 100, ...
    'Display', 'iter');
p0 = reshape(poses, [numel(poses), 1]);
poses2 = lsqnonlin(f, p0, -inf, inf, options);
%}
%%
%poses2 = reshape(poses2, [numPoses, 7]);
figure(10)
clf()
idx1 = vs.Connections.ViewId1;
idx2 = vs.Connections.ViewId2;
plot([poses(idx1, 5)'; poses(idx2, 5)'], ...
    [poses(idx1, 7)'; poses(idx2, 7)'], 'r')
hold on

plot(poses(:, 5), poses(:, 7), 'k', 'linewidth', 2)
axis equal


%plot(poses2(:, 5), poses2(:, 7))
%{
for k = 1:numPoses
    R = vs.Views.Orientation{k}';
    ex = R(:, 1) * 0.5;
    ez = R(:, 3) * 0.5;
    t = vs.Views.Location{k};
    
    plot(t(1) + [0, ex(1)], t(3) + [0, ex(3)], 'r')
    plot(t(1) + [0, ez(1)], t(3) + [0, ez(3)], 'b')
end
%}

hold off