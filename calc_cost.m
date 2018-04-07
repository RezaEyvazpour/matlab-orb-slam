function f = calc_cost(poses, odom, connections)
np = numel(poses) / 7;
poses = reshape(poses, [np, 7]);

nc = numel(odom) / 7;
odom = reshape(odom, [nc, 7]);

f = zeros(nc, 1);
To = eye(4);
T1 = eye(4);
T2 = eye(4);
for k = 1:nc
    To(1:3, 1:3) = rotationVectorToMatrix(odom(k, 2:4));
    To(1:3, 1:3) = To(1:3, 1:3) * exp(odom(k, 1));
    To(4, 1:3) = odom(k, 5:7);
    
    idx1 = connections.ViewId1(k);
    T1(1:3, 1:3) = rotationVectorToMatrix(poses(idx1, 2:4));
    T1(1:3, 1:3) = T1(1:3, 1:3) * exp(poses(idx1, 1));
    T1(4, 1:3) = poses(idx1, 5:7);
    
    idx2 = connections.ViewId2(k);
    T2(1:3, 1:3) = rotationVectorToMatrix(poses(idx2, 2:4));
    T2(1:3, 1:3) = T2(1:3, 1:3) * exp(poses(idx2, 1));
    T2(4, 1:3) = poses(idx2, 5:7);
    
    deltaT = T1 * inv_SIM3(T2);
    if abs(double(idx1) - double(idx2)) > 1
        deltaT(4, 1:3) = deltaT(4, 1:3) / norm(deltaT(4, 1:3), 2);
    end

    f(k) = norm(logm(To * deltaT), 'fro');
    
%     fprintf('(%4d, %4d), cost = %.4f\n', idx1, idx2, f(k))
%     disp(T2)
%     disp(T_odom)
end
end