clear all;
close all;

numMotions = 10;
motion_0 = generate_rough_loop(numMotions, pi/(numMotions));
	  
start_0 = [0, 0];
angle_0 = pi/2;
constraints = [1,numMotions+1]; % map pt 1 -> end

% CMA
initStdDev = 0.1;
maxNbrItrs = 200;
options = CMAOptions();
options = options.set({'sigma', initStdDev,'max iterations', maxNbrItrs,'should plot error?',true});
func_angle = @(x) cost_fn_from_angle(start_0, angle_0, motion_0, constraints, x);
x_0 = 0.2*rand([1,size(motion_0,1)]);
[x, feval, exitFlag] = cma(func_angle, x_0, options);


motion_update = add_angles_to_motion(motion_0, x);
oldPts = get_pts(start_0, angle_0, motion_0);
newPts = get_pts(start_0, angle_0, motion_update);
figure;
plot_pts_and_constraints(oldPts, constraints)
figure;
plot_pts_and_constraints(newPts, constraints)

%% Functions 

function cost = cost_fn_from_angle(start_0, angle_0, motion_0, constraints, angles_to_add)
	pts_0 = get_pts(start_0, angle_0, motion_0);
	
	motion_test = add_angles_to_motion(motion_0, angles_to_add);
	pts_test = get_pts(start_0, angle_0, motion_test);
	cost = cost_fn(pts_0, pts_test, constraints);
end

function cost = cost_fn_from_radius(start_0, angle_0, motion_0, constraints, radius_to_add)
	pts_0 = get_pts(start_0, angle_0, motion_0);
	
	motion_test = add_radius_to_motion(motion_0, radius_to_add);
	pts_test = get_pts(start_0, angle_0, motion_test);
	cost = cost_fn(pts_0, pts_test, constraints);
end

function motion = add_angles_to_motion(motion, angles)
	assert(all(size(motion(:,1)) == size(angles')));
	motion(:,1) = motion(:,1) + angles';
end

function motion = add_radius_to_motion(motion, radius)
	assert(all(size(motion(:,2)) == size(radius')));
	motion(:,2) = motion(:,2) + radius';
end

function cost = cost_fn(pts_0, pts_test, constraints)
	constraints_err = get_constraint_error(pts_test, constraints);
	dist_err = get_avg_distance_error(pts_0, pts_test);
	angles_err_avg = get_avg_sqr_angle_error(pts_0, pts_test);
	angles_err_max = get_max_sqr_angle_error(pts_0, pts_test);
	
	% Note the hand tuned gains
	cost = (5*constraints_err+1)^2 + (0*dist_err+1)^2 + (30*angles_err_avg) + (10*angles_err_max)^2;
end

function constraints_err = get_constraint_error(pts, constraints)
	constraints_err = 0;
	for i = 1:size(constraints_err,1)
		constraints_err = constraints_err + norm(pts(constraints(i,1),:) - pts(constraints(i,2),:));
	end
end

function dists_err = get_avg_distance_error(pts_actual, pts_test)
	dists_actual = get_distances_between_pts(pts_actual);
	dists_test = get_distances_between_pts(pts_test);
	dists_err = sum((dists_actual-dists_test).^2)/(size(pts_test,1)-1);
end

function angles_err = get_avg_sqr_angle_error(pts_actual, pts_test)
	angles_actual = get_angles_between_pts(pts_actual);
	angles_test = get_angles_between_pts(pts_test);
	angles_err = sum((angles_actual-angles_test).^2)/(size(pts_test,1)-2);
end

function angles_err = get_max_sqr_angle_error(pts_actual, pts_test)
	angles_actual = get_angles_between_pts(pts_actual);
	angles_test = get_angles_between_pts(pts_test);
	angles_err = max((angles_actual-angles_test).^2);
end

function dists = get_distances_between_pts(pts)
	assert(size(pts,2) == 2);
	assert(size(pts,1) > 1);
	
	numPts = size(pts,1);
	numDists = numPts-1;
	dists = zeros([numDists, 1]);
	for i = 1:numDists
		dists(i) = norm(pts(i+1,:)-pts(i,:));
	end
end

function angles = get_angles_between_pts(pts)
	assert(size(pts,2) == 2);
	assert(size(pts,1) > 2);
	numPts = size(pts,1);
	numAngles = numPts-2;
	angles = zeros([numAngles, 1]);
	for i = 1:numAngles
		angles(i) = get_angle_between_three_pts(pts(i:i+2,:));
	end
end

function angle = get_angle_between_three_pts(pts)
	assert(all(size(pts) == [3,2]))
	P1 = pts(1,:);
	P2 = pts(2,:);
	P3 = pts(3,:);
	X = 1;
	Y = 2;
	angle = atan2(P3(Y)-P2(Y), P3(X)-P2(X)) - atan2(P1(Y)-P2(Y), P1(X)-P1(X));
end

function pts = get_pts(start, angle_0, motion)
	numMotions = size(motion,1);
	pts = zeros([numMotions+1 2]);
	
	pts(1,:) = start;
	angle = angle_0;
	for i = 1:numMotions
		angle = angle + motion(i,1);
		radius = motion(i,2);
		pts(i+1,:) = [pts(i,1)+radius*cos(angle), pts(i,2)+radius*sin(angle)];
	end
end

% Plotting

function motions = generate_rough_loop(numMotions, noise_param)
	motions = repmat([0, 1],[numMotions, 1]);
	angle = -3/2*pi/numMotions;
	motions(2:end,1) = angle+noise_param*(rand([numMotions-1, 1])-.5);
end

function plot_pts_and_constraints(pts, constraints)
	hold on
	plot(pts(:,1),pts(:,2),'b')
	plot(pts(constraints,1),pts(constraints,2),'r--')
% 	xlim([-1, 2])
% 	ylim([-1, 2])
	hold off
end