% Define global variables here? 
clear all; clc; 

qs = [0;0];
pf = [1.1081;1.5825];
obstacles = [-1;1;0.5];
rrt_2dof(qs, pf, obstacles) 

function rrt_2dof(qs, pf, obstacles) 
% qs as start position in configuration space
% pf as end position in workspace
% Obstacles defined by x, y, rad

num_iter = 0; 
max_iter = 2500; 
max_step = deg2rad(2); 
% tolerance = 0.1; % in terms of workspace distance

L1 = 1; 
L2 = 1; 

figure(1); hold on; grid on;

qf = inverseKinematics(pf); 
% qf = [deg2rad(40); deg2rad(40)];
eeLoc = zeros(2,1);
eeLoc(1) = L1*cos(qs(1)) + L2*cos(qs(1)+qs(2));
eeLoc(2) = L1*sin(qs(1)) + L2*sin(qs(1)+qs(2));

plot(eeLoc(1), eeLoc(2), 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');

plot(pf(1), pf(2), 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');

num_obstacles = size(obstacles);
num_obstacles = num_obstacles(2);

for i=1:num_obstacles
    plot(obstacles(1), obstacles(2), 'bo', 'MarkerSize',5, 'MarkerFaceColor','b');
end

q1_min = deg2rad(0);
q1_max = deg2rad(90);

q2_min = deg2rad(0);
q2_max = deg2rad(90);

%variable for storing vertices
route = [qs];
%variable for storing edges
edges = [1];
q_rand = zeros(2,1);

while (num_iter <= max_iter)
   q_rand(1) = (q1_max - q1_min)*rand + q1_min; 
   q_rand(2) = (q2_max - q2_min)*rand + q2_min; 
   
   if inObs(q_rand, obstacles) == 0
       num_iter = num_iter + 1; 
       [q_near, near_ind] = findNearestVert(q_rand, route); 
       
       if norm(q_near - qf) < max_step
           route = horzcat(route, qf); 
           edges = horzcat(edges, near_ind);
           break; 
       end       
       % define a q_new that will be appended
           
       q_new = (q_rand - q_near)/(norm(q_rand - q_near))*max_step + q_near;
       
       if edge_collision(obstacles, q_near, q_new) == 1
           num_iter = num_iter - 1; 
           continue; 
       end
       route = horzcat(route, q_new); 
       edges = horzcat(edges, near_ind);  
       
   end
   
end

   % now sort out a trajectory here
   num_nodes = size(route);
   num_nodes = num_nodes(2);
   
   traj = [route(:,num_nodes)];
   q_curr = route(:,num_nodes);
   next_ind = edges(num_nodes); 
   
   while ~isequal(q_curr, qs)
       traj = horzcat(q_curr, traj);
       q_curr = route(:, next_ind);
       next_ind = edges(next_ind);
   end
   
   % Convert from joint to workspace
   traj_len = size(traj);
   traj_len = traj_len(2); 
   
   pos_traj = [];
   
   for i=1:traj_len
        x = L1*cos(traj(1,i)) + L2*cos(traj(1,i) + traj(2,i));
        y = L1*sin(traj(1,i)) + L2*sin(traj(1,i) + traj(2,i));
        curr_pos = [x; y];
        pos_traj = horzcat(pos_traj, curr_pos);
   end
   
   plot(pos_traj(1, :), pos_traj(2,:));
end

function col = inObs(pos,obs)

col = 0;
L1 = 1;
L2 = 1;
q = inverseKinematics(pos);
eeLoc(1,1) = L1*cos(q(1)) + L2*cos(q(1)+q(2));
eeLoc(2,1) = L1*sin(q(1)) + L2*sin(q(1)+q(2));
jointLoc(1,1) = L1*cos(q(1));
jointLoc(2,1) = L1*sin(q(1));
baseLoc(1,1) = 0;
baseLoc(2,1) = 0;

for i=1:length(obs(1,:))
    %checking ee and joint collisions
    if norm(eeLoc - obs(1:2,i)) < obs(3,i) 
        col = 1; 
        return; 
    elseif norm(jointLoc - obs(1:2,i)) < obs(3,i) 
        col = 1; 
        return; 
    end
end
col = 0; 
end

function col = edge_collision(obs,p1,p2)
col = 0;
resolution = 25;
m = (p2(2) - p1(2))/(p2(1) - p1(1));
xdiff = p2(1) - p1(1);
for i=2:(resolution-1)
    pos(1,1) = p1(1) + i*(xdiff/resolution);
    pos(2,1) = p1(2) + m*(pos(1,1)-p1(1));
    col = inObs(pos,obs);
    if col == 1
        return;
    end
end
end

function [q_near, ind_near] = findNearestVert(q_curr, route)

    numPoints = size(route);
    numPoints = numPoints(2); 
    
    min_dist = intmax; 
    q_near = zeros(2,1);
    
    for i=1:numPoints
        if norm(q_curr - route(:,i)) < min_dist
            q_near = route(:,i);
            ind_near = i; 
            min_dist = norm(q_curr - route(:,i)); 
        end
    end
end

function q = inverseKinematics(pos)
    x = pos(1);
    y = pos(2); 
    
    L1 = 1; 
    L2 = 1; 
    
    q = zeros(2,1);
    % Stand in for L1 and L2
    theta = acos((x^2 + y^2 - L1^2 - L2^2)/(-2*L1*L2)); 
    q(2) = pi - theta; 
    alpha = acos((L2^2 - (x^2 + y^2) - L1^2)/(-2*sqrt(x^2 + y^2)*L1)); 
    beta = atan2(x, y);
    
    q(1) = pi/2 - alpha - beta;
end
