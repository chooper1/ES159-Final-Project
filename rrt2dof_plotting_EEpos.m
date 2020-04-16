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
   
   if isInObs(q_rand, obstacles) == 0
       num_iter = num_iter + 1; 
       [q_near, near_ind] = findNearestVert(q_rand, route); 
       
       if norm(q_near - qf) < max_step
           route = horzcat(route, qf); 
           edges = horzcat(edges, near_ind);
           break; 
       end       
       % define a q_new that will be appended
           
       q_new = (q_rand - q_near)/(norm(q_rand - q_near))*max_step + q_near;
       
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
   traj_len = traj_len(2)
   
   pos_traj = [];
   
   for i=1:traj_len
        x = L1*cos(traj(1,i)) + L2*cos(traj(1,i) + traj(2,i));
        y = L1*sin(traj(1,i)) + L2*sin(traj(1,i) + traj(2,i));
        curr_pos = [x; y];
        pos_traj = horzcat(pos_traj, curr_pos);
   end
   
   plot(pos_traj(1, :), pos_traj(2,:));
end

function inCol = isInObs(q_curr, obs)
    % calculate the ee location in the workspace
    L1 = 1; 
    L2 = 1; 
    
    eeLoc = zeros(2,1);
    eeLoc(1) = L1*cos(q_curr(1)) + L2*cos(q_curr(1)+q_curr(2));
    eeLoc(2) = L1*sin(q_curr(1)) + L2*sin(q_curr(1)+q_curr(2));
    
    jointLoc = zeros(2,1);
    jointLoc(1) = L1*cos(q_curr(1));
    jointLoc(2) = L1*sin(q_curr(1));
    
    %assuming base is located at (0,0)
    baseLoc(1) = 0;
    baseLoc(2) = 0;
    
    numObs = size(obs);
    numObs = numObs(2);
    
    for i=1:numObs
        %checking ee and joint collisions
        if norm(eeLoc - obs(1:2,i)) < obs(3,i) 
            inCol = 1; 
            return; 
        elseif norm(jointLoc - obs(1:2,i)) < obs(3,i) 
            inCol = 1; 
            return; 
        end
        %checking collisions with middle of joints       
        %https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
        a = jointLoc(2)-eeLoc(2);
        b = eeLoc(1)-jointLoc(1);
        c = (jointLoc(1)-eeLoc(1))*jointLoc(2)+jointLoc(1)*(eeLoc(2)-jointLoc(2));
        dist = abs(a*obs(1,i)+b*obs(2,i)+c) / sqrt(a^2+b^2);
        
        if dist < obs(3,i)
            inCol = 1; 
            return;
        end
        
        a = baseLoc(2)-jointLoc(2);
        b = jointLoc(1)-baseLoc(1);
        c = (baseLoc(1)-jointLoc(1))*baseLoc(2)+baseLoc(1)*(jointLoc(2)-baseLoc(2));
        dist = abs(a*obs(1,i)+b*obs(2,i)+c) / sqrt(a^2+b^2);
        
        if dist < obs(3,i)
            inCol = 1; 
            return;
        end
    end
    inCol = 0; 
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