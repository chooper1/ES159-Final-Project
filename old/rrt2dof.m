% Define global variables here? 
clear all; clc; 

global route

function rrt2dof(qs, pf, obstacles) 
% qs as start position in configuration space
% pf as end position in workspace
% Obstacles defined by x, y, rad

num_iter = 0; 
max_iter = 1; 

q1_min = deg2rad(-30);
q1_max = deg2rad(30);

q2_min = deg2rad(-30);
q2_max = deg2rad(30);

global route = [qs];
q_curr = zeros(2,1);

while (num_iter <= max_iter)
   num_iter = num_iter + 1; 
   q_curr(1) = (q1_max - q1_min)*rand + q1_min; 
   q_curr(2) = (q2_max - q2_min)*rand + q2_min; 
   if isInObs(q_curr) == 0
       q_near = findNearestVert(q_curr); 
       
   end
end

end

function inCol = isInObs(q_curr, obstacles)
    % calculate the ee location in the workspace
    L1 = 1; 
    L2 = 1; 
    
    eeLoc = zeros(2,1);
    
    eeLoc(1) = L1*cos(q_curr(1)) + L2*cos(q_curr(1)+q_curr(2));
    eeLoc(2) = L1*sin(q_curr(1)) + L2*sin(q_curr(1)+q_curr(2));
    
    numObs = size(obstacles);
    numObs = numObs(2);
    
    for i=1:numObs
        if norm(eeLoc - obs(1:2,i)) < obs(3,i)
            inCol = 1; 
            return; 
        end
    end
    inCol = 0; 
end

function q_near = findNearestVert(q_curr)
    route = global route; 
    numPoints = size(route);
    
    numPoints = numPoints(2); 
    min_dist = intmax; 
    q_near = zeros(2,1);
    
    for i=1:numPoints
        if norm(q_curr - route(:,i)) < min_dist
            q_near = route(:,i);
        end
    end
end
