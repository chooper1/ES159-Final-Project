% Define global variables here? 
clear all; clc; 

global route;
global edges;

qs = [0;0;];
pf = [-2;0;];
obstacles = [-1;1;0.5;];
rrt_2dof(qs, pf, obstacles) 
route
edges

function rrt_2dof(qs, pf, obstacles) 
% qs as start position in configuration space
% pf as end position in workspace
% Obstacles defined by x, y, rad

num_iter = 0; 
max_iter = 1000; 
tolerance = 0.1; % in terms of workspace distance

L1 = 1; 
L2 = 1; 

figure(1); hold on; grid on;
eeLoc = zeros(2,1);
eeLoc(1) = L1*cos(qs(1)) + L2*cos(qs(1)+qs(2));
eeLoc(2) = L1*sin(qs(1)) + L2*sin(qs(1)+qs(2));
plot(eeLoc(1), eeLoc(2), 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
plot(pf(1), pf(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

q1_min = deg2rad(-180);
q1_max = deg2rad(180);

q2_min = deg2rad(0);
q2_max = deg2rad(180);

%variable for storing vertices
global route 
route = [qs];
%variable for storing edges
global edges; 

q_rand = zeros(2,1);
while (num_iter <= max_iter)
   q_rand(1) = (q1_max - q1_min)*rand + q1_min; 
   q_rand(2) = (q2_max - q2_min)*rand + q2_min; 
   if isInObs(q_rand, obstacles) == 0
       num_iter = num_iter + 1; 
       q_near = findNearestVert(q_rand); 
       route(:,num_iter+1) = q_rand; 
       edges(:,num_iter) = [
           q_near;
           q_rand;
       ];
       eeLoc(1) = L1*cos(q_rand(1)) + L2*cos(q_rand(1)+q_rand(2));
       eeLoc(2) = L1*sin(q_rand(1)) + L2*sin(q_rand(1)+q_rand(2));
       prevLoc(1) = L1*cos(q_near(1)) + L2*cos(q_near(1)+q_near(2));
       prevLoc(2) = L1*sin(q_near(1)) + L2*sin(q_near(1)+q_near(2));

       plot([eeLoc(1); prevLoc(1);],[eeLoc(2); prevLoc(2);], 'r');
       if norm(eeLoc - pf) < tolerance
          break;
       end
       pause(0);
   end
   
end

%if num_iter < max_iter
%    path.pos(1).x = xGoal; path.pos(1).y = yGoal;
%    path.pos(2).x = tree.vertex(end).x; path.pos(2).y = tree.vertex(end).y;
%    pathIndex = tree.vertex(end).indPrev;
%    j=0;
%    while 1
%        path.pos(j+3).x = tree.vertex(pathIndex).x;
%        path.pos(j+3).y = tree.vertex(pathIndex).y;
%        pathIndex = tree.vertex(pathIndex).indPrev;
%        if pathIndex == 1
%            break
%        end
%        j=j+1;
%    end
%    path.pos(end+1).x = xInit; path.pos(end).y = yInit;
%    for j = 2:length(path.pos)
%        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
%    %     plot([tree.vertex(i).x; tree.vertex(ind).x],[tree.vertex(i).y; tree.vertex(ind).y], 'r');
%    %     pause(0);
%    end
%else
%    disp('No path found. Increase number of iterations and retry.');
%end

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

function q_near = findNearestVert(q_curr)
    global route; 
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