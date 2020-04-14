% ES 159/259, Spring 2020

clear all; close all; clc;
pos = [
    0 -2;
    2 0;
    ];
obs = [
    -0.5;
    0.5;
    0.25;
    ];
% start and end positions
ps = pos(:,1);
pf = pos(:,2);
% solve for the trajectory
[P,Q] = RRT(ps, pf, obs);
[P,Q] = addConfigs(obs, P);
q = Q
save solution3.mat q;

%graph storing all vertices and links to previous nodes
function rrt = AddNode(rrt,p,prev)
node.p = p;
node.prev = prev;
rrt{end+1} = node;
end

function [P,Q] = RRT(ps,pf,obs)
thresh = 0.5;
max_iterations = 100000;
P = [];
rrt = {};
Q = [];

rrt = AddNode(rrt,ps,0);
iter = 1;
while iter <= max_iterations
    
    random = rand(2,1);
    random(1) = random(2)*2;
    random(2) = random(2)*360 - 180; %degrees
    
    pos(1,1) = cosd(random(2)) * random(1);
    pos(2,1) = sind(random(2)) * random(1);
    %try
    %    inverseKinematics(pos);       
    %    valid = 1;
    %catch exception
    %    valid = 0;
    %end
    
    if inObs(pos,obs) == 0 
        for i=1:length(rrt) % find nearest node
            dist = norm(rrt{i}.p - pos);
            if (i==1) || (dist < mindist)
                mindist = dist;
                imin = i;
                nearest_pos = rrt{i}.p;
            end
        end
        if edge_collision(obs,pos,nearest_pos) == 0
            rrt = AddNode(rrt,pos,imin); 
            dist = norm(pos-pf);
            if (dist < thresh) % add goal pos
                if edge_collision(obs,pos,pf) == 0
                    rrt = AddNode(rrt,pf,length(rrt));
                    
                    %connect final path
                    
                    i = length(rrt);
                    P(:,1) = rrt{i}.p;
                    Q(:,1) = inverseKinematics(rrt{i}.p);
                    while 1
                        i = rrt{i}.prev;
                        if i == 0
                            return
                        end
                        P = [rrt{i}.p P];
                        Q = [inverseKinematics(rrt{i}.p) Q];
                    end
                end
            end
        end
    end 
    iter = iter + 1;
end
end

function col = inObs(pos,obs)
col = 0;
%radius = 0.5;
L1 = 1;
L2 = 1;
q = armInverseKinematics(pos);
eeLoc(1) = L1*cos(q(1)) + L2*cos(q(1)+q(2));
eeLoc(2) = L1*sin(q(1)) + L2*sin(q(1)+q(2));
jointLoc(1) = L1*cos(q(1));
jointLoc(2) = L1*sin(q(1));
baseLoc(1) = 0;
baseLoc(2) = 0;

%for i=1:length(obs(1,:)) 
%    dist = sqrt((pos(1)-obs(1,i))^2+(pos(2)-obs(2,i))^2);
%    %dist = norm(pos-obs(:,i));
%    if dist < (radius + obs_rad)
%        col = 1;
%        return;
%    end 
%end
for i=1:length(obs(1,:))
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

function col = edge_collision(obs,p1,p2)
col = 0;
resolution = 50;
m = (p2(2) - p1(2))/(p2(1) - p1(1));
xdiff = p2(1) - p1(1);
for i=2:(resolution-1)
    pos(1) = p1(1) + i*(xdiff/resolution);
    pos(2) = p1(2) + m*(pos(1)-p1(1));
    col = inObs(pos,obs); 
    if col == 1
        return;
    end
end
end

% Given a 2x1 vector "p" representing the end-effector position (x, y) and
% return current configuration "q".
function q = inverseKinematics(p)
    % TODO: solve the inverse kinematics
    q = armInverseKinematics(p);
end

%smoothing algorithm
function [newP,newQ] = addConfigs(obs, P) 
resolution = 20;
pointNum = 1;
for i=1:length(P)
    if i ~= length(P)
        newP(:,pointNum) = P(:,i);
        newQ(:,pointNum) = inverseKinematics(P(:,i));
        p1 = P(:,i);
        p2 = P(:,i+1);
        m = (p2(2) - p1(2))/(p2(1) - p1(1));
        xdiff = p2(1) - p1(1);
        pointNum = pointNum + 1; 
        pos = p1;
        for j=2:(resolution-1)
            p1(1) = pos(1) + j*(xdiff/resolution);
            p1(2) = pos(2) + m*(p1(1)-pos(1));
            newP(:,pointNum) = p1;
            newQ(:,pointNum) = inverseKinematics(p1);
            pointNum = pointNum + 1; 
            %try inverseKinematics(p1); 
            %    newP(:,pointNum) = p1;
            %    newQ(:,pointNum) = inverseKinematics(p1);
            %    pointNum = pointNum + 1; 
            %catch exception
            %    continue;
            %end
        end
    else
        newP(:,pointNum) = P(:,i);
        newQ(:,pointNum) = inverseKinematics(P(:,i));
    end
end
end
