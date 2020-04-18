% ES 159/259, Spring 2020

clear all; close all; clc;
pos = [
    0 -2;
    2 0;
    ];
obs = [
    1
    1
    0.25
    ];
save inputs1.mat pos obs;
% start and end positions
ps = pos(:,1);
pf = pos(:,2);
%testpos = [
%   -0.8472;
%    1.3540;
%];

%inObs(testpos,obs) 

% solve for the trajectory
[P,Q] = RRT(ps, pf, obs);
[P,Q] = addConfigs(obs, P);
q = Q
save solution1.mat q;

%graph storing all vertices and links to previous nodes
function rrt = AddNode(rrt,p,prev)
node.p = p;
node.prev = prev;
rrt{end+1} = node;
end

function [P,Q] = RRT(ps,pf,obs)
thresh = 0.75;
max_iterations = 10000;
%P = [ps];
rrt = {};
rrt = AddNode(rrt,ps,0);
%qs = inverseKinematics(ps);
%Q = [qs];
%P = [];
%Q = [];
iter = 1;
while iter <= max_iterations
    random = rand(2,1);
    random(1) = random(1)*2;
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
            %dist
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
L1 = 1;
L2 = 1;
q = armInverseKinematics(pos);
eeLoc(1,1) = L1*cosd(q(1)) + L2*cosd(q(1)+q(2));
eeLoc(2,1) = L1*sind(q(1)) + L2*sind(q(1)+q(2));
jointLoc(1,1) = L1*cosd(q(1));
jointLoc(2,1) = L1*sind(q(1));
baseLoc(1,1) = 0;
baseLoc(2,1) = 0;

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
        col = 1; 
        return; 
    elseif norm(jointLoc - obs(1:2,i)) < obs(3,i) 
        col = 1; 
        return; 
    end
    %checking collisions with middle of joints       
    %https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
    
    ax = eeLoc(1,1);
    ay = eeLoc(2,1);
    bx = jointLoc(1,1);
    by = jointLoc(2,1);
    cx = obs(1,i);
    cy = obs(2,i);
    r  = obs(3,i);
    ax = ax - cx;
    ay = ay - cy;
    bx = bx - cx;
    by = by - cy;
    c = ax^2 + ay^2 - r^2;
    b = 2*(ax*(bx - ax) + ay*(by - ay));
    a = (bx - ax)^2 + (by - ay)^2;
    disc = b^2 - 4*a*c;
    if(disc > 0)
        %endpoint check
        if ax > bx
            if 0 < ax && 0 > bx % changed from cx
                col = 1;
                return;
            end
        else 
            if 0 < bx && 0 > ax
                col = 1;
                return;
            end
        end
    end
    
    ax = baseLoc(1,1);
    ay = baseLoc(2,1);
    bx = jointLoc(1,1);
    by = jointLoc(2,1);
    cx = obs(1,i);
    cy = obs(2,i);
    r  = obs(3,i);
    ax = ax - cx;
    ay = ay - cy;
    bx = bx - cx;
    by = by - cy;
    a = (bx - ax)^2 + (by - ay)^2;
    b = 2*(ax*(bx - ax) + ay*(by - ay));
    c = ax^2 + ay^2 - r^2;
    disc = b^2 - 4*a*c;
    if(disc > 0)
        %endpoint check
        if ax > bx
            if 0 < ax && 0 > bx
                col = 1;
                return;
            end
        else 
            if 0 < bx && 0 > ax
                col = 1;
                return;
            end
        end
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

% Given a 2x1 vector "p" representing the end-effector position (x, y) and
% return current configuration "q".
function q = inverseKinematics(p)
    % TODO: solve the inverse kinematics
    q = armInverseKinematics(p);
end

%smoothing algorithm
function [newP,newQ] = addConfigs(obs, P) 
resolution = 25;
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
        pos(1,1) = p1(1);
        pos(2,1) = p1(2);
        for j=2:(resolution-1)
            p1(1) = pos(1,1) + j*(xdiff/resolution);
            p1(2) = pos(2,1) + m*(p1(1)-pos(1,1));
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
