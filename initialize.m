mdl_puma560;
rob = p560;
% Configure start/goal configuration; min/max c-space bounds
qStart = [0 -0.78 0 -0.78 0 0];
qGoal = [0.0007 -2.9677 0.0015 -2.7938 0 0];
qMin = [-pi/2,-pi,0,-pi 0, 0];
qMax = [pi/2,0,0,0, 0,0];
 % Set up obstacle
sphere1Center = [-1;0.2;-0.1];
sphere2Center = [0;-0.5;0.2];
sphere3Center = [-0.2;0;1];
sphereRadius = 0.2;
[numSamplesMat radiusMat] = meshgrid(500:500:3000, 0.05:0.03:0.3);
% Plot robot and sphere
rob.plot(qStart);
hold on;	
drawSphere(sphere1Center,sphereRadius);
drawSphere(sphere2Center,sphereRadius);
drawSphere(sphere3Center,sphereRadius);
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 3000;
prmRadius = 0.5;
qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphereRadius, sphere2Center, sphere3Center);
% Plot robot following path
qTraj = interpMilestones(qMilestones);
rob.plot(qTraj);
positions = rob.fkine(qTraj);
for i=1:length(qTraj)
    pos = positions(1:3,4,i);
    plot3(pos(1), pos(2), pos(3), '*r')
end
 
function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end

% 
% Evaluate whether the configuration <q> is in collision with a spherical
% obstacle centered at <sphereCenter> with radius <r>.
% 
% input: q -> 1x4 vector of joint angles
%        sphereCenter -> 3x1 vector that denotes sphere center
%        r -> radius of sphere 
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = robotCollision(rob,q,sphereCenter,r)

    % get points on elbow and at EE
    x1 = [0;0;0];
    T2 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x2 = T2(1:3,4);
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x3 = T3(1:3,4);
    
    vec = 0:0.1:1;
    m = size(vec,2);
    
    x12 = repmat(x2-x1,1,m) .* repmat(vec,3,1) + repmat(x1,1,m);
    x23 = repmat(x3-x2,1,m) .* repmat(vec,3,1) + repmat(x2,1,m);
    x = [x12 x23];
    
    if sum(sum((x - repmat(sphereCenter,1,size(x,2))).^2,1) < r^2) > 0
        collision = 1;
    else
        collision = 0;
    end

end



function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function collision = checkCollision(rob,q,sphereCenter,r)

    % get points on elbow and at EE
    x1 = [0;0;0];
    T2 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x2 = T2(1:3,4);
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x3 = T3(1:3,4);
    
    vec = 0:0.1:1;
    m = size(vec,2);
    
    x12 = repmat(x2-x1,1,m) .* repmat(vec,3,1) + repmat(x1,1,m);
    x23 = repmat(x3-x2,1,m) .* repmat(vec,3,1) + repmat(x2,1,m);
    x = [x12 x23];
    
    if sum(sum((x - repmat(sphereCenter,1,size(x,2))).^2,1) < r^2) > 0
        collision = 1;
    else
        collision = 0;
    end

end

function collision = checkEdge(rob,q1,q2,sphereCenter,r)

    n = 10; % Num pts along vector to check
    m = size(q1,2); % Dimension of c-space
    
    % Generate n pts between q1 and q2
    viaPts = repmat(q2-q1,[n,1]) .* repmat(linspace(0,1,n)', [1 m]) + repmat(q1,[n,1]);
    
    % Check those pts for collision
    collision = 0;
    for i = 1:n
        collision = robotCollision(rob,viaPts(i,:),sphereCenter,r);
        if collision
            break
        end
    end
    
end

% Find a path from qStart to qGoal using RRT.
% 
% input: qStart -> 1x4 vector describing start configuration
%        qGoal -> 1x4 vector describing goal configuration
%        qMin -> 1x4 vector of minimum joint angle values
%        qMax -> 1x4 vector of maximum joint angle values
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle 
% output: qMilestones -> mx4 matrix of vertices along path from start to
%                        goal.
function qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphereRadius, sphere2Center,sphere3Center)

    % Your code here
    Q = qStart;
    nodes = 0;
    for i=1:200
        q = [qMax(1,1)-qMin(1,1).*rand(1,1),qMax(1,2)-qMin(1,2).*rand(1,1),qMax(1,3)-qMin(1,3).*rand(1,1),qMax(1,4)-qMin(1,4).*rand(1,1), qMax(1,5)-qMin(1,5).*rand(1,1), qMax(1,6)-qMin(1,6).*rand(1,1)];
        if (checkCollision(rob,q,sphere1Center,sphereRadius)==0 && checkCollision(rob,q,sphere2Center,sphereRadius)==0 && checkCollision(rob,q,sphere3Center,sphereRadius)==0)
            row = 1;
            minDist = 1000;
            for j=1;size(Q,1)
                fk1 = rob.fkine(Q(j,:));
                pos1 = fk1(1:3,4);
                fk2 = rob.fkine(q);
                pos2 = fk2(1:3,4);
                l2norm = norm(pos1-pos2,2);
                if minDist > l2norm
                    row =j; 
                    minDist = l2norm;
                end 
            end
            if((checkEdge(rob,Q(row,:),q,sphere1Center,sphereRadius)==0) && (checkEdge(rob,Q(row,:),q,sphere2Center,sphereRadius)==0) && (checkEdge(rob,Q(row,:),q,sphere3Center,sphereRadius)==0))
                Q=[Q; q];
                nodes = [nodes; row];
            end
        end
    end            
    q= qGoal;
    row = 1;
    minDist = 1000;
    for j=1:size(Q,1)
        fk1 = rob.fkine(Q(j,:));
        pos1 = fk1(1:3,4);
        fk2 = rob.fkine(q);
        pos2 = fk2(1:3,4);
        l2norm = norm(pos1-pos2,2);
        if minDist > l2norm
            row =j; 
            minDist = l2norm;
        end 
    end
    Q=[Q; q];
    nodes = [nodes; row];
    qMilestones = [Q(end,:)];
    antecedant = nodes(end);
    while antecedant~=0 
        qMilestones = [Q(antecedant,:);qMilestones];
        antecedant = nodes(antecedant);
    end 
end





    