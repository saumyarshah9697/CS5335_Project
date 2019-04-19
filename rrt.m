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
function qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphereCenters,sphereRadius)

    % Your code here
    Q = qStart;
    nodes = 0;
    for i=1:200
        q = [qMax(1,1)-qMin(1,1).*rand(1,1),qMax(1,2)-qMin(1,2).*rand(1,1),qMax(1,3)-qMin(1,3).*rand(1,1),qMax(1,4)-qMin(1,4).*rand(1,1), qMax(1,5)-qMin(1,5).*rand(1,1), qMax(1,6)-qMin(1,6).*rand(1,1)];
        bool=true;
        for b=1:size(sphereCenters,1)
            bool=bool & (checkCollision(rob,q,sphereCenters(b,:)',sphereRadius(b)==0));
        end
        if bool
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
            bool=true;
            for b=1:size(sphereCenters,1)
                bool=bool & (checkEdge(rob,Q(row,:),q,sphereCenters(b,:)',sphereRadius(b))==0);
            end
            if bool
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