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
function qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphereCenters,sphereRadius,num)

V=[qStart];
Parents=[0];
for i=1:num
    iQ=[(qMax(1,1)-qMin(1,1)).*rand(1,1)+qMin(1,1),(qMax(1,2)-qMin(1,2)).*rand(1,1)+qMin(1,2),(qMax(1,3)-qMin(1,3)).*rand(1,1)+qMin(1,3), (qMax(1,4)-qMin(1,4)).*rand(1,1)+qMin(1,4),(qMax(1,5)-qMin(1,5)).*rand(1,1)+qMin(1,5),(qMax(1,6)-qMin(1,6)).*rand(1,1)+qMin(1,6)];
    bool=true;
        for b=1:size(sphereCenters,1)
            if checkCollision(rob,iQ,sphereCenters(b,:)',sphereRadius(b))
                bool=false;
                break
            end
        end
    if bool
        index=getMinimum(rob,V,iQ);
        bool=true;
            for b=1:size(sphereCenters,1)
                if(checkEdge(rob,V(index,:),iQ,sphereCenters(b,:)',sphereRadius(b)))
                    bool=false;
                    break;
                end
            end
        if bool
            V=[V;iQ];
            Parents=[Parents;index];
        end
    end
end
iQ=qGoal;
index=1;
minDis=1000;
    for i=2:size(V,1)
        temp=norm(getLoc(rob,iQ)-getLoc(rob,V(i,:)),2);
        bool=true;
            for b=1:size(sphereCenters,1)
                if(checkEdge(rob,V(i,:),iQ,sphereCenters(b,:)',sphereRadius(b)))
                    bool=false;
                    break;
                end
            end
        if temp<minDis && bool
            index=i;
            minDis=temp;
        end
    end
V=[V;iQ];
Parents=[Parents;index];

qMilestones=[V(end,:)];
parent=Parents(end);

while parent~=0
    qMilestones=[V(parent,:);qMilestones];
    parent=Parents(parent);
end
end

function index=getMinimum(rob,vertices,vertex)
    index=1;
    minDis=1000;
    for i=1:size(vertices,1)
        temp=norm(getLoc(rob,vertex)-getLoc(rob,vertices(i,:)),2);
        if temp<minDis
            index=i;
            minDis=temp;
        end
    end
end

function location=getLoc(rob,q)
Hom=rob.fkine(q);
location=Hom(1:3,4);
end