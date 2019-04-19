% Create the sPRM graph for a given set of samples.
% input: samples -> nx4 matrix of samples for which to create r-disk graph
%        prmRadius -> radius to be used to construct r-disk-graph
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle.
% output: samplesFree -> prmNumSamples x 4 matrix of roadmap vertices. These are
%                        the vertices that you sample. But, rows 1 and 2 are
%                        special: row 1 should be set to qStart; row 2 should be
%                        set of qGoal.
%         adjacencyMat -> nxn adjacency matrix. If there is an edge between
%                         vertices i and j, then element i,j should be set
%                         to the Euclidean distance between i and j. OW, it
%                         element i,j should be set to zero.
%                         Note that this matrix should have zeros on the
%                         diagonal and it should be symmetric.
%
function [samplesFree, adjacencyMat] = sPRM(rob,samples,prmRadius,sphereCenters,sphereRadius)

    % Your code here
    samplesFree=[samples(1,:);samples(2,:)];
    for i=3:size(samples,1)
        bool=true;
        bool=true;
        for b=1:size(sphereCenters,1)
            if checkCollision(rob,iQ,sphereCenters(b,:)',sphereRadius(b))
                bool=false;
                break
            end
        end
    if bool
            samplesFree=[samplesFree;samples(i,:)];
        end
    end
    disp("End of loop 1\n");
    adjacencyMat=zeros(size(samples,1));
    for i=1:size(samples,1)-1
        for j=i+1:size(samples,1)
            disp(i);
            bool=true;
            for b=1:size(sphereCenters,1)
                if(checkEdge(rob,V(index,:),iQ,sphereCenters(b,:)',sphereRadius(b)))
                    bool=false;
                    break;
                end
            end
        if bool
                pt1=rob.fkine(samples(i,:));
                pt2=rob.fkine(samples(j,:));
                dist=norm(pt1(1:3,4)-pt2(1:3,4),2);
                if dist<=prmRadius
                    adjacencyMat(i,j)=dist;
                    adjacencyMat(j,i)=dist;
                end
            end
        end
    end
    
end

