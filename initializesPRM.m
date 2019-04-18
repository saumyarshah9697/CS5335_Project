startup_rvc;
mdl_puma560;
rob = p560;
% Configure start/goal configuration; min/max c-space bounds
qStart = [0 -0.78 0 -0.78 0 0];
qGoal = [0.0007 -2.9677 0.0015 -2.7938 0 0];
qMin = [-pi/2,-pi,0,-pi 0, 0];
qMax = [pi/2,0,0,0, 0,0];
 % Set up obstacle
sphere1Center = -[-1;0.2;-0.1];
sphere2Center = -[0;-0.5;0.2];
sphere3Center = -[-0.2;0;1];
sphere1Radius = 0.2;
sphere2Radius = 0.2;
sphere3Radius = 0.2;
[numSamplesMat,radiusMat] = meshgrid(500:5000:3000, 0.05:0.03:0.3);
% Plot robot and sphere
rob.plot(qStart);
hold on;	
drawSphere(sphere1Center,sphereRadius);
drawSphere(sphere2Center,sphereRadius);
drawSphere(sphere3Center,sphereRadius);
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 300;
prmRadius = 5;

samples = rand(prmNumSamples,4) .* repmat(qMax - qMin,[prmNumSamples 1]) + repmat(qMin,[prmNumSamples 1]);
samples(1,:) = qStart;
samples(2,:) = qGoal;
% Prune collision samples and get weighted adjacency matrix for
% r-disk graph.
[samplesFree, adjacencyMat] = Q4(rob,samples,prmRadius,sphere1Center,sphere1Radius,sphere2Center,sphere2Radius,sphere3Center,sphere3Radius);

% Create matlab graph object
G = graph(adjacencyMat);

% Display number of connected components
numComponents = size(unique(conncomp(G)),2);
display('number of connected components: %d',numComponents);

% Find shortest path
[path, pathLength] = shortestpath(G,1,2);
display('path length: %f',pathLength);
qMilestones = samplesFree(path,:);

if pathLength == Inf
    error('No path found!')
end

for i=1:length(qTraj)
    fk = rob.fkine(qTraj(i,:));
    pos = fk(1:3,4);
    rob.plot(qTraj(i,:))
    %pos = positions(1:3,4,i);
    plot3(pos(1), pos(2), pos(3), '*r')
end








    