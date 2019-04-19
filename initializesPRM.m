startup_rvc;
mdl_puma560;
rob = p560;
% Configure start/goal configuration; min/max c-space bounds
qStart = [0 -0.78 0 -0.78 0 0];
qGoal = [0.0007 -2.9677 0.0015 -2.7938 0 0];
qMin = [-pi/2,-pi,0,-pi 0, 0];
qMax = [pi/2,0,0,0, 0,0];
 % Set up obstacle
sphere1Center = [[-1,0.2,-0.1]; [0,-0.5,0.2];[-0.2,0,1]];
sphere1Radius = [0.2;0.2;0.2];
[numSamplesMat,radiusMat] = meshgrid(500:5000:3000, 0.05:0.03:0.3);
% Plot robot and sphere
rob.plot(qStart);
hold on;	
for i=1:size(sphere1Center,1)
    drawSphere(sphere1Center(i,:),sphere1Radius(i));
end
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 300;
prmRadius = 5;

samples = rand(prmNumSamples,6) .* repmat(qMax - qMin,[prmNumSamples 1]) + repmat(qMin,[prmNumSamples 1]);
samples(1,:) = qStart;
samples(2,:) = qGoal;
% Prune collision samples and get weighted adjacency matrix for
% r-disk graph.
[samplesFree, adjacencyMat] = sPRM(rob,samples,prmRadius,sphere1Center,sphere1Radius);

% Create matlab graph object
G = graph(adjacencyMat);

% Display number of connected components
numComponents = size(unique(conncomp(G)),2);
display(sprintf('number of connected components: %d',numComponents));

% Find shortest path
[path, pathLength] = shortestpath(G,1,2);
display(sprintf('path length: %f',pathLength));
qMilestones = samplesFree(path,:);
qTraj = interpMilestones(qMilestones);
if pathLength == Inf
    error('No path found!')
end

path_size=0;
fk = rob.fkine(qTraj(i,:));
prevPos=fk(1:3,4);

for i=1:length(qTraj)
    fk = rob.fkine(qTraj(i,:));
    pos = fk(1:3,4);
    rob.plot(qTraj(i,:));
    plot3(pos(1), pos(2), pos(3), '-b')
    path_size=path_size+norm(prevPos-pos,2);
end








    