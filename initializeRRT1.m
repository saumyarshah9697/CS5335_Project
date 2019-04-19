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
% prmNumSamples = 300;
% prmRadius = 5;
qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphere1Radius);
% Plot robot following path
qTraj = interpMilestones(qMilestones);
%rob.plot(qTraj);
%positions = rob.fkine(qTraj);
path_size=0;
fk = rob.fkine(qTraj(i,:));
prevPos=fk(1:3,4);
for i=1:length(qTraj)
    fk = rob.fkine(qTraj(i,:));
    pos = fk(1:3,4);
    rob.plot(qTraj(i,:))
    plot3(pos(1), pos(2), pos(3), '.r')
    path_size=path_size+norm(prevPos-pos,2);
end


















    