qStart = [0 -0.78 0 -0.78 0 0];
qGoal = [0.0007 -2.9677 0.0015 -2.7938 0 0];
qMin = [-pi/2,-pi,0,-pi 0, 0];
qMax = [pi/2,0,0,0, 0,0];
 % Set up obstacle
Mainsphere1Center = [[-1,0.2,-0.1]; [0,-0.5,0.2];[-0.2,0,1]];
Mainsphere1Radius = [0.2;0.2;0.2];
[numSamplesMat,radiusMat] = meshgrid(500:5000:3000, 0.05:0.03:0.3);
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 300;
prmRadius = 5;

rob = initialize();

for k=1:3
    % Plot robot and sphere
rob.plot(qStart);
sphereCenter=Mainsphere1Center(1:k,:);
sphereRadius=Mainsphere1Radius(1:k,:);
hold on;

for i=1:size(sphere1Center,1)
    drawSphere(sphere1Center(i,:),sphere1Radius(i));
end
qTraj_rrt = initializeRRT1(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphere1Radius);
qTraj_prm = initializesPRM(rob,prmNumSamples,prmRadius,sphere1Center,sphere1Radius,qStart, qGoal,qMax, qMin);

path_size_rrt=0;
path_size_prm=0;
fk = rob.fkine(qTraj_rrt(i,:));
prevPos=fk(1:3,4);
for i=1:length(qTraj_rrt)
    fk = rob.fkine(qTraj_rrt(i,:));
    pos = fk(1:3,4);
    rob.plot(qTraj_rrt(i,:))
    plot3(pos(1), pos(2), pos(3), '-r')
    path_size_rrt=path_size_rrt+norm(prevPos-pos,2);
end

fk = rob.fkine(qTraj_prm(i,:));
prevPos=fk(1:3,4);

for i=1:length(qTraj_prm)
    fk = rob.fkine(qTraj_prm(i,:));
    pos = fk(1:3,4);
    rob.plot(qTraj_prm(i,:));
    plot3(pos(1), pos(2), pos(3), '*b');
    path_size_prm=path_size_prm+norm(prevPos-pos,2);
end

display(sprintf('path size rrt: %f, path size sprm %f',path_size_rrt, path_size_prm));
hold off
end