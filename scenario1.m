
[rrt1, prm1] = pathPlanWithObstacles(1);
[rrt2, prm2] = pathPlanWithObstacles(2);
[rrt3, prm3] = pathPlanWithObstacles(3);
lengths = [rrt1 prm1;rrt2 prm2;rrt3 prm3];
bar(lengths)

function [path_size_rrt, path_size_prm] = pathPlanWithObstacles(numOfObstacles)
    % Plot robot and sphere
    rob = initialize();
    XGoal = -[0.5;0.5;-0.5];

qStart = [0 -0.78 0 -0.78 0 0];
qGoal = rob.ikine6s(transl(XGoal));

qMin = -[5.58505,4.36332,4.71239,5.23599, 3.49066,9.28515];
qMax = [5.58505,4.36332,4.71239,5.23599, 3.49066,9.28515];
 % Set up obstacle
Mainsphere1Center = -[[-0.1,-0.25, -0.7]; [0.4,-0.2,-0.4];[0,-0.2,-0.4]];
Mainsphere1Radius = [0.1;0.2;0.1];
[numSamplesMat,radiusMat] = meshgrid(500:5000:3000, 0.05:0.03:0.3);
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 300;
prmRadius = 5;


rob.plot(qStart);
sphereCenter=Mainsphere1Center(1:numOfObstacles,:);
sphereRadius=Mainsphere1Radius(1:numOfObstacles,:);
hold on;
fk = rob.fkine(qGoal);
pos=fk(1:3,4);
plot3(pos(1), pos(2), pos(3), 'X','markersize',12);
for i=1:size(sphereCenter,1)
    drawSphere(sphereCenter(i,:),sphereRadius(i));
end
num=100;
qTraj_rrt = initializeRRT1(rob,qStart,qGoal,qMin,qMax,sphereCenter,sphereRadius,num);
%qTraj_prm = initializesPRM(rob,prmNumSamples,prmRadius,sphereCenter,sphereRadius,qStart, qGoal,qMax, qMin);

path_size_rrt=0;
path_size_prm=0;
fk = rob.fkine(qTraj_rrt(1,:));
prevPos=fk(1:3,4);
for i=1:length(qTraj_rrt)
    fk = rob.fkine(qTraj_rrt(i,:));
    pos = fk(1:3,4);
%     rob.plot(qTraj_rrt(i,:));
    plot3(pos(1), pos(2), pos(3), '.r');
    path_size_rrt=path_size_rrt+norm(prevPos-pos,2);
    prevPos=pos;
end
disp(prevPos)

% fk = rob.fkine(qTraj_prm(i,:));
% prevPos=fk(1:3,4);
% positions = 
% for i=1:length(qTraj_prm)
%     fk = rob.fkine(qTraj_prm(i,:));
%     pos = fk(1:3,4);
%     rob.plot(qTraj_prm(i,:));
%     plot3(pos(1), pos(2), pos(3), '*b');
%     path_size_prm=path_size_prm+norm(prevPos-pos,2);
% end
% 
% display(sprintf('path size rrt: %f, path size sprm %f',path_size_rrt, path_size_prm));
hold off
savefig(strcat('TrajectoryWithObstacle_', int2str(numOfObstacles)))
path_size_rrt;
path_size_prm;
close(gcf)
end