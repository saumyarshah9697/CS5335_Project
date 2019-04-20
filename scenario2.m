num=[2,10,20,40,60,80,100,500,1000,2000]';
data=zeros(size(num,1),1);
for i=1:size(num,1)
    data(i,1)=pathPlanWithNumberRRT(num(i));
end
plot(num,data)

function data = pathPlanWithNumberRRT(num)
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



rob.plot(qStart);
sphereCenter=Mainsphere1Center;
sphereRadius=Mainsphere1Radius;
hold on;
fk = rob.fkine(qGoal);
pos=fk(1:3,4);
plot3(pos(1), pos(2), pos(3), 'X','markersize',12);
for i=1:size(sphereCenter,1)
    drawSphere(sphereCenter(i,:),sphereRadius(i));
end
qTraj_rrt = initializeRRT1(rob,qStart,qGoal,qMin,qMax,sphereCenter,sphereRadius,num);
disp("RRT");

path_size_rrt=0;

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
data=path_size_rrt;
hold off
savefig(strcat('TrajectoryforNumberRRT_', int2str(num)))
close(gcf)
end