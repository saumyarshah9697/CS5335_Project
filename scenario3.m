num=[2,5,10,20,40,60,80,100,150,200]';
data=zeros(size(num,1),1);
radii=[1;2;5];
Datas=[];
for j=1:size(radii,1)
for i=1:size(num,1)
    disp(num(i));
    disp(radii(j))
    data(i,1)=pathPlanWithNumberPRM(num(i),radii(j));
end
Datas=[Datas;data'];
end
plot(num,Datas(1,:),'r',num,Datas(2,:),'g',num,Datas(3,:),'b');

function data = pathPlanWithNumberPRM(num,radii)
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

prmNumSamples = num;
prmRadius = radii;


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

qTraj_prm = initializesPRM(rob,prmNumSamples,prmRadius,sphereCenter,sphereRadius,qStart, qGoal,qMax, qMin);
path_size_prm=0;

if size(qTraj_prm,1)==0
   data=0; 
else
fk = rob.fkine(qTraj_prm(1,:));
prevPos=fk(1:3,4);
for i=1:length(qTraj_prm)
    fk = rob.fkine(qTraj_prm(i,:));
    pos = fk(1:3,4);
%     rob.plot(qTraj_prm(i,:));
    plot3(pos(1), pos(2), pos(3), '*b');
    path_size_prm=path_size_prm+norm(prevPos-pos,2);
    prevPos=pos;
end
disp(prevPos)
data=path_size_prm;
hold off
savefig(strcat('TrajectoryforNumberPRM_', int2str(num),'_',int2str(radii)));
close(gcf)
end
end