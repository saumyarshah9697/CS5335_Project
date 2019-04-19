function qTraj = initializeRRT1(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphere1Radius)



% Parameters for PRM and RRG in Q4 and Q5.
% prmNumSamples = 300;
% prmRadius = 5;
qMilestones = rrt(rob,qStart,qGoal,qMin,qMax,sphere1Center,sphere1Radius);
% Plot robot following path
qTraj = interpMilestones(qMilestones);
%rob.plot(qTraj);
%positions = rob.fkine(qTraj);

end


















    