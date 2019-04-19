qStart = [0 -0.78 0 -0.78 0 0];
qGoal = [0.0007 -2.9677 0.0015 -2.7938 0 0];
qMin = [-pi/2,-pi,0,-pi 0, 0];
qMax = [pi/2,0,0,0, 0,0];
 % Set up obstacle
sphere1Center = [[-1,0.2,-0.1]; [0,-0.5,0.2];[-0.2,0,1]];
sphere1Radius = [0.2;0.2;0.2];
[numSamplesMat,radiusMat] = meshgrid(500:5000:3000, 0.05:0.03:0.3);
% Parameters for PRM and RRG in Q4 and Q5.
prmNumSamples = 300;
prmRadius = 5;

samples = rand(prmNumSamples,6) .* repmat(qMax - qMin,[prmNumSamples 1]) + repmat(qMin,[prmNumSamples 1]);
rob = initialize();
