%%% by FishCan
%%% Implementation for Analytical Inverse Kinematic Computation
%%% for 7-DOF Redundant Manipulators With Joint Limits
%%% and Its Application to Redundancy Resolution

close all
clear
clc

d_bs=317;
d_se=450;
d_ew=480;
d_wt=300;

L1 = Revolute('offset',0,       'd',d_bs,  'a',0,    'alpha',-pi/2);
L2 = Revolute('offset',0,       'd',0,      'a',0,     'alpha',pi/2);
L3 = Revolute('offset',0,       'd',d_se,  'a',0,   'alpha',-pi/2);
L4 = Revolute('offset',0,       'd',0,      'a',0,   'alpha',pi/2);
L5 = Revolute('offset',0,       'd',d_ew,  'a',0,   'alpha',-pi/2);
L6 = Revolute('offset',0,       'd',0,      'a',0,  'alpha',pi/2);
L7 = Revolute('offset',0,       'd',d_wt,   'a',0,      'alpha',0);

mybot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name','mybot');
W = [-1200,+1200,-1200,+1200,-1200,+1200];

Theta = [-60 45 -30 60 0 -15 45]/180*pi;
forwarda = mybot.fkine(Theta);

figure
mybot.plot(Theta,'tilesize',150,'workspace',W);
mybot.teach(forwarda,'rpy');

qss=[];
for pusai=-pi:0.01:pi
   qss=[qss;ikSolver(pusai,forwarda,[d_bs d_se d_ew d_wt])]; 
end

mybot.plot(qss,'delay',0.01);
