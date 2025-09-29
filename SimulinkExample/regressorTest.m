clc;
clear;
load('UR5_Data.mat');
% tic
% load('UR5_Regressor.mat');
% toc


tic;
Wval = zeros(600,60);
tau = zeros(600,1);
grav = [0;0;0;0;0;-9.8];
omg = 2*pi;
t=0:0.01:15;

% Generate sinusoidal trajectories
th = pi*sin(omg*t) + pi/4*cos(omg*t);
thSet = repmat(th,[6,1]);
dth = omg*pi*cos(omg*t) - omg*pi/4*sin(omg*t);
dthSet = repmat(dth,[6,1]);
ddth = -omg*omg*pi*sin(omg*t) - omg*omg*pi/4*cos(omg*t);
ddthSet = repmat(ddth,[6,1]);
nMeas = length(t);
Wval = zeros(nMeas*6,60);

% Ground truth
TruePara = zeros(60,1);
for i=1:6
    TruePara(10*(i-1)+1:10*i,:) = InertiaToVec(Mlist(:,:,i));
end

for i=1:nMeas
   % % First generate random trajectories
   %  th = -10+20*rand(6,1);
   %  dth = -10+20*rand(6,1);
   %  ddth = -10+20*rand(6,1);

    % Point on sinusoidal trajectory
    th = thSet(:,i);
    dth = dthSet(:,i);
    ddth = ddthSet(:,i);

    % Solve inverse dynamics to obtain torque measurement
    tau(6*(i-1)+1:1:6*i,:) = InvDyn(th,dth,ddth,zeros(6,1),Twists,TfLists,Mlist,grav);

    % Evalaute regressor
    %Wval(6*(i-1)+1:1:6*i,:) = regressorUR5(th(1),th(2),th(3),th(4),th(5),th(6),dth(1),dth(2),dth(3),dth(4),dth(5),dth(6),ddth(1),ddth(2),ddth(3),ddth(4),ddth(5),ddth(6));
    Wval(6*(i-1)+1:1:6*i,:) = regressorUR5_opt(th(1),th(2),th(3),th(4),th(5),th(6),dth(1),dth(2),dth(3),dth(4),dth(5),dth(6),ddth(1),ddth(2),ddth(3),ddth(4),ddth(5),ddth(6));
end
toc;

% Identify base parameters
[Q,R,P] = qr(Wval);
n = rank(Wval)
R1 = R(1:n,1:n);
R2 = R(1:n,n+1:end);

mapTobase = [ eye(n), inv(R1)*R2; zeros(60-n,n), eye(60-n) ]*P';
ToBasePara = [eye(n),inv(R1)*R2]*P';
Wo = Wval*P(:,1:n);

% Solve for base parameters. Also compute true base parameter from the
% ground truth
estBasePara = Wo\tau;
trueBasePara = ToBasePara*TruePara;

% Convert base parameters to original link parameter.  Also compute true base parameter from the
% ground truth
tmp = P'*TruePara;
estTruePara = mapTobase\[estBasePara; tmp(n+1:end,:)];

