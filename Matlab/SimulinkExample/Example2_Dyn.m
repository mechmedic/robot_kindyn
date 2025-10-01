% ----------------------------------------------------------------- %
% MATLAB Demo script for UST Class
% Inverse dyanmic using Newton Euler Algorithm for two link manipulator
% ----------------------------------------------------------------- %
clc;
clear all;

nJoint = 2;

% [Tau, F] =  InvDyn(theta, dtheta, ddtheta, Fext, tfList, TwistList, MassList)
%theta = [pi/2,0];
theta = [0,pi/2];
%dtheta = [1,0];
dtheta = [1,2];
ddtheta = [0,0];
%Fext = zeros(6,1);
Fext = [0,0,0,0,-1,0]';
L1 = 10;	L2 = 5;
m1 = 3;		m2 = 2;
I1 = zeros(3,3);	I1(3,3) = 1/12*m1*L1*L1;	I1(2,2) = I1(3,3);
I2 = zeros(3,3);	I2(3,3) = 1/12*m2*L2*L2;	I2(2,2) = I2(3,3); 

tfList = zeros(4,4,nJoint+1);
tfList(:,:,1) = RpToTrans(eye(3),[L1/2,0,0]');
tfList(:,:,2) = RpToTrans(eye(3),[L1+L2/2,0,0]');
tfList(:,:,3) = RpToTrans(eye(3),[L1+L2,0,0]');

TwistList = zeros(6,nJoint);
w = [0,0,1];	v = [0,0,0];	TwistList(:,1) = [w,v]';
w = [0,0,1];	v = [0,-L1,0];	TwistList(:,2) = [w,v]';

MassList = zeros(6,6,nJoint);
MassList(:,:,1) = [I1,zeros(3,3); zeros(3,3), m1*eye(3)];
MassList(:,:,2) = [I2,zeros(3,3); zeros(3,3), m2*eye(3)];

[Tau, F] =  InvDyn(theta, dtheta, ddtheta, Fext, tfList, TwistList, MassList)