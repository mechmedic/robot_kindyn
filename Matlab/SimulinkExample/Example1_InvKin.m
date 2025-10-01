% ----------------------------------------------------------------- %
% MATLAB Demo script for UST Class
%
% ----------------------------------------------------------------- %
clc;
clear all;

% Twist parameters for UR5, from textbook page 125
H1 = 89;	H2 = 95;	% in mm
L1 = 425;	L2 = 392;	% in mm
W1 = 109;	W2 = 82;	% in mm

M = [-1,0,0,L1+L2; 0,0,1,W1+W2; 0,1,0,H1-H2; 0,0,0,1];

xi1 = [0,0,1, 0,0,0];			xi2 = [0,1,0, -H1,0,0];
xi3 = [0,1,0, -H1,0,L1];		xi4 = [0,1,0, -H1,0,L1+L2];
xi5 = [0,0,-1, -W1,L1+L2,0];	xi6 = [0,1,0, H2-H1,0,L1+L2];

Twists = [xi1',xi2',xi3',xi4',xi5',xi6'];

Jang = rand(6,1);

T = FwdKin(M,Twists,Jang);
Js = JacobianSpace(Twists,Jang);

TwistsInBody = Adjoint(TransInv(M))*Twists;
Jb = JacobianBody(TwistsInBody,Jang);

xx = Adjoint(T)*Jb - Js;

[sol,flag] = InvKin(T, M, Twists, Jang*1.1); 

figure; hold;
for i=1:6
	DrawScrew(gca, VecTose3(Twists(:,i)),[0,0,1],30);
	DrawScrew(gca, VecTose3(Js(:,i)),[0,1,0],30);
	hold on;
end

