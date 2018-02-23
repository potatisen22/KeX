clc;
close all;
clear all;
fi = %rotation about x2-axis
tetha = %rotation about y1-axis
my = %rotation about ze-axis
dfi = %time derivative of fi
dtetha = %time derivative of tetha
dmy = %time derivative of my
TETHA = [fi, tetha,my]'
dTetha = [pfi,ptetha,pmy]' %NOT ANGULAR VELOCITY VECTOR!
internalw = [1,0,-sin(tetha);0,cos(fi),cos(tetha)*sin(fi);0,-sin(fi),cos(tetha)*sin(fi)]*dTetha;
Rx =[1,0,0;0,cos(fi),-sin(fi);0,sin(fi),cos(fi)];
Ry = [cos(tetha),0,sin(tetha);0,1,0;-sin(tetha),0,cos(tetha)];
Rz =[cos(my),-sin(my),0;sin(my),cos(my),0;0,0,1];
R=Rz*Ry*Rx;
Tinv=[1,0,-sin(tetha);0,cos(fi),cos(tetha)*sin(fi);0,-sin(fi),cos(tetha)*cos(fi)];
T = inv(Tinv);
J =[R,zeros(3,3);zeros(3,3),T];
Vb = 
wb =
v=[Vb,wb]'
dxi=J*v;
%dynamics below
m = ;%mass of the quad
Ixx = ;
Iyy = ;
Izz = ;
I = [Ixx,0,0;0,Iyy,0;0,0,Izz]; %inertia matrix is diagonal with our definition of the quadrotor