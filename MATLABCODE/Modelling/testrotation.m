clc;
close all;
clear all;
%angles and velocities ar measurable
fi = pi %rotation about x2-axis (angle between Y2 and Y3 in Y-Z plane
tetha =pi %rotation about y1-axis (angle between Z1 and Z2) in X-Z plane
psi =pi %rotation about ze-axis (angle between Xe and X1) in Y-X plane
dfi =0.02 %time derivative of fi
dtetha =0.02 %time derivative of tetha
dpsi =0.02 %time derivative of my
TETHA = [fi, tetha,psi]'
dTetha = [dfi,dtetha,dpsi]' %NOT ANGULAR VELOCITY VECTOR!
internalw = [1,0,-sin(tetha);0,cos(fi),cos(tetha)*sin(fi);0,-sin(fi),cos(tetha)*sin(fi)]*dTetha;
Rx =[1,0,0;0,cos(fi),-sin(fi);0,sin(fi),cos(fi)];
Ry = [cos(tetha),0,sin(tetha);0,1,0;-sin(tetha),0,cos(tetha)];
Rz =[cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1];
R=Rz*Ry*Rx;
Tinv=[1,0,-sin(tetha);0,cos(fi),cos(tetha)*sin(fi);0,-sin(fi),cos(tetha)*cos(fi)];
T = inv(Tinv);
J =[R,zeros(3,3);zeros(3,3),T];
Vb = [0,10,0]; %should be read from sensors?
wb = [0,0,0]; %should be read from sensors?
v=[Vb,wb]'
dxi=J*v;
%dynamics below
m = 1;%mass of the quad in Kg 
Ixx = 1; %CHECK MOMENTS OF INERTIA
Iyy = 1;
Izz = 2;
I = [Ixx,0,0;0,Iyy,0;0,0,Izz]; %inertia matrix is diagonal with our definition of the quadrotor
%% 
fz = 37 %forces in z-direction
tx = 0%torque around x-axis
ty = 0%torque around y-axis
tz = 0%torque around z-axis
Ftaub = [0,0,fz,tx,ty,tz]' + [0,0,-m*9.82,0,0,0]' %input! first vector is from motors second vector is outer forces,  
dVbdwb=inv([m.*eye(3),zeros(3);zeros(3),I])*[Ftaub]-[cross(wb,(m.*Vb))';cross(wb,(I*wb'))']
%dxi = dVbdwb + dxi ? 