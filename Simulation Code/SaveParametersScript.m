%% Instructions
%There is no garuntee that variale names will be kept save between segments
%Use the struct prm.variable to 'pass' values between sections

clear('prm'); %clear varaible if exists in workspace as not to add to it
%% Length Parameters
prm.H=0.2;
prm.L=0.1;
%% Trajectory Position
X=[0.1,0.25];
Y=[0.05,-0.15];
Z=[0.25,0.35];

prm.X=X;
prm.Y=Y;
prm.Z=Z;

prm.dX=X(2)-X(1);
prm.dY=Y(2)-Y(1);
prm.dZ=Z(2)-Z(1);
%% Time
T=2;
dT=1/6*T;

prm.T=2;
prm.dT=dT;
prm.T1=dT;
prm.T2=T-dT;
prm.dt=1/1000;
%% Direct Kinematics and Jacobians
syms t1 t2 d3 H L real
%----- Direct Kinematics
R1t0 = Rot(0,0,1,t1);
t1t0 = [0,0,H]';
A1t0 = BuildA(R1t0,t1t0);

R2t1 = Rot(1,0,0,t2);
t2t1 = [L,0,0]';
A2t1 = BuildA(R2t1,t2t1);

R3t2 = Rot(0,0,0,0);
t3t2 = [0,0,d3]';
A3t2 = BuildA(R3t2,t3t2);

A2t0=A1t0*A2t1;
A3t0=A2t0*A3t2;

prm.f_A1t0=matlabFunction(subs(A1t0,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});
prm.f_A2t0=matlabFunction(subs(A2t0,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});
prm.f_A3t0=matlabFunction(subs(A3t0,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});

%Linear Jacobian
q=[t1,t2,d3];
T=A3t0(1:3,4);
JL=jacobian(T,q);

prm.f_JL=matlabFunction(subs(JL,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});

%Rotational Jacobian
R2t0=R1t0*R2t1;
JA=sym(zeros(3,3));
JA(:,1)=[0,0,1]';
JA(:,2)=R1t0*[1,0,0]';
JA(:,3)=[0,0,0]';

prm.f_JA=matlabFunction(subs(JA,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});

%Tool Jaobians
R3t0=R1t0*R2t1*R3t2;
R0t3=R3t0';
JL_tool=R0t3*JL;
JA_tool=R0t3*JA;

prm.f_JL_tool=matlabFunction(subs(JL_tool,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});
prm.f_JA_tool=matlabFunction(subs(JA_tool,[H,L],[prm.H,prm.L]),'vars',{[t1,t2,d3]'});
%% Save to file
save('Parameters','prm');

%% Functions
function [R] = Rot(nx, ny, nz, theta) 
%Calcualte rotation matrix from normalized vector [nx,ny,nz] and angle in
%radians theta
N = [0 -nz ny;
     nz 0 -nx;
     -ny nx 0];
 
R=eye(3)+sin(theta)*N+(1-cos(theta))*N^2; %Rodrigues formula
% R = simplify(expm(N*theta)); %equivalent 
end

function A=BuildA(R,t)
    %create 4x4 transformation matrix from 3x3 rotation matrix and 3x1
    %translation vector
    h=[0,0,0,1];
    A=[R,t; h];
end