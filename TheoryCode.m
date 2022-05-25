%% Calculat Direct kinematics
syms t1 t2 d3 t4 t5 H L l1 l2 real

R1t0 = Rot(0,0,1,t1);
t1t0 = [0,0,H]';
A1t0 = BuildA(R1t0,t1t0);

R2t1 = Rot(1,0,0,t2);
t2t1 = [L,0,0]';
A2t1 = BuildA(R2t1,t2t1);

R3t2 = Rot(0,0,0,0);
t3t2 = [0,0,d3]';
A3t2 = BuildA(R3t2,t3t2);

R4t3 = Rot(0,0,1,t4);
t4t3 = [0,0,l1]';
A4t3 = BuildA(R4t3,t4t3);

R5t4 = Rot(0,-1,0,t5);
t5t4 = [0,0,0]';
A5t4 = BuildA(R5t4,t5t4);

RTt5 = Rot(0,0,0,0);
tTt5 = [0,0,l2]';
ATt5 = BuildA(RTt5,tTt5);

ATt0=A1t0*A2t1*A3t2*A4t3*A5t4*ATt5;
%% Finding Linear Jacobian
q=[t1,t2,d3,t4,t5];
T=ATt0(1:3,4);
J_L=jacobian(T,q);
%% Finding Angluar Jacobian - Whitney
R2t0=R1t0*R2t1;
R3t0=R2t0*R3t2;
R4t0=R3t0*R4t3;
R5t0=R4t0*R5t4;

J_A=sym(zeros(3,5));
J_A(:,1)=[0,0,1]';
J_A(:,2)=R1t0*[1,0,0]';
J_A(:,3)=[0,0,0]';
J_A(:,4)=R3t0*[0,0,1]';
J_A(:,5)=R4t0*[0,-1,0]';
%% Finding Angular Jacobian - Omega and derivatives - DID NOT WORK!!
syms t1 t2 d3 t4 t5 H L l1 l2 t real
RTt0=R1t0*R2t1*R3t2*R4t3*R5t4*RTt5;

k = regexprep(char(RTt0),{'\(t1\)','\(t2\)','\(d3\)','\(t4\)','\(t5\)'},{'\(t1\(t\)\)',...
    '\(t2\(t\)\)', '\(d3\(t\)\)','\(t4\(t\)\)','\(t5\(t\)\)'});
RTt0_dot=feval(symengine,'diff', k, t);
Omega=RTt0_dot*RTt0'; %<<<<<<< NOT ANIT SYMMETRIC!! WRONG CALCULATION!!

%% Transformation to Tool coordiante jacobians
RTt0=R1t0*R2t1*R3t2*R4t3*R5t4*RTt5;
R0tT=RTt0';
J_L_tool=R0tT*J_L;
J_A_tool=R0tT*J_A;
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
