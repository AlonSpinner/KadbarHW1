%% Check Trajectories
%run before any of the sections
load('Parameters.mat');
T=prm.T;
dt=prm.dt;
t=0:dt:T;
%% Const xyz
x=x_plan('const',t);
v=v_plan('const',t);
a=a_plan('const',t);

fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[m]'); title('Position')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[m/s]'); title('Velocity')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('Acceleration')

plot(ax11,t,x','linewidth',2);
plot(ax12,t,v','linewidth',2);
plot(ax13,t,a','linewidth',2);

legend(ax11,'x','y','z');
sgtitle(fig,'Constant Velocity - Tool Kinematics');
%% Const q,q_dot,q,q_dot2
q=q_plan('const',t,'elbows',[1,1]);
q_dot_Numeric=q_dot_plan('const',t,'elbows',[1,1],'method','numeric');
q_dot2_Numeric=q_dot2_plan('const',t,'elbows',[1,1],'method','numeric');
q_dot_Jacobian=q_dot_plan('const',t,'elbows',[1,1],'method','jacobian');
q_dot2_Jacobian=q_dot2_plan('const',t,'elbows',[1,1],'method','jacobian');

%plot q
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m]'); title('d_3')

plot(ax11,t,rad2deg(q(1,:)),'linewidth',2);
plot(ax11,ax11.XLim,180*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax11,ax11.XLim,-180*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax12,t,rad2deg(q(2,:)),'linewidth',2); 
plot(ax12,ax11.XLim,90*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax12,ax11.XLim,-90*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax13,t,q(3,:),'linewidth',2);
plot(ax13,ax13.XLim,0*[-1,1],'linewidth',2,'color','red'); %boundries

sgtitle(fig,'Constant Velocity - Joint Position');
legend(ax11,'\theta_1','Boundries');
legend(ax12,'\theta_2','Boundries');
legend(ax13,'d_3','Boundries')

%plot q_dot
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s]'); title('\theta_1 velocity')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s]'); title('\theta_2 velocity')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s]'); title('d_3 velocity')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s]'); 
title(sprintf('theta_1 velocity difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s]');
title(sprintf('theta_2 velocity difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s]'); 
title(sprintf('d_3 velocity difference\nNumeric-Jacobian'));

plot(ax11,t,rad2deg(q_dot_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot_Numeric(1,:)-q_dot_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot_Numeric(2,:)-q_dot_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot_Numeric(3,:)-q_dot_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Constant Velocity - Joint Velocities');
legend(ax11,'Numeric','Jacobian');

%Plot q_dot_2
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s^2]'); title('\theta_1 acceleration')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s^2]'); title('\theta_2 acceleration')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('d_3 acceleration')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s^2]'); 
title(sprintf('theta_1 acceleration difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s^2]');
title(sprintf('theta_2 acceleration difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s^2]'); 
title(sprintf('d_3 acceleration difference\nNumeric-Jacobian'));


plot(ax11,t,rad2deg(q_dot2_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot2_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot2_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot2_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot2_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot2_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot2_Numeric(1,:)-q_dot2_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot2_Numeric(2,:)-q_dot2_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot2_Numeric(3,:)-q_dot2_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Constant Velocity - Joint Acceleration');
legend(ax11,'Numeric','Jacobian');
%% Trapezoid xyz
x=x_plan('trapz',t);
v=v_plan('trapz',t);
a=a_plan('trapz',t);

fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[m]'); title('Position')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[m/s]'); title('Velocity')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('Acceleration')

plot(ax11,t,x','linewidth',2);
plot(ax12,t,v','linewidth',2);
plot(ax13,t,a','linewidth',2);

legend(ax11,'x','y','z');
sgtitle(fig,'Trapezoid Velocity - Tool Kinematics');
%% Trapz q,q_dot,q,q_dot2
q=q_plan('trapz',t,'elbows',[1,1]);
q_dot_Numeric=q_dot_plan('trapz',t,'elbows',[1,1],'method','numeric');
q_dot2_Numeric=q_dot2_plan('trapz',t,'elbows',[1,1],'method','numeric');
q_dot_Jacobian=q_dot_plan('trapz',t,'elbows',[1,1],'method','jacobian');
q_dot2_Jacobian=q_dot2_plan('trapz',t,'elbows',[1,1],'method','jacobian');

%plot q
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m]'); title('d_3')

plot(ax11,t,rad2deg(q(1,:)),'linewidth',2);
plot(ax11,ax11.XLim,180*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax11,ax11.XLim,-180*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax12,t,rad2deg(q(2,:)),'linewidth',2); 
plot(ax12,ax11.XLim,90*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax12,ax11.XLim,-90*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax13,t,q(3,:),'linewidth',2);
plot(ax13,ax13.XLim,0*[-1,1],'linewidth',2,'color','red'); %boundries

sgtitle(fig,'Trapezoid Velocity - Joint Position');
legend(ax11,'\theta_1','Boundries');
legend(ax12,'\theta_2','Boundries');
legend(ax13,'d_3','Boundries')

%plot q_dot
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s]'); title('\theta_1 velocity')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s]'); title('\theta_2 velocity')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s]'); title('d_3 velocity')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s]'); 
title(sprintf('theta_1 velocity difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s]');
title(sprintf('theta_2 velocity difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s]'); 
title(sprintf('d_3 velocity difference\nNumeric-Jacobian'));

plot(ax11,t,rad2deg(q_dot_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot_Numeric(1,:)-q_dot_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot_Numeric(2,:)-q_dot_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot_Numeric(3,:)-q_dot_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Trapezoid Velocity - Joint Velocities');
legend(ax11,'Numeric','Jacobian');

%Plot q_dot_2
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s^2]'); title('\theta_1 acceleration')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s^2]'); title('\theta_2 acceleration')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('d_3 acceleration')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s^2]'); 
title(sprintf('theta_1 acceleration difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s^2]');
title(sprintf('theta_2 acceleration difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s^2]'); 
title(sprintf('d_3 acceleration difference\nNumeric-Jacobian'));


plot(ax11,t,rad2deg(q_dot2_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot2_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot2_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot2_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot2_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot2_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot2_Numeric(1,:)-q_dot2_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot2_Numeric(2,:)-q_dot2_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot2_Numeric(3,:)-q_dot2_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Trapezoid Velocity - Joint Acceleration');
legend(ax11,'Numeric','Jacobian');
%% Poly xyz
x=x_plan('poly',t);
v=v_plan('poly',t);
a=a_plan('poly',t);

fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[m]'); title('Position')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[m/s]'); title('Velocity')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('Acceleration')

plot(ax11,t,x','linewidth',2);
plot(ax12,t,v','linewidth',2);
plot(ax13,t,a','linewidth',2);

legend(ax11,'x','y','z');
sgtitle(fig,'Polynomial Trajectory - Tool Kinematics');

%% poly q,q_dot,q,q_dot2
q=q_plan('poly',t,'elbows',[1,1]);
q_dot_Numeric=q_dot_plan('poly',t,'elbows',[1,1],'method','numeric');
q_dot2_Numeric=q_dot2_plan('poly',t,'elbows',[1,1],'method','numeric');
q_dot_Jacobian=q_dot_plan('poly',t,'elbows',[1,1],'method','jacobian');
q_dot2_Jacobian=q_dot2_plan('poly',t,'elbows',[1,1],'method','jacobian');

%plot q
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m]'); title('d_3')

plot(ax11,t,rad2deg(q(1,:)),'linewidth',2);
plot(ax11,ax11.XLim,180*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax11,ax11.XLim,-180*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax12,t,rad2deg(q(2,:)),'linewidth',2); 
plot(ax12,ax11.XLim,90*[1,1],'linewidth',2,'color','red'); %boundries top
plot(ax12,ax11.XLim,-90*[1,1],'linewidth',2,'color','red'); %boundries bot
plot(ax13,t,q(3,:),'linewidth',2);
plot(ax13,ax13.XLim,0*[-1,1],'linewidth',2,'color','red'); %boundries

sgtitle(fig,'Polynomial Velocity - Joint Position');
legend(ax11,'\theta_1','Boundries');
legend(ax12,'\theta_2','Boundries');
legend(ax13,'d_3','Boundries')

%plot q_dot
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s]'); title('\theta_1 velocity')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s]'); title('\theta_2 velocity')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s]'); title('d_3 velocity')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s]'); 
title(sprintf('theta_1 velocity difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s]');
title(sprintf('theta_2 velocity difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s]'); 
title(sprintf('d_3 velocity difference\nNumeric-Jacobian'));

plot(ax11,t,rad2deg(q_dot_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot_Numeric(1,:)-q_dot_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot_Numeric(2,:)-q_dot_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot_Numeric(3,:)-q_dot_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Polynomial Velocity - Joint Velocities');
legend(ax11,'Numeric','Jacobian');

%Plot q_dot_2
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(2,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s^2]'); title('\theta_1 acceleration')
ax12=subplot(2,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s^2]'); title('\theta_2 acceleration')
ax13=subplot(2,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('d_3 acceleration')

ax21=subplot(2,3,4,'parent',fig); 
hold(ax21,'on'); grid(ax21,'on'); xlabel(ax21,'[s]'); ylabel(ax21,'[degree/s^2]'); 
title(sprintf('theta_1 acceleration difference\nNumeric-Jacobian'));
ax22=subplot(2,3,5,'parent',fig);
hold(ax22,'on'); grid(ax22,'on'); xlabel(ax22,'[s]'); ylabel(ax22,'[degree/s^2]');
title(sprintf('theta_2 acceleration difference\nNumeric-Jacobian'));
ax23=subplot(2,3,6,'parent',fig); 
hold(ax23,'on'); grid(ax23,'on'); xlabel(ax23,'[s]'); ylabel(ax23,'[m/s^2]'); 
title(sprintf('d_3 acceleration difference\nNumeric-Jacobian'));


plot(ax11,t,rad2deg(q_dot2_Numeric(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_dot2_Jacobian(1,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_dot2_Numeric(2,:)),'linewidth',2); 
plot(ax12,t,rad2deg(q_dot2_Jacobian(2,:)),'linewidth',2); 
plot(ax13,t,q_dot2_Numeric(3,:),'linewidth',2);
plot(ax13,t,q_dot2_Jacobian(3,:),'linewidth',2);

plot(ax21,t,rad2deg(q_dot2_Numeric(1,:)-q_dot2_Jacobian(1,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax22,t,rad2deg(q_dot2_Numeric(2,:)-q_dot2_Jacobian(2,:)),'linewidth',2,'color',[0,0.5,0]);
plot(ax23,t,q_dot2_Numeric(3,:)-q_dot2_Jacobian(3,:),'linewidth',2,'color',[0,0.5,0]);

sgtitle(fig,'Polynomial Velocity - Joint Acceleration');
legend(ax11,'Numeric','Jacobian');

%% Graphic Comparison In Joint Space - position
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m]'); title('d_3')

q_const=q_plan('const',t,'elbows',[1,1]);
q_trapz=q_plan('trapz',t,'elbows',[1,1]);
q_poly=q_plan('poly',t,'elbows',[1,1]);

plot(ax11,t,rad2deg(q_const(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_trapz(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_poly(1,:)),'linewidth',2);

plot(ax12,t,rad2deg(q_const(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_trapz(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_poly(2,:)),'linewidth',2);

plot(ax13,t,q_const(3,:),'linewidth',2);
plot(ax13,t,q_trapz(3,:),'linewidth',2);
plot(ax13,t,q_poly(3,:),'linewidth',2);

legend(ax11,'const','trapz','poly');
sgtitle(fig,'Trajectory Comparison - Joint Position');

%% Graphic Comparison In Joint Space - Velocity
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s]'); title('d_3')

q_const=q_dot_plan('const',t,'elbows',[1,1]);
q_trapz=q_dot_plan('trapz',t,'elbows',[1,1]);
q_poly=q_dot_plan('poly',t,'elbows',[1,1]);

plot(ax11,t,rad2deg(q_const(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_trapz(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_poly(1,:)),'linewidth',2);

plot(ax12,t,rad2deg(q_const(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_trapz(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_poly(2,:)),'linewidth',2);

plot(ax13,t,q_const(3,:),'linewidth',2);
plot(ax13,t,q_trapz(3,:),'linewidth',2);
plot(ax13,t,q_poly(3,:),'linewidth',2);

legend(ax11,'const','trapz','poly');
sgtitle(fig,'Trajectory Comparison - Joint Velocity');

%% Graphic Comparison In Joint Space - Acceleration
fig=figure('color',[1,1,1],'position',[300,100,1100,600]);
ax11=subplot(1,3,1,'parent',fig); 
hold(ax11,'on'); grid(ax11,'on'); xlabel(ax11,'[s]'); ylabel(ax11,'[degree/s^2]'); title('\theta_1')
ax12=subplot(1,3,2,'parent',fig);
hold(ax12,'on'); grid(ax12,'on'); xlabel(ax12,'[s]'); ylabel(ax12,'[degree/s^2]'); title('\theta_2')
ax13=subplot(1,3,3,'parent',fig); 
hold(ax13,'on'); grid(ax13,'on'); xlabel(ax13,'[s]'); ylabel(ax13,'[m/s^2]'); title('d_3')

q_const=q_dot2_plan('const',t,'elbows',[1,1]);
q_trapz=q_dot2_plan('trapz',t,'elbows',[1,1]);
q_poly=q_dot2_plan('poly',t,'elbows',[1,1]);

plot(ax11,t,rad2deg(q_const(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_trapz(1,:)),'linewidth',2);
plot(ax11,t,rad2deg(q_poly(1,:)),'linewidth',2);

plot(ax12,t,rad2deg(q_const(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_trapz(2,:)),'linewidth',2);
plot(ax12,t,rad2deg(q_poly(2,:)),'linewidth',2);

plot(ax13,t,q_const(3,:),'linewidth',2);
plot(ax13,t,q_trapz(3,:),'linewidth',2);
plot(ax13,t,q_poly(3,:),'linewidth',2);

legend(ax11,'const','trapz','poly');
sgtitle(fig,'Trajectory Comparison - Joint Acceleration');