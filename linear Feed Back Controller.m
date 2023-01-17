clc
clear all
close all

%my Parameters
mass     = 0.20;
p = 1.255;  %Air density
l = 0.6223; %length
breadth = 0.2032; %breadth
j = (l^2+breadth^2)*mass/2; %inertia
a = 0.19;               %cross area
K = 0;



%% system dynamics

% A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0 ; 0 0 0 0 0 0; 0 0 0 0 0 0];
% B = [0 0 0; 0 0 0; 0 0 0; 1/mass 1/mass 0; 0 0 1; length/j -length/j 0];

A = [0 0 0 1 0; 0 0 1 0 0; 0 0 0 0 1; 0 0 0 0 0; 0 0 0 0 -K];
B = [0 0; 0 0; 0 0; 1/mass 1/mass; l/j -l/j];
ctrb(A,B);
k = place(A,B,[-0.1 -0.2 -0.3 -0.4 -0.5]);
AA = A-B*k;
sys1 = ss(AA,B,[0 0 0 0 0], 0);
f1 = 0.01;
f2 = 0.01;
T = 0:0.001:65;

%% way point 1

x0_1 = 0;
y0_1 = 0;
xr_1 = 30;
yr_1 = 30;
ex_1 = x0_1 - xr_1;
ey_1 = y0_1 - yr_1;
initial = [ex_1 ey_1 0*pi/180 0 0];

T = 0:0.001:65;

u_1 = 0.01*zeros(2,length(T));
[y_1, T, x_1] = lsim(sys1,u_1,T,initial);
X_1(:,1) = xr_1+x_1(:,1);
Y_1(:,1) = yr_1+x_1(:,2);

%% plots
figure(1)
plot(T,x_1);
title("Lsim Plot");
legend(["x", "y", "phi", "V_x", "omega"]);
xlabel("Time (seconds)");

figure(2)
plot(x_1(:,1),x_1(:,2));
title("Trajectory");
xlabel("x position");
ylabel("y position");

figure(3)
plot(X_1(:,1),Y_1(:,1));
title("Trajectory");
xlabel("x position");
ylabel("y position");

%% waypoint 2

x0_2 = X_1(length(T),1);
y0_2 = Y_1(length(T),1);
xr_2 = 60;
yr_2 = 0;
ex_2 = x0_2 - xr_2;
ey_2 = y0_2 - yr_2;

initial = [ex_2 ey_2 x_1(length(T),3)*pi/180 0 0];

T = 0:0.001:65;

u_2 = 0.01*zeros(2,length(T));
[y_2, T, x_2] = lsim(sys1,u_2,T,initial);
X_2(:,1) = xr_2+x_2(:,1);
Y_2(:,1) = yr_2+x_2(:,2);

%% plots
figure(4)
plot(T,x_2);
title("Lsim Plot");
legend(["x", "y", "phi", "V_x", "omega"]);
xlabel("Time (seconds)");

figure(5)
plot(x_2(:,1),x_2(:,2));
title("Trajectory");
xlabel("x position");
ylabel("y position");

figure(6)
plot(X_2(:,1),Y_2(:,1));
title("Trajectory");
xlabel("x position");
ylabel("y position");

%% waypoint 3

x0_3 = X_2(length(T),1);
y0_3 = Y_2(length(T),1);
xr_3 = 30;
yr_3 = -30;
ex_3 = x0_3 - xr_3;
ey_3 = y0_3 - yr_3;
initial = [ex_3 ey_3 x_2(length(T),3)*pi/180 0 0];

T = 0:0.001:65;

u_3 = 0.01*zeros(2,length(T));
[y_3, T, x_3] = lsim(sys1,u_3,T,initial);
X_3(:,1) = xr_3+x_3(:,1);
Y_3(:,1) = yr_3+x_3(:,2);

%% plots
figure(7)
plot(T,x_3);
title("Lsim Plot");
legend(["x", "y", "phi", "V_x", "omega"]);
xlabel("Time (seconds)");

figure(8)
plot(x_3(:,1),x_3(:,2));
title("Trajectory");
xlabel("x position");
ylabel("y position");

figure(9)
plot(X_3(:,1),Y_3(:,1));
title("Trajectory");
xlabel("x position");
ylabel("y position");

%% waypoint 4

x0_4 = X_3(length(T),1);
y0_4 = Y_3(length(T),1);
xr_4 = 0;
yr_4 = 0;
ex_4 = x0_4 - xr_4;
ey_4 = y0_4 - yr_4;
initial = [ex_4 ey_4 x_3(length(T),3)*pi/180 0 0];

T = 0:0.001:65;

u_4 = 0.01*zeros(2,length(T));
[y_4, T, x_4] = lsim(sys1,u_4,T,initial);
X_4(:,1) = xr_4+x_4(:,1);
Y_4(:,1) = yr_4+x_4(:,2);

%% plots
figure(10)
plot(T,x_4);
title("Lsim Plot");
legend(["x", "y", "phi", "V_x", "omega"]);
xlabel("Time (seconds)");

figure(11)
plot(x_4(:,1),x_4(:,2));
title("Trajectory");
xlabel("x position");
ylabel("y position");

figure(12)
plot(X_4(:,1),Y_4(:,1));
title("Trajectory");
xlabel("x position");
ylabel("y position");

%% Square trajectory
x1 = [0 30];
y1 = [0 30];
x2 = [30 60];
y2 = [30 0];
x3 = [60 30];
y3 = [0 -30];
x4 = [30 0];
y4 = [-30 0];



%% Full Plots
Full_t = [T ;length(T)+T; 2*length(T)+T; 3*length(T)+T];
yaw(:) = [x_1(:,3); x_2(:,3); x_3(:,3); x_4(:,3)];
positionx(:) = [X_1(:,1);X_2(:,1);X_3(:,1);X_4(:,1)];
positiony(:) = [Y_1(:,1);Y_2(:,1);Y_3(:,1);Y_4(:,1)];

figure(13)
plot(positionx,positiony);
hold on
line(x1,y1,'Color','red','LineStyle','--')
hold on
line(x2,y2,'Color','red','LineStyle','--')
hold on
line(x3,y3,'Color','red','LineStyle','--')
hold on
line(x4,y4,'Color','red','LineStyle','--')

figure(14)
plot(Full_t,yaw);
