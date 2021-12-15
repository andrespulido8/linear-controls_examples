clc; clear;
%% LQR Controller for altitude control
% Author: Andres Pulido
% November 2021

A = [ -0.0064, 9.2550, 0, -9.81, -0.896;
    -0.0234, -5.1414, 0, 0, 0.9519;
    0, 25., 0, -24.9836, 0;
    0, 0, 0, 0, 1.0;
    -0.7047, -485.6225, 0, 0, -31.4645
    ];
B = [-0.0520; 0.1016; 0; 0; 116.3];
C = eye(5);
D = [0;0;0;0;0];
Q = diag([1, 1, 1, 1, 1]);
R = 0.1;
P = ss(A, B, C, D);

Actuator = tf(20,[1, 20]);
K = lqr(A,B,Q,R);
PA = series(Actuator, P);

%% Regulation
T = feedback(PA,K,1,[1 2 3 4 5], -1);

t = 0:.01:5;
y = initial(T,[22.352; 0; 100; 0; 0; 0],t);
plot(t,y(:,3))


Q1 = diag([1, 1, 1.11, 1, 1]);
Q2 = diag([100, 1, 1, 1, 1]);
Q3 = diag([1, 100, 1, 1, 1]);
R = 1;
 
K = lqr(A,B,Q,R);
K1 = lqr(A,B,Q1,R);
K2 = lqr(A,B,Q2,R);
K3 = lqr(A,B,Q3,R);

T = feedback(PA,K,1,[1 2 3 4 5], -1);
T1 = feedback(PA,K1,1,[1 2 3 4 5], -1);
T2 = feedback(PA,K2,1,[1 2 3 4 5], -1);
T3 = feedback(PA,K3,1,[1 2 3 4 5], -1);

y1 = initial(T1,[22.352; 0; 100; 0; 0; 0],t);
y2 = initial(T2,[22.352; 0; 100; 0; 0; 0],t);
y3 = initial(T3,[22.352; 0; 100; 0; 0; 0],t);
y = initial(T,[22.352; 0; 100; 0; 0; 0],t);

figure(1)
plot(t,y(:,3),'k',t,y1(:,3),'r',t,y2(:,3),'g', t, y3(:,3),'y');
% figure(2)
% plot(t,y(:,4),'k',t,y1(:,4),'r',t,y2(:,4),'g', t, y3(:,4),'y');
% figure(3)
% plot(t,y2(:,3),'k',t,y3(:,3),'r',t,y4(:,3),'g');

%% Tracking
Abar = [A zeros(5,1);-[0 0 1 0 0] 0];
Bbar = [B;0];
 
Q = eye(6);
Q1 = diag([1, 1, 100, 1, 1, 1]);
Q2 = diag([100, 1, 1, 1, 1, 1]);
Q3 = diag([1, 100, 1, 1, 1, 1]);

R = 1;

X = series(K1(3),T);

% K = lqr(Abar,Bbar,Q,R);
K1 = lqr(Abar,Bbar,Q1,R);
% K2 = lqr(Abar,Bbar,Q2,R);
% K3 = lqr(Abar,Bbar,Q3,R);

%PAbar = series(Actuator, Pbar);

I = tf(1,[1 0]);
 
T = feedback(PA,K1(1:5),1,[1 2 3 4 5], -1);
G = series(I,-K1(6));
S = series(G,T);
X1 = feedback(S,1,1,3);
% T = feedback(P,K2(1:2),1,[1:2]);
% G = series(I,-K2(3));
% S = series(G,T);
% X2 = feedback(S,1,1,1);
% 
% T = feedback(P,K3(1:2),1,[1:2]);
% G = series(I,-K3(3));
% S = series(G,T);
% X3 = feedback(S,1,1,1);
 
% t = 0:.1:10;
% opt = stepDataOptions;
% opt.InputOffset = [100 0 0 0 0 ];
% opt.StepAmplitude = 100;
% [y1,t1] = step(X1,t, opt);
y = lsim(X, repelem(200,length(t)), t, [22.352; 0; 100; 0; 0; 0]);
% y1 = lsim(X1, repelem(200,length(t)), t, [22.352; 0; 100; 0; 0; 0; 0]);
% [y2,t2] = step(X2,t);
% [y3,t3] = step(X3,t);
% 
figure(4)
%plot(t,y1(:,1),'k',t,y2(:,1),'g',t,y3(:,1),'r');
plot(t,y(:,3),'k', t, repelem(200,length(t)),'b');
grid on
title('LQR')
xlabel('time(s)')
ylabel('pitch rate(deg)')

figure(6)
rlocus(X(5))
% 
% 
% 
% %---------------------------------------------------------------
% % Make closed-loop with observer
% %---------------------------------------------------------------
% 
% %
% % My guess for the dynamics (purposely make them wrong)
% %
% Aguess = A*.8;       % my guess for state matrix is wrong
% Bguess = B;          % my guess for input matrix is correct
% Cguess = [1 0];      % my guess is we can only measure the first state
% Dguess = 0;          % my guess for feedthrough matrix is correct
% 
% %
% % Try Q,R matrices for dual problem
% %
% Qdual = [1 0;0 100];
% Rdual = 0.001;
% 
% %
% % Make observer gain as controller for dual system
% %
% L = lqr(Aguess',Cguess',Qdual,Rdual);   % compute controller
% L = L';                                 % take transpose from dual system
% 
% %
% % Make observer to have full-state output
% %
% Aobserver = Aguess - L*Cguess;
% Bobserver = [Bguess L];
% Cobserver = eye(2);
% Dobserver = zeros(2,2);
% O = ss(Aobserver,Bobserver,Cobserver,Dobserver);
% 
% %
% % Augment previous LQR controller to get states from observer
% %
% KK = series(O,K3(1:2));
% 
% %
% % Make closed-loop system
% %
% T = feedback(P,KK(1:2),1,[3 1]);
% G = series(I,-K3(3));
% S = series(G,T);
% X = feedback(S,1,1,1);
% 
% %
% % Compute step response
% %
% t = 0:.1:10;
% [yy,tt] = step(X,t);
% 
% %
% % Plot the response using full-state feedback (we called them y3 made K3) 
% %   against the response using observer-estimated states
% %
% figure(5)
% plot(t,y3(:,1),'k',t,yy(:,1),'r');
% legend('measured states','observer states');
% ylabel('Position');
% 
% figure(6)
% plot(t,y3(:,2),'k',t,yy(:,2),'r');
% legend('measured states','observer states');
% ylabel('Velocity');
% 
% 
% figure(7)
% plot(t,y3(:,3),'k',t,yy(:,3),'r');
% legend('measured states','observer states');
% ylabel('Actuation');



