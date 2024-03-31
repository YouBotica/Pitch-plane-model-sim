%% Parameters:
Ws = 3500; % lbs
Iy = 25000; % in-lb-sec^2
l1 = 8*12; % in
l2 = 8*12; % in
g = 386.06; % in/sec^2
% K1 = 162.89; % 589.5; % lbs/in
% K2 = 114.02; % 842.14; % lbs/in
C1 = 0.0; % lbs*sec/in
C2 = 0.0; % lbs*sec/in
ms = Ws / g;

%% 1.)  Analytically determine equivalent suspension 
% stiffness and damping at each corner that will provide a 2 Hz ride frequency. 

omega_des = (2*2*pi);

% x_sp = -0.08*2*l1;

Ka = ms*omega_des^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc = Ka*l1^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2 = Ka / 1.7;
K1 = 0.7*K2;

Kb = K1*l1 - K2*l2; 

%% Linear pitch-plane model:

A = [-Ka/ms, Kb/ms; 
     Kb/Iy, -Kc/Iy];

B = [K1/ms, K2/ms; -l1*K1/Iy, l2*K2/Iy];
C = [1, 0];
D = [0, 0];

%%

% A = [
%     0, 1, 0, 0;
%     (-K1 - K2)/ms, 0, (K1*l1 - K2*l2)/ms, 0;
%     0, 0, 0, 1;
%     (K1*l1 - K2*l2)/Iy, 0, (-K1*l1^2 - K2*l2^2)/Iy, 0;
%     ];

A = [
    0, 1, 0, 0;
    (-K1 - K2)/ms, 0, (K1*l1 - K2*l2)/ms, 0;
    0, 0, 0, 1;
    (K1*l1 - K2*l2)/Iy, 0, (-K1*l1^2 - K2*l2^2)/Iy, 0;
    ];

B = [K1/ms, K2/ms; 0, 0; -l1*K1/Iy, l2*K2/Iy; 0, 0];

C = [1, 0, 0, 0; 0, 0, 1, 0];
D = [0, 0; 0, 0];

[magx,phasex,wx]=bode(ss(A,B(:,1),C(1,:),[0]),logspace(0,2));
 mgx(1:50)=magx;
 phx(1:50)=phasex;
 
[magtheta,phasetheta,wtheta]=bode(ss(A,B(:,1),C(2,:),[0]),logspace(0,2));
 mgth(1:50)=magtheta;
 phth(1:50)=phasetheta;
 
subplot(1,1,1),semilogx(wx,20*log10(mgx),wtheta,20*log10(mgth))

%%
%%pitch_plane_model = ss(A, B(:,1), C, [0]);





