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


C2_array = logspace(log10(0.1), log10(100), 10);
C1_array = 0.7*C2_array;

%% 1.)  Analytically determine equivalent suspension 
% stiffness and damping at each corner that will provide a 2 Hz ride frequency. 

omega_des = (2*2*pi);

% x_sp = -0.08*2*l1;

Ka = ms*omega_des^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc = Ka*l1^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2 = Ka / 1.7;
K1 = 0.7*K2;

Kb = K1*l1 - K2*l2; 

legendEntries = {};

for i = 1:length(C2_array)
   pitch_plane_model(K1, K2, C1_array(i), C2_array(i), l1, l2, ms, Iy); 
   legendEntries{end+1} = ['C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for x']; % Example legend entry for mgx
   legendEntries{end+1} = ['C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for theta']; % Example legend entry for mgth
   legend(legendEntries); % Update legend with the new entries
   hold on; % Ensure new plots are added without clearing the old ones
   hold on;
end




%% 2.)  Using Olley ride criteria determine appropriate front and rear corner stiffnesses.

omega_des = (2*pi); % 1 Hz ride and heave desired frequencies

% x_sp = -0.08*2*l1;

Ka = ms*omega_des^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc = Iy*omega_des^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2 = Ka / 1.7;
K1 = 0.7*K2;

Kb = K1*l1 - K2*l2; 


pitch_plane_model(K1, K2, C1, C2, l1, l2, ms, Iy);

%% 3.)  Construct a block diagram of the pitch plane vehicle system.  
% Simulate the system response to the vehicle experiencing a time phased 1 in step input.


%%

function pitch_plane_model(K1, K2, C1, C2, l1, l2, ms, Iy)

    A = [
        0, 1, 0, 0;
        (-K1 - K2)/ms, (-C1 - C2)/ms, (K1*l1 - K2*l2)/ms, (l1*C1 - l2*C2)/ms;
        0, 0, 0, 1;
        (K1*l1 - K2*l2)/Iy, (l1*C1 - l2*C2)/Iy, (-K1*l1^2 - K2*l2^2)/Iy, (-l1*l1*C1 - l2*l2*C2)/Iy;
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

end





