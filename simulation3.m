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

%% Plot the results:

legends_mgx = {};
legends_mgth = {};

for i = 1:length(C2_array)
   pitch_plane_model(K1, K2, C1_array(i), C2_array(i), l1, l2, ms, Iy); 
   legends_mgx{end+1} = ['Heave mag. plot with C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for x']; % Example legend entry for mgx
   legends_mgth{end+1} = ['Pitch mag. plot with C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for theta']; % Example legend entry for mgth
   % legend(legendEntries); % Update legend with the new entries
end

subplot(2,1,1);
legend(legends_mgx);
subplot(2,1,2);
legend(legends_mgth);




%% 2.)  Using Olley ride criteria determine appropriate front and rear corner stiffnesses.

omega_des = (2*pi); % 1 Hz ride and heave desired frequencies

% x_sp = -0.08*2*l1;

Ka = ms*omega_des^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc = Iy*omega_des^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2 = Ka / 1.7;
K1 = 0.7*K2;

Kb = K1*l1 - K2*l2; 

%% Plot the results:

legends_mgx = {};
legends_mgth = {};

for i = 1:length(C2_array)
   pitch_plane_model(K1, K2, C1_array(i), C2_array(i), l1, l2, ms, Iy); 
   legends_mgx{end+1} = ['Heave mag. plot with C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for x']; % Example legend entry for mgx
   legends_mgth{end+1} = ['Pitch mag. plot with C1 = ' num2str(C1_array(i)) ', C2 = ' num2str(C2_array(i)) ' for theta']; % Example legend entry for mgth

end

subplot(2,1,1);
legend(legends_mgx);
subplot(2,1,2);
legend(legends_mgth);


%% 3.)  Construct a block diagram of the pitch plane vehicle system.  
% Simulate the system response to the vehicle experiencing a time phased 1 in step input.
% 4.) Use the above simulation to compare the response of the parameters determined in
% 1.) and 2.) above at various speeds.
% t_des = 1;

speeds = [10, 30, 60, 90, 100, 120]; % (l1 + l2) / t_des;


mph2in_sec = 1/0.0568182;


legends_x1_in = {};
legends_x2_in = {};
for i = 1:length(speeds)
    speed = speeds(i)*mph2in_sec; % Convert speed to in/sec before feeding into the Simulink model

    % Simulate:
    out = sim("block_sim3");

    legends_x1_in{end+1} = ['Speed = ' num2str(speeds(i))]; % Example legend entry for mgx
    
    % Input plots:
    figure(1);
    % Plot:
    subplot(2,1,1);
    plot(out.x1_in, 'LineWidth', 1.5);
    hold on;
    grid on;
    
    subplot(2,1,2);
    plot(out.x2_in, 'LineWidth', 1.5);
    hold on;
    grid on;

    % Output plots:
    figure(2);
    subplot(4,1,1);
    plot(out.x, 'LineWidth', 1.5);
    hold on;
    grid on;

    subplot(4,1,2);
    plot(out.dx, 'LineWidth', 1.5);
    hold on;
    grid on;

    subplot(4,1,3);
    plot(out.th, 'LineWidth', 1.5);
    hold on;
    grid on;

    subplot(4,1,4);
    plot(out.dth, 'LineWidth', 1.5);
    hold on;
    grid on;


end
figure(1)
subplot(2,1,1);
legend(legends_x1_in);
ylim([0,1.2]);
subplot(2,1,2);
legend(legends_x1_in);
ylim([0,1.2]);

figure(2)
subplot(4,1,1);
legend(legends_x1_in);
subplot(4,1,2);
legend(legends_x1_in);
subplot(4,1,3);
legend(legends_x1_in);
subplot(4,1,4);
legend(legends_x1_in);





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
    
    % Plot mgx
    subplot(2,1,1); % This creates the first subplot in a 2-row, 1-column grid
    semilogx(wx, 20*log10(mgx));
    title('Magnitude of x');
    xlabel('Frequency (rad/sec)');
    ylabel('Magnitude (dB)');
    hold on;
    grid on;

    % Plot mgth
    subplot(2,1,2); % This creates the second subplot in the same grid
    semilogx(wtheta, 20*log10(mgth));
    title('Magnitude of theta');
    xlabel('Frequency (rad/sec)');
    ylabel('Magnitude (dB)');
    hold on;
    grid on;
    % subplot(1,1,1),semilogx(wx,20*log10(mgx),wtheta,20*log10(mgth))

end





