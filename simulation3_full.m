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

omega_des_1 = (2*2*pi);

% x_sp = -0.08*2*l1;

Ka_1 = ms*omega_des_1^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc_1 = Ka_1*l1^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2_1 = Ka_1 / 1.7;
K1_1 = 0.7*K2_1;

C2_1 = 21;
C1_1 = 15;

Kb_1 = K1_1*l1 - K2_1*l2; 



%% 2.)  Using Olley ride criteria determine appropriate front and rear corner stiffnesses.

omega_des_2 = (2*pi); % 1 Hz ride and heave desired frequencies

% x_sp = -0.08*2*l1;

Ka_2 = ms*omega_des_2^2; %(ms / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;
Kc_2 = Iy*omega_des_2^2; %(Iy / (1 - x_sp*sqrt(ms/Iy)))*omega_des^2;

K2_2 = Ka_2 / 1.7;
K1_2 = 0.7*K2_2;

C1_2 = 21;
C2_2 = 15;

Kb_2 = K1_2*l1 - K2_2*l2; 


%% 3.)  Construct a block diagram of the pitch plane vehicle system.  
% Simulate the system response to the vehicle experiencing a time phased 1 in step input.
% 4.) Use the above simulation to compare the response of the parameters determined in
% 1.) and 2.) above at various speeds.
% t_des = 1;

speeds = [30, 60, 100, 150]; % (l1 + l2) / t_des;


mph2in_sec = 1/0.0568182;


legends_x1_in = {};

legends_x = {};

line_width1 = 1.5;
line_width2 = 1.5;

for i = 1:length(speeds)
    speed = speeds(i)*mph2in_sec; % Convert speed to in/sec before feeding into the Simulink model

    % Simulate both configurations 
    % Config1: K1 = 589.5 lbs/in K2 = 842.1 lbs/in
    % .........C1 = 15 lbs*sec/in, C2 = 21 lbs*sec/in
    % Config2: K1 = 147.4 lbs/in K2 = 210.5 lbs/in
    % .........C1 = 15 lbs*sec/in, C2 = 21 lbs*sec/in
    
    % Simulate config1:
    K1 = K1_1; K2 = K2_1;
    C1 = C1_1; C2 = C2_1;
    out_1 = sim("block_sim3");
    
    % Simulate config2:
    K1 = K1_2; K2 = K2_2;
    C1 = C1_2; C2 = C2_2;
    out_2 = sim("block_sim3");
    

    legends_x1_in{end+1} = ['Speed = ' num2str(speeds(i))]; % Example legend entry for mgx

    % Input plots:
    figure(1);
    % Plot:
    subplot(2,1,1);
    plot(out_1.x1_in, 'LineWidth', line_width1);
    hold on;
    grid on;
    
    subplot(2,1,2);
    plot(out_1.x2_in, 'LineWidth', line_width1); % line_width2
    hold on;
    grid on;


    % Output plots:
    figure(2);
    subplot(4,1,1);
    % plot(out_1.x, 'LineWidth', line_width2, 'Marker', '_', 'Color', [0, 0, 1]);
    plot(out_1.x, '--', 'LineWidth', line_width2);
    legends_x{end+1} = ['Config 1: Speed = ' num2str(speeds(i))]; 
    hold on;
    grid on;
    % plot(out_2.x, 'LineWidth', line_width2, 'Marker', '_', 'Color', [1, 0, 0]);
    plot(out_2.x, 'LineWidth', line_width2);
    legends_x{end+1} = ['Config 2 Speed = ' num2str(speeds(i))]; 

    subplot(4,1,2);
    % plot(out_1.dx, 'LineWidth', line_width2, 'Marker', '*', 'Color', [0, 0, 1]);
    plot(out_1.dx, '--', 'LineWidth', line_width2);
    hold on;
    grid on;
    % plot(out_2.dx, 'LineWidth', line_width2, 'Marker', '+', 'Color', [1, 0, 0]);
    plot(out_2.dx, 'LineWidth', line_width2);

    subplot(4,1,3);
    % plot(out_1.th, 'LineWidth', line_width2, 'Marker', '*', 'Color', [0, 0, 1]);
    plot(out_1.th,  '--', 'LineWidth', line_width2);
    hold on;
    grid on;
    % plot(out_2.th, 'LineWidth', line_width2, 'Marker', '+', 'Color', [1, 0, 0]);
    plot(out_2.th, 'LineWidth', line_width2);

    subplot(4,1,4);
    % plot(out_1.dth, 'LineWidth', 1.5, 'Marker', '*', 'Color', [0, 0, 1]);
    plot(out_1.dth,  '--', 'LineWidth', line_width2);
    hold on;
    grid on;
    % plot(out_2.dth, 'LineWidth', 1.5, 'Marker', '+', 'Color', [1, 0, 0]);
    plot(out_2.dth, 'LineWidth', line_width2);


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
legend(legends_x);
subplot(4,1,2);
legend(legends_x);
subplot(4,1,3);
legend(legends_x);
subplot(4,1,4);
legend(legends_x);








