% Define the file name and range
filename = 'tank_data.xlsx';
xlRange = 'B2:E10002';

% Import the data from the specified Excel file and range
data = xlsread(filename, xlRange);

% Assign the columns to respective variables
h1 = data(:, 1);
h2 = data(:, 2);
h3 = data(:, 3);
h4 = data(:, 4);

% Combine the variables into a single matrix
z_true = [h1, h2, h3, h4];


A1 = 28; %(cm^2)
A2 = 32;
A3 = 28;
A4 = 32;
a1 = 0.071; a3 = 0.071; %(cm^2)
a2 = 0.057; a4 = 0.057;
kc = 0.5; % (V/cm) 
g = 981; %(cm/s^2)
gamma1 = 0.7; gamma2 = 0.6;      % constants, determined from valve position
k1 = 3.33; k2 = 3.35; %[cm^3/Vs]

% Define the initial values for the input and height 
u0 = [3; 3]; % (V)
x0 = [12.4; 12.7; 1.8; 1.4];
P0 = 10^5 * eye(4);


T1 = (A1/a1) * sqrt(2 * x0(1) / g);
T2 = (A2/a2) * sqrt(2 * x0(2) / g);
T3 = (A3/a3) * sqrt(2 * x0(3) / g); 
T4 = (A4/a4) * sqrt(2 * x0(4) / g);

% Define matrices for the continuous dynamic system
Ac = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4];
Bc = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];
Hc = [kc 0 0 0; 0 kc 0 0]; 
Dc = [0 0; 0 0];

% Convert the continuous to discrete with timestep = 0.1 s
sys = ss(Ac, Bc, Hc, Dc);
Hd = c2d(sys, 0.1);
A = Hd.A;
B = Hd.B;
C = Hd.C;



% Tuning parameters Q (process noise) and R (measurement noise)
Q = 10 * eye(4);
R = 2 * eye(2);
xprior = zeros(4,10000);
xposterior = zeros(4,10000);
Pprior=zeros(4);
Pposterior=zeros(4);
p_pri_store = zeros(1,10000);
p_post_store= zeros(1,10000);
kalman_norm_store = zeros(10000,2);

innovation_store = zeros(1,10000);
residual_store = zeros(1,10000);


for i = 1:length(z_true)
    
    if i == 1
        xprior(:, i) = A * x0 + B * u0;
        Pprior = A * P0 * A' + Q;
        K = (Pprior * C') / (C * Pprior * C' + R);
        xposterior(:, i) = xprior(:, i) + K * (z_true(i,1:2)' - C * xprior(:, i));
        Pposterior = (eye(4) - K * C) * Pprior;
        Residue=z_true(i,1:2)' - C * xprior(:, i);
        Innovation=z_true(i,1:2)' - C * xposterior(:, i);
        
        p_pri_store(i) = trace(Pprior);
        p_post_store(i) = trace(Pposterior);
        kalman_norm_store(i, 1) = norm(K(:, 1));
        kalman_norm_store(i, 2) = norm(K(:, 2));
        innovation_store(i) = norm(Innovation);
        residual_store(i) = norm(Residue);
    else
        xprior(:, i) = A * xposterior(:, i - 1) + B * u0;

        Pprior = A * Pposterior * A' + Q;
        K = (Pprior * C') / (C * Pprior * C' + R);
        xposterior(:, i) = xprior(:, i) + K * (z_true(i,1:2)' - C * xprior(:, i));
        Pposterior = (eye(4) - K * C) * Pprior;
        Residue=z_true(i,1:2)' - C * xprior(:, i);
        Innovation=z_true(i,1:2)' - C * xposterior(:, i);
        p_pri_store(i) = trace(Pprior);
        p_post_store(i) = trace(Pposterior);
        kalman_norm_store(i, 1) = norm(K(:, 1));
        kalman_norm_store(i, 2) = norm(K(:, 2));
        innovation_store(i) = norm(Innovation);
        residual_store(i) = norm(Residue);
    end
end





% Plotting xprior and xposterior
figure;
subplot(2, 1, 1);
plot(1:length(z_true), xprior(1, :), 'b', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xprior(2, :), 'r', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xprior(3, :), 'g', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xprior(4, :), 'm', 'LineWidth', 1.5);
title('xprior vs Iterations');
xlabel('Iterations');
ylabel('Value');
legend('x1', 'x2', 'x3', 'x4');

subplot(2, 1, 2);
plot(1:length(z_true), xposterior(1, :), 'b', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xposterior(2, :), 'r', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xposterior(3, :), 'g', 'LineWidth', 1.5);
hold on;
plot(1:length(z_true), xposterior(4, :), 'm', 'LineWidth', 1.5);
title('xposterior vs Iterations');
xlabel('Iterations');
ylabel('Value');
legend('x1', 'x2', 'x3', 'x4');

figure;
subplot(1, 2, 1);
plot(1:i-1, p_pri_store(1:i-1), 'b', 'LineWidth', 1.5);
hold on;
plot(1:i-1, p_post_store(1:i-1), 'r', 'LineWidth', 1.5);
title('P_{pri} and P_{post} Trace vs Iterations');
xlabel('Iterations');
ylabel('Trace');
legend('P_{pri}', 'P_{post}');

% Plotting variation of Kalman Gain with iterations
subplot(1, 2, 2);
plot(1:i-1, kalman_norm_store(1:i-1, 1), 'b', 'LineWidth', 1.5);
hold on;
plot(1:i-1, kalman_norm_store(1:i-1, 2), 'r', 'LineWidth', 1.5);
title('Variation of Kalman Gain with Iterations');
xlabel('Iterations');
ylabel('Norm');
legend('Kalman Gain 1', 'Kalman Gain 2');

% Plotting variation of Innovation and Residuals with Iterations
figure;
subplot(2, 1, 1);
plot(1:i-1, innovation_store(1:i-1), 'b', 'LineWidth', 1.5);
title('Variation of Innovation with Iterations');
xlabel('Iterations');
ylabel('Innovation');

subplot(2, 1, 2);
plot(1:i-1, residual_store(1:i-1), 'r', 'LineWidth', 1.5);
title('Variation of Residuals with Iterations');
xlabel('Iterations');
ylabel('Residuals');

disp("Thus the final height of the tanks are");
disp(xposterior(:,10000));



