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
syms h1 h2 h3 h4    
% Define the equations
eq1 = ((-a1/A1)*(sqrt(2*g))*(sqrt(h1))) + ((a3/A1)*(sqrt(2*g))*(sqrt(h3)));
eq2 = ((-a2/A2)*(sqrt(2*g))*(sqrt(h2))) + ((a4/A2)*(sqrt(2*g))*(sqrt(h4)));
eq3 = (-a3/A3)*(sqrt(2*g))*(sqrt(h3));
eq4 = (-a4/A4)*(sqrt(2*g))*(sqrt(h4));

% Create a vector of equations
eq_vector = [eq1; eq2; eq3; eq4];

% Compute the Jacobian matrix
Ajc = jacobian(eq_vector, [h1, h2, h3, h4]);

Bc = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];
Hc = [kc 0 0 0; 0 kc 0 0];
Dc = [0 0; 0 0];



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



    for i=1:9
        if i == 1
           Ajc=subs(Ajc,[h1,h2,h3,h4],[x0(1),x0(2),x0(3),x0(4)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=10:19
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,9),xposterior(2,9),xposterior(3,9),xposterior(4,9)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=20:29
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,19),xposterior(2,19),xposterior(3,19),xposterior(4,19)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=30:39
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,29),xposterior(2,29),xposterior(3,29),xposterior(4,29)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=40:49
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,39),xposterior(2,39),xposterior(3,39),xposterior(4,39)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=50:59
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,49),xposterior(2,49),xposterior(3,49),xposterior(4,49)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=60:69
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,59),xposterior(2,59),xposterior(3,59),xposterior(4,59)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=70:79
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,69),xposterior(2,69),xposterior(3,69),xposterior(4,69)]);
          Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=80:89
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,79),xposterior(2,79),xposterior(3,79),xposterior(4,79)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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
   for i=90:10000
        Ajc=subs(Ajc,[h1,h2,h3,h4],[xposterior(1,89),xposterior(2,89),xposterior(3,89),xposterior(4,89)]);
           Aj=double(Ajc);
           % Convert the continuous to discrete with timestep = 0.1 s
           sys = ss(Aj, Bc, Hc, Dc);
           Hd = c2d(sys, 0.1);
           A = Hd.A;
           B = Hd.B;
           C = Hd.C;
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


disp('Thus the final height of the tanks are');
disp(xposterior(:,10000));




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

plot(1:i-1, p_pri_store(1:i-1), 'b', 'LineWidth', 1.5);
hold on;
plot(1:i-1, p_post_store(1:i-1), 'r', 'LineWidth', 1.5);
title('P_{pri} and P_{post} Trace vs Iterations');
xlabel('Iterations');
ylabel('Trace');
legend('P_{pri}', 'P_{post}');

figure;
% Plotting variation of Kalman Gain with iterations

plot(1:i-1, kalman_norm_store(1:i-1, 1), 'b', 'LineWidth', 1.5);
hold on;
plot(1:i-1, kalman_norm_store(1:i-1, 2), 'r', 'LineWidth', 1.5);
title('Variation of Kalman Gain with Iterations');
xlabel('Iterations');
ylabel('Norm');
legend('Kalman Gain 1', 'Kalman Gain 2');

% Plotting variation of Innovation and Residuals with Iterations
figure;

plot(1:i-1, innovation_store(1:i-1), 'b', 'LineWidth', 1.5);
title('Variation of Innovation with Iterations');
xlabel('Iterations');
ylabel('Innovation');

figure;

plot(1:i-1, residual_store(1:i-1), 'r', 'LineWidth', 1.5);
title('Variation of Residuals with Iterations');
xlabel('Iterations');
ylabel('Residuals');

disp('Thus the final height of the tanks are');
disp(xposterior(:,10000));


