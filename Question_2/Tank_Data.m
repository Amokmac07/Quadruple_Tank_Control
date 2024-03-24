% Defining the constants
a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057;
A1 = 28; A2 = 32; A3 = 28; A4 = 32;
g = 981;
g1 = 0.7;
g2 = 0.6;
k1 = 3.33; k2 = 3.35;
kc = 0.5;

% Define the function representing the system of differential equations
tank_system = @(t, h) [
    ((-1) * a1 / A1) * (sqrt(2 * g)) * (sqrt(h(1))) + ((a3 / A1) * (sqrt(2 * g)) * (sqrt(h(3))) + (g1 * k1 * 3.0 / A1));
    ((-1) * a2 / A2) * (sqrt(2 * g)) * (sqrt(h(2))) + ((a4 / A2) * (sqrt(2 * g)) * (sqrt(h(4))) + (g2 * k2 * 3.0 / A2));
    ((-1) * a3 / A3) * (sqrt(2 * g)) * (sqrt(h(3))) + ((1 - g2) * k2 * 3.0 / A3);
    ((-1) * a4 / A4) * (sqrt(2 * g)) * (sqrt(h(4))) + ((1 - g1) * k1 * 3.0 / A4)
    ];

% Initial conditions and time points
h0 = [12.4; 12.7; 1.8; 1.4]; % Initial heights
t = linspace(0, 10000, 10000); % Time points

% Solve the ODE
[t, sol] = ode45(tank_system, t, h0);

% Extract the data
h1 = sol(:, 1);
h2 = sol(:, 2);
h3 = sol(:, 3);
h4 = sol(:, 4);

% Save the data 
save('Tank_data.mat', 'h1', 'h2', 'h3', 'h4');
