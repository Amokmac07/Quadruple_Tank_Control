% Model parameters
A1 = 28; %(cm^2)
A2 = 32;
A3 = 28;
A4 = 32;
a1 = 0.071; a3 = 0.071; %(cm^2)
a2 = 0.057; a4 = 0.057;
kc = 1; % (V/cm)
g = 981; %(cm/s^2)
gamma1 = 0.7; gamma2 = 0.6; % constants, determined from valve position
k1 = 3.33; k2 = 3.35; %[cm^3/Vs]
% Initial conditions
x0 = [12.4; 12.7; 1.8; 1.4];
% Calculate time constants
T1 = (A1/a1) * sqrt(2 * x0(1) / g);
T2 = (A2/a2) * sqrt(2 * x0(2) / g);
T3 = (A3/a3) * sqrt(2 * x0(3) / g);
T4 = (A4/a4) * sqrt(2 * x0(4) / g);
% Define continuous-time system matrices
Ac = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4];
Bc = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];
Hc = [kc 0 0 0; 0 kc 0 0];
Dc = [0 0; 0 0];
% Discretize the system
sys = ss(Ac, Bc, Hc, Dc);
Hd = c2d(sys, 0.1);
plant = Hd;
% Assign names to I/O variables
plant.InputName = {'Flow1'; 'Flow2'}; 
plant.OutputName = {'Level1'; 'Level2'};
% Design MPC Controller
Ts = 0.1; % Sampling time
p = 15; % Prediction horizon
c = 4; % Control horizon
mpcobj = mpc(plant, Ts, p, c);
% Specify MV constraints
mpcobj.MV = struct('Min', {0; 0}, 'Max', {20; 20}, 'RateMin', {-5; -5}, 'RateMax', {5; 5});
% Define weights on manipulated and controlled variables
mpcobj.Weights = struct('MV', [0 0], 'MVRate', [0.1 0.1], 'OV', [1 1]);
% Set nominal conditions
mpcobj.U = [3; 3]; % Set the initial manipulated variable values
mpcobj.Model.Nominal = struct('X', [12.4 12.7 1.8 1.4], 'Y', [12.4 12.7], 'DX', [0 0]);
% Simulate Closed-Loop Response
Tstop = 30; % Simulation time
Tf = round(Tstop / Ts); % Number of simulation steps
r = [13.4 13.7]; % Reference signal
v = [zeros(Tf/3, 1); ones(2*Tf/3, 1)]; % Disturbance signal
% Run the closed-loop simulation and plot results
sim(mpcobj, Tf, r);