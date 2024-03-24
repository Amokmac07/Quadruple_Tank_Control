
% Part2 - Using Particle Filter
clear;
clc;
%Loading the measurement data from the generated data 
load("Tank_data.mat","h1","h2")

% defining the system properties
A1 = 28; %(cm^2)
A2 = 32;
A3 = 28;
A4 = 32;
a1 = 0.071; a3 = 0.071; %(cm^2)
a2 = 0.057; a4 = 0.057;

kc = 0.5; % (V/cm)
g = 981; %(cm/s^2)
gamma1 = 0.7; gamma2 = 0.6; % constants, determined from valve position
k1 = 3.33; k2 = 3.35; %[cm^3/Vs]
v1 = 3; v2 = 3; % in V
x0 = [12.4; 12.7; 1.8; 1.4];

T1 = (A1/a1) * sqrt(2 * x0(1) / g);
T2 = (A2/a2) * sqrt(2 * x0(2) / g);
T3 = (A3/a3) * sqrt(2 * x0(3) / g);
T4 = (A4/a4) * sqrt(2 * x0(4) / g);


% Define matrices for the continuous dynamic system
A = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4];
B = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];
C = [kc 0 0 0; 0 kc 0 0];
Dc = [0 0; 0 0];
n = 4;    %  number of states being measured
N = 15;   % no of particles
P0 = (10^5)*eye(n);
Q = 50*eye(n);
R = 2*eye(2);
L = chol(P0);

for k = 1: length(h1)
    x(:,:,1) = (x0*ones(1,N))';
    x(:,:,k) = x(:,:,k) + randn(N,n)*L;   % for roughening of the prior  
    x1posterior(:,1,k) = x(:,1,k);
    x2posterior(:,1,k) = x(:,2,k);
    x3posterior(:,1,k) = x(:,3,k);
    x4posterior(:,1,k) = x(:,4,k);

     % addition of processn noise
    w(:,:,k) = chol(Q)*randn(n,N);
    w1(1,:,k) = w(1,:,k);
    w2(1,:,k) = w(2,:,k);
    w3(1,:,k) = w(3,:,k);
    w4(1,:,k) = w(4,:,k);

    x1posterior(:,1,k) = x1posterior(:,1,k) + w1(1,:,k)';
    x2posterior(:,1,k) = x2posterior(:,1,k) + w2(1,:,k)';
    x3posterior(:,1,k) = x3posterior(:,1,k) + w3(1,:,k)';
    x4posterior(:,1,k) = x4posterior(:,1,k) + w4(1,:,k)';

    % Prediction  of System 
    for j = 1:N
        x1prior(1,j,k) = -a1/A1*sqrt(2*g*x1posterior(j,1,k)) + a3/A1*sqrt(2*g*x3posterior(j,1,k)) + (gamma1*k1*v1)/A1 + w1(1,j,k);
        x2prior(1,j,k) = -a2/A2*sqrt(2*g*x2posterior(j,1,k)) + a4/A2*sqrt(2*g*x4posterior(j,1,k)) + (gamma2*k1*v2)/A2 + w2(1,j,k);
        x3prior(1,j,k) = -a3/A3*sqrt(2*g*x3posterior(j,1,k)) + (1 - gamma2)*k2*v2/A3  +  w3(1,j,k);
        x4prior(1,j,k) = -a4/A4*sqrt(2*g*x4posterior(j,1,k)) + (1 - gamma1)*k1*v1/A4  +  w4(1,j,k);

    end
    x1prior(1,:,k) = abs(x1prior(1,:,k));
    x2prior(1,:,k) = abs(x2prior(1,:,k));
    x3prior(1,:,k) = abs(x3prior(1,:,k));
    x4prior(1,:,k) = abs(x4prior(1,:,k));
    x_prior(:,:,k) = [x1prior(1,:,k); x2prior(1,:,k); x3prior(1,:,k); x4prior(1,:,k)];
    
    % Importance Weights
    z_true(:,:,k) = [h1(k); h2(k)]*ones(1,N);
    z_estimated(:,:,k) = C*x_prior(:,:,k);
    e(:,:,k) = z_true(:,:,k) - z_estimated(:,:,k);
    for j = 1:N
        q(1,j,k) = exp(-0.5*(e(:,j,k)'*inv(R)*e(:,j,k)));
    %end
    %for i = 1:N
        wt(1,j,k) = q(1,j,k)/sum(q(1,:,k));  %Normalizing the weights
    end
   
    % Resampling
    M(k) = length(wt(1,:,k));
    P = zeros(1,N);
    P(1,:,k) = cumsum(wt(1,:,k));
    index(1,:,k) = zeros(1, N);
    T(1,:,k) = linspace(0,1-1/N,N) + rand/N;
        i = 1; j = 1;
    while(i<=N & j<=M)
        while P(1,j,k) < T(1,i,k)
            j = j + 1;
        end
        index(1,i,k) = j;
        x1posterior(i,1,k) = x1prior(1,j,k);
        x2posterior(i,1,k) = x2prior(1,j,k);
        x3posterior(i,1,k) = x3prior(1,j,k);
        x4posterior(i,1,k) = x4prior(1,j,k);
        i = i + 1;
    end
    

    x1_est(:,k) = mean(x1posterior(:,1,k));
    x2_est(:,k) = mean(x2posterior(:,1,k));
    x3_est(:,k) = mean(x3posterior(:,1,k));
    x4_est(:,k) = mean(x4posterior(:,1,k));

    x(:,:,k+1) = [x1posterior(:,1,k) x2posterior(:,1,k) x3posterior(:,1,k) x4posterior(:,1,k)];
end



figure;
hold on;
for k = 1:length(h1)
    plot(k, x1_est(:,k), 'r.', k, x2_est(:,k), 'b.', k, x3_est(:,k), 'g.', k, x4_est(:,k), 'm.');
end
hold off;
xlabel('Time / iterations');
ylabel('Estimated Values');
title('Estimated values of x1, x2, x3, and x4 with time');
legend('x1', 'x2', 'x3', 'x4');
