clc;
clear ;
%% System Parameters
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
x0=[12.4;12.7;1.8;1.4];
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
Ap=A;
Bp=B;
Cp=C;
Np=40;
Nc=20;
[Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e]= mpcgain(Ap,Bp,Cp,Nc,Np);
[n,n_in]=size(B_e);
xm=[12.4;12.7;1.8;1.4];
Xf=zeros(n,1);
N_sim=200;
r=[13.4;13.7];
u=[3;3]; % u(k-1) =0
y=0;
u1=zeros(2,N_sim);
y1=zeros(2,N_sim);
for kk=1:N_sim;
 DeltaU=inv(Phi_Phi+1*eye(size(Phi_Phi)))*(Phi_R*r-Phi_F*Xf);
 deltau=DeltaU(1:2,1);
 u=u+deltau;
 u1(:,kk)=u;
 y1(:,kk)=y;
 xm_old=xm; 
 xm=Ap*xm+Bp*u;
 y=Cp*xm;
 Xf1(:,kk)=Xf;
 Xf=[xm-xm_old;y];
end
k=1:(N_sim);
figure
plot(k,(y1/2),'LineWidth',2)
xlabel('Sampling Instant')
ylabel('Output')
legend('Output h1','Output h2')
figure;
plot(k,u1,'LineWidth',2)
xlabel('Sampling Instant')
ylabel('Input')
legend('Control u1','Control u2')
%%
L=inv(Phi_Phi+0.1*eye(size(Phi_Phi)))*Phi_F;
Kmpc=L(1:2,:);
% Closed-loop system matrix
A_closed_loop = A_e - B_e* Kmpc;
% Compute eigenvalues
closed_loop_eigenvalues = eig(A_closed_loop);
% Display the eigenvalues
disp('Closed-loop eigenvalues initially :');
disp(closed_loop_eigenvalues);
%%
z=xm/2; % the final steady state values
%to define the clodes loop stability at the final value we need
% to find the Kmpc using the new Augmented matrices
T1 = (A1/a1) * sqrt(2 * z(1) / g);
T2 = (A2/a2) * sqrt(2 * z(2) / g);
T3 = (A3/a3) * sqrt(2 * z(3) / g);
T4 = (A4/a4) * sqrt(2 * z(4) / g);
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
Ap=A;
Bp=B;
Cp=C;
Np=40;
Nc=20;
[Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e]= mpcgain(Ap,Bp,Cp,Nc,Np);
L=inv(Phi_Phi+0.1*eye(size(Phi_Phi)))*Phi_F;
Kmpc=L(1:2,:);
% Closed-loop system matrix
A_closed_loop = A_e - B_e* Kmpc;
% Compute eigenvalues
closed_loop_eigenvalues = eig(A_closed_loop);
% Display the eigenvalues
disp('Closed-loop eigenvalues at steady state :');
disp(closed_loop_eigenvalues);