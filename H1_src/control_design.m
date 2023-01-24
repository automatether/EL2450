%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
init_tanks;
g = 9.82;
Tau = 1/alpha1*sqrt(2*tank_h10/g);
k_tank = 60*beta*Tau;
gamma_tank = alpha1^2/alpha2^2;
uss = alpha2/beta*sqrt(2*g*tank_init_h2)*100/15; % steady state input
yss = 40; % steady state output

%added parameters
chi=0.5;
zeta=0.8;
omega0=0.2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uppertank = tf(k_tank,[Tau 1]); % Transfer function for upper tank
lowertank = tf(gamma_tank,[Tau*gamma_tank 1]); % Transfer function for upper tank
G = uppertank*lowertank; % Transfer function from input to lower tank level

% Calculate PID parameters
[K, Ti, Td, N]=polePlacePID(chi, omega0, zeta, Tau, gamma_tank, k_tank);
s=tf('s');
F=K*(1+(1/(Ti*s))+(Td*N*s/(s+N)));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.27; % Sampling time
Fd=c2d(F,Ts,'ZOH');
[num, den]=tfdata(Fd,'v');
[A_discretized,B_discretized,C_discretized,D_discretized] = tf2ss(num, den);

% Discretize the continous controller, save it in state space form
% [A_discretized,B_discretized,C_discretized,D_discretized] =

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Discretize the continous state space system, save it in state space form
% [Phi,Gamma,C,D] = 

% Observability and reachability
Wc = 1;
Wo = 1;

% State feedback controller gain
L = 1;
% observer gain
K_obs = 1;
% reference gain
lr = 1;

% augmented system matrices
Aa = 1;
Ba = 1;
