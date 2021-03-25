% to generate TPCMPC code
% 20201215 bit

g = 9.8;
zc = 0.7;
t_lag = 0.01;
ome = sqrt(g / zc);
Ac = [-1 / t_lag 1 / t_lag 0; 0 0 1; 0 0 0];
Bc = [-1 / ome / ome / t_lag; 0; 1];
Cc = [1 0 0];
dc = 0;
CONTROL_T = 0.004;
T_pre = 1.0;
R = 8e-5; %1e-4

[Mu_out, Mc_out, A, B, C] = MPC_controller_writefun(Ac, Bc, Cc, dc, CONTROL_T, T_pre, R, 1, 1, 'TPCMPC', 'Zmp');
