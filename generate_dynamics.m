%
% This script uses spatial_v2 to derive the equations of motion of a double pendulum.
% These equations are then saved as Matlab functions, where they can be used by
% future scripts. 
%

clear all; close all; clc;

disp("Defining Double Pendulum Model")

% Define state variables
syms theta1 theta2 real
syms theta1_dot theta2_dot real
q_sym =[theta1 theta2]';
qd_sym = [theta1_dot theta2_dot]';
x_sym = [q_sym; qd_sym];

% Define control variables
syms tau1 tau2 real
tau_sym = [tau1 tau2]';
u_sym = tau_sym;

double_pendulum_model = autoTree(2);  % produces an unbranched kinematic tree with 2 bodies 
double_pendulum_model.gravity = [0;-9.81;0];   % point gravity in the -y direction

disp("  Deriving Equation of Motion")
% Derive equations of motion with the Articulated Body Algorithm (ABA).
% The ABA gives us the joint accelerations qdd that result from applying
% torques tau when the current joint angles are q and qd.
qdd_sym = FDab(double_pendulum_model, q_sym, qd_sym, tau_sym);
f_sym = [qd_sym; qdd_sym];
f_sym = simplify(f_sym);

disp("  Saving Dynamics.m")
% Generate and save the file Dynamics.m
comment_string = ...
[" This function describes the equations of motion x' = f(x,u) of a double pendulum," ...
 " where x = [q,qd] = [theta1,theta2,theta1',theta2'] and u = [tau1,tau2]." ...
];
f_func = matlabFunction(f_sym, 'file', 'Dynamics','vars',{x_sym,u_sym}, 'Comments', comment_string);

disp("  Deriving End Effector Position")
% Symbolic expression for end-effector position
xe_sym = cos(theta1) + cos(theta1+theta2);
ye_sym = sin(theta1) + sin(theta1+theta2);
p_sym = [xe_sym;ye_sym];

disp("  Saving EndEffector.m")
% Generate and save EndEffector.m
comment_string = ...
[" This function describes the end effector position y = p(q) of a double pendulum," ...
 " where q = [theta1,theta2]." ...
];
p_func = matlabFunction(p_sym, 'file', 'EndEffector','vars', {q_sym}, 'Comments', comment_string);


disp("  Deriving End Effector Jacobian")
% Symbolic expression for end-effector jacobian
J_sym = jacobian(p_sym,[theta1;theta2]);

disp("  Saving EndEffectorJacobian.m")
% Generate and save EndEffectorJacobian.m
comment_string = ...
[" This function returns the end effector jacobian J(q) of the double pendulum," ...
 " as a function of q = [theta1,theta2]." ...
];
J_func = matlabFunction(J_sym, 'file', 'EndEffectorJacobian', 'vars', {q_sym}, 'Comments', comment_string);

disp("  Deriving Mass Matrix, Corriolis Matrix, and Gravitation terms")
% Symbolic expression for mass matrix and correolis + gravitational terms
[M_sym Cqd_plus_g_sym] = HandC(double_pendulum_model, q_sym, qd_sym);

% Symbolic expression for g(q), the torques due to gravity
ret = EnerMo(double_pendulum_model, q_sym, qd_sym);
U_sym = ret.PE;
g_sym = jacobian(U_sym,q_sym)';

% Symbolic expression for the corriolis term C(q)*qd alone
Cqd_sym = Cqd_plus_g_sym - g_sym;

disp("  Saving MassMatrix.m")
% Generate and save MassMatrix.m
comment_string = ...
[" This function returns the mass matrix H(q) of the double pendulum," ...
 " as a function of q = [theta1,theta2]." ...
];
M_func = matlabFunction(M_sym, 'file', 'MassMatrix', 'vars', {q_sym}, 'Comments', comment_string);

disp("  Saving CoriolisMatrixTimesQd.m")
% Generate and save CoriolisMatrixTimesQd.m"
[" This function returns the Coriolis matrix times qd C(q)*qd for the double pendulum," ...
 " as a function of q = [theta1,theta2] and qd = [theta1', theta2']." ...
];
Cqd_func = matlabFunction(Cqd_sym, 'file', 'CoriolisMatrixTimesQd', 'vars', {q_sym, qd_sym}, 'Comments', comment_string);

disp("  Saving GravityTorque.m")
% Generate and save GravityTorque.m
comment_string = ...
[" This function returns the gravitational torque g(q) of the double pendulum," ...
 " as a function of q = [theta1,theta2]." ...
];
g_func = matlabFunction(g_sym, 'file', 'GravityTorque', 'vars', {q_sym}, 'Comments', comment_string);
