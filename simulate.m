% A quick demonstration of using spatial_v2 to simulate a double pendulum.
%
%   generate_dynamics.m must be run first.

clear all; close all; clc;

% Simulation parameters
dt = 5e-3;    % discretization time in seconds
T = 5;        % total simulation time in seconds

% Set initial conditions
q0 = [pi/4; pi/3];  % joint angles
qd0 = [0; 0];        % joint velocities

% Stuff to record for later animation
t_sim = [];
q_sim = [];

% Run the simulation
q = q0;
qd = qd0;

for t=0:dt:T

    % Compute some relevant dynamics quantities
    M = MassMatrix(q);
    Cqd = CoriolisMatrixTimesQd(q,qd);
    g = GravityTorque(q);

    x_task = EndEffector(q);
    J_task = EndEffectorJacobian(q);

    % Specify a control torque
    % (modify to use your control law)
    tau = -0.5*qd;

    % Compute the state at the next timestep using a
    % forward Euler integration scheme
    qdd = Dynamics(q,qd,tau);
    q = q + qd*dt;
    qd = qd + qdd*dt;

    % Update the stored data
    t_sim(end+1) = t;
    q_sim(end+1,:) = q;

end

% Make an animation of the solution
animate_double_pendulum(t_sim, q_sim)



