%% Cart-Pole System with LQR Control Example
% Clear workspace, command window, and close figures
clear; clc; close all;

% System Parameters
m_c = 4;         % Mass of the cart
m_p = 1.5;       % Mass of the pendulum
l = 1.5;         % Length of the pendulum
g = -10;         % Gravitational acceleration (negative as defined)

% Linearized system matrices (around the equilibrium)
A = [0 1 0 0;
     0 -1/m_c -m_p*g/m_c 0;
     0 0 0 1;
     0 -1/(m_c*l) -1*(m_c+m_p)*g/(m_c*l) 0];
B = [0; 1/m_c; 0; 1/(m_c*l)];

% LQR weight matrices
Q = diag([1, 1, 10, 100]);
R = 0.0001;

% Compute the LQR gain matrix K
K = lqr(A, B, Q, R);

% Desired state: [x, dx/dt, theta, dtheta/dt]
y_desired = [1; 0; pi; 0];

% Define the dynamics function with LQR control using a function handle
dynamics = @(t, y) cartPoleDynamics(y, m_c, m_p, l, g, K, y_desired);

% Simulation settings
tspan = [0 10];          % Time interval for simulation
y0 = [0; 0; pi/2; 0.2];    % Initial state

% Simulate the system using ode45
[t, y] = ode45(dynamics, tspan, y0);

% Plot the simulation results
figure;
subplot(4,1,1);
plot(t, y(:,1), 'LineWidth', 2);
ylabel('x');
title('Cart-Pole System States with LQR Control');

subplot(4,1,2);
plot(t, y(:,2), 'LineWidth', 2);
ylabel('dx/dt');

subplot(4,1,3);
plot(t, y(:,3), 'LineWidth', 2);
ylabel('theta');

subplot(4,1,4);
plot(t, y(:,4), 'LineWidth', 2);
ylabel('dtheta/dt');
xlabel('Time (s)');

% Dynamics function definition for the cart-pole system
function dy = cartPoleDynamics(y, m_c, m_p, l, g, K, y_desired)
    % Compute control input using LQR: u = -K*(y - y_desired)
    u = -K * (y - y_desired);
    
    % Denominate term used in the dynamics
    D = m_c + m_p * (sin(y(3)))^2;
    
    % Allocate derivative vector
    dy = zeros(4,1);
    
    % Equations of motion:
    dy(1) = y(2);
    dy(2) = (1/D) * (u + m_p*sin(y(3))*(l*y(4)^2 + g*cos(y(3))));
    dy(3) = y(4);
    dy(4) = -(1/(D*l)) * (-u*cos(y(3)) - m_p*l*y(4)^2*cos(y(3))*sin(y(3)) - (m_c+m_p)*g*sin(y(3)));
end
