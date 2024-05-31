syms M C tau(q) B m_c m_p theta(t) l g q q_dot q_ddot u f_x x(t) x_dot

q = [x; theta];

M(q) = [m_c+m_p m_p*l*cos(q(2)) ; m_p*l*cos(q(2)) m_p*l^2]; % Inertia matrix
C(q, q_dot) = [0 -m_p*l*q_dot*sin(q) ; 0 0]; % Coriolis forces
tau(q) = [0 ; -m_p*g*l*sin(q)]; % Torque due to gravity
B = [1; 0]; % Actuation
u = f_x;
% 

