m_c = 5; % Mass of the cart
m_p = 1; % Mass of the pendulum
l = 2; % Length of the pendulum
g = -10;

% https://underactuated.csail.mit.edu/acrobot.html#cart_pole %

% y = [x, dx/dt, theta, dtheta/dt]
function dy = dynamics(y, m_c, m_p, l, g, u)

D = (m_c + m_p*(sin(y(3))^2));

dy = zeros(4,1);
dy(1, 1) = y(2);
dy(2, 1) = (1/D) * (u + m_p * sin(y(3)) * (l * y(4)^2 + g * cos(y(3))));
dy(3, 1) = y(4);
dy(4, 1) = (1/D*l) * (-u * cos(y(3)) - m_p * l * y(4)^2 * 0.5*sin(2*y(3)) - (m_c+m_p)*g*sin(y(3)));

end

tspan = 0:.1:10;
y0 = [0; 0; pi; .5];
[t, y] = ode45(@(t, y)dynamics(y, m_c, m_p, l, g, 0), tspan, y0)
plot(t, y)
legend('x', 'dx/dt', 'theta', 'dtheta/dt')
