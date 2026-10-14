function dxdt = rk4(x, f, u, dt)
k1 = f(x, u);
k2 = f(x + 0.5 * dt * k1, u);
k3 = f(x + 0.5 * dt * k2, u);
k4 = f(x + dt * k3, u);
dxdt = (k1 + 2*k2 + 2*k3 + k4) / 6;
end
