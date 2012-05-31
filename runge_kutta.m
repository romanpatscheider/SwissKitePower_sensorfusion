function x_est=runge_kutta(f,x,t,dt)
% approximates x_est at time t+dt based on x at time t and the equations of
% motion in f using the classical 4th order runge kutta
k1 = dt*f(t,x);
k2 = dt*f(t+dt/2, x+0.5*k1);
k3 = dt*f(t+dt/2, x+0.5*k2); 
k4 = dt*f(t+dt, x+k3);
x_est = x + (k1 + 2*k2 + 2*k3 + k4)/6; %main equation