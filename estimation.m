%estimation
function [x_est,P_est] = estimation (A,P,Q,x)
x_est = A*x;
P_est = A*P*A' + Q;
