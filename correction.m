%correction
function [x_new,P] = correction (P_est,H,R,z,x_est)
K=P_est*H'/(H*P_est*H'+R);
x_new= x_est+K*(z-H*x_est);
P=(eye(size(K*H))-K*H)*P_est;