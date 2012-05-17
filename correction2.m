%correction
function [x_new,P] = correction2 (P_est,H,R,z,x_est,j)
K=P_est(j,:)*H'/(H*P_est*H'+R);
x_new= x_est(j)+K*(z-H*x_est);
P=(eye(size(K*H))-K*H)*P_est;%dimensions?