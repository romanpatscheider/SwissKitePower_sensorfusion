function [z,A]=jaccsd_h_euler(fun,x_est,x,t,dis,G,mag)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,s)
% z = f(x)
% J = f'(x)
%
z=fun(x_est,x,t,dis,G,mag);
n=numel(x_est);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x_est;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun(x1,x,t,dis,G,mag))/h;
end