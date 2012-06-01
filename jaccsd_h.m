function [z,A]=jaccsd_h(fun,x,x_old,DCM_bi,t,dis)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,s)
% z = f(x)
% J = f'(x)
%
z=fun(x,x_old,DCM_bi,t,dis);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun(x1,x_old,DCM_bi,t,dis))/h;
end