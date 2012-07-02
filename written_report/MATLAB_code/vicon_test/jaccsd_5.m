function [z,A]=jaccsd_5(fun,x,t)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,s)
% z = f(x)
% J = f'(x)
%
z=fun(x,t);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun(x1,t))/h;
end