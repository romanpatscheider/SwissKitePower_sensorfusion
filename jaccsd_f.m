function [z,A]=jaccsd_f(fun,fun_imag,x,DCM_ir, DCM_br,t)
% JACCSD Jacobian through complex step differentiation
% [z J] = jaccsd(f,s)
% z = f(x)
% J = f'(x)
%
z=fun(x,DCM_ir,DCM_br,t);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*i;
    A(:,k)=imag(fun_imag(x1,DCM_ir,DCM_br,t))/h;
end