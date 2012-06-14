function [x_est]=f5(x,t)
%%
%------------------------
% the physical model is defined. 
%------------------------


    x(4)=0.8465;
    x(8)=0;
    %calculate new phys model variables
    x_est=runge_kutta(@pendulum,x,0,t);
    x_est(4)=0.8465;
    x_est(8)=0;
    
   