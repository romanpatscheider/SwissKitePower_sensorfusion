function [x_est]=f_imag(x,DCM_ir,DCM_br,t)
%%
%------------------------
% the physical model and the measurement-state realation are defined. the
% jacobian matrix is calculated
%------------------------

% x_i=(lat-lat0)*Rl;
% y_i=(long-long0)*Rp;
% z_i=-alt;
% pos=[x_i,y_i,z_i];
% R=norm(pos);
% rotvec=vrrotvec(pos./R,[0,0,-1]);
% alpha=rotvec(4);

pos =x(1:3);
dpos=x(4:6);
cardan=x(7:9);
dcardan=x(10:12);
%Conversion from KF variables to model variables
    %%Achtung definition von Phi und Theta und Psii!!!!!
    %R=sqrt(pos(1)^2+pos(2)^2+pos(3)^2);
    R=norm(pos);
    y1=-pos(3);
    x1=sqrt(pos(1)^2+pos(2)^2);
    Theta=2*atan( (sqrt(x1^2+y1^2)-x1)/y1);
    y2=-pos(3);
    x2=sqrt(pos(1)^2+pos(2)^2);
    Psii=2*atan( (sqrt(x2^2+y2^2)-x2)/y2);
    
    
    %Theta=atan2(-pos(3),sqrt(pos(1)^2+pos(2)^2));
    %Psii=atan2(pos(2),pos(1));
    dR=0;
    %dR=[cos(Psii)*cos(Theta) sin(Psii)*cos(Theta) -sin(Theta)]*dpos;
    dPsii=[-sin(Psii)/(R*cos(Theta)) cos(Psii)/(R*cos(Theta)) 0]*dpos;
    dTheta=[-cos(Psii)*sin(Theta)/R -sin(Psii)*sin(Theta)/R, -cos(Theta)/R] *dpos;
    
%     DCM_ir=calc_DCM_ir(q);

    DCM_ri=DCM_ir';
%     DCM_br=calc_DCM_br(cardan(1),cardan(2),cardan(3));
    DCM_bi=DCM_br*DCM_ri;
    
    y4=DCM_bi(2,3);
    x4=DCM_bi(1,3);
    y5=-DCM_bi(3,2);
    x5=-DCM_bi(3,1);
    
    cardan_mod=[2*atan( (sqrt(x4^2+y4^2)-x4)/y4) ...
           asin(DCM_bi(3,3)) ...
           2*atan( (sqrt(x5^2+y5^2)-x5)/y5)]';
%     cardan_mod=[atan2(DCM_bi(2,3),DCM_bi(1,3)) ...
%            asin(DCM_bi(3,3)) ...
%            atan2(-DCM_bi(3,2),-DCM_bi(3,1))]';
    
    deuler2body=calc_deuler2body(cardan(1),cardan(2),cardan(3));   
    body2deuler_mod=calc_body2deuler(cardan_mod(1),cardan_mod(2),cardan_mod(3));
    
    dcardan_mod=body2deuler_mod*[0,0,-1;0,1,0;1,0,0]*deuler2body*dcardan;
    %end of conversion kf variables to model variables
    %--------------------------------------------------------------------
    %calculate new phys model variables
    state=[cardan_mod(1) Theta Psii R dcardan_mod(1) dTheta dPsii dR]';
    [state_n]=runge_kutta(@pendulum,state,0,t);
    cardan_mod_n=state_n(1:3); %cardan_mod_n = cardan_mod_n';
    dcardan_mod_n=state_n(5:7); %dcardan_mod_n = dcardan_mod_n';
    R_n=state_n(4);
    dR_n=state_n(8);
    %end of calculation of new model variables
    %--------------------------------------------------------------------
    
    %conversion of new model variables to new kf variables:
   
    DCM_b2i_n=calc_DCM_br(cardan_mod_n(1),cardan_mod_n(2),cardan_mod_n(3));
    DCM_br_n=[0,0,1;0,1,0;-1,0,0]*DCM_b2i_n*DCM_ir;
    
      
    y6=DCM_br_n(2,3);
    x6=DCM_br_n(3,3);
    y7=DCM_br_n(1,2);
    x7=DCM_br_n(1,1);
    
    
     cardan_n=[2*atan( (sqrt(x6^2+y6^2)-x6)/y6) ...
              asin(-DCM_br_n(1,3)) ...
              2*atan( (sqrt(x7^2+y7^2)-x7)/y7)]';
   
    body2deuler_n=calc_body2deuler(cardan_n(1),cardan_n(2),cardan_n(3));   
    deuler2body_mod_n=calc_deuler2body(cardan_mod_n(1),cardan_mod_n(2),cardan_mod_n(3));
    
    dcardan_n=body2deuler_n*[0,0,1;0,1,0;-1,0,0]*deuler2body_mod_n*dcardan_mod_n;
    pos_n=R_n*[cos(cardan_mod_n(3))*cos(cardan_mod_n(2));...
        sin(cardan_mod_n(3))*cos(cardan_mod_n(2));...
        -sin(cardan_mod_n(2))];
    dpos_n=[cos(cardan_mod_n(2))*cos(cardan_mod_n(3))*dR_n-sin(cardan_mod_n(2))*cos(cardan_mod_n(3))*R_n*dcardan_mod_n(2)-cos(cardan_mod_n(2))*sin(cardan_mod_n(3))*R_n*dcardan_mod_n(3);...
        cos(cardan_mod_n(2))*sin(cardan_mod_n(3))*dR_n-sin(cardan_mod_n(2))*sin(cardan_mod_n(3))*R_n*dcardan_mod_n(2)+cos(cardan_mod_n(2))*cos(cardan_mod_n(3))*R_n*dcardan_mod_n(3);...
        -sin(cardan_mod_n(2))*dR_n-cos(cardan_mod_n(2))*R_n*dcardan_mod_n(2)];


    %end of conversion 
    %--------------------------------------------------------------------

x_est=[pos_n;dpos_n;cardan_n;dcardan_n;x(13:21)];