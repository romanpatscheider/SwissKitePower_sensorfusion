clear all;
close all;
clc;


dt=0.001;
m=1;
g=9.81;

%pos=[x_i,y_i,z_i];
%dpos=[vn,ve,vd];
pos=[cos(pi/3),0,-sin(pi/3)]';
dpos=[0,0.01,0]';%dpos=[0,1,0]';
cardan=[0,-pi/6,0]';
dcardan=[0,0,0.02]';%dcardan=[0,0,2]';

DCM_ir=eye(3);






for i=1:10000
    %Conversion from KF variables to model variables
    %%Achtung definition von Phi und Theta und Psi!!!!!
    R=norm(pos);
    Theta=atan2(-pos(3),sqrt(pos(1)^2+pos(2)^2));
    Psi=atan2(pos(2),pos(1));
    dR=0;
    %dR=[cos(Psi)*cos(Theta) sin(Psi)*cos(Theta) -sin(Theta)]*dpos;
    dPsi=[-sin(Psi)/(R*cos(Theta)) cos(Psi)/(R*cos(Theta)) 0]*dpos;
    dTheta=[-cos(Psi)*sin(Theta)/R -sin(Psi)*sin(Theta)/R, -cos(Theta)/R] *dpos;
    
    %DCM_ir=calc_DCM_ir(q);
    DCM_ri=DCM_ir';
    DCM_br=calc_DCM_br(cardan(1),cardan(2),cardan(3));
    DCM_bi=DCM_br*DCM_ri;
    
    
    cardan_mod=[atan2(DCM_bi(2,3),DCM_bi(1,3)) ...
           asin(DCM_bi(3,3)) ...
           atan2(-DCM_bi(3,2),-DCM_bi(3,1))]';
    
    deuler2body=calc_deuler2body(cardan(1),cardan(2),cardan(3));   
    deuler2body_mod=calc_deuler2body(cardan_mod(1),cardan_mod(2),cardan_mod(3));
    
    dcardan_mod=deuler2body_mod\[0,0,-1;0,1,0;1,0,0]*deuler2body*dcardan;
    %end of conversion kf variables to model variables
    %--------------------------------------------------------------------
    %calculate new phys model variables
    x=[cardan_mod(1) Theta Psi R dcardan_mod(1) dTheta dPsi dR]';
    [x_n]=runge_kutta(@pendulum,x,0,dt);
    cardan_mod_n=x_n(1:3); %cardan_mod_n = cardan_mod_n';
    dcardan_mod_n=x_n(5:7); %dcardan_mod_n = dcardan_mod_n';
    R_n=x_n(4);
    dR_n=x_n(8);
    %end of calculation of new model variables
    %--------------------------------------------------------------------
    
    %conversion of new model variables to new kf variables:
   
    DCM_b2i_n=calc_DCM_br(cardan_mod_n(1),cardan_mod_n(2),cardan_mod_n(3));
    DCM_br_n=[0,0,1;0,1,0;-1,0,0]*DCM_b2i_n*DCM_ir;
    
    cardan_n=[atan2(DCM_br_n(2,3),DCM_br_n(3,3)) ...
              asin(-DCM_br_n(1,3)) ...
              atan2(DCM_br_n(1,2),DCM_br_n(1,1))];
   
    deuler2body_n=calc_deuler2body(cardan_n(1),cardan_n(2),cardan_n(3));   
    deuler2body_mod_n=calc_deuler2body(cardan_mod_n(1),cardan_mod_n(2),cardan_mod_n(3));
    
    dcardan_n=deuler2body_n\[0,0,1;0,1,0;-1,0,0]*deuler2body_mod_n*dcardan_mod_n;
    
    pos_n=R_n*[cos(cardan_mod_n(3))*cos(cardan_mod_n(2));...
        sin(cardan_mod_n(3))*cos(cardan_mod_n(2));...
        -sin(cardan_mod_n(2))];
    dpos_n=[cos(cardan_mod_n(2))*cos(cardan_mod_n(3))*dR_n-sin(cardan_mod_n(2))*cos(cardan_mod_n(3))*R_n*dcardan_mod_n(2)-cos(cardan_mod_n(2))*sin(cardan_mod_n(3))*R_n*dcardan_mod_n(3);...
        cos(cardan_mod_n(2))*sin(cardan_mod_n(3))*dR_n-sin(cardan_mod_n(2))*sin(cardan_mod_n(3))*R_n*dcardan_mod_n(2)+cos(cardan_mod_n(2))*cos(cardan_mod_n(3))*R_n*dcardan_mod_n(3);...
        -sin(cardan_mod_n(2))*dR_n-cos(cardan_mod_n(2))*R_n*dcardan_mod_n(2)];


    %end of conversion 
    %--------------------------------------------------------------------
   
    cardan=cardan_n;
    dcardan=dcardan_n;
    pos=pos_n;
    dpos=dpos_n;
    timeseries(:,i)=x_n';
    posseries(:,i)=pos;
end

subplot(121)
plot(timeseries')
legend('phi','theta','psi','r','d/dt phi','d/dt theta','d/dt psi', 'd/dt r')
xlabel('Time [s]');
ylabel('[rad] and [rad/s]');
subplot(122)
plot3(posseries(1,:),posseries(2,:),posseries(3,:))
xlabel('e_x^I');
ylabel('e_y^I');
zlabel('e_z^I');
grid on;
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);