function [state,state_est] = state_estimation_3(M,meas_time,counter)

%------------------------
% constants are defined
%------------------------
t=0.01; % [s] time interval between two measurements
G= [0;0;9.85]; % [m/s^2] gravitation in zurich
dis=[0;0;0];%displacementvector of the box
MAG1=0.2145;% [Gauss] magnetic field in zurich
MAG2=0.0060;
MAG3=0.4268;
Rl= geocradius(47+24/60); %from zurich
Rp= Rl*cos(8+32/60); %from zurich
lat0=47+24/60;
long0=8+32/60;
% noise form Xsens datasheet
NOISE_ACC_b=[0.002;0.002;0.002];%noise accelerometer
NOISE_GYRO_b=[0.05;0.05;0.05];%noise gyro
NOISE_MAG_b=[0.5;0.5;0.5];%check data sheet again!!!!!!!!!!!!!!

NOISE_GPS_POS=0.005;% Noise in position of the GPS
NOISE_GPS_VEL=0.005;%Noise in velocity of the GPS



%------------------------
% variables with initial values are defined
%------------------------
x = [-0.08372,-0.2276,-0.8123,-0.5,-0.2,0,0,-0.2901,-1.9233, 0, 0.0171, -2.2197,0,0,0,0,0,0,0,0,0]';
P = zeros(size(x,1),size(x,1));
q = [0,0,1,0]';

syms q1 q2 q3 q4;
syms vn_old ve_old vd_old;
syms lat long alt vn ve vd phi thet pssi d_phi d_thet d_psi bax bay baz bgx bgy bgz bmx bmy bmz vn_old ve_old vd_old phi_old thet_old psi_old d_phi_old d_thet_old d_psi_old real;
% syms lat_e long_e alt_e vn_e ve_e vd_e phi_e thet_e psi_e d_phi_e d_thet_e d_psi_e bax_e bay_e baz_e bgx_e bgy_e bgz_e bmx_e bmy_e bmz_e real;
vf=[lat, long, alt, vn, ve, vd, phi, thet, pssi, d_phi, d_thet, d_psi, bax, bay, baz, bgx, bgy, bgz, bmx, bmy, bmz];
quat= [q1, q2, q3, q4];
% vh=[lat_e, long_e, alt_e, vn_e, ve_e, vd_e, phi_e, thet_e, psi_e, d_phi_e, d_thet_e, d_psi_e, bax_e, bay_e, baz_e, bgx_e, bgy_e, bgz_e, bmx_e, bmy_e, bmz_e];



%------------------------
% direct cosine matrice gets calculated
%------------------------
DCM_ir=calc_DCM_ir(quat);
DCM_br=calc_DCM_br(phi,thet,pssi);
DCM_bi=DCM_br*DCM_ir';

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
pos =[lat,long,alt]';
dpos= [vn, ve, vd]';
cardan=[phi, thet, pssi]';
dcardan=[d_phi,d_thet,d_psi]';
%Conversion from KF variables to model variables
    %%Achtung definition von Phi und Theta und Psii!!!!!
    R=sqrt(pos(1)^2+pos(2)^2+pos(3)^2);
    %R=norm(pos);
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
    
    %DCM_ir=calc_DCM_ir(q);
    DCM_ri=DCM_ir';
    %DCM_br=calc_DCM_br(cardan(1),cardan(2),cardan(3));
%     DCM_bi=DCM_br*DCM_ri;
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
    
%     cardan_n=[atan2(DCM_br_n(2,3),DCM_br_n(3,3)) ...
%               asin(-DCM_br_n(1,3)) ...
%               atan2(DCM_br_n(1,2),DCM_br_n(1,1))];
   
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

f=[pos_n;dpos_n;cardan_n;dcardan_n;bax;bay;baz;bgx;bgy;bgz;bmx;bmy;bmz];
%%

%definition of h
w=[d_phi;0;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[0;d_thet;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0;0;d_psi];
w_old=[d_phi_old;0;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[0;d_thet_old;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[cos(thet_old), 0, -sin(thet_old);0,1,0;sin(thet_old),0,cos(thet_old)]*[0;0;d_psi_old];
vg=(cross(w',dis'))';
vg_old=(cross(w_old',dis'))';
v=[vn;ve;vd];
v_old=[vn_old;ve_old;vd_old];

h1=[lat;long;alt] + DCM_bi'*dis; %pos
h2=v + DCM_bi'*vg; %vel
h3=DCM_bi*(((v+DCM_bi'*vg)-(v_old+DCM_bi'*vg_old))./t-G);%acc
h4=w;%gyr
h5=DCM_bi*[MAG1;MAG2;MAG3];%magnetometer
h6=0;%pressure sensor
h=[h1;h2;h3;h4;h5;h6];

%%
%tmp_A= jacobian(f,vf);
A=eye(21);
tmp_H= jacobian(h,vf);

%------------------------
% Q and R are calculated
%------------------------
NOISE_ACC_i=DCM_ir*(NOISE_ACC_b.^2);
NOISE_VEL_i=(NOISE_ACC_i.^2)*t;
NOISE_POS_i=(NOISE_VEL_i.^2)*t;
NOISE_GYRO_i=DCM_ir*(NOISE_GYRO_b.^2);
% q_diag=[NOISE_POS_i',NOISE_VEL_i',NOISE_GYRO_i',0,0,0,0,0,0,0,0,0];
q_diag=[0.001,0.001,0.001,0.001,0.001,0.001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0,0,0,0,0,0,0,0,0];
Q=diag(q_diag);


r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b.^2)',(NOISE_MAG_b.^2)'];
R=diag(r);


%%
%------------------------
% for every measurement the Extended Kalman Filter is used for state
% estimation
%------------------------
totalTime=meas_time(1);
i=1;
while (i<size(M,2))
    totalTime=totalTime+t;
    % values for the symbols for the x state
    lat = x(1);
    long= x(2);
    alt = x(3);
    vn  = x(4);
    ve  = x(5);
    vd  = x(6);
    phi = x(7);
    thet = x(8);
    pssi = x(9);
    d_phi  = x(10);
    d_thet  = x(11);
    d_psi  = x(12);
    bax = x(13);
    bay = x(14);
    baz = x(15);
    bgx = x(16);
    bgy = x(17);
    bgz = x(18);
    bmx = x(19);
    bmy = x(20);
    bmz = x(21);
    q1  = q(1);
    q2  = q(2);
    q3  = q(3);
    q4  = q(4);
    A = eval(tmp_A);
    
    %estimation step
    [x_est,P_est]=estimation(A,P,Q,x);
    
    if(meas_time(i)>totalTime)
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        
    else
        % the values are now from the estimated x
    lat = x_est(1);
    long= x_est(2);
    alt = x_est(3);
    vn  = x_est(4);
    ve  = x_est(5);
    vd  = x_est(6);
    phi = x_est(7);
    thet = x_est(8);
    pssi = x_est(9);
    d_phi  = x_est(10);
    d_thet  = x_est(11);
    d_psi  = x_est(12);
    bax = x_est(13);
    bay = x_est(14);
    baz = x_est(15);
    bgx = x_est(16);
    bgy = x_est(17);
    bgz = x_est(18);
    bmx = x_est(19);
    bmy = x_est(20);
    bmz = x_est(21);
    vn_old=x(4);
    ve_old=x(5);
    vd_old=x(6);
    phi_old=x(7);
    thet_old=x(8);
    psi_old=x(9);
    d_phi_old=x(10);
    d_thet_old=x(11);
    d_psi_old=x(12);
    H = eval(tmp_H);
    
    %correction step
    
    i_old =i-1;
    while(meas_time(i+1)<=totalTime)
        i=i+1;
    end
    z_new=M(:,i);
    counter_new=counter(:,i);
    counter_old=counter(:,i_old);
    length_z=size(z_new);
   
    meas_control= d_meas(counter_new,counter_old,length_z);
    
    x_tmp=x_est;
    P_tmp=P_est;
    for j=1:size(meas_control)                  % if we have new data, correction step is done
        if meas_control(j)==1
            [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp,j);
            
        end
            
    end
    x_new=x_tmp;
    P=P_tmp;
    end
    
    
    %reset
    phi = x_new(7);
    thet= x_new(8);
    pssi = x_new(9);
    
    DCM=eval(DCM_bi');
    q=DCMtoQ(DCM);
    deuler2body=calc_deuler2body(x_new(7),x_new(8),x_new(9));
    
    
    x_new(7)=0;
    x_new(8)=0;
    x_new(9)=0;
    x_new(10:12)=DCM*deuler2body*x_new(10:12);
    
    
    x=x_new;
    save(:,i)=x_new;
    save_est(:,i)=x_est;
    
    
end





