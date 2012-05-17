function [state,state_est] = state_estimation_3(M,W,meas_time,counter)

%------------------------
% constants are defined
%------------------------
t=0.01; % [s] time interval between two measurements
G= 9.85; % [m/s^2] gravitation in zurich
dis=[0;0;0];%displacementvector of the box
MAG1=21449.9;% [nano tesla] magnetic field in zurich
MAG2=595.1;
MAG3=42682.6;
Rl= geocradius(47+24/60); %from zurich
Rp= Rl*cos(8+32/60); %from zurich
lat0=47+24/60;
long0=8+32/60;
% noise form Xsens datasheet
NOISE_ACC_b=[0.002;0.002;0.002];%noise accelerometer
NOISE_GYRO_b=[0.05;0.05;0.05];%noise gyro
NOISE_MAG_b=[0.5;0.5;0.5];%check data sheet again!!!!!!!!!!!!!!

NOISE_GPS_POS=0;% Noise in position of the GPS
NOISE_GPS_VEL=0;%Noise in velocity of the GPS



%------------------------
% variables with initial values are defined
%------------------------
x = zeros(21)';
P = zeros(size(x,2),size(x,2));
q = [1,0,0,0]';

syms q1 q2 q3 q4;
syms vn_old ve_old vd_old;
syms lat long alt vn ve vd phi thet psi d_phi d_thet d_psi bax bay baz bgx bgy bgz bmx bmy bmz vn_old ve_old vd_old phi_old thet_old psi_old d_phi_old d_thet_old d_psi_old real;
syms lat_e long_e alt_e vn_e ve_e vd_e phi_e thet_e psi_e d_phi_e d_thet_e d_psi_e bax_e bay_e baz_e bgx_e bgy_e bgz_e bmx_e bmy_e bmz_e real;
vf=[lat, long, alt, vn, ve, vd, phi, thet, psi, d_phi, d_thet, d_psi, bax, bay, baz, bgx, bgy, bgz, bmx, bmy, bmz];
quat= [q1, q2, q3, q4];
vh=[lat_e, long_e, alt_e, vn_e, ve_e, vd_e, phi_e, thet_e, psi_e, d_phi_e, d_thet_e, d_psi_e, bax_e, bay_e, baz_e, bgx_e, bgy_e, bgz_e, bmx_e, bmy_e, bmz_e];



%------------------------
% direct cosine matrice gets calculated
%------------------------
DCM_ir=calc_DCM_ir(quat);
DCM_br=calc_DCM_br(phi,thet,psi);
DCM_bi=DCM_br*DCM_ir';


%------------------------
% the physical model and the measurement-state realation are defined. the
% jacobian matrix is calculated
%------------------------

x_i=(lat-lat0)*Rl;
y_i=(long-long0)*Rp;
z_i=-alt;
pos=[x_i,y_i,z_i];
R=norm(pos);
rotvec=vrrotvec(pos./R,[0,0,-1]);
alpha=rotvec(4);

f=




%definition of h
w=[d_phi;0;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[0;d_thet;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0;0;d_psi];
w_old=[d_phi_old;0;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[0;d_thet_old;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[cos(thet_old), 0, -sin(thet_old);0,1,0;sin(thet_old),0,cos(thet_old)]*[0;0;d_psi_old];
vg=(cross(w',dis'))';
vg_old=(cross(w_old',dis'))';
v=[vn;ve;vd];
v_old=[vn_old,ve_old,vd_old];

h1=[lat;long;alt] + DCM_bi'*dis; %pos
h2=v + DCM_bi'*vg; %vel
h3=DCM_bi*(((v+DCM_bi'*vg)-(v_old+DCM_bi'*vg_old))./t+G);%acc
h4=w;%gyr
h5=DCM_bi*[MAG1;MAG2;MAG3];%magnetometer
h6=0;%pressure sensor
h=[h1;h2;h3;h4;h5;h6];


tmp_A= jacobian(f,vf);
tmp_H= jacobian(h,vf);

%------------------------
% Q and R are calculated
%------------------------
NOISE_ACC_i=DCM_ir*(NOISE_ACC_b.^2);
NOISE_VEL_i=(NOISE_ACC_i.^2)*t;
NOISE_POS_i=(NOISE_VEL_i.^2)*t;
NOISE_GYRO_i=DCM_ir(NOISE_GYRO_b.^2);
q_diag=[NOISE_POS_i',NOISE_VEL_i',NOISE_GYRO_i',0,0,0,0,0,0,0,0,0];
Q=diag(q_diag);


r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b.^2)',(NOISE_MAG_B.^)'];
R=diag(r);



%------------------------
% for every measurement the Extended Kalman Filter is used for state
% estimation
%------------------------
totalTime=0;
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
    psi = x(9);
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
    psi = x_est(9);
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
    while(meas_time(i)<=totalTime)
        i=i+1;
    end
    z_new=M(:,i);
    counter_new=counter(:,i);
    counter_old=counter(:,i_old);
    length_z=size(z_new);
   
    meas_control= d_meas(counter_new,counter_old,length_z);
    
    for j=1:size(meas_control)
        if meas_control(j)==1
            [x_new(j),P(j,:)]= correction2(P_est,H(j,:),R(j,:),z_new(j),x_est,j);
        else
            x_new(j)=x_est(j);
            P(j,:)=P_est(j,:);
        end
    end
    end
    
    
    %rest
    phi = x_new(7);
    psi = x_new(8);
    thet = x_new(9);
    
    DCM=eval(DCM_bi');
    q=DCMtoQ(DCM);

    x_new(7)=0;
    x_new(8)=0;
    x_new(9)=0;
    
    x=x_new;
    state(:,i)=x_new;
    state_est(:,i)=x_est;
    
    
end





