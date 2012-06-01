%state_estimation_4
%------------------------
% constants are defined
%------------------------
t=0.01; % [s] time interval between two measurements
G= [0;0;9.85]; % [m/s^2] gravitation in zurich
dis=[0;0;0];%displacementvector of the box
MAG1=0.2145;% [Gauss] magnetic field in zurich
MAG2=0.0060;
MAG3=0.4268;
mag=[MAG1,MAG2,MAG3];
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
quat = [0,0,1,0]';
%------------------------
% Q and R are calculated
%------------------------
q_diag=[0.001,0.001,0.001,0.001,0.001,0.001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0,0,0,0,0,0,0,0,0];
Q=diag(q_diag);

r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b.^2)',(NOISE_GYRO_b.^2)',(NOISE_MAG_b.^2)'];
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
    %------------------------
    % direct cosine matrice gets calculated
    %------------------------
    DCM_ir=calc_DCM_ir(quat);
    DCM_br=calc_DCM_br(x(7),x(8),x(9));
    DCM_bi=DCM_br*DCM_ir';
    
    [x_est,A]=jaccsd_f(@f,@f_imag,x,DCM_ir,DCM_br,t);
    
    %estimation step
    P_est=A*P*A'+Q;
    
    if(meas_time(i)>totalTime)
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        
    else
    
    
    %correction step
   
    [z_est,H]=jaccsd_h(@h,x_est,x,DCM_bi,t,dis,G,mag);
    
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
%             [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp,j);
              P12=P_tmp*H(j,:)';                   %cross covariance
              % K=P12*inv(H*P12+R);       %Kalman filter gain
              % x=x1+K*(z-z1);            %state estimate
              % P=P-K*P12';               %state covariance matrix
              S=chol(H(j,:)*P12+R(j,j));            %Cholesky factorization
              U=P12/S;                    %K=U/R'; Faster because of back substitution
              x_tmp=x_tmp+U*(S'\(z(j)-z_est(j)));         %Back substitution to get state update
              P_tmp=P_tmp-U*U';
            
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
    quat=DCMtoQ(DCM);
    deuler2body=calc_deuler2body(x_new(7),x_new(8),x_new(9));
    
    
    x_new(7)=0;
    x_new(8)=0;
    x_new(9)=0;
    x_new(10:12)=DCM*deuler2body*x_new(10:12);
    
    
    x=x_new;
    save(:,i)=x_new;
    save_est(:,i)=x_est;
    
    
end
