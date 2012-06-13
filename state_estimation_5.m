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

%Rl= geocradius(47+24/60); %from zurich
%Rp= Rl*cos(8+32/60); %from zurich
lat0=47+24/60;
long0=8+32/60;
% noise form Xsens datasheet
NOISE_ACC_b=10*[0.14;0.14;0.14];% [m/s^2/sqrt(Hz)]noise acc   Xsens: [0.002;0.002;0.002]
NOISE_GYRO_b=10*[0.3;0.3;0.003]*2*pi/360;% [rad/s] noise gyro      Xsens: [0.05;0.05;0.05]./360.*2*pi
NOISE_MAG_b=1*[0.002;0.002;0.002];%[gauss]                         Xsens: [0.5e-3;0.5e-3;0.5e-3]

NOISE_GPS_POS=0.0005;% Noise in position of the GPS
NOISE_GPS_VEL=0.005;%Noise in velocity of the GPS



%------------------------
% variables with initial values are defined (probably needs to be tuned by
% correction....
x= [1.9;1.0077;-0.0606;...
    0.8465;...
   -2.5 ;0.1617;-1.7352;...
    0;];
x(1)=mod(x(1),2*pi);
    x(2)=mod(x(2),2*pi);
    x(3)=mod(x(3),2*pi);

% %---------------------------------
% %estimation of initial values by correction... (worth a try)
% x=[0.451525000000002;-0.0250371165313001;-0.715591800000000;-0.163500000000122;-0.775600000000603;-0.0343958737111145;-1.01123647853415;0.777590980977478;3.14979246534883;-3.46919594694992;-5.15601930029043;7.46617258810799;0;0;0;0;0;0;0;0;0;];
% %---------------------------------
P = zeros(size(x,1),size(x,1));%P(1,1)=10;P(5,5)=10;
quat = [1,0,0,0]';
%------------------------
% Q and R are calculated
%------------------------
q_diag=0.1*[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001];
Q=diag(q_diag);

r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b)',(NOISE_GYRO_b)',(NOISE_MAG_b)'];
R=diag(r);






%%
%------------------------
% for every measurement the Extended Kalman Filter is used for state
% estimation
%------------------------
totalTime=meas_time(1);
i=1;
k=1;
while (i<size(M,2))
    totalTime=totalTime+t
    %------------------------
    % direct cosine matrice gets calculated
    %------------------------
    DCM_ir=calc_DCM_ir(quat);
    
    %------------------------
    % estimation step
    %------------------------
    [x_est,A]=jaccsd_5(@f5,x,t);
    deviation0=x_est-x;
    
    P_est=A*P*A'+Q;
    
    if(meas_time(i+1)>totalTime)
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        [z_est,H]=jaccsd_5(@h5,x_est,t);
        
    else
    
    %-----------------------
    % correction step
    %-----------------------
    
   
    [z_est,H]=jaccsd_5(@h5,x_est,t);
    
    if(i==1)
        i_old=10;
    else
        i_old =i-1;
    end
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
    
    %deviation1=z_est-z_new;
    %do not use mag!
    
%     P12=P_tmp*H([1:6 7:9 10:12 13:15],:)';                   %cross covariance
%     % K=P12*inv(H*P12+R);       %Kalman filter gain
%     % x=x1+K*(z-z1);            %state estimate
%     % P=P-K*P12';               %state covariance matrix
%     S=chol(H([1:6 7:9 10:12 13:15],:)*P12+R([1:6 7:9 10:12 13:15],[1:6 7:9 10:12 13:15]));            %Cholesky factorization
%     U=P12/S;                    %K=U/R'; Faster because of back substitution
%     
%     x_tmp=x_tmp+U*(S'\(z_new([1:6 7:9 10:12 13:15])-z_est([1:6 7:9 10:12 13:15])));  %Back substitution to get state update
%     
%     P_tmp=P_tmp-U*U';
    
    
    
    
    %for j=1:3
%     for j=1:size(meas_control,2)                  % if we have new data, correction step is done
%         if meas_control(j)==1 && (j<=6 || (j>=7 && j<=9))
% %             [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp,j);
%               P12=P_tmp*H(j,:)';                   %cross covariance
%               % K=P12*inv(H*P12+R);       %Kalman filter gain
%               % x=x1+K*(z-z1);            %state estimate
%               % P=P-K*P12';               %state covariance matrix
%               S=chol(H(j,:)*P12+R(j,j));            %Cholesky factorization
%               U=P12/S;                    %K=U/R'; Faster because of back substitution
%               x_tmp=x_tmp+U*(S'\(z_new(j)-z_est(j)));         %Back substitution to get state update
%               P_tmp=P_tmp-U*U';
%             
%         end
%             
%     end
    x_new=x_tmp;
    P=P_tmp;
    end
    
    %saving:
    save(:,k)=x;
    save_est(:,k)=x_est;
    save_corr(:,k)=x_new;
    save_t(k)=totalTime;
    save_z_est(:,k)=z_est;
    save_z(:,k)=z_new;
    [val,ind]=min(abs(meas_time_P-meas_time(i)));
    save_anlges(:,k)=angles_VI_p(:,ind);
    k=k+1;
    %reset
        
%     DCM_br=calc_DCM_br(x_new(7),x_new(8),x_new(9));
%     DCM_bi=DCM_br*DCM_ir';
    
%     quat=DCMtoQ(DCM_bi');
%     deuler2body=calc_deuler2body(x_new(7),x_new(8),x_new(9));
%     
%     
%     x_new(7)=0;
%     x_new(8)=0;
%     x_new(9)=0;
%     %x_new(10:12)=DCM_br'*deuler2body*x_new(10:12);
%     x_new(10:12)=deuler2body*x_new(10:12);
    
    deviation2=x_new-x_est;
    x=x_new;
    
    x(1)=mod(x(1),2*pi);
    x(2)=mod(x(2),2*pi);
    x(3)=mod(x(3),2*pi);
end


%%
%plot(save_t,save(1:3,:),save_t,save_est(1:3,:),meas_time,M(1:3,:))
figure(1);plot(save_t,save_anlges(1:3,:),'.',save_t,save_est(1:3,:),'o',save_t,save_corr(1:3,:),'x-');
figure(2);plot(save_t,save(5:7,:),'o-',save_t,save_est(5:7,:),'.',save_t,save_corr(5:7,:),'x-');
figure(3);plot(save_t,save([4 8],:),'o-',save_t,save_est([4 8],:),'.',save_t,save_corr([4 8],:),'x-');

%%
figure(1);plot(save_t,save_z_est(1:3,:),'o-',save_t,save_z(1:3,:),'.');
figure(2);plot(save_t,save_z_est(4:6,:),'o-',save_t,save_z(4:6,:),'.');
figure(3);plot(save_t,save_z_est(7:9,:),'o-',save_t,save_z(7:9,:),'.');
figure(4);plot(save_t,save_z_est(10:12,:),'o-',save_t,save_z(10:12,:),'.');
figure(5);plot(save_t,save_z_est(13:15,:),'o-',save_t,save_z(13:15,:),'.');
figure(6);plot(save_t,save_z_est(16:18,:),'o-',save_t,save_anlges(1:3,:),'.');