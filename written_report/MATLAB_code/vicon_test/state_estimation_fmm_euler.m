% free mass model

%constants are defined
t=0.01; % [s] time interval between two measurements
G= [0;0;9.85]; % [m/s^2] gravitation in zurich
dis=[0;0;0];%displacementvector of the box
MAG1=0.2145;% [Gauss] magnetic field in zurich
MAG2=0.0060;
MAG3=0.4268;
true_mag=[-0.119784172661871;-0.0330935251798561;0.341007194244604;];
mag_angle=-1.496;
mag=[cos(mag_angle),sin(mag_angle),0;-sin(mag_angle),cos(mag_angle),0;0,0,1]*true_mag;
%Rl= geocradius(47+24/60); %from zurich
%Rp= Rl*cos(8+32/60); %from zurich
lat0=47+24/60;
long0=8+32/60;
% defining the noise covariance
NOISE_ACC_b=50*[0.14;0.14;0.14];% [m/s^2/sqrt(Hz)]noise acc   Xsens: [0.002;0.002;0.002]
NOISE_GYRO_b=[0.3;0.3;0.3]*2*pi/360;% [rad/s] noise gyro      Xsens: [0.05;0.05;0.05]./360.*2*pi
NOISE_MAG_b=0.1*[0.002;0.002;0.002];%[gauss]                         Xsens: [0.5e-3;0.5e-3;0.5e-3]

NOISE_GPS_POS=0.001;% Noise in position of the GPS
NOISE_GPS_VEL=0.001;%Noise in velocity of the GPS


%------------------------
% variables with initial values are defined
%------------------------

x = [0.451525,-0.02503,-0.7155918,...
    -0.1635,-0.7756,-0.0852,...
    0,-0.563581455457894, -0.0553776831161063,...
    0,0.173924051749699,-1.73248360301804,...
    0,0,0,0,0,0,0,0,0]';
P = zeros(size(x,1),size(x,1));
%------------------------
% Q and R are calculated
%------------------------


% noise predection
q_diag=[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0,0,0,0,0,0,0,0,0];
Q=diag(q_diag);

% noise correction
r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b)',(NOISE_GYRO_b)',(NOISE_MAG_b)'];
R=diag(r);



%%
%------------------------
% for every measurement the Extended Kalman Filter is used for state
% estimation
%------------------------
i=1;
totalTime=meas_time(i) %time of the next estimated state

k=1;
while (i<size(M,2))%size(M,2)
    totalTime=totalTime+t
    %------------------------
    % direct cosine matrice gets calculated
    %------------------------

    DCM_bi=calc_DCM_br(x(7),x(8),x(9));
    

   
    
    
    [x_est,A]=jaccsd_free_mass_model(@free_mass_model,x,t); % the jaccobi matrix is callculated and the the state estimated
    
    %estimation step
    P_est=A*P*A'+Q;
    
    % there is no new sensor value. the ekf is just propagating
    if(meas_time(i+1)>totalTime)
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        [z_est,H]=jaccsd_h_euler(@h_euler_x,x_est,x,t,dis,G,mag);
        
    else
    
    
    %correction step
   
    [z_est,H]=jaccsd_h_euler(@h_euler_x,x_est,x,t,dis,G,mag); % the jaccobi
    % matrix of the measurement model and the estimated measurements are calculated
    
    
    
    
    if(i==1)
        i_old=10;
    else
        i_old =i;
    end
    
    while(meas_time(i+1)<=totalTime) %looking for the closest measurement value to the estimation time "totalTime"
        i=i+1;
    end
    
    z_new=M(:,i);
    counter_new=counter(:,i);
    counter_old=counter(:,i_old);
    length_z=size(z_new);
    
    
    meas_control= d_meas(counter_new,counter_old,length_z);% which sensors have new datas
    
    x_tmp=x_est;
    P_tmp=P_est;
    for j=1:size(meas_control,2)                  % only for the measurements from which we have new sensor data, the correction step is done
        if meas_control(j)==1
%             [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp,j);
              P12=P_tmp*H(j,:)';                   %cross covariance
              % K=P12*inv(H*P12+R);       %Kalman filter gain
              % x=x1+K*(z-z1);            %state estimate
              % P=P-K*P12';               %state covariance matrix
              [S,p]=chol(H(j,:)*P12+R(j,j));  %Cholesky factorization
               
              if p~=0
                    disp('not positive definite') % this is required for the argument of chol(...), otherwise the filter is unstable
                    i
                    j
              end
             
                   U=P12/S;                    %K=U/R'; Faster because of back substitution
                   x_tmp=x_tmp+U*(S'\(z_new(j)-z_est(j)));         %Back substitution to get state update
                   P_tmp=P_tmp-U*U';

            
        end
            
    end
    x_new=x_tmp;
    P=P_tmp;
    end
    
    
    % the coordinate systems of the IMU and the vicon has to be alligned
    DCM_b2i_n=calc_DCM_br(real(x_new(7)),real(x_new(8)),real(x_new(9)));
    psi=-0.2;
    thet=0.07;
    DCM_br_n=[cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[1 0 0;0 -1 0; 0 0 -1]*DCM_b2i_n;
    diss=[0.0;-0.02;0];
    


    % all data is saved to plot it afterwards
    save_t(k)=totalTime;
    save_pos(:,k)=[x_new(1);x_new(2);x_new(3)]+transp(DCM_br_n)*diss;
    save_orientation(:,k)=[-atan2(-DCM_br_n(3,2),-DCM_br_n(3,3))  ...
              asin(-DCM_br_n(3,1)) ...
              atan2(-DCM_br_n(2,1),-DCM_br_n(1,1))];
    
    
    k=k+1;
    
    x=x_new; % the corrected estimated state x_new is now the state x
  
    
end

%% Plots against ground truth
range=[75 78];
plot_range=[max(save_t(1),range(1))-2 min(save_t(size(save_t,2)),range(2))+2];
[val,mintime]=min(abs(meas_time_VI_gt-save_t(1)));
[val,maxtime]=min(abs(meas_time_VI_gt-save_t(size(save_t,2))));
% figure(1);subplot(3,1,1);plot(save_t,save_pos(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(1,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos x [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,2);plot(save_t,save_pos(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(2,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos y [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,3);plot(save_t,save_pos(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(3,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos z [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% figure(2);subplot(3,1,1);plot(save_t,save_orientation(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(6,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\phi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,2);plot(save_t,save_orientation(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(5,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\theta [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,3);plot(save_t,save_orientation(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(4,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\psi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
% 
figure(1);subplot(3,2,1);plot(save_t,save_pos(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(1,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos x [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
subplot(3,2,3);plot(save_t,save_pos(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(2,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos y [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
subplot(3,2,5);plot(save_t,save_pos(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(3,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos z [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
subplot(3,2,2);plot(save_t,save_orientation(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(6,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\phi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
subplot(3,2,4);plot(save_t,save_orientation(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(5,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\theta [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
subplot(3,2,6);plot(save_t,save_orientation(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(4,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\psi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
%% plot errors

range=[0 135];
plot_range=[max(save_t(1),range(1)) min(save_t(size(save_t,2)),range(2))];
[val,mintime]=min(abs(save_t-plot_range(1)));
[val,maxtime]=min(abs(save_t-plot_range(2)));

VI_gt_k=interp1(meas_time_VI_gt,VI_gt',save_t);
VI_gt_k=VI_gt_k';
pos_error=sqrt((save_pos(1,:)-VI_gt_k(1,:)).^2+(save_pos(2,:)-VI_gt_k(2,:)).^2+(save_pos(3,:)-VI_gt_k(3,:)).^2);
psi_error=abs(save_orientation(3,:)-VI_gt_k(4,:));
figure(3);
[AX,H1,H2]=plotyy(save_t(mintime:maxtime),pos_error(mintime:maxtime),save_t(mintime:maxtime),psi_error(mintime:maxtime));
xlabel('time [s]');
set(get(AX(1),'Ylabel'),'String','Error in position [m]');
set(get(AX(2),'Ylabel'),'String','Error in \psi [rad]') 

%% calculate mean errors
range=[55 135];
plot_range=[max(save_t(1),range(1)) min(save_t(size(save_t,2)),range(2))];
[val,mintime]=min(abs(save_t-plot_range(1)));
[val,maxtime]=min(abs(save_t-plot_range(2)));

mean_pos_error=mean(pos_error(mintime:maxtime))
mean_psi_error=mean(psi_error(mintime:maxtime))

%--------------------------------------
% for X sens
%--------------------------------------

%% pos and vel compared to ground truth
figure(5);
subplot(2,1,1);plot(save_t,save_corr(1,:),save_t,save_est(1,:),segment1_time_ground_truth_X,segment1_ground_truth_X(1,:));legend('pos x corrected','pos x estimated','pos x ground truth')
subplot(2,1,2);plot(save_t,save_corr(4,:),save_t,save_est(4,:),segment1_time_ground_truth_X,segment1_ground_truth_X(4,:));legend('vel x corrected','vel x estimated','vel x ground truth')
%% psi
figure(6);plot(save_t,-mod(save_est(9,:),2*pi)+pi,save_t,-mod(save_corr(9,:),2*pi)+pi,segment1_time_ground_truth_X, segment1_ground_truth_X(7,:));legend('psi estimated','psi corrected','psi ground truth')
%% thet
figure(7);plot(save_t,save_est(7,:),segment1_time_ground_truth_X, segment1_ground_truth_X(9,:));legend('euler x','euler gt')
%% phi
figure(8);plot(save_t,-save_est(8,:),save_t,-save_corr(8,:),segment1_time_ground_truth_X, segment1_ground_truth_X(9,:));legend('est','new','euler gt')


%% plot actual measurements against expected measurements
figure(1);plot(save_t,save_z_est(1:3,:),'o-',save_t,save_z(1:3,:),'.');title('position')
figure(2);plot(save_t,save_z_est(4:6,:),'o-',save_t,save_z(4:6,:),'.');title('velocity')
figure(3);plot(save_t,save_z_est(7:9,:),'o-',save_t,save_z(7:9,:),'.');title('accelerometer')
figure(4);plot(save_t,save_z_est(10:12,:),'o-',save_t,save_z(10:12,:),'.');title('gyros')
figure(5);plot(save_t,save_z_est(13:15,:),'o-',save_t,save_z(13:15,:),'.');title('magnetometer')
%%
figure(6);plot(save_t,save(7:9,:),'o-',save_t,save_anlges(1:3,:),'.');title('vicon orientation')

%%
figure(9);plot(save_t,save_deviation(8,:))
%%
