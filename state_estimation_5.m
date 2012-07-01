%state_estimation_5
%------------------------
% constants are defined
%------------------------
t=0.01; % [s] time interval between two measurements
G= [0;0;9.85]; % [m/s^2] gravitation in zurich
dis=[0;0;0];%displacementvector of the box
MAG1=0.2145;% [Gauss] magnetic field in zurich
MAG2=0.0060;
MAG3=0.4268;
RAD=0.8465;


%Rl= geocradius(47+24/60); %from zurich
%Rp= Rl*cos(8+32/60); %from zurich
lat0=47+24/60;
long0=8+32/60;
% noise form Xsens datasheet
NOISE_ACC_b=10*[0.14;0.14;0.14];% [m/s^2/sqrt(Hz)]noise acc   Xsens: [0.002;0.002;0.002]
NOISE_GYRO_b=50*[0.3;0.3;0.3]*2*pi/360;% [rad/s] noise gyro      Xsens: [0.05;0.05;0.05]./360.*2*pi
NOISE_MAG_b=100*[0.002;0.002;0.002];%[gauss]                         Xsens: [0.5e-3;0.5e-3;0.5e-3]

NOISE_GPS_POS=0.001;% Noise in position of the GPS
NOISE_GPS_VEL=0.01;%Noise in velocity of the GPS



%------------------------
% variables with initial values are defined (probably needs to be tuned by
% correction....)
% x= [-2.5;1.0077;-0.0606;...
%     0.8465;...
%    -0.1 ;0.1617;-1.7352;...
%     0;];
x=[-2.8;1;0;RAD;0;0;0;0];
x(1)=mod(x(1),2*pi);
    x(2)=mod(x(2),2*pi);
    x(3)=mod(x(3),2*pi);

%---------------------------------
%initial covariance Matrix
P = eye(size(x,1),size(x,1));%P(1,1)=10;P(5,5)=10;
quat = [1,0,0,0]';
%------------------------
% Q and R are calculated (estimation only by trial and error)
%------------------------
q_diag=0.05*[0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001];
Q=diag(q_diag);

r=[NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_POS,NOISE_GPS_VEL,NOISE_GPS_VEL,NOISE_GPS_VEL,(NOISE_ACC_b)',(NOISE_GYRO_b)',(NOISE_MAG_b)'];
R=diag(r);






%%
%------------------------
% for every measurement the Extended Kalman Filter is used for state
% estimation
%------------------------
totalTime=meas_time(1); %time of the next estimated state
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
    [x_est,A]=jaccsd_5(@f5,x,t); % jacobian matrix(A) of f5 (pendulum model) and estimated state(x_est)are calculated
    
    
    P_est=A*P*A'+Q;
    
    if(meas_time(i+1)>totalTime) % if no new sensor measurements are available, the filter keeps propagating
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        [z_est,H]=jaccsd_5(@h5_x,x_est,t);
        
    else
    
    %-----------------------
    % correction step
    %-----------------------
    
   
    [z_est,H]=jaccsd_5(@h5_x,x_est,t);  % jacobian matrix (H) of the h5_x (measurement-state relation) 
                                        % and estimated measurements (z_est) are calculated
    
    %----------------------
    % find the newest measurement value
    %----------------------
    if(i==1)
        i_old=10;
    else
        i_old =i;
    end
    while(meas_time(i+1)<=totalTime)
        i=i+1;
    end
    z_new=M(:,i); %all sensor data is saved in the matrix M
    counter_new=counter(:,i);
    counter_old=counter(:,i_old);
    length_z=size(z_new);
   
    meas_control= d_meas(counter_new,counter_old,length_z); % d_meas tells us which sensors have new data
    
    x_tmp=x_est;
    P_tmp=P_est;
    
    %-----------------------
    % only perform correction using new measurements
    %-----------------------
    
    for j=1:size(meas_control,2)                  % if we have new data, correction step is done
        if meas_control(j)==1 && (j<=6 || (j>=7 && j<=15)) % after && define wich measurements are to be used for correction (in this care magn is ignored)
%             [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp,j);
              P12=P_tmp*H(j,:)';                   %cross covariance
              % K=P12*inv(H*P12+R);       %Kalman filter gain
              % x=x1+K*(z-z1);            %state estimate
              % P=P-K*P12';               %state covariance matrix
              S=chol(H(j,:)*P12+R(j,j));            %Cholesky factorization
              U=P12/S;                    %K=U/R'; Faster because of back substitution
              x_tmp=x_tmp+U*(S'\(z_new(j)-z_est(j)));         %Back substitution to get state update
              P_tmp=P_tmp-U*U';
        
            
        end
            
    end
    x_new=x_tmp;%x_new is the corrected step according to the measurements
    P=P_tmp;
    end
    
    %saving:

     save_z(:,k)=z_new;
     save_z_used(:,k)=z_new.*meas_control';

    
    % coordinate system of vicon and IMU has to be aligned
    DCM_b2i_n=calc_DCM_br(real(x_new(1)),real(x_new(2)),real(x_new(3)));
    psi=-0.2;
    thet=0.07;
    DCM_br_n=[cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0,0,-1;0,1,0;1,0,0]*DCM_b2i_n;
    dis=[0.0;-0.02;0];
    


    % the data is savet to plot it afterwards
    save_t(k)=totalTime;
    save_pos(:,k)=RAD*[cos(x_new(3))*cos(x_new(2));sin(x_new(3))*cos(x_new(2));-sin(x_new(2))]+transp(DCM_br_n)*dis;
    save_orientation(:,k)=[-atan2(-DCM_br_n(3,2),-DCM_br_n(3,3))  ...
              asin(-DCM_br_n(3,1)) ...
              atan2(DCM_br_n(2,1),DCM_br_n(1,1))];
    k=k+1;

  
    x=x_new;%the corrected state is the next state
    
    x(1)=mod(x(1),2*pi);
    x(2)=mod(x(2),2*pi);
    x(3)=mod(x(3),2*pi);
end

%% Plots against ground truth
range=[47 57];
plot_range=[max(save_t(1),range(1))-0.2 min(save_t(size(save_t,2)),range(2))+2];
[val,mintime]=min(abs(meas_time_VI_gt-save_t(1)));
[val,maxtime]=min(abs(meas_time_VI_gt-save_t(size(save_t,2))));
% figure(1);subplot(3,1,1);plot(save_t,save_pos(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(1,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos x [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,2);plot(save_t,save_pos(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(2,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos y [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,3);plot(save_t,save_pos(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(3,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('pos z [m]');xlabel('time [s]');set(gca,'xlim',plot_range);
% figure(2);subplot(3,1,1);plot(save_t,save_orientation(1,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(6,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\phi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,2);plot(save_t,save_orientation(2,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(5,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\theta [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);
% subplot(3,1,3);plot(save_t,save_orientation(3,:),'r-',meas_time_VI_gt(mintime:maxtime),VI_gt(4,mintime:maxtime),'k--');legend('state estimator','vicon');ylabel('\psi [rad]');xlabel('time [s]');set(gca,'xlim',plot_range);

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

%%
%plot(save_t,save(1:3,:),save_t,save_est(1:3,:),meas_time,M(1:3,:))
figure(1);plot(save_t,save_anlges(1:3,:),'.',save_t,save_est(1:3,:),'o',save_t,save_corr(1:3,:),'x-');
figure(2);plot(save_t,save(5:7,:),'o-',save_t,save_est(5:7,:),'.',save_t,save_corr(5:7,:),'x-');
figure(3);plot(save_t,save([4 8],:),'o-',save_t,save_est([4 8],:),'.',save_t,save_corr([4 8],:),'x-');

%% plot expected measurements against actual measurements!
%figure(1);plot(save_t,save_z_est(1:3,:),'o-',save_t,save_z(1:3,:),'.');title('position')
figure(1);plot(save_t,save_z_est(1:3,:),'o-',save_t,save_z_used(1:3,:),'.');title('position')
%figure(2);plot(save_t,save_z_est(4:6,:),'o-',save_t,save_z(4:6,:),'.');title('velocity')
figure(2);plot(save_t,save_z_est(4:6,:),'o-',save_t,save_z_used(4:6,:),'.');title('velocity')
figure(3);plot(save_t,save_z_est(7:9,:),'o-',save_t,save_z(7:9,:),'.');title('acceleration')
figure(4);plot(save_t,save_z_est(10:12,:),'o-',save_t,save_z(10:12,:),'.');title('gyros')
%figure(4);plot(save_t,save_est(5,:),'o-',save_t,-save_est(7,:).*sin(save_est(2,:)),'o-',save_t,save_z(10:12,:),'.');
figure(5);plot(save_t,save_z_est(13:15,:),'o-',save_t,save_z(13:15,:),'.');title('magnetometer')
%figure(6);plot(save_t,save_z_est(16:18,:),'o-',save_t,save_anlges(1:3,:),'.');title('vicon orientation')