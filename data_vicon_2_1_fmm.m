% This file imports the data from the vicon, the pxhawk and the xsens. Then
% this date is arranged in order to be processed by the ekf
%

%---------------------------------------------
%Import DATA
%---------------------------------------------
%% Importing PixHawk
fid_pixhawk = fopen('p_test_2_1.txt');
[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid_pixhawk);
%% Import vicon file
VI=csvread('positionlog_KiteBox_19.23.21.533.csv');


%------------------------
%bringing sensors and vicon together. NO NOISE added to vicon
%------------------------
%% deleting error measurements
[meas_time_P,acc_P,gyro_P,magn_P,counter_P]=delete_meas_error(10,580,meas_time_P,acc_P,gyro_P,magn_P,counter_P);

%% VI posiotion and velocity + PixHawk
dt_vicon=0.00100;
meas_time_VI=VI(:,1).*dt_vicon;
meas_time_VI_new=meas_time_VI';
VI_new=VI';

%position
for i=2:4
      pos_VI_p_noisy(i-1,:)=awgn(interp1(meas_time_VI_new+42.622+0.5100+0.49,VI_new(i,:),meas_time_P),-5);
      pos_VI_p(i-1,:)=interp1(meas_time_VI_new+42.622+0.5100+0.49,VI_new(i,:),meas_time_P);
end

% velocity . The velocity is calculted and added with noise to simulate the
% velocity from the gps
vel_tmp=zeros(3,size(meas_time_P,2)-1);
vel=zeros(3,size(meas_time_P,2));
for i=1:3
    vel_tmp(i,:)=diff(pos_VI_p(i,:))./diff(meas_time_P);
    
end
vel=([vel_tmp(:,1),vel_tmp(:,:)]);
for j=2:size(meas_time_P,2);

    if isnan(vel(1,j))
        vel(1,j)=vel(1,j-1);
    end
    if isnan(vel(2,j))
        vel(2,j)=vel(2,j-1);
    end
    if isnan(vel(3,j))  
       vel(3,j)=vel(3,j-1);
    end
end
vel_noisy=awgn(vel,-5);

%angles
for i=5:7
     angles_VI_p(i-4,:)=interp1(meas_time_VI_new+42.622+0.5100+0.49,VI_new(i,:),meas_time_P);
end

% %calculating the quaternion
% for i=1:size(meas_time_P,2)
%     DCM_bi=calc_DCM_br(angles_VI_p(1,i),angles_VI_p(3,i),angles_VI_p(2,i));
%     quat_VI_1(:,i)=DCMtoQ(DCM_bi'); 
% end
% 
% for i=1:size(meas_time_P,2)
%     DCM_bi=calc_DCM_br(angles_VI_p(2,i),angles_VI_p(3,i),angles_VI_p(1,i));
%     quat_VI_2(:,i)=DCMtoQ(DCM_bi'); 
% end
% 
% for i=1:size(meas_time_P,2)
%     DCM_bi=calc_DCM_br(angles_VI_p(3,i),angles_VI_p(2,i),angles_VI_p(1,i));
%     quat_VI_3(:,i)=DCMtoQ(DCM_bi'); 
% end
% for i=1:size(meas_time_P,2)
%     DCM_bi=calc_DCM_br(angles_VI_p(3,i),angles_VI_p(1,i),angles_VI_p(2,i));
%     quat_VI_4(:,i)=DCMtoQ(DCM_bi'); 
% end

ground_truth=[pos_VI_p/1000;vel/1000;angles_VI_p]; %position, velocity and angels without noise.
Z_p=[(pos_VI_p_noisy)/1000;vel_noisy/1000;acc_P;gyro_P;magn_P];

%%
%plot(meas_time_P,magn_P(1:3,:),meas_time_P,quat_VI(1,:));legend('x','y','z','phi');

%% Counter
% A counter is calculated for the measurements form the vicon. This counter
% is needed, weather we have new senosor data

%to adjust gps reading frequency set vicon_freq to desired value! set it
%high to use every measurement available
%to set a gps outage set vicon_outage(1) to start of outage and
%vicon_outage(2) to end of outage
vicon_freq=10;
vicon_outage=[62 63];
j_old_p=1;
j_old_v=1;
counter_P_pv(1,1)=1;
counter_P_pv(2,1)=1;

for j=2:size(pos_VI_p,2)
    if (meas_time_P(j)<=vicon_outage(1) || meas_time_P(j)>=vicon_outage(2))
        if  meas_time_P(j)-meas_time_P(j_old_p)>=1/vicon_freq && (pos_VI_p(1,j) ~= pos_VI_p(1,j_old_p) || pos_VI_p(2,j) ~= pos_VI_p(2,j_old_p) || pos_VI_p(3,j) ~= pos_VI_p(3,j_old_p))
            counter_P_pv(1,j)=j;
            j_old_p=j;
        else counter_P_pv(1,j)=j_old_p;
        end
        if  meas_time_P(j)-meas_time_P(j_old_v)>=1/vicon_freq  && (vel(1,j) ~= vel(1,j_old_v) || vel(2,j) ~= vel(2,j_old_v) || vel(3,j) ~= vel(3,j_old_v)) ;
            counter_P_pv(2,j)=j;
            j_old_v=j;
        else counter_P_pv(2,j)=j_old_v;
        end
    else
       disp('outage');
       counter_P_pv(1,j)=j_old_p;
       counter_P_pv(2,j)=j_old_v;
    end
        
end



counter_P_new=[counter_P_pv;counter_P];

%plot(meas_time_VI+8.5646+0.6363,VI_new(2,:),meas_time_P,magn_P(3,:));
%%
%plot(meas_time_P,pos_VI_p(1,:),meas_time_P,acc_P(1,:),meas_time_P,acc_P(3,:)+1000,meas_time_P,magn_P(3,:),meas_time_P,acc_P(2,:));legend('position','accelerometerX','accelerometerZ','magnetometer','accY');
%plot(meas_time_P,pos_VI_p(1:3,:));legend('X','Y','Z');


%----------------------
%Building segments and running ekf
%----------------------
%% Building Segments
[segment1_Z_P,segment1_counter_P,segment1_time_P]=build_segment(57.8240, 149.7180, Z_p,counter_P_new,meas_time_P);
[segment1_ground_truth, segment1_time_ground_truth]=build_segment_ground_truth(57.8240, 149.7180,ground_truth,meas_time_P);

segment1_Z_P(1:3,:)=[segment1_Z_P(1,:)-0.13165*ones(1,size(segment1_Z_P,2));segment1_Z_P(2,:)-0.21*ones(1,size(segment1_Z_P,2));segment1_Z_P(3,:)-1.347185*ones(1,size(segment1_Z_P,2))];
segment1_ground_truth(1:3,:)=[segment1_ground_truth(1,:)-0.13165*ones(1,size(segment1_ground_truth,2));segment1_ground_truth(2,:)-0.21*ones(1,size(segment1_ground_truth,2));segment1_ground_truth(3,:)-1.347185*ones(1,size(segment1_ground_truth,2))];
%%
% diffed=diff(segment1_Z_P(1:3,:)');
% %%
% %plot(segment1_time_P,segment1_Z_P(1:3,:),segment1_time_P(1:size(diffed,1)),diffed');legend('X','Y','Z','dx','dy','dz');
% plot(segment1_time_P,10*segment1_Z_P(1:3,:),segment1_time_P,segment1_Z_P(7:9,:),segment1_time_P,segment1_Z_P(10:12,:),'.');legend('X','Y','Z','ax','ay','az','gx','gy','gz');
% %plot(segment4_time_P,10*segment4_Z_P(1:3,:),segment4_time_P,segment4_Z_P(7:9,:),segment4_time_P,segment4_Z_P(10:12,:),'.');legend('X','Y','Z','ax','ay','az','gx','gy','gz');

% %%
% [segment1_Z_X,segment1_counter_X,segment1_time_X]=build_segment(57.8240, 149.7180, Z_x,counter_X_new,meas_time_X);
% 
% [segment2_Z_P,segment2_counter_P,segment2_time_P]=build_segment(149.7180, 228.2, Z_p,counter_P_new,meas_time_P);
% [segment2_Z_X,segment2_counter_X,segment2_time_X]=build_segment(149.7180, 228.2, Z_x,counter_X_new,meas_time_X);
% %%
% [segment3_Z_P,segment3_counter_P,segment3_time_P]=build_segment(228.2, 314.6150, Z_p,counter_P_new,meas_time_P);
% %%
% [segment3_Z_X,segment3_counter_X,segment3_time_X]=build_segment(228.2, 314.6150, Z_x,counter_X_new,meas_time_X);
% %%
% [segment4_Z_P,segment4_counter_P,segment4_time_P]=build_segment(314.6150, 4.444950000000000e+02, Z_p,counter_P_new,meas_time_P);
% %%
% [segment4_Z_X,segment4_counter_X,segment4_time_X]=build_segment(314.6150, 4.444950000000000e+02, Z_x,counter_X_new,meas_time_X);
% 
% [segment5_Z_P,segment5_counter_P,segment5_time_P]=build_segment(4.444950000000000e+02, 580, Z_p,counter_P_new,meas_time_P);
% [segment5_Z_X,segment5_counter_X,segment5_time_X]=build_segment(4.444950000000000e+02, 580, Z_x,counter_X_new,meas_time_X);
% 

% 
% %% Running the kalman filter function
% [x_state,x_est]= state_estimation_3(segment1_Z_P,segment1_time_P,segment1_counter_P);

%%
meas_time=segment1_time_P;
M=segment1_Z_P;
counter=segment1_counter_P;