%daten f?r free mass modnel
% Running the kalman filter for the vicon test

%---------------------------------------------
%Import DATA
%---------------------------------------------
%% Importing PixHawk
fid_pixhawk = fopen('p_test_2_1.txt');
[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid_pixhawk);
% Import vicon file
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

for i=2:4
      pos_VI_p(i-1,:)=interp1(meas_time_VI_new+42.622+0.5100+0.49,VI_new(i,:),meas_time_P);
end

vel_tmp=zeros(3,size(meas_time_P,2)-1);

size(vel_tmp)

vel=zeros(3,size(meas_time_P,2));


for i=1:3
    vel_tmp(i,:)=diff(pos_VI_p(i,:))./diff(meas_time_P);
    
end
vel=[vel_tmp(:,1),vel_tmp(:,:)];

Z_p=[(pos_VI_p)/1000;vel/1000;acc_P;gyro_P;magn_P];

counter_P_pv(1,1)=1;
for j=2:size(pos_VI_p,2)
    if pos_VI_p(1,j) == pos_VI_p(1,j-1) && pos_VI_p(2,j) == pos_VI_p(2,j-1) && pos_VI_p(3,j) == pos_VI_p(3,j-1);
        counter_P_pv(1,j)=j-1;
    else counter_P_pv(1,j)=j;
    end
end
counter_P_pv(2,1)=1;
for j=2:size(meas_time_P);
    if vel(1,j) == vel(1,j-1) && vel(2,j) == vel(2,j-1) && vel(3,j) == vel(3,j-1);
        counter_P_pv(2,j)=j-1;
    else counter_P_pv(2,j)=j;
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


segment1_Z_P(1:3,:)=[segment1_Z_P(1,:)-0.13165*ones(1,size(segment1_Z_P,2));segment1_Z_P(2,:)-0.21*ones(1,size(segment1_Z_P,2));segment1_Z_P(3,:)-1.347185*ones(1,size(segment1_Z_P,2))];

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