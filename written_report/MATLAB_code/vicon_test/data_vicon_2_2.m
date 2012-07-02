% import pixhawk and calibrated xsens and vicon for the long radius
% pendulum set up

% Importing Xsens
fid_xsens_cal = fopen('x_test_2_2.txt');
[pos_X_c,vel_X_c, acc_X_c,gyro_X_c,magn_X_c,meas_time_X_c,quat_X_c,counter_X_c,pos_counter_X_c] =import_Xsens_cal(fid_xsens_cal);
% Importing PixHawk
fid_pixhawk = fopen('p_test_2_2.txt');
[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid_pixhawk);
% Import vicon file
VI=csvread('positionlog_KiteBox_20.25.14.615.csv');

shift=0.51-0.02;
%------------------------
%bringing sensors and vicon together.
%------------------------
% deleting error measurements: measurements wit incorrect time stap get
% deleted. for the algorithm the time has to be constantely increasing
[meas_time_X_c,acc_X_c,gyro_X_c,magn_X_c,counter_X_c]=delete_meas_error(0.8,450,meas_time_X_c,acc_X_c,gyro_X_c,magn_X_c,counter_X_c);
[meas_time_P,acc_P,gyro_P,magn_P,counter_P]=delete_meas_error(0.8,450,meas_time_P,acc_P,gyro_P,magn_P,counter_P);

%------------------------------------
% VI posiotion and velocity + PixHawk
%------------------------------------
dt_vicon=0.00100;
meas_time_VI=VI(:,1).*dt_vicon;
meas_time_VI_new=meas_time_VI';
VI_new=VI';

% position
% the poition from vicon is interpolated on the data of the PX4 measurement and added with noise
% the time is also shifted in order the time form vicon and PX4 match
for i=2:4
      pos_VI_p_noisy(i-1,:)=awgn(interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P),-25);
      pos_VI_p(i-1,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P);
end


%meas_time_VI_gt=meas_time_VI_new+8.5646+0.6363-0.07+shift;
%VI_gt=[VI_new(2,:)/1000-0.13165*ones(1,size(VI_new,2));VI_new(3,:)/1000-0.21*ones(1,size(VI_new,2));VI_new(4,:)/1000-1.347185*ones(1,size(VI_new,2));VI_new(5:7,:)];




% velocity
% calculation of the velocity from the position measurement of the vicon
% the velocity is added with noise
vel_P_tmp=zeros(3,size(meas_time_P,2)-1);
vel_P=zeros(3,size(meas_time_P,2));
for i=1:3
    vel_P_tmp(i,:)=diff(pos_VI_p(i,:))./diff(meas_time_P);
    
end
vel_P=([vel_P_tmp(:,1),vel_P_tmp(:,:)]);
% if there is no velocity the counter is constant. In the algorithm is this
% interpreted as sensor outage of the velocity
for j=2:size(meas_time_P,2);

    if isnan(vel_P(1,j))
        vel_P(1,j)=vel_P(1,j-1);
    end
    if isnan(vel_P(2,j))
        vel_P(2,j)=vel_P(2,j-1);
    end
    if isnan(vel_P(3,j))  
       vel_P(3,j)=vel_P(3,j-1);
    end
end
vel_p_noisy=awgn(vel_P,-25);

%angles
for i=5:7
     angles_VI_p(i-4,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P);
end

% ground truth is saved in a vector to be able to compare the results.
ground_truth_P=[pos_VI_p/1000;vel_P/1000;angles_VI_p]; %position, velocity and angels without noise.
Z_p=[(pos_VI_p_noisy)/1000;vel_p_noisy/1000;acc_P;gyro_P;magn_P]; % This matrix contains all PX4 measurements in the right units


%%
%plot(meas_time_P,pos_VI_p,'-',meas_time_P,pos_VI_p_noisy,'.');
%% Counter
% A counter is calculated for the measurements form the vicon. This counter
% is needed, weather we have new senosor data

%to adjust gps reading frequency set vicon_freq to desired value! set it
%high to use every measurement available
%to set a gps outage set vicon_outage(1) to start of outage and
%vicon_outage(2) to end of outage
vicon_freq=10;
vicon_outage=[0 0];
j_old_p=1;
j_old_v=1;
counter_P_pv(1,1)=1;
counter_P_pv(2,1)=1;

for j=2:size(pos_VI_p,2)
    if (meas_time_P(j)<=vicon_outage(1) || meas_time_P(j)>=vicon_outage(2))
        if (meas_time_P(j)<=47.6 && meas_time_P(j)>=47) || (meas_time_P(j)-meas_time_P(j_old_p)>=1/vicon_freq && (pos_VI_p(1,j) ~= pos_VI_p(1,j_old_p) || pos_VI_p(2,j) ~= pos_VI_p(2,j_old_p) || pos_VI_p(3,j) ~= pos_VI_p(3,j_old_p)))
            counter_P_pv(1,j)=j;
            j_old_p=j;
        else counter_P_pv(1,j)=j_old_p;
        end
        if (meas_time_P(j)<=47.6 && meas_time_P(j)>=47) || meas_time_P(j)-meas_time_P(j_old_v)>=1/vicon_freq  && ((vel_P(1,j) ~= vel_P(1,j_old_v) || vel_P(2,j) ~= vel_P(2,j_old_v) || vel_P(3,j) ~= vel_P(3,j_old_v))) ;
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


%---------------------------------
% VI position and velocity + Xsens
%---------------------------------

dt_vicon=0.00100;
meas_time_VI=VI(:,1).*dt_vicon;
meas_time_VI=meas_time_VI';

% position
% the poition from vicon is interpolated on the data of the MTi-G measurement and added with noise
% the time is also shifted in order the time form vicon and MTi-G match
for i=2:4
     pos_VI_x_noisy(i-1,:)=awgn(interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c),-5);
     pos_VI_x(i-1,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c);
end

%velocity
% calculation of the velocity from the position measurement of the vicon
% the velocity is added with noise
vel_X_tmp=zeros(3,size(meas_time_X_c,2)-1);
vel_X=zeros(3,size(meas_time_X_c,2));

for i=1:3
    vel_X_tmp(i,:)=diff(pos_VI_x(i,:))./diff(meas_time_X_c);
    
end
vel_X=([vel_X_tmp(:,1),vel_X_tmp(:,:)]);
% if there is no velocity the counter is constant. In the algorithm is this
% interpreted as sensor outage of the velocity
for j=2:size(meas_time_P,2);

    if isnan(vel_X(1,j))
        vel_X(1,j)=vel_X(1,j-1);
    end
    if isnan(vel_X(2,j))
        vel_X(2,j)=vel_X(2,j-1);
    end
    if isnan(vel_X(3,j))  
       vel_X(3,j)=vel_X(3,j-1);
    end
end
vel_x_noisy=awgn(vel_X,-5);

%angles
for i=5:7
     angles_VI_x(i-4,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c);
end

% ground truth
ground_truth_X=[pos_VI_x/1000;vel_X/1000;angles_VI_x]; %position, velocity and angels without noise.
Z_x=[(pos_VI_x_noisy)/1000;vel_x_noisy/1000;acc_X_c;gyro_X_c;magn_X_c].*[1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2))];

%% Counter
% A counter is calculated for the measurements form the vicon. This counter
% is needed, weather we have new senosor data

%to adjust gps reading frequency set vicon_freq to desired value! set it
%high to use every measurement available
%to set a gps outage set vicon_outage(1) to start of outage and
%vicon_outage(2) to end of outage
vicon_freq=10;
vicon_outage=[0 0];
j_old_p=1;
j_old_v=1;
counter_X_pv(1,1)=1;
counter_X_pv(2,1)=1;

for j=2:size(pos_VI_x,2)
    if (meas_time_X_c(j)<=vicon_outage(1) || meas_time_X_c(j)>=vicon_outage(2))
        if (meas_time_X_c(j)<=47.6 && meas_time_X_c(j)>=47) || (meas_time_X_c(j)-meas_time_X_c(j_old_p)>=1/vicon_freq && (pos_VI_x(1,j) ~= pos_VI_x(1,j_old_p) || pos_VI_x(2,j) ~= pos_VI_x(2,j_old_p) || pos_VI_x(3,j) ~= pos_VI_x(3,j_old_p)))
            counter_X_pv(1,j)=j;
            j_old_p=j;
        else counter_X_pv(1,j)=j_old_p;
        end
        if (meas_time_X_c(j)<=47.6 && meas_time_X_c(j)>=47) || (meas_time_X_c(j)-meas_time_X_c(1)<=1 || meas_time_X_c(j)-meas_time_X_c(j_old_v)>=1/vicon_freq  && (vel_X(1,j) ~= vel_X(1,j_old_v) || vel_X(2,j) ~= vel_X(2,j_old_v) || vel_X(3,j) ~= vel_X(3,j_old_v))) ;
            counter_X_pv(2,j)=j;
            j_old_v=j;
        else counter_X_pv(2,j)=j_old_v;
        end
    else
       disp('outage');
       counter_X_pv(1,j)=j_old_p;
       counter_X_pv(2,j)=j_old_v;
    end
        
end



counter_X_new=[counter_X_pv;counter_X_c];


%% correct the pixhawk magnetometer.... (don't ask me why...)
Z_p(13:15,:)=2*Z_p(13:15,:)-[-0.689*ones(size(meas_time_P));-0.341*ones(size(meas_time_P));-0.16*ones(size(meas_time_P))];

%%

% The next lines cut the whole file into pieces of interest.
[segment1_Z_P,segment1_counter_P,segment1_time_P]=build_segment(47.1620, 158.7230, Z_p,counter_P_new,meas_time_P);
[segment1_Z_X,segment1_counter_X,segment1_time_X]=build_segment(47.1620, 158.7230, Z_x,counter_X_new,meas_time_X_c);

[segment1_ground_truth_P, segment1_time_ground_truth_P]=build_segment_ground_truth(47.1620, 158.7230,ground_truth_P,meas_time_P);
[segment1_ground_truth_X, segment1_time_ground_truth_X]=build_segment_ground_truth(47.1620, 158.7230,ground_truth_X,meas_time_X_c);

% We have to put the (0,0,0) point of the vicon system and the one of the
% propagation model together.
segment1_Z_P(1:3,:)=[segment1_Z_P(1,:)-0.13165*ones(1,size(segment1_Z_P,2));segment1_Z_P(2,:)-0.21*ones(1,size(segment1_Z_P,2));segment1_Z_P(3,:)-1.347185*ones(1,size(segment1_Z_P,2))];
segment1_ground_truth_P(1:3,:)=[segment1_ground_truth_P(1,:)-0.13165*ones(1,size(segment1_ground_truth_P,2));segment1_ground_truth_P(2,:)-0.21*ones(1,size(segment1_ground_truth_P,2));segment1_ground_truth_P(3,:)-1.347185*ones(1,size(segment1_ground_truth_P,2))];

segment1_Z_X(1:3,:)=[segment1_Z_X(1,:)-0.13165*ones(1,size(segment1_Z_X,2));segment1_Z_X(2,:)-0.21*ones(1,size(segment1_Z_X,2));segment1_Z_X(3,:)-1.347185*ones(1,size(segment1_Z_X,2))];
segment1_ground_truth_X(1:3,:)=[segment1_ground_truth_X(1,:)-0.13165*ones(1,size(segment1_ground_truth_X,2));segment1_ground_truth_X(2,:)-0.21*ones(1,size(segment1_ground_truth_X,2));segment1_ground_truth_X(3,:)-1.347185*ones(1,size(segment1_ground_truth_X,2))];


%% run this cell to run the estimator afterwards based on the Pix Hawk measurements
meas_time=segment1_time_P;
M=segment1_Z_P;
counter=segment1_counter_P;
%% run this cell to run the estimator afterwards based on the Xsens measurements
meas_time=segment1_time_X;
M=segment1_Z_X;
counter=segment1_counter_X;