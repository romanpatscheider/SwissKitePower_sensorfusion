% import pixhawk and calibrated xsens

% Importing Xsens
fid_xsens_cal = fopen('x_test_2_2.txt');
[pos_X_c,vel_X_c, acc_X_c,gyro_X_c,magn_X_c,meas_time_X_c,quat_X_c,counter_X_c,pos_counter_X_c] =import_Xsens_cal(fid_xsens_cal);
% Importing PixHawk
fid_pixhawk = fopen('p_test_2_2.txt');
[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid_pixhawk);
% Import vicon file
VI=csvread('positionlog_KiteBox_20.25.14.615.csv');

shift=0.51;
%------------------------
%bringing sensors and vicon together.
%------------------------
% deleting error measurements
[meas_time_X_c,acc_X_c,gyro_X_c,magn_X_c,counter_X_c]=delete_meas_error(0.8,450,meas_time_X_c,acc_X_c,gyro_X_c,magn_X_c,counter_X_c);
[meas_time_P,acc_P,gyro_P,magn_P,counter_P]=delete_meas_error(0.8,450,meas_time_P,acc_P,gyro_P,magn_P,counter_P);

%------------------------------------
% VI posiotion and velocity + PixHawk
%------------------------------------
dt_vicon=0.00100;
meas_time_VI=VI(:,1).*dt_vicon;
meas_time_VI_new=meas_time_VI';
VI_new=VI';

%position
for i=2:4
      pos_VI_p_noisy(i-1,:)=awgn(interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P),-5);
      pos_VI_p(i-1,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P);
end

% velocity
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
vel_p_noisy=awgn(vel,-5);

%angles
for i=5:7
     angles_VI_p(i-4,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_P);
end

% ground truth
ground_truth_P=[pos_VI_p/1000;vel/1000;angles_VI_p]; %position, velocity and angels without noise.
Z_p=[(pos_VI_p_noisy)/1000;vel_p_noisy/1000;acc_P;gyro_P;magn_P];


% Counter
counter_P_pv(1,1)=1;
for j=2:size(pos_VI_p,2)
    if pos_VI_p(1,j) == pos_VI_p(1,j-1) && pos_VI_p(2,j) == pos_VI_p(2,j-1) && pos_VI_p(3,j) == pos_VI_p(3,j-1);
        counter_P_pv(1,j)=j-1;
    else counter_P_pv(1,j)=j;
    end
end
counter_P_pv(2,1)=1;



for j=2:size(meas_time_P,2);
    
    if vel(1,j) == vel(1,j-1) && vel(2,j) == vel(2,j-1) && vel(3,j) == vel(3,j-1) ;
        counter_P_pv(2,j)=j-1;
    else counter_P_pv(2,j)=j;
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
for i=2:4
     pos_VI_x_noisy(i-1,:)=awgn(interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c),-5);
     pos_VI_x(i-1,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c);
end

%velocity
% velocity 
vel_tmp=zeros(3,size(meas_time_X_c,2)-1);
vel=zeros(3,size(meas_time_X_c,2));

for i=1:3
    vel_tmp(i,:)=diff(pos_VI_x(i,:))./diff(meas_time_X_c);
    
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
vel_x_noisy=awgn(vel,-5);

%angles
for i=5:7
     angles_VI_x(i-4,:)=interp1(meas_time_VI_new+8.5646+0.6363-0.07+shift,VI_new(i,:),meas_time_X_c);
end

% ground truth
ground_truth_X=[pos_VI_x/1000;vel/1000;angles_VI_x]; %position, velocity and angels without noise.
Z_x=[(pos_VI_x_noisy)/1000;vel_x_noisy/1000;acc_X_c;gyro_X_c;magn_X_c].*[1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));-1*ones(1,size(pos_VI_x_noisy,2));1*ones(1,size(pos_VI_x_noisy,2))];


% counter
counter_X_pv(1,1)=1;
for j=2:size(pos_VI_x,2)
    if pos_VI_x(1,j) == pos_VI_x(1,j-1) && pos_VI_x(2,j) == pos_VI_x(2,j-1) && pos_VI_x(3,j) == pos_VI_x(3,j-1);
        counter_X_pv(1,j)=j-1;
    else counter_X_pv(1,j)=j;
    end
end
counter_X_pv(2,1)=1;

for j=2:size(meas_time_X_c,2);
    
    if vel(1,j) == vel(1,j-1) && vel(2,j) == vel(2,j-1) && vel(3,j) == vel(3,j-1) ;
        counter_X_pv(2,j)=j-1;
    else counter_X_pv(2,j)=j;
    end
end

counter_X_new=[counter_X_pv;counter_X_c];

[segment1_Z_P,segment1_counter_P,segment1_time_P]=build_segment(47.1620, 158.7230, Z_p,counter_P_new,meas_time_P);
[segment1_Z_X,segment1_counter_X,segment1_time_X]=build_segment(47.1620, 158.7230, Z_x,counter_X_new,meas_time_X_c);

[segment1_ground_truth_P, segment1_time_ground_truth_P]=build_segment_ground_truth(47.1620, 158.7230,ground_truth_P,meas_time_P);
[segment1_ground_truth_X, segment1_time_ground_truth_X]=build_segment_ground_truth(47.1620, 158.7230,ground_truth_X,meas_time_X_c);

segment1_Z_P(1:3,:)=[segment1_Z_P(1,:)-0.13165*ones(1,size(segment1_Z_P,2));segment1_Z_P(2,:)-0.21*ones(1,size(segment1_Z_P,2));segment1_Z_P(3,:)-1.347185*ones(1,size(segment1_Z_P,2))];
segment1_ground_truth_P(1:3,:)=[segment1_ground_truth_P(1,:)-0.13165*ones(1,size(segment1_ground_truth_P,2));segment1_ground_truth_P(2,:)-0.21*ones(1,size(segment1_ground_truth_P,2));segment1_ground_truth_P(3,:)-1.347185*ones(1,size(segment1_ground_truth_P,2))];

segment1_Z_X(1:3,:)=[segment1_Z_X(1,:)-0.13165*ones(1,size(segment1_Z_X,2));segment1_Z_X(2,:)-0.21*ones(1,size(segment1_Z_X,2));segment1_Z_X(3,:)-1.347185*ones(1,size(segment1_Z_X,2))];
segment1_ground_truth_X(1:3,:)=[segment1_ground_truth_X(1,:)-0.13165*ones(1,size(segment1_ground_truth_X,2));segment1_ground_truth_X(2,:)-0.21*ones(1,size(segment1_ground_truth_X,2));segment1_ground_truth_X(3,:)-1.347185*ones(1,size(segment1_ground_truth_X,2))];

%%
plot(segment1_time_P,segment1_Z_P(3,:)*10+8,segment1_time_P,segment1_Z_P(9,:)+10,segment1_time_P,segment1_Z_P(12,:))*10;legend('pos z','acc z','gyro')

%%
meas_time=segment1_time_P;
M=segment1_Z_P;
counter=segment1_counter_P;
%%
meas_time=segment1_time_X;
M=segment1_Z_X;
counter=segment1_counter_X;