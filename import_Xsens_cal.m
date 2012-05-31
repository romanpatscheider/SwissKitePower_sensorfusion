% import Xsens calibrated data

function [pos_X_c,vel_X_c, acc_X_c,gyro_X_c,magn_X_c,meas_time_X_c,quat_X_c,counter_X_IMU_c,counter_X_pos_c]= import_Xsens_cal(fid)

a1=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','headerLines',2);

%time in [s]
meas_time_X_c=a1{20}*1e-3;
%accelerometer
for i=1:3
    acc_X_c(:,i)=(typecast(int32(a1{i}),'single'));
end
%gyro
for i=4:6
    gyro_X_c(:,i-3)=(typecast(int32(a1{i}),'single'));
end
%magnetometer
for i=7:9
    magn_X_c(:,i-6)=(typecast(int32(a1{i}),'single'));
end
%position
for i=14:16
    pos_X_c(:,i-13)=(typecast(int32(a1{i}),'single'));
end
%velocity
for i=17:19
    vel_X_c(:,i-16)=(typecast(int32(a1{i}),'single'));
end
%quaternion
for i=10:13
    quat_X_c(:,i-9)=(typecast(int32(a1{i}),'single'));
end

%counter
counter_X_c(1,1)=1;
for j=2:size(pos_X_c,1)
    if pos_X_c(j,1) == pos_X_c(j-1,1) && pos_X_c(j,2) == pos_X_c(j-1,2) && pos_X_c(j,3) == pos_X_c(j-1,3);
        counter_X_c(j,1)=j-1;
    else counter_X_c(j,1)=j;
    end
end
counter_X_c(1,2)=1;
for j=2:size(vel_X_c,1)
    if vel_X_c(j,1) == vel_X_c(j-1,1) && vel_X_c(j,2) == vel_X_c(j-1,2) && vel_X_c(j,3) == vel_X_c(j-1,3);
        counter_X_c(j,2)=j-1;
    else counter_X_c(j,2)=j;
    end
end
counter_X_c(1,3)=1;
for j=2:size(acc_X_c,1)
    if acc_X_c(j,1) == acc_X_c(j-1,1) && acc_X_c(j,2) == acc_X_c(j-1,2) && acc_X_c(j,3) == acc_X_c(j-1,3);
        counter_X_c(j,3)=j-1;
    else counter_X_c(j,3)=j;
    end
end

counter_X_c(1,4)=1;
for j=2:size(gyro_X_c,1)
    if gyro_X_c(j,1) == gyro_X_c(j-1,1) && gyro_X_c(j,2) == gyro_X_c(j-1,2) && gyro_X_c(j,3) == gyro_X_c(j-1,3)
        counter_X_c(j,4)=j-1;
    else counter_X_c(j,4)=j;
    end
end

counter_X_c(1,5)=1;
for j=2:size(magn_X_c,1)
    if magn_X_c(j,1) == magn_X_c(j-1,1) && magn_X_c(j,2) == magn_X_c(j-1,2) && magn_X_c(j,3) == magn_X_c(j-1,3)
        counter_X_c(j,5)=j-1;
    else counter_X_c(j,5)=j;
    end
end

counter_X_IMU_c=counter_X_c(:,3:5);
counter_X_pos_c=counter_X_c(:,1:2);

acc_X_c=acc_X_c';
gyro_X_c=gyro_X_c';
magn_X_c=magn_X_c';
counter_X_IMU_c=counter_X_IMU_c';
counter_X_pos_c=counter_X_pos_c';
meas_time_X_c=meas_time_X_c';
