%import Xsens data; creating virtual counter =[acc, gyro, magn]'
function [acc_X,gyro_X,magn_X,pos_X,vel_X,meas_time_X,counter_X] =import_Xsens(fid)


a=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','headerLines',2);

%time in [s]
meas_time_X=a{24}*1e-3;
%accelerometer
for i=1:3
    acc_X(:,i)=a{i};
end
%gyro
for i=4:6
    gyro_X(:,i-3)=a{i};
end
%magnetometer
for i=7:9
    magn_X(:,i-6)=a{i};
end
%position
for i=14:16
    pos_X(:,i-13)=a{i};
end
%velocity
for i=17:19
    vel_X(:,i-16)=a{i};
end
%counter
counter_X(1,1)=1;
for j=2:size(acc_X,1)
    if acc_X(j,1) == acc_X(j-1,1) && acc_X(j,2) == acc_X(j-1,2) && acc_X(j,3) == acc_X(j-1,3);
        counter_X(j,1)=j-1;
    else counter_X(j,1)=j;
    end
end

counter_X(1,2)=1;
for j=2:size(gyro_X,1)
    if gyro_X(j,1) == gyro_X(j-1,1) && gyro_X(j,2) == gyro_X(j-1,2) && gyro_X(j,3) == gyro_X(j-1,3);
        counter_X(j,2)=j-1;
    else counter_X(j,2)=j;
    end
end

counter_X(1,3)=1;
for j=2:size(magn_X,1)
    if magn_X(j,1) == magn_X(j-1,1) && magn_X(j,2) == magn_X(j-1,2) && magn_X(j,3) == magn_X(j-1,3);
        counter_X(j,3)=j-1;
    else counter_X(j,3)=j;
    end
end

acc_X=acc_X';
gyro_X=gyro_X';
magn_X=magn_X';
counter_X=counter_X';


