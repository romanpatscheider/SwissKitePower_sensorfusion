%reading logfile of PxHawk IMU

function [gyro_DATA_P,acc_DATA_P,magn_DATA_P,press_DATA_P]= import_logfile(fid)


a=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f [%f] !%f!','headerLines',2);



%gyro
for i=1:4
    gyro_DATA_P(:,i)=a{i};
end
gyro_DATA_P(:,5)=a{16}+a{17}*1e-9;

%accelerometer
for i=5:8
    acc_DATA_P(:,i-4)=a{i};
end
acc_DATA_P(:,5)=a{16}+a{17}*1e-9;


%magnetometer
for i=9:12
    magn_DATA_P(:,i-8)=a{i};
end
magn_DATA_P(:,5)=a{16}+a{17}*1e-9;

%pressure
for i=13:15
    press_DATA_P(:,i-12)=a{i};
end
press_DATA_P(:,5)=a{16}+a{17}*1e-9;

fclose(fid);