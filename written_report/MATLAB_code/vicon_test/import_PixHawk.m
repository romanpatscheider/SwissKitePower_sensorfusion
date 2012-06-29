% import pixHawk data; counter=[acc,gyro,magn]'
% the data is brought in the correct units and saved in matrixes according
% the sensor
function [acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid)


a=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f [%f] !%f!','headerLines',2);

%time
meas_time_P=a{16}+a{17}*1e-9;
%gyro
for i=1:4
    gyro_P(:,i)=a{i}/20300*2*pi;
end
%accelerometer
for i=5:8
    acc_P(:,i-4)=a{i}/200;
end
%magnetometer
for i=9:12
    magn_P(:,i-8)=a{i}/1390;
end
%counter
counter_P(:,1)=a{8};
counter_P(:,2)=a{4};
counter_P(:,3)=a{12};

meas_time_P=meas_time_P';
gyro_P=gyro_P';
acc_P=acc_P';
magn_P=magn_P';
counter_P=counter_P';