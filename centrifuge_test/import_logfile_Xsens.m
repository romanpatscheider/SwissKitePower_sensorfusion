%import_logfile_Xsens
function [gyro_DATA_X,acc_DATA_X,press_DATA_X,temp_DATA_X,pos_DATA_X,vel_DATA_X,magn_DATA_X] =import_logfile_Xsens(fid)


a=textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','headerLines',2);



%gyro
for i=4:6
    gyro_DATA_X(:,i-3)=a{i};
end
gyro_DATA_X(:,4)=a{24}*1e-3;

%accelerometer
for i=1:3
    acc_DATA_X(:,i)=a{i};
end
acc_DATA_X(:,4)=a{24}*1e-3;%time


%magnetometer
for i=7:9
    magn_DATA_X(:,i-6)=a{i};
end
magn_DATA_X(:,4)=a{24}*1e-3;

%pressure
press_DATA_X(:,1)=a{11};
press_DATA_X(:,2)=a{24}*1e-9;

%temperature
temp_DATA_X(:,1)=a{10};
temp_DATA_X(:,2)=a{24}*1e-9;

%bPrs
%temp_bPrs_X(:,1)=a{12};
%temp_bPrs_X(:,2)=a{24}*1e-9;

%position
for i=14:16
    pos_DATA_X(:,i-13)=a{i};
end
pos_DATA_X(:,4)=a{24}*1e-3;

%velocity
for i=17:19
    vel_DATA_X(:,i-16)=a{i};
end
vel_DATA_X(:,4)=a{24}*1e-3;
