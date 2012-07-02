% deleting error data
function [time,acc,gyro,magn,pos,vel,counter] = delete_meas_error_ct(min_time,max_time,time_error,acc_error,gyro_error,magn_error,pos_error,vel_error,counter_error)

time1=time_error(time_error<max_time);
time=time1(time1>min_time);

for i=1:3
    acc1=acc_error(i,time_error<max_time);
    acc(i,:)=acc1(time1>min_time);

    gyro1=gyro_error(i,time_error<max_time);
    gyro(i,:)=gyro1(time1>min_time);

    magn1=magn_error(i,time_error<max_time);
    magn(i,:)=magn1(time1>min_time);
    
    counter1=counter_error(i,time_error<max_time);
    counter(i,:)=counter1(time1>min_time);
    
    pos1=pos_error(i,time_error<max_time);
    pos(i,:)=pos1(time1>min_time);
    
    vel1=vel_error(i,time_error<max_time);
    vel(i,:)=vel1(time1>min_time);
    
end



