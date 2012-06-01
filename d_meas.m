function [meas_control_vector]=d_meas(counter,counter_old,length_z)
for i=1:5
    if counter(i)==counter_old(i)
        meas_control_vector(3*i-2)=0;
        meas_control_vector(3*i-1)=0;
        meas_control_vector(3*i)=0;
    else
        meas_control_vector(3*i-2)=1;
        meas_control_vector(3*i-1)=1;
        meas_control_vector(3*i)=1;
    end
end
     