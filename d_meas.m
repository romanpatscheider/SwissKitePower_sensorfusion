function [meas_control_vector]=d_meas(counter,counter_old,length_z)
for i=1:length_z
    if counter((i-1)*3+1)==counter_old((i-1)*3+1)
        meas_control_vector(i)=0;
    else
        meas_control_vector(i)=1;
    end
end
     