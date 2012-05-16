function [meas_control_vector]=d_meas(z_new,z_old)
for i=1:size(z_new)
    if abs(z_new(i)-z_old(i)<eps)
        meas_control_vector(i)=0;
    else
        meas_control_vetor(i)=1;
    end
end
     