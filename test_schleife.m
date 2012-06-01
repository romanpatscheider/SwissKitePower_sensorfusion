% test while schleife
totalTime=meas_time(1);
i=1;
k=1;
while (i<size(M,2))
    totalTime=totalTime+t
    % values for the symbols for the x state
    lat = x(1);
    long= x(2);
    alt = x(3);
    vn  = x(4);
    ve  = x(5);
    vd  = x(6);
    phi = x(7);
    thet = x(8);
    pssi = x(9);
    d_phi  = x(10);
    d_thet  = x(11);
    d_psi  = x(12);
    bax = x(13);
    bay = x(14);
    baz = x(15);
    bgx = x(16);
    bgy = x(17);
    bgz = x(18);
    bmx = x(19);
    bmy = x(20);
    bmz = x(21);
    q1  = q(1);
    q2  = q(2);
    q3  = q(3);
    q4  = q(4);
    A = eval(tmp_A);
    
    %estimation step
    [x_est,P_est]=estimation(A,P,Q,x);
    
    if(meas_time(i)>totalTime)
        disp('no new value within t')
        x_new=x_est;
        P=P_est;
        
    else
        % the values are now from the estimated x
    lat = x_est(1);
    long= x_est(2);
    alt = x_est(3);
    vn  = x_est(4);
    ve  = x_est(5);
    vd  = x_est(6);
    phi = x_est(7);
    thet = x_est(8);
    pssi = x_est(9);
    d_phi  = x_est(10);
    d_thet  = x_est(11);
    d_psi  = x_est(12);
    bax = x_est(13);
    bay = x_est(14);
    baz = x_est(15);
    bgx = x_est(16);
    bgy = x_est(17);
    bgz = x_est(18);
    bmx = x_est(19);
    bmy = x_est(20);
    bmz = x_est(21);
    vn_old=x(4);
    ve_old=x(5);
    vd_old=x(6);
    phi_old=x(7);
    thet_old=x(8);
    psi_old=x(9);
    d_phi_old=x(10);
    d_thet_old=x(11);
    d_psi_old=x(12);
    H = eval(tmp_H);
    
    %correction step
    if i==1
        i_old=10;
    else
        i_old =i-1;
    end
    while(meas_time(i+1)<=totalTime)
        i=i+1;
    end
    z_new=M(:,i);
    counter_new=counter(:,i);
    counter_old=counter(:,i_old);
    length_z=size(z_new);
   
    meas_control= d_meas(counter_new,counter_old,length_z);
    
    x_tmp=x_est;
    P_tmp=P_est;
    for j=1:size(meas_control,2)                  % if we have new data, correction step is done
        if meas_control(j)==1
            [x_tmp,P_tmp]= correction(P_tmp,H(j,:),R(j,j),z_new(j),x_tmp);
            
            
        end
            
    end
    x_new=x_tmp;
    P=P_tmp;
    end
    
    
    %reset
    phi = x_new(7);
    thet= x_new(8);
    pssi = x_new(9);
    
    DCM=eval(DCM_bi');
    q=DCMtoQ(DCM);
    deuler2body=calc_deuler2body(x_new(7),x_new(8),x_new(9));
    
    
    x_new(7)=0;
    x_new(8)=0;
    x_new(9)=0;
    x_new(10:12)=DCM*deuler2body*x_new(10:12);
    
    
    x=x_new;
    save(:,k)=x_new;
    save_est(:,k)=x_est;
    k=k+1;
    
end



