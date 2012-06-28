function [z_est]=h_euler_x(x_est,x_old,t,dis,G,mag)

    
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
    vn_old=x_old(4);
    ve_old=x_old(5);
    vd_old=x_old(6);
    phi_old=x_old(7);
    thet_old=x_old(8);
    psi_old=x_old(9);
    d_phi_old=x_old(10);
    d_thet_old=x_old(11);
    d_psi_old=x_old(12);
    
    true_mag=[-0.0375785801027502;-0.449114660422007;0.823831720011575;];
    mag_angle=2.417-pi/2-0.4;
    mag=[cos(mag_angle),sin(mag_angle),0;-sin(mag_angle),cos(mag_angle),0;0,0,1]*true_mag;
    
DCM_bi=[-1 0 0;0 1 0; 0 0 1]*calc_DCM_br(x_est(7),x_est(8),x_est(9));
deuler2body=calc_deuler2body(x_est(7),x_est(8),x_est(9));
w=[-1 0 0;0 -1 0; 0 0 -1]*deuler2body*x_est(10:12);
deuler2body_old=calc_deuler2body(x_old(7),x_old(8),x_old(9));
w_old=[-1 0 0;0 -1 0; 0 0 1]*deuler2body_old*x_old(10:12);

%vg=transp(cross(transp(w),transp(dis))); %vg and vg_old are additional velocitys introduced by the displacement of the Imu from the center of mass
%vg_old=transp(cross(transp(w_old),transp(dis)));
vg=[0;0;0];
v=[vn;ve;vd];
v_old=[vn_old;ve_old;vd_old];

h1=[lat;long;alt] + transp(DCM_bi)*dis; %pos
h2=v + transp(DCM_bi)*vg; %vel
h3=DCM_bi*(-(v-v_old)./t-G);
%h3=DCM_bi*(((v+transp(DCM_bi)*vg)-(v_old+transp(DCM_bi)*vg_old))./t-G);%acc
h4=w;%gyr
h5=DCM_bi*mag;%magnetometer

z_est=[h1;h2;h3;h4;h5];
