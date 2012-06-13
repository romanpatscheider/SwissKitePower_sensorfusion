function [z_est]=h(x_est,x_old,DCM_ir,t,dis,G,mag)

    
    MAG1=0.2145;% [Gauss] magnetic field in zurich
    MAG2=0.0060;
    MAG3=0.4268;
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
    
DCM_br_est=calc_DCM_br(x_est(7),x_est(8),x_est(9));
DCM_bi=DCM_br_est*transp(DCM_ir);


w=[d_phi;0;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[0;d_thet;0] ...
    +[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]...
    *[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0;0;d_psi];
w_old=[d_phi_old;0;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[0;d_thet_old;0] ...
    +[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]...
    *[cos(thet_old), 0, -sin(thet_old);0,1,0;sin(thet_old),0,cos(thet_old)]*[0;0;d_psi_old];
vg=transp(cross(transp(w),transp(dis)));
vg_old=transp(cross(transp(w_old),transp(dis)));
v=[vn;ve;vd];
v_old=[vn_old;ve_old;vd_old];

h1=[lat;long;alt] + transp(DCM_bi)*dis; %pos
h2=v + transp(DCM_bi)*vg; %vel


h3=DCM_bi*(((v+transp(DCM_bi)*vg)-(v_old+transp(DCM_bi)*vg_old))./t-G);%acc
h4=w;%gyr
h5=DCM_bi*[MAG1;MAG2;MAG3];%magnetometer

z_est=[h1;h2;h3;h4;h5];
