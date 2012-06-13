function [z_est]=h5(x_est,t)
    
    R=0.8465;
    dR=0;
    G= [0;0;9.85]; % [m/s^2] gravitation in zurich
    dis=[0;0;0];%displacementvector of the box
    MAG1=0.2145;% [Gauss] magnetic field in zurich
    MAG2=0.0060;
    MAG3=0.4268;
    true_mag=[-0.119784172661871;-0.0330935251798561;0.341007194244604;];
    mag_angle=1.496;
    mag=[cos(mag_angle),sin(mag_angle),0;-sin(mag_angle),cos(mag_angle),0;0,0,1]*true_mag;
    
deuler2body=calc_deuler2body(x_est(1),x_est(2),x_est(3));
w_body=[0,0,1;0,1,0;-1,0,0]*deuler2body*x_est(5:7);

    
DCM_bi=[0,0,1;0,1,0;-1,0,0]*calc_DCM_br(x_est(1),x_est(2),x_est(3));

vg=transp(cross(transp(w_body),transp(dis)));


% pos_n=x_est(4)*[cos(x_est(3))*cos(x_est(2));...
%         sin(x_est(3))*cos(x_est(2));...
%         -sin(x_est(2))];
% dpos_n=[cos(x_est(2))*cos(x_est(3))*x_est(8)-sin(x_est(2))*cos(x_est(3))*x_est(4)*x_est(6)-cos(x_est(2))*sin(x_est(3))*x_est(4)*x_est(7);...
%         cos(x_est(2))*sin(x_est(3))*x_est(8)-sin(x_est(2))*sin(x_est(3))*x_est(4)*x_est(6)+cos(x_est(2))*cos(x_est(3))*x_est(4)*x_est(7);...
%         -sin(x_est(2))*x_est(8)-cos(x_est(2))*x_est(4)*x_est(6)];

pos_n=R*[cos(x_est(3))*cos(x_est(2));...
        sin(x_est(3))*cos(x_est(2));...
        -sin(x_est(2))];

dpos_n=[cos(x_est(2))*cos(x_est(3))*dR-sin(x_est(2))*cos(x_est(3))*R*x_est(6)-cos(x_est(2))*sin(x_est(3))*R*x_est(7);...
        cos(x_est(2))*sin(x_est(3))*dR-sin(x_est(2))*sin(x_est(3))*R*x_est(6)+cos(x_est(2))*cos(x_est(3))*R*x_est(7);...
        -sin(x_est(2))*dR-cos(x_est(2))*R*x_est(6)];

h1=pos_n + transp(DCM_bi)*dis; %pos
h2=dpos_n + transp(DCM_bi)*vg; %vel


a_zp=-[0;0;norm((dpos_n))^2/(R)];
a_zp_dis=-transp(cross(transp(w_body),transp(cross(transp(w_body),transp(dis)))));
a_G=DCM_bi*(-G).*[0;0;1];
h3=a_zp+a_zp_dis+a_G;
%h3=DCM_bi*(((v+DCM_bi'*vg)-(v_old+DCM_bi'*vg_old))./t-G);%acc
h4=w_body;%gyr
h5=DCM_bi*mag;%magnetometer

DCM_b2i_n=calc_DCM_br(x_est(1),x_est(2),x_est(3));
    DCM_br_n=[0,0,1;0,1,0;-1,0,0]*DCM_b2i_n;

cardan_n=[atan2(DCM_br_n(2,3),DCM_br_n(3,3)) ...
              asin(-DCM_br_n(1,3)) ...
              atan2(DCM_br_n(1,2),DCM_br_n(1,1))];
h6=cardan_n';
z_est=[h1;h2;h3;h4;h5;h6];
