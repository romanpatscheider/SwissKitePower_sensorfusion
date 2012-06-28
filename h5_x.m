function [z_est]=h5(x_est,t)

%--------------------------
% the expected measurements are calculated. 
% Attention: h6 is only used for compairison to ground truth. no correction
% is based on h6.
%--------------------------

    R=0.8465;
    dR=0;
    G= [0;0;9.85]; % [m/s^2] gravitation in zurich
    dis=[0.0;0;0];%displacementvector of the box
    MAG1=0.2145;% [Gauss] magnetic field in zurich
    MAG2=0.0060;
    MAG3=0.4268;
    
%     % magnetic field as measured by magnetometer in resting position. Angle from vicon. 
%     true_mag=[-0.119784172661871;-0.0330935251798561;0.341007194244604;];
%     mag_angle=-1.496;
    true_mag=[-0.0375785801027502;-0.449114660422007;0.823831720011575;];
    mag_angle=2.417-pi/2-0.4;
    mag=[cos(mag_angle),sin(mag_angle),0;-sin(mag_angle),cos(mag_angle),0;0,0,1]*true_mag;
%     
%calculate matrix to convert changes in euler angles to rotations in the
%body frame
deuler2body=calc_deuler2body(x_est(1),x_est(2),x_est(3));
w_body=[0,0,1;0,-1,0;-1,0,0]*deuler2body*x_est(5:7);

% calculate DCM to get from inertial to body frame
DCM_bi=[0,0,-1;0,1,0;-1,0,0]*calc_DCM_br(x_est(1),x_est(2),x_est(3));


vg=transp(cross(transp(w_body),transp(dis)));

%not used at the moment...
% pos_n=x_est(4)*[cos(x_est(3))*cos(x_est(2));...
%         sin(x_est(3))*cos(x_est(2));...
%         -sin(x_est(2))];
% dpos_n=[cos(x_est(2))*cos(x_est(3))*x_est(8)-sin(x_est(2))*cos(x_est(3))*x_est(4)*x_est(6)-cos(x_est(2))*sin(x_est(3))*x_est(4)*x_est(7);...
%         cos(x_est(2))*sin(x_est(3))*x_est(8)-sin(x_est(2))*sin(x_est(3))*x_est(4)*x_est(6)+cos(x_est(2))*cos(x_est(3))*x_est(4)*x_est(7);...
%         -sin(x_est(2))*x_est(8)-cos(x_est(2))*x_est(4)*x_est(6)];

%calculate position and velocity
pos_n=R*[cos(x_est(3))*cos(x_est(2));...
        sin(x_est(3))*cos(x_est(2));...
        -sin(x_est(2))];

dpos_n=[cos(x_est(2))*cos(x_est(3))*dR-sin(x_est(2))*cos(x_est(3))*R*x_est(6)-cos(x_est(2))*sin(x_est(3))*R*x_est(7);...
        cos(x_est(2))*sin(x_est(3))*dR-sin(x_est(2))*sin(x_est(3))*R*x_est(6)+cos(x_est(2))*cos(x_est(3))*R*x_est(7);...
        -sin(x_est(2))*dR-cos(x_est(2))*R*x_est(6)];

h1=pos_n + transp(DCM_bi)*dis; %pos
h2=dpos_n + transp(DCM_bi)*vg; %vel

% calculate expected acceleration
a_zp=-[0;0;norm((dpos_n))^2/(R)];
a_zp_dis=-transp(cross(transp(w_body),transp(cross(transp(w_body),transp(dis)))));
a_G=DCM_bi*(-G).*[0;0;1.05];
h3=a_zp+a_zp_dis+a_G;

h4=w_body;%gyr
% h5=DCM_bi*mag.*[1;-1;1]+[-0.2;-0.2;0];%magnetometer
h5=DCM_bi*mag;
% calculate cardan angles from staes (not used for correction! -> complex
% atan2 warning may be ignored
DCM_b2i_n=calc_DCM_br(real(x_est(1)),real(x_est(2)),real(x_est(3)));
psi=-0.2;
thet=0.07;
    DCM_br_n=[cos(psi),sin(psi),0;-sin(psi),cos(psi),0;0,0,1]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0,0,-1;0,1,0;1,0,0]*DCM_b2i_n;

cardan_n=[-atan2(-DCM_br_n(3,2),-DCM_br_n(3,3))  ...
              asin(-DCM_br_n(3,1)) ...
              atan2(DCM_br_n(2,1),DCM_br_n(1,1))];

h6=cardan_n';
z_est=[h1;h2;h3;h4;h5;h6];
