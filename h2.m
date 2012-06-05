function [z_est]=h(x_est,x_old,DCM_bi,t,dis,G,mag)

    
    MAG1=0.2145;% [Gauss] magnetic field in zurich
    MAG2=0.0060;
    MAG3=0.4268;
   

deuler2body=calc_deuler2body(x_est(7),x_est(8),x_est(9));
w_body=deuler2body*x_est(10:12);


%w=[d_phi;0;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[0;d_thet;0]+[1,0,0;0,cos(phi),sin(phi);0,-sin(phi),cos(phi)]*[cos(thet), 0, -sin(thet);0,1,0;sin(thet),0,cos(thet)]*[0;0;d_psi];
%w_old=[d_phi_old;0;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[0;d_thet_old;0]+[1,0,0;0,cos(phi_old),sin(phi_old);0,-sin(phi_old),cos(phi_old)]*[cos(thet_old), 0, -sin(thet_old);0,1,0;sin(thet_old),0,cos(thet_old)]*[0;0;d_psi_old];
vg=(cross(w_body',dis'))';
%vg_old=(cross(w_old',dis'))';
%v=[vn;ve;vd];
%v_old=[vn_old;ve_old;vd_old];

h1=x_est(1:3) + DCM_bi'*dis; %pos
h2=x_est(4:6) + DCM_bi'*vg; %vel
h3=-[0;0;norm((x_est(4:6)))^2/(norm(x_est(1:3)))]-(cross(w_body',(cross(w_body',dis'))))'+DCM_bi*(-G);
%h3=DCM_bi*(((v+DCM_bi'*vg)-(v_old+DCM_bi'*vg_old))./t-G);%acc
h4=w_body;%gyr
h5=DCM_bi*[MAG1;MAG2;MAG3];%magnetometer

z_est=[h1;h2;h3;h4;h5];
