%free mass model

function [x_est]= free_mass_model(x,t)

lat=x(1);long=x(2);alt=x(3);
vn=x(4);ve=x(5);vd=x(6);
phi=x(7);thet=x(8);pssi=x(9);
d_phi=x(10);d_thet=x(11);d_psi=x(12);



f1=[lat+t*vn;long+t*ve;alt+t*vd]; 
f2=[vn;ve;vd];
f3=[phi+t*d_phi;thet+t*d_thet;pssi+t*d_psi];
f4=[d_phi;d_thet;d_psi];

x_est=[f1;f2;f3;f4;x(13:21)];