function [q]= DCMtoQ(DCM)
qw= sqrt(1 + DCM(1,1) + DCM(2,2) + DCM(3,3)) /2;
qx = (DCM(3,2) - DCM(2,3))/( 4 *qw);
qy = (DCM(1,3) - DCM(3,1))/( 4 *qw);
qz = (DCM(2,1) - DCM(1,2))/( 4 *qw);
q=[qw;qx;qy;qz];