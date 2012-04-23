clear all;

dt=0.1;
m=1;
g=9.81;
%pos=[x_i,y_i,z_i];
%dpos=[vn,ve,vd];
pos=[1,0,0];
dpos=[0,0,-1];
pause on;





for i=1:1000
    
    rad=norm(pos);
    rotvec=vrrotvec(pos/rad,[0,0,1]);
    beta=rotvec(4);
    rotvec=vrrotvec([pos(1),pos(2),0]/norm(pos(1:2)),[1,0,0]);
    alpha=rotvec(4);

    dir1=cross([0,0,1],pos);
    dir1=dir1/norm(dir1);
    dir2=cross(dir1,pos);
    dir2=dir2/norm(dir2);
    alpha_n=alpha+dt*dot(dpos,dir1)/norm(pos(1:2));
    beta_n=beta+dt*dot(dpos,dir2)/rad;
    pos_n=[rad*cos(alpha_n)*sin(beta_n),rad*sin(alpha_n)*sin(beta_n),rad*cos(beta_n)];
    dpos_n=dpos+dt*(g*[1,0,0]-(norm(dpos)/norm(pos))^2*pos);
    pos=pos_n;
    dpos=dpos_n;
    
    timeseries(i,:)=pos;
    scatter3(timeseries(:,1),timeseries(:,2),timeseries(:,3),'filled');
    pause;
end

scatter3(timeseries(:,1),timeseries(:,2),timeseries(:,3),'filled');
