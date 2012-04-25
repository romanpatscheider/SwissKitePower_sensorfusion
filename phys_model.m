clear all;

dt=0.01;
m=1;
g=9.81;
%pos=[x_i,y_i,z_i];
%dpos=[vn,ve,vd];
pos=[1,0,0];
dpos=[0,-.5,0];
pause on;





for i=1:1000
    
    rad=norm(pos);
    
    if norm(pos(1:2))/rad<.05
        dpos_n=[dpos(1:2),0];
        pos_n=pos+dt*dpos;
        disp('vertical')
    else
        
        rotvecb=vrrotvec(pos/rad,[0,0,1]);
        beta=rotvecb(4)
        rotvec=vrrotvec([pos(1),pos(2),0]/norm(pos(1:2)),[1,0,0]);
        if rotvec(3)>0
            alpha=rotvec(4)
        else 
            alpha=2*pi-rotvec(4)
        end
        dir1=cross([0,0,1],pos);
        dir1=dir1/norm(dir1);
        dir2=cross(dir1,pos);
        dir2=dir2/norm(dir2);
        
        dalpha=dot(dpos,dir1)/norm(pos(1:2));
        dbeta=dot(dpos,dir2)/rad;
    
        beta_n=beta+dt*dbeta
        alpha_n=alpha+dt*dalpha;
    
    
        pos_n=[rad*cos(alpha_n)*sin(beta_n),rad*sin(alpha_n)*sin(beta_n),rad*cos(beta_n)];
    
        dpos_n=dpos+dt*(g*[0,0,1]-(norm(dpos)/norm(pos))^2*pos-pos*(g*cos(beta)));
    end
    
    
   
    pos=pos_n;
    dpos=dpos_n;
    timeseries(i,:)=pos;
    scatter3(timeseries(:,1),timeseries(:,2),timeseries(:,3),'filled');
    pause;
end

scatter3(timeseries(:,1),timeseries(:,2),timeseries(:,3),'filled');
