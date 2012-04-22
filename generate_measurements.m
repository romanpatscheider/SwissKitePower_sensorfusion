t=linspace(0,100,10001);
R=2;
omega=0.5;
phi0=0;
declination=degtorad(1+35/60); %E+ W-
inclination=degtorad(63+19/60); %Down+ Up-
mag=47773; %nT

gpsNoise=0.01;
accNoise=0.1;
gyrNoise=0.1;
magNoise=5000;

F=21;
[b,g] = sgolay(4,F);
halfWin=((F+1)/2)-1;
meas=zeros(15,size(t,2));

Rl= geocradius(47+24/60); %from zurich
Rp= Rl*cos(8+32/60); %from zurich

for i=1:size(t,2)
    meas(1,i)=(R*cos(omega*t(i)))/Rl+47+24/60;                % lat (in deg!!)
    meas(2,i)=(R*sin(omega*t(i)))/Rp+8+32/60;                % lon (in deg!!)
    meas(3,i)=0;                                % alt [m]
    
    meas(1:3,i)=meas(1:3,i)+gpsNoise*[1/Rl;1/Rp;1].*rand(3,1); %add noise
    
    if i<F                                     % calculate velocity: 
        meas(4:6,i)=[0;0;0];
    else
        meas(4,i-halfWin) =   dot(g(:,2), (meas(1,i - 2*halfWin: i)-(47+24/60)).*ones(1,2*halfWin+1)*Rl);
        meas(5,i-halfWin) =   dot(g(:,2), (meas(2,i - 2*halfWin: i)-(8+32/60)).*ones(1,2*halfWin+1)*Rp);
        meas(6,i-halfWin) =   dot(g(:,2), meas(3,i - 2*halfWin: i ));
        %meas(4,i)=(meas(1,i)-meas(1,i-1))/(t(i)-t(i-1));
        %meas(5,i)=(meas(2,i)-meas(2,i-1))/(t(i)-t(i-1));
        %meas(6,i)=(meas(3,i)-meas(3,i-1))/(t(i)-t(i-1));
    end
    
    meas(7,i)=omega^2*R;                        % acc X
    meas(8,i)=0;                                % acc Y
    meas(9,i)=0;                                % acc Z
    
    meas(7:9,i)=meas(7:9,i)+accNoise*rand(3,1); %add noise
    
    meas(10,i)=0;                               % gyr X
    meas(11,i)=0;                               % gyr Y
    meas(12,i)=omega;                           % gyr Z
    
    meas(10:12,i)=meas(10:12,i)+gyrNoise*rand(3,1); %add noise
    
    meas(13,i)=mag*cos(omega*t(i)+declination)*cos(inclination) ;                               % mag X
    meas(14,i)=mag*sin(omega*t(i)+declination)*cos(inclination);                               % mag Y
    meas(15,i)=mag*sin(inclination);                           % mag Z
    
    meas(13:15,i)=meas(13:15,i)+magNoise*rand(3,1); %add noise    
    
    
end
subplot(6,1,1);plot(t,meas(1:2,:)-[47+24/60;8+32/60]*ones(1,size(t,2)));title('position');legend('N','E');
subplot(6,1,2);plot(t,meas(3,:));title('altitude');legend('D');
subplot(6,1,3);plot(t,meas(4:6,:));title('velocity');legend('x','y','z');
subplot(6,1,4);plot(t,meas(7:9,:));title('acc');legend('x','y','z');
subplot(6,1,5);plot(t,meas(10:12,:));title('gyr');legend('x','y','z');
subplot(6,1,6);plot(t,meas(13:15,:));title('mag');legend('x','y','z');
    