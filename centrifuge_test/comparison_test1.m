
eval('DatenaufbereitungUndPlot_xIMU');
%eval('read_out');

%% accelerometer
figure(10);
set(10,'name','Xsens vs PxHawk vs X-IMU','numbertitle','off');
subplot(5,1,1);plot(acc_DATA_X(:,4),(acc_DATA_X(:,1)-2^15)/(100*4),acc_DATA_P(:,5),acc_DATA_P(:,1)/200,PacketMagGyroAcc*t+60*60*13+19*60+44, AccelerometerX0x28g0x29*9.81);title('acc');legend('Xsens[m/s^2]','PxHawk[m/s^2]','Ximu[m/s^2]');
ylabel('[m/s^2]');xlabel('time of the day [s]');
% GPS time: time of the day in winter time in UTC!
% ximu:h=15;m=19;s=44 in summer time.

 %% gyrometer
 subplot(5,1,2);plot(gyro_DATA_X(:,4),(gyro_DATA_X(:,3)-2^15)*(-1)/27900,gyro_DATA_P(:,5),gyro_DATA_P(:,3)*0.0174532925/2000,PacketMagGyroAcc*t+60*60*13+19*60+44, GyroscopeZ0x28deg0x2Fs0x29/(-360));title('gyro');legend('Xsens','PxHawk','Ximu');
 ylabel('[turns/s]');xlabel('time of the day [s]');
 %% magnetometer
 subplot(5,1,3);plot(magn_DATA_X(:,4),(magn_DATA_X(:,3)-2^15)*(-1),magn_DATA_P(:,5),magn_DATA_P(:,3),PacketMagGyroAcc*t+60*60*13+19*60+44, MagnetometerZ0x28G0x29);title('magnetometer');legend('Xsens','PxHawk','Ximu');
 ylabel('[???]');xlabel('time of the day [s]');
 
 %% GPS
subplot(5,1,4);plot(pos_DATA_X(:,4),pos_DATA_X(:,1:2)*1e-7);title('position');legend('N','E');
ylabel('[deg]');xlabel('time of the day [s]');
subplot(5,1,5);plot(pos_DATA_X(:,4),pos_DATA_X(:,3)*1e-3);title('altitude');legend('D');
ylabel('[m]');xlabel('time of the day [s]');

%% magn. PixHawk
figure(11);
set(11,'name','PixHawk','numbertitle','off');
plot(magn_DATA_P(:,5),magn_DATA_P(:,3)/1370);

figure(12);
set(12,'name','gyro','numbertitle','off');
plot(gyro_DATA_P(:,5),gyro_DATA_P(:,3)/(1000*8.75)/360*2*pi,PacketMagGyroAcc*t+60*60*13+19*60+44, GyroscopeZ0x28deg0x2Fs0x29*-0.0174532925/131);legend('pixHawk','X-IMU')