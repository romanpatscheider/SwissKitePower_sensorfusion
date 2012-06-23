%centrifuge test plot

%% position
figure(1)
subplot(3,1,1);plot(meas_time_X(1,:),pos_X(1,:)*1e-7);title('GPS position');legend('North');xlabel('[s]');ylabel('[deg]');
subplot(3,1,2);plot(meas_time_X(1,:),pos_X(2,:)*1e-7);title('GPS position');legend('East');xlabel('[s]');ylabel('[deg]');
subplot(3,1,3);plot(meas_time_X(1,:),pos_X(3,:)*1e-3);title('GPS position');legend('Down');xlabel('[s]');ylabel('[m]');
%% velocity
figure(2)
subplot(3,1,1);plot(meas_time_X(1,:),vel_X(1,:)*1e-0);title('GPS velocity x-axis');legend('x');xlabel('[s]');ylabel('[m/s]');
subplot(3,1,2);plot(meas_time_X(1,:),vel_X(2,:)*1e-0);title('GPS velocity y-axis');legend('y');xlabel('[s]');ylabel('[m/s]');
subplot(3,1,3);plot(meas_time_X(1,:),vel_X(3,:)*1e-0);title('GPS velocity z-axis');legend('z');xlabel('[s]');ylabel('[m/s]');
%% accelerometer 
% PixHawk is restricted to 4 g, could be +/- 16 g; accP*9.81/2048
figure(3);
subplot(3,1,1);plot(meas_time_P(1,:),acc_P(1,:)*9.81/2048,PacketMagGyroAcc*t+60*60*13+19*60+44, AccelerometerX0x28g0x29*9.81,meas_time_X(1,:),(acc_X(1,:)-2^15)/(100*4));title('Accelerometer x-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[m/s^2]');
subplot(3,1,2);plot(meas_time_P(1,:),-acc_P(2,:)*9.81/2048,PacketMagGyroAcc*t+60*60*13+19*60+44, AccelerometerY0x28g0x29*9.81,meas_time_X(1,:),-(acc_X(2,:)-2^15)/(100*4));title('Accelerometer y-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[m/s^2]');
subplot(3,1,3);plot(meas_time_P(1,:),-acc_P(3,:)*9.81/2048,PacketMagGyroAcc*t+60*60*13+19*60+44, AccelerometerZ0x28g0x29*9.81,meas_time_X(1,:),(acc_X(3,:)-2^15)/(100*4));title('Accelerometer z-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[m/s^2]');

%% gyrometer
% gyro_P * 17.5/(1000)
figure(4);
subplot(3,1,1);plot(meas_time_P(1,:),gyro_P(1,:)*17.5/(1000),PacketMagGyroAcc*t+60*60*13+19*60+44, -GyroscopeX0x28deg0x2Fs0x29,meas_time_X(1,:),(gyro_X(1,:)-2^15)*5.7*0.889/(100*4));title('Gyroscope x-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[deg/s]');
subplot(3,1,2);plot(meas_time_P(1,:),-gyro_P(2,:)*17.5/(1000),PacketMagGyroAcc*t+60*60*13+19*60+44, -GyroscopeY0x28deg0x2Fs0x29,meas_time_X(1,:),(gyro_X(2,:)-2^15)*5.7*0.889/(100*4));title('Gyroscope y-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[deg/s]');
subplot(3,1,3);plot(meas_time_P(1,:),gyro_P(3,:)*17.5/(1000),PacketMagGyroAcc*t+60*60*13+19*60+44, -GyroscopeZ0x28deg0x2Fs0x29,meas_time_X(1,:),-(gyro_X(3,:)-2^15)*5.7*0.889/(100*4));title('Gyroscope z-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[deg/s]');

%% magnetometer
% 
figure(5);
subplot(3,1,1);plot(meas_time_P,magn_P(1,:)/1370+0.3781-0.07,PacketMagGyroAcc*t+60*60*13+19*60+44, MagnetometerX0x28G0x29-0.5,meas_time_X(1,:),(magn_X(1,:)-2^15)/15007.5);title('Magnetometer x-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[gauss]');
subplot(3,1,2);plot(meas_time_P,magn_P(2,:)/1370+0.3781-0.07-0.13,PacketMagGyroAcc*t+60*60*13+19*60+44, -MagnetometerY0x28G0x29-0.5,meas_time_X(1,:),-(magn_X(2,:)-2^15)/15007.5);title('Magnetometer x-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]');ylabel('[gauss]');
subplot(3,1,3);plot(meas_time_P,magn_P(3,:)/1370,PacketMagGyroAcc*t+60*60*13+19*60+44, -MagnetometerZ0x28G0x29+0.5,meas_time_X(1,:),-(magn_X(3,:)-2^15)/15007.5);title('Magnetometer x-axis');legend('PX4','x-IMU','MTi-G');xlabel('[s]','MTi-G');ylabel('[gauss]');

