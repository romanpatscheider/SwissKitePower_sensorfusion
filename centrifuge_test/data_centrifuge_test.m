% daten centrifuge test
fid1 = fopen('p_test_1.txt');
fid2 = fopen('x_test_1.txt');

[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid1);
[pos_X,vel_X,acc_X,gyro_X,magn_X,meas_time_X,counter_X_IMU,counter_X_pos] =import_Xsens_ct(fid2);

counter_X=[counter_X_IMU;counter_X_pos];

% Daten?bertragungsrate xIMU, siehe Registereintrag xIMU:
f = 32; %[Hz]

%Vornummerierung der CSV-Datei, siehe Nummer der zu ladenden Files:
x = '00001';

%CSV-Datei importieren
IMPORTFILE([x '_DateTime.csv']);
PacketTime = PacketNumber;
IMPORTFILE([x '_CalBattAndTherm.csv']);
PacketBattTherm = PacketNumber;
IMPORTFILE([x '_CalInertialAndMag.csv']);
PacketMagGyroAcc = PacketNumber;
IMPORTFILE([x '_EulerAngles.csv']);
PacketEulerAngles = PacketNumber;

clear PacketNumber;

deltaPacket = PacketTime(3) - PacketTime(2);
t =  1/(f*deltaPacket);

%deleting measurements
[meas_time_X,acc_X,gyro_X,magn_X,pos_X,vel_X,counter_X]=delete_meas_error_ct(47946,49190,meas_time_X,acc_X,gyro_X,magn_X,pos_X,vel_X,counter_X);
[meas_time_P,acc_P,gyro_P,magn_P,counter_P]=delete_meas_error(47946,49190,meas_time_P,acc_P,gyro_P,magn_P,counter_P);