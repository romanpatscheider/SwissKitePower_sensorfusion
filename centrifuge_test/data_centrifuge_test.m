% daten centrifuge test
fid1 = fopen('p_test_1.txt');
fid2 = fopen('x_test_1.txt');

[acc_P,gyro_P,magn_P,meas_time_P,counter_P]= import_PixHawk(fid1);
[acc_X,gyro_X,magn_X,meas_time_X,counter_X,pos_counter_X] =import_Xsens(fid2);



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

deltaPacket = PacketTime(3) - PacketTime(2)
t =  1/(f*deltaPacket);