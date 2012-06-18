% *************************************************************************
% DatenaufbereitungUndPlot_xIMU.m
% *************************************************************************
% Autor: Raphael M?ller
% Datum: 27.03.2012
% *************************************************************************
% 
% Bemerkung:
% ----------
% Alle Sensoren m?ssen die gleiche Taktfrequenz zur
% Datenaufzeichnung aufweisen!
%
% *************************************************************************

% 
% clear all;
% clc;
% close all;

% Daten?bertragungsrate xIMU, siehe Registereintrag xIMU:
f = 32; %[Hz]

%Vornummerierung der CSV-Datei, siehe Nummer der zu ladenden Files:
x = '00001';

%CSV-Datei importieren
IMPORTFILE([x '_DateTime.csv'])
PacketTime = PacketNumber;
IMPORTFILE([x '_CalBattAndTherm.csv'])
PacketBattTherm = PacketNumber;
IMPORTFILE([x '_CalInertialAndMag.csv'])
PacketMagGyroAcc = PacketNumber;
IMPORTFILE([x '_EulerAngles.csv'])
PacketEulerAngles = PacketNumber;

clear PacketNumber;

%Zeit pro Packet definieren: (deltaPacket = 1/f)
deltaPacket = PacketTime(3) - PacketTime(2)
t =  1/(f*deltaPacket);

%StartZeit ausgeben
figure;
title(['Starttime  = Year: ' int2str(Year(1)) ' Month: ' int2str(Month(1)) ...
    ' Day: ' int2str(Day(1)) ' Hours: ' int2str(Hours(1)) ' Minutes: ' ...
    int2str(Minutes(1)) ' Seconds: ' int2str(Seconds(1))]);

%Plotte Batterie-Spannung und Thermometer
figure;
subplot(2,1,1);
plot(PacketBattTherm*t,  BatteryVoltage0x28V0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Spannung [V]');
grid on;
title('Batterie-Spannung und Thermometer', 'FontSize',14);

subplot(2,1,2);
plot(PacketBattTherm*t, Thermometer0x28degreesC0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Temperatur [?C]');
grid on;;



%% Plotte Accelerometer, Gyroscope, Magnetometer
figure;
subplot(3,1,1);
plot(PacketMagGyroAcc*t, AccelerometerX0x28g0x29, PacketMagGyroAcc*t, ...
    AccelerometerY0x28g0x29, PacketMagGyroAcc*t, AccelerometerZ0x28g0x29);legend('X','Y','Z')
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Beschleunigung [g] (Rechne: g*9.81 [m/s^2])');
grid on;
title('Accelerometer, Gyroscope, Magnetometer', 'FontSize',14);

subplot(3,1,2);
plot(PacketMagGyroAcc*t, GyroscopeX0x28deg0x2Fs0x29, PacketMagGyroAcc*t, ...
    GyroscopeY0x28deg0x2Fs0x29, PacketMagGyroAcc*t, GyroscopeZ0x28deg0x2Fs0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Gyroscope [?/s]');
grid on;

subplot(3,1,3);
plot(PacketMagGyroAcc*t, MagnetometerX0x28G0x29, PacketMagGyroAcc*t, ...
    MagnetometerY0x28G0x29, PacketMagGyroAcc*t, MagnetometerZ0x28G0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Magnetometer [Gauss]  (1 Gauss = 0.1 mT)');
grid on;


%Plotte EulerAngles: Roll, Pitch, Yaw
figure;
subplot(3,1,1);
plot(PacketEulerAngles*t, Roll0x7CPhi0x7CX0x28degrees0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Roll [?]');
grid on;
title('Roll, Pitch, Yaw (EulerAngles-Approx)', 'FontSize',14);

subplot(3,1,2);
plot(PacketEulerAngles*t, Pitch0x7CTheta0x7CY0x28degrees0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Pitch [?]');
grid on;

subplot(3,1,3);
plot(PacketEulerAngles*t, Yaw0x7CPsi0x7CZ0x28degrees0x29);
set(gca, 'FontSize', 10);
xlabel('t [s]'); ylabel('Yaw [?]');
grid on;
