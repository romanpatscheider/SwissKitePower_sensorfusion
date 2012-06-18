fid1 = fopen('p_test_1.txt');
[gyro_DATA_P,acc_DATA_P,magn_DATA_P,press_DATA_P]=import_logfile(fid1);


fid2 = fopen('x_test_1.txt');
[gyro_DATA_X,acc_DATA_X,press_DATA_X,temp_DATA_X,pos_DATA_X,vel_DATA_X,magn_DATA_X]=import_logfile_Xsens(fid2);


%%
start_time=47946;
end_time=49190;


for i=acc_DATA_P(:,5)<start_time
    acc_DATA_P(i,:)=[];
end
for i=gyro_DATA_P(:,5)<start_time
    gyro_DATA_P(i,:)=[];
end
for i=magn_DATA_P(:,5)<start_time
    magn_DATA_P(i,:)=[];
end
for i=press_DATA_P(:,5)<start_time
    press_DATA_P(i,:)=[];
end

for i=acc_DATA_X(:,4)<start_time
    acc_DATA_X(i,:)=[];
end
for i=gyro_DATA_X(:,4)<start_time
    gyro_DATA_X(i,:)=[];
end
for i=magn_DATA_X(:,4)<start_time
    magn_DATA_X(i,:)=[];
end
% for i=press_DATA_X(:,4)<start_time
%     press_DATA_X(i,:)=[];
% end
% for i=temp_DATA_X(:,4)<start_time
%     temp_DATA_X(i,:)=[];
% end
for i=pos_DATA_X(:,4)<start_time
    pos_DATA_X(i,:)=[];
end
for i=vel_DATA_X(:,4)<start_time
    vel_DATA_X(i,:)=[];
end




for i=acc_DATA_P(:,5)>end_time
    acc_DATA_P(i,:)=[];
end
for i=gyro_DATA_P(:,5)>end_time
    gyro_DATA_P(i,:)=[];
end
for i=magn_DATA_P(:,5)>end_time
    magn_DATA_P(i,:)=[];
end
for i=press_DATA_P(:,5)>end_time
    press_DATA_P(i,:)=[];
end

for i=acc_DATA_X(:,4)>end_time
    acc_DATA_X(i,:)=[];
end
for i=gyro_DATA_X(:,4)>end_time
    gyro_DATA_X(i,:)=[];
end
for i=magn_DATA_X(:,4)>end_time
    magn_DATA_X(i,:)=[];
end
% for i=press_DATA_X(:,4)>end_time
%     press_DATA_X(i,:)=[];
% end
% for i=temp_DATA_X(:,4)>end_time
%     temp_DATA_X(i,:)=[];
% end
for i=pos_DATA_X(:,4)>end_time
    pos_DATA_X(i,:)=[];
end
for i=vel_DATA_X(:,4)>end_time
    vel_DATA_X(i,:)=[];
end

% pos_DATA_X(:,1)=pos_DATA_X(:,1)-mean(pos_DATA_X(:,1));
% pos_DATA_X(:,2)=pos_DATA_X(:,2)-mean(pos_DATA_X(:,2));
% pos_DATA_X(:,3)=pos_DATA_X(:,3)-mean(pos_DATA_X(:,3));
%%
figure(1);
set(1,'name','Xsens','numbertitle','off');
subplot(6,1,1);plot(pos_DATA_X(:,4),pos_DATA_X(:,1:2));title('position');legend('N','E');
subplot(6,1,2);plot(pos_DATA_X(:,4),pos_DATA_X(:,3));title('altitude');legend('D');
subplot(6,1,3);plot(vel_DATA_X(:,4),vel_DATA_X(:,1:3));title('velocity');legend('x','y','z');
subplot(6,1,4);plot(acc_DATA_X(:,4),acc_DATA_X(:,1:3));title('acc');legend('x','y','z');
subplot(6,1,5);plot(gyro_DATA_X(:,4),gyro_DATA_X(:,1:3));title('gyr');legend('x','y','z');
subplot(6,1,6);plot(magn_DATA_X(:,4),magn_DATA_X(:,1:3));title('mag');legend('x','y','z');


%%
% gyro_DATA_P(1,:)=[];
% magn_DATA_P(1,:)=[];
% acc_DATA_P(1,:)=[];
figure(2)
set(2,'name','PxHawk','numbertitle','off');
subplot(3,1,1);plot(acc_DATA_P(:,5),acc_DATA_P(:,1:3));title('acc');legend('x','y','z');
subplot(3,1,2);plot(gyro_DATA_P(:,5),gyro_DATA_P(:,1:3));title('gyr');legend('x','y','z');
subplot(3,1,3);plot(magn_DATA_P(:,5),magn_DATA_P(:,1:3));title('mag');legend('x','y','z');

%%
figure(3)
set(3,'name','comparison','numbertitle','off');

subplot(3,1,1);plot(acc_DATA_P(:,5),acc_DATA_P(:,1),acc_DATA_X(:,4),acc_DATA_X(:,1)-2^15);title('acc x');legend('P','X');
subplot(3,1,2);plot(gyro_DATA_P(:,5),gyro_DATA_P(:,1),gyro_DATA_X(:,4),gyro_DATA_X(:,1)-2^15);title('gyro x');legend('P','X');
subplot(3,1,3);plot(magn_DATA_P(:,5),magn_DATA_P(:,1),magn_DATA_X(:,4),magn_DATA_X(:,1)-2^15);title('magn x');legend('P','X');

%%
% fclose(fid1);
% fclose(fid2);