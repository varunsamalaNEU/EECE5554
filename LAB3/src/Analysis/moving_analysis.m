clear 
clf

% Importing csv file for plotting
data = readtable('moving_data.csv');
Time = table2array(data(:,1));

% Assigning columns to be read
qx= table2array(data(:,10));
qy= table2array(data(:,11));
qz= table2array(data(:,12));
qw= table2array(data(:,13));

% Angular Velocities
gyrox = table2array(data(:,15));
gyroy = table2array(data(:,16));
gyroz = table2array(data(:,17));
combinegyro = [gyrox,gyroy,gyroz];

% Create a 2x2 subplot
figure(1)
subplot(3,1,1)
plot(Time,gyrox);
title('angular velocity x vs time')
%xlabel('time (seconds)')
ylabel('angular velocity (rad/sec)')

subplot(3,1,2)
plot(Time,gyroy);
title('angular velocity y vs time')
%xlabel('time (seconds)')
ylabel('angular velocity (rad/sec)')

subplot(3,1,3)
plot(Time,gyroz);
title('angular velocity z vs time')
xlabel('time (seconds)')
ylabel('angular velocity (rad/sec)')

set(gcf,'Position',[100 100 1000 800])

%Linear acclerations
accx = table2array(data(:,19));
accy = table2array(data(:,20));
accz = table2array(data(:,21));
combineacc = [accx,accy,accz];

% mean and stddev
medain_value = median(accx)
mean_value = mean(accx)
std_deviation = std(accx)

medain_value = median(accy)
mean_value = mean(accy)
std_deviation = std(accy)

medain_value = median(accz)
mean_value = mean(accz)
std_deviation = std(accz)

% Magnetometer
magx = table2array(data(:,27));
magy = table2array(data(:,28));
magz = table2array(data(:,29));
combinemag = [magx,magy,magz];
Time = table2array(data(:,1));

figure(8)
subplot(3,1,1)
plot(Time,magx);
title('Magnetic field x vs time')
%xlabel('time (seconds)')
ylabel('Gauss')

subplot(3,1,2)
plot(Time, magy);
title('Magnetic field y vs time')
%xlabel('time (seconds)')
ylabel('Gauss')

subplot(3,1,3)
plot(Time,magz);
title('Magnetic field z vs time')
xlabel('time (seconds)')
ylabel('Gauss')

figure(9)
subplot(3,1,1)
plot(Time,accx);
title('linear acceleration x vs time')
%xlabel('time (seconds)')
ylabel('linear acceleration (m/s^2)')

subplot(3,1,2)
plot(Time, accy);
title('linear acceleration y vs time')
%xlabel('time (seconds)')
ylabel('linear acceleration (m/s^2)')

subplot(3,1,3)
plot(Time,accz);
title('linear acceleration z vs time')
xlabel('time (seconds)')
ylabel('linear acceleration (m/s^2)')


figure(12)
histogram(accx);
title('Histogram for linear acceleration x')
xlabel('linear accleration x (m/s^2)')
ylabel('frequency')


%Euler Angles
qt = [qw,qx,qy,qz];
eulXYZ = quat2eul(qt, "XYZ");
eulXYZ = rad2deg(eulXYZ); 

figure(15)
subplot(3,1,1)
plot(Time,eulXYZ(:,1));
title('Euler x vs time')
%xlabel('time (seconds)')
ylabel('angle (degrees)')

subplot(3,1,2)
plot(Time, eulXYZ(:,2));
title('Euler y vs time')
%xlabel('time (seconds)')
ylabel('angle (degrees)')

subplot(3,1,3)
plot(Time,eulXYZ(:,3));
title('Euler z vs time')
xlabel('time (seconds)')
ylabel('angle (degrees)')

sgtitle('Euler Angles', 'FontWeight','bold')


