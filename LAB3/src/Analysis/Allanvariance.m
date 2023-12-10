
clc
clear all
clf

bag = rosbag("LocationC.bag");
bSel = select(bag, 'Topic', '/vectornav');
msgStructs = readMessages(bSel, 'DataFormat', 'struct');

gyroData = zeros(length(msgStructs), 3);
accData = zeros(length(msgStructs), 3);
timestamps = zeros(length(msgStructs), 1);

for i = 1:length(msgStructs)
   msg = msgStructs{i}.Data;
   if contains(msg, '$VNYMR')
       C = strsplit(msg, ',');
        if length(C) >= 13
           D = strsplit(C{13}, '*');
           gyroData(i,:) = [str2double(C{11}), str2double(C{12}), str2double(D{1})];
           accData(i,:) = [str2double(C{8}), str2double(C{9}), str2double(C{10})];
           timestamps(i) = msgStructs{i}.Header.Stamp.Sec + msgStructs{i}.Header.Stamp.Nsec*1e-9;
        end
   end
end

timestamps = timestamps - timestamps(1);
figure(1);
plot(timestamps, gyroData(:,1), 'r', 'LineWidth', 2); hold on;
plot(timestamps, gyroData(:,2), 'g', 'LineWidth', 2); hold on;
plot(timestamps, gyroData(:,3), 'b', 'LineWidth', 2); hold off;
xlabel('Time (s)');
ylabel('Gyro (rad/s)');
legend('Gyro X', 'Gyro Y', 'Gyro Z');
title("Location B")

t0 = 1/40;
thetax = cumsum(gyroData(:, 1))*t0;
maxNumMx = 100;
Lx = size(thetax, 1);
maxMx = 2^floor(log2(Lx/2));
mx = logspace(log10(1), log10(maxMx), maxNumMx)';
mx = ceil(mx); 
mx = unique(mx);
taux = mx*t0;
avarx = zeros(numel(mx), 1);
for i = 1:numel(mx)
    mi = mx(i);
    avarx(i) = sum(...
        (thetax(1+2*mi:Lx) - 2*thetax(1+mi:Lx-mi) + thetax(1:Lx-2*mi)).^2, 1);
end
avarx = avarx ./ (2*taux.^2 .* (Lx - 2*mx));
adevx = sqrt(avarx);

thetay = cumsum(gyroData(:, 2))*t0;
maxNumMy = 100;
Ly = size(thetay, 1);
maxMy = 2^floor(log2(Ly/2));
my = logspace(log10(1), log10(maxMy), maxNumMy)';
my = ceil(my); 
my = unique(my); 
tauy = my*t0;
avary = zeros(numel(my), 1);
for i = 1:numel(my)
    mi = my(i);
    avary(i) = sum(...
        (thetay(1+2*mi:Ly) - 2*thetay(1+mi:Ly-mi) + thetay(1:Ly-2*mi)).^2, 1);
end
avary = avary ./ (2*tauy.^2 .* (Ly - 2*my));
adevy = sqrt(avary);

thetaz = cumsum(gyroData(:, 3))*t0;
maxNumMz = 100;
Lz = size(thetaz, 1);
maxMz = 2^floor(log2(Lz/2));
mz = logspace(log10(1), log10(maxMz), maxNumMz)';
mz = ceil(mz); 
mz = unique(mz); 
tauz = mz*t0;
avarz = zeros(numel(mz), 1);
for i = 1:numel(mz)
    mi = mz(i);
    avarz(i) = sum(...
        (thetaz(1+2*mi:Lz) - 2*thetaz(1+mi:Lz-mi) + thetaz(1:Lz-2*mi)).^2, 1);
end
avarz = avarz ./ (2*tauz.^2 .* (Lz - 2*mz));
adevz = sqrt(avarz);


figure(2);
loglog(taux, adevx) 
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal

hold on
loglog(tauy, adevy)
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
axis equal

hold on
loglog(tauz, adevz)
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on
legend("x","y","z")
axis equal
hold off

slope = -0.5;
logtau = log10(taux);
logadev = log10(adevx);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope*logtau(i);
logN = slope*log(1) + b;
N = 10^logN
tauN = 1;
lineN = N ./ sqrt(taux);

% Figure 3: Allan Deviation with Angle Random Walk (Using Y-Axis Data)
slope = -0.5;
logtau = log10(tauy);
logadev = log10(adevy);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope*logtau(i);
logNy = slope*log(1) + b;
Ny = 10^logNy
tauNy = 1;
lineNy = Ny ./ sqrt(tauy);

figure(3);
loglog(tauy, adevy, tauy, lineNy, '--', tauNy, Ny, 'o')
title('Allan Deviation with Angle Random Walk (Y-Axis)')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma_Y', '\sigma_N_y')
text(tauNy, Ny, 'N_y')
grid on
axis equal

% Figure 4: Allan Deviation with Rate Random Walk (Using Y-Axis Data)
slope = 0.5;
logtau = log10(tauy);
logadev = log10(adevy);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope*logtau(i);
logKy = slope*log10(3) + b;
Ky = 10^logKy
tauKy = 3;
lineKy = Ky .* sqrt(tauy/3);

figure(4);
loglog(tauy, adevy, tauy, lineKy, '--', tauKy, Ky, 'o')
title('Allan Deviation with Rate Random Walk (Y-Axis)')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma_Y', '\sigma_K_y')
text(tauKy, Ky, 'K_y')
grid on
axis equal

% Figure 5: Allan Deviation with Bias Instability (Using Y-Axis Data)
slope = 0;
logtau = log10(tauy);
logadev = log10(adevy);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope*logtau(i);
scfBy = sqrt(2*log(2)/pi);
logBy = b - log10(scfBy);
By = 10^logBy
tauBy = tauy(i);
lineBy = By * scfBy * ones(size(tauy));

figure(5);
loglog(tauy, adevy, tauy, lineBy, '--', tauBy, scfBy*By, 'o')
title('Allan Deviation with Bias Instability (Y-Axis)')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma_Y', '\sigma_B_y')
text(tauBy, scfBy*By, '0.664B_y')
grid on
axis equal

% Figure 6: Allan Deviation with Noise Parameters (Using Y-Axis Data)
tauParamsY = [tauNy, tauKy, tauBy];
paramsY = [Ny, Ky, scfBy*By];

figure(6);
loglog(tauy, adevy, tauy, [lineNy, lineKy, lineBy], '--', ...
    tauParamsY, paramsY, 'o')
title('Allan Deviation with Noise Parameters (Y-Axis)')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma_Y (rad/s)$', '$\sigma_{N_y} ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_{K_y} ((rad/s)\sqrt{Hz})$', '$\sigma_{B_y} (rad/s)$', 'Interpreter', 'latex')
text(tauParamsY, paramsY, {'N_y', 'K_y', '0.664B_y'})
grid on
axis equal



