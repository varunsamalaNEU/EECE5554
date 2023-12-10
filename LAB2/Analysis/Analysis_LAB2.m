clc
bag_1os = rosbag('open_stat.bag');
bag_2ow = rosbag('open_walk.bag');
bag_3cs = rosbag('occ_stat.bag');
bag_4cw = rosbag('occ_walk.bag');

gps_topic_1os = select(bag_os, 'topic', '/gps');
gps_topic_2ow = select(bag_ow, 'topic', '/gps');
gps_topic_3cs = select(bag_cs, 'topic', '/gps');
gps_topic_4cw = select(bag_cw, 'topic', '/gps');

topic_struct_1os = readMessages(gps_topic_os, 'DataFormat', 'struct');
topic_struct_2ow = readMessages(gps_topic_ow, 'DataFormat', 'struct');
topic_struct_3cs = readMessages(gps_topic_cs, 'DataFormat', 'struct');
topic_struct_4cw = readMessages(gps_topic_cw, 'DataFormat', 'struct');

% Extract UTM coordinates and calculate relative values
os_x1 = cellfun(@(m) double(m.UTMEasting), topic_struct_os);
os_y1 = cellfun(@(m) double(m.UTMNorthing), topic_struct_os);
os_x01 = os_x1 - os_x1(1);
os_y01 = os_y1 - os_y1(1);

ow_x1 = cellfun(@(m) double(m.UTMEasting), topic_struct_ow);
ow_y1 = cellfun(@(m) double(m.UTMNorthing), topic_struct_ow);
ow_x01 = ow_x1 - ow_x1(1);
ow_y01 = ow_y1 - ow_y1(1);

cs_x1 = cellfun(@(m) double(m.UTMEasting), topic_struct_cs);
cs_y1 = cellfun(@(m) double(m.UTMNorthing), topic_struct_cs);
cs_x01 = cs_x1 - cs_x1(1);
cs_y01 = cs_y1 - cs_y1(1);

cw_x1 = cellfun(@(m) double(m.UTMEasting), topic_struct_cw);
cw_y1 = cellfun(@(m) double(m.UTMNorthing), topic_struct_cw);
cw_x01 = cw_x1 - cw_x1(1);
cw_y01 = cw_y1 - cw_y1(1);

% Plot the data
figure(1);
scatter(os_x01, os_y01, 'filled');
grid on;
legend('OpenStationaryScatterPlots');
title('Northing vs. Easting - Open Walking Data');
xlabel('UTMEasting (meters)');
ylabel('UTMNorthing (meters)');

figure(2);
scatter(ow_x01, ow_y01, 'filled');
grid on;
legend('OpenWalking - ScatterPlots');
title('Northing vs. Easting - Open Walking Data');
xlabel('UTMEasting (meters)');
ylabel('UTMNorthing (meters)');

figure(3);
scatter(cs_x01, cs_y01, 'filled');
grid on;
legend('ClosedStationary - ScatterPlots');
title('Northing vs. Easting - Closed Stationary Data');
xlabel('UTMEasting (meters)');
ylabel('UTMNorthing (meters)');

figure(4);
scatter(cw_x01, cw_y01, 'filled');
grid on;
legend('ClosedWalking - ScatterPlots');
title('Northing vs. Easting - Closed Walking Data');
xlabel('UTMEasting (meters)');
ylabel('UTMNorthing (meters)');

% Error Calculation
% Known Values
ow_east1 = 326315.78125;	
ow_north1 = 4686126.5;

cw_east2 = 326424.34375;
cw_north2 = 4686146;

% Calculate the error
new_ow_east1 = ow_east1 - ow_x1(1);
new_ow_north1 = ow_north1 - ow_y1(1);
new_cw_east2 = cw_east2 - cw_x1(1);
new_cw_north2 = cw_north2 - cw_y1(1);

% Calculate the Euclidean distance for the error
error_open = sqrt(new_ow_east1^2 + new_ow_north1^2);
error_occluded = sqrt(new_cw_east2^2 + new_cw_north2^2);

disp('error_open:');
disp(error_open);
disp('error_occluded:');
disp(error_occluded);




