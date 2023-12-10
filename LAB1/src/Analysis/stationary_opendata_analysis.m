bag = rosbag("stationary_open.bag")
bagInfo = rosbag("info","stationary_open.bag")
rosbag info stationary_open.bag
gps_topic = select(bag, 'Topic','/gps');
topic_struct = readMessages(bag,'DataFormat','struct');
utm_north = cellfun(@(m) double(m.UTMNorthing),topic_struct);
utm_east = cellfun(@(m) double(m.UTMEasting),topic_struct);
alt = cellfun(@(m) double(m.Altitude),topic_struct);
time_axis = cellfun(@(m) double(m.Header.Stamp.Sec),topic_struct);

long = cellfun(@(m) double(m.Longitude),topic_struct);
lat = cellfun(@(m) double(m.Latitude),topic_struct);

utm_east_s=utm_east(:) - utm_east(1);
utm_north_s=utm_north(:) - utm_north(1);
time_axis=time_axis(:) - time_axis(1);

deltaX = cellfun(@(m) double(m.UTMEasting)-topic_struct{267}.UTMEasting, topic_struct);
deltaY = cellfun(@(m) double(m.UTMNorthing)-topic_struct{267}.UTMNorthing, topic_struct); 
error = sqrt(deltaX.^2+deltaY.^2);
h1=histogram(error);
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")
grid on 
legend 
title('Histogram', 'Open Stationary Data');

%error_north = 0;

figure(2);
scatter(utm_east,utm_north,"filled")
grid on 
legend('OpenStationaryPlots') 
title('Northing vs. Easting', 'Open Stationary Data');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")

figure(3);
scatter(utm_east_s,utm_north_s,"filled")
grid on 
legend('OpenStationaryPlots')  
title('Northing vs. Easting', 'With the 1st value subtracted from each dataset for Open Area');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")

figure(4);
scatter(time_axis,alt,"filled");
grid on 
legend('OpenScatterPlot')  
title('Altitude vs. Time', 'Scatterplots for Open');
xlabel("Time(sec)")
ylabel("Altitude(m)")

figure(5);
plot(time_axis,error)
grid on 
legend('ScatterPlotOccluded')
title('Error vs. Time', 'Scatterplots for Occluded Error Vs Time');
xlabel("Time(sec)")
ylabel("Error(m)")

%values = [error];
%mean_value = mean(values);
%disp(mean_value);

values = [error];
disp(class(values(1)));

A = [error];
B = round(A); % convert double to integer
disp(B) % display the integer array
mean_B = mean(B); % find the average
median_B = median(B); % find the median
disp(mean_B) % display the average
disp(median_B) % display the median



