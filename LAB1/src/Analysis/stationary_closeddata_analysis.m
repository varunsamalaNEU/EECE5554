
clc
bag = rosbag("stationary_closed.bag");
bagInfo = rosbag("info","stationary_closed.bag");
rosbag info stationary_closed.bag
%print('hello');
gps_topic = select(bag, 'Topic','/gps');
topic_struct = readMessages(bag,'DataFormat','struct');

utm_north = cellfun(@(m) double(m.UTMNorthing),topic_struct);
utm_east = cellfun(@(m) double(m.UTMEasting),topic_struct);


alt = cellfun(@(m) double(m.Altitude),topic_struct);
time_axis = cellfun(@(m) double(m.Header.Stamp.Sec),topic_struct);

utm_east_s=utm_east(:) - utm_east(1);
utm_north_s=utm_north(:) - utm_north(1);
time_axis=time_axis(:) - time_axis(1);

long = cellfun(@(m) double(m.Longitude),topic_struct);
lat = cellfun(@(m) double(m.Latitude),topic_struct);

known_location = [42.338914, -71.088073];
estimated_location = [utm_north, utm_east];

%errors = estimated_location - known_location;
%mean_error = mean(errors, 1);
%print(mean_error)


figure(1);
deltaX = cellfun(@(m) double(m.UTMEasting)-topic_struct{267}.UTMEasting, topic_struct);
deltaY = cellfun(@(m) double(m.UTMNorthing)-topic_struct{267}.UTMNorthing, topic_struct); 
error = sqrt(deltaX.^2+deltaY.^2);
H1=histogram(error);


xlabel("Error(meters)")
ylabel("Time(sec)")
grid on 
legend 
title('Histogram', 'Occluded Stationary Data');

utm_error=sqrt((672012.19-utm_east).^2+(4689518.12-utm_north).^2);
print=(utm_error);

%lat_mean = mean(lat);
%long_mean = mean(long);
%error_north = 0;

figure(2);
scatter(utm_east,utm_north,"filled")

grid on 
legend('OccludedScatterPlots')
title('Northing vs. Easting', 'Occluded Stationary Data');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")

figure(3);
scatter(utm_east_s,utm_north_s,"filled")

grid on 
legend('OccludedScatterPlots')
title('Northing vs. Easting', 'With the 1st value subtracted from each dataset for Occluded');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")

figure(4);
scatter(time_axis,alt,"filled");
grid on 
legend('ScatterPlotOccluded')
title('Altitude vs. Time', 'Scatterplots for Occluded');
xlabel("Time(sec)")
ylabel("Altitude(m)")

figure(5);
plot(time_axis,error)
grid on 
legend('ScatterPlotOccluded')
title('Error vs. Time', 'Scatterplots for Occluded Error Vs Time');
xlabel("Time(sec)")
ylabel("Error(m)")
