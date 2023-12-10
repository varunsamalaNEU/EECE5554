clc
clear all
bag = rosbag("walking.bag")
bagInfo = rosbag("info","walking.bag")
rosbag info walking.bag
gps_topic = select(bag, 'Topic','/gps');
topic_struct = readMessages(bag,'DataFormat','struct');
utm_north = cellfun(@(m) double(m.UTMNorthing),topic_struct);

long = cellfun(@(m) double(m.Longitude),topic_struct);
lat = cellfun(@(m) double(m.Latitude),topic_struct);

utm_east = cellfun(@(m) double(m.UTMEasting),topic_struct);
alt = cellfun(@(m) double(m.Altitude),topic_struct);
time_axis = cellfun(@(m) double(m.Header.Stamp.Sec),topic_struct);
r = polyfit(utm_east, utm_north,1);
s = polyval(r, utm_east);

utm_east_s=utm_east(:) - utm_east(1);
utm_north_s=utm_north(:) - utm_north(1);
time_axis=time_axis(:) - time_axis(1);
r = polyfit(utm_east, utm_north,1);
s = polyval(r, utm_east);


figure(1);
scatter(time_axis,alt,"filled")
xlabel("Time(sec)")
ylabel("Altitude(meters)")
grid on 
legend('ScatterPlotsWalking')
title('Altitude Vs Time', 'Walking Data');


figure(2);
%scatter(utm_east,utm_north,"filled")
plot(utm_east,utm_north, '.', utm_east, s, '-');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")
grid on 
legend('GPSDdata', 'ActualStraightline')
title('Northing vs. Easting', 'Walking Data');

figure(3);
scatter(utm_east_s,utm_north_s,"filled")
%hold on;
%plot (utm_error);
%plot(utm_east_s,utm_north_s, '.', utm_east_s, s, '-');
grid on 
legend('Scatter Plots');
title('Northing vs. Easting', 'With the 1st value subtracted from each dataset for Occluded');
xlabel("UTMEasting(meters)")
ylabel("UTMNorthing(meters)")

long = cellfun(@(m) double(m.Longitude),topic_struct);
lat = cellfun(@(m) double(m.Latitude),topic_struct);

figure(4);
deltaX = cellfun(@(m) double(m.UTMEasting)-topic_struct{67}.UTMEasting, topic_struct);
deltaY = cellfun(@(m) double(m.UTMNorthing)-topic_struct{67}.UTMNorthing, topic_struct); 
error = sqrt(deltaX.^2+deltaY.^2);
H1=histogram(error);


xlabel("Error(meters)")
ylabel("Time(sec)")
grid on 
legend 
title('Histogram', 'Walking Data');


k_lat_start = 42.20500;
k_lat_end = 42.2045446;
k_lon_start = -71.058514;
k_lon_end = -71.0592692;

lat_interval = (k_lat_end - k_lat_start) / 67;
lon_interval = (k_lon_end - k_lon_start) / 67;

k_lats = k_lat_start:lat_interval:k_lat_end;
k_lons = k_lon_start:lon_interval:k_lon_end;

Error = zeros(1, 67);

real_value = double(Error);
integer_value = int64(Error);

sum = 0;
for i = 1:67
%Error(i) 
Answer = distance(lat(i), long(i), k_lats(i), k_lons(i));
%disp(Answer)
sum = sum+Answer;
end

disp(Error);
mean = sum/67;
disp(mean);

%Error = 1.0e+06;
mean = mean(Error);
disp(mean);

function d = distance(lat1, lon1, lat2, lon2)
    % convert decimal degrees to radians
    lat1 = deg2rad(lat1);
    lon1 = deg2rad(lon1);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);
    
    % haversine formula
    dlat = lat2 - lat1;
    dlon = lon2 - lon1;
    a = sin(dlat/2)*2 + cos(lat1) * cos(lat2) * sin(dlon/2)*2;
    a=-a
    if a<0
        d=NaN;
    return;
    end
    disp(a);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    r = 6.3781*10^6; % radius of earth in kilometers
    d = c * r;
end


