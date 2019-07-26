function output = AnalyseBagFile(file)
%ANALYSEBAGFILE Summary of this function goes here
%   Detailed explanation goes here

bag = rosbag(file);
start = bag.StartTime;
topicselect = select(bag, 'Time', [start + 30 start + 60], 'Topic', '/odom');
ts = timeseries(topicselect, 'Pose.Pose.Position.X', 'Twist.Twist.Angular.Z')

figure;
plot(ts);

output = 0;
end

