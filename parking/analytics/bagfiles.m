bag = rosbag('slam2.bag');
bag.AvailableTopics;

msgs = readMessages(bag);
%showdetails(msgs{2})

%extract data as timeseries
ts = timeseries(bag, 'Pose.Position.X');

%plot(ts);

x = ts.Time;
y = ts.Data;
%plot(x, y);

%extract the range of data the is meaningful
%usually: y value reaches a max and then drops --> only consider data til
%max
%find time (x-value) for which y is max --> no!
%k is index for which y is max!
k = find(y==max(y));
x_mod = x(1:k);
%and since k is an index it is also applicable to y:
y_mod = y(1:k);

%plot the extracted range
%plot(x_mod, y_mod);

%shift x-values so that x-axis starts at 0
x_mod = x_mod - x_mod(1);
%plot(x_mod, y_mod);

%Fast Fourier Transform
Y_mod = fft(y_mod);
Y_mag = abs(Y_mod);
T = x_mod(k);
Fs = 1/T;
Fbins = ((0: 1/k: 1-1/k)*Fs).';
plot(Fbins, Y_mag);

%moving average
coeff = ones(1, 10)/10; %one second split into 10 10th of a second
avg = filter(coeff, 1, y_mod);
%plot(x_mod, y_mod)
%hold on
%plot(x_mod, avg);

%find max difference between original data and average data
diff = zeros(k, 1);
for c = 1:1:k
diff(c) = abs(y_mod(c) - avg(c));
end
max_diff = max(diff);

j = find(diff == max(diff)); %at what time is difference maximum?
%--> problem: difference is largest in the end



