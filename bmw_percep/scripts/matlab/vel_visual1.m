%% Visualize the predictive capability of velocities

path = '../../data/centroid_estimate/';
file1 = 'shr_normal1_kf_vel_05.txt';
file2 = 'shr_normal1_kf_vel_01.txt';
file3 = 'shr_normal1_acc_05.txt';
file4 = 'shr_normal_acc_5.txt';
file = 'median_8.txt';
ws_min = [.05, .84];
ws_max = [4.3, 4.0];

raw_data = csvread(strcat(path,file));

total_frames = size(raw_data,1);

obs_pos = raw_data(:,1:2);
est_pos = raw_data(:,3:4);
est_vel = raw_data(:, 5:6);

frame_times = zeros(total_frames, 1);

for i=2:total_frames %first frame is zero
    frame_times(i) = frame_times(i-1) + raw_data(i,7);
end

prediction0 = .25;
prediction1 = .5; %seconds
prediction2 = 1.; %seconds

cur_err = [];
p1_err = [];
p2_err = [];
p0_err = [];

% visualization for the current estimate(green) vs ground (red)
figure
for i=1:total_frames
    clf
    xlim([ws_min(1) ws_max(1)]);
    ylim([ws_min(2) ws_max(2)]);
    hold on
    
    %current
    cur_obs_p = obs_pos(i,:);
    cur_est_p = est_pos(i,:);
    plot(cur_obs_p(1), cur_obs_p(2), 'r+', 'MarkerSize', 12);    
    plot(cur_est_p(1), cur_est_p(2), 'g+', 'MarkerSize', 12);
    cur_err = [cur_err; cur_obs_p-cur_est_p]; 
    cur_time = frame_times(i);
    
    %predictions
    j=1;
    p2_found = 0;
    p1_found = 0;
    p0_found = 0;
    while(~p2_found)
        
        if (j+i>total_frames)
           break; 
        end
        
       
        if(frame_times(j+i)-cur_time>prediction0 && ~p0_found)
            p0_frame = j+i;
            p0_found = 1;
        end
        
        if(frame_times(j+i)-cur_time>prediction1 && ~p1_found)
            p1_frame = j+i;
            p1_found = 1;
        end
        
        if(frame_times(j+i)-cur_time>prediction2)
            p2_frame = j+i;
            p2_found = 1;
        end
        j = j+1;
    end
    
    if (~p2_found)
        break; % not enough frames to look ahead
    end
    
    p1_obs = obs_pos(p1_frame,:);
    p2_obs = obs_pos(p2_frame,:);
    p0_obs = obs_pos(p0_frame,:);
    
    cur_est_v = est_vel(i,:);
    p1_time = frame_times(p1_frame) - frame_times(i);
    p2_time = frame_times(p2_frame) - frame_times(i);
    p0_time = frame_times(p0_frame) - frame_times(i);
    
    p1_est = cur_est_p + p1_time * cur_est_v;
    p2_est = cur_est_p + p2_time * cur_est_v;
    p0_est = cur_est_p + p0_time * cur_est_v;
    
    plot(p1_obs(1), p1_obs(2), 'r*', 'MarkerSize', 12);    
    plot(p1_est(1), p1_est(2), 'g*', 'MarkerSize', 12);

    plot(p2_obs(1), p2_obs(2), 'ro', 'MarkerSize', 12);    
    plot(p2_est(1), p2_est(2), 'go', 'MarkerSize', 12);
    
    plot(p0_obs(1), p0_obs(2), 'rx', 'MarkerSize', 12);    
    plot(p0_est(1), p0_est(2), 'gx', 'MarkerSize', 12);
    
    p1_err = [p1_err; p1_obs-p1_est];
    p2_err = [p2_err; p2_obs-p2_est];
    p0_err = [p0_err; p0_obs-p0_est];
    pause(0.05);
    hold off
end

% show error plots
cur_err_dist = sqrt(cur_err(:,1).^2 + cur_err(:,2).^2);
p1_err_dist = sqrt(p1_err(:,1).^2 + p1_err(:,2).^2);
p2_err_dist = sqrt(p2_err(:,1).^2 + p2_err(:,2).^2);
p0_err_dist = sqrt(p0_err(:,1).^2 + p0_err(:,2).^2);

% Error means
'Error on Current Estimate = '
mean(cur_err_dist)

'Error on Current Prediction1 = '
mean(p1_err_dist)

'Error on Current Prediction2 = '
mean(p2_err_dist)

figure
plot(cur_err_dist,'b')
hold on
plot(p1_err_dist,'r')
plot(p2_err_dist,'g')
plot(p0_err_dist,'m')
hold off