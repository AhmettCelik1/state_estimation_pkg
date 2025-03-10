clc;
clear;
close all;


% to run this matlab code, publish and save your ground truth  data as geometry_msgs::msg::Vector3stamp
% vector.vector.x=roll
% vector.vector.y=pitch
% vector.vector.z=yaw


% and also save the imu data as sensor_msgs/Imu Message with respected to 
% your ground truth timestamps

currentDir = pwd;
plotDir = fullfile(currentDir, 'plot');
save_directory = plotDir;
if ~exist(save_directory, 'dir')
    mkdir(save_directory);
end   


bag_path = "/home/ahmet/workspace/ros2_ws/ros2_test_ws/seconds_30_first_bag/rosbag2_2025_03_10-01_43_44/rosbag2_2025_03_10-01_43_44_0.db3";

bag_reader = ros2bagreader(bag_path);

imu_topic_name = "/sensors/imu_with_timestamp";
imu_topic = select(bag_reader, 'Topic', imu_topic_name);
imu_msgs = readMessages(imu_topic);
num_messages_imu = length(imu_msgs);

ground_truth_euler_topic_name = "/imu/ground_truth_euler";
ground_truth_euler_topic = select(bag_reader, 'Topic', ground_truth_euler_topic_name);
ground_truth_euler_msgs = readMessages(ground_truth_euler_topic);
num_messages_ground_truth_euler = length(ground_truth_euler_msgs);

imu_timestamps = zeros(num_messages_imu, 1);
ground_truth_timestamps = zeros(num_messages_ground_truth_euler, 1);

for i = 1:num_messages_imu
    imu_timestamps(i) = double(imu_msgs{i}.header.stamp.sec) + double(imu_msgs{i}.header.stamp.nanosec) * 1e-9;
end

for i = 1:num_messages_ground_truth_euler
    ground_truth_timestamps(i) = double(ground_truth_euler_msgs{i}.header.stamp.sec) + double(ground_truth_euler_msgs{i}.header.stamp.nanosec) * 1e-9;
end

number_of_test = 30;

% Set Init Kp Ki value to tune 
Kp=0.25;
Ki=0.0;
diff_kp_value = 0.01;
diff_ki_value = 0.01; 

%the flag that flag_kp_is_tuning true
% for kp tuning, change it false for ki tuning
flag_kp_is_tuning = true;

%  change these flag_increased_kp, flag_increased_ki for decresed or increased
flag_increased_kp = false; 
flag_increased_ki = false; 

plot_yaw_pitch_roll = true; 
plot_is_enable_Q = true; 

Kp_values = zeros(number_of_test, 1);
Ki_values = zeros(number_of_test, 1); 
Q_values = zeros(number_of_test, 1); 

q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0; 
integralFBx = 0.0; integralFBy = 0.0; integralFBz = 0.0; 
useAcc = true; 
dt = 0.01; 


rMat = eye(3);

roll_angles = zeros(num_messages_imu, 1);
pitch_angles = zeros(num_messages_imu, 1);
yaw_angles = zeros(num_messages_imu, 1);
cost_function = zeros(num_messages_imu, 1);

ground_truth_roll_angles = zeros(num_messages_imu, 1);
ground_truth_pitch_angles = zeros(num_messages_imu, 1);
ground_truth_yaw_angles = zeros(num_messages_imu, 1);



for test = 1:number_of_test
    if flag_kp_is_tuning
        if test == 1
            % Initial Kp value
            Kp = Kp; 
        else
            if flag_increased_kp
                Kp = Kp + diff_kp_value; 
            else
                Kp = Kp - diff_kp_value;
            end
        end
        Kp_values(test) = Kp; 
        Ki = Ki; 
    else
        if test == 1
            % Initial Ki value
            Ki = Ki;
        else
            if flag_increased_ki
                Ki = Ki + diff_ki_value; 
            else
                Ki = Ki - diff_ki_value; 
            end
        end
        Ki_values(test) = Ki;
        Kp = Kp; 
    end
    
    q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0;
    integralFBx = 0.0; integralFBy = 0.0; integralFBz = 0.0;
    rMat = eye(3);
    
    for i = 1:num_messages_imu
        ax = imu_msgs{i}.linear_acceleration.x;
        ay = imu_msgs{i}.linear_acceleration.y;
        az = imu_msgs{i}.linear_acceleration.z;
        
        gx = imu_msgs{i}.angular_velocity.x;
        gy = imu_msgs{i}.angular_velocity.y;
        gz = imu_msgs{i}.angular_velocity.z;
        
        [q_w, q_x, q_y, q_z, rMat] = mahonyAHRSupdate(q_w, q_x, q_y, q_z, ...
            gx, gy, gz, ax, ay, az, Kp, Ki, integralFBx, integralFBy, integralFBz, useAcc, dt, rMat);
        
        [roll, pitch, yaw] = quat2eul([q_w, q_x, q_y, q_z]);
        
        roll_angles(i) = rad2deg(roll);
        pitch_angles(i) = rad2deg(pitch);
        yaw_angles(i) = rad2deg(yaw);
        
        [~, idx] = min(abs(ground_truth_timestamps - imu_timestamps(i)));
        
        ground_truth_roll_angles(i) = ground_truth_euler_msgs{idx}.vector.x;
        ground_truth_pitch_angles(i) = ground_truth_euler_msgs{idx}.vector.y;
        ground_truth_yaw_angles(i) = ground_truth_euler_msgs{idx}.vector.z;
        
        if i == 1
            cost_function(i) = (ground_truth_roll_angles(i) - roll_angles(i))^2 + ...
                              (ground_truth_pitch_angles(i) - pitch_angles(i))^2;
        else
            cost_function(i) = cost_function(i-1) + ...
                              (ground_truth_roll_angles(i) - roll_angles(i))^2 + ...
                              (ground_truth_pitch_angles(i) - pitch_angles(i))^2;
        end
    end
    
    Q_values(test) = cost_function(end);
 
  if plot_yaw_pitch_roll
        figure;
        
        subplot(3, 1, 1);
        plot(imu_timestamps, roll_angles, 'b', 'LineWidth', 1.5);
        hold on;
        plot(imu_timestamps, ground_truth_roll_angles, 'k--', 'LineWidth', 1.5);
        title(['Roll Angle (Kp = ', num2str(Kp), ', Ki = ', num2str(Ki), ')']);
        xlabel('Time (seconds)');
        ylabel('Angle (degrees)');
        legend('Estimated Roll', 'Ground Truth Roll');
        grid on;
        
        subplot(3, 1, 2);
        plot(imu_timestamps, pitch_angles, 'r', 'LineWidth', 1.5);
        hold on;
        plot(imu_timestamps, ground_truth_pitch_angles, 'k--', 'LineWidth', 1.5);
        title(['Pitch Angle (Kp = ', num2str(Kp), ', Ki = ', num2str(Ki), ')']);
        xlabel('Time (seconds)');
        ylabel('Angle (degrees)');
        legend('Estimated Pitch', 'Ground Truth Pitch');
        grid on;
        
        subplot(3, 1, 3);
        plot(imu_timestamps, yaw_angles, 'g', 'LineWidth', 1.5);
        hold on;
        plot(imu_timestamps, ground_truth_yaw_angles, 'k--', 'LineWidth', 1.5);
        title(['Yaw Angle (Kp = ', num2str(Kp), ', Ki = ', num2str(Ki), ')']);
        xlabel('Time (seconds)');
        ylabel('Angle (degrees)');
        legend('Estimated Yaw', 'Ground Truth Yaw');
        grid on;

        % Save the figure as a JPEG image
        set(gcf, 'Position', [0, 0, 1920, 1080]); % Set resolution to 1920x1080
        saveas(gcf, fullfile(save_directory, ['yaw_pitch_roll_plot_test_', num2str(test), '.jpg']));
    end
end

if plot_is_enable_Q
    figure;
    bar(Q_values);
    title('Cumulative Cost Function Q for Different Kp and Ki Values');
    xlabel('Test Number');
    ylabel('Cumulative Q');
    xticks(1:number_of_test);
    
    if flag_kp_is_tuning
        labels = arrayfun(@(x) sprintf('Kp=%.3f, Ki=%.3f', x, Ki), Kp_values, 'UniformOutput', false);
    else
        labels = arrayfun(@(x) sprintf('Kp=%.3f, Ki=%.3f', Kp, x), Ki_values, 'UniformOutput', false);
    end
    xticklabels(labels);
    
    grid on;

    % Save the figure as a JPEG image
    set(gcf, 'Position', [0, 0, 1920, 1080]); % Set resolution to 1920x1080
    saveas(gcf, fullfile(save_directory, 'cost_function_plot.jpg'));
end

function [q_w, q_x, q_y, q_z, rMat] = mahonyAHRSupdate(q_w, q_x, q_y, q_z, ...
    gx, gy, gz, ax, ay, az, Kp, Ki, integralFBx, integralFBy, integralFBz, useAcc, dt, rMat)
    
    if useAcc
        recipAccNorm = 1.0 / norm([ax, ay, az]);
        ax = ax * recipAccNorm;
        ay = ay * recipAccNorm;
        az = az * recipAccNorm;
        
        ex = (ay * rMat(3,3) - az * rMat(3,2));
        ey = (az * rMat(3,1) - ax * rMat(3,3));
        ez = (ax * rMat(3,2) - ay * rMat(3,1));
    else
        ex = 0.0; ey = 0.0; ez = 0.0;
    end
    
    if Ki > 0.0
        integralFBx = integralFBx + Ki * ex * dt;
        integralFBy = integralFBy + Ki * ey * dt;
        integralFBz = integralFBz + Ki * ez * dt;
    else
        integralFBx = 0.0; integralFBy = 0.0; integralFBz = 0.0;
    end
    
    gx = gx + Kp * ex + integralFBx;
    gy = gy + Kp * ey + integralFBy;
    gz = gz + Kp * ez + integralFBz;
    
    gx = gx * 0.5 * dt;
    gy = gy * 0.5 * dt;
    gz = gz * 0.5 * dt;
    
    buffer_w = q_w; buffer_x = q_x; buffer_y = q_y; buffer_z = q_z;
    
    q_w = q_w + (-buffer_x * gx - buffer_y * gy - buffer_z * gz);
    q_x = q_x + (buffer_w * gx + buffer_y * gz - buffer_z * gy);
    q_y = q_y + (buffer_w * gy - buffer_x * gz + buffer_z * gx);
    q_z = q_z + (buffer_w * gz + buffer_x * gy - buffer_y * gx);
    
    recipNorm = 1.0 / norm([q_w, q_x, q_y, q_z]);
    q_w = q_w * recipNorm;
    q_x = q_x * recipNorm;
    q_y = q_y * recipNorm;
    q_z = q_z * recipNorm;
    
    rMat = quat2rotm([q_w, q_x, q_y, q_z]);
end

function [roll, pitch, yaw] = quat2eul(q)
    q_w = q(1); q_x = q(2); q_y = q(3); q_z = q(4);
    
    roll = atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x^2 + q_y^2));
    pitch = asin(2*(q_w*q_y - q_z*q_x));
    yaw = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y^2 + q_z^2));
end