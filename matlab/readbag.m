clc;
clear;
close all;

% Define the path to the ROS 2 bag file
bag_path = "/home/ahmet/workspace/ros2_ws/ros2_test_ws/croped_bag_euler_attitude/rosbag2_2025_03_06-18_27_56/rosbag2_2025_03_06-18_27_56_0.db3";

% Create a bag reader object
bag_reader = ros2bagreader(bag_path);

% Define the topics to read from
topics = {
    "/imu/mahony_ahrs/pose_euler",
    "/imu/tools/com_non_adaptive/pose_euler",
    "/imu/tools/com_adaptive/pose_euler",
    "/imu/tools/madwick/pose_euler",
    "/imu/ground_truth_euler"
};

% Define the legend names for plotting
legend_name = {
    "imu-mahony-ahrs",
    "imu-tools-com-non-adaptive",
    "imu-tools-com-adaptive",
    "imu-tools-madwick",
    "imu-ground-truth"
};

% Initialize arrays to store the roll, pitch, yaw values, and timestamps
mahony_ahrs_roll = [];
mahony_ahrs_pitch = [];
mahony_ahrs_yaw = [];
mahony_ahrs_time = [];

imu_tools_com_non_adaptive_roll = [];
imu_tools_com_non_adaptive_pitch = [];
imu_tools_com_non_adaptive_yaw = [];
imu_tools_com_non_adaptive_time = [];

imu_tools_com_adaptive_roll = [];
imu_tools_com_adaptive_pitch = [];
imu_tools_com_adaptive_yaw = [];
imu_tools_com_adaptive_time = [];

imu_tools_madwick_roll = [];
imu_tools_madwick_pitch = [];
imu_tools_madwick_yaw = [];
imu_tools_madwick_time = [];

imu_ground_truth_roll = [];
imu_ground_truth_pitch = [];
imu_ground_truth_yaw = [];
imu_ground_truth_time = [];

% Loop through each topic and extract the messages
for i = 1:length(topics)
    % Select the topic
    topic = topics{i};
    
    % Read the messages from the topic
    msgs = readMessages(select(bag_reader, 'Topic', topic));
    
    % Extract roll, pitch, yaw, and timestamps from each message
    for j = 1:length(msgs)
        msg = msgs{j};
        roll = msg.vector.x;
        pitch = msg.vector.y;
        yaw = msg.vector.z;
        time = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
        
        % Store the values in the appropriate arrays
        switch topic
            case "/imu/mahony_ahrs/pose_euler"
                mahony_ahrs_roll = [mahony_ahrs_roll; roll];
                mahony_ahrs_pitch = [mahony_ahrs_pitch; pitch];
                mahony_ahrs_yaw = [mahony_ahrs_yaw; yaw];
                mahony_ahrs_time = [mahony_ahrs_time; time];
            case "/imu/tools/com_non_adaptive/pose_euler"
                imu_tools_com_non_adaptive_roll = [imu_tools_com_non_adaptive_roll; roll];
                imu_tools_com_non_adaptive_pitch = [imu_tools_com_non_adaptive_pitch; pitch];
                imu_tools_com_non_adaptive_yaw = [imu_tools_com_non_adaptive_yaw; yaw];
                imu_tools_com_non_adaptive_time = [imu_tools_com_non_adaptive_time; time];
            case "/imu/tools/com_adaptive/pose_euler"
                imu_tools_com_adaptive_roll = [imu_tools_com_adaptive_roll; roll];
                imu_tools_com_adaptive_pitch = [imu_tools_com_adaptive_pitch; pitch];
                imu_tools_com_adaptive_yaw = [imu_tools_com_adaptive_yaw; yaw];
                imu_tools_com_adaptive_time = [imu_tools_com_adaptive_time; time];
            case "/imu/tools/madwick/pose_euler"
                imu_tools_madwick_roll = [imu_tools_madwick_roll; roll];
                imu_tools_madwick_pitch = [imu_tools_madwick_pitch; pitch];
                imu_tools_madwick_yaw = [imu_tools_madwick_yaw; yaw];
                imu_tools_madwick_time = [imu_tools_madwick_time; time];
            case "/imu/ground_truth_euler"
                imu_ground_truth_roll = [imu_ground_truth_roll; roll];
                imu_ground_truth_pitch = [imu_ground_truth_pitch; pitch];
                imu_ground_truth_yaw = [imu_ground_truth_yaw; yaw];
                imu_ground_truth_time = [imu_ground_truth_time; time];
        end
    end
end

% Plot the roll, pitch, and yaw values in separate figures with respect to timestamps
figure('Position', [0, 0, 1920, 1080]); % Set figure size to 1920x1080
plot(mahony_ahrs_time, mahony_ahrs_roll, 'LineWidth', 2, 'DisplayName', legend_name{1});
hold on;
plot(imu_tools_com_non_adaptive_time, imu_tools_com_non_adaptive_roll, 'LineWidth', 2, 'DisplayName', legend_name{2});
plot(imu_tools_com_adaptive_time, imu_tools_com_adaptive_roll, 'LineWidth', 2, 'DisplayName', legend_name{3});
plot(imu_tools_madwick_time, imu_tools_madwick_roll, 'LineWidth', 2, 'DisplayName', legend_name{4});
plot(imu_ground_truth_time, imu_ground_truth_roll, 'LineWidth', 2, 'DisplayName', legend_name{5});
title('Roll');
xlabel('Time (seconds)');
ylabel('Angle (degree)');
legend;
grid on;
set(gca, 'FontSize', 12); % Increase font size for better readability

figure('Position', [0, 0, 1920, 1080]); % Set figure size to 1920x1080
plot(mahony_ahrs_time, mahony_ahrs_pitch, 'LineWidth', 2, 'DisplayName', legend_name{1});
hold on;
plot(imu_tools_com_non_adaptive_time, imu_tools_com_non_adaptive_pitch, 'LineWidth', 2, 'DisplayName', legend_name{2});
plot(imu_tools_com_adaptive_time, imu_tools_com_adaptive_pitch, 'LineWidth', 2, 'DisplayName', legend_name{3});
plot(imu_tools_madwick_time, imu_tools_madwick_pitch, 'LineWidth', 2, 'DisplayName', legend_name{4});
plot(imu_ground_truth_time, imu_ground_truth_pitch, 'LineWidth', 2, 'DisplayName', legend_name{5});
title('Pitch');
xlabel('Time (seconds)');
ylabel('Angle (degree)');
legend;
grid on;
set(gca, 'FontSize', 12); % Increase font size for better readability

figure('Position', [0, 0, 1920, 1080]); % Set figure size to 1920x1080
plot(mahony_ahrs_time, mahony_ahrs_yaw, 'LineWidth', 2, 'DisplayName', legend_name{1});
hold on;
plot(imu_tools_com_non_adaptive_time, imu_tools_com_non_adaptive_yaw, 'LineWidth', 2, 'DisplayName', legend_name{2});
plot(imu_tools_com_adaptive_time, imu_tools_com_adaptive_yaw, 'LineWidth', 2, 'DisplayName', legend_name{3});
plot(imu_tools_madwick_time, imu_tools_madwick_yaw, 'LineWidth', 2, 'DisplayName', legend_name{4});
plot(imu_ground_truth_time, imu_ground_truth_yaw, 'LineWidth', 2, 'DisplayName', legend_name{5});
title('Yaw');
xlabel('Time (seconds)');
ylabel('Angle (degree)');
legend;
grid on;
set(gca, 'FontSize', 12); % Increase font size for better readability

% Loop through each topic and create a figure with 3 subplots
for i = 1:length(topics)
    % Create a new figure for each topic
    figure('Position', [0, 0, 1920, 1080]); % Set figure size to 1920x1080
    sgtitle(['Comparison of ', legend_name{i}, ' with Ground Truth'], 'FontSize', 14); % Set a title for the entire figure
    
    % Extract the topic's roll, pitch, yaw data, and timestamps
    switch topics{i}
        case "/imu/mahony_ahrs/pose_euler"
            topic_roll = mahony_ahrs_roll;
            topic_pitch = mahony_ahrs_pitch;
            topic_yaw = mahony_ahrs_yaw;
            topic_time = mahony_ahrs_time;
        case "/imu/tools/com_non_adaptive/pose_euler"
            topic_roll = imu_tools_com_non_adaptive_roll;
            topic_pitch = imu_tools_com_non_adaptive_pitch;
            topic_yaw = imu_tools_com_non_adaptive_yaw;
            topic_time = imu_tools_com_non_adaptive_time;
        case "/imu/tools/com_adaptive/pose_euler"
            topic_roll = imu_tools_com_adaptive_roll;
            topic_pitch = imu_tools_com_adaptive_pitch;
            topic_yaw = imu_tools_com_adaptive_yaw;
            topic_time = imu_tools_com_adaptive_time;
        case "/imu/tools/madwick/pose_euler"
            topic_roll = imu_tools_madwick_roll;
            topic_pitch = imu_tools_madwick_pitch;
            topic_yaw = imu_tools_madwick_yaw;
            topic_time = imu_tools_madwick_time;
        case "/imu/ground_truth_euler"
            continue; % Skip ground truth, as it is the reference
    end
    
    % Subplot 1: Roll comparison
    subplot(3, 1, 1);
    plot(topic_time, topic_roll, 'LineWidth', 2, 'DisplayName', legend_name{i});
    hold on;
    plot(imu_ground_truth_time, imu_ground_truth_roll, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    title('Roll Comparison', 'FontSize', 12);
    xlabel('Time (seconds)', 'FontSize', 12);
    ylabel('Angle (degree)', 'FontSize', 12);
    legend;
    grid on;
    
    % Subplot 2: Pitch comparison
    subplot(3, 1, 2);
    plot(topic_time, topic_pitch, 'LineWidth', 2, 'DisplayName', legend_name{i});
    hold on;
    plot(imu_ground_truth_time, imu_ground_truth_pitch, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    title('Pitch Comparison', 'FontSize', 12);
    xlabel('Time (seconds)', 'FontSize', 12);
    ylabel('Angle (degree)', 'FontSize', 12);
    legend;
    grid on;
    
    % Subplot 3: Yaw comparison
    subplot(3, 1, 3);
    plot(topic_time, topic_yaw, 'LineWidth', 2, 'DisplayName', legend_name{i});
    hold on;
    plot(imu_ground_truth_time, imu_ground_truth_yaw, 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    title('Yaw Comparison', 'FontSize', 12);
    xlabel('Time (seconds)', 'FontSize', 12);
    ylabel('Angle (degree)', 'FontSize', 12);
    legend;
    grid on;
end