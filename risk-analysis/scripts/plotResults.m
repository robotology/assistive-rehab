data_application_speed_obstacle = importdata("results/data_experiment_obstacle_0.3.txt");
data_maximum_speed_obstacle = importdata("results/data_experiment_obstacle_0.6.txt");

data_application_speed_stop = importdata("results/data_experiment_stop_0.3.txt");
data_maximum_speed_stop = importdata("results/data_experiment_stop_0.6.txt");

time_to_send_stop_app_speed = data_application_speed_obstacle(:,2);
time_to_send_stop_max_speed = data_maximum_speed_obstacle(:,2);

time_to_stop_app_speed = data_application_speed_stop(:,2);
time_to_stop_max_speed = data_maximum_speed_stop(:,2);

avg_t_app_speed = mean(time_to_send_stop_app_speed)
avg_t_max_speed = mean(time_to_send_stop_max_speed)
dev_t_app_speed = std(time_to_send_stop_app_speed)
dev_t_max_speed = std(time_to_send_stop_max_speed)

avg_t_app_speed_stop = mean(time_to_stop_app_speed)
avg_t_max_speed_stop = mean(time_to_stop_max_speed)
dev_t_app_speed_stop = std(time_to_stop_app_speed)
dev_t_max_speed_stop = std(time_to_stop_max_speed)

figure(1);
histogram(time_to_send_stop_app_speed, 'FaceColor','b'); hold on;
histogram(time_to_send_stop_max_speed, 'FaceColor','r'); hold off;
legend('app speed', 'max speed');
xlabel('time to send stop command [s]')
ylabel('number of instances');
saveas(gcf, 'time_to_send_stop', 'jpg');

figure(2);
histogram(time_to_stop_app_speed, 'FaceColor','b'); hold on;
histogram(time_to_stop_max_speed, 'FaceColor','r'); hold off;
legend('app speed', 'max speed');
xlabel('time to completely stop [s]')
ylabel('number of instances');
saveas(gcf, 'time_to_completely_stop', 'jpg');
