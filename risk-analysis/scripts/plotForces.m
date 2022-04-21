clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             PARAMETERS                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mrobot = 54; %kg
app_speed = 0.3; %m/s
max_speed = 0.6; %m/s
distance_collision = 0.005; %m
outliers_threshold = 4000; %N
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

results_folder = pwd + "/results_vale_7";
folders = dir(results_folder);
folder_names = {folders(3:end).name};

data_app_speed = [];
maxval_fmag_app_speed = [];
data_max_speed = [];
maxval_fmag_max_speed = [];
for i = 1 : length(folder_names)
  subfolder = folder_names{i};
  k = strfind(subfolder,"_");
  if length(k) >= 3
      trial_number = str2double(subfolder(k(2)+1:k(3)-1));
      if contains(subfolder, "trial")
          curr_trial = importdata(results_folder+"/"+subfolder+"/data.log");
          if size(curr_trial, 1) == 0
              warning("empty data");
          else
              fx_curr_trial = curr_trial(:,3);
              fy_curr_trial = curr_trial(:,4);
              fz_curr_trial = curr_trial(:,5);
              fmag_curr_trial = sqrt(fx_curr_trial.*fx_curr_trial + fy_curr_trial.*fy_curr_trial + fz_curr_trial.*fz_curr_trial);
              if contains(subfolder, "0.3")
                  data_app_speed = [data_app_speed; trial_number.*ones(size(curr_trial,1),1) curr_trial];
                  maxval_fmag_app_speed = [maxval_fmag_app_speed; trial_number max(fmag_curr_trial)];
              end
              if contains(subfolder, "0.6")
                  data_max_speed = [data_max_speed; trial_number.*ones(size(curr_trial,1),1) curr_trial];
                  maxval_fmag_max_speed = [maxval_fmag_max_speed; trial_number max(fmag_curr_trial)];
              end
          end
      end
  end
end

% t_app_speed = data_app_speed(:,3)-data_app_speed(1,3);
fgt_app_speed = mrobot*(app_speed*app_speed) / (2*distance_collision);
fx_app_speed = data_app_speed(:,4);
fy_app_speed = data_app_speed(:,5);
fz_app_speed = data_app_speed(:,6);
fmag_app_speed = sqrt(fx_app_speed.*fx_app_speed + fy_app_speed.*fy_app_speed + fz_app_speed.*fz_app_speed);

% t_max_speed = data_max_speed(:,3)-data_max_speed(1,3);
fgt_max_speed = mrobot*(max_speed*max_speed) / (2*distance_collision);
fx_max_speed = data_max_speed(:,4);
fy_max_speed = data_max_speed(:,5);
fz_max_speed = data_max_speed(:,6);
fmag_max_speed = sqrt(fx_max_speed.*fx_max_speed + fy_max_speed.*fy_max_speed + fz_max_speed.*fz_max_speed);

% remove outliers
maxval_fmag_app_speed = maxval_fmag_app_speed(maxval_fmag_app_speed(:,2)<outliers_threshold, :);
maxval_fmag_max_speed = maxval_fmag_max_speed(maxval_fmag_max_speed(:,2)<outliers_threshold, :);
mean(maxval_fmag_app_speed(:,2))
std(maxval_fmag_app_speed(:,2))
max(maxval_fmag_app_speed(:,2))
mean(maxval_fmag_max_speed(:,2))
std(maxval_fmag_max_speed(:,2))
max(maxval_fmag_max_speed(:,2))

%% figure
figure(1);
plot(fmag_app_speed, 'b'); hold on;
plot(fmag_max_speed, 'r');
plot(fgt_max_speed.*ones(length(fx_max_speed), 1), 'r--'); 
plot(fgt_app_speed.*ones(length(fx_app_speed), 1), 'b--');
hold off;
xlabel('time [s]');
ylabel('Fx [N]');
legend('App speed = 0.3 m/s', 'Max speed = 0.6 m/s');

figure(2);
histogram(maxval_fmag_app_speed(:,2), 'FaceColor','b', 'BinWidth', 100); hold on;
histogram(maxval_fmag_max_speed(:,2), 'FaceColor','r', 'BinWidth', 100); hold off;
legend('App speed = 0.3 m/s', 'Max speed = 0.6 m/s');
xlabel('impact force [N]')
ylabel('number of instances');
% saveas(gcf, 'forces', 'jpg');

figure(3);
plot(maxval_fmag_app_speed(:,1), maxval_fmag_app_speed(:,2), 'bx'); hold on;
plot(maxval_fmag_max_speed(:,1), maxval_fmag_max_speed(:,2), 'rx'); hold off;
legend('App speed = 0.3 m/s', 'Max speed = 0.6 m/s');
xlabel('trial number');
ylabel('impact force [N]');