%% MAIN SCRIPT TO PERFORM THE CALIBRATION WITH SIMULATION DATA

clear all, close all, clc

%% load simulated observations

load("simulation_data/simul_data_600_observations.mat");

T_BF = optimal_solution.T_BF;
T_PM = optimal_solution.T_PM;
n_observations = optimal_solution.n_observations;

%% add noise and bias to observations

% % EXAMPLE:
% T_final = addNoiseToTransformationMatrix(T_initial, mean_ang, mean_pos, stand_dev_ang, stand_dev_pos)

T_BF = addNoiseToTransformationMatrix(T_BF, [0;0;0], [0;0;0], deg2rad([0.05;0.05;0.05]), [0.1;0.1;0.1]/1000);
T_PM = addNoiseToTransformationMatrix(T_PM, [0;0;0], [0;0;0], deg2rad([0.04;0.04;0.04]), [0.04;0.04;0.04]/1000);


%% create .json file with data to the optimizer
% scale factor in translation parameters
scale_factor = 1/100000;
for i = 1:n_observations
   T_BF(1:3,4,i) = T_BF(1:3,4,i)*scale_factor; 
   T_PM(1:3,4,i) = T_PM(1:3,4,i)*scale_factor;
end


observations = zeros(n_observations,32);
for i = 1:size(T_BF,3)
    temp1=T_BF(:,:,i);
    temp2=T_PM(:,:,i);
    observations(i,:) = [temp1(:);temp2(:)]';
end

first_initialization = zeros(12,1);
n_initializations = 30;
max_iterations = 100000;


myStruct.n_initializations = num2str(n_initializations);
myStruct.max_iterations = num2str(max_iterations);
myStruct.observations = mat2str(observations);
myStruct.observations(1) = [];
myStruct.observations(end) = [];
myStruct.observations = string(myStruct.observations);
myStruct.first_initialization = mat2str(first_initialization);
myStruct.first_initialization(1) = [];
myStruct.first_initialization(end) = [];
myStruct.first_initialization = string(myStruct.first_initialization);


json_file = jsonencode(myStruct);
fid = fopen("C:\Dev\Robot_TrackingSystem_Calibration\observations.json","wt");
fprintf(fid, json_file);
fclose(fid);

%% execute optimizer and read the results

exe_file_path_and_name = "C:\Dev\Robot_TrackingSystem_Calibration\build\CalibrationSolver.exe";

system(exe_file_path_and_name);

fileID = fopen('C:\Dev\Robot_TrackingSystem_Calibration\results.txt','r');
formatSpec = '%f';
out = fscanf(fileID,formatSpec);


% back to meters 
out(4:6) = out(4:6)/scale_factor;
out(10:12) = out(10:12)/scale_factor;
for i = 1:n_observations
   T_BF(1:3,4,i) = T_BF(1:3,4,i)/scale_factor; 
   T_PM(1:3,4,i) = T_PM(1:3,4,i)/scale_factor;
end

%% calculate error between the optimal and obtained homogeneous transformations

T_FM_result = transfParameters_to_transfMatrix(out(1:6));
T_PB_result = transfParameters_to_transfMatrix(out(7:12));

T_FM_error = calculate_error_between_2_transf_matrices(T_FM_result, optimal_solution.T_FM);
T_PB_error = calculate_error_between_2_transf_matrices(T_PB_result, optimal_solution.T_PB);

disp("The rotational error between the optimal and the obtained T_FM matrix was " + num2str(rad2deg(T_FM_error(1))) + " degrees.")
disp("The translational error between the optimal and the obtained T_FM matrix was " + num2str(T_FM_error(2)*1000) + " mm.")
disp("The rotational error between the optimal and the obtained T_PB matrix was " + num2str(rad2deg(T_PB_error(1))) + " degrees.")
disp("The translational error between the optimal and the obtained T_PB matrix was " + num2str(T_PB_error(2)*1000) + " mm.")

%% calculate the obtained error (distance) between observations

T_FM_result = transfParameters_to_transfMatrix(out(1:6));
T_PB_result = transfParameters_to_transfMatrix(out(7:12));
total_error = real_calibration_error(T_FM_result, T_PB_result, T_BF, T_PM, 1, 1);

format long
optimization_mean_error = mean(total_error*1000)
optimization_std_error = std(total_error*1000)
format short


%%
% function residual = transf_multip(T, T_BF, T_PM)
% 
%     T_FM = transfParameters_to_transfMatrix(T(1:6));
% 
%     T_PB = transfParameters_to_transfMatrix(T(7:12));
% 
%     residual = 0;
%     for j = 1:n_observations
%         ident = T_PB * T_BF(:,:,j) * T_FM * invert_transformation_matrix(T_PM(:,:,j));
% 
%         f11 = (ident(1,1)-1)^2; f12 = (ident(1,2))^2; f13 = (ident(1,3))^2; f14 = (ident(1,4))^2;
%         f21 = (ident(2,1))^2; f22 = (ident(2,2)-1)^2; f23 = (ident(2,3))^2; f24 = (ident(2,4))^2;
%         f31 = (ident(3,1))^2; f32 = (ident(3,2))^2; f33 = (ident(3,3)-1)^2; f34 = (ident(3,4))^2;
% 
%         residual = residual + f11+f12+f13+f14+f21+f22+f23+f24+f31+f32+f33+f34;
% 
%     end
% end





