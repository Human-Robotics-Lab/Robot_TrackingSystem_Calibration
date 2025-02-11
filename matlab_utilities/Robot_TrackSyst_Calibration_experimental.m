%% MAIN SCRIPT TO PERFORM THE CALIBRATION WITH EXPERIMENTAL DATA

clear all, close all, clc

%% load observations and create .json file with data to the optimizer

load("C:\Dev\Robot_TrackingSystem_Calibration\plus_config\acquired_data\experimental_data_1.mat")

n_observations = size(T_BF,3);

% scale factor in translation parameters
scale_factor = 1/100000;
for i = 1:size(T_BF,3)
   T_BF(1:3,4,i) = T_BF(1:3,4,i)*scale_factor; 
   T_PM(1:3,4,i) = T_PM(1:3,4,i)*scale_factor;
end


observations = zeros(size(T_BF,3),32);
for i = 1:n_observations
    temp1=T_BF(:,:,i);
    temp2=T_PM(:,:,i);
    observations(i,:) = [temp1(:);temp2(:)]';
end

first_initialization = zeros(12,1);
n_initializations = 40;
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
fid = fopen("C:\Dev\Robot_TrackingSystem_Calibration\observations.json", "wt");
if fid == -1
    error('Failed to open file for writing.');
end
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

%% calculate the obtained error

T_FM_result = transfParameters_to_transfMatrix(out(1:6));
T_PB_result = transfParameters_to_transfMatrix(out(7:12));
total_error = real_calibration_error(T_FM_result, T_PB_result, T_BF, T_PM, 1, 1);

format long
optimization_mean_error = mean(total_error*1000)
optimization_std_error = std(total_error*1000)
format short


%%
% function residual = cost_function(T, T_BF, T_PM)
% 
%     T_FT = transfParameters_to_transfMatrix(T(1:6));
% 
%     T_PB = transfParameters_to_transfMatrix(T(7:12));
% 
%     residual = 0;
%     for j = 1:n_observations
%         ident = T_PB * T_BF(:,:,j) * T_FT * invert_transformation_matrix(T_PM(:,:,j));
%     
%         f11 = (ident(1,1)-1)^2; f12 = (ident(1,2))^2; f13 = (ident(1,3))^2; f14 = (ident(1,4))^2;
%         f21 = (ident(2,1))^2; f22 = (ident(2,2)-1)^2; f23 = (ident(2,3))^2; f24 = (ident(2,4))^2;
%         f31 = (ident(3,1))^2; f32 = (ident(3,2))^2; f33 = (ident(3,3)-1)^2; f34 = (ident(3,4))^2;
%     
%         residual = residual + f11+f12+f13+f14+f21+f22+f23+f24+f31+f32+f33+f34;
%        
%     end
% end

