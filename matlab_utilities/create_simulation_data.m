%% 
% Creates simulation data without any noise and with a desired number of observations

n_observations = 600;

%% create T_PM matrices

[xx, yy, zz] = trajectories_simulation(30, 4, n_observations,1);
[aa, bb, cc] = trajectories_simulation(30, 2*pi, n_observations,1);

T_PM = zeros(4,4,n_observations);
for i=1:n_observations
    transf_param = [aa(i),bb(i),cc(i),xx(i),yy(i),zz(i)];

    T_PM(:,:,i) = transfParameters_to_transfMatrix(transf_param);  
end

%% create T_FM and T_PB matrices randomly
T0 = zeros(1,12);
T0(1:3) = rand(1,3)*2*pi-pi;
T0(4:6) = rand(1,3)*0.4-0.2;
T0(7:9) = rand(1,3)*2*pi-pi;
T0(10:12) = rand(1,3)*6-3;

T_FM = transfParameters_to_transfMatrix(T0(1:6));
T_PB = transfParameters_to_transfMatrix(T0(7:12));

%% create T_BF matrices
T_BF = zeros(4,4,n_observations);
for i=1:n_observations

    T_BF(:,:,i) = invert_transformation_matrix(T_PB) * T_PM(:,:,i) * invert_transformation_matrix(T_FM);
         
end

%% save to .mat file in "simulation_data" folder

optimal_solution.n_observations = n_observations;
optimal_solution.T0 = T0;
optimal_solution.T_FM = T_FM;
optimal_solution.T_PB = T_PB;

optimal_solution.T_PM = T_PM;
optimal_solution.T_BF = T_BF;

file_name = "simul_data_" + num2str(n_observations) + "_observations";

save("simulation_data/" + file_name + ".mat", "optimal_solution")


