% Function that calculates the distance (error) between the real trajectory
% and the one obtained with the optimization results.
% 
% Inputs: 
%     T_FM -> Calculated transformation matrix from frame M to frame F
%     T_PB -> Calculated transformation matrix from frame B to frame P
%     T_BF -> Transformation matrix from frame F to frame B for every
%     observation
%     T_PM -> Transformation matrix from frame M to frame P for every
%     observation
%     flag_plot_trajectory -> Flag that indicates if the user wants to plot the trajectory (bool).
%     flag_plot_error -> Flag that indicates if the user wants to plot the observations error (bool).
% 
% Output:
%     error -> vector with the error (distance between real and calculated)
%     of each observation

function error = real_calibration_error(T_FM, T_PB, T_BF, T_PM, flag_plot_trajectory, flag_plot_error)

    n_observations = size(T_PM,3);

    if ~isscalar(n_observations) || n_observations <= 0 || mod(n_observations, 1) ~= 0
        error('n_spline_points must be a positive integer scalar.');
    end
    if ~isequal(size(T_FM), [4, 4])
        error('T_FT transformation matrix must be 4x4.');
    end
    if ~isequal(size(T_PB), [4, 4])
        error('T_PB transformation matrix must be 4x4.');
    end
    if ~isequal(size(T_BF,1), 4) || ~isequal(size(T_BF,2), 4)
        error('T_BF transformation matrix must be 4x4xn.');
    end
    if ~isequal(size(T_PM,1), 4) || ~isequal(size(T_PM,2), 4)
        error('T_PT transformation matrix must be 4x4xn.');
    end
    if ~isequal(size(T_PM,3), n_observations)
        error(['Both T_PT and T_BF transformation matrices must be 4x4xn, ' ...
            'where n corresponds to the number of observations and therefore ' ...
            'must be the same in both.']);
    end

    points_BF = zeros(4,n_observations);
    points_BF2 = zeros(4,n_observations);
    error = zeros(n_observations,1);
    
    for i = 1:n_observations
        
        points_BF(:,i) = T_BF(:,:,i) * [0;0;0;1];
        
        points_BF2(:,i) = invert_transformation_matrix(T_PB) *  T_PM(:,:,i) * invert_transformation_matrix(T_FM) * [0;0;0;1];

        error(i) = sqrt((points_BF(1,i)-points_BF2(1,i))^2 + (points_BF(2,i)-points_BF2(2,i))^2 + (points_BF(3,i)-points_BF2(3,i))^2);
    end

    if flag_plot_trajectory
        figure, hold on, axis equal, grid on
        plot3(points_BF(1,:),points_BF(2,:),points_BF(3,:),'o--')
        plot3(points_BF2(1,:),points_BF2(2,:),points_BF2(3,:),'o--')
        hold off
        xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
        legend('Base -> Flange', 'Base -> Polaris -> Tracker -> Flange')
    end

    if flag_plot_error
        figure, plot(error*1000,'-o'), xlabel('Observation number'), ylabel('error [mm]')
    end
  
end