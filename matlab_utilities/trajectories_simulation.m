% Function that generates a 3D trajectory based on randomly generated points.
% It creates a spline interpolation of these points, producing a smooth 
% trajectory that can be visualized in a 3D plot.
% 
% Inputs:
%     n_spline_points -> The number of random points to generate for the spline (integer).
%     values_range -> The range within which the random points will be generated (scalar).
%     number_points -> The number of points to interpolate along the spline (integer).
%     visualization_flag -> Flag that indicates if the user wants to plot the created trajectory (bool).     
% 
% Outputs:
%     xx -> The interpolated x-coordinates of the trajectory (vector).
%     yy -> The interpolated y-coordinates of the trajectory (vector).
%     zz -> The interpolated z-coordinates of the trajectory (vector).
%     x -> The original randomly generated x-coordinates (vector).
%     y -> The original randomly generated y-coordinates (vector).
%     z -> The original randomly generated z-coordinates (vector).
% 
% Visualization:
%     The function also creates a 3D scatter plot of the original points and overlays the interpolated trajectory.

function [xx, yy, zz, x, y, z] = trajectories_simulation(n_spline_points, values_range, number_points, visualization_flag)

    % Input validation
    if ~isscalar(n_spline_points) || n_spline_points <= 0 || mod(n_spline_points, 1) ~= 0
        error('n_spline_points must be a positive integer scalar.');
    end
    if ~isscalar(values_range) || values_range <= 0
        error('values_range must be a positive scalar.');
    end
    if ~isscalar(number_points) || number_points <= 0 || mod(number_points, 1) ~= 0
        error('number_points must be a positive integer scalar.');
    end

    % Sample point
    x = rand(n_spline_points,1)*values_range - (values_range/2);
    y = rand(n_spline_points,1)*values_range - (values_range/2);
    z = rand(n_spline_points,1)*values_range - (values_range/2);
    t = 1:n_spline_points; % Assumed time stamp
    % Apply interpolation for each x,y and z 
    tt = linspace(t(1),t(end),number_points);
    xx = interp1(t,x,tt,'spline');
    yy = interp1(t,y,tt,'spline');
    zz = interp1(t,z,tt,'spline');
    
    if visualization_flag ~= 0
        % Visualize the resultant trajectory
        figure
        scatter3(x,y,z)
        hold on
        plot3(xx,yy,zz)
        hold off
        xlabel("x"), ylabel("y"), zlabel("z")
        title("3D Trajectory Simulation");
        axis equal
    end
    
end