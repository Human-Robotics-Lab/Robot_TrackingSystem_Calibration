% This function calculates the rotational and translational error between 
% two given homogeneous transformation matrices
%
% input: [T1, T2]
%     T1 -> First transformation matrix
%     T2 -> Second transformation matrix
%
% output: total_error = [rotational_error [rad], translational_error [m]]

function total_error = calculate_error_between_2_transf_matrices(T1, T2)  

    % Check if the input matrices are 4x4
    if ~isequal(size(T1), [4, 4])
        error('First input transformation matrix must be 4x4.');
    end
    if ~isequal(size(T2), [4, 4])
        error('Second input transformation matrix must be 4x4.');
    end

    % rotational error calculation
    R_error = T1(1:3,1:3)' * T2(1:3,1:3);
    ang_error = rotm2axang(R_error);
    ang_error = ang_error(4);

    % translational error calculation
    pos_error = sqrt((T1(1,4) - T2(1,4))^2 + (T1(2,4) - T2(2,4))^2 + (T1(3,4) - T2(3,4))^2 );
    
    %
    total_error = [ang_error, pos_error];

end