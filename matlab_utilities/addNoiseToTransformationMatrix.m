% Adds Gaussian noise to a given transformation matrix.
% 
% Inouts:
%     T_initial: A 4x4 transformation matrix.
%     mean_ang: A 3-element vector for mean angles (in radians).
%     mean_pos: A 3-element vector for mean positions.
%     stand_dev_ang: A 3-element vector for standard deviation of angle noise (in radians).
%     stand_dev_pos: A 3-element vector for standard deviation of position noise.
% 
% Output:
%     noisyMatrix: The transformation matrix with added noise.

function noisyMatrix = addNoiseToTransformationMatrix(T_initial, mean_ang, mean_pos, stand_dev_ang, stand_dev_pos)

    % Check if the input matrix is 4x4
    if size(T_initial, 1) ~= 4 || size(T_initial, 2) ~= 4
        error('Input transformation matrix must be 4x4xn.');
    end
    
    % Check if mean and standard deviation vectors are of correct size
    if length(mean_ang) ~= 3 || length(mean_pos) ~= 3 || ...
       length(stand_dev_ang) ~= 3 || length(stand_dev_pos) ~= 3
        error('mean_ang, mean_pos, stand_dev_ang, and stand_dev_pos must be 3-element vectors.');
    end

    noisyMatrix = zeros(4, 4, size(T_initial,3));

    for i = 1:size(T_initial,3)
    
        an = randn*stand_dev_ang(1) + mean_ang(1);
        bn = randn*stand_dev_ang(2) + mean_ang(2);
        cn = randn*stand_dev_ang(3) + mean_ang(3);

        xn = randn*stand_dev_pos(1) + mean_pos(1);
        yn = randn*stand_dev_pos(2) + mean_pos(2);
        zn = randn*stand_dev_pos(3) + mean_pos(3);

        T_noise = transfParameters_to_transfMatrix([an;bn;cn;xn;yn;zn]);

        noisyMatrix(:,:,i) = T_initial(:,:,i) * T_noise;
    
    end
    
end

