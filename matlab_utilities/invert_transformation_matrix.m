% Function that inverts a given homogeneous transformation matrix
% 
% If T_AB corresponds to the transformation matrix form the 
% frame B to the frame A, this function calculates the 
% trasformation matrix T_BA, which is the tranformation 
% from the frame A to frame B
% 
% Input:
%     T_original -> Initial transformation matrix
% 
% Output:
%     T_inverted -> Inverted transformation matrix


function T_inverted = invert_transformation_matrix(T_original)

    % Check if the input matrix is 4x4
    if ~isequal(size(T_original), [4, 4])
        error('Input transformation matrix must be 4x4.');
    end

    R_original = T_original(1:3,1:3);
    b_original = T_original(1:3,4);

    R_inverted = R_original';

    b_inverted = -R_inverted * b_original;

    T_inverted = eye(4); 
    T_inverted(1:3,1:3) = R_inverted; 
    T_inverted(1:3,4) = b_inverted;

end