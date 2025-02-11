% This function calculates the homogeneous transformation matrix
% for the given values of transformation parameters
% 
% Inputs: [a, b, c, x, y, z]
%     a -> rotation angle arround x-axis [rads]
%     b -> rotation angle arround y-axis [rads]
%     c -> rotation angle arround z-axis [rads]
%     x -> translation along the x-axis [m]
%     y -> translation along the y-axis [m]
%     z -> translation along the z-axis [m]
%
% Outputs: 
%     T -> homogeneous transformation matrix

function T = transfParameters_to_transfMatrix(transf_param)

    a = transf_param(1); b = transf_param(2); c = transf_param(3);
    x = transf_param(4); y = transf_param(5); z = transf_param(6);

    T = [cos(b)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c), x;
         cos(b)*sin(c), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), y;
         -sin(b), sin(a)*cos(b), cos(a)*cos(b), z;
         0,0,0,1];

end