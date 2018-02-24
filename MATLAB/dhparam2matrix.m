function T = dhparam2matrix(theta, d, a, alpha)

% Build individual transformations
RotZ = [cos(theta), -sin(theta), 0, 0; sin(theta), cos(theta), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
TransZ = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
TransX = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
RotX = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0; 0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];

% Combine all transformations
T = RotZ*TransZ*TransX*RotX;

end