function DCM = RotationMatrix321(attitude321)
%ROTATIONMATRIX321 Creates a rotation matrix for 321
%   Detailed explanation goes here
alpha = attitude321(1);
beta = attitude321(2);
gamma = attitude321(3);

R1 = @(theta) rotx(theta);
R2 = @(theta) roty(theta);
R3 = @(theta) rotz(theta);

DCM = R1(alpha)*(R2(beta)*R3(gamma));

end

