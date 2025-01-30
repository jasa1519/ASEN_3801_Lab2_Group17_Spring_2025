function DCM = RotationMatrix313(attitude313)
%ROTATIONMATRIX313 Creates a rotation matrix for 313
%   Detailed explanation goes here
alpha = attitude313(1);
beta = attitude313(2);
gamma = attitude313(3);

R1 = @(theta) rotx(theta);
R2 = @(theta) roty(theta);
R3 = @(theta) rotz(theta);

DCM = R3(alpha)*(R1(beta)*R3(gamma));

end

