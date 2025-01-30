function attitude321 = EulerAngles321(DCM)
%EULERANGLES321 Retrieves attitude (in 3-2-1) from a DCM
%   Detailed explanation goes here

alpha = atan2(DCM(2,3),DCM(3,3));
beta = -1*asin(DCM(1,3));
gamma = atan2(DCM(1,2),DCM(1,1));

attitude321 = [alpha;
               beta;
               gamma];

end

