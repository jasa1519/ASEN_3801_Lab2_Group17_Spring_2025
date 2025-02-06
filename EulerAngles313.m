function attitude313 = EulerAngles313(DCM)
%ATTITUDE313 Extracts attitude (in 3-1-3) from a DCM
%   Detailed explanation goes here

alpha = atan2(DCM(3,1),-1*DCM(3,2));
beta = acos(DCM(3,3));
gamma = atan2(DCM(1,3),DCM(2,3));

attitude313 = [alpha;
               beta;
               gamma];
end
