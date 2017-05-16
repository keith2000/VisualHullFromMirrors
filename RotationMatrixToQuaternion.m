function Q=RotationMatrixToQuaternion(R)

if any(size(R)~=[3,3])
    error('R must be 3x3')
end

T = trace(R) + 1;

if ( T > 0.00000001 )
%    disp('here')
    S = 0.5 / sqrt(T);    
    Qw = 0.25 / S;        
    Qx = ( R(3,2) - R(2,3) ) * S;    
    Qy = ( R(1,3) - R(3,1) ) * S;    
    Qz = ( R(2,1) - R(1,2) ) * S;    
else    
    [dummy, ind] = max(diag(R));
    
    switch ind
    case 1
%        disp(ind)
        S  = sqrt( 1.0 + R(1,1) - R(2,2) - R(3,3) ) * 2;
        Qx = 0.5 / S;
        Qy = (R(1,2) + R(2,1) ) / S;
        Qz = (R(1,3) + R(3,1) ) / S;
        Qw = (R(2,3) + R(3,2) ) / S;        
    case 2
 %               disp(ind)
        S  = sqrt( 1.0 + R(2,2) - R(1,1) - R(3,3) ) * 2;        
        Qx = (R(1,2) + R(2,1) ) / S;
        Qy = 0.5 / S;
        Qz = (R(2,3) + R(3,2) ) / S;
        Qw = (R(1,3) + R(3,1) ) / S;
    case 3
  %              disp(ind)
        S  = sqrt( 1.0 + R(3,3) - R(1,1) - R(2,2) ) * 2;        
        Qx = (R(1,3) + R(3,1) ) / S;
        Qy = (R(2,3) + R(3,2) ) / S;
        Qz = 0.5 / S;
        Qw = (R(1,2) + R(2,1) ) / S;
    otherwise
        error('Impossible index.')
    end
        
end


Q = [Qx;Qy;Qz;Qw];

% Q48. How do I convert a rotation matrix to a quaternion?
% --------------------------------------------------------
% 
%   A rotation may be converted back to a quaternion through the use of
%   the following algorithm:
% 
%   The process is performed in the following stages, which are as follows:
% 
%     Calculate the trace of the matrix T from the equation:
% 
%                 2     2     2
%       T = 4 - 4x  - 4y  - 4z
% 
%                  2    2    2
%         = 4( 1 -x  - y  - z )
% 
%         = mat[0] + mat[5] + mat[10] + 1
% 
% 
%     If the trace of the matrix is greater than zero, then
%     perform an "instant" calculation.
% 
%       S = 0.5 / sqrt(T)
% 
%       W = 0.25 / S
% 
%       X = ( mat[9] - mat[6] ) * S
% 
%       Y = ( mat[2] - mat[8] ) * S
% 
%       Z = ( mat[4] - mat[1] ) * S
% 
% 
%     If the trace of the matrix is less than or equal to zero
%     then identify which major diagonal element has the greatest
%     value.
% 
%     Depending on this value, calculate the following:
% 
%       Column 0:
%         S  = sqrt( 1.0 + mr[0] - mr[5] - mr[10] ) * 2;
% 
%         Qx = 0.5 / S;
%         Qy = (mr[1] + mr[4] ) / S;
%         Qz = (mr[2] + mr[8] ) / S;
%         Qw = (mr[6] + mr[9] ) / S;
% 
%       Column 1:
%         S  = sqrt( 1.0 + mr[5] - mr[0] - mr[10] ) * 2;
% 
%         Qx = (mr[1] + mr[4] ) / S;
%         Qy = 0.5 / S;
%         Qz = (mr[6] + mr[9] ) / S;
%         Qw = (mr[2] + mr[8] ) / S;
% 
%       Column 2:
%         S  = sqrt( 1.0 + mr[10] - mr[0] - mr[5] ) * 2;
% 
%         Qx = (mr[2] + mr[8] ) / S;
%         Qy = (mr[6] + mr[9] ) / S;
%         Qz = 0.5 / S;
%         Qw = (mr[1] + mr[4] ) / S;
% 
%      The quaternion is then defined as:
% 
%        Q = | Qx Qy Qz Qw |
% 
% 
