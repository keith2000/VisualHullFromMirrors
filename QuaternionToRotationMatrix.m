function R = QuaternionToRotationMatrix(Q)

if any( size(Q) ~= [4,1] )
   error('Q must be a 4x1 vector (quaternion)')   
end

Q=Q/norm(Q); %ensure unit quaternion
X=Q(1);
Y=Q(2);
Z=Q(3);
W=Q(4);

R(1,1)=1-2*Y^2-2*Z^2;
R(1,2)=2*X*Y-2*Z*W;
R(1,3)=2*X*Z+2*Y*W;

R(2,1)=2*X*Y+2*Z*W;
R(2,2)=1-2*X^2-2*Z^2;
R(2,3)=2*Y*Z-2*X*W;

R(3,1)=2*X*Z-2*Y*W;
R(3,2)=2*Y*Z+2*X*W;
R(3,3)=1-2*X^2-2*Y^2;




%Q47. How do I convert a quaternion to a rotation matrix?
%--------------------------------------------------------
%
%  Assuming that a quaternion has been created in the form:
%
%    Q = |X Y Z W|
%
%  Then the quaternion can then be converted into a 4x4 rotation
%  matrix using the following expression:
%
%
%        |       2     2                                |
%        | 1 - 2Y  - 2Z    2XY - 2ZW      2XZ + 2YW     |
%        |                                              |
%        |                       2     2                |
%    M = | 2XY + 2ZW       1 - 2X  - 2Z   2YZ - 2XW     |
%        |                                              |
%        |                                      2     2 |
%        | 2XZ - 2YW       2YZ + 2XW      1 - 2X  - 2Y  |
%        |                                              |
%
%
%  If a 4x4 matrix is required, then the bottom row and right-most column
%  may be added.
%
%  The matrix may be generated using the following expression:
%
%    ----------------
%
%    xx      = X * X;
%    xy      = X * Y;
%    xz      = X * Z;
%    xw      = X * W;
%
%    yy      = Y * Y;
%    yz      = Y * Z;
%    yw      = Y * W;
%
%    zz      = Z * Z;
%    zw      = Z * W;
%
%    mat[0]  = 1 - 2 * ( yy + zz );
%    mat[1]  =     2 * ( xy - zw );
%    mat[2]  =     2 * ( xz + yw );
%
%    mat[4]  =     2 * ( xy + zw );
%    mat[5]  = 1 - 2 * ( xx + zz );
%    mat[6]  =     2 * ( yz - xw );
%
%    mat[8]  =     2 * ( xz - yw );
%    mat[9]  =     2 * ( yz + xw );
%    mat[10] = 1 - 2 * ( xx + yy );
%
%    mat[3]  = mat[7] = mat[11 = mat[12] = mat[13] = mat[14] = 0;
%    mat[15] = 1;
%
%    ----------------

