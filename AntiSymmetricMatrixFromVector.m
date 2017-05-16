function A = AntiSymmetricMatrixFromVector(a);


% As a previous poster wrote, you need to do this by components.  But,
% there is a slight trick that makes it fairly easy.  The problem is
% working out the curl of a cross product or cross with a curl.  Think
% of (V cross) or curl as an operator on a 3-dimensional vector.  Then
% we can write it as an antisymmetric matrix which will operate on the
% right-hand vector:
%       |  0  -Vz  Vy |
% V x = | Vz   0  -Vx |  The curl is similar, but with partial diff.
%       |-Vy   Vx  0  |
% operators in place of the vector components.  Multiplying two such
% matrices will give you a symmetric matrix which will display the 
% patterns you are seeking.

A(1,:) = [0,      -a(3), +a(2)];
A(2,:) = [+a(3),     0,  -a(1)];
A(3,:) = [-a(2),  +a(1),   0 ];