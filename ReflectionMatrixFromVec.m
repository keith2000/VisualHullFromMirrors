function M = ReflectionMatrixFromVec( vec )


vec = vec/norm(vec);

nx = vec(1);
ny = vec(2);
nz = vec(3);

% 
% 
% 
% M(1,:) = [          -nx^2+nz^2+ny^2,                 -2*ny*nx,                 -2*nz*nx];
% M(2,:) = [                 -2*ny*nx,           nx^2+nz^2-ny^2,                 -2*nz*ny];
% M(3,:) = [                 -2*nz*nx,                 -2*nz*ny,           nx^2+ny^2-nz^2];


M(1,:) = [          -nx^2+ny^2+nz^2,                 -2*ny*nx,                 -2*nz*nx];
M(2,:) = [                 -2*ny*nx,           +nx^2-ny^2+nz^2,                 -2*nz*ny];
M(3,:) = [                 -2*nz*nx,                 -2*nz*ny,           +nx^2+ny^2-nz^2];
