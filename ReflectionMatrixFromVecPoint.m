function M = ReflectionMatrixFromVecPoint( vec, point )

ms_assert('length(vec)==3')
ms_assert('abs( norm(vec)-1 ) < 10*eps')
ms_assert('length(point)==3')


nx = vec(1);
ny = vec(2);
nz = vec(3);

px = point(1);
py = point(2);
pz = point(3);



M(1,:) = [          -nx^2+nz^2+ny^2,                 -2*ny*nx,                 -2*nz*nx, 2*nx*(nx*px+ny*py+nz*pz)];
M(2,:) = [                 -2*ny*nx,           nx^2+nz^2-ny^2,                 -2*nz*ny, 2*ny*(nx*px+ny*py+nz*pz)];
M(3,:) = [                 -2*nz*nx,                 -2*nz*ny,           nx^2+ny^2-nz^2, 2*nz*(nx*px+ny*py+nz*pz)];
M(4,:) = [                        0,                        0,                        0,           nx^2+nz^2+ny^2];