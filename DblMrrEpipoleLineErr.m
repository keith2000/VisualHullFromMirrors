function [err, eRV1, eRV2, eV1V12, eV2V21] = DblMrrEpipoleLineErr( vec, polyBoundaryVec, vecScl, draw)



ms_assert('length(vec(:)) == 6')

vec = vec .* vecScl;

eRV1 = [vec(1); vec(2)];
eRV2 = [vec(3); vec(4)];
a = vec(5);
b = vec(6);

lineDir = eRV2 - eRV1;
lineDirHat = lineDir/norm(lineDir);

eV2V21 = eRV1 + a*lineDirHat;
eV1V12 = eRV2 - b*lineDirHat;

if draw
    plot2D(eV2V21, 'm*', 'MarkerSize', 15)
    plot2D(eV1V12, 'c*', 'MarkerSize', 15)
    plot2D(eRV1, 'r*', 'MarkerSize', 15)
    plot2D(eRV2, 'g*', 'MarkerSize', 15)
end

err = [...
    FourErrorDistances( eRV1, polyBoundaryVec{1}, polyBoundaryVec{2} ), ...
    FourErrorDistances( eRV1, polyBoundaryVec{3}, polyBoundaryVec{4} ), ...
    FourErrorDistances( eRV2, polyBoundaryVec{1}, polyBoundaryVec{3} ), ...
    FourErrorDistances( eRV2, polyBoundaryVec{2}, polyBoundaryVec{5} ), ...
    FourErrorDistances( eV2V21, polyBoundaryVec{5}, polyBoundaryVec{3} ), ...
    FourErrorDistances( eV1V12, polyBoundaryVec{4}, polyBoundaryVec{2} )    ];








