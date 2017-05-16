function [polyBoundaryCellOut, indVec] = OrderMostConsistentEpipoles( polyBoundaryVecInit )

numPolys = length(polyBoundaryVecInit);
ms_assert('numPolys==5')

[polyBoundaryVecInit, indVec] = OrderAroundConvexHull( polyBoundaryVecInit );

bestErr = inf;

polyRange = 1:numPolys;

for combLoop = 1:numPolys,
    
    polyBoundaryVec = CreateCombination( polyBoundaryVecInit, combLoop );
    
    [eRV1, eRV2, eV1V12, eV2V21 ] = DblMrrEpipoles( polyBoundaryVec );
    
    err = [...
            FourErrorDistances( eRV1, polyBoundaryVec{1}, polyBoundaryVec{2} ), ...
            FourErrorDistances( eRV1, polyBoundaryVec{3}, polyBoundaryVec{4} ), ...
            FourErrorDistances( eRV2, polyBoundaryVec{1}, polyBoundaryVec{3} ), ...
            FourErrorDistances( eRV2, polyBoundaryVec{2}, polyBoundaryVec{5} ), ...
            FourErrorDistances( eV2V21, polyBoundaryVec{5}, polyBoundaryVec{3} ), ...
            FourErrorDistances( eV1V12, polyBoundaryVec{4}, polyBoundaryVec{2} )    ];
    
    
    EvalPrint('rms(err)')
    if rms(err) < bestErr
        bestErr = rms(err);
        bestLoop = combLoop;
    end
    
end

polyBoundaryCellOut = CreateCombination( polyBoundaryVecInit, bestLoop );

indVec = [indVec(bestLoop:end), indVec(1:bestLoop-1)];

function polyBoundaryVec = CreateCombination( polyBoundaryVecConvArrange, num )

numPolys = length(polyBoundaryVecConvArrange);
polyRange = 1:numPolys;
polyComb = [polyRange(num:end), polyRange(1:num-1)];


polyBoundaryVec{1} = polyBoundaryVecConvArrange{polyComb(1)};
polyBoundaryVec{2} = polyBoundaryVecConvArrange{polyComb(2)};
polyBoundaryVec{3} = polyBoundaryVecConvArrange{polyComb(5)};
polyBoundaryVec{4} = polyBoundaryVecConvArrange{polyComb(3)};
polyBoundaryVec{5} = polyBoundaryVecConvArrange{polyComb(4)};
