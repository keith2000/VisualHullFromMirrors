function [polyBoundaryCellOut, indVec] = OrderAroundConvexHull( polyBoundaryCell )

%orders the polygons clockwise along their convex hull

numPolys = length(polyBoundaryCell);


allPointsVec = [];
    indVec = [ ];
for polyLoop = 1:numPolys
    allPointsVec = [allPointsVec, polyBoundaryCell{polyLoop}];
    indVec = [indVec, polyLoop * ones(1, size(polyBoundaryCell{polyLoop},2) ) ];
end

k = convhull(allPointsVec(1,:), allPointsVec(2,:));

indVec = indVec(k);

for polyLoop = 1:numPolys
    
    ind = find(indVec == polyLoop);
    indVec(ind(2:end)) = 0;
end

indVec = indVec(find(indVec));

ms_assert('length(indVec) == numPolys', 'not all polys on the convex hull')


for polyLoop = 1:numPolys
   polyBoundaryCellOut{polyLoop} = polyBoundaryCell{indVec(polyLoop)};
end

