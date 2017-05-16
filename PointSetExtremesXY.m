function [minX, minY, maxX, maxY] = PointSetExtremesXY( pointVec )

minX = min( pointVec(1,:) );
minY = min( pointVec(2,:) );

maxX = max( pointVec(1,:) );
maxY = max( pointVec(2,:) );