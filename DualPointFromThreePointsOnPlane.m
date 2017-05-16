function pDual = DualPointFromThreePointsOnPlane( p1, p2, p3);

ensure('( size(p1,1) == 3 )');
ensure('( size(p2,1) == 3 )');
ensure('( size(p3,1) == 3 )');

numPoints = max([size(p1,2),  size(p2,2),  size(p3,2)]);

if size(p1,2) == 1 
    p1 = repmat(p1, 1, numPoints);
end
if size(p2,2) == 1 
    p2 = repmat(p2, 1, numPoints);
end
if size(p3,2) == 1 
    p3 = repmat(p3, 1, numPoints);
end

ensure('( size(p1,2) == numPoints )');
ensure('( size(p2,2) == numPoints )');
ensure('( size(p3,2) == numPoints )');




vA = p2-p1;
vB = p3-p1;

faceNormal = cross(vA,vB);

d = - dot(faceNormal, p1 );


pDual = faceNormal./repmat(d, 3, 1);

