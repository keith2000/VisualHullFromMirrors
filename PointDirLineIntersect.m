function p0 = PointDirLineIntersect( p1, dir1, p2, dir2 )

aX = p1(1);
aY = p1(2);

bX = dir1(1);
bY = dir1(2);

cX = p2(1);
cY = p2(2);

dX = dir2(1);
dY = dir2(2);


u = (aY*bX-bY*aX+bY*cX-cY*bX)/(-bY*dX+dY*bX);

p0 = p2 + u*dir2;