function inPoly = IsPointInConvPoly( p0, poly )

sideOfEdge = cross( aug( CoordAdd(poly,-p0)),  aug(  CoordAdd([poly(:,2:end), poly(:,1) ],-p0))  );

inPoly = all(sideOfEdge(3,:) > 0) | all(sideOfEdge(3,:) < 0);