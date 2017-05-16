function [dist, meetingPoint] = PerpDist2DPointTo2DLine( linePointA,  directionVec, thePoint )


ms_assert('all(size(linePointA)==[2,1])')
ms_assert('all(size(directionVec)==[2,1])')
ms_assert('all(size(thePoint)==[2,1])')

perpVec(1,1) = -directionVec(2);
perpVec(2,1) = directionVec(1);

L1 = cross( aug(linePointA), aug(linePointA + directionVec) );
L2 = cross( aug(thePoint), aug(thePoint + perpVec) );

joinHomogeneous = cross( L1, L2 );

meetingPoint = joinHomogeneous(1:2)/joinHomogeneous(3);

dist = norm(thePoint-meetingPoint);



