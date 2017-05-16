function err = FourErrorDistances( epipole, polyBoundaryRed, polyBoundaryGreen )
[pRedUpper, pRedLower] = PointPolygonTangentExtremes( epipole, polyBoundaryRed ); %find tangency points for red poly from epipole
[pGreenUpper, pGreenLower] = PointPolygonTangentExtremes( epipole, polyBoundaryGreen ); %find tangency points for green poly from epipole

if PerpDist2DPointTo2DLine( pRedUpper,  pRedUpper-epipole, pGreenLower ) < ...
        PerpDist2DPointTo2DLine( pRedUpper,  pRedUpper-epipole, pGreenUpper )    
    temp = pGreenUpper;     pGreenUpper = pGreenLower; pGreenLower = temp;
end

%plot2d([ epipole, pRedUpper], 'r-')
%plot2d([ epipole, pGreenUpper], 'g-')

err = [ PerpDist2DPointTo2DLine( pRedUpper,  pRedUpper-epipole, pGreenUpper ), ...
        PerpDist2DPointTo2DLine( pRedLower,  pRedLower-epipole, pGreenLower ), ...
        PerpDist2DPointTo2DLine( pGreenUpper,  pGreenUpper-epipole, pRedUpper ), ...
        PerpDist2DPointTo2DLine( pGreenLower,  pGreenLower-epipole, pRedLower )];