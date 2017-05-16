function [aInd,bInd, pA, pB] = OrthoTangencies( thePoly, epiDirection)


if ( size(thePoly,1) ~= 2 )
    error('thePoly must have 2 rows')
end

if ( any(size(epiDirection) ~= [2 1] ) )
    error('epiDirection must be 2x1')
end

thePolyAlignedX = [-epiDirection(2),epiDirection(1)] * thePoly;

[dummy, aInd] = max(thePolyAlignedX);
[dummy, bInd] = min(thePolyAlignedX);

pA = thePoly(:,aInd);
pB = thePoly(:,bInd);