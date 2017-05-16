function [p1A, p1B, p2A, p2B, ind1A, ind1B, ind2A, ind2B] = OuterTangents( polyA, polyB )

%returns the points on each of the two tangents associated with polyA an
%polyB: the tangent that touches both polygons on one side, and on the
%other

ms_assert('size(polyA,1)==2');
ms_assert('size(polyB,1)==2');
ms_assert('size(polyA,2)>=3');
ms_assert('size(polyB,2)>=3');

bothPolys = [ polyA, polyB];

kk = convhull( bothPolys(1,:),  bothPolys(2,:) );
kk = kk(1:end-1);

polyLabel = ( kk <= length(polyA) );

theDiff = polyLabel - [polyLabel(2:end);polyLabel(1)];

ind1 = find(theDiff==+1);

ind1A = kk(ind1);
p1A = polyA(:, ind1A );
ind1 = mod(ind1 + 1 -1, length(kk))+1;
ind1B = kk(ind1)-length(polyA);
p1B = polyB(:, ind1B);


ind2 = find(theDiff==-1);
ind2B = kk(ind2)-length(polyA);
p2A = polyB(:,ind2B);
ind2 = mod(ind2 + 1 -1, length(kk))+1;
ind2A = kk(ind2);
p2B = polyA(:,ind2A );

