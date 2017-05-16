function [pA, pB, minInd, maxInd] = PointPolygonTangentExtremes(p0,poly)

%[pA,pB]=PointPolygonTangentExtremes(p0,poly)
%returns points on the lines passing through po that just touch poly

if iscell(poly) %//PolySet
    q=poly;
    poly=[];
    for loop=1:length(q)
        poly = [poly,q{loop}];
    end    
end

ang=atan2(poly(2,:)-p0(2),poly(1,:)-p0(1))*(180/pi);

if (max(ang)-min(ang)>180)
    ang=ang+(ang<0)*360;
end

[dummy,minInd]=min(ang);
pA=poly(:,minInd);

[dummy,maxInd]=max(ang);
pB=poly(:,maxInd);

%save
if IsPointInConvPoly( p0(1:2,1), poly(1:2,:) )
    pA = [NaN;NaN];    
    pB = [NaN;NaN];    
    maxInd = NaN;
    minInd = NaN;
end

% 
% figure
% hold on
% ShowPoly( poly(1:2,:), 'FaceColor', 'c');
% plot2d( pA(1:2), '*')
% plot2d( pB(1:2), '*')